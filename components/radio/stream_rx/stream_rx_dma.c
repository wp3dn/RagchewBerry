#include "stream_rx_dma.h"

#include <string.h>

#include "hardware/dma.h"
#include "hardware/irq.h"

/* DMA_IRQ_0 heeft één handler; daarom ondersteunt deze module één instantie. */
static stream_rx_dma_t *irq_context;

static int take_free_buffer_locked(stream_rx_dma_t *ctx)
{
    if (ctx->free_mask == 0u) {
        return -1;
    }

    const uint8_t index = (uint8_t)__builtin_ctz((unsigned)ctx->free_mask);
    ctx->free_mask &= (uint8_t)~(1u << index);
    return index;
}

static void start_buffer_locked(stream_rx_dma_t *ctx, uint8_t index)
{
    ctx->active_buffer = (int8_t)index;

    dma_channel_set_write_addr(ctx->dma_channel, ctx->buffers[index], false);
    dma_channel_set_trans_count(ctx->dma_channel,
                                STREAM_RX_DMA_WORDS_PER_BUFFER,
                                true);
}

static void stream_rx_dma_irq0_handler(void)
{
    stream_rx_dma_t *ctx = irq_context;
    if (ctx == NULL) {
        return;
    }

    const uint32_t channel_mask = 1u << (uint)ctx->dma_channel;
    if ((dma_hw->ints0 & channel_mask) == 0u) {
        return;
    }

    /* Schrijven van een 1 wist de interruptstatus. */
    dma_hw->ints0 = channel_mask;

    uint32_t irq_state = spin_lock_blocking(ctx->lock);

    if (ctx->running && ctx->active_buffer >= 0) {
        const uint8_t completed = (uint8_t)ctx->active_buffer;
        ctx->active_buffer = -1;
        ctx->ready_mask |= (uint8_t)(1u << completed);
        ctx->stats.completed_buffers++;

        const int next = take_free_buffer_locked(ctx);
        if (next >= 0) {
            start_buffer_locked(ctx, (uint8_t)next);
        } else {
            /* DMA wordt hervat zodra de gebruiker een buffer vrijgeeft. */
            ctx->stats.stalled_transfers++;
        }
    }

    spin_unlock(ctx->lock, irq_state);
}

bool stream_rx_dma_init(stream_rx_dma_t *ctx, PIO pio, uint sm)
{
    if (ctx == NULL || sm >= NUM_PIO_STATE_MACHINES || irq_context != NULL) {
        return false;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->pio = pio;
    ctx->sm = sm;
    ctx->active_buffer = -1;

    ctx->dma_channel = dma_claim_unused_channel(false);
    if (ctx->dma_channel < 0) {
        return false;
    }

    const int lock_num = spin_lock_claim_unused(false);
    if (lock_num < 0) {
        dma_channel_unclaim(ctx->dma_channel);
        ctx->dma_channel = -1;
        return false;
    }
    ctx->lock = spin_lock_init((uint)lock_num);

    dma_channel_config config =
        dma_channel_get_default_config(ctx->dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, pio_get_dreq(pio, sm, false));

    dma_channel_configure(ctx->dma_channel,
                          &config,
                          ctx->buffers[0],
                          &pio->rxf[sm],
                          STREAM_RX_DMA_WORDS_PER_BUFFER,
                          false);

    irq_context = ctx;
    irq_set_exclusive_handler(DMA_IRQ_0, stream_rx_dma_irq0_handler);
    dma_channel_set_irq0_enabled(ctx->dma_channel, true);
    irq_set_enabled(DMA_IRQ_0, true);

    ctx->free_mask = (uint8_t)((1u << STREAM_RX_DMA_BUFFER_COUNT) - 1u);
    ctx->initialized = true;
    return true;
}

bool stream_rx_dma_start(stream_rx_dma_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return false;
    }

    stream_rx_dma_stop(ctx);
    pio_sm_clear_fifos(ctx->pio, ctx->sm);

    uint32_t irq_state = spin_lock_blocking(ctx->lock);
    ctx->free_mask = (uint8_t)((1u << STREAM_RX_DMA_BUFFER_COUNT) - 1u);
    ctx->ready_mask = 0u;
    ctx->acquired_mask = 0u;
    ctx->running = true;

    const int first = take_free_buffer_locked(ctx);
    start_buffer_locked(ctx, (uint8_t)first);
    spin_unlock(ctx->lock, irq_state);

    return true;
}

void stream_rx_dma_stop(stream_rx_dma_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    uint32_t irq_state = spin_lock_blocking(ctx->lock);
    ctx->running = false;
    spin_unlock(ctx->lock, irq_state);

    dma_channel_abort(ctx->dma_channel);
    dma_hw->ints0 = 1u << (uint)ctx->dma_channel;

    irq_state = spin_lock_blocking(ctx->lock);
    ctx->active_buffer = -1;
    ctx->free_mask = (uint8_t)((1u << STREAM_RX_DMA_BUFFER_COUNT) - 1u);
    ctx->ready_mask = 0u;
    ctx->acquired_mask = 0u;
    spin_unlock(ctx->lock, irq_state);
}

bool stream_rx_dma_try_acquire(stream_rx_dma_t *ctx,
                               const uint32_t **words,
                               uint8_t *buffer_index)
{
    if (ctx == NULL || words == NULL || buffer_index == NULL ||
        !ctx->initialized) {
        return false;
    }

    uint32_t irq_state = spin_lock_blocking(ctx->lock);
    if (ctx->ready_mask == 0u) {
        spin_unlock(ctx->lock, irq_state);
        return false;
    }

    const uint8_t index =
        (uint8_t)__builtin_ctz((unsigned)ctx->ready_mask);
    const uint8_t bit = (uint8_t)(1u << index);
    ctx->ready_mask &= (uint8_t)~bit;
    ctx->acquired_mask |= bit;
    spin_unlock(ctx->lock, irq_state);

    *words = ctx->buffers[index];
    *buffer_index = index;
    return true;
}

void stream_rx_dma_release(stream_rx_dma_t *ctx, uint8_t buffer_index)
{
    if (ctx == NULL || !ctx->initialized ||
        buffer_index >= STREAM_RX_DMA_BUFFER_COUNT) {
        return;
    }

    const uint8_t bit = (uint8_t)(1u << buffer_index);
    uint32_t irq_state = spin_lock_blocking(ctx->lock);

    if ((ctx->acquired_mask & bit) != 0u) {
        ctx->acquired_mask &= (uint8_t)~bit;
        ctx->free_mask |= bit;

        if (ctx->running && ctx->active_buffer < 0) {
            const int next = take_free_buffer_locked(ctx);
            start_buffer_locked(ctx, (uint8_t)next);
        }
    }

    spin_unlock(ctx->lock, irq_state);
}

stream_rx_dma_stats_t stream_rx_dma_get_stats(stream_rx_dma_t *ctx)
{
    stream_rx_dma_stats_t result = {0};
    if (ctx == NULL || !ctx->initialized) {
        return result;
    }

    uint32_t irq_state = spin_lock_blocking(ctx->lock);
    result = ctx->stats;
    spin_unlock(ctx->lock, irq_state);
    return result;
}

void stream_rx_dma_deinit(stream_rx_dma_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    stream_rx_dma_stop(ctx);
    dma_channel_set_irq0_enabled(ctx->dma_channel, false);
    dma_channel_unclaim(ctx->dma_channel);
    spin_lock_unclaim(spin_lock_get_num(ctx->lock));

    irq_context = NULL;
    ctx->initialized = false;
    ctx->lock = NULL;
    ctx->dma_channel = -1;
}
