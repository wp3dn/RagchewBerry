#include "stream_tx_dma.h"

#include <string.h>

#include "hardware/dma.h"
#include "hardware/irq.h"

#define STREAM_TX_DMA_PREFILL_READY_COUNT 1u

/* DMA_IRQ_1 heeft één handler; daarom ondersteunt deze module één instantie. */
static stream_tx_dma_t *irq_context;

static int take_free_buffer_locked(stream_tx_dma_t *ctx)
{
    if (ctx->free_mask == 0u) {
        return -1;
    }

    const uint8_t index = (uint8_t)__builtin_ctz((unsigned)ctx->free_mask);
    ctx->free_mask &= (uint16_t)~(1u << index);
    return index;
}

static int take_ready_buffer_locked(stream_tx_dma_t *ctx)
{
    if (ctx->ready_mask == 0u) {
        return -1;
    }

    const uint8_t index = (uint8_t)__builtin_ctz((unsigned)ctx->ready_mask);
    ctx->ready_mask &= (uint16_t)~(1u << index);
    return index;
}

static uint8_t count_ready_buffers_locked(const stream_tx_dma_t *ctx)
{
    return (uint8_t)__builtin_popcount((unsigned)ctx->ready_mask);
}

static uint8_t count_free_buffers_locked(const stream_tx_dma_t *ctx)
{
    return (uint8_t)__builtin_popcount((unsigned)ctx->free_mask);
}

static void start_buffer_locked(stream_tx_dma_t *ctx, uint8_t index)
{
    ctx->active_buffer = (int8_t)index;

    dma_channel_set_read_addr(ctx->dma_channel, ctx->buffers[index], false);
    dma_channel_set_trans_count(ctx->dma_channel,
                                STREAM_TX_DMA_WORDS_PER_BUFFER,
                                true);
}

static void stream_tx_dma_irq1_handler(void)
{
    stream_tx_dma_t *ctx = irq_context;
    if (ctx == NULL) {
        return;
    }

    const uint32_t channel_mask = 1u << (uint)ctx->dma_channel;
    if ((dma_hw->ints1 & channel_mask) == 0u) {
        return;
    }

    /* Schrijven van een 1 wist de interruptstatus. */
    dma_hw->ints1 = channel_mask;

    uint32_t irq_state = spin_lock_blocking(ctx->lock);

    if (ctx->running && ctx->active_buffer >= 0) {
        const uint8_t completed = (uint8_t)ctx->active_buffer;
        ctx->active_buffer = -1;
        ctx->free_mask |= (uint16_t)(1u << completed);
        ctx->stats.completed_buffers++;

        const int next = take_ready_buffer_locked(ctx);
        if (next >= 0) {
            start_buffer_locked(ctx, (uint8_t)next);
        } else {
            /* Geen data klaar: TX wacht op een volgende submit. */
            ctx->stats.stalled_transfers++;
            ctx->stats.underruns++;
        }
    }

    spin_unlock(ctx->lock, irq_state);
}

bool stream_tx_dma_init(stream_tx_dma_t *ctx, PIO pio, uint sm)
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
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, pio_get_dreq(pio, sm, true));

    dma_channel_configure(ctx->dma_channel,
                          &config,
                          &pio->txf[sm],
                          ctx->buffers[0],
                          STREAM_TX_DMA_WORDS_PER_BUFFER,
                          false);

    irq_context = ctx;
    irq_set_exclusive_handler(DMA_IRQ_1, stream_tx_dma_irq1_handler);
    dma_channel_set_irq1_enabled(ctx->dma_channel, true);
    irq_set_enabled(DMA_IRQ_1, true);

    ctx->free_mask = (uint16_t)((1u << STREAM_TX_DMA_BUFFER_COUNT) - 1u);
    ctx->initialized = true;
    return true;
}

bool stream_tx_dma_start(stream_tx_dma_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return false;
    }

    stream_tx_dma_stop(ctx);
    pio_sm_clear_fifos(ctx->pio, ctx->sm);
    pio_sm_set_enabled(ctx->pio, ctx->sm, true);

    uint32_t irq_state = spin_lock_blocking(ctx->lock);
    ctx->free_mask = (uint16_t)((1u << STREAM_TX_DMA_BUFFER_COUNT) - 1u);
    ctx->ready_mask = 0u;
    ctx->filling_mask = 0u;
    ctx->active_buffer = -1;
    ctx->running = true;
    spin_unlock(ctx->lock, irq_state);

    return true;
}

void stream_tx_dma_stop(stream_tx_dma_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    uint32_t irq_state = spin_lock_blocking(ctx->lock);
    ctx->running = false;
    spin_unlock(ctx->lock, irq_state);

    dma_channel_abort(ctx->dma_channel);
    dma_hw->ints1 = 1u << (uint)ctx->dma_channel;

    irq_state = spin_lock_blocking(ctx->lock);
    ctx->active_buffer = -1;
    ctx->free_mask = (uint16_t)((1u << STREAM_TX_DMA_BUFFER_COUNT) - 1u);
    ctx->ready_mask = 0u;
    ctx->filling_mask = 0u;
    spin_unlock(ctx->lock, irq_state);
}

bool stream_tx_dma_try_acquire(stream_tx_dma_t *ctx,
                               uint32_t **words,
                               uint8_t *buffer_index)
{
    if (ctx == NULL || words == NULL || buffer_index == NULL ||
        !ctx->initialized || !ctx->running) {
        return false;
    }

    uint32_t irq_state = spin_lock_blocking(ctx->lock);
    const uint8_t free_count = count_free_buffers_locked(ctx);
    if (free_count == 0u) {
        spin_unlock(ctx->lock, irq_state);
        return false;
    }

    /*
     * Keep one free buffer in reserve when possible, but do not force drops
     * when the pipeline has no ready backlog left.
     */
    if (free_count <= STREAM_TX_DMA_RESERVED_FREE_BUFFERS &&
        ctx->ready_mask != 0u) {
        spin_unlock(ctx->lock, irq_state);
        return false;
    }

    const int index = take_free_buffer_locked(ctx);
    if (index < 0) {
        spin_unlock(ctx->lock, irq_state);
        return false;
    }

    const uint16_t bit = (uint16_t)(1u << (uint8_t)index);
    ctx->filling_mask |= bit;
    spin_unlock(ctx->lock, irq_state);

    *words = ctx->buffers[(uint8_t)index];
    *buffer_index = (uint8_t)index;
    return true;
}

void stream_tx_dma_submit(stream_tx_dma_t *ctx, uint8_t buffer_index)
{
    if (ctx == NULL || !ctx->initialized || !ctx->running ||
        buffer_index >= STREAM_TX_DMA_BUFFER_COUNT) {
        return;
    }

    const uint16_t bit = (uint16_t)(1u << buffer_index);
    uint32_t irq_state = spin_lock_blocking(ctx->lock);

    if ((ctx->filling_mask & bit) != 0u) {
        ctx->filling_mask &= (uint16_t)~bit;
        ctx->ready_mask |= bit;

        if (ctx->active_buffer < 0 &&
            count_ready_buffers_locked(ctx) >= STREAM_TX_DMA_PREFILL_READY_COUNT) {
            const int next = take_ready_buffer_locked(ctx);
            if (next >= 0) {
                start_buffer_locked(ctx, (uint8_t)next);
            }
        }
    }

    spin_unlock(ctx->lock, irq_state);
}

stream_tx_dma_stats_t stream_tx_dma_get_stats(stream_tx_dma_t *ctx)
{
    stream_tx_dma_stats_t result = {0};
    if (ctx == NULL || !ctx->initialized) {
        return result;
    }

    uint32_t irq_state = spin_lock_blocking(ctx->lock);
    result = ctx->stats;
    spin_unlock(ctx->lock, irq_state);
    return result;
}

void stream_tx_dma_deinit(stream_tx_dma_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    stream_tx_dma_stop(ctx);
    dma_channel_set_irq1_enabled(ctx->dma_channel, false);
    dma_channel_unclaim(ctx->dma_channel);
    spin_lock_unclaim(spin_lock_get_num(ctx->lock));

    irq_context = NULL;
    ctx->initialized = false;
    ctx->lock = NULL;
    ctx->dma_channel = -1;
}
