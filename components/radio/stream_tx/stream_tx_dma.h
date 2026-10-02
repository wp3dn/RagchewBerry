#ifndef STREAM_TX_DMA_H
#define STREAM_TX_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/pio.h"
#include "hardware/sync.h"

/* Eén HPSDR-batch bevat 126 I/Q-paren: 1 woord per paar (I16|Q16). */
#define STREAM_TX_DMA_BUFFER_COUNT       4u
#define STREAM_TX_DMA_IQ_PAIRS           126u
#define STREAM_TX_DMA_WORDS_PER_BUFFER   4096u
#define STREAM_TX_DMA_RESERVED_FREE_BUFFERS 0u
#define STREAM_TX_DMA_WORDS              STREAM_TX_DMA_WORDS_PER_BUFFER

typedef struct {
    uint32_t completed_buffers;
    uint32_t stalled_transfers;
    uint32_t underruns;
} stream_tx_dma_stats_t;

typedef struct {
    PIO pio;
    uint sm;
    int dma_channel;

    uint32_t buffers[STREAM_TX_DMA_BUFFER_COUNT]
                    [STREAM_TX_DMA_WORDS_PER_BUFFER];

    spin_lock_t *lock;
    uint16_t free_mask;
    uint16_t ready_mask;
    uint16_t filling_mask;
    int8_t active_buffer;

    bool initialized;
    bool running;
    stream_tx_dma_stats_t stats;
} stream_tx_dma_t;

/*
 * Initialiseert één DMA-kanaal voor de TX FIFO van pio/sm.
 * Deze module gebruikt DMA_IRQ_1 en beheert daarvan de exclusive handler.
 */
bool stream_tx_dma_init(stream_tx_dma_t *ctx, PIO pio, uint sm);

/* Start een nieuwe TX-stream. Eventuele oude bufferstatus wordt gewist. */
bool stream_tx_dma_start(stream_tx_dma_t *ctx);

/* Stop de DMA en maak alle buffers weer vrij. */
void stream_tx_dma_stop(stream_tx_dma_t *ctx);

/*
 * Haal zonder blokkeren een lege schrijfbuffer op.
 * De producer vult STREAM_TX_DMA_WORDS woorden en commit daarna met submit.
 */
bool stream_tx_dma_try_acquire(stream_tx_dma_t *ctx,
                               uint32_t **words,
                               uint8_t *buffer_index);

/* Commit een gevulde buffer voor verzending richting PIO TX FIFO. */
void stream_tx_dma_submit(stream_tx_dma_t *ctx, uint8_t buffer_index);

/* Momentopname van de diagnostische tellers. */
stream_tx_dma_stats_t stream_tx_dma_get_stats(stream_tx_dma_t *ctx);

/* Stop en geef DMA-kanaal en spinlock terug aan de SDK. */
void stream_tx_dma_deinit(stream_tx_dma_t *ctx);

#endif /* STREAM_TX_DMA_H */
