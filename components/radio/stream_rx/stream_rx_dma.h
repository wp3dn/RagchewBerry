#ifndef STREAM_RX_DMA_H
#define STREAM_RX_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/pio.h"
#include "hardware/sync.h"

/* Eén HPSDR-pakket bevat 126 I/Q-paren: 126 * 2 PIO-woorden. */
#define STREAM_RX_DMA_BUFFER_COUNT       2u
#define STREAM_RX_DMA_IQ_PAIRS           126u
#define STREAM_RX_DMA_WORDS_PER_BUFFER  (STREAM_RX_DMA_IQ_PAIRS * 2u)
/* 126 IQ-paren × 2 woorden (I en Q). */
#define STREAM_RX_DMA_WORDS 252u

typedef struct {
    bool have_i;
    uint32_t i_sample;
    uint16_t pair_count;
} iq_assembler_t;

typedef struct {
    uint32_t completed_buffers;
    uint32_t stalled_transfers;
} stream_rx_dma_stats_t;

typedef struct {
    PIO pio;
    uint sm;
    int dma_channel;

    uint32_t buffers[STREAM_RX_DMA_BUFFER_COUNT]
                    [STREAM_RX_DMA_WORDS_PER_BUFFER];

    spin_lock_t *lock;
    uint8_t free_mask;
    uint8_t ready_mask;
    uint8_t acquired_mask;
    int8_t active_buffer;

    bool initialized;
    bool running;
    stream_rx_dma_stats_t stats;
} stream_rx_dma_t;

/*
 * Initialiseert één DMA-kanaal voor de RX FIFO van pio/sm.
 * Deze module gebruikt DMA_IRQ_0 en beheert daarvan de exclusive handler.
 */
bool stream_rx_dma_init(stream_rx_dma_t *ctx, PIO pio, uint sm);

/* Start een nieuwe stream. Eventuele oude bufferstatus wordt gewist. */
bool stream_rx_dma_start(stream_rx_dma_t *ctx);

/* Stop de DMA en maak alle buffers weer vrij. */
void stream_rx_dma_stop(stream_rx_dma_t *ctx);

/*
 * Haal zonder blokkeren een volle buffer op.
 * De data blijft geldig totdat stream_rx_dma_release() wordt aangeroepen.
 */
bool stream_rx_dma_try_acquire(stream_rx_dma_t *ctx,
                               const uint32_t **words,
                               uint8_t *buffer_index);

/* Geef een eerder verkregen buffer terug aan de DMA. */
void stream_rx_dma_release(stream_rx_dma_t *ctx, uint8_t buffer_index);

/* Momentopname van de diagnostische tellers. */
stream_rx_dma_stats_t stream_rx_dma_get_stats(stream_rx_dma_t *ctx);

/* Stop en geef DMA-kanaal en spinlock terug aan de SDK. */
void stream_rx_dma_deinit(stream_rx_dma_t *ctx);


#endif /* STREAM_RX_DMA_H */
