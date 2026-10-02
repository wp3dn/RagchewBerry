#ifndef RADIOBERRY_RADIO_H
#define RADIOBERRY_RADIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/pio.h"
#include "stream_rx_dma.h"
#include "stream_tx_dma.h"

typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t fpga;
    uint8_t receiver_count;
    uint8_t transmitter_count;
} rb_radio_info_t;

typedef struct {
    stream_rx_dma_stats_t rx;
    stream_tx_dma_stats_t tx;
} rb_radio_stats_t;

typedef struct {
    PIO pio;
    uint rx_sm;
    uint tx_sm;
    stream_rx_dma_t rx_dma;
    stream_tx_dma_t tx_dma;
    rb_radio_info_t info;
    bool initialized;
} rb_radio_t;

bool rb_radio_init(rb_radio_t *radio, PIO pio, uint rx_sm, uint tx_sm);
bool rb_radio_detect(rb_radio_t *radio, unsigned attempts,
                     unsigned stable_reads, uint32_t delay_ms);
bool rb_radio_write_control(rb_radio_t *radio, uint8_t command,
                            uint32_t data, uint8_t response[6]);

bool rb_radio_rx_start(rb_radio_t *radio);
void rb_radio_rx_stop(rb_radio_t *radio);
bool rb_radio_rx_acquire(rb_radio_t *radio, const uint32_t **words,
                         uint8_t *buffer_index);
void rb_radio_rx_release(rb_radio_t *radio, uint8_t buffer_index);

bool rb_radio_tx_start(rb_radio_t *radio);
void rb_radio_tx_stop(rb_radio_t *radio);
bool rb_radio_tx_acquire(rb_radio_t *radio, uint32_t **words,
                         uint8_t *buffer_index);
void rb_radio_tx_submit(rb_radio_t *radio, uint8_t buffer_index);

rb_radio_stats_t rb_radio_get_stats(rb_radio_t *radio);
void rb_radio_deinit(rb_radio_t *radio);

#endif /* RADIOBERRY_RADIO_H */
