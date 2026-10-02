#include <radioberry/radio.h>

#include <string.h>

#include "pico/stdlib.h"
#include "radio_spi.h"
#include "rb-rx.pio.h"
#include "rb-tx.pio.h"

#define RB_CONTROL_COMMAND 0x05u

bool rb_radio_init(rb_radio_t *radio, PIO pio, uint rx_sm, uint tx_sm)
{
    if (radio == NULL || rx_sm == tx_sm) return false;
    memset(radio, 0, sizeof(*radio));
    radio->pio = pio; radio->rx_sm = rx_sm; radio->tx_sm = tx_sm;
    radioberry_spi_init();
    uint rx_offset = pio_add_program(pio, &rx_iq_sample_program);
    rx_iq_sample_program_init(pio, rx_sm, rx_offset);
    if (!stream_rx_dma_init(&radio->rx_dma, pio, rx_sm)) return false;
    uint tx_offset = pio_add_program(pio, &tx_iq_sample_program);
    tx_iq_sample_program_init(pio, tx_sm, tx_offset);
    if (!stream_tx_dma_init(&radio->tx_dma, pio, tx_sm)) {
        stream_rx_dma_deinit(&radio->rx_dma); return false;
    }
    radio->initialized = true;
    return true;
}

bool rb_radio_detect(rb_radio_t *radio, unsigned attempts,
                     unsigned stable_reads, uint32_t delay_ms)
{
    if (radio == NULL || !radio->initialized || stable_reads == 0u) return false;
    uint8_t tx[6] = {0}, rx[6] = {0}, last_major = 0, last_minor = 0;
    unsigned stable = 0;
    for (unsigned n = 0; n < attempts && stable < stable_reads; ++n) {
        radioberry_spi_write_read_blocking(tx, rx, sizeof(tx));
        uint8_t major = rx[4], minor = rx[5];
        if (major != 0u && major != 0xffu && major == last_major &&
            minor == last_minor) stable++;
        else { stable = 0; last_major = major; last_minor = minor; }
        sleep_ms(delay_ms);
    }
    if (stable < stable_reads) return false;
    radio->info = (rb_radio_info_t){.major=last_major, .minor=last_minor,
        .fpga=rx[3]&0x03u, .receiver_count=(rx[3]&0x3cu)>>2u,
        .transmitter_count=(rx[3]&0xc0u)>>6u};
    return true;
}

bool rb_radio_write_control(rb_radio_t *radio, uint8_t command,
                            uint32_t data, uint8_t response[6])
{
    if (radio == NULL || !radio->initialized) return false;
    uint8_t tx[6] = {RB_CONTROL_COMMAND, command, data>>24, data>>16, data>>8, data};
    uint8_t local_response[6] = {0};
    uint8_t *rx = response != NULL ? response : local_response;
    return radioberry_spi_write_read_blocking(tx, rx, sizeof(tx)) == sizeof(tx);
}

bool rb_radio_rx_start(rb_radio_t *r) { return stream_rx_dma_start(&r->rx_dma); }
void rb_radio_rx_stop(rb_radio_t *r) { stream_rx_dma_stop(&r->rx_dma); }
bool rb_radio_rx_acquire(rb_radio_t *r,const uint32_t **w,uint8_t *i)
{ return stream_rx_dma_try_acquire(&r->rx_dma,w,i); }
void rb_radio_rx_release(rb_radio_t *r,uint8_t i){stream_rx_dma_release(&r->rx_dma,i);}
bool rb_radio_tx_start(rb_radio_t *r) { return stream_tx_dma_start(&r->tx_dma); }
void rb_radio_tx_stop(rb_radio_t *r) { stream_tx_dma_stop(&r->tx_dma); }
bool rb_radio_tx_acquire(rb_radio_t *r,uint32_t **w,uint8_t *i)
{ return stream_tx_dma_try_acquire(&r->tx_dma,w,i); }
void rb_radio_tx_submit(rb_radio_t *r,uint8_t i){stream_tx_dma_submit(&r->tx_dma,i);}
rb_radio_stats_t rb_radio_get_stats(rb_radio_t *r) { return (rb_radio_stats_t){
    .rx=stream_rx_dma_get_stats(&r->rx_dma), .tx=stream_tx_dma_get_stats(&r->tx_dma)}; }
void rb_radio_deinit(rb_radio_t *r) { if (r && r->initialized) {
    stream_tx_dma_deinit(&r->tx_dma); stream_rx_dma_deinit(&r->rx_dma);
    r->initialized=false; } }
