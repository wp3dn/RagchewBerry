#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { bool initialized,link_up,discovered,control_seen,mox_requested; uint32_t rx_frequency_hz,tx_frequency_hz,discovery_count,control_count; bool rx_streaming; uint32_t rx_packets_sent,rx_packets_dropped,tx_iq_frames_received,tx_iq_frames_dropped,tx_iq_underflows; } p1_network_status_t;
bool p1_network_start(void); void p1_network_stop(void); void p1_network_task(void);
bool p1_network_queue_rx_iq(const int32_t *i24,const int32_t *q24);
int p1_network_read_tx_iq(int16_t *iq_interleaved,int max_frames);
int p1_network_tx_iq_available(void); void p1_network_flush_tx_iq(void); void p1_network_note_tx_underflow(void);
const p1_network_status_t *p1_network_status(void);
