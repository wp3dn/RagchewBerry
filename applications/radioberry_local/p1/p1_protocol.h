#pragma once
#include <stdint.h>
#define P1_PACKET_BYTES 1032
#define P1_SAMPLES_PER_PACKET 126
void p1_protocol_reset(void);
void p1_build_discovery_reply(uint8_t *reply, int len);
int p1_classify(const uint8_t *buf, int len);
void p1_build_ep6(uint8_t *pkt,const int32_t *i24,const int32_t *q24,const int16_t *mic16);
int p1_unpack_ep2(const uint8_t *pkt,int len,int16_t *iq16,int max_frames,int *mox,uint32_t *tx_hz,uint32_t *rx_hz);
