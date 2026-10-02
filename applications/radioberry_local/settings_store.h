#ifndef RAGCHEWBERRY_SETTINGS_STORE_H
#define RAGCHEWBERRY_SETTINGS_STORE_H

#include <stdbool.h>
#include <stdint.h>

#define RCB_PERSIST_BAND_COUNT 10u

typedef struct {
    uint32_t current_frequency_hz;
    uint32_t band_frequency_hz[RCB_PERSIST_BAND_COUNT];
    uint32_t mode;
    uint8_t band_mode[RCB_PERSIST_BAND_COUNT];
    uint32_t tune_step_hz;
    uint32_t rx_filter_high_hz;
    uint32_t band_rf_gain_percent[RCB_PERSIST_BAND_COUNT];
    int32_t band_spectrum_offset_db[RCB_PERSIST_BAND_COUNT];
    uint32_t band_tx_iq_gain_x100[RCB_PERSIST_BAND_COUNT];
    uint32_t spectrum_span_hz;
    uint32_t agc_max_gain;
    uint32_t af_percent;
    uint32_t mic_gain_percent;
    uint32_t tx_drive;
    uint32_t tune_level_percent;
    uint32_t reserved[4];
} rcb_persist_payload_t;

/* Scan the two-sector journal and return the newest valid record. */
bool rcb_settings_store_load(rcb_persist_payload_t *out, uint32_t *sequence_out);

/* Append one atomic page record. When a sector fills, rotate to the other
 * sector so the previous valid sector survives until the new record exists. */
bool rcb_settings_store_save(const rcb_persist_payload_t *settings,
                             uint32_t *sequence_in_out);

#endif
