#include "settings_store.h"

#include <stddef.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/regs/addressmap.h"

#define RCB_SETTINGS_MAGIC       0x52434253u /* 'RCBS' */
#define RCB_SETTINGS_VERSION     4u
#define RCB_SETTINGS_SECTORS     2u
#define RCB_SETTINGS_PAGE_BYTES  FLASH_PAGE_SIZE
#define RCB_SETTINGS_SECTOR_BYTES FLASH_SECTOR_SIZE
#define RCB_SETTINGS_TOTAL_BYTES (RCB_SETTINGS_SECTORS * RCB_SETTINGS_SECTOR_BYTES)
#define RCB_SETTINGS_SLOTS_PER_SECTOR (RCB_SETTINGS_SECTOR_BYTES / RCB_SETTINGS_PAGE_BYTES)
#define RCB_SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - RCB_SETTINGS_TOTAL_BYTES)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t crc32;
    rcb_persist_payload_t payload;
} rcb_settings_record_t;

_Static_assert(sizeof(rcb_settings_record_t) <= RCB_SETTINGS_PAGE_BYTES,
               "settings record must fit in one flash page");
/* v0.7.22/v3 payload, retained so v0.7.23 can migrate the existing journal. */
typedef struct {
    uint32_t current_frequency_hz;
    uint32_t band_frequency_hz[10];
    uint32_t mode;
    uint32_t tune_step_hz;
    uint32_t rx_filter_high_hz;
    uint32_t band_rf_gain_percent[10];
    int32_t band_spectrum_offset_db[10];
    uint32_t band_tx_iq_gain_x100[10];
    uint32_t spectrum_span_hz;
    uint32_t agc_max_gain;
    uint32_t af_percent;
    uint32_t mic_gain_percent;
    uint32_t tx_drive;
    uint32_t tune_level_percent;
    uint32_t reserved[4];
} rcb_persist_payload_v3_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t crc32;
    rcb_persist_payload_v3_t payload;
} rcb_settings_record_v3_t;

/* v0.7.15 payload, retained so v0.7.16 can migrate the existing journal. */
typedef struct {
    uint32_t current_frequency_hz;
    uint32_t band_frequency_hz[10];
    uint32_t mode;
    uint32_t tune_step_hz;
    uint32_t rx_filter_high_hz;
    uint32_t band_rf_gain_percent[10];
    int32_t band_spectrum_offset_db[10];
    uint32_t spectrum_span_hz;
    uint32_t agc_max_gain;
    uint32_t af_percent;
    uint32_t mic_gain_percent;
    uint32_t tx_drive;
    uint32_t tune_level_percent;
    uint32_t reserved[4];
} rcb_persist_payload_v2_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t crc32;
    rcb_persist_payload_v2_t payload;
} rcb_settings_record_v2_t;

/* v0.7.13/v0.7.14 payload, retained so v0.7.16 can migrate the newest
 * existing journal record without throwing away the user's settings. */
typedef struct {
    uint32_t current_frequency_hz;
    uint32_t band_frequency_hz[11];
    uint32_t mode;
    uint32_t tune_step_hz;
    uint32_t rx_filter_high_hz;
    uint32_t rf_gain_percent;
    uint32_t agc_max_gain;
    uint32_t af_percent;
    uint32_t mic_gain_percent;
    uint32_t tx_drive;
    uint32_t tune_level_percent;
    uint32_t reserved[12];
} rcb_persist_payload_v1_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t crc32;
    rcb_persist_payload_v1_t payload;
} rcb_settings_record_v1_t;


typedef struct {
    bool found;
    unsigned sector;
    unsigned slot;
    uint32_t sequence;
    rcb_persist_payload_t payload;
} scan_result_t;

static uint32_t crc32_bytes(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xffffffffu;
    while (len--) {
        crc ^= *data++;
        for (unsigned b = 0; b < 8u; ++b)
            crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
    return ~crc;
}

static bool seq_newer(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}

static const uint8_t *page_ptr(unsigned sector, unsigned slot)
{
    const uint32_t off = RCB_SETTINGS_FLASH_OFFSET +
                         sector * RCB_SETTINGS_SECTOR_BYTES +
                         slot * RCB_SETTINGS_PAGE_BYTES;
    return (const uint8_t *)(XIP_BASE + off);
}

static bool page_is_blank(unsigned sector, unsigned slot)
{
    const uint8_t *p = page_ptr(sector, slot);
    for (unsigned i = 0; i < RCB_SETTINGS_PAGE_BYTES; ++i)
        if (p[i] != 0xffu) return false;
    return true;
}

static bool record_valid(const rcb_settings_record_t *r)
{
    if (r->magic != RCB_SETTINGS_MAGIC) return false;
    if (r->version != RCB_SETTINGS_VERSION) return false;
    if (r->payload_size != sizeof(rcb_persist_payload_t)) return false;
    return crc32_bytes((const uint8_t *)&r->payload, sizeof(r->payload)) == r->crc32;
}

static scan_result_t scan_journal(void)
{
    scan_result_t best = {0};

    for (unsigned s = 0; s < RCB_SETTINGS_SECTORS; ++s) {
        for (unsigned slot = 0; slot < RCB_SETTINGS_SLOTS_PER_SECTOR; ++slot) {
            const uint8_t *page = page_ptr(s, slot);
            const rcb_settings_record_t *r = (const rcb_settings_record_t *)page;
            bool valid = record_valid(r);
            rcb_persist_payload_t converted;
            uint32_t seq = r->sequence;

            if (valid) {
                memcpy(&converted, &r->payload, sizeof(converted));
            } else {
                const rcb_settings_record_v3_t *v3 = (const rcb_settings_record_v3_t *)page;
                if (v3->magic == RCB_SETTINGS_MAGIC && v3->version == 3u &&
                    v3->payload_size == sizeof(rcb_persist_payload_v3_t) &&
                    crc32_bytes((const uint8_t *)&v3->payload, sizeof(v3->payload)) == v3->crc32) {
                    memset(&converted, 0, sizeof(converted));
                    converted.current_frequency_hz = v3->payload.current_frequency_hz;
                    converted.mode = v3->payload.mode;
                    for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i) {
                        converted.band_frequency_hz[i] = v3->payload.band_frequency_hz[i];
                        converted.band_mode[i] = v3->payload.mode;
                        converted.band_rf_gain_percent[i] = v3->payload.band_rf_gain_percent[i];
                        converted.band_spectrum_offset_db[i] = v3->payload.band_spectrum_offset_db[i];
                        converted.band_tx_iq_gain_x100[i] = v3->payload.band_tx_iq_gain_x100[i];
                    }
                    converted.tune_step_hz = v3->payload.tune_step_hz;
                    converted.rx_filter_high_hz = v3->payload.rx_filter_high_hz;
                    converted.spectrum_span_hz = v3->payload.spectrum_span_hz;
                    converted.agc_max_gain = v3->payload.agc_max_gain;
                    converted.af_percent = v3->payload.af_percent;
                    converted.mic_gain_percent = v3->payload.mic_gain_percent;
                    converted.tx_drive = v3->payload.tx_drive;
                    converted.tune_level_percent = v3->payload.tune_level_percent;
                    memcpy(converted.reserved, v3->payload.reserved, sizeof(converted.reserved));
                    seq = v3->sequence;
                } else {
                    const rcb_settings_record_v2_t *v2 = (const rcb_settings_record_v2_t *)page;
                    if (v2->magic == RCB_SETTINGS_MAGIC && v2->version == 2u &&
                        v2->payload_size == sizeof(rcb_persist_payload_v2_t) &&
                        crc32_bytes((const uint8_t *)&v2->payload, sizeof(v2->payload)) == v2->crc32) {
                        memset(&converted, 0, sizeof(converted));
                        converted.current_frequency_hz = v2->payload.current_frequency_hz;
                        for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i) {
                            converted.band_frequency_hz[i] = v2->payload.band_frequency_hz[i];
                            converted.band_mode[i] = v2->payload.mode;
                            converted.band_rf_gain_percent[i] = v2->payload.band_rf_gain_percent[i];
                            converted.band_spectrum_offset_db[i] = v2->payload.band_spectrum_offset_db[i];
                            converted.band_tx_iq_gain_x100[i] = 40u;
                        }
                        converted.mode = v2->payload.mode;
                        converted.tune_step_hz = v2->payload.tune_step_hz;
                        converted.rx_filter_high_hz = v2->payload.rx_filter_high_hz;
                        converted.spectrum_span_hz = v2->payload.spectrum_span_hz;
                        converted.agc_max_gain = v2->payload.agc_max_gain;
                        converted.af_percent = v2->payload.af_percent;
                        converted.mic_gain_percent = v2->payload.mic_gain_percent;
                        converted.tx_drive = v2->payload.tx_drive;
                        converted.tune_level_percent = v2->payload.tune_level_percent;
                        seq = v2->sequence;
                    } else {
                    const rcb_settings_record_v1_t *old = (const rcb_settings_record_v1_t *)page;
                    if (old->magic != RCB_SETTINGS_MAGIC || old->version != 1u ||
                        old->payload_size != sizeof(rcb_persist_payload_v1_t) ||
                        crc32_bytes((const uint8_t *)&old->payload, sizeof(old->payload)) != old->crc32)
                        continue;

                    memset(&converted, 0, sizeof(converted));
                    converted.current_frequency_hz = old->payload.current_frequency_hz;
                    for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i) {
                        converted.band_frequency_hz[i] = old->payload.band_frequency_hz[i];
                        converted.band_rf_gain_percent[i] = old->payload.rf_gain_percent;
                        converted.band_spectrum_offset_db[i] = 0;
                        converted.band_tx_iq_gain_x100[i] = 40u;
                    }
                    converted.mode = old->payload.mode;
                    converted.tune_step_hz = old->payload.tune_step_hz;
                    converted.rx_filter_high_hz = old->payload.rx_filter_high_hz;
                    converted.spectrum_span_hz = 48000u;
                    converted.agc_max_gain = old->payload.agc_max_gain;
                    converted.af_percent = old->payload.af_percent;
                    converted.mic_gain_percent = old->payload.mic_gain_percent;
                    converted.tx_drive = old->payload.tx_drive;
                    converted.tune_level_percent = old->payload.tune_level_percent;
                    for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i)
                        converted.band_mode[i] = old->payload.mode;
                    seq = old->sequence;
                    }
                }
            }

            if (!best.found || seq_newer(seq, best.sequence)) {
                best.found = true;
                best.sector = s;
                best.slot = slot;
                best.sequence = seq;
                best.payload = converted;
            }
        }
    }
    return best;
}

bool rcb_settings_store_load(rcb_persist_payload_t *out, uint32_t *sequence_out)
{
    if (!out || !sequence_out) return false;
    const scan_result_t best = scan_journal();
    if (!best.found) return false;
    *out = best.payload;
    *sequence_out = best.sequence;
    return true;
}

static void flash_write_locked(uint32_t page_offset, const uint8_t page[FLASH_PAGE_SIZE],
                               bool erase_sector, uint32_t erase_offset)
{
    /* Core1 registered as a lockout victim before the radio loop starts.
     * Hold it in SRAM while XIP is unavailable, and keep core0 IRQ handlers
     * from trying to execute out of flash during erase/program. */
    multicore_lockout_start_blocking();
    const uint32_t irq_state = save_and_disable_interrupts();
    if (erase_sector)
        flash_range_erase(erase_offset, FLASH_SECTOR_SIZE);
    flash_range_program(page_offset, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq_state);
    multicore_lockout_end_blocking();
}

bool rcb_settings_store_save(const rcb_persist_payload_t *settings,
                             uint32_t *sequence_in_out)
{
    if (!settings || !sequence_in_out) return false;

    scan_result_t best = scan_journal();
    unsigned target_sector = 0u;
    unsigned target_slot = 0u;
    bool erase_target = false;

    if (best.found) {
        target_sector = best.sector;

        /* Prefer the first genuinely blank page after the newest record. */
        bool found_blank = false;
        for (unsigned slot = best.slot + 1u;
             slot < RCB_SETTINGS_SLOTS_PER_SECTOR; ++slot) {
            if (page_is_blank(target_sector, slot)) {
                target_slot = slot;
                found_blank = true;
                break;
            }
        }

        if (!found_blank) {
            /* Rotate. Erase the other sector, but leave the sector containing
             * the previous valid record untouched until the new page exists. */
            target_sector ^= 1u;
            target_slot = 0u;
            erase_target = true;
        }
    } else {
        /* Fresh firmware normally sees erased flash here. If not, clean only
         * sector 0 and leave sector 1 untouched. */
        target_sector = 0u;
        target_slot = 0u;
        erase_target = !page_is_blank(0u, 0u);
    }

    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));

    rcb_settings_record_t record;
    memset(&record, 0, sizeof(record));
    record.magic = RCB_SETTINGS_MAGIC;
    record.version = RCB_SETTINGS_VERSION;
    record.payload_size = sizeof(record.payload);
    record.sequence = best.found ? best.sequence + 1u : (*sequence_in_out + 1u);
    record.payload = *settings;
    record.crc32 = crc32_bytes((const uint8_t *)&record.payload, sizeof(record.payload));
    memcpy(page, &record, sizeof(record));

    const uint32_t sector_offset = RCB_SETTINGS_FLASH_OFFSET +
                                   target_sector * RCB_SETTINGS_SECTOR_BYTES;
    const uint32_t page_offset = sector_offset + target_slot * RCB_SETTINGS_PAGE_BYTES;

    flash_write_locked(page_offset, page, erase_target, sector_offset);

    /* Verify through XIP after flash is restored. */
    const rcb_settings_record_t *written =
        (const rcb_settings_record_t *)(XIP_BASE + page_offset);
    if (!record_valid(written) || written->sequence != record.sequence)
        return false;

    *sequence_in_out = record.sequence;
    return true;
}
