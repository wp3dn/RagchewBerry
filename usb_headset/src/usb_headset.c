#include "usb_headset.h"
#include "usb_headset_config.h"

#include <string.h>

#include "pico/stdlib.h"
#include "pico/sync.h"

#include "pio_usb.h"
#include "tusb.h"
#include "host/usbh.h"

#define BYTES_PER_SAMPLE 2u
#define MIC_PACKET_BYTES \
    (USB_HEADSET_FRAME_SAMPLES * USB_HEADSET_MIC_CHANNELS * BYTES_PER_SAMPLE)
#define HP_PACKET_BYTES \
    (USB_HEADSET_FRAME_SAMPLES * USB_HEADSET_PLAY_CHANNELS * BYTES_PER_SAMPLE)

typedef struct {
    bool found;
    uint8_t itf_num;
    uint8_t alt;
    uint8_t ep_addr;
    uint16_t max_packet;
    uint8_t terminal_link;
    tusb_desc_endpoint_t ep_desc;
} audio_choice_t;

static volatile bool g_setup_pending[CFG_TUH_DEVICE_MAX + 1];
static uint8_t g_config_buf[USB_HEADSET_CONFIG_DUMP_BYTES]
    __attribute__((aligned(4)));

static audio_choice_t g_mic;
static audio_choice_t g_hp;
static uint8_t g_audio_dev = 0;
static volatile bool g_streaming = false;

// ISO IN
static uint8_t g_mic_usb_buf[256] __attribute__((aligned(4)));
static tuh_xfer_t g_mic_xfer;
static volatile bool g_mic_need_packet = false;
static volatile bool g_mic_packet_ready = false;
static volatile uint32_t g_mic_ready_len = 0;

// ISO OUT
static uint8_t g_hp_usb_buf[HP_PACKET_BYTES] __attribute__((aligned(4)));
static tuh_xfer_t g_hp_xfer;
static volatile bool g_hp_busy = false;

// Application-facing 1 ms frames.
static int16_t g_mic_frame[USB_HEADSET_FRAME_SAMPLES];
static bool g_mic_frame_unread = false;

static int16_t g_play_frame[USB_HEADSET_FRAME_SAMPLES * 2u];
static bool g_play_frame_pending = false;

// HID
static volatile uint8_t g_button_value = 0;

// Stats
static volatile usb_headset_stats_t g_stats;
static usb_headset_volume_info_t g_volume_info;
static usb_headset_uac_diag_t g_uac_diag;

// Cross-core protection for application-facing frame buffers.
static critical_section_t g_frame_lock;
static bool g_lock_ready = false;

static uint32_t u24le(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16);
}

static bool format_matches_48k(const uint8_t *d,
                               uint8_t len,
                               uint8_t channels) {
    if (len < 8 || d[1] != 0x24 || d[2] != 0x02 || d[3] != 0x01)
        return false;

    if (d[4] != channels || d[5] != BYTES_PER_SAMPLE || d[6] != 16)
        return false;

    uint8_t nfreq = d[7];

    if (nfreq == 0) {
        if (len < 14) return false;
        uint32_t lo = u24le(&d[8]);
        uint32_t hi = u24le(&d[11]);
        return USB_HEADSET_SAMPLE_RATE >= lo &&
               USB_HEADSET_SAMPLE_RATE <= hi;
    }

    uint8_t count = (uint8_t)((len - 8u) / 3u);
    if (nfreq < count) count = nfreq;

    for (uint8_t i = 0; i < count; i++) {
        if (u24le(&d[8u + 3u * i]) == USB_HEADSET_SAMPLE_RATE)
            return true;
    }

    return false;
}

static bool find_audio_alt(uint8_t dev_addr,
                           bool want_in,
                           uint8_t channels,
                           uint16_t required_packet,
                           audio_choice_t *choice) {
    memset(choice, 0, sizeof(*choice));
    memset(g_config_buf, 0, sizeof(g_config_buf));

    if (tuh_descriptor_get_configuration_sync(
            dev_addr, 0, g_config_buf, sizeof(g_config_buf))
        != XFER_RESULT_SUCCESS) {
        return false;
    }

    tusb_desc_configuration_t const *cfg =
        (tusb_desc_configuration_t const *)g_config_buf;

    uint16_t total = cfg->wTotalLength;
    if (total > sizeof(g_config_buf))
        total = sizeof(g_config_buf);

    uint16_t off = cfg->bLength;
    uint8_t cur_itf = 0xff;
    uint8_t cur_alt = 0xff;
    uint8_t cur_class = 0xff;
    uint8_t cur_sub = 0xff;
    bool cur_pcm = false;
    bool cur_format_ok = false;
    uint8_t cur_terminal_link = 0;

    while (off + 2 <= total) {
        const uint8_t *d = &g_config_buf[off];
        uint8_t len = d[0];
        uint8_t type = d[1];

        if (len < 2 || off + len > total)
            break;

        if (type == TUSB_DESC_INTERFACE && len >= 9) {
            cur_itf = d[2];
            cur_alt = d[3];
            cur_class = d[5];
            cur_sub = d[6];
            cur_pcm = false;
            cur_format_ok = false;
            cur_terminal_link = 0;
        } else if (type == 0x24 &&
                   cur_class == TUSB_CLASS_AUDIO &&
                   cur_sub == 2) {
            if (len >= 7 && d[2] == 0x01) {
                cur_terminal_link = d[3];
                uint16_t fmt_tag =
                    (uint16_t)d[5] | ((uint16_t)d[6] << 8);
                cur_pcm = (fmt_tag == 0x0001);
            } else if (d[2] == 0x02) {
                cur_format_ok =
                    format_matches_48k(d, len, channels);
            }
        } else if (type == TUSB_DESC_ENDPOINT &&
                   len >= 7 &&
                   cur_class == TUSB_CLASS_AUDIO &&
                   cur_sub == 2) {
            uint8_t ep_addr = d[2];
            uint8_t attr = d[3];
            uint16_t mps =
                ((uint16_t)d[4] | ((uint16_t)d[5] << 8)) & 0x07ffu;

            bool is_iso = ((attr & 0x03u) == 1u);
            bool dir_in = ((ep_addr & 0x80u) != 0u);

            if (cur_alt != 0 &&
                cur_pcm &&
                cur_format_ok &&
                is_iso &&
                dir_in == want_in &&
                mps >= required_packet) {
                choice->found = true;
                choice->itf_num = cur_itf;
                choice->alt = cur_alt;
                choice->ep_addr = ep_addr;
                choice->max_packet = mps;
                choice->terminal_link = cur_terminal_link;
                memcpy(&choice->ep_desc,
                       d,
                       sizeof(tusb_desc_endpoint_t));
                return true;
            }
        }

        off += len;
    }

    return false;
}

static bool set_endpoint_sample_rate(uint8_t dev_addr,
                                     uint8_t ep_addr,
                                     uint32_t rate) {
    static uint8_t rate3[3];
    static tusb_control_request_t req;
    xfer_result_t result = XFER_RESULT_INVALID;

    rate3[0] = (uint8_t)(rate & 0xff);
    rate3[1] = (uint8_t)((rate >> 8) & 0xff);
    rate3[2] = (uint8_t)((rate >> 16) & 0xff);

    memset(&req, 0, sizeof(req));
    req.bmRequestType_bit.recipient = TUSB_REQ_RCPT_ENDPOINT;
    req.bmRequestType_bit.type = TUSB_REQ_TYPE_CLASS;
    req.bmRequestType_bit.direction = TUSB_DIR_OUT;
    req.bRequest = 0x01;             // UAC1 SET_CUR
    req.wValue = tu_htole16(0x0100); // SAMPLING_FREQ_CONTROL
    req.wIndex = tu_htole16(ep_addr);
    req.wLength = tu_htole16(3);

    tuh_xfer_t xfer = {
        .daddr = dev_addr,
        .ep_addr = 0,
        .setup = &req,
        .buffer = rate3,
        .complete_cb = NULL,
        .user_data = (uintptr_t)&result
    };

    if (!tuh_control_xfer(&xfer))
        return false;

    return result == XFER_RESULT_SUCCESS;
}


static bool uac1_control_xfer_sync(uint8_t dev_addr,
                                   uint8_t direction,
                                   uint8_t request,
                                   uint8_t control_selector,
                                   uint8_t channel,
                                   uint8_t unit_id,
                                   uint8_t control_itf,
                                   void *buffer,
                                   uint16_t length) {
    tusb_control_request_t req;
    xfer_result_t result = XFER_RESULT_INVALID;

    memset(&req, 0, sizeof(req));
    req.bmRequestType_bit.recipient = TUSB_REQ_RCPT_INTERFACE;
    req.bmRequestType_bit.type = TUSB_REQ_TYPE_CLASS;
    req.bmRequestType_bit.direction = direction;
    req.bRequest = request;
    req.wValue = tu_htole16(
        ((uint16_t)control_selector << 8) | channel);
    req.wIndex = tu_htole16(
        ((uint16_t)unit_id << 8) | control_itf);
    req.wLength = tu_htole16(length);

    tuh_xfer_t xfer = {
        .daddr = dev_addr,
        .ep_addr = 0,
        .setup = &req,
        .buffer = (uint8_t *)buffer,
        .complete_cb = NULL,
        .user_data = (uintptr_t)&result
    };

    if (!tuh_control_xfer(&xfer))
        return false;

    return result == XFER_RESULT_SUCCESS;
}

static bool find_playback_feature_unit(uint8_t dev_addr,
                                       uint8_t playback_terminal_link,
                                       uint8_t *control_itf,
                                       uint8_t *feature_id,
                                       uint8_t *control_size,
                                       uint32_t *master_controls) {
    memset(g_config_buf, 0, sizeof(g_config_buf));

    if (tuh_descriptor_get_configuration_sync(
            dev_addr, 0, g_config_buf, sizeof(g_config_buf))
        != XFER_RESULT_SUCCESS) {
        return false;
    }

    tusb_desc_configuration_t const *cfg =
        (tusb_desc_configuration_t const *)g_config_buf;

    uint16_t total = cfg->wTotalLength;
    if (total > sizeof(g_config_buf))
        total = sizeof(g_config_buf);

    uint16_t off = cfg->bLength;
    uint8_t cur_itf = 0xff;
    uint8_t cur_class = 0xff;
    uint8_t cur_sub = 0xff;

    while (off + 2u <= total) {
        const uint8_t *d = &g_config_buf[off];
        uint8_t len = d[0];
        uint8_t type = d[1];

        if (len < 2u || off + len > total)
            break;

        if (type == TUSB_DESC_INTERFACE && len >= 9u) {
            cur_itf = d[2];
            cur_class = d[5];
            cur_sub = d[6];
        } else if (type == 0x24 &&
                   cur_class == TUSB_CLASS_AUDIO &&
                   cur_sub == 1 &&
                   len >= 7u &&
                   d[2] == 0x06) {
            /*
             * UAC1 FEATURE_UNIT:
             *   bUnitID     d[3]
             *   bSourceID   d[4]
             *   bControlSize d[5]
             *   bmaControls(0) starts at d[6]
             *
             * For this headset, and for the common UAC1 playback topology,
             * the speaker Feature Unit is sourced directly by the
             * USB-streaming Input Terminal named by the AS terminal link.
             */
            uint8_t unit_id = d[3];
            uint8_t source_id = d[4];
            uint8_t csize = d[5];

            if (source_id == playback_terminal_link &&
                csize >= 1u &&
                (uint16_t)(6u + csize) <= len) {
                uint32_t controls = 0u;
                uint8_t n = csize > 4u ? 4u : csize;
                for (uint8_t i = 0; i < n; ++i)
                    controls |= ((uint32_t)d[6u + i]) << (8u * i);

                *control_itf = cur_itf;
                *feature_id = unit_id;
                *control_size = csize;
                *master_controls = controls;
                return true;
            }
        }

        off += len;
    }

    return false;
}


static bool uac1_set_volume_raw(uint8_t dev_addr,
                                uint8_t unit_id,
                                uint8_t control_itf,
                                uint8_t channel,
                                int16_t raw_value)
{
    enum {
        UAC1_SET_CUR = 0x01,
        UAC1_VOLUME_CONTROL = 0x02
    };

    uint8_t raw[2];
    raw[0] = (uint8_t)((uint16_t)raw_value & 0xffu);
    raw[1] = (uint8_t)(((uint16_t)raw_value >> 8) & 0xffu);

    return uac1_control_xfer_sync(
        dev_addr, TUSB_DIR_OUT, UAC1_SET_CUR,
        UAC1_VOLUME_CONTROL, channel,
        unit_id, control_itf,
        raw, 2u);
}

static int16_t s16le(const uint8_t b[2])
{
    return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

static void scan_uac1_feature_units(uint8_t dev_addr)
{
    enum {
        UAC1_GET_CUR = 0x81,
        UAC1_GET_MIN = 0x82,
        UAC1_GET_MAX = 0x83,
        UAC1_GET_RES = 0x84,
        UAC1_MUTE_CONTROL = 0x01,
        UAC1_VOLUME_CONTROL = 0x02
    };

    memset(&g_uac_diag, 0, sizeof(g_uac_diag));
    g_uac_diag.playback_terminal_link = g_hp.terminal_link;
    g_uac_diag.mic_terminal_link = g_mic.terminal_link;

    memset(g_config_buf, 0, sizeof(g_config_buf));
    if (tuh_descriptor_get_configuration_sync(
            dev_addr, 0, g_config_buf, sizeof(g_config_buf))
        != XFER_RESULT_SUCCESS) {
        return;
    }

    tusb_desc_configuration_t const *cfg =
        (tusb_desc_configuration_t const *)g_config_buf;
    uint16_t total = cfg->wTotalLength;
    if (total > sizeof(g_config_buf)) total = sizeof(g_config_buf);

    uint16_t off = cfg->bLength;
    uint8_t cur_itf = 0xff;
    uint8_t cur_class = 0xff;
    uint8_t cur_sub = 0xff;

    while (off + 2u <= total &&
           g_uac_diag.count < USB_HEADSET_UAC_DIAG_MAX_CONTROLS) {
        const uint8_t *d = &g_config_buf[off];
        uint8_t len = d[0];
        uint8_t type = d[1];
        if (len < 2u || off + len > total) break;

        if (type == TUSB_DESC_INTERFACE && len >= 9u) {
            cur_itf = d[2];
            cur_class = d[5];
            cur_sub = d[6];
        } else if (type == 0x24 &&
                   cur_class == TUSB_CLASS_AUDIO &&
                   cur_sub == 1 &&
                   len >= 8u &&
                   d[2] == 0x06) {
            uint8_t unit_id = d[3];
            uint8_t source_id = d[4];
            uint8_t csize = d[5];

            if (csize > 0u && len >= (uint8_t)(7u + csize)) {
                /*
                 * UAC1 Feature Unit:
                 * bytes 6.. contain bmaControls(master, ch1, ch2, ...)
                 * and the last byte is iFeature.
                 */
                uint8_t control_count = (uint8_t)((len - 7u) / csize);

                for (uint8_t ch = 0;
                     ch < control_count &&
                     g_uac_diag.count < USB_HEADSET_UAC_DIAG_MAX_CONTROLS;
                     ++ch) {
                    usb_headset_uac_control_diag_t *r =
                        &g_uac_diag.controls[g_uac_diag.count++];

                    memset(r, 0, sizeof(*r));
                    r->valid = true;
                    r->control_itf = cur_itf;
                    r->unit_id = unit_id;
                    r->source_id = source_id;
                    r->channel = ch;

                    uint32_t controls = 0u;
                    uint8_t n = csize > 4u ? 4u : csize;
                    for (uint8_t b = 0; b < n; ++b) {
                        uint16_t pos = (uint16_t)6u +
                                       (uint16_t)ch * csize + b;
                        if (pos < (uint16_t)len - 1u)
                            controls |= ((uint32_t)d[pos]) << (8u * b);
                    }
                    r->advertised_controls = controls;

                    r->mute_advertised =
                        (controls & (1u << 0)) != 0u;
                    r->volume_advertised =
                        (controls & (1u << 1)) != 0u;

                    if (r->mute_advertised) {
                        uint8_t mute = 0xff;
                        r->mute_cur_ok = uac1_control_xfer_sync(
                            dev_addr, TUSB_DIR_IN, UAC1_GET_CUR,
                            UAC1_MUTE_CONTROL, ch,
                            unit_id, cur_itf, &mute, 1u);
                        if (r->mute_cur_ok) r->mute_cur = mute;
                    }

                    if (r->volume_advertised) {
                        uint8_t raw[2];

                        raw[0] = raw[1] = 0;
                        r->volume_cur_ok = uac1_control_xfer_sync(
                            dev_addr, TUSB_DIR_IN, UAC1_GET_CUR,
                            UAC1_VOLUME_CONTROL, ch,
                            unit_id, cur_itf, raw, 2u);
                        if (r->volume_cur_ok)
                            r->volume_cur_raw = s16le(raw);

                        raw[0] = raw[1] = 0;
                        r->volume_min_ok = uac1_control_xfer_sync(
                            dev_addr, TUSB_DIR_IN, UAC1_GET_MIN,
                            UAC1_VOLUME_CONTROL, ch,
                            unit_id, cur_itf, raw, 2u);
                        if (r->volume_min_ok)
                            r->volume_min_raw = s16le(raw);

                        raw[0] = raw[1] = 0;
                        r->volume_max_ok = uac1_control_xfer_sync(
                            dev_addr, TUSB_DIR_IN, UAC1_GET_MAX,
                            UAC1_VOLUME_CONTROL, ch,
                            unit_id, cur_itf, raw, 2u);
                        if (r->volume_max_ok)
                            r->volume_max_raw = s16le(raw);

                        raw[0] = raw[1] = 0;
                        r->volume_res_ok = uac1_control_xfer_sync(
                            dev_addr, TUSB_DIR_IN, UAC1_GET_RES,
                            UAC1_VOLUME_CONTROL, ch,
                            unit_id, cur_itf, raw, 2u);
                        if (r->volume_res_ok)
                            r->volume_res_raw = s16le(raw);
                    }
                }
            }
        }

        off += len;
    }

    g_uac_diag.ready = true;
}

/* Kept for compatibility with the earlier API; v0.4 is read-only. */
static void configure_playback_hardware_volume(uint8_t dev_addr)
{
    /*
     * v0.5 hardware test:
     * The Logitech descriptor diagnostics showed FU22 as a stereo
     * volume control, channels 1 and 2, with range -41..0 dB and
     * both channels sitting at -16 dB.
     *
     * Set ONLY FU22 channels 1 and 2 to 0 dB.
     * Do not change FU19, FU35, or any mute control.
     */
    memset(&g_volume_info, 0, sizeof(g_volume_info));

    scan_uac1_feature_units(dev_addr);

    bool ch1_ok = uac1_set_volume_raw(dev_addr, 22u, 0u, 1u, 0);
    bool ch2_ok = uac1_set_volume_raw(dev_addr, 22u, 0u, 2u, 0);

    /*
     * Re-scan so the diagnostic snapshot reports the values after SET_CUR.
     * The booleans are intentionally not exported yet; the readback tells
     * us whether the headset accepted the requests.
     */
    (void)ch1_ok;
    (void)ch2_ok;
    scan_uac1_feature_units(dev_addr);
}


static void mic_xfer_complete(tuh_xfer_t *xfer) {
    if (!g_streaming)
        return;

    if (xfer->result == XFER_RESULT_SUCCESS) {
        // Important Stage 3.2 fix: save actual_len NOW.
        g_mic_ready_len = xfer->actual_len;
        g_stats.mic_packets++;
        g_mic_packet_ready = true;
    } else {
        g_stats.mic_errors++;
        g_mic_need_packet = true;
    }
}

static void hp_xfer_complete(tuh_xfer_t *xfer) {
    if (!g_streaming)
        return;

    g_hp_busy = false;

    if (xfer->result == XFER_RESULT_SUCCESS)
        g_stats.playback_packets++;
    else
        g_stats.playback_errors++;
}

static bool queue_mic_packet(void) {
    memset(&g_mic_xfer, 0, sizeof(g_mic_xfer));
    g_mic_xfer.daddr = g_audio_dev;
    g_mic_xfer.ep_addr = g_mic.ep_addr;
    g_mic_xfer.buffer = g_mic_usb_buf;
    g_mic_xfer.buflen = MIC_PACKET_BYTES;
    g_mic_xfer.complete_cb = mic_xfer_complete;

    return tuh_edpt_xfer(&g_mic_xfer);
}

static bool queue_hp_packet(void) {
    memset(&g_hp_xfer, 0, sizeof(g_hp_xfer));
    g_hp_xfer.daddr = g_audio_dev;
    g_hp_xfer.ep_addr = g_hp.ep_addr;
    g_hp_xfer.buffer = g_hp_usb_buf;
    g_hp_xfer.buflen = HP_PACKET_BYTES;
    g_hp_xfer.complete_cb = hp_xfer_complete;

    if (!tuh_edpt_xfer(&g_hp_xfer))
        return false;

    g_hp_busy = true;
    return true;
}

static void publish_mic_frame(uint32_t nbytes) {
    if (nbytes > MIC_PACKET_BYTES)
        nbytes = MIC_PACKET_BYTES;

    nbytes &= ~1u;

    int16_t temp[USB_HEADSET_FRAME_SAMPLES];
    memset(temp, 0, sizeof(temp));

    uint32_t samples = nbytes / 2u;
    if (samples > USB_HEADSET_FRAME_SAMPLES)
        samples = USB_HEADSET_FRAME_SAMPLES;

    for (uint32_t i = 0; i < samples; i++) {
        temp[i] =
            (int16_t)((uint16_t)g_mic_usb_buf[2u * i] |
                      ((uint16_t)g_mic_usb_buf[2u * i + 1u] << 8));
    }

    critical_section_enter_blocking(&g_frame_lock);
    memcpy(g_mic_frame, temp, sizeof(g_mic_frame));
    g_mic_frame_unread = true;
    critical_section_exit(&g_frame_lock);
}

static void prepare_next_playback_frame(void) {
    int16_t temp[USB_HEADSET_FRAME_SAMPLES * 2u];
    bool have_frame = false;
    static int16_t last_l = 0;
    static int16_t last_r = 0;
    static bool recovering_from_gap = false;

    critical_section_enter_blocking(&g_frame_lock);
    if (g_play_frame_pending) {
        memcpy(temp, g_play_frame, sizeof(temp));
        g_play_frame_pending = false;
        have_frame = true;
    }
    critical_section_exit(&g_frame_lock);

    if (!have_frame) {
        /* Port16: never turn a late 1 ms producer slot into a hard-zero
         * discontinuity.  Hold the last delivered sample until a fresh
         * frame arrives. */
        for (uint32_t i = 0; i < USB_HEADSET_FRAME_SAMPLES; ++i) {
            temp[2u * i] = last_l;
            temp[2u * i + 1u] = last_r;
        }
        recovering_from_gap = true;
    } else if (recovering_from_gap) {
        /* Bridge gently back into the live waveform after a missed slot.
         * Eight samples is only 167 us at 48 kHz and avoids a single hard
         * step from the held sample to an arbitrary new waveform sample. */
        const uint32_t bridge = 8u;
        for (uint32_t i = 0; i < bridge; ++i) {
            const int32_t a = (int32_t)(bridge - i);
            const int32_t b = (int32_t)(i + 1u);
            temp[2u * i] = (int16_t)((a * last_l + b * temp[2u * i]) / (int32_t)(bridge + 1u));
            temp[2u * i + 1u] = (int16_t)((a * last_r + b * temp[2u * i + 1u]) / (int32_t)(bridge + 1u));
        }
        recovering_from_gap = false;
    }

    last_l = temp[(USB_HEADSET_FRAME_SAMPLES - 1u) * 2u];
    last_r = temp[(USB_HEADSET_FRAME_SAMPLES - 1u) * 2u + 1u];

    for (uint32_t i = 0;
         i < USB_HEADSET_FRAME_SAMPLES * 2u;
         i++) {
        uint16_t s = (uint16_t)temp[i];
        g_hp_usb_buf[2u * i] = (uint8_t)(s & 0xffu);
        g_hp_usb_buf[2u * i + 1u] = (uint8_t)(s >> 8);
    }
}

static bool configure_streams(uint8_t dev_addr) {
    if (!find_audio_alt(dev_addr,
                        true,
                        USB_HEADSET_MIC_CHANNELS,
                        MIC_PACKET_BYTES,
                        &g_mic)) {
        return false;
    }

    if (!find_audio_alt(dev_addr,
                        false,
                        USB_HEADSET_PLAY_CHANNELS,
                        HP_PACKET_BYTES,
                        &g_hp)) {
        return false;
    }

    xfer_result_t result = XFER_RESULT_INVALID;

    if (!tuh_interface_set(dev_addr,
                           g_mic.itf_num,
                           g_mic.alt,
                           NULL,
                           (uintptr_t)&result) ||
        result != XFER_RESULT_SUCCESS) {
        return false;
    }

    if (!tuh_edpt_open(dev_addr, &g_mic.ep_desc))
        return false;

    // The proven headset accepts this. If another headset ignores it,
    // descriptor selection still guarantees a compatible 48 kHz alternate.
    (void)set_endpoint_sample_rate(
        dev_addr, g_mic.ep_addr, USB_HEADSET_SAMPLE_RATE);

    result = XFER_RESULT_INVALID;

    if (!tuh_interface_set(dev_addr,
                           g_hp.itf_num,
                           g_hp.alt,
                           NULL,
                           (uintptr_t)&result) ||
        result != XFER_RESULT_SUCCESS) {
        return false;
    }

    if (!tuh_edpt_open(dev_addr, &g_hp.ep_desc))
        return false;

    (void)set_endpoint_sample_rate(
        dev_addr, g_hp.ep_addr, USB_HEADSET_SAMPLE_RATE);

    /*
     * Hardware-test derivative only: raise the headset's own UAC1
     * playback Feature Unit to its advertised maximum.
     */
    configure_playback_hardware_volume(dev_addr);

    g_audio_dev = dev_addr;

    memset((void *)&g_stats, 0, sizeof(g_stats));

    critical_section_enter_blocking(&g_frame_lock);
    memset(g_mic_frame, 0, sizeof(g_mic_frame));
    memset(g_play_frame, 0, sizeof(g_play_frame));
    g_mic_frame_unread = false;
    g_play_frame_pending = false;
    critical_section_exit(&g_frame_lock);

    g_button_value = 0;
    g_mic_packet_ready = false;
    g_mic_need_packet = true;
    g_hp_busy = false;
    g_streaming = true;

    return true;
}

static void setup_task(void) {
    for (uint8_t d = 1; d <= CFG_TUH_DEVICE_MAX; d++) {
        if (g_setup_pending[d]) {
            g_setup_pending[d] = false;
            (void)configure_streams(d);
        }
    }
}

bool usb_headset_init(void) {
    if (!g_lock_ready) {
        critical_section_init(&g_frame_lock);
        g_lock_ready = true;
    }

    memset((void *)g_setup_pending, 0, sizeof(g_setup_pending));
    memset((void *)&g_stats, 0, sizeof(g_stats));

    g_audio_dev = 0;
    g_streaming = false;
    g_button_value = 0;
    memset(&g_volume_info, 0, sizeof(g_volume_info));
    memset(&g_uac_diag, 0, sizeof(g_uac_diag));

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = USB_HEADSET_DP_PIN;
    pio_cfg.pio_tx_num = USB_HEADSET_PIO_TX_NUM;
    pio_cfg.pio_rx_num = USB_HEADSET_PIO_RX_NUM;
    pio_cfg.sm_tx = USB_HEADSET_SM_TX;
    pio_cfg.sm_rx = USB_HEADSET_SM_RX;
    pio_cfg.sm_eop = USB_HEADSET_SM_EOP;
    pio_cfg.tx_ch = USB_HEADSET_DMA_TX_CH;

    tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
    return tuh_init(1);
}

void usb_headset_task(void) {
    tuh_task();
    setup_task();

    /*
     * Preserve the cadence that was proven by Stage 4:
     * every completed 1 ms MIC frame drives one 1 ms headphone OUT slot.
     */
    if (g_streaming && g_mic_packet_ready) {
        uint32_t got = g_mic_ready_len;
        g_mic_packet_ready = false;

        publish_mic_frame(got);

        if (!g_hp_busy) {
            prepare_next_playback_frame();

            if (!queue_hp_packet())
                g_stats.playback_errors++;
        } else {
            g_stats.playback_drops++;
        }

        g_mic_need_packet = true;
    }

    if (g_streaming &&
        g_mic_need_packet &&
        !g_mic_packet_ready) {
        g_mic_need_packet = false;

        if (!queue_mic_packet()) {
            g_stats.mic_errors++;
            g_mic_need_packet = true;
        }
    }
}

bool usb_headset_connected(void) {
    return g_streaming;
}

bool usb_headset_get_mic_frame(
    int16_t out_mono[USB_HEADSET_FRAME_SAMPLES]) {
    if (!out_mono || !g_lock_ready)
        return false;

    bool have = false;

    critical_section_enter_blocking(&g_frame_lock);
    if (g_mic_frame_unread) {
        memcpy(out_mono, g_mic_frame, sizeof(g_mic_frame));
        g_mic_frame_unread = false;
        have = true;
    }
    critical_section_exit(&g_frame_lock);

    return have;
}

bool usb_headset_submit_playback_stereo(
    const int16_t interleaved_lr[USB_HEADSET_FRAME_SAMPLES * 2u]) {
    if (!interleaved_lr || !g_lock_ready)
        return false;

    critical_section_enter_blocking(&g_frame_lock);
    memcpy(g_play_frame,
           interleaved_lr,
           sizeof(g_play_frame));
    g_play_frame_pending = true;
    critical_section_exit(&g_frame_lock);

    return true;
}

bool usb_headset_submit_playback_mono(
    const int16_t mono[USB_HEADSET_FRAME_SAMPLES]) {
    if (!mono || !g_lock_ready)
        return false;

    int16_t stereo[USB_HEADSET_FRAME_SAMPLES * 2u];

    for (uint32_t i = 0; i < USB_HEADSET_FRAME_SAMPLES; i++) {
        stereo[2u * i] = mono[i];
        stereo[2u * i + 1u] = mono[i];
    }

    return usb_headset_submit_playback_stereo(stereo);
}

uint8_t usb_headset_button_value(void) {
    return g_button_value;
}

bool usb_headset_ptt_pressed(void) {
    return g_button_value == USB_HEADSET_PTT_VALUE;
}

bool usb_headset_tune_pressed(void) {
    return g_button_value == USB_HEADSET_TUNE_VALUE;
}


void usb_headset_get_playback_volume_info(
    usb_headset_volume_info_t *out) {
    if (!out)
        return;

    *out = g_volume_info;
}

void usb_headset_get_uac_diag(usb_headset_uac_diag_t *out)
{
    if (out == NULL) return;
    memcpy(out, &g_uac_diag, sizeof(*out));
}


void usb_headset_get_stats(usb_headset_stats_t *out) {
    if (!out)
        return;

    out->mic_packets = g_stats.mic_packets;
    out->mic_errors = g_stats.mic_errors;
    out->playback_packets = g_stats.playback_packets;
    out->playback_errors = g_stats.playback_errors;
    out->playback_drops = g_stats.playback_drops;
}

void usb_headset_reset_stats(void) {
    memset((void *)&g_stats, 0, sizeof(g_stats));
}

/* ---------------- TinyUSB host callbacks owned by this package ---------------- */

void tuh_mount_cb(uint8_t dev_addr) {
    if (dev_addr <= CFG_TUH_DEVICE_MAX)
        g_setup_pending[dev_addr] = true;
}

void tuh_umount_cb(uint8_t dev_addr) {
    if (g_audio_dev == dev_addr) {
        g_streaming = false;
        g_audio_dev = 0;
        g_button_value = 0;

        critical_section_enter_blocking(&g_frame_lock);
        g_mic_frame_unread = false;
        g_play_frame_pending = false;
        critical_section_exit(&g_frame_lock);
    }
}

void tuh_hid_mount_cb(uint8_t dev_addr,
                      uint8_t instance,
                      uint8_t const *desc_report,
                      uint16_t desc_len) {
    (void)desc_report;
    (void)desc_len;
    tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
    (void)dev_addr;
    (void)instance;
}

void tuh_hid_report_received_cb(uint8_t dev_addr,
                                uint8_t instance,
                                uint8_t const *report,
                                uint16_t len) {
    /*
     * Normalize the two bench-proven HID report formats.
     * Logitech:          03 01 / 03 02 / 03 00
     * C-Media 8086:0808: 01 00 00 00 / 02 00 00 00 / 00 00 00 00
     * C-Media S=04 and latching MUTE=08 are deliberately ignored.
     */
    if (len >= 2 && report[0] == 0x03) {
        g_button_value = report[1];
    } else if (len == 4 &&
               report[1] == 0x00 &&
               report[2] == 0x00 &&
               report[3] == 0x00) {
        uint8_t v = report[0];
        g_button_value = (v == 0x01u || v == 0x02u) ? v : 0x00u;
    }

    tuh_hid_receive_report(dev_addr, instance);
}
