#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/clocks.h"
#include "pico/bootrom.h"

#include "remote_ui.h"
#include "settings_store.h"

#include "tusb.h"
#include "p1_network.h"
#include "usb_headset.h"

#include <radioberry/gateware.h>
#include "gateware_pico.h"
#include "radioberry_rbf.h"
#include <radioberry/radio.h>
#include <radioberry/dsp.h>

/*
 * WP3DN RagchewBerry v0.7.17 - AM mode
 *
 * First actual local receiver:
 *   Radioberry 48 kHz complex IQ
 *       -> 256 point STFT using the project's rb_fft_forward/inverse()
 *       -> USB/LSB frequency-domain sideband filter
 *       -> overlap/add
 *       -> simple block AGC + AF gain
 *       -> USB headset at 48 kHz
 *
 * Native USB-C remains TinyUSB CDC diagnostics/control.
 * PIO USB J2 remains the proven headset host on GP28/GP29 / PIO2.
 */

#define SAMPLE_RATE             48000u
#define FFT_N                   256u
#define FFT_HOP                 (FFT_N / 2u)

#define AUDIO_RING_SAMPLES      2048u
#define TX_FRAME_RING_COUNT      12u

#define RX_AUDIO_LOW_HZ_DEFAULT  100.0f
#define RX_AUDIO_HIGH_HZ_DEFAULT 2700.0f

#define TX_AUDIO_LOW_HZ         300.0f
#define TX_AUDIO_HIGH_HZ        2700.0f
#define TUNE_TONE_HZ            1000.0f
#define TX_MONITOR_RF_SCALE     0.02f  /* -34 dB while transmitting */

/* GPA-10 RF deck control.
 * GP16 = single PTT input, active HIGH (HIGH -> TX, LOW -> RX).
 * The GPA-10 performs its own RF switching; there is no external coax/PA
 * sequencer in this profile.  LPF selection is owned by the Display RP2350. */
#define GPA10_PTT_GPIO          16u

/* Port10 CW Stage 1: dedicated straight-key input.
 * GP0 idles HIGH on its internal pull-up; grounding it keys CW. */
#define CW_KEY_GPIO              0u
#define CW_PITCH_HZ_DEFAULT      700u
#define CW_HANG_MS_DEFAULT        300u
#define CW_EDGE_SAMPLES          240u  /* 5.00 ms at 48 kHz */

#define DEFAULT_FREQ_HZ         14074000u
#define DEFAULT_AF_PERCENT      100u
#define TUNE_STEP_HZ            1000u
#define SETTINGS_SAVE_DELAY_MS  5000u

typedef enum {
    MODE_LSB = 0,
    MODE_USB = 1,
    MODE_AM  = 2,
    MODE_CWL = 3,
    MODE_CWU = 4
} rx_mode_t;

static rb_radio_t g_radio;
static volatile bool g_radio_ready = false;

static volatile uint32_t g_rx_pairs = 0;
static volatile uint32_t g_rx_bad_tags = 0;
static volatile uint32_t g_fft_blocks = 0;
static volatile float g_fft_peak = 0.0f;
static volatile int g_fft_peak_bin = 0;

static volatile uint32_t g_frequency_hz = DEFAULT_FREQ_HZ;
static volatile bool g_p1_mode = false;
static volatile uint32_t g_p1_last_rx_hz = 0u;
static volatile uint32_t g_p1_last_tx_hz = 0u;
static volatile rx_mode_t g_mode = MODE_USB;
static volatile uint32_t g_cw_wpm = 20u;
static volatile uint32_t g_cw_pitch_hz = CW_PITCH_HZ_DEFAULT;
static volatile uint32_t g_cw_hang_ms = CW_HANG_MS_DEFAULT;
static volatile uint32_t g_af_percent = DEFAULT_AF_PERCENT;
static volatile uint32_t g_rf_gain_percent = 100u;
static volatile float g_rf_gain_scale = 1.0f / 8388608.0f;
static volatile uint32_t g_mic_gain_percent = 50u;
/* Port8 TX voice processing. Defaults are intentionally transparent. */
static volatile uint32_t g_tx_audio_percent = 100u;
static volatile int32_t g_tx_eq_low_db = 0;
static volatile int32_t g_tx_eq_mid_db = 0;
static volatile int32_t g_tx_eq_high_db = 0;
static volatile uint32_t g_tx_comp_level = 0u;
static float g_tx_comp_gain = 1.0f;
static volatile uint32_t g_tx_drive = 4u;
static volatile uint32_t g_tx_iq_gain_x100 = 40u;
static volatile uint32_t g_tune_level_percent = 50u;
static volatile uint32_t g_am_rx_bw_hz = 6000u;
static volatile uint32_t g_am_carrier_percent = 50u;
static float g_am_dc = 0.0f;
static volatile bool g_tune_active = false;
static volatile bool g_tx_request = false;
static uint64_t g_tune_next_frame_us = 0u; /* core1-owned while TUNE is active */
static int16_t g_tune_frame[USB_HEADSET_FRAME_SAMPLES];
static uint32_t g_tune_frame_level_percent = 0u;
static volatile float g_rx_audio_low_hz = RX_AUDIO_LOW_HZ_DEFAULT;
static volatile float g_rx_audio_high_hz = RX_AUDIO_HIGH_HZ_DEFAULT;
static volatile float g_agc_max_gain = 500.0f;
static volatile uint32_t g_tune_step_hz = 100u;
/* Local RIT: dial/TX remain fixed; only RX NCO moves. Not persisted. */
static bool g_rit_on = false;
static int16_t g_rit_hz = 0;
static volatile bool g_tx_active = false;

/* Forward declaration: CW Stage 1 writes directly into the existing TX DMA path. */
static bool tx_put_word(uint32_t word);

/* Port10 CW Stage 1.  Ragchew Machine uses a ~5 ms Blackman-Harris
 * forward/reverse envelope.  RagchewBerry runs TX IQ at 48 kHz, so 240
 * samples preserves that same edge time instead of stretching the 96 kHz
 * reference table to ~10 ms. */
static float g_cw_envelope[CW_EDGE_SAMPLES + 1u];
static uint16_t g_cw_envelope_pos = 0u;
static volatile bool g_cw_tx_active = false;
static volatile float g_cw_monitor_env = 0.0f;
static uint64_t g_cw_next_frame_us = 0u;
static uint32_t g_cw_sidetone_phase = 0u;
static uint32_t g_cw_rf_phase = 0u;
static uint64_t g_cw_hang_deadline_us = 0u;

static void cw_envelope_init(void)
{
    /* 4-term Blackman-Harris, edge->center, normalized exactly 0..1. */
    const float a0=0.35875f, a1=0.48829f, a2=0.14128f, a3=0.01168f;
    const float w0 = a0 - a1 + a2 - a3;
    const float wc = a0 + a1 + a2 + a3;
    for (unsigned n=0; n<=CW_EDGE_SAMPLES; ++n) {
        float x = (float)n / (float)CW_EDGE_SAMPLES;
        float t = 3.14159265358979323846f * x;
        float w = a0 - a1*cosf(t) + a2*cosf(2.0f*t) - a3*cosf(3.0f*t);
        float e = (w - w0) / (wc - w0);
        if (e < 0.0f) e = 0.0f;
        if (e > 1.0f) e = 1.0f;
        g_cw_envelope[n] = e;
    }
    g_cw_envelope[0] = 0.0f;
    g_cw_envelope[CW_EDGE_SAMPLES] = 1.0f;
}

static inline float cw_envelope_next_48k(bool key_down)
{
    if (key_down) {
        if (g_cw_envelope_pos < CW_EDGE_SAMPLES) ++g_cw_envelope_pos;
    } else {
        if (g_cw_envelope_pos > 0u) --g_cw_envelope_pos;
    }
    return g_cw_envelope[g_cw_envelope_pos];
}

static void cw_feed_frame(bool key_down)
{
    const float iq_gain = (float)g_tx_iq_gain_x100 * 0.01f;
    const uint32_t pitch = g_cw_pitch_hz;
    const uint32_t inc = (uint32_t)(((double)pitch * 4294967296.0) / (double)SAMPLE_RATE);
    const int dir = (g_mode == MODE_CWL) ? 1 : -1; /* corrected for RagchewBerry IQ convention */
    for (unsigned n=0; n<USB_HEADSET_FRAME_SAMPLES; ++n) {
        float env = cw_envelope_next_48k(key_down);
        g_cw_monitor_env = env;
        g_cw_rf_phase = (dir > 0) ? (g_cw_rf_phase + inc) : (g_cw_rf_phase - inc);
        float ph = (float)g_cw_rf_phase * (6.2831853071795864769f / 4294967296.0f);
        float i = cosf(ph) * env * iq_gain;
        float q = sinf(ph) * env * iq_gain;
        if (i > 0.95f) i = 0.95f; if (i < -0.95f) i = -0.95f;
        if (q > 0.95f) q = 0.95f; if (q < -0.95f) q = -0.95f;
        int16_t si = (int16_t)lrintf(i * 32767.0f);
        int16_t sq = (int16_t)lrintf(q * 32767.0f);
        uint32_t packed = ((uint32_t)(uint16_t)si << 16) | (uint16_t)sq;
        if (!tx_put_word(packed)) break;
    }
}

static void cw_make_sidetone_frame(int16_t out[USB_HEADSET_FRAME_SAMPLES])
{
    const uint32_t inc = (uint32_t)(((double)g_cw_pitch_hz * 4294967296.0) / (double)SAMPLE_RATE);
    float env = g_cw_monitor_env;
    for (unsigned n=0; n<USB_HEADSET_FRAME_SAMPLES; ++n) {
        g_cw_sidetone_phase += inc;
        float ph = (float)g_cw_sidetone_phase * (6.2831853071795864769f / 4294967296.0f);
        float y = sinf(ph) * env * 0.25f;
        out[n] = (int16_t)lrintf(y * 32767.0f);
    }
}


typedef enum {
    RFSEQ_RX = 0,
    RFSEQ_TX_ACTIVE
} rfseq_state_t;

static rfseq_state_t g_rfseq_state = RFSEQ_RX;
static volatile uint8_t g_spectrum[RCB_OLED_WIDTH];
static volatile int32_t g_spectrum_offset_db = 0;
static volatile uint32_t g_spectrum_span_hz = 48000u;
static volatile float g_smeter_rms = 0.0f;
static volatile uint32_t g_tx_words_generated = 0;
static volatile uint32_t g_tx_submit_count = 0;
static volatile uint32_t g_tx_acquire_waits = 0;

/* Persistence coordination. Flash XIP is briefly unavailable while a page is
 * programmed, so core1 pauses the Radioberry RX stream before core0 commits. */
static volatile bool g_settings_pause_request = false;
static volatile bool g_settings_core1_paused = false;
static uint32_t g_settings_sequence = 0u;
static bool g_settings_dirty = false;
static uint32_t g_settings_dirty_since_ms = 0u;

/* Core0 -> core1 TX audio frame ring. Core0 owns USB host/mic capture;
 * core1 owns all TX FFT/IFFT and Radioberry TX DMA production. */
static critical_section_t g_tx_frame_cs;
static int16_t g_tx_frame_ring[TX_FRAME_RING_COUNT][USB_HEADSET_FRAME_SAMPLES];
static uint8_t g_tx_frame_wr = 0u;
static uint8_t g_tx_frame_rd = 0u;
static uint8_t g_tx_frame_count = 0u;
static volatile uint32_t g_tx_frame_drops = 0u;

/* Forward declarations used by the TX path, which appears before helpers. */
static void cdc_write(const char *text);
static void cdc_printf(const char *fmt, ...);
static const char *mode_name(void);
static unsigned nearest_band_index(uint32_t hz);
static void settings_note_frequency(uint32_t hz);

/* Shared RX/TX Hann window. Initialized by dsp_init(). */
static float g_window[FFT_N];


/*
 * TX path: headset microphone -> 256 point FFT/IFFT analytic SSB -> Radioberry.
 *
 * The Radioberry SPI command byte is the original Protocol-1 C0 byte:
 *   bits 6:1 = command address, bit 0 = PTT/MOX.
 * Therefore TX1 frequency (ADDR 1) is C0=0x02 and RX1 frequency
 * (ADDR 2) is C0=0x04.  This is also how the FPGA extracts cmd_addr/cmd_ptt.
 */
static rb_iq_f32_t g_tx_time[FFT_N];
static rb_iq_f32_t g_tx_fft[FFT_N];
static float g_tx_ola_i[FFT_HOP];
static float g_tx_ola_q[FFT_HOP];
static size_t g_tx_fill = 0u;

static uint32_t *g_tx_dma_words = NULL;
static uint8_t g_tx_dma_index = 0u;
static size_t g_tx_dma_used = 0u;

static void tx_frame_ring_reset(void)
{
    critical_section_enter_blocking(&g_tx_frame_cs);
    g_tx_frame_wr = 0u;
    g_tx_frame_rd = 0u;
    g_tx_frame_count = 0u;
    critical_section_exit(&g_tx_frame_cs);
}

static void tx_frame_ring_push(const int16_t frame[USB_HEADSET_FRAME_SAMPLES])
{
    critical_section_enter_blocking(&g_tx_frame_cs);

    if (g_tx_frame_count == TX_FRAME_RING_COUNT) {
        /* Keep latency bounded: discard the oldest 1 ms frame. */
        g_tx_frame_rd = (uint8_t)((g_tx_frame_rd + 1u) % TX_FRAME_RING_COUNT);
        g_tx_frame_count--;
        g_tx_frame_drops++;
    }

    memcpy(g_tx_frame_ring[g_tx_frame_wr], frame,
           sizeof(g_tx_frame_ring[g_tx_frame_wr]));
    g_tx_frame_wr = (uint8_t)((g_tx_frame_wr + 1u) % TX_FRAME_RING_COUNT);
    g_tx_frame_count++;

    critical_section_exit(&g_tx_frame_cs);
}

static bool tx_frame_ring_pop(int16_t frame[USB_HEADSET_FRAME_SAMPLES])
{
    bool have = false;
    critical_section_enter_blocking(&g_tx_frame_cs);
    if (g_tx_frame_count != 0u) {
        memcpy(frame, g_tx_frame_ring[g_tx_frame_rd],
               sizeof(g_tx_frame_ring[g_tx_frame_rd]));
        g_tx_frame_rd = (uint8_t)((g_tx_frame_rd + 1u) % TX_FRAME_RING_COUNT);
        g_tx_frame_count--;
        have = true;
    }
    critical_section_exit(&g_tx_frame_cs);
    return have;
}

static void tx_stream_reset(void)
{
    tx_frame_ring_reset();
    g_tx_dma_words = NULL;
    g_tx_dma_index = 0u;
    g_tx_dma_used = 0u;
    g_tx_fill = 0u;
    memset(g_tx_time, 0, sizeof(g_tx_time));
    memset(g_tx_fft, 0, sizeof(g_tx_fft));
    memset(g_tx_ola_i, 0, sizeof(g_tx_ola_i));
    memset(g_tx_ola_q, 0, sizeof(g_tx_ola_q));
    g_tx_comp_gain = 1.0f;
}

static void uac_diag_flush_line(void)
{
    /* Native USB is dedicated to the Protocol-1 network. */
}


static bool tx_put_word(uint32_t word)
{
    if (g_tx_dma_words == NULL) {
        if (!rb_radio_tx_acquire(&g_radio, &g_tx_dma_words, &g_tx_dma_index)) {
            g_tx_acquire_waits++;
            return false;
        }
        g_tx_dma_used = 0u;
    }

    g_tx_dma_words[g_tx_dma_used++] = word;
    g_tx_words_generated++;

    if (g_tx_dma_used == STREAM_TX_DMA_WORDS_PER_BUFFER) {
        rb_radio_tx_submit(&g_radio, g_tx_dma_index);
        g_tx_submit_count++;
        g_tx_dma_words = NULL;
        g_tx_dma_used = 0u;
    }
    return true;
}

static void tx_filter_sideband_bins(rb_iq_f32_t *bins)
{
    const float hz_per_bin = (float)SAMPLE_RATE / (float)FFT_N;
    int low_bin = (int)ceilf(TX_AUDIO_LOW_HZ / hz_per_bin);
    int high_bin = (int)floorf(TX_AUDIO_HIGH_HZ / hz_per_bin);
    if (low_bin < 1) low_bin = 1;
    if (high_bin > (int)(FFT_N / 2u - 1u))
        high_bin = (int)(FFT_N / 2u - 1u);

    rx_mode_t mode = g_mode;

    for (int k = 0; k < (int)FFT_N; ++k) {
        bool keep = false;

        /*
         * First TX trial: conventional analytic orientation.
         * If RF USB/LSB is reversed on hardware, only this selection swaps.
         */
        /*
         * Hardware test v0.2 proved the Radioberry TX complex spectrum
         * orientation is opposite the conventional analytic assumption.
         * Swap the signed-bin selection so commanded USB/LSB matches RF.
         */
        if (mode == MODE_AM && !g_tune_active) {
            /* Proven Ragchew Machine AM TX architecture, translated to the
             * Radioberry complex-baseband FFT: retain both microphone
             * sidebands. The carrier itself is added after OLA below. */
            const int neg_low = (int)FFT_N - high_bin;
            const int neg_high = (int)FFT_N - low_bin;
            keep = (k >= low_bin && k <= high_bin) ||
                   (k >= neg_low && k <= neg_high);
        } else if (mode == MODE_LSB || mode == MODE_CWL) {
            keep = (k >= low_bin && k <= high_bin);
        } else {
            const int neg_low = (int)FFT_N - high_bin;
            const int neg_high = (int)FFT_N - low_bin;
            keep = (k >= neg_low && k <= neg_high);
        }

        if (!keep) {
            bins[k].i = 0.0f;
            bins[k].q = 0.0f;
        } else {
            if (mode == MODE_AM && !g_tune_active) {
                /* Same conservative modulation scale as the proven
                 * Ragchew Machine Fourier AM transmitter. */
                bins[k].i *= 0.35f;
                bins[k].q *= 0.35f;
            } else {
                /* Analytic SSB/TUNE: restore energy removed with the opposite half. */
                bins[k].i *= 2.0f;
                bins[k].q *= 2.0f;
            }
        }
    }
}

static void tx_apply_voice_eq(rb_iq_f32_t *bins)
{
    if (g_tune_active) return;

    const int low_db = g_tx_eq_low_db;
    const int mid_db = g_tx_eq_mid_db;
    const int high_db = g_tx_eq_high_db;
    if (low_db == 0 && mid_db == 0 && high_db == 0) return;

    /* Three broad speech bands. This operates on the microphone FFT before
     * the existing TX sideband filter, so it does not disturb RX or the
     * proven RF/IQ calibration. */
    const float low_gain = powf(10.0f, (float)low_db / 20.0f);
    const float mid_gain = powf(10.0f, (float)mid_db / 20.0f);
    const float high_gain = powf(10.0f, (float)high_db / 20.0f);
    const float hz_per_bin = (float)SAMPLE_RATE / (float)FFT_N;

    for (int k = 0; k < (int)FFT_N; ++k) {
        int signed_bin = (k <= (int)(FFT_N / 2u)) ? k : k - (int)FFT_N;
        float hz = fabsf((float)signed_bin * hz_per_bin);
        float gain = 1.0f;
        if (hz >= 300.0f && hz < 700.0f) gain = low_gain;
        else if (hz >= 700.0f && hz < 1500.0f) gain = mid_gain;
        else if (hz >= 1500.0f && hz <= 2700.0f) gain = high_gain;
        bins[k].i *= gain;
        bins[k].q *= gain;
    }
}

static float tx_compressor_frame_gain(const int16_t mic[USB_HEADSET_FRAME_SAMPLES])
{
    const uint32_t level = g_tx_comp_level;
    if (g_tune_active || level == 0u) {
        g_tx_comp_gain = 1.0f;
        return 1.0f;
    }

    /* Port9 speech compressor: deliberately stronger than Port8.
     *
     * Port8 mostly reduced peaks and then added only a small amount of
     * make-up gain. That sounded clean, but normal speech stayed around
     * 1-2 W while a whistle could reach full output. Here COMP 1..10 first
     * adds controlled drive, then applies 3:1 compression above a falling
     * threshold. This raises average speech energy while keeping strong
     * single-frequency peaks from simply getting louder.
     *
     * Approximate pre-drive:
     *   COMP 1  ~= +1 dB
     *   COMP 5  ~= +6 dB
     *   COMP 10 ~= +11 dB
     *
     * TUNE still bypasses the compressor. TX AUDIO, EQ, MIC and TX-IQ
     * behavior are otherwise unchanged from Port8. */
    int32_t peak_i = 0;
    for (size_t n = 0; n < USB_HEADSET_FRAME_SAMPLES; ++n) {
        int32_t v = mic[n];
        if (v < 0) v = -v;
        if (v > peak_i) peak_i = v;
    }

    const float peak = (float)peak_i / 32768.0f;

    /* Same useful threshold progression as Port8: about -7.6 dBFS at
     * COMP 1 down to about -20 dBFS at COMP 10. */
    float threshold = 0.46f - 0.036f * (float)level;
    if (threshold < 0.10f) threshold = 0.10f;

    /* Stronger speech drive. 2.5 dB + 0.7 dB/step gives about +6 dB at
     * COMP 5 and +9.5 dB at COMP 10. A small extra lift at the top end
     * makes COMP 10 land close to +11 dB without making mid settings wild. */
    float drive_db = 2.5f + 0.7f * (float)level;
    if (level >= 8u) drive_db += 0.5f * (float)(level - 7u);
    const float drive = powf(10.0f, drive_db / 20.0f);

    float target = drive;
    if (peak > 1.0e-6f) {
        const float driven_peak = peak * drive;
        if (driven_peak > threshold) {
            const float compressed = threshold + (driven_peak - threshold) / 3.0f;
            target = compressed / peak;
        }
    }

    /* Keep the frame gain bounded. The per-sample 0.98 limiter in
     * tx_feed_mic_frame() remains the final protection before the FFT. */
    if (target > 4.0f) target = 4.0f;
    if (target < 0.20f) target = 0.20f;

    /* Faster gain reduction, slower recovery/make-up. Updated once per
     * 1 ms USB microphone frame to avoid pumping between individual samples. */
    const float a = (target < g_tx_comp_gain) ? 0.35f : 0.025f;
    g_tx_comp_gain += a * (target - g_tx_comp_gain);
    return g_tx_comp_gain;
}

static void tx_process_fft_block(void)
{
    /* Cache the menu-controlled microphone gain once per DSP block.
     * Keep the 48 kHz USB mic ingest path identical to the proven v0.6.5
     * baseline: no per-sample floating-point divide/clamp was added there.
     */
    const float tx_iq_gain = (float)g_tx_iq_gain_x100 * 0.01f;
    const float tx_block_gain = tx_iq_gain *
        (g_tune_active ? 1.0f : ((float)g_mic_gain_percent * 0.01f));
    /* Ragchew Machine capped 100% AM carrier at 12000/32768 peak.
     * Keep that same conservative normalized carrier before per-band IQ gain. */
    const float am_carrier = (12000.0f / 32768.0f) *
        ((float)g_am_carrier_percent * 0.01f) * tx_iq_gain;

    for (size_t n = 0; n < FFT_N; ++n) {
        g_tx_fft[n].i = g_tx_time[n].i * g_window[n];
        g_tx_fft[n].q = 0.0f;
    }

    if (rb_fft_forward(g_tx_fft, FFT_N) != 0) return;
    tx_apply_voice_eq(g_tx_fft);
    tx_filter_sideband_bins(g_tx_fft);
    if (rb_fft_inverse(g_tx_fft, FFT_N) != 0) return;

    for (size_t n = 0; n < FFT_HOP; ++n) {
        float i = g_tx_fft[n].i + g_tx_ola_i[n];
        float q = g_tx_fft[n].q + g_tx_ola_q[n];

        g_tx_ola_i[n] = g_tx_fft[n + FFT_HOP].i;
        g_tx_ola_q[n] = g_tx_fft[n + FFT_HOP].q;

        i *= tx_block_gain;
        q *= tx_block_gain;
        if (g_mode == MODE_AM && !g_tune_active) {
            i += am_carrier;
            q = 0.0f;
        }

        if (i > 0.95f) i = 0.95f;
        if (i < -0.95f) i = -0.95f;
        if (q > 0.95f) q = 0.95f;
        if (q < -0.95f) q = -0.95f;

        int16_t si = (int16_t)lrintf(i * 32767.0f);
        int16_t sq = (int16_t)lrintf(q * 32767.0f);
        uint32_t packed = ((uint32_t)(uint16_t)si << 16) |
                          (uint32_t)(uint16_t)sq;

        if (!tx_put_word(packed)) break;
    }
}

static void tx_feed_mic_frame(const int16_t mic[USB_HEADSET_FRAME_SAMPLES])
{
    const float comp_gain = tx_compressor_frame_gain(mic);
    const float audio_gain = (float)g_tx_audio_percent * 0.01f;
    const bool processing_active = (g_tx_comp_level != 0u) || (g_tx_audio_percent != 100u);
    const float pre_gain = comp_gain * audio_gain;

    for (size_t m = 0; m < USB_HEADSET_FRAME_SAMPLES; ++m) {
        /* Port8 keeps the proven mic ingest unchanged when TX AUDIO=100 and
         * COMP=0. Voice level/compression are inserted ahead of the FFT. */
        float x = (float)mic[m] / 32768.0f;
        if (processing_active) {
            x *= pre_gain;
            if (x > 0.98f) x = 0.98f;
            if (x < -0.98f) x = -0.98f;
        }
        g_tx_time[g_tx_fill].i = x;
        g_tx_time[g_tx_fill].q = 0.0f;
        g_tx_fill++;

        if (g_tx_fill == FFT_N) {
            tx_process_fft_block();
            memmove(&g_tx_time[0], &g_tx_time[FFT_HOP],
                    FFT_HOP * sizeof(g_tx_time[0]));
            g_tx_fill = FFT_HOP;
        }
    }
}

static void set_radio_ptt(bool on)
{
    /* Local split NCO operation: ADDR0 DATA bit2 = Duplex.
     * Preserve it on BOTH PTT edges. P1 retains its prior control behavior. */
    (void)rb_radio_write_control(&g_radio, on ? 0x01u : 0x00u,
                                 g_p1_mode ? 0u : 0x00000004u, NULL);
}

static void set_tx_frequency(uint32_t hz)
{
    /* Protocol-1 ADDR 1 -> wire C0 0x02. */
    (void)rb_radio_write_control(&g_radio, 0x02u, hz, NULL);
}

static void set_rx_frequency(uint32_t hz)
{
    /* Protocol-1 ADDR 2 -> wire C0 0x04. */
    (void)rb_radio_write_control(&g_radio, 0x04u, hz, NULL);
}

static void refresh_tune_frame(void)
{
    uint32_t pct = g_tune_level_percent;
    if (pct == g_tune_frame_level_percent) return;

    /* 1 kHz at 48 kHz is exactly 48 samples/cycle, exactly one USB frame.
     * Build the waveform only when TUNE LVL changes; never call sinf() in
     * the 1 ms USB/TX service path. */
    const float level = 0.70f * ((float)pct * 0.01f);
    for (size_t n = 0; n < USB_HEADSET_FRAME_SAMPLES; ++n) {
        float phase = 2.0f * (float)M_PI * (float)n /
                      (float)USB_HEADSET_FRAME_SAMPLES;
        g_tune_frame[n] = (int16_t)lrintf(sinf(phase) * level * 32767.0f);
    }
    g_tune_frame_level_percent = pct;
}

static inline void gpa10_ptt_set(bool on)
{
    /* GPA-10 control is active HIGH: HIGH = TX, LOW = RX. */
    gpio_put(GPA10_PTT_GPIO, on ? 1 : 0);
}

static void rfdeck_force_rx_safe(void)
{
    set_radio_ptt(false);
    gpa10_ptt_set(false);
    g_rfseq_state = RFSEQ_RX;
    g_tx_active = false;
    if (g_p1_mode) rcb_ui_frequency(g_frequency_hz);
}

static void tx_start(void)
{
    if (g_rfseq_state != RFSEQ_RX) return;

    tx_stream_reset();
    uint32_t tx_hz = (g_p1_mode && g_p1_last_tx_hz) ? g_p1_last_tx_hz : g_frequency_hz;
    set_tx_frequency(tx_hz);
    if (g_p1_mode) rcb_ui_frequency(tx_hz);  /* LPF follows P1 TX frequency. */

    /* Conservative Hermes TX drive nibble = 4/15. ADDR 9 -> C0 0x12. */
    (void)rb_radio_write_control(&g_radio, 0x12u,
                                 (uint32_t)(g_tx_drive & 0x0fu) << 28, NULL);

    /* Start the Radioberry TX stream with MOX still OFF. */
    if (!rb_radio_tx_start(&g_radio)) {
        rfdeck_force_rx_safe();
        return;
    }

    /* GPA-10 needs only one active-high PTT. No external sequencer. */
    gpa10_ptt_set(true);
    g_tx_active = true;
    g_rfseq_state = RFSEQ_TX_ACTIVE;
    set_radio_ptt(true);
}

static void tx_stop(void)
{
    if (g_rfseq_state == RFSEQ_RX) return;

    /* Remove RF first, then release the GPA-10 PTT. */
    set_radio_ptt(false);
    gpa10_ptt_set(false);
    rb_radio_tx_stop(&g_radio);
    tx_stream_reset();

    g_rfseq_state = RFSEQ_RX;
    g_tx_active = false;
}

static inline void rfdeck_sequence_service(void)
{
    /* GPA-10 profile intentionally has no external sequencer. */
}

/* RX word parser */
static bool g_have_i = false;
static uint32_t g_i24 = 0;
static uint8_t g_expected_tag = 0;

/* STFT input and work arrays */
static rb_iq_f32_t g_input[FFT_N];
static rb_iq_f32_t g_fft[FFT_N];
static float g_ola_tail[FFT_HOP];
static float g_ola_tail_q[FFT_HOP];
static size_t g_input_fill = 0u;

/* Simple block AGC */
static volatile float g_agc_gain = 1.0f;
static volatile float g_last_rms = 0.0f;

/* Cross-core AF ring: core1 produces, core0 consumes. */
static critical_section_t g_audio_cs;
static int16_t g_audio_ring[AUDIO_RING_SAMPLES];
static size_t g_audio_wr = 0u;
static size_t g_audio_rd = 0u;
static size_t g_audio_count = 0u;
static volatile uint32_t g_audio_overruns = 0u;
static volatile uint32_t g_audio_underruns = 0u;

static void cdc_write(const char *text)
{
    (void)text; /* Network-only native USB: no CDC console. */
}

static void cdc_printf(const char *fmt, ...)
{
    (void)fmt; /* Network-only native USB: no CDC console. */
}

static int32_t sign24(uint32_t v)
{
    v &= 0x00ffffffu;
    return (v & 0x00800000u) ?
        (int32_t)(v | 0xff000000u) : (int32_t)v;
}

static void audio_ring_push_block(const int16_t *samples, size_t count)
{
    critical_section_enter_blocking(&g_audio_cs);

    for (size_t n = 0; n < count; ++n) {
        if (g_audio_count == AUDIO_RING_SAMPLES) {
            /* Drop the oldest sample: keep latency bounded. */
            g_audio_rd = (g_audio_rd + 1u) % AUDIO_RING_SAMPLES;
            g_audio_count--;
            g_audio_overruns++;
        }

        g_audio_ring[g_audio_wr] = samples[n];
        g_audio_wr = (g_audio_wr + 1u) % AUDIO_RING_SAMPLES;
        g_audio_count++;
    }

    critical_section_exit(&g_audio_cs);
}

static bool audio_ring_pop_frame(int16_t out[USB_HEADSET_FRAME_SAMPLES])
{
    bool complete = true;

    critical_section_enter_blocking(&g_audio_cs);

    if (g_audio_count < USB_HEADSET_FRAME_SAMPLES) {
        complete = false;
    } else {
        for (size_t n = 0; n < USB_HEADSET_FRAME_SAMPLES; ++n) {
            out[n] = g_audio_ring[g_audio_rd];
            g_audio_rd = (g_audio_rd + 1u) % AUDIO_RING_SAMPLES;
            g_audio_count--;
        }
    }

    critical_section_exit(&g_audio_cs);

    if (!complete) {
        memset(out, 0, sizeof(int16_t) * USB_HEADSET_FRAME_SAMPLES);
        g_audio_underruns++;
    }
    return complete;
}

static size_t audio_ring_level(void)
{
    size_t level;
    critical_section_enter_blocking(&g_audio_cs);
    level = g_audio_count;
    critical_section_exit(&g_audio_cs);
    return level;
}

static void audio_ring_trim_latency(void)
{
    /*
     * Stage 2 hardware test showed that RX starts producing before the USB
     * headset finishes enumeration, leaving ~30 ms of stale audio queued.
     * Keep only a few milliseconds of backlog once playback is live.
     */
    const size_t high_water = 384u;  /* 8 ms at 48 kHz */
    const size_t target     = 144u;  /* 3 ms at 48 kHz */

    critical_section_enter_blocking(&g_audio_cs);

    if (g_audio_count > high_water) {
        size_t discard = g_audio_count - target;
        g_audio_rd = (g_audio_rd + discard) % AUDIO_RING_SAMPLES;
        g_audio_count -= discard;
    }

    critical_section_exit(&g_audio_cs);
}

static void audio_ring_reset(void)
{
    critical_section_enter_blocking(&g_audio_cs);
    g_audio_wr = 0u;
    g_audio_rd = 0u;
    g_audio_count = 0u;
    critical_section_exit(&g_audio_cs);
}

static void stft_init(void)
{
    /*
     * Periodic Hann. At 50% overlap:
     *     w[n] + w[n + N/2] = 1
     * which gives a clean overlap/add reconstruction for this STFT.
     */
    for (size_t n = 0; n < FFT_N; ++n) {
        g_window[n] =
            0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)n / (float)FFT_N);
    }

    memset(g_ola_tail, 0, sizeof(g_ola_tail));
    memset(g_ola_tail_q, 0, sizeof(g_ola_tail_q));
}

static inline float gentle_ssb_bin_weight(float audio_hz)
{
    /* v0.7.26 diagnostic: keep the useful nominal SSB passband, but
     * replace the near-brick-wall FFT edges with a raised-cosine skirt.
     *
     * The nominal low/high settings remain the -6 dB (amplitude 0.5)
     * edge frequencies.  A 500 Hz transition is centered on each edge:
     *     low edge : low-250 .. low+250 Hz
     *     high edge: high-250 .. high+250 Hz
     *
     * This deliberately trades only the transition region for a much
     * shorter/softer impulse response.  It does NOT turn the RX filter
     * into a 3.2 kHz flat passband. */
    const float transition_hz = 500.0f;
    const float half_t = 0.5f * transition_hz;
    const float lo0 = g_rx_audio_low_hz - half_t;
    const float lo1 = g_rx_audio_low_hz + half_t;
    const float hi0 = g_rx_audio_high_hz - half_t;
    const float hi1 = g_rx_audio_high_hz + half_t;

    if (audio_hz <= lo0 || audio_hz >= hi1)
        return 0.0f;

    if (audio_hz < lo1) {
        float x = (audio_hz - lo0) / transition_hz;
        if (x < 0.0f) x = 0.0f;
        if (x > 1.0f) x = 1.0f;
        return 0.5f - 0.5f * cosf((float)M_PI * x);
    }

    if (audio_hz <= hi0)
        return 1.0f;

    {
        float x = (audio_hz - hi0) / transition_hz;
        if (x < 0.0f) x = 0.0f;
        if (x > 1.0f) x = 1.0f;
        return 0.5f + 0.5f * cosf((float)M_PI * x);
    }
}

static void filter_sideband_bins(rb_iq_f32_t *bins)
{
    const float hz_per_bin = (float)SAMPLE_RATE / (float)FFT_N;
    const rx_mode_t mode = g_mode;

    for (int k = 0; k < (int)FFT_N; ++k) {
        int audio_k = -1;

        /* Preserve the spectrum orientation proven in v0.7.8:
         * LSB occupies the positive half, USB the negative half. */
        if (mode == MODE_LSB || mode == MODE_CWL) {
            if (k > 0 && k < (int)(FFT_N / 2u))
                audio_k = k;
        } else {
            if (k > (int)(FFT_N / 2u))
                audio_k = (int)FFT_N - k;
        }

        if (audio_k < 0) {
            bins[k].i = 0.0f;
            bins[k].q = 0.0f;
            continue;
        }

        const float audio_hz = (float)audio_k * hz_per_bin;
        const float w = gentle_ssb_bin_weight(audio_hz);
        if (w <= 0.0f) {
            bins[k].i = 0.0f;
            bins[k].q = 0.0f;
        } else if (w < 1.0f) {
            bins[k].i *= w;
            bins[k].q *= w;
        }
    }
}

static void filter_am_bins(rb_iq_f32_t *bins)
{
    /* Ragchew Machine AM RX: symmetric Fourier channel filter around the
     * carrier. Here the Radioberry stream is already complex baseband, so
     * the carrier is FFT bin 0 and AM BW spans +/- BW/2. */
    const float hz_per_bin = (float)SAMPLE_RATE / (float)FFT_N;
    const float half_bw = 0.5f * (float)g_am_rx_bw_hz;
    int hi = (int)ceilf(half_bw / hz_per_bin);
    if (hi > (int)(FFT_N/2u - 1u)) hi = (int)(FFT_N/2u - 1u);
    for (int k=0; k<(int)FFT_N; ++k) {
        int signed_k = (k <= (int)FFT_N/2) ? k : k-(int)FFT_N;
        float f = fabsf((float)signed_k * hz_per_bin);
        if (f > half_bw + 0.5f*hz_per_bin || abs(signed_k) > hi+1) {
            bins[k].i = 0.0f; bins[k].q = 0.0f;
        } else if (f > half_bw - 0.5f*hz_per_bin) {
            /* Fractional edge, same Velvet overlap idea used by SSB. */
            float w = (half_bw + 0.5f*hz_per_bin - f) / hz_per_bin;
            if (w < 0.0f) w=0.0f; if (w > 1.0f) w=1.0f;
            bins[k].i *= w; bins[k].q *= w;
        }
    }
}

static void process_fft_block(void)
{
    for (size_t n = 0; n < FFT_N; ++n) {
        g_fft[n].i = g_input[n].i * g_window[n];
        g_fft[n].q = g_input[n].q * g_window[n];
    }

    if (rb_fft_forward(g_fft, FFT_N) != 0)
        return;

    /* Diagnostic peak before the sideband mask. Ignore DC for this display. */
    float peak = 0.0f;
    int peak_bin = 0;
    for (size_t k = 1; k < FFT_N; ++k) {
        float p = g_fft[k].i * g_fft[k].i +
                  g_fft[k].q * g_fft[k].q;
        if (p > peak) {
            peak = p;
            peak_bin = (int)k;
        }
    }

    /* v0.7.4: spectrum is display-only, so keep it cheap enough to run
     * while transmitting.  The old path did 128 log10f() calls for every
     * 128-sample hop (~48,000 log10f calls/s).  That was a measurable part
     * of the TX starvation.
     *
     * Update the display spectrum only once every 8 FFT hops (~47 Hz) and
     * approximate 10*log10(power) from the IEEE-754 exponent + mantissa.
     * This is more than adequate for a 22-pixel-high OLED spectrum and keeps
     * the real-time RX/TX DSP path free of libm log calls. */
    static uint8_t spectrum_div = 0u;
    spectrum_div++;
    if ((spectrum_div & 7u) == 0u) {
        /* Display-only zoom. The receiver still processes the full 48 kHz.
         * 48K = 256 FFT bins, 24K = 128 bins, 12K = 64 bins. */
        int span_bins = (int)((g_spectrum_span_hz * FFT_N) / SAMPLE_RATE);
        if (span_bins < 64) span_bins = 64;
        if (span_bins > FFT_N) span_bins = FFT_N;
        for (int x = 0; x < (int)RCB_OLED_WIDTH; ++x) {
            int sx = ((int)RCB_OLED_WIDTH - 1) - x;
            int signed_bin = (sx * span_bins) / (int)RCB_OLED_WIDTH - span_bins / 2;
            int k0 = signed_bin & (FFT_N - 1);
            float p = g_fft[k0].i*g_fft[k0].i + g_fft[k0].q*g_fft[k0].q;
            /* At 48 kHz span each pixel covers two FFT bins; retain the
             * proven max-of-two behavior. Narrow spans use one bin/pixel. */
            if (span_bins == FFT_N) {
                int k1 = (k0 + 1) & (FFT_N - 1);
                float p1 = g_fft[k1].i*g_fft[k1].i + g_fft[k1].q*g_fft[k1].q;
                if (p1 > p) p = p1;
            }

            int h = 0;
            if (p > 1.0e-12f) {
                uint32_t bits;
                memcpy(&bits, &p, sizeof(bits));
                int exp2 = (int)((bits >> 23) & 0xffu) - 127;
                float m = (float)(bits & 0x7fffffu) * (1.0f / 8388608.0f);
                /* log2(1+m), 0<=m<1: inexpensive quadratic approximation. */
                float frac = m * (1.3465558f - 0.3465558f * m);
                float db = 3.01029996f * ((float)exp2 + frac) + (float)g_spectrum_offset_db;
                h = (int)((db + 72.0f) * (22.0f / 60.0f));
                if (h < 0) h = 0;
                if (h > 22) h = 22;
            }

            uint8_t old = g_spectrum[x];
            g_spectrum[x] = (uint8_t)((3u * old + (uint8_t)h) / 4u);
        }
    }

    if (g_mode == MODE_AM) filter_am_bins(g_fft);
    else filter_sideband_bins(g_fft);

    if (rb_fft_inverse(g_fft, FFT_N) != 0)
        return;

    float temp[FFT_HOP];
    float sumsq = 0.0f;

    for (size_t n = 0; n < FFT_HOP; ++n) {
        /*
         * Real part of the selected analytic sideband becomes headphone AF.
         * The previous block's second half overlaps this block's first half.
         */
        float yi = g_fft[n].i + g_ola_tail[n];
        float yq = g_fft[n].q + g_ola_tail_q[n];
        g_ola_tail[n] = g_fft[n + FFT_HOP].i;
        g_ola_tail_q[n] = g_fft[n + FFT_HOP].q;
        float y;
        if (g_mode == MODE_AM) {
            /* Proven detector: complex magnitude followed by slow DC removal. */
            float env = sqrtf(yi*yi + yq*yq);
            g_am_dc += (env - g_am_dc) * (1.0f / 4096.0f);
            y = env - g_am_dc;
        } else {
            y = yi;
        }
        temp[n] = y;
        sumsq += y * y;
    }

    float rms = sqrtf(sumsq / (float)FFT_HOP + 1.0e-12f);
    g_last_rms = rms;
    /* Relative S-meter source: actual filtered signal level before AGC.
     * Fast-ish rise, slower fall keeps the five-bar display readable. */
    if (rms > g_smeter_rms)
        g_smeter_rms = 0.70f * g_smeter_rms + 0.30f * rms;
    else
        g_smeter_rms = 0.95f * g_smeter_rms + 0.05f * rms;

    /*
     * Conservative receiver AGC.
     * Fast attack, slower release; the AF setting remains a separate user
     * level after AGC.
     */
    /*
     * First on-air test proved AGC action is correct but headphone level
     * was too conservative. Raise the post-filter AGC target substantially;
     * final sample clamp still protects against digital overrange.
     */
    const float target_rms = 0.25f;
    float wanted = target_rms / (rms + 1.0e-6f);
    if (wanted < 0.10f) wanted = 0.10f;
    if (wanted > g_agc_max_gain) wanted = g_agc_max_gain;

    if (wanted < g_agc_gain)
        g_agc_gain = 0.75f * g_agc_gain + 0.25f * wanted;
    else
        g_agc_gain = 0.98f * g_agc_gain + 0.02f * wanted;

    const float af = (float)g_af_percent / 100.0f;
    int16_t audio[FFT_HOP];

    for (size_t n = 0; n < FFT_HOP; ++n) {
        float y = temp[n] * g_agc_gain * af;

        if (y > 0.95f) y = 0.95f;
        if (y < -0.95f) y = -0.95f;

        audio[n] = (int16_t)lrintf(y * 32767.0f);
    }

    audio_ring_push_block(audio, FFT_HOP);

    g_fft_peak = sqrtf(peak);
    g_fft_peak_bin = peak_bin;
    g_fft_blocks++;
}

static void process_rx_pair(int32_t si, int32_t sq)
{
    /* RF GAIN is deliberately ahead of the FFT filter and AGC.  It changes
     * how hard the received IQ drives the receiver without changing speaker
     * volume. Keep the math floating-point so reducing gain also preserves
     * headroom for very strong signals. */
    /* Keep the full-duplex RX engine running during TX, but heavily
     * desensitize its DSP/AGC/display path so local TX leakage does not
     * pin the AGC and saturate the spectrum. The user RF GAIN setting is
     * untouched and is restored automatically on RX. */
    const float rf = g_rf_gain_scale * (g_tx_active ? TX_MONITOR_RF_SCALE : 1.0f);
    g_input[g_input_fill].i = (float)si * rf;
    g_input[g_input_fill].q = (float)sq * rf;
    g_input_fill++;
    g_rx_pairs++;

    if (g_input_fill == FFT_N) {
        process_fft_block();

        memmove(&g_input[0],
                &g_input[FFT_HOP],
                FFT_HOP * sizeof(g_input[0]));
        g_input_fill = FFT_HOP;
    }
}

static void process_rx_word(uint32_t raw)
{
    uint8_t tag = (uint8_t)((raw >> 24) & 0x0fu);
    uint32_t u24 = raw & 0x00ffffffu;

    /* Stage 2 uses RX0 only. Other receiver tags are ignored. */
    if (tag > 1u)
        return;

    if (tag != g_expected_tag) {
        g_rx_bad_tags++;
        g_have_i = false;
        g_expected_tag = 0u;
        if (tag != 0u)
            return;
    }

    if (tag == 0u) {
        g_i24 = u24;
        g_have_i = true;
        g_expected_tag = 1u;
        return;
    }

    if (!g_have_i)
        return;

    process_rx_pair(sign24(g_i24), sign24(u24));

    g_have_i = false;
    g_expected_tag = 0u;
}

static void core1_radio(void)
{
    /* Required before core0 performs safe flash erase/program operations. */
    multicore_lockout_victim_init();

    while (!g_radio_ready)
        tight_loop_contents();

    if (!rb_radio_rx_start(&g_radio))
        panic("RX DMA start failed");

    bool last_request = false;
    bool last_tune = false;
    int16_t tx_frame[USB_HEADSET_FRAME_SAMPLES];

    while (true) {
        if (g_settings_pause_request && !g_tx_active) {
            rb_radio_rx_stop(&g_radio);
            g_settings_core1_paused = true;
            while (g_settings_pause_request)
                tight_loop_contents();
            g_settings_core1_paused = false;
            if (!rb_radio_rx_start(&g_radio))
                panic("RX DMA restart failed after settings save");
        }

        /* v0.8: P1 owns IQ while selected; local FFT/headset DSP sleeps.
         * LPF/frequency/PTT still go through the common RagchewBerry controls. */
        if (g_p1_mode) {
            const p1_network_status_t *ps = p1_network_status();
            const bool request = ps->mox_requested;
            if (request && !last_request) tx_start();
            else if (!request && last_request) tx_stop();
            rfdeck_sequence_service();

            if (request && g_tx_active) {
                int16_t iq[252];
                int nf = p1_network_read_tx_iq(iq, 126);
                if (nf > 0) {
                    for (int k=0; k<nf; ++k) {
                        uint32_t w=((uint32_t)(uint16_t)iq[2*k]<<16)|(uint16_t)iq[2*k+1];
                        (void)tx_put_word(w);
                    }
                }
            }
            last_request=request; last_tune=false;

            const uint32_t *pw; uint8_t pi;
            if (rb_radio_rx_acquire(&g_radio,&pw,&pi)) {
                int32_t ii[126],qq[126]; unsigned pairs=0;
                for (size_t n=0; n+1<STREAM_RX_DMA_WORDS_PER_BUFFER && pairs<126; n+=2) {
                    uint32_t wi=pw[n],wq=pw[n+1];
                    if (((wi>>24)&0x0f)==0u && ((wq>>24)&0x0f)==1u) {
                        ii[pairs]=sign24(wi&0x00ffffffu);
                        qq[pairs]=sign24(wq&0x00ffffffu);
                        pairs++;
                    }
                }
                if (pairs==126) (void)p1_network_queue_rx_iq(ii,qq);
                rb_radio_rx_release(&g_radio,pi);
            } else tight_loop_contents();
            continue;
        }

        /* Core1 owns the complete Radioberry real-time side in LOCAL mode. */
        const bool cw_mode = (g_mode == MODE_CWL || g_mode == MODE_CWU);
        const bool cw_key_down = cw_mode && !gpio_get(CW_KEY_GPIO);
        uint64_t cw_now_us = time_us_64();
        if (cw_key_down)
            g_cw_hang_deadline_us = cw_now_us + (uint64_t)g_cw_hang_ms * 1000ull;
        const bool cw_need_tail = cw_mode && (g_cw_envelope_pos != 0u);
        const bool cw_hang = cw_mode && !cw_key_down &&
                             (g_cw_hang_deadline_us != 0u) &&
                             ((int64_t)(g_cw_hang_deadline_us - cw_now_us) > 0);
        const bool cw_request = cw_key_down || cw_need_tail || cw_hang;
        const bool request = cw_request || g_tx_request;
        const bool tune = g_tune_active && !cw_request;

        if (request && !last_request) {
            tx_start();
            if (cw_request) g_cw_next_frame_us = time_us_64();
            else if (tune) g_tune_next_frame_us = time_us_64();
        } else if (!request && last_request) {
            tx_stop();
        }

        /* Non-blocking relay/PA sequencer. Keep servicing this every Core1
         * pass so RX DMA never waits on mechanical relay delays. */
        rfdeck_sequence_service();

        if (request && g_tx_active) {
            if (cw_request) {
                g_cw_tx_active = true;
                uint64_t now_us = time_us_64();
                unsigned catchup = 0u;
                while ((int64_t)(now_us - g_cw_next_frame_us) >= 0 && catchup < 4u) {
                    cw_feed_frame(cw_key_down);
                    g_cw_next_frame_us += 1000u;
                    catchup++;
                }
                if ((int64_t)(now_us - g_cw_next_frame_us) >= 0)
                    g_cw_next_frame_us = now_us + 1000u;
            } else if (tune) {
                if (!last_tune) g_tune_next_frame_us = time_us_64();
                uint64_t now_us = time_us_64();
                unsigned catchup = 0u;
                while ((int64_t)(now_us - g_tune_next_frame_us) >= 0 && catchup < 4u) {
                    tx_feed_mic_frame(g_tune_frame);
                    g_tune_next_frame_us += 1000u;
                    catchup++;
                }
                if ((int64_t)(now_us - g_tune_next_frame_us) >= 0)
                    g_tune_next_frame_us = now_us + 1000u;
            } else {
                /* Voice remains paced by actual UAC mic frames for this
                 * diagnostic. Only the expensive TX DSP/DMA work moved. */
                while (tx_frame_ring_pop(tx_frame))
                    tx_feed_mic_frame(tx_frame);
            }
        }

        if (!cw_request) {
            g_cw_tx_active = false;
            g_cw_monitor_env = 0.0f;
            if (!cw_key_down) g_cw_hang_deadline_us = 0u;
        } else if (!cw_key_down && g_cw_envelope_pos == 0u) {
            /* Hang interval: relay/PTT remains in TX, RF envelope is zero. */
            g_cw_tx_active = false;
            g_cw_monitor_env = 0.0f;
        }
        last_request = request;
        last_tune = tune;

        const uint32_t *words;
        uint8_t index;
        if (rb_radio_rx_acquire(&g_radio, &words, &index)) {
            for (size_t n = 0; n < STREAM_RX_DMA_WORDS_PER_BUFFER; ++n)
                process_rx_word(words[n]);
            rb_radio_rx_release(&g_radio, index);
        } else {
            tight_loop_contents();
        }
    }
}

static const char *mode_name(void)
{
    if (g_mode == MODE_AM)  return "AM";
    if (g_mode == MODE_CWL) return "CWL";
    if (g_mode == MODE_CWU) return "CWU";
    return g_mode == MODE_USB ? "USB" : "LSB";
}

static void tune_radio(uint32_t hz)
{
    uint8_t response[6] = {0};

    if (hz < 100000u) hz = 100000u;
    if (hz > 60000000u) hz = 60000000u;

    uint32_t rx_hz = hz;
    if (g_rit_on && !g_p1_mode) {
        int64_t shifted = (int64_t)hz + g_rit_hz;
        if (shifted < 100000) shifted = 100000;
        if (shifted > 60000000) shifted = 60000000;
        rx_hz = (uint32_t)shifted;
    }
    if (rb_radio_write_control(&g_radio, 0x04u, rx_hz, response)) {
        /* Keep TX NCO tracking the displayed VFO as well. */
        (void)rb_radio_write_control(&g_radio, 0x02u, hz, NULL);
        g_frequency_hz = hz;
        /* Keep local tuning persistence exactly as before, but do not touch
         * the flash-journal persistence path while Protocol 1 owns the VFO.
         * This isolates P1 retuning from settings storage without changing
         * Radioberry tuning, TX tracking, LPF/display updates, or local mode. */
        if (!g_p1_mode) settings_note_frequency(hz);
        /* Keep the display-owned RF filter outputs synchronized with the
         * actual VFO immediately, independent of which UI page is visible. */
        rcb_ui_frequency(hz);
        cdc_printf("[TUNE] %lu Hz %s  rsp=%02X %02X %02X %02X %02X %02X\r\n",
                   (unsigned long)hz, mode_name(),
                   response[0], response[1], response[2],
                   response[3], response[4], response[5]);
    } else {
        cdc_write("[TUNE] SPI command failed\r\n");
    }
}

static void print_help(void)
{
    cdc_write(
        "\r\n[CONTROL]\r\n"
        "  l             LSB\r\n"
        "  u             USB\r\n"
        "  m             AM\r\n"
        "  c             CWL\r\n"
        "  w             CWU\r\n"
        "  f 14074000    Set RX frequency in Hz\r\n"
        "  a / d         -/+ 1 kHz, immediate (no Enter)\r\n"
        "  j / k         -/+ 100 Hz, immediate (no Enter)\r\n"
        "  v 0..200      Headphone AF level\r\n"
        "  ?             Help\r\n\r\n");
}

static void handle_command(char *line)
{
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') return;

    if ((line[0] == 'l' || line[0] == 'L') && line[1] == '\0') {
        g_mode = MODE_LSB;
        cdc_printf("[MODE] LSB  %lu Hz\r\n", (unsigned long)g_frequency_hz);
        return;
    }

    if ((line[0] == 'u' || line[0] == 'U') && line[1] == '\0') {
        g_mode = MODE_USB;
        cdc_printf("[MODE] USB  %lu Hz\r\n", (unsigned long)g_frequency_hz);
        return;
    }

    if ((line[0] == 'm' || line[0] == 'M') && line[1] == '\0') {
        g_mode = MODE_AM;
        cdc_printf("[MODE] AM   %lu Hz\r\n", (unsigned long)g_frequency_hz);
        return;
    }

    if (line[0] == '+' && line[1] == '\0') {
        tune_radio(g_frequency_hz + TUNE_STEP_HZ);
        return;
    }

    if (line[0] == '-' && line[1] == '\0') {
        uint32_t hz = g_frequency_hz;
        if (hz > TUNE_STEP_HZ) hz -= TUNE_STEP_HZ;
        tune_radio(hz);
        return;
    }

    if (line[0] == 'f' || line[0] == 'F') {
        char *p = line + 1;
        while (*p == ' ' || *p == '\t') ++p;
        unsigned long hz = strtoul(p, NULL, 10);
        if (hz != 0ul) {
            tune_radio((uint32_t)hz);
        } else {
            cdc_write("[CONTROL] usage: f 14074000\r\n");
        }
        return;
    }

    if (line[0] == 'v' || line[0] == 'V') {
        char *p = line + 1;
        while (*p == ' ' || *p == '\t') ++p;
        long v = strtol(p, NULL, 10);
        if (v < 0) v = 0;
        if (v > 200) v = 200;
        g_af_percent = (uint32_t)v;
        cdc_printf("[AF] %ld%%\r\n", v);
        return;
    }

    if (line[0] == '?' || line[0] == 'h' || line[0] == 'H') {
        print_help();
        return;
    }

    cdc_write("[CONTROL] unknown command. Type ?\r\n");
}

static bool handle_immediate_key(int ch)
{
    uint32_t hz = g_frequency_hz;
    switch (ch) {
        case 'a': case 'A':
            if (hz > 1000u) hz -= 1000u;
            tune_radio(hz); return true;
        case 'd': case 'D':
            tune_radio(hz + 1000u); return true;
        case 'j': case 'J':
            if (hz > 100u) hz -= 100u;
            tune_radio(hz); return true;
        case 'k': case 'K':
            tune_radio(hz + 100u); return true;
        default: return false;
    }
}

static void service_console(void)
{
    /* No native CDC console in the Ragchew Machine network-only USB personality. */
}

static void fpga_progress_oled(size_t done, size_t total, void *user)
{
    (void)user;
    rcb_ui_gateware_progress(done, total);
    tud_task();
}


typedef enum {
    UI_MENU_BAND = 0,
    UI_MENU_MODE,
    UI_MENU_STEP,
    UI_MENU_RX_FILTER,
    UI_MENU_AM_BW,
    UI_MENU_RF_GAIN,
    UI_MENU_SPEC_OFFSET,
    UI_MENU_SPEC_SPAN,
    UI_MENU_AGC,
    UI_MENU_SPK_VOL,
    UI_MENU_MIC_GAIN,
    UI_MENU_TX_AUDIO,
    UI_MENU_TX_EQ_LOW,
    UI_MENU_TX_EQ_MID,
    UI_MENU_TX_EQ_HIGH,
    UI_MENU_TX_COMP,
    UI_MENU_TX_DRIVE,
    UI_MENU_TX_IQ_GAIN,
    UI_MENU_AM_CARRIER,
    UI_MENU_TUNE_LEVEL,
    UI_MENU_CW_WPM,
    UI_MENU_CW_PITCH,
    UI_MENU_CW_HANG,
    UI_MENU_CQ,
    UI_MENU_RADIO_MODE,
    UI_MENU_PROGRAM,
    UI_MENU_EXIT,
    UI_MENU_COUNT
} ui_menu_item_t;

typedef enum {
    UI_NORMAL = 0,
    UI_MENU_BROWSE,
    UI_MENU_EDIT,
    UI_TX_IQ_BROWSE,
    UI_TX_IQ_EDIT,
    UI_PROGRAM_CONFIRM
} ui_state_t;

typedef struct {
    const char *name;
    uint32_t hz;
} band_preset_t;

static const band_preset_t g_bands[] = {
    {"160M", 1900000u}, {"80M", 3600000u}, {"60M", 5357000u},
    {"40M", 7200000u}, {"30M", 10136000u}, {"20M", 14074000u},
    {"17M", 18100000u}, {"15M", 21074000u}, {"12M", 24915000u},
    {"10M", 28074000u}
};

#define BAND_COUNT ((unsigned)(sizeof(g_bands) / sizeof(g_bands[0])))
_Static_assert(BAND_COUNT == RCB_PERSIST_BAND_COUNT, "persistence band count mismatch");

static uint32_t g_band_last_hz[RCB_PERSIST_BAND_COUNT] = {
    1900000u, 3600000u, 5357000u, 7200000u, 10136000u, 14074000u,
    18100000u, 21074000u, 24915000u, 28074000u
};
static rx_mode_t g_band_mode[RCB_PERSIST_BAND_COUNT] = {
    MODE_LSB, MODE_LSB, MODE_USB, MODE_LSB, MODE_USB,
    MODE_USB, MODE_USB, MODE_USB, MODE_USB, MODE_USB
};
static uint32_t g_band_rf_gain_percent[RCB_PERSIST_BAND_COUNT] = {
    100u,100u,100u,100u,100u,100u,100u,100u,100u,100u
};
static int32_t g_band_spectrum_offset_db[RCB_PERSIST_BAND_COUNT] = {0};
static uint32_t g_band_tx_iq_gain_x100[RCB_PERSIST_BAND_COUNT] = {
    40u,40u,40u,40u,40u,40u,40u,40u,40u,40u
};
static unsigned g_active_band = 5u; /* 20 m default */
static rcb_persist_payload_t g_settings_observed;
static bool g_settings_observed_valid = false;


static const uint32_t g_steps[] = {1u, 10u, 100u, 1000u, 10000u};
static const uint32_t g_filter_highs[] = {1800u, 2400u, 2700u, 3000u};
static const uint32_t g_agc_maxes[] = {30u, 60u, 120u, 250u, 500u};

static unsigned nearest_band_index(uint32_t hz)
{
    unsigned best = 0u;
    uint32_t best_delta = 0xffffffffu;
    for (unsigned i = 0; i < sizeof(g_bands)/sizeof(g_bands[0]); ++i) {
        uint32_t f = g_bands[i].hz;
        uint32_t d = hz > f ? hz - f : f - hz;
        if (d < best_delta) { best_delta = d; best = i; }
    }
    return best;
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void settings_note_frequency(uint32_t hz)
{
    unsigned new_band = nearest_band_index(hz);
    if (new_band != g_active_band && new_band < RCB_PERSIST_BAND_COUNT) {
        if (g_active_band < RCB_PERSIST_BAND_COUNT) {
            g_band_mode[g_active_band] = g_mode;
            g_band_rf_gain_percent[g_active_band] = g_rf_gain_percent;
            g_band_spectrum_offset_db[g_active_band] = g_spectrum_offset_db;
            g_band_tx_iq_gain_x100[g_active_band] = g_tx_iq_gain_x100;
        }
        g_active_band = new_band;
        g_mode = g_band_mode[g_active_band];
        g_rf_gain_percent = g_band_rf_gain_percent[g_active_band];
        g_rf_gain_scale = ((float)g_rf_gain_percent * 0.01f) / 8388608.0f;
        g_spectrum_offset_db = g_band_spectrum_offset_db[g_active_band];
        g_tx_iq_gain_x100 = g_band_tx_iq_gain_x100[g_active_band];
    } else {
        g_active_band = new_band;
    }
    if (g_active_band < RCB_PERSIST_BAND_COUNT)
        g_band_last_hz[g_active_band] = hz;
}

static void settings_capture(rcb_persist_payload_t *s)
{
    memset(s, 0, sizeof(*s));
    s->current_frequency_hz = g_frequency_hz;
    for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i)
        s->band_frequency_hz[i] = g_band_last_hz[i];
    s->mode = (uint32_t)g_mode;
    s->tune_step_hz = g_tune_step_hz;
    s->rx_filter_high_hz = (uint32_t)g_rx_audio_high_hz;
    for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i) {
        s->band_mode[i] = (uint32_t)((i == g_active_band) ? g_mode : g_band_mode[i]);
        s->band_rf_gain_percent[i] = (i == g_active_band) ? g_rf_gain_percent : g_band_rf_gain_percent[i];
        s->band_spectrum_offset_db[i] = (i == g_active_band) ? g_spectrum_offset_db : g_band_spectrum_offset_db[i];
        s->band_tx_iq_gain_x100[i] = (i == g_active_band) ? g_tx_iq_gain_x100 : g_band_tx_iq_gain_x100[i];
    }
    s->spectrum_span_hz = g_spectrum_span_hz;
    s->agc_max_gain = (uint32_t)g_agc_max_gain;
    s->af_percent = g_af_percent;
    s->mic_gain_percent = g_mic_gain_percent;
    s->tx_drive = g_tx_drive;
    s->tune_level_percent = g_tune_level_percent;
    s->reserved[0] = g_am_rx_bw_hz;
    s->reserved[1] = g_am_carrier_percent + 1u; /* 0 means pre-AM v3 record */
    /* Port8 uses the two remaining reserved words without changing the flash
     * record format. +1 preserves zero as the pre-Port8 marker. */
    s->reserved[2] = g_tx_audio_percent + 1u;
    s->reserved[3] = 0xA8000000u |
        ((uint32_t)(g_tx_eq_low_db + 12) & 0x1fu) |
        (((uint32_t)(g_tx_eq_mid_db + 12) & 0x1fu) << 5) |
        (((uint32_t)(g_tx_eq_high_db + 12) & 0x1fu) << 10) |
        ((g_tx_comp_level & 0x0fu) << 15);
}

static void settings_apply(const rcb_persist_payload_t *s)
{
    g_frequency_hz = clamp_u32(s->current_frequency_hz, 100000u, 60000000u);
    for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i) {
        uint32_t f = s->band_frequency_hz[i];
        g_band_last_hz[i] = (f >= 100000u && f <= 60000000u) ? f : g_bands[i].hz;
    }

    rx_mode_t saved_global_mode = MODE_USB;
    if (s->mode == (uint32_t)MODE_LSB) saved_global_mode = MODE_LSB;
    else if (s->mode == (uint32_t)MODE_AM)  saved_global_mode = MODE_AM;
    else if (s->mode == (uint32_t)MODE_CWL) saved_global_mode = MODE_CWL;
    else if (s->mode == (uint32_t)MODE_CWU) saved_global_mode = MODE_CWU;

    const uint32_t steps[] = {1u, 10u, 100u, 1000u, 10000u};
    unsigned best_step = 0u;
    uint32_t best_delta = 0xffffffffu;
    for (unsigned i = 0; i < sizeof(steps)/sizeof(steps[0]); ++i) {
        uint32_t d = s->tune_step_hz > steps[i] ? s->tune_step_hz - steps[i] : steps[i] - s->tune_step_hz;
        if (d < best_delta) { best_delta = d; best_step = i; }
    }
    g_tune_step_hz = steps[best_step];

    g_rx_audio_high_hz = (float)clamp_u32(s->rx_filter_high_hz, 1800u, 3000u);
    for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i) {
        uint32_t bm = s->band_mode[i];
        g_band_mode[i] = (bm == (uint32_t)MODE_LSB) ? MODE_LSB :
                         (bm == (uint32_t)MODE_AM)  ? MODE_AM  :
                         (bm == (uint32_t)MODE_CWL) ? MODE_CWL :
                         (bm == (uint32_t)MODE_CWU) ? MODE_CWU :
                         (bm == (uint32_t)MODE_USB) ? MODE_USB : saved_global_mode;
        g_band_rf_gain_percent[i] = clamp_u32(s->band_rf_gain_percent[i], 10u, 200u);
        int32_t off = s->band_spectrum_offset_db[i];
        if (off < -30) off = -30; if (off > 20) off = 20;
        g_band_spectrum_offset_db[i] = off;
        g_band_tx_iq_gain_x100[i] = clamp_u32(s->band_tx_iq_gain_x100[i], 10u, 150u);
    }
    g_spectrum_span_hz = (s->spectrum_span_hz == 12000u || s->spectrum_span_hz == 24000u) ? s->spectrum_span_hz : 48000u;
    g_agc_max_gain = (float)clamp_u32(s->agc_max_gain, 30u, 500u);
    g_af_percent = clamp_u32(s->af_percent, 0u, 200u);
    g_mic_gain_percent = clamp_u32(s->mic_gain_percent, 25u, 300u);
    g_tx_drive = clamp_u32(s->tx_drive, 0u, 15u);
    g_tune_level_percent = clamp_u32(s->tune_level_percent, 10u, 100u);
    g_am_rx_bw_hz = (s->reserved[0] >= 2000u && s->reserved[0] <= 8000u) ? s->reserved[0] : 6000u;
    g_am_carrier_percent = (s->reserved[1] >= 1u && s->reserved[1] <= 101u) ? (s->reserved[1]-1u) : 50u;
    g_tx_audio_percent = (s->reserved[2] >= 1u && s->reserved[2] <= 201u) ?
                         (s->reserved[2] - 1u) : 100u;
    if ((s->reserved[3] & 0xFF000000u) == 0xA8000000u) {
        uint32_t p = s->reserved[3];
        int low = (int)(p & 0x1fu) - 12;
        int mid = (int)((p >> 5) & 0x1fu) - 12;
        int high = (int)((p >> 10) & 0x1fu) - 12;
        uint32_t comp = (p >> 15) & 0x0fu;
        g_tx_eq_low_db = low < -12 ? -12 : (low > 12 ? 12 : low);
        g_tx_eq_mid_db = mid < -12 ? -12 : (mid > 12 ? 12 : mid);
        g_tx_eq_high_db = high < -12 ? -12 : (high > 12 ? 12 : high);
        g_tx_comp_level = comp > 10u ? 10u : comp;
    } else {
        g_tx_eq_low_db = g_tx_eq_mid_db = g_tx_eq_high_db = 0;
        g_tx_comp_level = 0u;
    }
    g_tx_comp_gain = 1.0f;
    g_tune_frame_level_percent = 0u;

    g_active_band = nearest_band_index(g_frequency_hz);
    g_band_last_hz[g_active_band] = g_frequency_hz;
    g_mode = g_band_mode[g_active_band];
    g_rf_gain_percent = g_band_rf_gain_percent[g_active_band];
    g_rf_gain_scale = ((float)g_rf_gain_percent * 0.01f) / 8388608.0f;
    g_spectrum_offset_db = g_band_spectrum_offset_db[g_active_band];
    g_tx_iq_gain_x100 = g_band_tx_iq_gain_x100[g_active_band];
}

static void settings_load_at_boot(void)
{
    rcb_persist_payload_t loaded;
    if (rcb_settings_store_load(&loaded, &g_settings_sequence)) {
        settings_apply(&loaded);
        cdc_printf("[SAVE] restored seq=%lu  %lu Hz %s\r\n",
                   (unsigned long)g_settings_sequence,
                   (unsigned long)g_frequency_hz, mode_name());
    } else {
        settings_note_frequency(g_frequency_hz);
        cdc_write("[SAVE] no valid record; using defaults\r\n");
    }

    settings_capture(&g_settings_observed);
    g_settings_observed_valid = true;
    g_settings_dirty = false;
}

static void settings_observe_changes(uint32_t now_ms)
{
    rcb_persist_payload_t current;
    settings_capture(&current);

    if (!g_settings_observed_valid ||
        memcmp(&current, &g_settings_observed, sizeof(current)) != 0) {
        g_settings_observed = current;
        g_settings_observed_valid = true;
        g_settings_dirty = true;
        g_settings_dirty_since_ms = now_ms;
    }
}

static void settings_save_service(uint32_t now_ms)
{
    if (!g_settings_dirty) return;
    if ((uint32_t)(now_ms - g_settings_dirty_since_ms) < SETTINGS_SAVE_DELAY_MS) return;
    if (g_tx_active || g_tx_request || g_tune_active) return;

    /* Ask core1 to stop RX DMA before the flash lockout. This prevents an RX
     * DMA overrun while both cores are briefly unable to execute from XIP. */
    g_settings_pause_request = true;
    const uint64_t deadline = time_us_64() + 20000u;
    while (!g_settings_core1_paused && time_us_64() < deadline) {
        tud_task();
        usb_headset_task();
        tight_loop_contents();
    }

    if (!g_settings_core1_paused) {
        g_settings_pause_request = false;
        g_settings_dirty_since_ms = now_ms; /* retry later */
        cdc_write("[SAVE] core1 pause timeout; deferred\r\n");
        return;
    }

    rcb_persist_payload_t current;
    settings_capture(&current);
    const bool ok = rcb_settings_store_save(&current, &g_settings_sequence);

    g_settings_pause_request = false;
    while (g_settings_core1_paused)
        tight_loop_contents();

    /* Discard any stale RX samples accumulated around the short flash pause. */
    audio_ring_reset();

    if (ok) {
        g_settings_observed = current;
        g_settings_dirty = false;
        cdc_printf("[SAVE] committed seq=%lu\r\n", (unsigned long)g_settings_sequence);
    } else {
        g_settings_dirty_since_ms = now_ms; /* retry after another quiet delay */
        cdc_write("[SAVE] verify failed; will retry\r\n");
    }
}

static unsigned find_u32_index(const uint32_t *a, unsigned n, uint32_t v)
{
    unsigned best = 0u;
    uint32_t best_delta = 0xffffffffu;
    for (unsigned i = 0; i < n; ++i) {
        uint32_t d = v > a[i] ? v - a[i] : a[i] - v;
        if (d < best_delta) { best_delta = d; best = i; }
    }
    return best;
}

static int wrap_index(int v, int n)
{
    while (v < 0) v += n;
    while (v >= n) v -= n;
    return v;
}

static void menu_adjust(ui_menu_item_t item, int delta)
{
    if (delta == 0) return;
    if (item == UI_MENU_BAND) {
        int i = (int)g_active_band;
        i = wrap_index(i + (delta > 0 ? 1 : -1), (int)BAND_COUNT);
        uint32_t target = g_band_last_hz[i];
        if (target < 100000u || target > 60000000u) target = g_bands[i].hz;
        tune_radio(target);
    } else if (item == UI_MENU_MODE) {
        if (delta > 0) {
            g_mode = (g_mode == MODE_LSB) ? MODE_USB :
                     (g_mode == MODE_USB) ? MODE_AM  :
                     (g_mode == MODE_AM)  ? MODE_CWL :
                     (g_mode == MODE_CWL) ? MODE_CWU : MODE_LSB;
        } else {
            g_mode = (g_mode == MODE_LSB) ? MODE_CWU :
                     (g_mode == MODE_CWU) ? MODE_CWL :
                     (g_mode == MODE_CWL) ? MODE_AM  :
                     (g_mode == MODE_AM)  ? MODE_USB : MODE_LSB;
        }
    } else if (item == UI_MENU_STEP) {
        int i = (int)find_u32_index(g_steps, sizeof(g_steps)/sizeof(g_steps[0]), g_tune_step_hz);
        i = wrap_index(i + (delta > 0 ? 1 : -1), (int)(sizeof(g_steps)/sizeof(g_steps[0])));
        g_tune_step_hz = g_steps[i];
    } else if (item == UI_MENU_RX_FILTER) {
        uint32_t now = (uint32_t)g_rx_audio_high_hz;
        int i = (int)find_u32_index(g_filter_highs, sizeof(g_filter_highs)/sizeof(g_filter_highs[0]), now);
        i = wrap_index(i + (delta > 0 ? 1 : -1), (int)(sizeof(g_filter_highs)/sizeof(g_filter_highs[0])));
        g_rx_audio_high_hz = (float)g_filter_highs[i];
    } else if (item == UI_MENU_AM_BW) {
        int v = (int)g_am_rx_bw_hz + (delta > 0 ? 500 : -500);
        if (v < 2000) v = 2000; if (v > 8000) v = 8000;
        g_am_rx_bw_hz = (uint32_t)v;
    } else if (item == UI_MENU_RF_GAIN) {
        int v = (int)g_rf_gain_percent + (delta > 0 ? 10 : -10);
        if (v < 10) v = 10; if (v > 200) v = 200;
        g_rf_gain_percent = (uint32_t)v;
        g_rf_gain_scale = ((float)g_rf_gain_percent * 0.01f) / 8388608.0f;
        if (g_active_band < RCB_PERSIST_BAND_COUNT) g_band_rf_gain_percent[g_active_band] = g_rf_gain_percent;
    } else if (item == UI_MENU_SPEC_OFFSET) {
        int v = (int)g_spectrum_offset_db + (delta > 0 ? 2 : -2);
        if (v < -30) v = -30; if (v > 20) v = 20;
        g_spectrum_offset_db = v;
        if (g_active_band < RCB_PERSIST_BAND_COUNT) g_band_spectrum_offset_db[g_active_band] = v;
    } else if (item == UI_MENU_SPEC_SPAN) {
        static const uint32_t spans[] = {12000u, 24000u, 48000u};
        int i = (int)find_u32_index(spans, 3u, g_spectrum_span_hz);
        i = wrap_index(i + (delta > 0 ? 1 : -1), 3);
        g_spectrum_span_hz = spans[i];
    } else if (item == UI_MENU_AGC) {
        uint32_t now = (uint32_t)g_agc_max_gain;
        int i = (int)find_u32_index(g_agc_maxes, sizeof(g_agc_maxes)/sizeof(g_agc_maxes[0]), now);
        i = wrap_index(i + (delta > 0 ? 1 : -1), (int)(sizeof(g_agc_maxes)/sizeof(g_agc_maxes[0])));
        g_agc_max_gain = (float)g_agc_maxes[i];
    } else if (item == UI_MENU_SPK_VOL) {
        int v = (int)g_af_percent + (delta > 0 ? 10 : -10);
        if (v < 0) v = 0; if (v > 200) v = 200;
        g_af_percent = (uint32_t)v;
    } else if (item == UI_MENU_MIC_GAIN) {
        int v = (int)g_mic_gain_percent + (delta > 0 ? 5 : -5);
        if (v < 25) v = 25; if (v > 300) v = 300;
        g_mic_gain_percent = (uint32_t)v;
    } else if (item == UI_MENU_TX_AUDIO) {
        int v = (int)g_tx_audio_percent + (delta > 0 ? 5 : -5);
        if (v < 0) v = 0; if (v > 200) v = 200;
        g_tx_audio_percent = (uint32_t)v;
    } else if (item == UI_MENU_TX_EQ_LOW) {
        int v = (int)g_tx_eq_low_db + (delta > 0 ? 1 : -1);
        if (v < -12) v = -12; if (v > 12) v = 12;
        g_tx_eq_low_db = v;
    } else if (item == UI_MENU_TX_EQ_MID) {
        int v = (int)g_tx_eq_mid_db + (delta > 0 ? 1 : -1);
        if (v < -12) v = -12; if (v > 12) v = 12;
        g_tx_eq_mid_db = v;
    } else if (item == UI_MENU_TX_EQ_HIGH) {
        int v = (int)g_tx_eq_high_db + (delta > 0 ? 1 : -1);
        if (v < -12) v = -12; if (v > 12) v = 12;
        g_tx_eq_high_db = v;
    } else if (item == UI_MENU_TX_COMP) {
        int v = (int)g_tx_comp_level + (delta > 0 ? 1 : -1);
        if (v < 0) v = 0; if (v > 10) v = 10;
        g_tx_comp_level = (uint32_t)v;
        g_tx_comp_gain = 1.0f;
    } else if (item == UI_MENU_TX_DRIVE) {
        int v = (int)g_tx_drive + (delta > 0 ? 1 : -1);
        if (v < 0) v = 0; if (v > 15) v = 15;
        g_tx_drive = (uint32_t)v;
    } else if (item == UI_MENU_TX_IQ_GAIN) {
        int v = (int)g_tx_iq_gain_x100 + (delta > 0 ? 1 : -1);
        if (v < 10) v = 10; if (v > 150) v = 150;
        g_tx_iq_gain_x100 = (uint32_t)v;
        if (g_active_band < RCB_PERSIST_BAND_COUNT)
            g_band_tx_iq_gain_x100[g_active_band] = g_tx_iq_gain_x100;
    } else if (item == UI_MENU_AM_CARRIER) {
        int v = (int)g_am_carrier_percent + (delta > 0 ? 5 : -5);
        if (v < 0) v = 0; if (v > 100) v = 100;
        g_am_carrier_percent = (uint32_t)v;
    } else if (item == UI_MENU_TUNE_LEVEL) {
        int v = (int)g_tune_level_percent + (delta > 0 ? 10 : -10);
        if (v < 10) v = 10; if (v > 100) v = 100;
        g_tune_level_percent = (uint32_t)v;
        g_tune_frame_level_percent = 0u;
    } else if (item == UI_MENU_CW_WPM) {
        int v = (int)g_cw_wpm + (delta > 0 ? 1 : -1);
        if (v < 5) v = 5; if (v > 50) v = 50;
        g_cw_wpm = (uint32_t)v;
        rcb_ui_cw_control((uint8_t)g_cw_wpm, false, (uint16_t)g_cw_pitch_hz);
    } else if (item == UI_MENU_CW_PITCH) {
        int v = (int)g_cw_pitch_hz + (delta > 0 ? 50 : -50);
        if (v < 300) v = 300; if (v > 1000) v = 1000;
        g_cw_pitch_hz = (uint32_t)v;
        rcb_ui_cw_control((uint8_t)g_cw_wpm, false, (uint16_t)g_cw_pitch_hz);
    } else if (item == UI_MENU_CW_HANG) {
        int v = (int)g_cw_hang_ms + (delta > 0 ? 50 : -50);
        if (v < 100) v = 100; if (v > 1000) v = 1000;
        g_cw_hang_ms = (uint32_t)v;
    } else if (item == UI_MENU_RADIO_MODE) {
        bool want = !g_p1_mode;
        if (want) {
            g_tx_request = false; g_tune_active = false;
            audio_ring_reset(); tx_frame_ring_reset();
            if (p1_network_start()) {
                g_rit_on = false; g_rit_hz = 0;
                g_p1_mode = true;
                g_p1_last_rx_hz = 0u;
                g_p1_last_tx_hz = 0u;
                cdc_write("[P1] USB Protocol 1 selected; DHCP server active\r\n");
            } else {
                cdc_write("[P1] USB network start FAILED\r\n");
            }
        } else {
            g_p1_mode = false;
            g_rit_on = false; g_rit_hz = 0;
            g_tx_request = false; g_tune_active = false;
            p1_network_stop();
            audio_ring_reset(); tx_frame_ring_reset();
            cdc_write("[P1] LOCAL selected\r\n");
        }
    }
}

static void menu_make_lines(char storage[UI_MENU_COUNT][22], const char *lines[UI_MENU_COUNT])
{
    unsigned b = g_active_band < BAND_COUNT ? g_active_band : nearest_band_index(g_frequency_hz);
    snprintf(storage[UI_MENU_BAND], 22, "BAND %s", g_bands[b].name);
    snprintf(storage[UI_MENU_MODE], 22, "MODE %s", mode_name());
    if (g_tune_step_hz >= 1000u)
        snprintf(storage[UI_MENU_STEP], 22, "STEP %luK", (unsigned long)(g_tune_step_hz/1000u));
    else
        snprintf(storage[UI_MENU_STEP], 22, "STEP %lu", (unsigned long)g_tune_step_hz);
    snprintf(storage[UI_MENU_RX_FILTER], 22, "RX FILT %lu", (unsigned long)g_rx_audio_high_hz);
    snprintf(storage[UI_MENU_AM_BW], 22, "AM BW %lu", (unsigned long)g_am_rx_bw_hz);
    snprintf(storage[UI_MENU_RF_GAIN], 22, "RF GAIN %lu", (unsigned long)g_rf_gain_percent);
    snprintf(storage[UI_MENU_SPEC_OFFSET], 22, "SPEC OFF %+ld", (long)g_spectrum_offset_db);
    snprintf(storage[UI_MENU_SPEC_SPAN], 22, "SPEC SPAN %luK", (unsigned long)(g_spectrum_span_hz/1000u));
    snprintf(storage[UI_MENU_AGC], 22, "AGC MAX %lu", (unsigned long)g_agc_max_gain);
    snprintf(storage[UI_MENU_SPK_VOL], 22, "SPK VOL %lu", (unsigned long)g_af_percent);
    snprintf(storage[UI_MENU_MIC_GAIN], 22, "MIC GAIN %lu", (unsigned long)g_mic_gain_percent);
    snprintf(storage[UI_MENU_TX_AUDIO], 22, "TX AUDIO %lu", (unsigned long)g_tx_audio_percent);
    snprintf(storage[UI_MENU_TX_EQ_LOW], 22, "TX EQ LOW %+ld", (long)g_tx_eq_low_db);
    snprintf(storage[UI_MENU_TX_EQ_MID], 22, "TX EQ MID %+ld", (long)g_tx_eq_mid_db);
    snprintf(storage[UI_MENU_TX_EQ_HIGH], 22, "TX EQ HIGH %+ld", (long)g_tx_eq_high_db);
    snprintf(storage[UI_MENU_TX_COMP], 22, "TX COMP %lu", (unsigned long)g_tx_comp_level);
    snprintf(storage[UI_MENU_TX_DRIVE], 22, "TX DRIVE %lu", (unsigned long)g_tx_drive);
    snprintf(storage[UI_MENU_TX_IQ_GAIN], 22, "TX IQ GAINS >");
    snprintf(storage[UI_MENU_AM_CARRIER], 22, "AM CARRIER %lu", (unsigned long)g_am_carrier_percent);
    snprintf(storage[UI_MENU_TUNE_LEVEL], 22, "TUNE LVL %lu", (unsigned long)g_tune_level_percent);
    snprintf(storage[UI_MENU_CW_WPM], 22, "CW WPM %lu", (unsigned long)g_cw_wpm);
    snprintf(storage[UI_MENU_CW_PITCH], 22, "CW PITCH %lu", (unsigned long)g_cw_pitch_hz);
    snprintf(storage[UI_MENU_CW_HANG], 22, "CW HANG %lums", (unsigned long)g_cw_hang_ms);
    snprintf(storage[UI_MENU_CQ], 22, "CQ TEST >");
    snprintf(storage[UI_MENU_RADIO_MODE], 22, "RADIO %s", g_p1_mode ? "P1 USB" : "LOCAL");
    snprintf(storage[UI_MENU_PROGRAM], 22, "PROGRAM");
    snprintf(storage[UI_MENU_EXIT], 22, "EXIT");
    for (unsigned i=0; i<UI_MENU_COUNT; ++i) lines[i]=storage[i];
}

#define TX_IQ_PAGE_COUNT (RCB_PERSIST_BAND_COUNT + 1u)
static void tx_iq_make_lines(char storage[TX_IQ_PAGE_COUNT][22],
                             const char *lines[TX_IQ_PAGE_COUNT])
{
    for (unsigned i = 0; i < RCB_PERSIST_BAND_COUNT; ++i) {
        uint32_t v = (i == g_active_band) ? g_tx_iq_gain_x100 : g_band_tx_iq_gain_x100[i];
        snprintf(storage[i], 22, "%s  %lu.%02lu", g_bands[i].name,
                 (unsigned long)(v / 100u), (unsigned long)(v % 100u));
        lines[i] = storage[i];
    }
    snprintf(storage[RCB_PERSIST_BAND_COUNT], 22, "BACK");
    lines[RCB_PERSIST_BAND_COUNT] = storage[RCB_PERSIST_BAND_COUNT];
}

static void tx_iq_adjust_band(unsigned band, int delta)
{
    if (band >= RCB_PERSIST_BAND_COUNT || delta == 0) return;
    int v = (int)g_band_tx_iq_gain_x100[band] + (delta > 0 ? 1 : -1);
    if (band == g_active_band)
        v = (int)g_tx_iq_gain_x100 + (delta > 0 ? 1 : -1);
    if (v < 10) v = 10;
    if (v > 150) v = 150;
    g_band_tx_iq_gain_x100[band] = (uint32_t)v;
    if (band == g_active_band) g_tx_iq_gain_x100 = (uint32_t)v;
}

int main(void)
{
    /* GPA-10 PTT is active HIGH. Latch LOW before enabling the output so
     * power-up is unconditionally RX. */
    gpio_init(GPA10_PTT_GPIO);
    gpio_put(GPA10_PTT_GPIO, 0);
    gpio_set_dir(GPA10_PTT_GPIO, GPIO_OUT);

    gpio_init(CW_KEY_GPIO);
    gpio_set_dir(CW_KEY_GPIO, GPIO_IN);
    gpio_pull_up(CW_KEY_GPIO);
    cw_envelope_init();

    sleep_ms(20);

    tud_init(0);
    bool oled_ok = rcb_ui_init();
    if (oled_ok) {
        rcb_ui_splash();
        for (unsigned i = 0; i < 90u; ++i) { tud_task(); sleep_ms(10); }
    }

    cdc_write("\r\n*******************************************\r\n");
    cdc_write("WP3DN RagchewBerry v0.8 Port12 - CW selector fix + GP0 Ragchew shaping\r\n");
    cdc_write("RX + USB headset mic -> our FFT/IFFT -> Radioberry TX\r\n");
    cdc_write("48 kHz local transceiver. GP0 LOW = shaped CW straight key.\r\n");
    cdc_write("RF deck: GPA-10 GP16 PTT active-high; no external sequencer.\r\n");
    cdc_write("*******************************************\r\n");

    critical_section_init(&g_audio_cs);
    critical_section_init(&g_tx_frame_cs);
    stft_init();
    settings_load_at_boot();

    fpga_loader_t loader;
    const fpga_loader_io_t io = fpga_loader_pico_io();

    if (!fpga_loader_init(&loader, &io))
        panic("FPGA loader init failed");

    fpga_loader_result_t result =
        fpga_loader_program_progress(&loader,
                                     radioberry_rbf_data,
                                     radioberry_rbf_size,
                                     oled_ok ? fpga_progress_oled : NULL,
                                     NULL);

    if (result != FPGA_LOADER_OK) {
        if (oled_ok) rcb_ui_gateware_result(false);
        panic("FPGA configuration failed");
    }
    if (oled_ok) rcb_ui_gateware_result(true);

    cdc_printf("[FPGA] configured: %s (%u bytes)\r\n",
               radioberry_rbf_name,
               (unsigned)radioberry_rbf_size);

    sleep_ms(500);

    if (!rb_radio_init(&g_radio, pio0, 0u, 1u))
        panic("Radioberry init failed");

    if (!rb_radio_detect(&g_radio, 50u, 3u, 100u))
        panic("Radioberry not detected");

    cdc_printf("[RADIO] gateware %u.%u FPGA=%u RX=%u TX=%u\r\n",
               g_radio.info.major, g_radio.info.minor,
               g_radio.info.fpga,
               g_radio.info.receiver_count,
               g_radio.info.transmitter_count);

    if (oled_ok) {
        rcb_ui_radio_info(g_radio.info.major, g_radio.info.minor,
                          g_radio.info.fpga,
                          g_radio.info.receiver_count,
                          g_radio.info.transmitter_count);
        for (unsigned i = 0; i < 80u; ++i) { tud_task(); sleep_ms(10); }
    }

    /*
     * Radioberry SPI carries Protocol-1 C0 in byte 1:
     * C0=0x00: ADDR0, MOX off; C0=0x02: TX1 frequency;
     * C0=0x04: RX1 frequency.
     */
    /* Enable independent TX1/RX1 NCOs before initial local tuning. */
    (void)rb_radio_write_control(&g_radio, 0x00u, 0x00000004u, NULL);
    sleep_ms(2);
    tune_radio(g_frequency_hz);
    set_tx_frequency(g_frequency_hz);

    if (!usb_headset_init())
        panic("USB headset host init failed");

    /* Port18: claim UART TX DMA only now, after the proven Radioberry and
     * PIO-USB DMA users have initialized. Failure is non-fatal: the UI
     * remains on the original blocking UART path. */
    bool ui_dma_ok = rcb_ui_enable_tx_dma();
    cdc_printf("[UI] UART1 TX DMA %s\r\n", ui_dma_ok ? "enabled" : "unavailable - blocking fallback");

    cdc_write("[USB] PIO2 headset host ready on GP28/GP29\r\n");
    cdc_printf("[CLOCK] sys=%lu peri=%lu usb=%lu\r\n",
               (unsigned long)clock_get_hz(clk_sys),
               (unsigned long)clock_get_hz(clk_peri),
               (unsigned long)clock_get_hz(clk_usb));

    cdc_printf("[RX] %lu Hz %s  passband %.0f-%.0f Hz  FFT=%u hop=%u\r\n",
               (unsigned long)g_frequency_hz,
               mode_name(),
               (double)g_rx_audio_low_hz,
               (double)g_rx_audio_high_hz,
               FFT_N, FFT_HOP);

    print_help();

    g_radio_ready = true;
    multicore_launch_core1(core1_radio);

    int16_t mic[USB_HEADSET_FRAME_SAMPLES];
    int16_t playback[USB_HEADSET_FRAME_SAMPLES];
    uint8_t last_button = 0xffu;
    bool volume_reported = false;
    bool headset_was_connected = false;
    uint32_t last_ui_ms = 0u;
    ui_state_t ui_state = UI_NORMAL;
    ui_menu_item_t menu_item = UI_MENU_BAND;
    unsigned tx_iq_item = 0u;

    while (true) {
        tud_task();
        if (g_p1_mode) {
            p1_network_task();
            const p1_network_status_t *ps = p1_network_status();
            if (ps->rx_frequency_hz && ps->rx_frequency_hz != g_p1_last_rx_hz) {
                g_p1_last_rx_hz = ps->rx_frequency_hz;
                tune_radio(ps->rx_frequency_hz);
            }
            if (ps->tx_frequency_hz && ps->tx_frequency_hz != g_p1_last_tx_hz) {
                g_p1_last_tx_hz = ps->tx_frequency_hz;
                set_tx_frequency(ps->tx_frequency_hz);
                if (ps->mox_requested) rcb_ui_frequency(ps->tx_frequency_hz);
            }
        }
        service_console();
        usb_headset_task();
        /* v0.7.4: OLED wire transfer remains DMA-driven and non-blocking.
         * Service only its tiny DMA state machine during TX; framebuffer
         * generation is throttled separately below.  Keep front-panel/menu
         * polling paused while keyed so the 1 ms USB mic cadence has priority. */
        if (oled_ok) {
            rcb_ui_task();
            usb_headset_task();
        }

        rcb_controls_t ctl = {0};
        if (!g_tx_active) rcb_controls_poll(&ctl);

        if (!g_tx_active && ui_state == UI_NORMAL) {
            if (!g_p1_mode && ctl.encoder_delta != 0) {
                if (g_rit_on) {
                    int64_t off = (int64_t)g_rit_hz +
                                  (int64_t)ctl.encoder_delta * (int64_t)g_tune_step_hz;
                    if (off < -9999) off = -9999;
                    if (off > 9999) off = 9999;
                    g_rit_hz = (int16_t)off;
                    int64_t rx = (int64_t)g_frequency_hz + off;
                    if (rx < 100000) rx = 100000;
                    if (rx > 60000000) rx = 60000000;
                    set_rx_frequency((uint32_t)rx);
                } else {
                    int64_t next = (int64_t)g_frequency_hz +
                                   (int64_t)ctl.encoder_delta * (int64_t)g_tune_step_hz;
                    if (next < 100000) next = 100000;
                    if (next > 60000000) next = 60000000;
                    tune_radio((uint32_t)next);
                }
            }
            if (!g_p1_mode && ctl.back_long) {
                g_rit_on = !g_rit_on;
                g_rit_hz = 0;
                set_rx_frequency(g_frequency_hz);
            }
            if (ctl.encoder_push) {
                ui_state = UI_MENU_BROWSE;
                menu_item = UI_MENU_BAND;
            }
            if (ctl.back) {
                unsigned i = find_u32_index(g_steps, sizeof(g_steps)/sizeof(g_steps[0]), g_tune_step_hz);
                i = (i + 1u) % (sizeof(g_steps)/sizeof(g_steps[0]));
                g_tune_step_hz = g_steps[i];
                cdc_printf("[UI] STEP %lu Hz\r\n", (unsigned long)g_tune_step_hz);
            }
            if (ctl.confirm) {
                g_mode = (g_mode == MODE_LSB) ? MODE_USB :
                         (g_mode == MODE_USB) ? MODE_AM :
                         (g_mode == MODE_AM)  ? MODE_CWL :
                         (g_mode == MODE_CWL) ? MODE_CWU : MODE_LSB;
                cdc_printf("[UI] CONFIRM -> %s\r\n", mode_name());
            }
        } else if (!g_tx_active && ui_state == UI_MENU_BROWSE) {
            if (ctl.encoder_delta != 0) {
                int m = wrap_index((int)menu_item + (ctl.encoder_delta > 0 ? 1 : -1), UI_MENU_COUNT);
                menu_item = (ui_menu_item_t)m;
            }
            if (ctl.back) ui_state = UI_NORMAL;
            if (ctl.encoder_push || ctl.confirm) {
                if (menu_item == UI_MENU_EXIT) {
                    ui_state = UI_NORMAL;
                } else if (menu_item == UI_MENU_PROGRAM) {
                    ui_state = UI_PROGRAM_CONFIRM;
                } else if (menu_item == UI_MENU_TX_IQ_GAIN) {
                    tx_iq_item = g_active_band;
                    ui_state = UI_TX_IQ_BROWSE;
                } else if (menu_item == UI_MENU_CQ) {
                    if (g_mode == MODE_CWL || g_mode == MODE_CWU) {
                        rcb_ui_cw_control((uint8_t)g_cw_wpm, true, (uint16_t)g_cw_pitch_hz);
                    }
                    ui_state = UI_MENU_BROWSE;
                } else {
                    ui_state = UI_MENU_EDIT;
                }
            }
        } else if (!g_tx_active && ui_state == UI_MENU_EDIT) {
            if (ctl.encoder_delta != 0) menu_adjust(menu_item, ctl.encoder_delta);
            if (ctl.back || ctl.encoder_push || ctl.confirm) ui_state = UI_MENU_BROWSE;
        } else if (!g_tx_active && ui_state == UI_TX_IQ_BROWSE) {
            if (ctl.encoder_delta != 0)
                tx_iq_item = (unsigned)wrap_index((int)tx_iq_item + (ctl.encoder_delta > 0 ? 1 : -1),
                                                  (int)TX_IQ_PAGE_COUNT);
            if (ctl.back) ui_state = UI_MENU_BROWSE;
            if (ctl.encoder_push || ctl.confirm) {
                if (tx_iq_item == RCB_PERSIST_BAND_COUNT) ui_state = UI_MENU_BROWSE;
                else ui_state = UI_TX_IQ_EDIT;
            }
        } else if (!g_tx_active && ui_state == UI_TX_IQ_EDIT) {
            if (ctl.encoder_delta != 0) tx_iq_adjust_band(tx_iq_item, ctl.encoder_delta);
            if (ctl.back || ctl.encoder_push || ctl.confirm) ui_state = UI_TX_IQ_BROWSE;
        } else if (!g_tx_active && ui_state == UI_PROGRAM_CONFIRM) {
            if (ctl.back || ctl.encoder_push) ui_state = UI_MENU_BROWSE;
            if (ctl.confirm) {
                g_tx_request = false;
                g_tune_active = false;
                sleep_ms(20);
                set_radio_ptt(false);
                reset_usb_boot(0, 0);
            }
        }

        bool headset_now_connected = usb_headset_connected();
        if (headset_now_connected && !headset_was_connected) {
            /*
             * Throw away audio accumulated while the headset was still
             * enumerating. New samples arrive immediately from core1.
             */
            audio_ring_reset();
            cdc_write("[AF] headset connected: stale RX queue cleared\r\n");
        }
        headset_was_connected = headset_now_connected;

        if (!volume_reported && headset_now_connected) {
            usb_headset_uac_diag_t diag;
            memset(&diag, 0, sizeof(diag));
            usb_headset_get_uac_diag(&diag);

            if (diag.ready) {
                cdc_printf("[UAC] playback TerminalLink=%u mic TerminalLink=%u controls=%u\r\n",
                           diag.playback_terminal_link,
                           diag.mic_terminal_link,
                           diag.count);
                uac_diag_flush_line();

                for (uint8_t i = 0; i < diag.count; ++i) {
                    const usb_headset_uac_control_diag_t *r =
                        &diag.controls[i];
                    if (!r->valid) continue;

                    cdc_printf("[UAC] FU=%u src=%u ACif=%u ch=%u ctrl=0x%08lX mute=%s",
                               r->unit_id, r->source_id,
                               r->control_itf, r->channel,
                               (unsigned long)r->advertised_controls,
                               r->mute_advertised ? "yes" : "no");

                    if (r->mute_advertised) {
                        if (r->mute_cur_ok)
                            cdc_printf("(cur=%u)", r->mute_cur);
                        else
                            cdc_write("(GET_CUR fail)");
                    }

                    cdc_printf(" vol=%s", r->volume_advertised ? "yes" : "no");

                    if (r->volume_advertised) {
                        if (r->volume_cur_ok)
                            cdc_printf(" cur=%.2fdB",
                                (double)r->volume_cur_raw / 256.0);
                        else cdc_write(" cur=FAIL");

                        if (r->volume_min_ok)
                            cdc_printf(" min=%.2fdB",
                                (double)r->volume_min_raw / 256.0);
                        else cdc_write(" min=FAIL");

                        if (r->volume_max_ok)
                            cdc_printf(" max=%.2fdB",
                                (double)r->volume_max_raw / 256.0);
                        else cdc_write(" max=FAIL");

                        if (r->volume_res_ok)
                            cdc_printf(" res=%.2fdB",
                                (double)r->volume_res_raw / 256.0);
                        else cdc_write(" res=FAIL");
                    }

                    cdc_write("\r\n");
                    uac_diag_flush_line();
                }
                volume_reported = true;
            }
        }

        /*
         * The frozen headset package's full-duplex cadence is paced by
         * completed 1 ms mic frames. We discard the mic for RX Stage 2 and
         * use each frame as the exact playback slot.
         */
        uint8_t b = usb_headset_button_value();
        bool ptt_now = usb_headset_ptt_pressed();
        bool tune_now = usb_headset_tune_pressed();
        bool tx_request = g_p1_mode ? false : (ptt_now || tune_now);

        /* Core0 owns only USB/HID capture. Core1 owns TX state, TUNE timing,
         * FFT/IFFT and Radioberry TX DMA. */
        if (tune_now && !g_tune_active)
            refresh_tune_frame();
        g_tune_active = g_p1_mode ? false : tune_now;
        g_tx_request = tx_request;

        if (!g_p1_mode && usb_headset_get_mic_frame(mic)) {
            if (g_tx_active) {
                /* Voice PTT remains paced by real 1 ms UAC microphone frames.
                 * During TUNE the mic is consumed/discarded only to keep the
                 * frozen headset host cadence healthy; TX tone generation is
                 * handled independently above. */
                if (!g_tune_active)
                    tx_frame_ring_push(mic);

                usb_headset_task();

                if (g_cw_tx_active) cw_make_sidetone_frame(playback);
                else memset(playback, 0, sizeof(playback));
                usb_headset_submit_playback_mono(playback);
            } else {
                audio_ring_trim_latency();
                audio_ring_pop_frame(playback);
                usb_headset_submit_playback_mono(playback);
            }
        }

        if (b != last_button) {
            last_button = b;
            cdc_printf("[HID] %02X PTT=%s TUNE=%s\r\n",
                       b, ptt_now ? "ON" : "OFF", tune_now ? "ON" : "OFF");
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* Port7 diagnostic: Protocol 1 must be completely persistence-blind.
         * Do not observe settings changes and do not service flash saves while
         * P1 owns the radio. Local mode keeps the proven persistence behavior. */
        if (!g_p1_mode) {
            settings_observe_changes(now);
            settings_save_service(now);
        }

        /* v0.7.4 TX-friendly live display:
         *   RX: 10 Hz UI refresh, same as the proven menu build.
         *   TX: 2 Hz main-screen refresh only.  No menu/font-list rendering or
         *       controls while keyed, but the RX spectrum stays visibly alive.
         * OLED transfer itself is still hardware-I2C DMA. */
        uint32_t ui_period_ms = g_tx_active ? 500u : 100u;
        if (oled_ok && (uint32_t)(now - last_ui_ms) >= ui_period_ms) {
            if (g_p1_mode && ui_state == UI_NORMAL) {
                const p1_network_status_t *ps = p1_network_status();
                const char *state = !ps->link_up ? "WAITING FOR USB" :
                                    !ps->discovered ? "WAIT DISCOVERY" :
                                    ps->mox_requested ? "TX STREAM" :
                                    ps->rx_streaming ? "RX STREAM" : "DISCOVERED";
                rcb_ui_p1_status(state);
            } else if (g_tx_active || ui_state == UI_NORMAL) {
                uint8_t spec[RCB_OLED_WIDTH];
                for (size_t i = 0; i < RCB_OLED_WIDTH; ++i) spec[i] = g_spectrum[i];
                rcb_ui_main(g_frequency_hz, mode_name(), g_tune_step_hz,
                            g_af_percent, g_tx_active, g_smeter_rms, spec,
                            g_rit_on && !g_p1_mode, g_rit_hz);
            } else if (ui_state == UI_PROGRAM_CONFIRM) {
                rcb_ui_prompt("USB PROGRAM MODE", "CONFIRM = YES", "BACK = NO");
            } else if (ui_state == UI_TX_IQ_BROWSE || ui_state == UI_TX_IQ_EDIT) {
                char storage[TX_IQ_PAGE_COUNT][22];
                const char *lines[TX_IQ_PAGE_COUNT];
                tx_iq_make_lines(storage, lines);
                rcb_ui_menu(ui_state == UI_TX_IQ_EDIT ? "TX IQ EDIT" : "TX IQ GAINS",
                            lines, TX_IQ_PAGE_COUNT, tx_iq_item,
                            ui_state == UI_TX_IQ_EDIT);
            } else {
                char storage[UI_MENU_COUNT][22];
                const char *lines[UI_MENU_COUNT];
                menu_make_lines(storage, lines);
                rcb_ui_menu(ui_state == UI_MENU_EDIT ? "EDIT" : "RAGCHEWBERRY MENU",
                            lines, UI_MENU_COUNT, (size_t)menu_item,
                            ui_state == UI_MENU_EDIT);
            }
            last_ui_ms = now;
        }

    }
}
