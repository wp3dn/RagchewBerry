#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_HEADSET_SAMPLE_RATE       48000u
#define USB_HEADSET_FRAME_SAMPLES     48u
#define USB_HEADSET_MIC_CHANNELS      1u
#define USB_HEADSET_PLAY_CHANNELS     2u



#define USB_HEADSET_UAC_DIAG_MAX_CONTROLS 16u

typedef struct {
    bool valid;
    uint8_t control_itf;
    uint8_t unit_id;
    uint8_t source_id;
    uint8_t channel;
    uint32_t advertised_controls;

    bool volume_advertised;
    bool volume_cur_ok;
    bool volume_min_ok;
    bool volume_max_ok;
    bool volume_res_ok;

    int16_t volume_cur_raw;
    int16_t volume_min_raw;
    int16_t volume_max_raw;
    int16_t volume_res_raw;

    bool mute_advertised;
    bool mute_cur_ok;
    uint8_t mute_cur;
} usb_headset_uac_control_diag_t;

typedef struct {
    bool ready;
    uint8_t playback_terminal_link;
    uint8_t mic_terminal_link;
    uint8_t count;
    usb_headset_uac_control_diag_t
        controls[USB_HEADSET_UAC_DIAG_MAX_CONTROLS];
} usb_headset_uac_diag_t;

/*
 * Read-only UAC1 Feature Unit diagnostic snapshot.
 * v0.4 does not change volume/mute values; it only enumerates the
 * Feature Units and queries advertised controls.
 */
void usb_headset_get_uac_diag(usb_headset_uac_diag_t *out);

typedef struct {
    bool feature_found;
    bool max_applied;
    uint8_t feature_unit_id;
    int16_t max_volume_raw;   /* UAC1 signed 8.8 dB units */
} usb_headset_volume_info_t;

/*
 * Playback-side UAC1 hardware volume information.
 * The driver automatically finds the Feature Unit associated with the
 * selected USB playback terminal, clears mute when supported, queries
 * GET_MAX, and applies that maximum with SET_CUR.
 */
void usb_headset_get_playback_volume_info(
    usb_headset_volume_info_t *out);

typedef struct {
    uint32_t mic_packets;
    uint32_t mic_errors;
    uint32_t playback_packets;
    uint32_t playback_errors;
    uint32_t playback_drops;
} usb_headset_stats_t;

/*
 * Call usb_headset_init() once from the core that will own the PIO-USB host,
 * then call usb_headset_task() continuously from that SAME core.
 *
 * The rest of the API may be called from the other RP2350 core.
 */
bool usb_headset_init(void);
void usb_headset_task(void);

bool usb_headset_connected(void);

/*
 * Pop the newest complete 1 ms microphone frame.
 * out_mono must hold USB_HEADSET_FRAME_SAMPLES int16_t samples.
 * Returns true only when a new frame was available.
 */
bool usb_headset_get_mic_frame(
    int16_t out_mono[USB_HEADSET_FRAME_SAMPLES]);

/*
 * Submit one 1 ms playback frame for the next USB OUT slot.
 *
 * Stereo format is interleaved:
 *   L0,R0,L1,R1,... L47,R47
 *
 * If no new playback frame is waiting when the next USB frame is due,
 * the driver sends silence. Samples are NOT scaled by the package.
 */
bool usb_headset_submit_playback_stereo(
    const int16_t interleaved_lr[USB_HEADSET_FRAME_SAMPLES * 2u]);

/* Convenience function: duplicate mono to L and R unchanged. */
bool usb_headset_submit_playback_mono(
    const int16_t mono[USB_HEADSET_FRAME_SAMPLES]);

/*
 * Raw Logitech-style HID value from reports such as:
 *   03 01 -> value 0x01
 *   03 02 -> value 0x02
 *   03 00 -> released
 */
uint8_t usb_headset_button_value(void);

/*
 * PTT interpretation uses USB_HEADSET_PTT_VALUE from usb_headset_config.h.
 * Default is 0x01; change the config when the desired physical button is known.
 */
bool usb_headset_ptt_pressed(void);
bool usb_headset_tune_pressed(void);

void usb_headset_get_stats(usb_headset_stats_t *out);
void usb_headset_reset_stats(void);

#ifdef __cplusplus
}
#endif
