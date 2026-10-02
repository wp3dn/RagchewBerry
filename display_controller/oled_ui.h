#ifndef RAGCHEWBERRY_OLED_UI_H
#define RAGCHEWBERRY_OLED_UI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * RagchewBerry front-panel board (1.3" 128x64 OLED + EC11 + BACK/CONFIRM)
 * Waveshare RP2350-PiZero 40-pin header mapping:
 *   SDA      GP2   physical pin 3
 *   SCL      GP3   physical pin 5
 *   TRA      GP4   physical pin 7
 *   TRB      GP5   physical pin 29
 *   PSH      GP16  physical pin 36
 *   BACK     GP23  physical pin 16
 *   CONFIRM  GP7   physical pin 26
 */
#define RCB_OLED_SDA_GPIO       2u
#define RCB_OLED_SCL_GPIO       3u
#define RCB_ENC_A_GPIO          5u
#define RCB_ENC_B_GPIO          6u
#define RCB_ENC_PUSH_GPIO       4u
#define RCB_BACK_GPIO           7u
#define RCB_CONFIRM_GPIO        1u

#define RCB_OLED_WIDTH          128u
#define RCB_OLED_HEIGHT         64u

typedef struct {
    int8_t encoder_delta;
    bool encoder_push;
    bool back;
    bool back_long;
    bool confirm;
    bool confirm_long;
} rcb_controls_t;

bool rcb_ui_init(void);
void rcb_ui_task(void);
void rcb_ui_splash(void);
void rcb_ui_gateware_progress(size_t done, size_t total);
void rcb_ui_gateware_result(bool ok);
void rcb_ui_radio_info(uint8_t major, uint8_t minor,
                       uint8_t fpga, uint8_t receivers, uint8_t transmitters);
void rcb_ui_main(uint32_t freq_hz, const char *mode, uint32_t step_hz,
                 uint32_t af_percent, bool tx, float signal_rms,
                 float fwd_w, float swr,
                 const uint8_t spectrum[RCB_OLED_WIDTH], bool rit_on, int16_t rit_hz);
void rcb_ui_message(const char *line1, const char *line2, uint32_t hold_ms);
void rcb_ui_menu(const char *title, const char *const items[], size_t count,
                 size_t selected, bool editing);
void rcb_ui_p1_status(const char *state, float fwd, float rev, float swr);
void rcb_ui_prompt(const char *title, const char *line1, const char *line2);
void rcb_controls_poll(rcb_controls_t *out);
void rcb_ui_lpf_editor(const char *band, uint8_t mask, bool bit_mode, unsigned selected_bit);

#endif
