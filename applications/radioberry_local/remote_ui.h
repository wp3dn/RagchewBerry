#ifndef RAGCHEWBERRY_REMOTE_UI_H
#define RAGCHEWBERRY_REMOTE_UI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Remote front-panel link: main GP4 UART1 TX, GP5 UART1 RX, 2 Mbaud. */
#define RCB_OLED_WIDTH          128u
#define RCB_OLED_HEIGHT         64u

typedef struct {
    int8_t encoder_delta;
    bool encoder_push;
    bool back;
    bool back_long;
    bool confirm;
} rcb_controls_t;

bool rcb_ui_init(void);
/* Enable nonblocking UART1 TX DMA only after radio + PIO-USB DMA users init. */
bool rcb_ui_enable_tx_dma(void);
void rcb_ui_task(void);
void rcb_ui_splash(void);
void rcb_ui_gateware_progress(size_t done, size_t total);
void rcb_ui_gateware_result(bool ok);
void rcb_ui_radio_info(uint8_t major, uint8_t minor,
                       uint8_t fpga, uint8_t receivers, uint8_t transmitters);
void rcb_ui_frequency(uint32_t freq_hz);
void rcb_ui_main(uint32_t freq_hz, const char *mode, uint32_t step_hz,
                 uint32_t af_percent, bool tx, float signal_rms,
                 const uint8_t spectrum[RCB_OLED_WIDTH], bool rit_on, int16_t rit_hz);
void rcb_ui_message(const char *line1, const char *line2, uint32_t hold_ms);
void rcb_ui_menu(const char *title, const char *const items[], size_t count,
                 size_t selected, bool editing);
void rcb_ui_prompt(const char *title, const char *line1, const char *line2);
void rcb_controls_poll(rcb_controls_t *out);
void rcb_ui_cw_control(uint8_t wpm, bool start_cq, uint16_t pitch_hz);

#endif
