#include "oled_ui.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/regs/i2c.h"

#define OLED_I2C       i2c1
#define OLED_ADDR      0x3cu
/* This exact 1.3" encoder board is normally SH1106.  SH1106 RAM starts
 * two columns before the visible 128 pixels. */
#define OLED_COL_OFFSET 2u

static uint8_t fb[RCB_OLED_WIDTH * RCB_OLED_HEIGHT / 8u];
static uint8_t tx_fb[RCB_OLED_WIDTH * RCB_OLED_HEIGHT / 8u];
static bool ui_ok = false;

/*
 * Runtime OLED refresh uses DMA into the RP2350 hardware-I2C TX FIFO.
 * The CPU only starts a transfer and polls for completion; it never waits for
 * the I2C wire transaction.  This keeps core 0 available for the 1 ms
 * PIO-USB/UAC1 service cadence.
 *
 * DMA channel allocation is deliberate:
 *   0/1 are left for Radioberry RX/TX DMA
 *   2   is left for the PIO-USB host
 *   3   is reserved here for OLED I2C TX
 */
#define OLED_DMA_CH 3u

static bool oled_dma_ready = false;
static bool refresh_active = false;
static bool refresh_pending = false;
static uint8_t refresh_page = 0u;

typedef enum {
    OLED_DMA_IDLE = 0,
    OLED_DMA_CMD,
    OLED_DMA_DATA,
    OLED_DMA_WAIT_BUS
} oled_dma_state_t;

static oled_dma_state_t dma_state = OLED_DMA_IDLE;
static oled_dma_state_t dma_completed_stage = OLED_DMA_IDLE;

/* DesignWare I2C DATA_CMD is a 32-bit register, but the command payload we
 * need fits in 16 bits: data[7:0], RESTART bit8, STOP bit9. */
static uint16_t dma_words[1u + RCB_OLED_WIDTH];

static bool i2c_write_buf(const uint8_t *p, size_t n)
{
    return i2c_write_blocking(OLED_I2C, OLED_ADDR, p, n, false) == (int)n;
}

static bool oled_cmd(uint8_t c)
{
    uint8_t b[2] = {0x00u, c};
    return i2c_write_buf(b, sizeof(b));
}

static void oled_update_blocking(void)
{
    if (!ui_ok) return;
    uint8_t tx[17];
    tx[0] = 0x40u;

    for (uint8_t page = 0; page < 8u; ++page) {
        oled_cmd((uint8_t)(0xb0u | page));
        oled_cmd((uint8_t)(0x00u | (OLED_COL_OFFSET & 0x0fu)));
        oled_cmd((uint8_t)(0x10u | ((OLED_COL_OFFSET >> 4) & 0x0fu)));

        const uint8_t *src = &fb[(size_t)page * RCB_OLED_WIDTH];
        for (uint8_t x = 0; x < RCB_OLED_WIDTH; x += 16u) {
            memcpy(&tx[1], &src[x], 16u);
            if (!i2c_write_buf(tx, sizeof(tx))) return;
        }
    }
}

static void oled_request_update(void)
{
    if (!ui_ok) return;
    refresh_pending = true;
}

static inline bool oled_i2c_bus_idle(void)
{
    i2c_hw_t *hw = i2c_get_hw(OLED_I2C);
    return (hw->status & I2C_IC_STATUS_ACTIVITY_BITS) == 0u;
}

static void oled_dma_start_words(size_t count, oled_dma_state_t stage)
{
    dma_channel_config cfg = dma_channel_get_default_config(OLED_DMA_CH);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, DREQ_I2C1_TX);

    dma_channel_configure(OLED_DMA_CH,
                          &cfg,
                          &i2c_get_hw(OLED_I2C)->data_cmd,
                          dma_words,
                          count,
                          true);
    dma_state = stage;
}

static void oled_dma_start_command(uint8_t page)
{
    dma_words[0] = 0x00u; /* SH1106 control byte: following bytes are commands */
    dma_words[1] = (uint16_t)(0xb0u | page);
    dma_words[2] = (uint16_t)(0x00u | (OLED_COL_OFFSET & 0x0fu));
    dma_words[3] = (uint16_t)(0x10u | ((OLED_COL_OFFSET >> 4) & 0x0fu)) |
                   I2C_IC_DATA_CMD_STOP_BITS;
    oled_dma_start_words(4u, OLED_DMA_CMD);
}

static void oled_dma_start_page_data(uint8_t page)
{
    dma_words[0] = 0x40u; /* SH1106 control byte: following bytes are display RAM */
    const uint8_t *src = &tx_fb[(size_t)page * RCB_OLED_WIDTH];
    for (size_t i = 0; i < RCB_OLED_WIDTH; ++i) {
        dma_words[1u + i] = src[i];
    }
    dma_words[RCB_OLED_WIDTH] |= I2C_IC_DATA_CMD_STOP_BITS;
    oled_dma_start_words(1u + RCB_OLED_WIDTH, OLED_DMA_DATA);
}

void rcb_ui_task(void)
{
    if (!ui_ok || !oled_dma_ready) return;

    if (!refresh_active) {
        if (!refresh_pending) return;
        memcpy(tx_fb, fb, sizeof(tx_fb));
        refresh_pending = false;
        refresh_active = true;
        refresh_page = 0u;
        dma_state = OLED_DMA_IDLE;
    }

    /* DMA completion only means all DATA_CMD words reached the I2C FIFO.
     * The controller may still be clocking the final bytes, so wait for the
     * hardware bus to become idle before beginning the next transaction. */
    if (dma_state == OLED_DMA_CMD || dma_state == OLED_DMA_DATA) {
        if (dma_channel_is_busy(OLED_DMA_CH)) return;
        dma_completed_stage = dma_state;
        dma_state = OLED_DMA_WAIT_BUS;
        return;
    }

    if (dma_state == OLED_DMA_WAIT_BUS) {
        if (!oled_i2c_bus_idle()) return;

        if (dma_completed_stage == OLED_DMA_CMD) {
            oled_dma_start_page_data(refresh_page);
            return;
        }

        /* One complete SH1106 page is now on the glass. */
        refresh_page++;
        if (refresh_page >= 8u) {
            refresh_active = false;
            refresh_page = 0u;
            dma_state = OLED_DMA_IDLE;
            return;
        }
        dma_state = OLED_DMA_IDLE;
    }

    if (dma_state == OLED_DMA_IDLE && refresh_active) {
        oled_dma_start_command(refresh_page);
    }
}

static void clear(void)
{
    memset(fb, 0, sizeof(fb));
}

static void pixel(int x, int y, bool on)
{
    if (x < 0 || y < 0 || x >= (int)RCB_OLED_WIDTH || y >= (int)RCB_OLED_HEIGHT)
        return;
    size_t i = (size_t)x + (size_t)(y >> 3) * RCB_OLED_WIDTH;
    uint8_t m = (uint8_t)(1u << (y & 7));
    if (on) fb[i] |= m; else fb[i] &= (uint8_t)~m;
}

static void hline(int x0, int x1, int y)
{
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    for (int x = x0; x <= x1; ++x) pixel(x, y, true);
}

static void rect_fill(int x, int y, int w, int h)
{
    for (int yy = y; yy < y + h; ++yy)
        for (int xx = x; xx < x + w; ++xx)
            pixel(xx, yy, true);
}

/* 5x7 glyphs represented as seven 5-bit rows.  UI intentionally uses
 * uppercase text, numbers and a small punctuation set. */
static void glyph_rows(char c, uint8_t r[7])
{
#define G(a,b,c_,d,e,f,g) do { r[0]=a;r[1]=b;r[2]=c_;r[3]=d;r[4]=e;r[5]=f;r[6]=g; } while(0)
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    switch (c) {
        case 'A': G(0x0e,0x11,0x11,0x1f,0x11,0x11,0x11); break;
        case 'B': G(0x1e,0x11,0x11,0x1e,0x11,0x11,0x1e); break;
        case 'C': G(0x0e,0x11,0x10,0x10,0x10,0x11,0x0e); break;
        case 'D': G(0x1e,0x11,0x11,0x11,0x11,0x11,0x1e); break;
        case 'E': G(0x1f,0x10,0x10,0x1e,0x10,0x10,0x1f); break;
        case 'F': G(0x1f,0x10,0x10,0x1e,0x10,0x10,0x10); break;
        case 'G': G(0x0e,0x11,0x10,0x17,0x11,0x11,0x0f); break;
        case 'H': G(0x11,0x11,0x11,0x1f,0x11,0x11,0x11); break;
        case 'I': G(0x0e,0x04,0x04,0x04,0x04,0x04,0x0e); break;
        case 'J': G(0x07,0x02,0x02,0x02,0x12,0x12,0x0c); break;
        case 'K': G(0x11,0x12,0x14,0x18,0x14,0x12,0x11); break;
        case 'L': G(0x10,0x10,0x10,0x10,0x10,0x10,0x1f); break;
        case 'M': G(0x11,0x1b,0x15,0x15,0x11,0x11,0x11); break;
        case 'N': G(0x11,0x19,0x15,0x13,0x11,0x11,0x11); break;
        case 'O': G(0x0e,0x11,0x11,0x11,0x11,0x11,0x0e); break;
        case 'P': G(0x1e,0x11,0x11,0x1e,0x10,0x10,0x10); break;
        case 'Q': G(0x0e,0x11,0x11,0x11,0x15,0x12,0x0d); break;
        case 'R': G(0x1e,0x11,0x11,0x1e,0x14,0x12,0x11); break;
        case 'S': G(0x0f,0x10,0x10,0x0e,0x01,0x01,0x1e); break;
        case 'T': G(0x1f,0x04,0x04,0x04,0x04,0x04,0x04); break;
        case 'U': G(0x11,0x11,0x11,0x11,0x11,0x11,0x0e); break;
        case 'V': G(0x11,0x11,0x11,0x11,0x11,0x0a,0x04); break;
        case 'W': G(0x11,0x11,0x11,0x15,0x15,0x15,0x0a); break;
        case 'X': G(0x11,0x11,0x0a,0x04,0x0a,0x11,0x11); break;
        case 'Y': G(0x11,0x11,0x0a,0x04,0x04,0x04,0x04); break;
        case 'Z': G(0x1f,0x01,0x02,0x04,0x08,0x10,0x1f); break;
        case '0': G(0x0e,0x11,0x13,0x15,0x19,0x11,0x0e); break;
        case '1': G(0x04,0x0c,0x14,0x04,0x04,0x04,0x1f); break;
        case '2': G(0x0e,0x11,0x01,0x02,0x04,0x08,0x1f); break;
        case '3': G(0x1e,0x01,0x01,0x0e,0x01,0x01,0x1e); break;
        case '4': G(0x02,0x06,0x0a,0x12,0x1f,0x02,0x02); break;
        case '5': G(0x1f,0x10,0x10,0x1e,0x01,0x01,0x1e); break;
        case '6': G(0x0e,0x10,0x10,0x1e,0x11,0x11,0x0e); break;
        case '7': G(0x1f,0x01,0x02,0x04,0x08,0x08,0x08); break;
        case '8': G(0x0e,0x11,0x11,0x0e,0x11,0x11,0x0e); break;
        case '9': G(0x0e,0x11,0x11,0x0f,0x01,0x01,0x0e); break;
        case '.': G(0,0,0,0,0,0x06,0x06); break;
        case ':': G(0,0x06,0x06,0,0x06,0x06,0); break;
        case '-': G(0,0,0,0x1f,0,0,0); break;
        case '+': G(0,0x04,0x04,0x1f,0x04,0x04,0); break;
        case '/': G(0x01,0x02,0x02,0x04,0x08,0x08,0x10); break;
        case '%': G(0x19,0x1a,0x04,0x04,0x0b,0x13,0); break;
        case '?': G(0x0e,0x11,0x01,0x02,0x04,0,0x04); break;
        case '>': G(0x10,0x08,0x04,0x02,0x04,0x08,0x10); break;
        case ' ': default: G(0,0,0,0,0,0,0); break;
    }
#undef G
}

static void draw_char(int x, int y, char c, int scale)
{
    uint8_t r[7];
    glyph_rows(c, r);
    for (int yy = 0; yy < 7; ++yy) {
        for (int xx = 0; xx < 5; ++xx) {
            if (r[yy] & (1u << (4 - xx))) {
                if (scale == 1) pixel(x + xx, y + yy, true);
                else rect_fill(x + xx * scale, y + yy * scale, scale, scale);
            }
        }
    }
}

static void text(int x, int y, const char *s, int scale)
{
    if (!s) return;
    int adv = 6 * scale;
    while (*s) {
        draw_char(x, y, *s++, scale);
        x += adv;
    }
}

static int text_width(const char *s, int scale)
{
    return s ? (int)strlen(s) * 6 * scale - scale : 0;
}

static void centered(int y, const char *s, int scale)
{
    int x = ((int)RCB_OLED_WIDTH - text_width(s, scale)) / 2;
    if (x < 0) x = 0;
    text(x, y, s, scale);
}

bool rcb_ui_init(void)
{
    i2c_init(OLED_I2C, 400000u);
    gpio_set_function(RCB_OLED_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(RCB_OLED_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(RCB_OLED_SDA_GPIO);
    gpio_pull_up(RCB_OLED_SCL_GPIO);

    const uint8_t inputs[] = {
        RCB_ENC_A_GPIO, RCB_ENC_B_GPIO, RCB_ENC_PUSH_GPIO,
        RCB_BACK_GPIO, RCB_CONFIRM_GPIO
    };
    for (size_t i = 0; i < sizeof(inputs); ++i) {
        gpio_init(inputs[i]);
        gpio_set_dir(inputs[i], GPIO_IN);
        gpio_pull_up(inputs[i]);
    }

    sleep_ms(20);

    /* SH1106 128x64 I2C initialization. */
    const uint8_t init[] = {
        0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00, 0x40,
        0xad, 0x8b, 0xa1, 0xc8, 0xda, 0x12, 0x81, 0x7f,
        0xd9, 0x22, 0xdb, 0x35, 0xa4, 0xa6, 0xaf
    };
    ui_ok = true;
    for (size_t i = 0; i < sizeof(init); ++i) {
        if (!oled_cmd(init[i])) { ui_ok = false; break; }
    }
    if (ui_ok) {
        clear();
        oled_update_blocking();

        /* Reserve OLED DMA without disturbing the proven channel layout used
         * by Radioberry (0/1) and PIO USB (2). */
        if (!dma_channel_is_claimed(OLED_DMA_CH)) {
            dma_channel_claim(OLED_DMA_CH);
            oled_dma_ready = true;
        } else {
            /* Display boot screens remain usable; runtime async refresh is
             * disabled rather than stealing another channel unexpectedly. */
            oled_dma_ready = false;
        }
    }
    return ui_ok;
}

void rcb_ui_splash(void)
{
    if (!ui_ok) return;
    clear();
    centered(6, "RAGCHEWBERRY", 1);
    centered(21, "WP3DN", 2);
    centered(42, "RP2350 SDR", 1);
    centered(54, "LOCAL RADIO", 1);
    oled_update_blocking();
}

void rcb_ui_gateware_progress(size_t done, size_t total)
{
    if (!ui_ok || total == 0u) return;
    unsigned pct = (unsigned)((done * 100u) / total);
    if (pct > 100u) pct = 100u;

    clear();
    centered(2, "RAGCHEWBERRY", 1);
    centered(17, "LOADING RADIOBERRY", 1);
    centered(27, "GATEWARE", 1);

    /* 104x8 real progress bar. */
    int x0 = 12, y0 = 42, w = 104;
    hline(x0, x0 + w - 1, y0);
    hline(x0, x0 + w - 1, y0 + 8);
    for (int y = y0; y <= y0 + 8; ++y) { pixel(x0, y, true); pixel(x0+w-1, y, true); }
    int fill = (int)((pct * (unsigned)(w - 4)) / 100u);
    if (fill > 0) rect_fill(x0 + 2, y0 + 2, fill, 5);

    char b[16];
    snprintf(b, sizeof(b), "%u%%", pct);
    centered(54, b, 1);
    oled_update_blocking();
}

void rcb_ui_gateware_result(bool ok)
{
    if (!ui_ok) return;
    clear();
    centered(12, "RADIOBERRY", 2);
    centered(38, ok ? "GATEWARE OK" : "GATEWARE ERROR", 1);
    oled_update_blocking();
}

void rcb_ui_radio_info(uint8_t major, uint8_t minor,
                       uint8_t fpga, uint8_t receivers, uint8_t transmitters)
{
    if (!ui_ok) return;
    char b[32];
    clear();
    centered(3, "RADIOBERRY READY", 1);
    snprintf(b, sizeof(b), "GATEWARE %u.%u", major, minor);
    centered(18, b, 1);
    snprintf(b, sizeof(b), "FPGA %u  RX %u  TX %u", fpga, receivers, transmitters);
    centered(31, b, 1);
    centered(49, "READY TO RAGCHEW", 1);
    oled_update_blocking();
}

static void format_frequency(char out[16], uint32_t hz)
{
    uint32_t mhz = hz / 1000000u;
    uint32_t khz = (hz / 1000u) % 1000u;
    uint32_t rem = hz % 1000u;
    snprintf(out, 16, "%lu.%03lu.%03lu",
             (unsigned long)mhz, (unsigned long)khz, (unsigned long)rem);
}

void rcb_ui_main(uint32_t freq_hz, const char *mode, uint32_t step_hz,
                 uint32_t af_percent, bool tx, float signal_rms,
                 float fwd_w, float swr,
                 const uint8_t spectrum[RCB_OLED_WIDTH], bool rit_on, int16_t rit_hz)
{
    if (!ui_ok) return;
    clear();

    char f[16];
    format_frequency(f, freq_hz);
    int fw = text_width(f, 2);
    int fx = ((int)RCB_OLED_WIDTH - fw) / 2;
    text(fx < 0 ? 0 : fx, 0, f, 2);

    char left[16], right[20];
    snprintf(left, sizeof(left), "%s %s", tx ? "TX" : "RX", mode ? mode : "?");
    if (step_hz >= 1000u)
        snprintf(right, sizeof(right), "STP %luK", (unsigned long)(step_hz / 1000u));
    else
        snprintf(right, sizeof(right), "STP %lu", (unsigned long)step_hz);
    text(0, 17, left, 1);
    int rw = text_width(right, 1);
    text((int)RCB_OLED_WIDTH - rw, 17, right, 1);

    hline(0, 127, 27);

    /* Little spectrum: 128 columns, y=29..51. */
    if (spectrum) {
        for (int x = 0; x < 128; ++x) {
            int h = (int)spectrum[x];
            if (h > 22) h = 22;
            if (h < 0) h = 0;
            for (int y = 0; y < h; ++y) pixel(x, 51 - y, true);
        }
    }
    pixel(64, 29, true); pixel(64, 30, true); pixel(64, 31, true);
    hline(0, 127, 52);

    /* Bottom line: AF level plus a compact relative S-meter.  v0.7.15 uses
     * the smoothed post-filter signal RMS measured immediately BEFORE AGC, so
     * the meter follows received signal level instead of AGC gain.  It is
     * intentionally relative, not calibrated in S-units. */
    char bot[16];
    if (rit_on && !tx) snprintf(bot, sizeof(bot), "RIT%+d", (int)rit_hz);
    else snprintf(bot, sizeof(bot), "AF%lu", (unsigned long)af_percent);
    text(0, 56, bot, 1);

    /* TX-only forward power + SWR.  Keep the existing S-meter at x=78.
     * At 6 px/character, "F18 S1.2" fits exactly in the gap after AF100. */
    if (tx) {
        unsigned fw = (unsigned)(fwd_w + 0.5f);
        if (fw > 99u) fw = 99u;
        if (swr < 1.0f) swr = 1.0f;
        if (swr > 9.9f) swr = 9.9f;
        char meter[12];
        snprintf(meter, sizeof(meter), "F%u S%.1f", fw, (double)swr);
        int mx = text_width(bot, 1) + 1;
        if (mx + text_width(meter, 1) < 78) text(mx, 56, meter, 1);
    }

    unsigned bars = 0u;
    if      (signal_rms >= 0.030f) bars = 5u;
    else if (signal_rms >= 0.010f) bars = 4u;
    else if (signal_rms >= 0.003f) bars = 3u;
    else if (signal_rms >= 0.001f) bars = 2u;
    else if (signal_rms >= 0.0003f) bars = 1u;

    text(78, 56, "S", 1);
    static const uint8_t bar_h[5] = {2u, 3u, 5u, 6u, 8u};
    for (unsigned i = 0u; i < bars; ++i) {
        int x = 88 + (int)i * 8;
        int h = (int)bar_h[i];
        rect_fill(x, 64 - h, 5, h);
    }
    oled_request_update();
}


void rcb_ui_menu(const char *title, const char *const items[], size_t count,
                 size_t selected, bool editing)
{
    if (!ui_ok) return;
    clear();
    centered(1, title ? title : "MENU", 1);
    hline(0, 127, 11);

    size_t first = 0u;
    if (count > 4u) {
        if (selected >= 3u) first = selected - 2u;
        if (first + 4u > count) first = count - 4u;
    }

    for (size_t row = 0u; row < 4u && first + row < count; ++row) {
        size_t idx = first + row;
        int y = 15 + (int)row * 12;
        if (idx == selected) text(0, y, ">", 1);
        text(8, y, items[idx] ? items[idx] : "", 1);
        if (idx == selected && editing) text(120, y, "+", 1);
    }
    oled_request_update();
}

void rcb_ui_lpf_editor(const char *band, uint8_t mask, bool bit_mode, unsigned selected_bit)
{
    if(!ui_ok)return;
    clear();
    centered(0,"LPF BAND TABLE",1);
    hline(0,127,10);
    centered(14,band,2);
    text(22,34,"1 2 3 4 5 6 7",1);
    for(unsigned i=0;i<7u;i++) {
        int x=22+(int)i*12;
        char digit[2]={((mask>>(6u-i))&1u)?'1':'0',0};
        text(x,45,digit,1);
        if(bit_mode && selected_bit==i) {hline(x-1,x+5,54);}
    }
    centered(57,bit_mode?"PUSH:TOGGLE BACK:END":"TURN:BAND PUSH:EDIT",1);
    oled_request_update();
}

/* Port29: P1 connection state plus local detector telemetry.  F/R are watts,
 * S is SWR. ADCs are wired to the display, not the radio controller. */
void rcb_ui_p1_status(const char *state, float fwd, float rev, float swr)
{
    if (!ui_ok) return;
    if (fwd < 0.0f) fwd=0.0f;
    if (rev < 0.0f) rev=0.0f;
    if (swr < 1.0f) swr=1.0f;
    if (swr > 9.99f) swr=9.99f;
    unsigned f=(unsigned)(fwd+0.5f);
    if(f>99u) f=99u;
    char meter[24];
    snprintf(meter,sizeof meter,"F%u R%.1f S%.2f",f,(double)rev,(double)swr);
    clear();
    centered(4,"PROTOCOL 1 USB",1);
    hline(0,127,15);
    centered(28,state?state:"",1);
    hline(0,127,52);
    centered(55,meter,1);
    oled_request_update();
}

void rcb_ui_prompt(const char *title, const char *line1, const char *line2)
{
    if (!ui_ok) return;
    clear();
    centered(4, title ? title : "", 1);
    hline(0, 127, 15);
    centered(28, line1 ? line1 : "", 1);
    centered(44, line2 ? line2 : "", 1);
    oled_request_update();
}

void rcb_ui_message(const char *line1, const char *line2, uint32_t hold_ms)
{
    if (!ui_ok) return;
    clear();
    centered(18, line1 ? line1 : "", 1);
    centered(34, line2 ? line2 : "", 1);
    oled_update_blocking();
    if (hold_ms) sleep_ms(hold_ms);
}

static bool debounce_press(uint gpio, bool *last_raw, bool *stable, uint32_t *changed_ms)
{
    bool raw = gpio_get(gpio) == 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (raw != *last_raw) {
        *last_raw = raw;
        *changed_ms = now;
    }
    if (raw != *stable && (uint32_t)(now - *changed_ms) >= 25u) {
        bool old = *stable;
        *stable = raw;
        return (!old && raw);
    }
    return false;
}

void rcb_controls_poll(rcb_controls_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));

    /* Gray-code transition table: robust against most EC11 bounce. */
    static uint8_t enc_prev = 0xffu;
    static int8_t enc_accum = 0;
    static const int8_t qtab[16] = {
         0,-1, 1, 0,
         1, 0, 0,-1,
        -1, 0, 0, 1,
         0, 1,-1, 0
    };
    uint8_t now = (uint8_t)((gpio_get(RCB_ENC_A_GPIO) ? 2u : 0u) |
                            (gpio_get(RCB_ENC_B_GPIO) ? 1u : 0u));
    if (enc_prev == 0xffu) enc_prev = now;
    enc_accum += qtab[(enc_prev << 2) | now];
    enc_prev = now;
    if (enc_accum >= 4) { out->encoder_delta = 1; enc_accum = 0; }
    else if (enc_accum <= -4) { out->encoder_delta = -1; enc_accum = 0; }

    static bool lp_r=false, lp_s=false; static uint32_t lp_t=0;
    static bool lb_r=false, lb_s=false; static uint32_t lb_t=0;
    static bool lc_r=false, lc_s=false; static uint32_t lc_t=0;
    out->encoder_push = debounce_press(RCB_ENC_PUSH_GPIO, &lp_r, &lp_s, &lp_t);
    /* STEP/BACK: defer short event until release; long event once at 650 ms. */
    (void)debounce_press(RCB_BACK_GPIO, &lb_r, &lb_s, &lb_t);
    static uint32_t back_down_ms = 0u;
    static bool back_was_down = false, back_long_sent = false;
    uint32_t ms = to_ms_since_boot(get_absolute_time());
    if (lb_s && !back_was_down) {
        back_down_ms = ms; back_was_down = true; back_long_sent = false;
    }
    if (lb_s && !back_long_sent && (uint32_t)(ms - back_down_ms) >= 650u) {
        out->back_long = true; back_long_sent = true;
    }
    if (!lb_s && back_was_down) {
        if (!back_long_sent) out->back = true;
        back_was_down = false;
    }
    (void)debounce_press(RCB_CONFIRM_GPIO, &lc_r, &lc_s, &lc_t);
    static uint32_t confirm_down_ms=0u;
    static bool confirm_was_down=false, confirm_long_sent=false;
    if(lc_s && !confirm_was_down){confirm_down_ms=ms;confirm_was_down=true;confirm_long_sent=false;}
    if(lc_s && !confirm_long_sent && (uint32_t)(ms-confirm_down_ms)>=1200u){out->confirm_long=true;confirm_long_sent=true;}
    if(!lc_s && confirm_was_down){if(!confirm_long_sent)out->confirm=true;confirm_was_down=false;}
}
