#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include "hardware/regs/addressmap.h"
#include <math.h>
#include "oled_ui.h"
#define LINK uart1
#define TX 8u
#define RX 9u
#define BAUD 2000000u

/* Display-local directional-coupler inputs.
 * GP26 = ADC0 = forward detector, GP27 = ADC1 = reflected detector.
 * Simple meter convention requested for RagchewBerry: 0..3.3 V = 0..40 W. */
#define FWD_ADC_GPIO 26u
#define REV_ADC_GPIO 27u
#define FWD_ADC_CH   0u
#define REV_ADC_CH   1u
#define POWER_FS_W   40.0f
#define POWER_CAL    1.0f

static uint16_t meter_adc_avg(uint input) {
    adc_select_input(input);
    (void)adc_read(); /* discard first sample after mux change */
    uint32_t sum = 0u;
    for (unsigned i = 0; i < 16u; ++i) sum += adc_read();
    return (uint16_t)((sum + 8u) / 16u);
}

static void meter_read(float *fwd_w, float *swr) {
    uint16_t fraw = meter_adc_avg(FWD_ADC_CH);
    uint16_t rraw = meter_adc_avg(REV_ADC_CH);
    float pf = ((float)fraw / 4095.0f) * POWER_FS_W * POWER_CAL;
    float pr = ((float)rraw / 4095.0f) * POWER_FS_W * POWER_CAL;
    float s = 1.0f;
    if (pf >= 0.25f) {
        float ratio = pr / pf;
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio >= 0.9801f) s = 99.9f;
        else {
            float rho = sqrtf(ratio);
            s = (1.0f + rho) / (1.0f - rho);
            if (s > 99.9f) s = 99.9f;
        }
    }
    *fwd_w = pf;
    *swr = s;
}

/* Port28: programmable per-band seven-bit outputs, OUT1=bit6 ... OUT7=bit0.
 * Display-local editor; Port27 radio firmware is unchanged. */
static const uint8_t lpf_gpio[7] = {10u,11u,12u,13u,14u,15u,0u};
static const char *const lpf_names[11] = {"160m","80m","60m","40m","30m","20m","17m","15m","12m","10m","6m"};
static const uint32_t lpf_centers[10] = {1900000u,3600000u,5357000u,7200000u,10136000u,14074000u,18100000u,21074000u,24915000u,28074000u};
static uint8_t lpf_masks[11] = {0x40u,0x20u,0x10u,0x10u,0x08u,0x08u,0x04u,0x04u,0x02u,0x02u,0x01u};
/* Port28 persistence: two alternating 4 KiB sectors at the end of the
 * SDK-configured flash region. No write on encoder movement. Each record is
 * one 256-byte page, with version, sequence and checksum. The older valid
 * sector remains intact until the new one has been programmed and verified.
 * IMPORTANT: reserve the last 8 KiB; do not place any other persistent data
 * there. Verify PICO_FLASH_SIZE_BYTES matches the physical flash board. */
#define LPF_MAGIC 0x4C504632u /* LPF2 */
#define LPF_VERSION 1u
#define LPF_STORE_BYTES (2u * FLASH_SECTOR_SIZE)
#define LPF_STORE_BASE (PICO_FLASH_SIZE_BYTES - LPF_STORE_BYTES)
_Static_assert(PICO_FLASH_SIZE_BYTES >= (LPF_STORE_BYTES + 65536u), "flash too small");
typedef struct {
    uint32_t magic, version, sequence;
    uint8_t masks[11];
    uint8_t reserved;
    uint32_t checksum;
} lpf_record_t;
static uint32_t lpf_sequence=0u;
static int lpf_slot=-1;
static bool lpf_dirty=false;
static uint32_t lpf_checksum(const lpf_record_t *r) {
    const uint8_t *b=(const uint8_t *)r;
    uint32_t h=2166136261u;
    for(unsigned i=0;i<offsetof(lpf_record_t,checksum);i++) {h^=b[i];h*=16777619u;}
    return h;
}
static bool lpf_valid(const lpf_record_t *r) {
    if(r->magic!=LPF_MAGIC || r->version!=LPF_VERSION) return false;
    for(unsigned i=0;i<11;i++) if(r->masks[i]&0x80u) return false;
    return r->checksum==lpf_checksum(r);
}
static void lpf_load(void) {
    const lpf_record_t *a=(const lpf_record_t *)(XIP_BASE+LPF_STORE_BASE);
    const lpf_record_t *b=(const lpf_record_t *)(XIP_BASE+LPF_STORE_BASE+FLASH_SECTOR_SIZE);
    bool va=lpf_valid(a), vb=lpf_valid(b);
    if(!va&&!vb) return; /* factory one-hot defaults */
    const lpf_record_t *r;
    if(va&&vb) {bool choose_b=(int32_t)(b->sequence-a->sequence)>0; r=choose_b?b:a; lpf_slot=choose_b?1:0;}
    else {r=vb?b:a;lpf_slot=vb?1:0;}
    memcpy(lpf_masks,r->masks,sizeof lpf_masks);
    lpf_sequence=r->sequence;
}
typedef struct {uint32_t offset; uint8_t page[FLASH_PAGE_SIZE];} lpf_write_t;
static void lpf_flash_write(void *arg) {
    lpf_write_t *w=(lpf_write_t *)arg;
    flash_range_erase(w->offset,FLASH_SECTOR_SIZE);
    flash_range_program(w->offset,w->page,FLASH_PAGE_SIZE);
}
static bool lpf_save(void) {
    if(!lpf_dirty) return true;
    lpf_write_t w;
    memset(&w,0xff,sizeof w);
    lpf_record_t r;
    memset(&r,0,sizeof r);
    r.magic=LPF_MAGIC;r.version=LPF_VERSION;r.sequence=lpf_sequence+1u;
    memcpy(r.masks,lpf_masks,sizeof lpf_masks);
    r.checksum=lpf_checksum(&r);
    memcpy(w.page,&r,sizeof r);
    int target=lpf_slot==0?1:0;
    w.offset=LPF_STORE_BASE+(uint32_t)target*FLASH_SECTOR_SIZE;
    /* flash_safe_execute coordinates flash access and interrupts. */
    if(flash_safe_execute(lpf_flash_write,&w,1000)!=PICO_OK) return false;
    const lpf_record_t *verify=(const lpf_record_t *)(XIP_BASE+w.offset);
    if(!lpf_valid(verify)||verify->sequence!=r.sequence||memcmp(verify->masks,r.masks,11)!=0) return false;
    lpf_slot=target;lpf_sequence=r.sequence;lpf_dirty=false;
    return true;
}
static unsigned lpf_active_band=10u;
static bool lpf_editor=false, lpf_bit_mode=false;
static unsigned lpf_edit_band=0u, lpf_edit_bit=0u;
static uint32_t lpf_last_frequency=0u;
static void lpf_apply(unsigned band) {
    if(band>10u) band=10u;
    lpf_active_band=band;
    /* Break-before-make: clear all outputs before asserting new mask. */
    for(unsigned i=0;i<7u;i++) gpio_put(lpf_gpio[i],0);
    sleep_us(100);
    for(unsigned i=0;i<7u;i++) gpio_put(lpf_gpio[i],(lpf_masks[band]>>(6u-i))&1u);
}
static unsigned lpf_for_frequency(uint32_t hz) {
    if(hz<1500000u || hz>30000000u) return 10u;
    unsigned best=0u; uint32_t distance=UINT32_MAX;
    for(unsigned i=0;i<10u;i++) {
        uint32_t d=hz>lpf_centers[i]?hz-lpf_centers[i]:lpf_centers[i]-hz;
        if(d<distance){distance=d;best=i;}
    }
    return best;
}
static void lpf_update_frequency(uint32_t hz) {
    lpf_last_frequency=hz;
    unsigned band=lpf_for_frequency(hz);
    if(band!=lpf_active_band) lpf_apply(band);
}
static void lpf_editor_draw(void) {
    rcb_ui_lpf_editor(lpf_names[lpf_edit_band],lpf_masks[lpf_edit_band],lpf_bit_mode,lpf_edit_bit);
}
static void lpf_editor_control(const rcb_controls_t *c) {
    if(c->back || c->back_long) {
        if(lpf_bit_mode) lpf_bit_mode=false;
        else {
            if(lpf_dirty && !lpf_save()) {rcb_ui_message("LPF SAVE FAILED","RETRY: ENTER/EXIT",1200);return;}
            lpf_editor=false;
        }
    } else if(c->encoder_push || c->confirm) {
        if(!lpf_bit_mode) {lpf_bit_mode=true;lpf_edit_bit=0u;}
        else {lpf_masks[lpf_edit_band]^=(uint8_t)(1u<<(6u-lpf_edit_bit));lpf_dirty=true;
            if(lpf_edit_band==lpf_active_band) lpf_apply(lpf_active_band);}
    }
    if(c->encoder_delta) {
        int v=(int)(lpf_bit_mode?lpf_edit_bit:lpf_edit_band)+c->encoder_delta;
        int count=lpf_bit_mode?7:11;
        while(v<0)v+=count; while(v>=count)v-=count;
        if(lpf_bit_mode)lpf_edit_bit=(unsigned)v;else lpf_edit_band=(unsigned)v;
    }
    if(lpf_editor)lpf_editor_draw();
}

enum { M_SPLASH=1,M_GATEPROG,M_GATERES,M_RADIOINFO,M_MAIN,M_MESSAGE,M_MENU,M_PROMPT,M_FREQUENCY,M_P1STATUS=10,M_CONTROL=0x80,M_CWCONTROL=0x81 };
static uint8_t buf[800]; static unsigned bn;

// 2 Mbaud UART RX capture ring.  The ISR only drains the hardware FIFO;
// all framing/parsing and UI work remains in the main loop.
#define RX_RING_SIZE 2048u
#define RX_RING_MASK (RX_RING_SIZE - 1u)
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overflow;

static void on_uart_rx(void) {
    while (uart_is_readable(LINK)) {
        uint8_t x = (uint8_t)uart_getc(LINK);
        uint16_t head = rx_head;
        uint16_t next = (uint16_t)((head + 1u) & RX_RING_MASK);
        if (next != rx_tail) {
            rx_ring[head] = x;
            rx_head = next;
        } else {
            // Preserve already-buffered bytes. Parser will resync on the
            // next A5 5A frame marker if traffic ever outruns the ring.
            rx_overflow++;
        }
    }
}

static bool rx_pop(uint8_t *x) {
    uint16_t tail = rx_tail;
    if (tail == rx_head) return false;
    *x = rx_ring[tail];
    rx_tail = (uint16_t)((tail + 1u) & RX_RING_MASK);
    return true;
}
static uint8_t sum8(const uint8_t*p,unsigned n){uint8_t s=0;while(n--)s^=*p++;return s;}
static void sendctl(int8_t e,uint8_t b){uint8_t p[2]={(uint8_t)e,b};uint8_t h[5]={0xA5,0x5A,M_CONTROL,2,0};uint8_t c=M_CONTROL^2^sum8(p,2);uart_write_blocking(LINK,h,5);uart_write_blocking(LINK,p,2);uart_putc_raw(LINK,c);}
typedef struct __attribute__((packed)){uint32_t f,step,af;uint8_t tx;float rms;char mode[8];uint8_t spec[128];uint8_t rit_on;int16_t rit_hz;} mainp_t;
static unsigned getstr(const uint8_t*p,unsigned n,unsigned o,char*out,unsigned cap){if(o>=n){*out=0;return o;}unsigned z=p[o++];if(z>=cap)z=cap-1;if(o+z>n)z=n-o;memcpy(out,p+o,z);out[z]=0;return o+p[o-1];}

/* Display-side CW keyer. Paddle contacts ground the inputs. CW_KEY is an
 * open-drain-style output: LOW while keyed, high-impedance while released.
 * The radio GP0 internal pull-up is the authority for key-up. */
#define CW_DIT_GPIO 25u
#define CW_DAH_GPIO 24u
#define CW_KEY_GPIO 23u
static uint8_t cw_wpm = 20u;
static uint16_t cw_pitch_hz = 700u;
static bool cw_macro_active;
static const char cw_macro_text[] = "CQ CQ CQ DE TEST TEST TEST K";
static const char *cw_macro_p;
static const char *cw_pat;
static unsigned cw_pat_i;
static uint64_t cw_deadline_us;
static uint8_t cw_phase; /* 0 idle, 1 key element, 2 intra gap, 3 char/word gap */
static bool dit_mem, dah_mem, last_was_dah;

static uint32_t cw_dit_us(void){ return 1200000u / (cw_wpm ? cw_wpm : 20u); }
static void cw_key(bool down){
    if(down){ gpio_put(CW_KEY_GPIO,0); gpio_set_dir(CW_KEY_GPIO,GPIO_OUT); }
    else gpio_set_dir(CW_KEY_GPIO,GPIO_IN);
}
static const char *cw_code(char c){
    switch(c){
    case 'A':return ".-"; case 'B':return "-..."; case 'C':return "-.-."; case 'D':return "-..";
    case 'E':return "."; case 'F':return "..-."; case 'G':return "--."; case 'H':return "....";
    case 'I':return ".."; case 'J':return ".---"; case 'K':return "-.-"; case 'L':return ".-..";
    case 'M':return "--"; case 'N':return "-."; case 'O':return "---"; case 'P':return ".--.";
    case 'Q':return "--.-"; case 'R':return ".-."; case 'S':return "..."; case 'T':return "-";
    case 'U':return "..-"; case 'V':return "...-"; case 'W':return ".--"; case 'X':return "-..-";
    case 'Y':return "-.--"; case 'Z':return "--..";
    case '0':return "-----"; case '1':return ".----"; case '2':return "..---"; case '3':return "...--";
    case '4':return "....-"; case '5':return "....."; case '6':return "-...."; case '7':return "--...";
    case '8':return "---.."; case '9':return "----."; default:return 0;
    }
}
static void cw_start_element(bool dah,uint64_t now){
    cw_key(true); last_was_dah=dah; cw_phase=1;
    cw_deadline_us=now+(uint64_t)cw_dit_us()*(dah?3u:1u);
}
static void cw_start_macro(void){
    cw_macro_active=true; cw_macro_p=cw_macro_text; cw_pat=0; cw_pat_i=0; cw_phase=0;
    dit_mem=dah_mem=false; cw_key(false); cw_deadline_us=time_us_64();
}
static void cw_macro_service(uint64_t now){
    if(!cw_macro_active || now<cw_deadline_us)return;
    uint32_t dit=cw_dit_us();
    if(cw_phase==1){ cw_key(false); cw_phase=2; cw_deadline_us=now+dit; return; }
    if(cw_phase==2){
        if(cw_pat && cw_pat[cw_pat_i]){ cw_start_element(cw_pat[cw_pat_i++]=='-',now); return; }
        cw_pat=0; cw_phase=3;
        /* We already consumed one dit of silence after the final element.
           Add 2 dits for a character gap, or 6 for a word gap. */
        bool word = (*cw_macro_p==' ');
        if(word) while(*cw_macro_p==' ')cw_macro_p++;
        cw_deadline_us=now+(uint64_t)dit*(word?6u:2u); return;
    }
    if(cw_phase==3) cw_phase=0;
    while(*cw_macro_p==' ')cw_macro_p++;
    if(!*cw_macro_p){ cw_macro_active=false; cw_key(false); return; }
    cw_pat=cw_code(*cw_macro_p++); cw_pat_i=0;
    if(!cw_pat){ cw_deadline_us=now+7ull*dit; return; }
    cw_start_element(cw_pat[cw_pat_i++]=='-',now);
}
static void cw_paddle_service(uint64_t now){
    bool dit=!gpio_get(CW_DIT_GPIO), dah=!gpio_get(CW_DAH_GPIO);
    if(dit)dit_mem=true; if(dah)dah_mem=true;
    if(cw_macro_active){ if(dit||dah){cw_macro_active=false;cw_phase=0;cw_key(false);} else return; }
    if(cw_phase && now<cw_deadline_us)return;
    if(cw_phase==1){ cw_key(false); cw_phase=2; cw_deadline_us=now+cw_dit_us(); return; }
    if(cw_phase==2) cw_phase=0;
    if(!cw_phase){
        bool use_dah=false, have=false;
        if(dit_mem && dah_mem){ use_dah=!last_was_dah; have=true; }
        else if(dit_mem){use_dah=false;have=true;}
        else if(dah_mem){use_dah=true;have=true;}
        if(have){ if(use_dah)dah_mem=false;else dit_mem=false; cw_start_element(use_dah,now); }
    }
}
static void cw_service(void){uint64_t now=time_us_64(); if(cw_macro_active)cw_macro_service(now); cw_paddle_service(now);}

static void handle(uint8_t t,const uint8_t*p,uint16_t n){
 if(t==M_CWCONTROL&&n>=2){cw_wpm=p[0];if(cw_wpm<5)cw_wpm=5;if(cw_wpm>50)cw_wpm=50;if(n>=4){cw_pitch_hz=(uint16_t)p[2]|((uint16_t)p[3]<<8);if(cw_pitch_hz<300)cw_pitch_hz=300;if(cw_pitch_hz>1000)cw_pitch_hz=1000;}if(p[1])cw_start_macro();return;}
 if(t==M_P1STATUS && n>0 && n<=24){char state[25];memcpy(state,p,n);state[n]=0;float f=0.0f,sw=1.0f;meter_read(&f,&sw);float r=0.0f;uint16_t raw=meter_adc_avg(REV_ADC_CH);r=((float)raw/4095.0f)*POWER_FS_W*POWER_CAL;if(!lpf_editor)rcb_ui_p1_status(state,f,r,sw);return;}
 if(t==M_SPLASH){rcb_ui_splash();return;} if(t==M_GATERES&&n>=1){rcb_ui_gateware_result(p[0]);return;}
 if(t==M_GATEPROG&&n>=8){uint32_t a,b;memcpy(&a,p,4);memcpy(&b,p+4,4);rcb_ui_gateware_progress(a,b);return;}
 if(t==M_RADIOINFO&&n>=5){rcb_ui_radio_info(p[0],p[1],p[2],p[3],p[4]);return;}
 if(t==M_FREQUENCY&&n==sizeof(uint32_t)){uint32_t f;memcpy(&f,p,sizeof f);lpf_update_frequency(f);return;}
 if(t==M_MAIN&&n==sizeof(mainp_t)){mainp_t q;memcpy(&q,p,sizeof q);q.mode[7]=0;lpf_update_frequency(q.f);float fwd_w=0.0f,swr=1.0f;if(q.tx)meter_read(&fwd_w,&swr);if(!lpf_editor)rcb_ui_main(q.f,q.mode,q.step,q.af,q.tx,q.rms,fwd_w,swr,q.spec,q.rit_on!=0,q.rit_hz);return;}
 char a[64],b[64],c[64];unsigned o=0;
 if(t==M_PROMPT){o=getstr(p,n,o,a,sizeof a);o=getstr(p,n,o,b,sizeof b);o=getstr(p,n,o,c,sizeof c);rcb_ui_prompt(a,b,c);return;}
 if(t==M_MESSAGE){o=getstr(p,n,o,a,sizeof a);o=getstr(p,n,o,b,sizeof b);uint32_t h=0;if(o+4<=n)memcpy(&h,p+o,4);rcb_ui_message(a,b,h);return;}
 if(t==M_MENU){o=getstr(p,n,o,a,sizeof a);if(o+3>n)return;uint8_t cnt=p[o++],sel=p[o++],edit=p[o++];static char text[32][64];static const char*lines[32];if(cnt>32)cnt=32;for(unsigned i=0;i<cnt;i++){o=getstr(p,n,o,text[i],sizeof text[i]);lines[i]=text[i];}rcb_ui_menu(a,lines,cnt,sel,edit);}
}
static void rx_task(void){uint8_t x;while(rx_pop(&x)){if(bn<sizeof buf)buf[bn++]=x;else bn=0;if(bn>=5&&buf[0]==0xA5&&buf[1]==0x5A){uint16_t n=buf[3]|((uint16_t)buf[4]<<8);if(n>sizeof(buf)-6){bn=0;continue;}if(bn==6u+n){uint8_t c=buf[2]^buf[3]^buf[4]^sum8(buf+5,n);if(c==buf[5+n])handle(buf[2],buf+5,n);bn=0;}}else if(bn>=2&&!(buf[0]==0xA5&&buf[1]==0x5A)){buf[0]=buf[bn-1];bn=1;}}}
int main(void){stdio_init_all();adc_init();adc_gpio_init(FWD_ADC_GPIO);adc_gpio_init(REV_ADC_GPIO);for(unsigned i=0;i<7u;++i){gpio_init(lpf_gpio[i]);gpio_put(lpf_gpio[i],0);gpio_set_dir(lpf_gpio[i],GPIO_OUT);}lpf_load();lpf_apply(10u);gpio_init(CW_DIT_GPIO);gpio_set_dir(CW_DIT_GPIO,GPIO_IN);gpio_pull_up(CW_DIT_GPIO);gpio_init(CW_DAH_GPIO);gpio_set_dir(CW_DAH_GPIO,GPIO_IN);gpio_pull_up(CW_DAH_GPIO);gpio_init(CW_KEY_GPIO);gpio_put(CW_KEY_GPIO,0);gpio_set_dir(CW_KEY_GPIO,GPIO_IN);sleep_ms(50);uart_init(LINK,BAUD);gpio_set_function(TX,GPIO_FUNC_UART);gpio_set_function(RX,GPIO_FUNC_UART);gpio_pull_up(RX);uart_set_fifo_enabled(LINK,true);irq_set_exclusive_handler(UART1_IRQ,on_uart_rx);irq_set_enabled(UART1_IRQ,true);uart_set_irq_enables(LINK,true,false);bool ok=rcb_ui_init();if(ok){rcb_ui_splash();rcb_ui_message("DISPLAY ZERO","WAITING FOR RADIO",400);}while(1){rx_task();cw_service();rcb_ui_task();rcb_controls_t c={0};rcb_controls_poll(&c);if(c.confirm_long && !lpf_editor){lpf_editor=true;lpf_bit_mode=false;lpf_edit_band=lpf_active_band;lpf_editor_draw();}else if(lpf_editor){lpf_editor_control(&c);}else{uint8_t b=(c.encoder_push?1:0)|(c.back?2:0)|(c.confirm?4:0)|(c.back_long?8:0);if(c.encoder_delta||b)sendctl(c.encoder_delta,b);}sleep_ms(1);} }
