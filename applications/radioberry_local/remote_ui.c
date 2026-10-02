#include "remote_ui.h"
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"

#define UI_UART uart1
#define UI_TX 4u
#define UI_RX 5u
#define UI_BAUD 2000000u

enum { M_SPLASH=1, M_GATEPROG, M_GATERES, M_RADIOINFO, M_MAIN, M_MESSAGE, M_MENU, M_PROMPT, M_FREQUENCY, M_P1STATUS=10, M_CONTROL=0x80, M_CWCONTROL=0x81 };
static uint8_t rx[32]; static unsigned rn;
static volatile int8_t enc_acc; static volatile uint8_t buttons;

/* Port18 diagnostic: early boot keeps the proven blocking UART path.
 * Only after Radioberry and the PIO-USB headset have initialized do we
 * claim a remaining DMA channel and switch normal UI TX to DMA. */
#define UI_TX_SLOTS 8u
#define UI_TX_MAX   706u
typedef struct { uint16_t len; uint8_t data[UI_TX_MAX]; } ui_tx_slot_t;
static ui_tx_slot_t ui_txq[UI_TX_SLOTS];
static uint8_t ui_tx_head, ui_tx_tail, ui_tx_count;
static int ui_tx_dma = -1;
static bool ui_tx_dma_enabled;

static uint8_t sum8(const uint8_t *p,unsigned n){uint8_t s=0;while(n--)s^=*p++;return s;}

static void ui_tx_kick(void){
 if(!ui_tx_dma_enabled || ui_tx_dma < 0 || ui_tx_count == 0u) return;
 if(dma_channel_is_busy((uint)ui_tx_dma)) return;
 ui_tx_slot_t *q=&ui_txq[ui_tx_tail];
 dma_channel_set_read_addr((uint)ui_tx_dma,q->data,false);
 dma_channel_set_trans_count((uint)ui_tx_dma,q->len,true);
}

static void ui_tx_service(void){
 if(!ui_tx_dma_enabled || ui_tx_dma < 0) return;
 if(ui_tx_count && !dma_channel_is_busy((uint)ui_tx_dma)){
   ui_tx_tail=(uint8_t)((ui_tx_tail+1u)%UI_TX_SLOTS);
   ui_tx_count--;
   ui_tx_kick();
 }
}

static void sendf(uint8_t type,const void *vp,uint16_t n){
 const uint8_t *p=vp; uint8_t h[5]={0xA5,0x5A,type,(uint8_t)n,(uint8_t)(n>>8)}, c=type^h[3]^h[4];
 if(n) c^=sum8(p,n);
 if(!ui_tx_dma_enabled){
   uart_write_blocking(UI_UART,h,5); if(n) uart_write_blocking(UI_UART,p,n); uart_putc_raw(UI_UART,c);
   return;
 }
 if((uint32_t)n+6u>UI_TX_MAX || ui_tx_count>=UI_TX_SLOTS) return;
 ui_tx_slot_t *q=&ui_txq[ui_tx_head];
 memcpy(q->data,h,5); if(n) memcpy(q->data+5,p,n); q->data[5u+n]=c; q->len=(uint16_t)(n+6u);
 ui_tx_head=(uint8_t)((ui_tx_head+1u)%UI_TX_SLOTS); ui_tx_count++;
 ui_tx_kick();
}
static void poll_rx(void){while(uart_is_readable(UI_UART)){uint8_t b=uart_getc(UI_UART); if(rn<sizeof(rx))rx[rn++]=b; else rn=0;
 if(rn>=6 && rx[0]==0xA5&&rx[1]==0x5A){uint16_t n=rx[3]|((uint16_t)rx[4]<<8); if(n>3){rn=0;continue;} if(rn==6u+n){uint8_t c=rx[2]^rx[3]^rx[4]^sum8(rx+5,n); if(c==rx[5+n]&&rx[2]==M_CONTROL&&n==2){enc_acc+=(int8_t)rx[5];buttons|=rx[6];} rn=0;}}
 else if(rn>=2 && !(rx[0]==0xA5&&rx[1]==0x5A)){rx[0]=rx[rn-1];rn=1;}}
}
bool rcb_ui_init(void){uart_init(UI_UART,UI_BAUD);gpio_set_function(UI_TX,GPIO_FUNC_UART);gpio_set_function(UI_RX,GPIO_FUNC_UART);gpio_pull_up(UI_RX);return true;}
bool rcb_ui_enable_tx_dma(void){
 if(ui_tx_dma_enabled) return true;
 int ch=dma_claim_unused_channel(false);
 if(ch<0) return false;
 dma_channel_config cfg=dma_channel_get_default_config((uint)ch);
 channel_config_set_transfer_data_size(&cfg,DMA_SIZE_8);
 channel_config_set_read_increment(&cfg,true);
 channel_config_set_write_increment(&cfg,false);
 channel_config_set_dreq(&cfg,DREQ_UART1_TX);
 dma_channel_configure((uint)ch,&cfg,&uart_get_hw(UI_UART)->dr,NULL,0,false);
 ui_tx_dma=ch; ui_tx_head=ui_tx_tail=ui_tx_count=0u; ui_tx_dma_enabled=true;
 return true;
}
void rcb_ui_task(void){ui_tx_service();poll_rx();}
void rcb_ui_splash(void){sendf(M_SPLASH,0,0);}
void rcb_ui_gateware_progress(size_t d,size_t t){uint32_t p[2]={(uint32_t)d,(uint32_t)t};sendf(M_GATEPROG,p,sizeof p);}
void rcb_ui_gateware_result(bool ok){uint8_t v=ok;sendf(M_GATERES,&v,1);}
void rcb_ui_radio_info(uint8_t a,uint8_t b,uint8_t f,uint8_t r,uint8_t t){uint8_t p[5]={a,b,f,r,t};sendf(M_RADIOINFO,p,5);}
typedef struct __attribute__((packed)){uint32_t f,step,af;uint8_t tx;float rms;char mode[8];uint8_t spec[128];uint8_t rit_on;int16_t rit_hz;} mainp_t;
void rcb_ui_p1_status(const char *state){char b[24]={0};strncpy(b,state,23);sendf(M_P1STATUS,b,(uint16_t)(strlen(b)+1u));}
void rcb_ui_frequency(uint32_t f){sendf(M_FREQUENCY,&f,sizeof f);}
void rcb_ui_main(uint32_t f,const char*m,uint32_t st,uint32_t af,bool tx,float rms,const uint8_t sp[128],bool rit_on,int16_t rit_hz){mainp_t p={.f=f,.step=st,.af=af,.tx=tx,.rms=rms,.rit_on=rit_on,.rit_hz=rit_hz};strncpy(p.mode,m,7);memcpy(p.spec,sp,128);sendf(M_MAIN,&p,sizeof p);}
static uint16_t putstr(uint8_t*p,uint16_t o,const char*s){size_t n=strlen(s);if(n>63)n=63;p[o++]=(uint8_t)n;memcpy(p+o,s,n);return o+n;}
void rcb_ui_message(const char*a,const char*b,uint32_t hold){uint8_t p[140];uint16_t o=0;o=putstr(p,o,a);o=putstr(p,o,b);memcpy(p+o,&hold,4);o+=4;sendf(M_MESSAGE,p,o);}
void rcb_ui_prompt(const char*a,const char*b,const char*c){uint8_t p[200];uint16_t o=0;o=putstr(p,o,a);o=putstr(p,o,b);o=putstr(p,o,c);sendf(M_PROMPT,p,o);}
void rcb_ui_menu(const char*title,const char*const*items,size_t count,size_t sel,bool edit){uint8_t p[700];uint16_t o=0;o=putstr(p,o,title);p[o++]=(uint8_t)count;p[o++]=(uint8_t)sel;p[o++]=edit;for(size_t i=0;i<count&&o<630;i++)o=putstr(p,o,items[i]);sendf(M_MENU,p,o);}
void rcb_ui_cw_control(uint8_t wpm,bool start_cq,uint16_t pitch_hz){
    uint8_t p[4]={wpm,start_cq?1u:0u,(uint8_t)(pitch_hz&0xffu),(uint8_t)(pitch_hz>>8)};
    sendf(M_CWCONTROL,p,4);
}
void rcb_controls_poll(rcb_controls_t*out){poll_rx();out->encoder_delta=enc_acc;enc_acc=0;uint8_t b=buttons;buttons=0;out->encoder_push=b&1;out->back=b&2;out->confirm=b&4;out->back_long=b&8;}
