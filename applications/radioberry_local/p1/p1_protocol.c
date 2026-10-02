#include "p1_protocol.h"
#include <string.h>
static uint32_t seq;
void p1_protocol_reset(void){seq=0;}
void p1_build_discovery_reply(uint8_t*r,int len){
    if(len<60)return;
    memset(r,0,60);
    r[0]=0xEF; r[1]=0xFE; r[2]=0x02;
    /* Johan RP2350 Hub discovery layout */
    r[3]=0x00; r[4]=0x1C; r[5]=0xC0; r[6]=0xA2; r[7]=0x22; r[8]=0x5B;
    r[9]=0x01;       /* gateware major */
    r[10]=0x06;      /* Hermes Lite device ID */
    r[19]=0x01;      /* one receiver */
    r[21]=0x25;      /* gateware minor */
}
int p1_classify(const uint8_t*b,int n){if(n>=4&&b[0]==0xEF&&b[1]==0xFE&&b[2]==0x02&&b[3]==0)return 1;if(n>=4&&b[0]==0xEF&&b[1]==0xFE&&b[2]==0x01&&b[3]==0x02)return 3;if(n>0)return 2;return 0;}
void p1_build_ep6(uint8_t*p,const int32_t*i,const int32_t*q,const int16_t*m){memset(p,0,P1_PACKET_BYTES);p[0]=0xEF;p[1]=0xFE;p[2]=1;p[3]=6;uint32_t s=seq++;p[4]=s>>24;p[5]=s>>16;p[6]=s>>8;p[7]=s;uint8_t*d=p+8;int k=0;for(int f=0;f<2;f++){d[0]=d[1]=d[2]=0x7F;d+=8;for(int j=0;j<63;j++,k++){int32_t I=i?i[k]:0,Q=q?q[k]:0;int16_t M=m?m[k]:0;*d++=I>>16;*d++=I>>8;*d++=I;*d++=Q>>16;*d++=Q>>8;*d++=Q;*d++=M>>8;*d++=M;}}}
int p1_unpack_ep2(const uint8_t*p,int n,int16_t*iq,int max,int*mox,uint32_t*tx,uint32_t*rx){if(!p||n<P1_PACKET_BYTES||p[0]!=0xEF||p[1]!=0xFE||p[2]!=1||p[3]!=2)return 0;const uint8_t*d=p+8;int out=0,mo=0;uint32_t th=0,rh=0;for(int f=0;f<2;f++){uint8_t c0=d[3];if(f==0)mo=c0&1;if(c0==2)th=((uint32_t)d[4]<<24)|((uint32_t)d[5]<<16)|((uint32_t)d[6]<<8)|d[7];if(c0==4)rh=((uint32_t)d[4]<<24)|((uint32_t)d[5]<<16)|((uint32_t)d[6]<<8)|d[7];d+=8;for(int j=0;j<63&&out<max;j++){d+=4;iq[2*out]=(int16_t)(((uint16_t)d[0]<<8)|d[1]);iq[2*out+1]=(int16_t)(((uint16_t)d[2]<<8)|d[3]);d+=4;out++;}}if(mox)*mox=mo;if(tx)*tx=th;if(rx)*rx=rh;return out;}
