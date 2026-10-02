/* Ragchew Machine DHCP server, derived from the Pico/MicroPython TinyUSB network example. */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <lwip/udp.h>
#include <pico/time.h>
#include "dhcpserver.h"

#define DHCPDISCOVER 1
#define DHCPOFFER 2
#define DHCPREQUEST 3
#define DHCPACK 5
#define DHCP_OPT_SUBNET_MASK 1
#define DHCP_OPT_ROUTER 3
#define DHCP_OPT_DNS 6
#define DHCP_OPT_REQUESTED_IP 50
#define DHCP_OPT_IP_LEASE_TIME 51
#define DHCP_OPT_MSG_TYPE 53
#define DHCP_OPT_SERVER_ID 54
#define DHCP_OPT_END 255
#define PORT_DHCP_SERVER 67
#define PORT_DHCP_CLIENT 68
#define DEFAULT_LEASE_TIME_S (24 * 60 * 60)
#define MAC_LEN 6

typedef struct {
    uint8_t op, htype, hlen, hops;
    uint32_t xid;
    uint16_t secs, flags;
    uint8_t ciaddr[4], yiaddr[4], siaddr[4], giaddr[4];
    uint8_t chaddr[16], sname[64], file[128], options[312];
} dhcp_msg_t;

static inline uint32_t get_ticks_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static int socket_new(struct udp_pcb **udp, void *cb_data, udp_recv_fn cb) {
    *udp = udp_new();
    if (!*udp) return -ENOMEM;
    udp_recv(*udp, cb, cb_data);
    return 0;
}
static void socket_free(struct udp_pcb **udp) { if (*udp) { udp_remove(*udp); *udp = NULL; } }
static int socket_bind(struct udp_pcb **udp, uint16_t port) { return udp_bind(*udp, IP_ANY_TYPE, port); }
static int socket_sendto(struct udp_pcb **udp, struct netif *nif, const void *buf, size_t len, uint32_t ip, uint16_t port) {
    if (len > 0xffff) len = 0xffff;
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (!p) return -ENOMEM;
    memcpy(p->payload, buf, len);
    ip_addr_t dest;
    IP4_ADDR(ip_2_ip4(&dest), ip >> 24 & 0xff, ip >> 16 & 0xff, ip >> 8 & 0xff, ip & 0xff);
    err_t err = nif ? udp_sendto_if(*udp, p, &dest, port, nif) : udp_sendto(*udp, p, &dest, port);
    pbuf_free(p);
    return err == ERR_OK ? (int)len : err;
}
static uint8_t *opt_find(uint8_t *opt, uint8_t cmd) {
    for (int i = 0; i < 308 && opt[i] != DHCP_OPT_END;) {
        if (opt[i] == cmd) return &opt[i];
        if (opt[i] == 0) { i++; continue; }
        i += 2 + opt[i + 1];
    }
    return NULL;
}
static void opt_write_n(uint8_t **opt, uint8_t cmd, size_t n, const void *data) {
    uint8_t *o=*opt; *o++=cmd; *o++=(uint8_t)n; memcpy(o,data,n); *opt=o+n;
}
static void opt_write_u8(uint8_t **opt, uint8_t cmd, uint8_t val) {
    uint8_t *o=*opt; *o++=cmd; *o++=1; *o++=val; *opt=o;
}
static void opt_write_u32(uint8_t **opt, uint8_t cmd, uint32_t val) {
    uint8_t *o=*opt; *o++=cmd; *o++=4; *o++=val>>24; *o++=val>>16; *o++=val>>8; *o++=val; *opt=o;
}

static void process(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *src_addr, u16_t src_port) {
    dhcp_server_t *d = arg; (void)upcb; (void)src_addr; (void)src_port;
    dhcp_msg_t msg;
    if (p->tot_len < 243) goto out;
    size_t len = pbuf_copy_partial(p, &msg, sizeof(msg), 0);
    if (len < 243) goto out;
    msg.op = DHCPOFFER;
    memcpy(msg.yiaddr, &ip4_addr_get_u32(ip_2_ip4(&d->ip)), 4);
    uint8_t *opt = msg.options + 4;
    uint8_t *mt = opt_find(opt, DHCP_OPT_MSG_TYPE);
    if (!mt) goto out;
    switch (mt[2]) {
        case DHCPDISCOVER: {
            int yi = DHCPS_MAX_IP;
            for (int i=0;i<DHCPS_MAX_IP;i++) {
                if (!memcmp(d->lease[i].mac,msg.chaddr,MAC_LEN)) { yi=i; break; }
                if (yi==DHCPS_MAX_IP) {
                    if (!memcmp(d->lease[i].mac,"\0\0\0\0\0\0",MAC_LEN)) yi=i;
                    uint32_t expiry=((uint32_t)d->lease[i].expiry<<16)|0xffffu;
                    if ((int32_t)(expiry-get_ticks_ms())<0) { memset(d->lease[i].mac,0,MAC_LEN); yi=i; }
                }
            }
            if (yi==DHCPS_MAX_IP) goto out;
            msg.yiaddr[3]=(uint8_t)(DHCPS_BASE_IP+yi);
            opt_write_u8(&opt,DHCP_OPT_MSG_TYPE,DHCPOFFER);
            break;
        }
        case DHCPREQUEST: {
            uint8_t *o=opt_find(opt,DHCP_OPT_REQUESTED_IP);
            if (!o) goto out;
            if (memcmp(o+2,&ip4_addr_get_u32(ip_2_ip4(&d->ip)),3)) goto out;
            uint8_t yi=(uint8_t)(o[5]-DHCPS_BASE_IP);
            if (yi>=DHCPS_MAX_IP) goto out;
            if (memcmp(d->lease[yi].mac,msg.chaddr,MAC_LEN)) {
                if (memcmp(d->lease[yi].mac,"\0\0\0\0\0\0",MAC_LEN)) goto out;
                memcpy(d->lease[yi].mac,msg.chaddr,MAC_LEN);
            }
            d->lease[yi].expiry=(uint16_t)((get_ticks_ms()+DEFAULT_LEASE_TIME_S*1000u)>>16);
            msg.yiaddr[3]=(uint8_t)(DHCPS_BASE_IP+yi);
            opt_write_u8(&opt,DHCP_OPT_MSG_TYPE,DHCPACK);
            printf("DHCPS: client connected IP=%u.%u.%u.%u\n",msg.yiaddr[0],msg.yiaddr[1],msg.yiaddr[2],msg.yiaddr[3]);
            break;
        }
        default: goto out;
    }
    opt_write_n(&opt,DHCP_OPT_SERVER_ID,4,&ip4_addr_get_u32(ip_2_ip4(&d->ip)));
    opt_write_n(&opt,DHCP_OPT_SUBNET_MASK,4,&ip4_addr_get_u32(ip_2_ip4(&d->nm)));
    if (d->set_router_and_dns) {
        opt_write_n(&opt,DHCP_OPT_ROUTER,4,&ip4_addr_get_u32(ip_2_ip4(&d->ip)));
        opt_write_n(&opt,DHCP_OPT_DNS,4,&ip4_addr_get_u32(ip_2_ip4(&d->ip)));
    }
    opt_write_u32(&opt,DHCP_OPT_IP_LEASE_TIME,DEFAULT_LEASE_TIME_S);
    *opt++=DHCP_OPT_END;
    socket_sendto(&d->udp,ip_current_input_netif(),&msg,(size_t)(opt-(uint8_t*)&msg),0xffffffffu,PORT_DHCP_CLIENT);
out:
    pbuf_free(p);
}

void dhcp_server_init(dhcp_server_t *d, ip_addr_t *ip, ip_addr_t *nm, bool set_router_and_dns) {
    ip_addr_copy(d->ip,*ip); ip_addr_copy(d->nm,*nm); d->set_router_and_dns=set_router_and_dns;
    memset(d->lease,0,sizeof(d->lease)); d->udp=NULL;
    if (socket_new(&d->udp,d,process)!=0) return;
    socket_bind(&d->udp,PORT_DHCP_SERVER);
}
void dhcp_server_deinit(dhcp_server_t *d) { socket_free(&d->udp); }
