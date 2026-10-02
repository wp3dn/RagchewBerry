#ifndef DHCPSERVER_H
#define DHCPSERVER_H
#include <stdbool.h>
#include <stdint.h>
#include <lwip/ip_addr.h>
#ifdef __cplusplus
extern "C" {
#endif
#define DHCPS_BASE_IP (16)
#define DHCPS_MAX_IP (8)
typedef struct _dhcp_server_lease_t { uint8_t mac[6]; uint16_t expiry; } dhcp_server_lease_t;
typedef struct _dhcp_server_t {
    ip_addr_t ip;
    ip_addr_t nm;
    bool set_router_and_dns;
    dhcp_server_lease_t lease[DHCPS_MAX_IP];
    struct udp_pcb *udp;
} dhcp_server_t;
void dhcp_server_init(dhcp_server_t *d, ip_addr_t *ip, ip_addr_t *nm, bool set_router_and_dns);
void dhcp_server_deinit(dhcp_server_t *d);
#ifdef __cplusplus
}
#endif
#endif
