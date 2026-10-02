#pragma once
#include <stdbool.h>
#include <lwip/ip.h>
bool usb_network_init(const ip4_addr_t*,const ip4_addr_t*,const ip4_addr_t*,bool);
bool usb_network_is_up(void); void usb_network_update(void); void usb_network_deinit(void);
