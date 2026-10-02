#pragma once

/*
 * Default HOST-ONLY TinyUSB configuration for wp3dn_usb_headset.
 *
 * A radio project that already has a tusb_config.h may point
 * USB_HEADSET_TUSB_CONFIG_DIR at its own configuration instead.
 */

#define CFG_TUSB_OS                 OPT_OS_PICO

#define CFG_TUD_ENABLED             0
#define CFG_TUH_ENABLED             1
#define CFG_TUH_RPI_PIO_USB         1

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__((aligned(4)))
#endif

#define CFG_TUH_ENUMERATION_BUFSIZE 1024
#define CFG_TUH_HUB                 1
#define CFG_TUH_DEVICE_MAX          4
#define CFG_TUH_HID                 4
#define CFG_TUH_HID_EPIN_BUFSIZE    64
#define CFG_TUH_HID_EPOUT_BUFSIZE   64

/* Generic endpoint API is used for custom UAC1 ISO streaming. */
#define CFG_TUH_API_EDPT_XFER       1
