#pragma once

/*
 * Waveshare RP2350 Pi Zero PIO-USB connector:
 *   GPIO28 = D+
 *   GPIO29 = D-
 *
 * Pico-PIO-USB derives D- from D+ + 1.
 */
#ifndef USB_HEADSET_DP_PIN
#define USB_HEADSET_DP_PIN 28u
#endif

/*
 * HID PTT mapping.
 * The proven Logitech headset reports:
 *   03 01 / 03 00
 *   03 02 / 03 00
 *
 * Set this to the value belonging to the physical button you want as PTT.
 */
#ifndef USB_HEADSET_PTT_VALUE
#define USB_HEADSET_PTT_VALUE 0x01u
#endif

#ifndef USB_HEADSET_TUNE_VALUE
#define USB_HEADSET_TUNE_VALUE 0x02u
#endif

/*
 * Headset format used by the proven WP3DN baseline.
 * The package auto-selects compatible UAC1 alternates:
 *   MIC: mono, 16-bit PCM, 48 kHz
 *   OUT: stereo, 16-bit PCM, 48 kHz
 */
#ifndef USB_HEADSET_CONFIG_DUMP_BYTES
#define USB_HEADSET_CONFIG_DUMP_BYTES 1024u
#endif

/* RadioberryCore resource assignment */
#ifndef USB_HEADSET_PIO_TX_NUM
#define USB_HEADSET_PIO_TX_NUM 2u
#endif
#ifndef USB_HEADSET_PIO_RX_NUM
#define USB_HEADSET_PIO_RX_NUM 2u
#endif
#ifndef USB_HEADSET_SM_TX
#define USB_HEADSET_SM_TX 0u
#endif
#ifndef USB_HEADSET_SM_RX
#define USB_HEADSET_SM_RX 1u
#endif
#ifndef USB_HEADSET_SM_EOP
#define USB_HEADSET_SM_EOP 2u
#endif
#ifndef USB_HEADSET_DMA_TX_CH
#define USB_HEADSET_DMA_TX_CH 2u
#endif
