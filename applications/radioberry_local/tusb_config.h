/* Exact Ragchew Machine RadioCore TinyUSB device configuration. */
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <lwip/opt.h>

#ifndef PICO_TUD_RHPORT
#define PICO_TUD_RHPORT 0
#endif

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_DEFAULT_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#define USE_ECM 0

#define CFG_TUD_NCM_IN_NTB_MAX_SIZE (2 * TCP_MSS + 100)
#define CFG_TUD_NCM_OUT_NTB_MAX_SIZE (2 * TCP_MSS + 100)
#ifndef CFG_TUD_NCM_OUT_NTB_N
#define CFG_TUD_NCM_OUT_NTB_N 1
#endif
#ifndef CFG_TUD_NCM_IN_NTB_N
#define CFG_TUD_NCM_IN_NTB_N 1
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

#define CFG_TUD_ECM_RNDIS USE_ECM
#define CFG_TUD_NCM (1 - CFG_TUD_ECM_RNDIS)

#ifdef __cplusplus
}
#endif
#endif
