/* Host test stub: just enough of the CMSIS device header for debug_coredump.c */
#ifndef _HOSTSTUB_GD32F4XX_H_
#define _HOSTSTUB_GD32F4XX_H_

#include <stdint.h>

typedef struct
{
  uint32_t CFSR, HFSR, DFSR, AFSR, MMFAR, BFAR;
} scb_stub_t;

extern scb_stub_t g_scb_stub;

#define SCB (&g_scb_stub)

#endif
