/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host stand-in for the CMSIS core header.
 *
 * The emulator compiles the REAL vendor header, include/gd32f4xx.h, so that
 * every register offset, bit position and mask it decodes comes from the same
 * SVD-generated definitions the firmware was built against. That header is
 * portable C except for one thing: it includes core_cm4.h for the __IOM family
 * of qualifiers and for the Cortex-M core peripherals. This file supplies the
 * qualifiers and nothing else - the core peripherals (NVIC, SCB, SysTick) are
 * architectural, live at fixed addresses, and are modeled in board.c.
 */

#ifndef _EMU_CORE_CM4_SHIM_H_
#define _EMU_CORE_CM4_SHIM_H_

#include <stdint.h>

#define __I    volatile const
#define __O    volatile
#define __IO   volatile
#define __IM   volatile const
#define __OM   volatile
#define __IOM  volatile

#endif /* _EMU_CORE_CM4_SHIM_H_ */
