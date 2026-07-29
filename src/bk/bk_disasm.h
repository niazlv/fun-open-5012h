/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * One PDP-11 instruction, in words.
 */

#ifndef _BK_DISASM_H_
#define _BK_DISASM_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include "bk_config.h"

#if BK_DISASM

/*- Prototypes --------------------------------------------------------------*/
/*
 * Writes the instruction at `addr` into `buf` and returns its length in bytes,
 * so that stepping through a listing is addr += the return value.
 *
 * Everything is octal, without a leading zero, because that is how this
 * machine has been written about for forty years and a hexadecimal PDP-11
 * listing is unreadable to anyone who knows one.
 */
int bk_disasm(uint16_t addr, char *buf, int size);

#endif // BK_DISASM
#endif // _BK_DISASM_H_
