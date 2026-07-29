/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Getting a ROM and a program into the machine.
 */

#ifndef _BK_LOAD_H_
#define _BK_LOAD_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "bk_config.h"

/*- Definitions -------------------------------------------------------------*/
/*
 * The names looked for on the SPI part.
 *
 * A monitor image is 8 KB and belongs at 0100000; BASIC is the 24 KB above it.
 * Neither is ours to ship - they are the machine's firmware, and this is a
 * 1985 Soviet computer whose ROM images circulate but are not public domain by
 * anything anybody has written down. So: put your own there, or run without
 * one and get the stand-in (BK_STUB_ROM).
 */
#define BK_ROM_MONITOR_FILE     "bk10mon.rom"
#define BK_ROM_BASIC_FILE       "bk10bas.rom"

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    uint16_t addr;              // where the tape said to put it
    uint16_t size;
    uint16_t start;             // where to begin, addr unless told otherwise
} bk_bin_t;

/*- Prototypes --------------------------------------------------------------*/
// Why the last load failed, NULL if it did not
const char *bk_load_error(void);

#if BK_LOAD_SPIFS
/*
 * The ROM images, if they are on the chip. Returns whether the monitor was
 * found - BASIC is optional and a machine without it simply has 24 KB that
 * reads as zero.
 */
bool bk_load_roms(void);

/*
 * A tape image. The format is two words - load address, then length - followed
 * by the bytes, which is what every BK program that survived is stored as.
 */
bool bk_load_bin(const char *name, bk_bin_t *out);

// Names on the chip that end in ".bin", for the picker. Returns how many were
// written to `names`, which holds pointers into spifs's own table.
int bk_load_list_bins(const char **names, int max);
#endif

#if BK_STUB_ROM
/*
 * The stand-in for a monitor.
 *
 * Assembled here rather than kept as a blob, because a table of octal words is
 * something nobody can check by reading and this is forty instructions that
 * have to be right. It sets the stack and the screen registers up the way the
 * monitor leaves them, points every vector at an RTI so that a stray trap does
 * not end the world, clears the screen, and jumps to `start`.
 *
 * It is not a monitor. There is no EMT dispatcher behind those vectors, so a
 * program that asks the firmware to print a character gets silence. It is
 * enough for a self-contained game, which is most of them.
 *
 * `start` of zero means nothing was loaded: it clears the screen and stops,
 * which at least demonstrates that the processor and the panel work.
 */
void bk_load_stub_rom(uint16_t start);
#endif

// Power on: registers cleared, processor at the ВМ1's boot address.
void bk_load_boot(void);

#endif // _BK_LOAD_H_
