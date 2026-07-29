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

/*
 * The pack, and why there is one.
 *
 * A tape image is eleven kilobytes on average and a collection is hundreds of
 * them, which spifs cannot hold: its table is sixteen records because each one
 * is 48 bytes of a TCM with a hundred bytes to spare, and every file rounds up
 * to a 4 KB sector besides. So the many-programs case is one file - built by
 * tools/bkpack.py, the same trade doom.tex makes.
 *
 * Its index is read off the chip a record at a time rather than cached, so the
 * size of somebody's collection costs the device nothing. The layout, little
 * endian:
 *
 *   0   magic    "BKPK"
 *   4   version
 *   8   count
 *   12  reserved
 *   16  entries[count], 32 bytes each:
 *         0   name    24 bytes, NUL padded
 *         24  offset  from the start of the pack
 *         28  size    payload bytes, the four byte tape header included
 *   ...  the payloads, each aligned to 4
 *
 * A single .bin on the chip still works and still shows up in the picker. The
 * pack is for when there are more of them than spifs can name.
 */
#define BK_PACK_FILE            "bkgames.pak"
#define BK_PACK_MAGIC           0x4B504B42u     // "BKPK"
#define BK_PACK_VERSION         1
#define BK_PACK_HEADER          16
#define BK_PACK_ENTRY           32
#define BK_PACK_NAME            24

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

/*
 * The pack, if there is one. Call this after a scan; it reads and checks the
 * pack's own header and returns how many programs are in it, zero if the file
 * is absent or does not look like a pack.
 */
int bk_load_pack_open(void);

// Entry `index`'s name, read off the chip. `max` counts the terminator.
bool bk_load_pack_name(int index, char *name, uint32_t max);

// Entry `index`, loaded the same way and with the same checks as a loose .bin.
bool bk_load_pack_run(int index, bk_bin_t *out);
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
