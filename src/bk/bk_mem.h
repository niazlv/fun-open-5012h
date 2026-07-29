/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The BK's 64 KB, and who answers for each part of it.
 */

#ifndef _BK_MEM_H_
#define _BK_MEM_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "bk_config.h"

/*- Definitions -------------------------------------------------------------*/
/*
 * The map, in the octal everything about this machine is written in:
 *
 *   000000-000777   system variables and the stack the monitor sets up
 *   001000-037777   the user's 15.5 KB
 *   040000-077777   screen, 16 KB, 256 lines of 64 bytes
 *   100000-117777   monitor ROM, 8 KB
 *   120000-177577   BASIC, 23.9 KB
 *   177600-177777   the registers
 *
 * There is no hole. Every address answers, which is why a bus error on this
 * machine means an odd word address and nothing else.
 */
#define BK_RAM_BASE             0000000u
#define BK_VRAM_BASE            0040000u        // 0x4000
#define BK_VRAM_SIZE            0040000u        // 16 KB
#define BK_ROM_BASE             0100000u        // 0x8000
#define BK_ROM_SIZE             0077600u
#define BK_IO_BASE              0177600u        // 0xFF80
#define BK_MEM_SIZE             0200000u        // the whole 64 KB

#define BK_VRAM_LINES           256
#define BK_VRAM_STRIDE          0100u           // 64 bytes a line

// Where the monitor and BASIC images go
#define BK_MON_BASE             0100000u
#define BK_MON_SIZE             0020000u        // 8 KB
#define BK_BAS_BASE             0120000u
#define BK_BAS_SIZE             0057600u        // to 0177577

/*- Variables ---------------------------------------------------------------*/
/*
 * The address space, flat.
 *
 * One array rather than separate RAM and ROM blocks, because every access then
 * decodes with two compares and an index instead of a search - and the
 * emulator's inner loop is nothing but accesses. ROM is protected on the write
 * path, not by living somewhere else.
 *
 * A pointer rather than an array so the caller decides where it lives: on the
 * device it is handed the SRAM the capture ring is not using, and the host
 * test hands it 64 KB of its own.
 */
extern uint8_t *bk_mem;

#if BK_VIDEO_DIRTY
// One bit a screen line, set by the write path, cleared by whoever painted it
extern uint32_t bk_vram_dirty[BK_VRAM_LINES / 32];
#endif

// Reads of I/O addresses nothing here decodes. Not an error - a real bus
// answers, and the number is worth seeing on the debug page when a program is
// talking to hardware this does not have.
extern uint32_t bk_io_unknown;

/*- Prototypes --------------------------------------------------------------*/
// Zero the RAM, drop the ROM, mark the whole screen for repaint.
void bk_mem_attach(uint8_t *base);
void bk_mem_clear(void);

// Copy an image in above 0100000. Returns false if it would run off the end.
bool bk_mem_load_rom(uint16_t addr, const uint8_t *data, uint32_t size);

// True once something has been put at the monitor's address
bool bk_mem_has_rom(void);
void bk_mem_set_has_rom(bool has);

// The I/O page. Word addresses only - the byte paths below fold to these.
uint16_t bk_io_read(uint16_t addr);
void bk_io_write(uint16_t addr, uint16_t value);
uint8_t bk_io_read_byte(uint16_t addr);
void bk_io_write_byte(uint16_t addr, uint8_t value);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
#if BK_VIDEO_DIRTY
static inline void bk_vram_mark(uint16_t addr)
{
    unsigned line = (addr - BK_VRAM_BASE) / BK_VRAM_STRIDE;

    bk_vram_dirty[line >> 5] |= 1u << (line & 31);
}

static inline void bk_vram_mark_all(void)
{
    for (unsigned i = 0; i < BK_VRAM_LINES / 32; i++)
        bk_vram_dirty[i] = 0xffffffffu;
}
#else
static inline void bk_vram_mark(uint16_t addr) { (void)addr; }
static inline void bk_vram_mark_all(void) { }
#endif

//-----------------------------------------------------------------------------
// The four accessors the processor runs on.
//
// Inline and in the header on purpose: they are the emulator's hot path, three
// or four of them per emulated instruction, and a call through a translation
// unit boundary would cost more than the decode does. Word addresses are
// forced even here rather than trusted - the odd-address trap is the
// processor's business, and by the time an address reaches this it has already
// been decided.
static inline uint16_t bk_read16(uint16_t addr)
{
    addr &= 0xfffeu;

    if (addr >= BK_IO_BASE)
        return bk_io_read(addr);

    return *(const uint16_t *)(const void *)&bk_mem[addr];
}

//-----------------------------------------------------------------------------
static inline void bk_write16(uint16_t addr, uint16_t value)
{
    addr &= 0xfffeu;

    if (addr >= BK_IO_BASE)
    {
        bk_io_write(addr, value);
        return;
    }

#if BK_MEM_ROM_READONLY
    if (addr >= BK_ROM_BASE)
        return;
#endif

    *(uint16_t *)(void *)&bk_mem[addr] = value;

    if (addr >= BK_VRAM_BASE && addr < BK_ROM_BASE)
        bk_vram_mark(addr);
}

//-----------------------------------------------------------------------------
static inline uint8_t bk_read8(uint16_t addr)
{
    if (addr >= BK_IO_BASE)
        return bk_io_read_byte(addr);

    return bk_mem[addr];
}

//-----------------------------------------------------------------------------
static inline void bk_write8(uint16_t addr, uint8_t value)
{
    if (addr >= BK_IO_BASE)
    {
        bk_io_write_byte(addr, value);
        return;
    }

#if BK_MEM_ROM_READONLY
    if (addr >= BK_ROM_BASE)
        return;
#endif

    bk_mem[addr] = value;

    if (addr >= BK_VRAM_BASE && addr < BK_ROM_BASE)
        bk_vram_mark(addr);
}

#endif // _BK_MEM_H_
