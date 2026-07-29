/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The 64 KB, and the sixty-four bytes of it that are not memory.
 *
 * The whole address space is one array. Two compares on the read path and
 * three on the write path decide whether an address is memory, read-only, or a
 * register - and that is the entire memory system, because a BK-0010-01 has no
 * management unit and nothing is banked. Everything that could be a table
 * lookup is a compare instead: the array is 64 KB and the comparisons are two
 * instructions, so there is nothing left for a table to save.
 */

/*- Includes ----------------------------------------------------------------*/
#include <string.h>
#include "bk_config.h"
#include "bk_mem.h"
#include "bk_io.h"

/*- Variables ---------------------------------------------------------------*/
uint8_t *bk_mem;

#if BK_VIDEO_DIRTY
uint32_t bk_vram_dirty[BK_VRAM_LINES / 32];
#endif

uint32_t bk_io_unknown;

static bool g_has_rom;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void bk_mem_attach(uint8_t *base)
{
    bk_mem = base;
    bk_mem_clear();
}

//-----------------------------------------------------------------------------
// Erased flash reads as ones and unwritten RAM reads as whatever was there;
// a BK powers up with neither, and a program that reads uninitialised memory
// should find the same thing every time so that a bug in it is reproducible.
void bk_mem_clear(void)
{
    if (NULL == bk_mem)
        return;

    memset(bk_mem, 0, BK_MEM_SIZE);

    g_has_rom = false;
    bk_io_unknown = 0;

    bk_vram_mark_all();
}

//-----------------------------------------------------------------------------
bool bk_mem_load_rom(uint16_t addr, const uint8_t *data, uint32_t size)
{
    if (NULL == bk_mem || NULL == data)
        return false;

    if ((uint32_t)addr + size > BK_MEM_SIZE)
        return false;

    memcpy(bk_mem + addr, data, size);

    if (addr <= BK_MON_BASE && (uint32_t)addr + size > BK_MON_BASE)
        g_has_rom = true;

    return true;
}

//-----------------------------------------------------------------------------
bool bk_mem_has_rom(void)
{
    return g_has_rom;
}

//-----------------------------------------------------------------------------
void bk_mem_set_has_rom(bool has)
{
    g_has_rom = has;
}

//-----------------------------------------------------------------------------
// Byte access to a register.
//
// The bus is sixteen bits wide and the registers are wired across all of it,
// so a byte access reads or writes the whole word and takes half of it. Which
// half is the low bit of the address - and MOVB to the odd address of a
// register is how BK code sets a high byte without disturbing the low one.
uint8_t bk_io_read_byte(uint16_t addr)
{
    uint16_t word = bk_io_read((uint16_t)(addr & 0xfffeu));

    return (uint8_t)((addr & 1) ? (word >> 8) : word);
}

//-----------------------------------------------------------------------------
void bk_io_write_byte(uint16_t addr, uint8_t value)
{
    uint16_t aligned = (uint16_t)(addr & 0xfffeu);

    // bk_io_reg() and not bk_io_read(): reading 0177662 clears the "a key is
    // waiting" flag and reading 0177716 clears its ready bit, and a byte write
    // that consumed a keystroke on its way past would be a keyboard that
    // loses characters whenever a program pokes at the system register.
    uint16_t word = bk_io_reg(aligned);

    if (addr & 1)
        word = (uint16_t)((word & 0x00ffu) | ((uint16_t)value << 8));
    else
        word = (uint16_t)((word & 0xff00u) | value);

    bk_io_write(aligned, word);
}
