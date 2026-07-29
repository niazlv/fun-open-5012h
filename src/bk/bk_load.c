/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Loading, and the small ROM that stands in when there is no real one.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bk_config.h"
#include "bk_load.h"
#include "bk_mem.h"
#include "bk_cpu.h"
#include "bk_io.h"
#include "bk_video.h"

#if BK_LOAD_SPIFS
#include "spifs.h"
#endif

/*- Variables ---------------------------------------------------------------*/
static const char *g_error;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const char *bk_load_error(void)
{
    return g_error;
}

//-----------------------------------------------------------------------------
#if BK_LOAD_SPIFS
static bool read_rom(const char *name, uint16_t addr, uint32_t max)
{
    const spifs_file_t *f = spifs_find(name);

    if (NULL == f)
        return false;

    if (f->size > max)
    {
        g_error = "ROM image is too big for its window";
        return false;
    }

    if (!spifs_read(f, 0, bk_mem + addr, f->size))
    {
        g_error = "SPI read failed";
        return false;
    }

    return true;
}

//-----------------------------------------------------------------------------
bool bk_load_roms(void)
{
    bool monitor;

    spifs_scan();

    monitor = read_rom(BK_ROM_MONITOR_FILE, BK_MON_BASE, BK_MON_SIZE);

    // BASIC is optional in a way the monitor is not: a machine without it
    // starts, and one without a monitor has nothing at its boot address
    read_rom(BK_ROM_BASIC_FILE, BK_BAS_BASE, BK_BAS_SIZE);

    bk_mem_set_has_rom(monitor);

    if (!monitor && NULL == g_error)
        g_error = "no " BK_ROM_MONITOR_FILE " on the SPI flash";

    return monitor;
}

//-----------------------------------------------------------------------------
bool bk_load_bin(const char *name, bk_bin_t *out)
{
    const spifs_file_t *f;
    uint8_t header[4];
    uint32_t addr, size;

    g_error = NULL;

    f = spifs_find(name);

    if (NULL == f)
    {
        g_error = "no such file on the SPI flash";
        return false;
    }

    if (f->size < sizeof(header))
    {
        g_error = "file is shorter than a tape header";
        return false;
    }

    if (!spifs_read(f, 0, header, sizeof(header)))
    {
        g_error = "SPI read failed";
        return false;
    }

    addr = (uint32_t)header[0] | ((uint32_t)header[1] << 8);
    size = (uint32_t)header[2] | ((uint32_t)header[3] << 8);

    /*
     * What the header says, checked against what the file is.
     *
     * A tape header is four bytes with no checksum and no magic, so anything
     * at all reads as a valid one - a JPEG loads as a program starting at
     * whatever its first two bytes happen to be. These three checks are all
     * there is between a mistyped filename and 16 KB of somebody's photograph
     * being executed, so they are worth being strict about.
     */
    if (size + sizeof(header) > f->size)
    {
        g_error = "header claims more bytes than the file has";
        return false;
    }

    if (0 == size)
    {
        g_error = "header says the program is empty";
        return false;
    }

    if (addr + size > BK_ROM_BASE)
    {
        g_error = "program would run past the top of RAM";
        return false;
    }

    if (!spifs_read(f, sizeof(header), bk_mem + addr, size))
    {
        g_error = "SPI read failed";
        return false;
    }

    bk_vram_mark_all();

    if (out)
    {
        out->addr = (uint16_t)addr;
        out->size = (uint16_t)size;
        out->start = (uint16_t)addr;
    }

    return true;
}

//-----------------------------------------------------------------------------
static bool ends_with_bin(const char *name)
{
    size_t n = strlen(name);

    return n > 4 && 0 == strcmp(name + n - 4, ".bin");
}

//-----------------------------------------------------------------------------
int bk_load_list_bins(const char **names, int max)
{
    int found = 0;

    spifs_scan();

    for (int i = 0; i < spifs_count() && found < max; i++)
    {
        const spifs_file_t *f = spifs_at(i);

        if (f && ends_with_bin(f->name))
            names[found++] = f->name;
    }

    return found;
}
#endif // BK_LOAD_SPIFS

//-----------------------------------------------------------------------------
#if BK_STUB_ROM

static uint16_t g_emit;

//-----------------------------------------------------------------------------
static void emit(uint16_t word)
{
    bk_mem[g_emit] = (uint8_t)word;
    bk_mem[g_emit + 1] = (uint8_t)(word >> 8);
    g_emit = (uint16_t)(g_emit + 2);
}

//-----------------------------------------------------------------------------
// MOV #value, @#addr - 012737, then the two words. The workhorse: everything
// this stub does is putting a constant somewhere.
static void emit_set(uint16_t value, uint16_t addr)
{
    emit(0012737u);
    emit(value);
    emit(addr);
}

//-----------------------------------------------------------------------------
// A vector is two words - where to go, and what priority to go there at.
// Priority 0340 means the handler runs with interrupts held off, which for a
// handler that only returns is the safe way round.
static void emit_vector(uint16_t vector, uint16_t handler)
{
    emit_set(handler, vector);
    emit_set(0340u, (uint16_t)(vector + 2));
}

//-----------------------------------------------------------------------------
void bk_load_stub_rom(uint16_t start)
{
    uint16_t handler;
    uint16_t loop;

    if (NULL == bk_mem)
        return;

    /*
     * The handler sits at the top of the monitor window rather than in front
     * of the reset code, because 0100000 is not a place this may choose: it is
     * where the processor starts, wired into the high byte of 0177716.
     *
     * One instruction: return. Nothing here can service an EMT - see the note
     * in the header - and returning from one at least leaves the program
     * running instead of following an empty vector into erased memory.
     */
    handler = (uint16_t)(BK_MON_BASE + BK_MON_SIZE - 2);

    g_emit = handler;
    emit(0000002u);                     // RTI

    g_emit = BK_MON_BASE;

    // MOV #01000, SP - the stack the monitor leaves, just under the program
    emit(0012706u);
    emit(0001000u);

    // The screen registers, as the monitor leaves them: full screen, no scroll
    emit_set(0001330u, 0177664u);

    /*
     * Every vector that a program can reach by accident. All of them return,
     * which turns "the machine stopped" into "that instruction did nothing" -
     * and a program that installs its own handler simply overwrites these.
     */
    emit_vector(0000004u, handler);     // bus error, and the STOP key
    emit_vector(0000010u, handler);     // an instruction this processor lacks
    emit_vector(0000014u, handler);     // BPT and the trace bit
    emit_vector(0000020u, handler);     // IOT
    emit_vector(0000030u, handler);     // EMT
    emit_vector(0000034u, handler);     // TRAP
    emit_vector(0000060u, handler);     // keyboard
    emit_vector(0000100u, handler);     // the 50 Hz line
    emit_vector(0000274u, handler);     // keyboard, the other one

    // Clear the screen: 8192 words from 040000
    emit(0012700u);                     // MOV #040000, R0
    emit(0040000u);
    emit(0012701u);                     // MOV #020000, R1
    emit(0020000u);
    loop = g_emit;
    emit(0005020u);                     // CLR (R0)+
    emit((uint16_t)(0077100u | ((g_emit + 2 - loop) / 2)));  // SOB R1, loop

    // Down to priority zero, or nothing would ever interrupt the program
    emit(0106427u);                     // MTPS #0
    emit(0000000u);

    if (start)
    {
        emit(0000137u);                 // JMP @#start
        emit(start);
    }
    else
    {
        // Nothing was loaded. Stopping is the truthful end - the screen has
        // been cleared, which says the processor and the panel both work, and
        // the application reports a HALT rather than sitting in a loop that
        // looks like a machine doing something.
        emit(0000000u);                 // HALT
    }

    bk_mem_set_has_rom(true);
}
#endif // BK_STUB_ROM

//-----------------------------------------------------------------------------
// Power on.
//
// The ВМ1 takes its first address from the high byte of 0177716, and on a BK
// that byte is wired to 0200 - so the machine starts at 0100000, which is
// where the monitor is. Priority 7, because the first thing any reset sequence
// does is set its own stack up and it must not be interrupted doing it.
void bk_load_boot(void)
{
    bk_io_reset();
    bk_cpu_reset(BK_MON_BASE, 0340u);
    bk_video_init();
}
