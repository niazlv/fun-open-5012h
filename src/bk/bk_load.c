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

#if BK_LOAD_SPIFS
// The pack, once bk_load_pack_open has found and checked one. Two words, so
// that a collection of any size costs the device nothing to have.
static const spifs_file_t *g_pack;
static int g_pack_count;
#endif

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
// A tape image at `at` inside `f`, with `avail` bytes of file behind it.
//
// Both callers come through here - a loose .bin is the whole file, an entry in
// the pack is a window into one - so the checks below are made once and made
// the same way.
static bool load_tape(const spifs_file_t *f, uint32_t at, uint32_t avail,
    bk_bin_t *out)
{
    uint8_t header[4];
    uint32_t addr, size;

    if (avail < sizeof(header))
    {
        g_error = "file is shorter than a tape header";
        return false;
    }

    if (!spifs_read(f, at, header, sizeof(header)))
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
    if (size + sizeof(header) > avail)
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

    if (!spifs_read(f, at + sizeof(header), bk_mem + addr, size))
    {
        g_error = "SPI read failed";
        return false;
    }

    bk_vram_mark_all();

    if (out)
    {
        out->addr = (uint16_t)addr;
        out->size = (uint16_t)size;

        /*
         * Where to begin, which is not the same question as where it loaded.
         *
         * A tape header says where the bytes go and says nothing about where
         * to start - on a real machine that is the address the operator types
         * after the monitor's S, and 001000 is what it defaults to. An image
         * that loads below 001000 is one carrying its own low memory: the
         * vectors and the variables a program sets up down there come off the
         * tape with it, and its code still begins at 001000. Starting at the
         * load address instead lands in the middle of a vector table.
         *
         * Measured, not assumed. Of the 77 programs in the pack, 49 stopped
         * the machine when started at their load address; starting the ones
         * below 001000 at 001000 took that to 27 and took the number that
         * draw a screen from 19 to 35. tests/build/bk_host --start is the
         * same experiment on the host.
         */
        out->start = (addr < 01000u) ? 01000u : (uint16_t)addr;
    }

    return true;
}

//-----------------------------------------------------------------------------
bool bk_load_bin(const char *name, bk_bin_t *out)
{
    const spifs_file_t *f;

    g_error = NULL;

    f = spifs_find(name);

    if (NULL == f)
    {
        g_error = "no such file on the SPI flash";
        return false;
    }

    return load_tape(f, 0, f->size, out);
}

//-----------------------------------------------------------------------------
// One 32 byte index record. Kept off the stack of the caller's caller by
// reading only the eight bytes that matter when only those are wanted.
static bool pack_entry(int index, uint32_t *offset, uint32_t *size)
{
    uint8_t rec[8];

    if (NULL == g_pack || index < 0 || index >= g_pack_count)
        return false;

    if (!spifs_read(g_pack, BK_PACK_HEADER + (uint32_t)index * BK_PACK_ENTRY +
            BK_PACK_NAME, rec, sizeof(rec)))
        return false;

    *offset = (uint32_t)rec[0] | ((uint32_t)rec[1] << 8) |
        ((uint32_t)rec[2] << 16) | ((uint32_t)rec[3] << 24);
    *size = (uint32_t)rec[4] | ((uint32_t)rec[5] << 8) |
        ((uint32_t)rec[6] << 16) | ((uint32_t)rec[7] << 24);

    // An index that points outside its own file is a pack somebody truncated,
    // and the window it describes is what load_tape would otherwise trust
    if (*offset > g_pack->size || *size > g_pack->size - *offset)
        return false;

    return true;
}

//-----------------------------------------------------------------------------
int bk_load_pack_open(void)
{
    uint8_t head[BK_PACK_HEADER];
    uint32_t magic, version, count;

    g_pack = NULL;
    g_pack_count = 0;

    g_pack = spifs_find(BK_PACK_FILE);

    if (NULL == g_pack || g_pack->size < BK_PACK_HEADER)
    {
        g_pack = NULL;
        return 0;
    }

    if (!spifs_read(g_pack, 0, head, sizeof(head)))
    {
        g_pack = NULL;
        return 0;
    }

    magic = (uint32_t)head[0] | ((uint32_t)head[1] << 8) |
        ((uint32_t)head[2] << 16) | ((uint32_t)head[3] << 24);
    version = (uint32_t)head[4] | ((uint32_t)head[5] << 8) |
        ((uint32_t)head[6] << 16) | ((uint32_t)head[7] << 24);
    count = (uint32_t)head[8] | ((uint32_t)head[9] << 8) |
        ((uint32_t)head[10] << 16) | ((uint32_t)head[11] << 24);

    // The index has to fit inside the file before anything reads a record out
    // of it, or a count of four billion walks off the end of the part
    if (BK_PACK_MAGIC != magic || BK_PACK_VERSION != version ||
        count > (g_pack->size - BK_PACK_HEADER) / BK_PACK_ENTRY)
    {
        g_pack = NULL;
        return 0;
    }

    g_pack_count = (int)count;

    return g_pack_count;
}

//-----------------------------------------------------------------------------
bool bk_load_pack_name(int index, char *name, uint32_t max)
{
    char raw[BK_PACK_NAME];

    if (NULL == g_pack || index < 0 || index >= g_pack_count || 0 == max)
        return false;

    if (!spifs_read(g_pack, BK_PACK_HEADER + (uint32_t)index * BK_PACK_ENTRY,
            (uint8_t *)raw, sizeof(raw)))
        return false;

    if (max > sizeof(raw))
        max = sizeof(raw) + 1;

    memcpy(name, raw, max - 1);
    name[max - 1] = 0;

    return true;
}

//-----------------------------------------------------------------------------
bool bk_load_pack_run(int index, bk_bin_t *out)
{
    uint32_t offset, size;

    g_error = NULL;

    if (!pack_entry(index, &offset, &size))
    {
        g_error = "the pack's index does not describe that program";
        return false;
    }

    return load_tape(g_pack, offset, size, out);
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
