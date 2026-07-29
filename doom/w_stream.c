/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Texture columns, read from somewhere that is not memory.
 *
 * The asset pack does not fit in the image beside the firmware, and the 8 MB
 * part it fits on is not in the address space - the MCU has no QSPI, so there
 * is no way to make a pointer into it work. Something has to stand between the
 * renderer and the bus, and this is it.
 *
 * The shape of that something was measured rather than guessed, by
 * tests/doom_cache.c over 1896 views of E1M1:
 *
 *   - 59374 texture reads per frame, but only 557 calls to R_DrawColumn behind
 *     them. Anything done per READ costs more than the misses do; a check per
 *     COLUMN costs a hundredth of that. So the unit here is the column.
 *   - a frame touches 118 distinct sources on average and 362 at worst, and
 *     the whole working set of the worst frame is 35 KB.
 *   - nothing needs predicting. R_DrawColumn is called with its source already
 *     chosen, so the fetch is a lookahead over a decision the renderer has
 *     already taken.
 *
 * Columns are at most 70 bytes in a --scale 2 pack, so a 128 byte slot holds
 * any of them whole and the cache is a plain direct-mapped table. Flats are
 * different: R_DrawSpan indexes them by a jumping offset rather than walking
 * them, so a flat has to be resident whole - all of them together are 21 KB,
 * which is small enough to just load once.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "doom.h"

/*- Definitions -------------------------------------------------------------*/
#define SLOT_SIZE       128         // one column, with room to spare
#define SLOT_COUNT      64          // 8 KB of cache
#define SLOT_MASK       (SLOT_COUNT - 1)

#define NO_TAG          0xFFFFFFFFu

/*
 * Which bits of the offset choose the slot.
 *
 * Columns sit about 64 bytes apart in TEXDATA, so shifting by 4 - dividing the
 * offset into 16 byte units - puts consecutive columns four slots apart and
 * leaves three quarters of the table permanently empty. Shifting by 6 makes
 * consecutive columns consecutive slots, which is the whole table.
 *
 * Measured over 1896 views of E1M1 with tests/doom_cache.c: the difference is
 * a factor of six in misses per frame.
 */
#ifndef COLUMN_SHIFT
#define COLUMN_SHIFT    6
#endif

/*- Variables ---------------------------------------------------------------*/
static uint8_t *g_slots;                    // SLOT_COUNT * SLOT_SIZE
static uint32_t g_tags[SLOT_COUNT];

static w_stream_read_t g_read;
static void *g_ctx;
static uint32_t g_tex_base;                 // TEXDATA, in the stream
static uint32_t g_tex_size;

static uint32_t g_fetches;                  // for the statistics line

// Handed out when there is nothing to read from, see w_column()
static const uint8_t g_nothing[SLOT_SIZE];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void w_stream_init(w_stream_read_t read, void *ctx, uint8_t *cache,
    uint32_t tex_base, uint32_t tex_size)
{
    g_read = read;
    g_ctx = ctx;
    g_slots = cache;
    g_tex_base = tex_base;
    g_tex_size = tex_size;
    g_fetches = 0;

    for (int i = 0; i < SLOT_COUNT; i++)
        g_tags[i] = NO_TAG;
}

//-----------------------------------------------------------------------------
// One column. The offset is into TEXDATA, exactly as texturecols stores it, so
// nothing above this line has to know where the pack really is.
//
// Direct mapped on the offset: an associative search would put a loop back
// into the path this exists to keep short. A collision costs one refetch, and
// with 64 slots against the ~118 distinct columns a frame uses, most of them
// land somewhere else.
const uint8_t *w_column(uint32_t offset)
{
    uint32_t slot;
    uint8_t *at;
    uint32_t want;

    // Nobody bound the streamed half. On the device that cannot reach here -
    // the application refuses to start and says why - but a renderer that
    // faults is a worse way to find out than one that draws black.
    if (NULL == g_slots)
        return g_nothing;

    slot = (offset >> COLUMN_SHIFT) & SLOT_MASK;
    at = g_slots + slot * SLOT_SIZE;

    if (g_tags[slot] == offset)
        return at;

    want = SLOT_SIZE;

    // The last column in the lump is shorter than a slot
    if (offset + want > g_tex_size)
        want = g_tex_size - offset;

    if (!g_read(g_ctx, g_tex_base + offset, at, want))
    {
        // A bus that stopped answering must not put the renderer into a
        // texture that is half of the last one. Black is wrong and visibly so.
        memset(at, 0, SLOT_SIZE);
        g_tags[slot] = offset;

        return at;
    }

    g_tags[slot] = offset;
    g_fetches++;

    return at;
}

//-----------------------------------------------------------------------------
uint32_t w_stream_fetches(void)
{
    return g_fetches;
}

//-----------------------------------------------------------------------------
void w_stream_reset_stats(void)
{
    g_fetches = 0;
}
