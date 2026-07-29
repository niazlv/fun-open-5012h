/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * How badly would DOOM miss, if its textures lived on the SPI part?
 *
 * The asset pack is ~208 KB and the firmware image has no room for it. The
 * chip has 8 MB and plenty, but it is not in the address space: every byte has
 * to come over the bus, 256 at a time, and the renderer reads the pack once
 * per screen pixel. Whether that is workable is not a matter of opinion - it
 * is a miss rate, and this measures it.
 *
 * The renderer is hardware-free, so it runs here exactly as it runs on the
 * device. DOOM_TRACE turns the three texture reads in r_draw.c into calls into
 * this file, which feeds them to a set of cache models. Nothing is simulated
 * about the rendering itself: these are the real reads, from the real map,
 * with the real projection.
 *
 *   cc -O2 -DDOOM_TRACE -I doom -o /tmp/doomcache tests/doom_cache.c doom/[a-z]*.c
 *   /tmp/doomcache doom/doom_assets.bin
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "doom.h"

#define VIEWHEIGHT      (41 * FRACUNIT)

// What a miss costs. A read is one command plus four address bytes plus the
// line, shifted at the clock the driver uses; flash.c runs SPI0 at PCLK/8 =
// 15.6 MHz today, and Fast Read at PCLK/4 would halve these.
#define SPI_HZ          15625000.0
#define SPI_HZ_FAST     31250000.0

// The frame the device actually has. DOOM renders flat out with no tick, but
// anything slower than this reads as a stutter rather than as a frame rate.
#define FRAME_MS        18.0

/*- The cache models --------------------------------------------------------*/

// Direct mapped, because that is what fits in a few hundred bytes of state on
// a Cortex-M4 without an associative search per pixel. A fully associative LRU
// would do better; the compulsory count below is its limit.
typedef struct
{
    const char *name;
    uint32_t line;              // bytes per line
    uint32_t capacity;          // bytes
    uint32_t *tags;             // one per line slot, 0xffffffff = empty
    uint64_t misses;
    uint64_t reads;
} cache_t;

#define MAX_CACHES      12
#define PACK_MAX        (1u << 20)

static cache_t g_caches[MAX_CACHES];
static int g_cache_count;

// Compulsory misses: distinct lines touched in a frame, which is what an
// infinitely large cache would fetch. The floor under every model above.
static uint8_t g_seen[PACK_MAX / 64];
static uint32_t g_compulsory;
static uint32_t g_compulsory_line = 64;

static uint8_t g_hostcache[W_STREAM_CACHE_SIZE];
static const uint8_t *g_blob;
static uint64_t g_reads;

// Per-frame counters for the whole-column model, see doom_trace_fetch below
static uint32_t g_col_calls, g_span_calls;
static uint32_t g_col_bytes;
static uint32_t g_distinct_src, g_src_seen[4096];
static uint32_t g_src_epoch = 1;

//-----------------------------------------------------------------------------
static void cache_add(const char *name, uint32_t line, uint32_t capacity)
{
    cache_t *c = &g_caches[g_cache_count++];

    c->name = name;
    c->line = line;
    c->capacity = capacity;
    c->tags = malloc(sizeof(uint32_t) * (capacity / line));

    memset(c->tags, 0xff, sizeof(uint32_t) * (capacity / line));
}

//-----------------------------------------------------------------------------
static void frame_reset(bool keep)
{
    for (int i = 0; i < g_cache_count; i++)
    {
        cache_t *c = &g_caches[i];

        if (!keep)
            memset(c->tags, 0xff, sizeof(uint32_t) * (c->capacity / c->line));

        c->misses = 0;
        c->reads = 0;
    }

    g_col_calls = 0;
    g_span_calls = 0;
    g_col_bytes = 0;
    g_distinct_src = 0;
    g_src_epoch++;

    memset(g_seen, 0, sizeof(g_seen));
    g_compulsory = 0;
    g_reads = 0;
}

//-----------------------------------------------------------------------------
// Called from the renderer's inner loops, once per screen pixel that samples a
// texture. Hot enough that the models are deliberately arithmetic only.
void doom_trace_read(const void *addr)
{
    uint32_t off = (uint32_t)((const uint8_t *)addr - g_blob);
    uint32_t cl = off / g_compulsory_line;

    g_reads++;

    // Since the columns come through w_stream.c, a read points into the cache
    // rather than into the pack, and there is no offset to model. The models
    // below stay for the historical comparison against a pack in flash; what
    // matters now is the fetch count from the cache itself.
    if (off >= PACK_MAX)
        return;

    if (0 == (g_seen[cl >> 3] & (1u << (cl & 7))))
    {
        g_seen[cl >> 3] |= 1u << (cl & 7);
        g_compulsory++;
    }

    for (int i = 0; i < g_cache_count; i++)
    {
        cache_t *c = &g_caches[i];
        uint32_t tag = off / c->line;
        uint32_t slot = tag & (c->capacity / c->line - 1);

        c->reads++;

        if (c->tags[slot] != tag)
        {
            c->tags[slot] = tag;
            c->misses++;
        }
    }
}

/*- Fetching whole columns instead --------------------------------------------
 *
 * The per-pixel model above answers "how often would a cache miss", and that
 * turns out to be the wrong question. There are 59 000 texture reads in a
 * frame; anything done per read costs more than the misses do. A direct mapped
 * check is five or six instructions, which at 250 MHz is well over a
 * millisecond a frame spent deciding whether to fetch.
 *
 * The renderer does not need one. R_DrawColumn is called once per screen
 * column with dc_source already chosen, and R_DrawSpan once per scanline of
 * floor with ds_source already chosen. The unit that has to be resident is
 * therefore a column or a flat, and it is known one call earlier - which is a
 * prefetch, not a prediction: nothing is being guessed.
 *
 * This counts those calls and the distinct sources behind them.
 */
void doom_trace_fetch(const void *addr, int bytes)
{
    uint32_t off = (uint32_t)((const uint8_t *)addr - g_blob);
    uint32_t slot = (off >> 6) & 4095;

    if (bytes > 0)
    {
        g_col_calls++;
        g_col_bytes += (uint32_t)bytes;
    }
    else
    {
        g_span_calls++;
    }

    if (g_src_seen[slot] != g_src_epoch)
    {
        g_src_seen[slot] = g_src_epoch;
        g_distinct_src++;
    }
}

/*- The sweep ---------------------------------------------------------------*/

static void *load_file(const char *path, long *size)
{
    FILE *f = fopen(path, "rb");
    void *buf;

    if (!f) { perror(path); exit(1); }

    fseek(f, 0, SEEK_END);
    *size = ftell(f);
    fseek(f, 0, SEEK_SET);

    buf = malloc(*size);

    if (1 != fread(buf, *size, 1, f)) { fprintf(stderr, "short read\n"); exit(1); }

    fclose(f);

    return buf;
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    long size;
    void *blob;
    int frames = 0;
    double worst_ms = 0;
    uint64_t total_reads = 0;
    uint32_t worst_compulsory = 0;
    uint64_t tot_cols = 0, tot_spans = 0, tot_col_bytes = 0, tot_src = 0;
    uint32_t worst_cols = 0, worst_src = 0;
    uint64_t tot_fetch = 0;
    uint32_t worst_fetch = 0;

    struct { uint64_t misses, worst; } acc[MAX_CACHES] = {{0, 0}};

    // Cold: every frame starts with nothing cached, which is what a first
    // frame after a teleport costs. Warm: the cache survives between frames,
    // which is what every frame of somebody walking around costs - the view
    // moves a little and the textures in it are the ones that were there a
    // sixtieth of a second ago. Both are worth knowing; only one of them is
    // what playing feels like.
    bool warm = (argc > 2) && (0 == strcmp(argv[2], "--warm"));

    if (argc < 2)
    {
        fprintf(stderr, "usage: %s assets.bin [angles]\n", argv[0]);
        return 1;
    }

    blob = load_file(argv[1], &size);

    dm = calloc(1, sizeof(doom_mem_t));

    if (!doom_assets_init(blob, g_hostcache) || !doom_level_load())
    {
        fprintf(stderr, "asset pack or level rejected\n");
        return 1;
    }

    g_blob = doom_assets_blob();

    R_Init();
    R_InitSegs();

    printf("%s cache.  pack %ld KB, map %s, %d subsectors\n\n", warm ? "WARM" : "COLD", size / 1024,
           doom_level_name(), numsubsectors);

    // The two the firmware can actually have: 64 slots of 128 bytes in the
    // memory borrowed from the oscilloscope, indexed two ways. w_stream.c
    // picks the slot with `offset >> 4`, and columns in the pack are about 64
    // bytes apart - so consecutive columns land four slots apart and only a
    // quarter of the table is ever used. `>> 6` is the same table, used whole.
    cache_add("as built, offset>>4", 128, 2 * 1024);   // 16 usable slots
    cache_add("indexed offset>>6", 128, 8 * 1024);     // all 64

    // A line is a bus transaction whatever its size, so the small ones are
    // included to show what the per-command overhead does to them.
    cache_add("64 B lines,  8 KB", 64, 8 * 1024);
    cache_add("64 B lines, 16 KB", 64, 16 * 1024);
    cache_add("64 B lines, 32 KB", 64, 32 * 1024);
    cache_add("128 B lines, 16 KB", 128, 16 * 1024);
    cache_add("128 B lines, 32 KB", 128, 32 * 1024);
    cache_add("128 B lines, 64 KB", 128, 64 * 1024);
    cache_add("256 B lines, 32 KB", 256, 32 * 1024);
    cache_add("256 B lines, 64 KB", 256, 64 * 1024);
    cache_add("256 B lines, 128 KB", 256, 128 * 1024);

    // Every subsector, from eight angles: the same sweep tests/doom_host.c
    // uses to find the renderer's high water marks, so the viewpoints are ones
    // the map can actually produce rather than ones that flatter the cache.
    for (int i = 0; i < numsubsectors; i++)
    {
        const mapsubsector_t *ss = &subsectors[i];
        const mapseg_t *seg = &segs[ss->firstseg];
        fixed_t x = (mapfix(vertexes[seg->v1].x) + mapfix(vertexes[seg->v2].x)) / 2;
        fixed_t y = (mapfix(vertexes[seg->v1].y) + mapfix(vertexes[seg->v2].y)) / 2;

        for (int a = 0; a < 8; a++)
        {
            int sub = R_PointInSubsector(x, y);
            const sector_t *sec = &sectors[sides[lines[segs[subsectors[sub].firstseg]
                .linedef].sidenum[segs[subsectors[sub].firstseg].side]].sector];
            fixed_t z = sec->floorheight + VIEWHEIGHT;

            frame_reset(warm);
            w_stream_reset_stats();
            R_RenderPlayerView(x, y, z, (angle_t)(a * (ANG45)));

            frames++;
            total_reads += g_reads;

            if (g_compulsory > worst_compulsory)
                worst_compulsory = g_compulsory;

            tot_fetch += w_stream_fetches();

            if (w_stream_fetches() > worst_fetch)
                worst_fetch = w_stream_fetches();

            tot_cols += g_col_calls;
            tot_spans += g_span_calls;
            tot_col_bytes += g_col_bytes;
            tot_src += g_distinct_src;

            if (g_col_calls > worst_cols) worst_cols = g_col_calls;
            if (g_distinct_src > worst_src) worst_src = g_distinct_src;

            for (int c = 0; c < g_cache_count; c++)
            {
                acc[c].misses += g_caches[c].misses;

                if (g_caches[c].misses > acc[c].worst)
                    acc[c].worst = g_caches[c].misses;
            }
        }
    }

    printf("real cache: %.1f fetches per frame, %u worst (COLUMN_SHIFT %d)\n",
           (double)tot_fetch / frames, worst_fetch, COLUMN_SHIFT);
    printf("            %.2f ms per frame at 15.6 MHz, %.2f ms worst\n\n",
           (double)tot_fetch / frames * 132 * 8 / SPI_HZ * 1000.0,
           worst_fetch * 132 * 8 / SPI_HZ * 1000.0);

    printf("%d frames, %.0f texture reads per frame\n",
           frames, (double)total_reads / frames);
    printf("per frame: %.0f column draws (%.1f KB of column bytes), "
           "%.0f span draws, %.0f distinct sources\n",
           (double)tot_cols / frames, (double)tot_col_bytes / frames / 1024.0,
           (double)tot_spans / frames, (double)tot_src / frames);
    printf("worst frame: %u columns, %u distinct sources\n",
           worst_cols, worst_src);
    printf("distinct 64 B lines in the worst frame: %u (%.1f KB)\n\n",
           worst_compulsory, worst_compulsory * 64 / 1024.0);

    printf("%-22s %10s %10s %9s %9s\n",
           "cache", "avg miss", "worst", "avg ms", "worst ms");

    for (int c = 0; c < g_cache_count; c++)
    {
        cache_t *k = &g_caches[c];
        double avg = (double)acc[c].misses / frames;
        // One command plus a 24 bit address plus the line, in bits
        double bits = (4 + k->line) * 8.0;

        printf("%-22s %10.0f %10llu %9.1f %9.1f\n", k->name, avg,
               (unsigned long long)acc[c].worst,
               avg * bits / SPI_HZ * 1000.0,
               acc[c].worst * bits / SPI_HZ * 1000.0);
    }

    printf("\nsame, with Fast Read at %.1f MHz:\n", SPI_HZ_FAST / 1e6);

    for (int c = 0; c < g_cache_count; c++)
    {
        cache_t *k = &g_caches[c];
        double avg = (double)acc[c].misses / frames;
        double bits = (5 + k->line) * 8.0;      // one dummy byte more

        printf("%-22s %9.1f ms avg  %9.1f ms worst\n", k->name,
               avg * bits / SPI_HZ_FAST * 1000.0,
               acc[c].worst * bits / SPI_HZ_FAST * 1000.0);
    }

    printf("\na frame is %.0f ms at the rate the device draws now\n", FRAME_MS);
    (void)worst_ms;

    return 0;
}
