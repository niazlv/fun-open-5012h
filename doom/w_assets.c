/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Asset pack access.
 *
 * tools/wadpack.py builds a blob of named sections; everything in it is already
 * in the form the engine consumes, so "loading" a level is binding a handful of
 * pointers into flash. The one exception is sectors, which the game mutates
 * (doors, lifts, lights), so those are expanded into a small RAM shadow.
 */

/*- Includes ----------------------------------------------------------------*/
#include <string.h>
#include "doom.h"

/*- Definitions -------------------------------------------------------------*/
#define BLOB_MAGIC          0x314b5044  // "DPK1"
#define NAME_LEN            12

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t count;
    uint32_t dir_offset;
} blob_header_t;

typedef struct
{
    char name[NAME_LEN];
    uint32_t offset;
    uint32_t size;
} blob_entry_t;

// Mirrors the LEVELINFO layout in tools/wadpack.py
typedef struct
{
    char name[8];
    uint16_t flatshift, skytexture;
    uint16_t numvertexes, numsegs, numsubsectors, numnodes;
    uint16_t numsides, numlines, numsectors, numthings;
    uint16_t numflats, numtextures;
    int16_t width, height, centerx, centery;
    int32_t focallength;
} level_info_t;

/*- Local prototypes --------------------------------------------------------*/
static bool blob_read(void *ctx, uint32_t offset, uint8_t *dst, uint32_t size);

/*- Variables ---------------------------------------------------------------*/
doom_mem_t *dm;

const uint16_t *dt_palette;
const uint8_t *dt_colormap;
const int32_t *dt_sinq;
const int32_t *dt_tanh;
const uint32_t *dt_tantoangle;
const int16_t *dt_viewangletox;
const uint32_t *dt_xtoviewangle;
const int32_t *dt_yslope;
const int32_t *dt_distscale;
const uint8_t *dt_scalelight;
const uint8_t *dt_zlight;

const mapvertex_t *vertexes;
const mapseg_t *segs;
const mapsubsector_t *subsectors;
const mapnode_t *nodes;
const maplinedef_t *lines;
const mapside_t *sides;
const mapthing_t *things;
const int16_t *blockmap;
sector_t *sectors;

int numvertexes, numsegs, numsubsectors, numnodes;
int numlines, numsides, numsectors, numthings;

const texture_t *textures;
const uint32_t *texturecols;
const uint8_t *flatdata;
int numtextures, numflats;
int flatshift;
int skytexture;

static const uint8_t *g_blob;
static const blob_entry_t *g_dir;
static int g_dir_count;
static const level_info_t *g_info;

static uint8_t *g_zone_ptr;
static int g_zone_left;

// Why the pack was rejected, for the screen that reports it. A blob that fails
// here is nearly always a flashing problem rather than a bad pack, and the
// difference is visible in the first word: 0xffffffff is unprogrammed flash,
// zero is the empty stand-in the Makefile links when no pack was built.
static const char *g_error;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void *zone_alloc(int size)
{
    void *p = g_zone_ptr;

    size = (size + 3) & ~3;

    if (size > g_zone_left)
        return NULL;

    g_zone_ptr += size;
    g_zone_left -= size;

    return p;
}

//-----------------------------------------------------------------------------
const void *doom_asset_find(const char *name, int *size)
{
    for (int i = 0; i < g_dir_count; i++)
    {
        if (0 == strncmp(g_dir[i].name, name, NAME_LEN))
        {
            if (size)
                *size = g_dir[i].size;

            return g_blob + g_dir[i].offset;
        }
    }

    if (size)
        *size = 0;

    return NULL;
}

//-----------------------------------------------------------------------------
static const void *require(const char *name)
{
    const void *p = doom_asset_find(name, NULL);

    if (NULL == p && NULL == g_error)
        g_error = name;

    return p;
}

//-----------------------------------------------------------------------------
// Where the pack starts. Only a measurement harness wants this - it is how a
// pointer handed out by doom_asset_find() becomes an offset that a cache
// model can index.
const uint8_t *doom_assets_blob(void)
{
    return g_blob;
}

//-----------------------------------------------------------------------------
const char *doom_assets_error(void)
{
    return g_error;
}

//-----------------------------------------------------------------------------
bool doom_assets_init(const void *blob, uint8_t *cache)
{
    const blob_header_t *hdr = (const blob_header_t *)blob;

    g_error = NULL;

    if (NULL == blob)
    {
        g_error = "null";
        return false;
    }

    if (BLOB_MAGIC != hdr->magic)
    {
        g_error = (0xffffffff == hdr->magic) ? "flash not written"
                : (0 == hdr->magic) ? "no pack in this build" : "bad magic";
        return false;
    }

    if (1 != hdr->version)
    {
        g_error = "version";
        return false;
    }

    g_blob = (const uint8_t *)blob;
    g_dir = (const blob_entry_t *)(g_blob + hdr->dir_offset);
    g_dir_count = hdr->count;

    dt_palette      = require("PALETTE");
    dt_colormap     = require("COLORMAP");
    dt_sinq         = require("SINQ");
    dt_tanh         = require("TANH");
    dt_tantoangle   = require("TANTOANGLE");
    dt_viewangletox = require("VIEWANGLETOX");
    dt_xtoviewangle = require("XTOVIEWANGLE");
    dt_yslope       = require("YSLOPE");
    dt_distscale    = require("DISTSCALE");
    dt_scalelight   = require("SCALELIGHT");
    dt_zlight       = require("ZLIGHT");

    g_info = require("LEVELINFO");

    if (!dt_palette || !dt_colormap || !dt_sinq || !dt_tanh || !dt_tantoangle ||
        !dt_viewangletox || !dt_xtoviewangle || !dt_yslope || !dt_distscale ||
        !dt_scalelight || !dt_zlight || !g_info)
        return false;

    // The tables are generated for one screen size; a mismatch would put every
    // projection off by a scale factor rather than fail visibly
    if (SCREENWIDTH != g_info->width || SCREENHEIGHT != g_info->height)
    {
        g_error = "screen size";
        return false;
    }

    textures    = require("TEXDIR");
    texturecols = require("TEXCOLS");

    if (!textures || !texturecols)
        return false;

    // TEXDATA and FLATS are not required here: in a pack built with --split
    // they are in the other file, and the caller binds them through
    // doom_assets_stream(). A whole pack still works though - if they are in
    // this blob, bind them straight away against a reader that copies out of
    // it, so there is exactly one path through the renderer either way.
    {
        const blob_entry_t *e;
        const void *tex = doom_asset_find("TEXDATA", NULL);

        flatdata = doom_asset_find("FLATS", NULL);

        // Only when the caller supplied somewhere to cache into: on the
        // device the pack is always split, and 8 KB of .bss for a path that
        // never runs there is 8 KB the instrument could have had.
        if (tex && flatdata && cache)
        {
            for (int i = 0; i < g_dir_count; i++)
            {
                e = &g_dir[i];

                if (0 == strncmp(e->name, "TEXDATA", NAME_LEN))
                {
                    w_stream_init(blob_read, NULL, cache, e->offset, e->size);
                    break;
                }
            }
        }
    }

    // TEXDIR leads with the count, the entries follow
    numtextures = *(const uint32_t *)textures;
    textures = (const texture_t *)((const uint32_t *)textures + 1);

    numflats = g_info->numflats;
    flatshift = g_info->flatshift;
    skytexture = g_info->skytexture;

    return true;
}

//-----------------------------------------------------------------------------
// Reader for a pack that is already in memory. The cache in front of it is
// pure overhead here, and deliberately so: one path is easier to be sure of
// than two, and 8 KB of it costs nothing on a host or in a build where the
// whole pack is linked in.
//-----------------------------------------------------------------------------
static bool blob_read(void *ctx, uint32_t offset, uint8_t *dst, uint32_t size)
{
    (void)ctx;

    if (NULL == g_blob)
        return false;

    memcpy(dst, g_blob + offset, size);

    return true;
}

//-----------------------------------------------------------------------------
// The half of the pack that lives somewhere unaddressable.
//
// The texture columns stay there and are fetched one at a time; the flats do
// not, because R_DrawSpan indexes a flat by a jumping offset rather than
// walking it, so a partly resident flat is not a thing that can be drawn. All
// of them together are 21 KB in a --scale 2 pack, which is small enough to
// simply hold.
//
// `blob` is the streamed pack's own header and directory, which the caller has
// already read into memory - it is 100 bytes and asking the bus for it twice
// would be sillier than passing it in.
bool doom_assets_stream(const void *dir, w_stream_read_t read, void *ctx,
    uint8_t *cache, uint8_t *flats, uint32_t flats_size)
{
    const blob_header_t *hdr = (const blob_header_t *)dir;
    const blob_entry_t *ents;
    const blob_entry_t *tex = NULL, *flat = NULL;

    if (NULL == dir || BLOB_MAGIC != hdr->magic || 1 != hdr->version)
    {
        g_error = "stream pack";
        return false;
    }

    ents = (const blob_entry_t *)((const uint8_t *)dir + hdr->dir_offset);

    for (unsigned i = 0; i < hdr->count; i++)
    {
        if (0 == strncmp(ents[i].name, "TEXDATA", NAME_LEN))
            tex = &ents[i];
        else if (0 == strncmp(ents[i].name, "FLATS", NAME_LEN))
            flat = &ents[i];
    }

    if (NULL == tex || NULL == flat)
    {
        g_error = "stream lumps";
        return false;
    }

    if (flat->size > flats_size)
    {
        g_error = "flats too big";
        return false;
    }

    if (!read(ctx, flat->offset, flats, flat->size))
    {
        g_error = "flats read";
        return false;
    }

    flatdata = flats;

    w_stream_init(read, ctx, cache, tex->offset, tex->size);

    return true;
}

//-----------------------------------------------------------------------------
bool doom_level_load(void)
{
    const mapsector_t *raw;

    vertexes   = require("VERTEXES");
    segs       = require("SEGS");
    subsectors = require("SSECTORS");
    nodes      = require("NODES");
    lines      = require("LINEDEFS");
    sides      = require("SIDEDEFS");
    things     = require("THINGS");
    blockmap   = require("BLOCKMAP");
    raw        = require("SECTORS");

    if (!vertexes || !segs || !subsectors || !nodes || !lines || !sides ||
        !things || !blockmap || !raw)
        return false;

    numvertexes   = g_info->numvertexes;
    numsegs       = g_info->numsegs;
    numsubsectors = g_info->numsubsectors;
    numnodes      = g_info->numnodes;
    numlines      = g_info->numlines;
    numsides      = g_info->numsides;
    numsectors    = g_info->numsectors;
    numthings     = g_info->numthings;

    g_zone_ptr = dm->zone;
    g_zone_left = sizeof(dm->zone);

    sectors = zone_alloc(numsectors * sizeof(sector_t));

    if (NULL == sectors)
    {
        g_error = "zone too small";
        return false;
    }

    for (int i = 0; i < numsectors; i++)
    {
        sectors[i].floorheight   = mapfix(raw[i].floorheight);
        sectors[i].ceilingheight = mapfix(raw[i].ceilingheight);
        sectors[i].floorpic      = raw[i].floorpic;
        sectors[i].ceilingpic    = raw[i].ceilingpic;
        sectors[i].lightlevel    = raw[i].lightlevel;
        sectors[i].special       = raw[i].special;
        sectors[i].tag           = raw[i].tag;
    }

    return true;
}

//-----------------------------------------------------------------------------
const char *doom_level_name(void)
{
    static char name[9];

    memcpy(name, g_info->name, 8);
    name[8] = 0;

    return name;
}
