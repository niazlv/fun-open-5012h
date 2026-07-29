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
const uint8_t *texturedata;
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
bool doom_assets_init(const void *blob)
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
    texturedata = require("TEXDATA");
    flatdata    = require("FLATS");

    if (!textures || !texturecols || !texturedata || !flatdata)
        return false;

    // TEXDIR leads with the count, the entries follow
    numtextures = *(const uint32_t *)textures;
    textures = (const texture_t *)((const uint32_t *)textures + 1);

    numflats = g_info->numflats;
    flatshift = g_info->flatshift;
    skytexture = g_info->skytexture;

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
