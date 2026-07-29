/*
 * Copyright (C) 1993-1996 Id Software, Inc.
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Derived from the DOOM source release (linuxdoom-1.10), published by
 * id Software under the GNU General Public License. This program is free
 * software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any
 * later version. It is distributed WITHOUT ANY WARRANTY; see doom/COPYING.
 *
 * A real DOOM renderer for the GD32F407VE.
 *
 * The algorithms are id's: binary space partition traversal, the solidsegs
 * clip list, R_StoreWallRange's perspective texture mapping, visplanes. What
 * differs is where the data lives. The original builds its tables, composites
 * its textures and unpacks its level into a multi-megabyte zone heap; here all
 * of that is done on the host by tools/wadpack.py and read straight out of
 * flash through const pointers, so the only RAM the renderer needs is the
 * frame buffer and the per-frame clipping state.
 *
 * Angles are the original binary angles (a full turn is 2^32) and all geometry
 * is 16.16 fixed point, so the fixed-point identities the renderer relies on
 * hold exactly as they do in the original.
 */

#ifndef _DOOM_H_
#define _DOOM_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*- Definitions -------------------------------------------------------------*/
#define SCREENWIDTH         320
#define SCREENHEIGHT        200

#define FRACBITS            16
#define FRACUNIT            (1 << FRACBITS)

#define ANGLETOFINESHIFT    19
#define FINEANGLES          8192
#define FINEMASK            (FINEANGLES - 1)

#define ANG45               0x20000000u
#define ANG90               0x40000000u
#define ANG180              0x80000000u
#define ANG270              0xc0000000u

#define SLOPERANGE          2048
#define SLOPEBITS           11
#define DBITS               (FRACBITS - SLOPEBITS)

#define NUMCOLORMAPS        32
#define LIGHTLEVELS         16
#define LIGHTSEGSHIFT       4
#define MAXLIGHTSCALE       48
#define LIGHTSCALESHIFT     12
#define MAXLIGHTZ           128
#define LIGHTZSHIFT         20

// Sized for this screen rather than for a PC: vanilla keeps 128 visplanes and
// 64 openings per column, which alone would be more memory than this part has.
// The whole block has to fit below CAPTURE_SPARE_RAM, which is 120 KB, and the
// figures below were measured over the level rather than guessed - see
// tests/doom_host.c, which reports the high-water mark for each of them.
#ifndef MAXVISPLANES
#define MAXVISPLANES        52
#endif
#define MAXDRAWSEGS         112
#define MAXOPENINGS         (SCREENWIDTH * 8)
#define MAXVISSPRITES       96
#define MAXSEGS             24      // solidsegs, one per visible wall run

// linedef flags
#define ML_BLOCKING         1
#define ML_BLOCKMONSTERS    2
#define ML_TWOSIDED         4
#define ML_DONTPEGTOP       8
#define ML_DONTPEGBOTTOM    16
#define ML_SECRET           32
#define ML_SOUNDBLOCK       64
#define ML_DONTDRAW         128
#define ML_MAPPED           256

#define NO_TEXTURE          0xffff
#define SKY_FLAT            0xffff

#define ARRAY_COUNT(a)      ((int)(sizeof(a) / sizeof((a)[0])))

/*- Types -------------------------------------------------------------------*/
typedef int32_t fixed_t;
typedef uint32_t angle_t;

// ---------------------------------------------------------------------------
// Level structures. These four are the WAD's own layout, byte for byte: the
// WAD is little-endian and so is the Cortex-M4, so the packer copies them
// across untouched and the renderer reads them in place.
// ---------------------------------------------------------------------------
typedef struct
{
    int16_t x, y;
} mapvertex_t;

typedef struct
{
    uint16_t v1, v2;
    int16_t angle;              // upper 16 bits of the binary angle
    uint16_t linedef;
    int16_t side;               // 0 front, 1 back
    int16_t offset;
} mapseg_t;

typedef struct
{
    uint16_t numsegs;
    uint16_t firstseg;
} mapsubsector_t;

typedef struct
{
    int16_t x, y, dx, dy;       // partition line
    int16_t bbox[2][4];         // [child][top, bottom, left, right]
    uint16_t children[2];       // subsector if bit 15 is set
} mapnode_t;

typedef struct
{
    uint16_t v1, v2;
    uint16_t flags;
    int16_t special, tag;
    uint16_t sidenum[2];        // 0xffff when absent
} maplinedef_t;

// The two the packer rewrites, to turn 8-byte texture names into indices
typedef struct
{
    int16_t xoffset, yoffset;
    uint16_t toptex, bottomtex, midtex;
    uint16_t sector;
} mapside_t;

typedef struct
{
    int16_t floorheight, ceilingheight;
    uint16_t floorpic, ceilingpic;
    int16_t lightlevel, special, tag;
} mapsector_t;

typedef struct
{
    int16_t x, y;
    int16_t angle;
    int16_t type;
    int16_t options;
} mapthing_t;

// ---------------------------------------------------------------------------
// Sectors are the one thing the game mutates (doors, lifts, light), so they get
// a RAM shadow. Heights are promoted to fixed point here, which is the form
// every consumer wants.
// ---------------------------------------------------------------------------
typedef struct
{
    fixed_t floorheight, ceilingheight;
    uint16_t floorpic, ceilingpic;
    int16_t lightlevel;
    int16_t special, tag;
} sector_t;

typedef struct
{
    uint16_t width, height;     // logical size, what the mapping uses
    uint8_t shift;              // stored copy is this many halvings smaller
    uint8_t masked;             // columns are posts, not solid runs
    uint16_t swidth, sheight;   // stored size
    uint16_t widthmask;
    uint32_t firstcol;          // index into the column-offset table
} texture_t;

// R_DrawPlanes closes a plane off by writing a sentinel one column past each
// end of its range, so for a plane that reaches a screen edge that write lands
// outside the columns. The original relies on struct padding absorbing it; here
// the guard cells are explicit and the arrays are addressed through PL_TOP and
// PL_BOTTOM, which put column 0 at index 1.
typedef struct
{
    fixed_t height;
    uint16_t picnum;
    int16_t lightlevel;
    int16_t minx, maxx;
    uint8_t topbuf[SCREENWIDTH + 2];
    uint8_t bottombuf[SCREENWIDTH + 2];
} visplane_t;

#define PL_TOP(pl)          ((pl)->topbuf + 1)
#define PL_BOTTOM(pl)       ((pl)->bottombuf + 1)

typedef struct
{
    int16_t x1, x2;
    fixed_t scale1, scale2, scalestep;
    int16_t silhouette;         // 0, SIL_BOTTOM, SIL_TOP, SIL_BOTH
    fixed_t bsilheight, tsilheight;
    const int16_t *sprtopclip;
    const int16_t *sprbottomclip;
    int16_t *maskedtexturecol;
    uint16_t curline;           // seg index, for the masked pass
} drawseg_t;

#define SIL_NONE            0
#define SIL_BOTTOM          1
#define SIL_TOP             2
#define SIL_BOTH            3

// The clip list carries sentinels well outside the screen, so it stays 32-bit
typedef struct
{
    int32_t first, last;
} cliprange_t;

// ---------------------------------------------------------------------------
// Everything the renderer needs to write to, in one block so it can be placed
// in the capture SRAM on the target and malloc'd on the host.
// ---------------------------------------------------------------------------
typedef struct
{
    uint8_t screen[SCREENWIDTH * SCREENHEIGHT];

    visplane_t visplanes[MAXVISPLANES];
    drawseg_t drawsegs[MAXDRAWSEGS];
    int16_t openings[MAXOPENINGS];

    int16_t floorclip[SCREENWIDTH];
    int16_t ceilingclip[SCREENWIDTH];

    // Constant clip arrays for walls that hide everything behind them
    int16_t screenheightarray[SCREENWIDTH];
    int16_t negonearray[SCREENWIDTH];

    // Per-column state handed from R_StoreWallRange to R_DrawPlanes
    int16_t spanstart[SCREENHEIGHT];

    // A visplane is walked one screen row at a time, and every row of a given
    // plane shares its distance and texture step - so they are worked out once
    // per row and reused for the whole plane
    fixed_t cachedheight[SCREENHEIGHT];
    fixed_t cacheddistance[SCREENHEIGHT];
    fixed_t cachedxstep[SCREENHEIGHT];
    fixed_t cachedystep[SCREENHEIGHT];

    cliprange_t solidsegs[MAXSEGS];

    // Level state that has to be writable: the sector shadow, and later the
    // thinkers. Everything else is read in place out of flash.
    uint8_t zone[5120];
} doom_mem_t;

// How close a frame came to the limits above, so they can be checked against
// real views instead of assumed
typedef struct
{
    int visplanes, drawsegs, openings, solidsegs, maskedcols;
} r_stats_t;

extern r_stats_t r_stats;

// The base of the asset pack, for a harness that measures access patterns
const uint8_t *doom_assets_blob(void);

/*
 * Texture reads, counted.
 *
 * The two loops below are where the renderer touches the pack once per screen
 * pixel, and they are the only reads that would have to come over a bus if the
 * pack ever lived on the SPI part rather than in the address space. Built for
 * the device this expands to nothing; tests/doom_cache.c defines DOOM_TRACE
 * and models a cache behind it.
 */
#ifdef DOOM_TRACE
void doom_trace_read(const void *addr);
void doom_trace_fetch(const void *addr, int bytes);
#define DOOM_TRACE_READ(p)      doom_trace_read(p)
#define DOOM_TRACE_FETCH(p, n)  doom_trace_fetch(p, n)
#else
#define DOOM_TRACE_READ(p)      ((void)0)
#define DOOM_TRACE_FETCH(p, n)  ((void)0)
#endif

/*- Variables ---------------------------------------------------------------*/
extern doom_mem_t *dm;

// Asset pointers, all into flash
extern const uint16_t *dt_palette;      // 256 entries, already RGB565
extern const uint8_t *dt_colormap;      // 32 * 256
extern const int32_t *dt_sinq;          // quarter sine wave, 2048
extern const int32_t *dt_tanh;          // positive half of finetangent, 2048
extern const uint32_t *dt_tantoangle;   // 2049
extern const int16_t *dt_viewangletox;  // 4096
extern const uint32_t *dt_xtoviewangle; // SCREENWIDTH + 1
extern const int32_t *dt_yslope;        // SCREENHEIGHT
extern const int32_t *dt_distscale;     // SCREENWIDTH
extern const uint8_t *dt_scalelight;    // LIGHTLEVELS * MAXLIGHTSCALE
extern const uint8_t *dt_zlight;        // LIGHTLEVELS * MAXLIGHTZ

extern const mapvertex_t *vertexes;
extern const mapseg_t *segs;
extern const mapsubsector_t *subsectors;
extern const mapnode_t *nodes;
extern const maplinedef_t *lines;
extern const mapside_t *sides;
extern const mapthing_t *things;
extern const int16_t *blockmap;         // header then offsets then lists
extern sector_t *sectors;

extern int numvertexes, numsegs, numsubsectors, numnodes;
extern int numlines, numsides, numsectors, numthings;

extern const texture_t *textures;
extern const uint32_t *texturecols;     // offsets into texturedata
extern const uint8_t *texturedata;
extern const uint8_t *flatdata;
extern int numtextures, numflats;
extern int flatshift;                   // flats are 64 >> flatshift wide
extern int skytexture;

extern fixed_t viewx, viewy, viewz;
extern angle_t viewangle;
extern fixed_t viewcos, viewsin;
extern int extralight;
extern int centerx, centery;
extern fixed_t centerxfrac, centeryfrac, projection;
extern angle_t clipangle;
extern int validcount;

// Renderer internals, shared between the BSP walk and the wall renderer the
// same way the original shares them
extern const mapseg_t *curline;
extern const mapside_t *sidedef;
extern const maplinedef_t *linedef;
extern const sector_t *frontsector;
extern const sector_t *backsector;
extern angle_t rw_angle1;
extern fixed_t rw_distance;
extern angle_t rw_normalangle;
extern drawseg_t *ds_p;
extern visplane_t *floorplane;
extern visplane_t *ceilingplane;

/*- Prototypes --------------------------------------------------------------*/
// w_assets.c
bool doom_assets_init(const void *blob);
const char *doom_assets_error(void);
const void *doom_asset_find(const char *name, int *size);
bool doom_level_load(void);
const char *doom_level_name(void);

// r_main.c
void R_Init(void);
void R_RenderPlayerView(fixed_t x, fixed_t y, fixed_t z, angle_t angle);
angle_t R_PointToAngle(fixed_t x, fixed_t y);
angle_t R_PointToAngle2(fixed_t x1, fixed_t y1, fixed_t x2, fixed_t y2);
fixed_t R_PointToDist(fixed_t x, fixed_t y);
fixed_t R_ScaleFromGlobalAngle(angle_t visangle);
int R_PointOnSide(fixed_t x, fixed_t y, const mapnode_t *node);
int R_PointOnSegSide(fixed_t x, fixed_t y, const mapseg_t *line);
int R_PointInSubsector(fixed_t x, fixed_t y);

// r_bsp.c
void R_ClearClipSegs(void);
void R_ClearDrawSegs(void);
void R_RenderBSPNode(int bspnum);

// r_segs.c
void R_InitSegs(void);
void R_StoreWallRange(int start, int stop);
void R_RenderMaskedSegRange(drawseg_t *ds, int x1, int x2);
void R_DrawMasked(void);

// r_plane.c
void R_ClearPlanes(void);
visplane_t *R_FindPlane(fixed_t height, int picnum, int lightlevel);
visplane_t *R_CheckPlane(visplane_t *pl, int start, int stop);
void R_DrawPlanes(void);

// r_draw.c
void R_InitDrawFlats(int shift);
void R_DrawColumn(void);
void R_DrawMaskedColumn(const uint8_t *post);
void R_DrawSpan(void);

extern fixed_t sprtopscreen, spryscale;
extern const int16_t *mfloorclip, *mceilingclip;

extern int dc_x, dc_yl, dc_yh;
extern fixed_t dc_iscale, dc_texturemid;
extern const uint8_t *dc_source;
extern const uint8_t *dc_colormap;
extern int dc_texheight;

extern int ds_y, ds_x1, ds_x2;
extern fixed_t ds_xfrac, ds_yfrac, ds_xstep, ds_ystep;
extern const uint8_t *ds_source;
extern const uint8_t *ds_colormap;

// p_player.c
#define PLAYER_RADIUS       (16 * FRACUNIT)
#define PLAYER_HEIGHT       (56 * FRACUNIT)
#define VIEWHEIGHT          (41 * FRACUNIT)
#define MAXBOB              (16 * FRACUNIT)
#define TICRATE             35

typedef struct
{
    fixed_t x, y, z;
    fixed_t momx, momy;
    fixed_t bob;
    angle_t angle;
} player_t;

// One tic of intent, so the key mapping stays in the application and the host
// harness can drive the same simulation from a script
typedef struct
{
    int8_t forward;             // -1 back, 0, 1 forward
    int8_t side;                // -1 left, 0, 1 right
    int8_t turn;                // -1 right, 0, 1 left
    bool run;
} ticcmd_t;

extern player_t player;
extern uint32_t leveltime;

void P_SpawnPlayer(void);
void P_PlayerTic(const ticcmd_t *cmd);
fixed_t P_ViewZ(void);
void P_RenderPlayerView(void);

// p_doors.c
void P_DoorsClear(void);
void P_DoorsTic(fixed_t playerx, fixed_t playery, fixed_t playerz);
bool P_UseLines(fixed_t x, fixed_t y, angle_t angle);

// p_move.c
bool P_CheckPosition(fixed_t x, fixed_t y, fixed_t z, fixed_t radius, fixed_t height);
bool P_TryMove(fixed_t *x, fixed_t *y, fixed_t *z, fixed_t nx, fixed_t ny,
               fixed_t radius, fixed_t height);
const sector_t *P_SectorAt(fixed_t x, fixed_t y);
int P_PointOnLineSide(fixed_t x, fixed_t y, const maplinedef_t *ld);
fixed_t P_FloorHeight(fixed_t x, fixed_t y);

/*- Inline helpers ----------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Map geometry is int16 in the WAD and 16.16 everywhere in the renderer. The
// obvious shift is undefined for a negative value, and half the map has
// negative coordinates, so the promotion goes through unsigned - same
// instruction, defined result.
static inline fixed_t mapfix(int v)
{
    return (fixed_t)((uint32_t)v << FRACBITS);
}

//-----------------------------------------------------------------------------
static inline fixed_t FixedMul(fixed_t a, fixed_t b)
{
    return (fixed_t)(((int64_t)a * (int64_t)b) >> FRACBITS);
}

//-----------------------------------------------------------------------------
// Saturates the way the original does: overflow here is not an error, several
// places in the renderer lean on the clamp
static inline fixed_t FixedDiv(fixed_t a, fixed_t b)
{
    fixed_t aa = a < 0 ? -a : a;
    fixed_t ab = b < 0 ? -b : b;

    if (0 == b)
        return (a ^ b) < 0 ? INT32_MIN : INT32_MAX;

    if ((aa >> 14) >= ab)
        return (a ^ b) < 0 ? INT32_MIN : INT32_MAX;

    return (fixed_t)(((int64_t)a * FRACUNIT) / b);
}

//-----------------------------------------------------------------------------
// The quarter wave is mirrored back into the full table the original stores:
// sin(pi - x) == sin(x) and sin(x + pi) == -sin(x), and the half-step offset
// the table is generated with makes both exact rather than approximate
static inline fixed_t finesine(unsigned a)
{
    int neg;

    a &= FINEMASK;
    neg = (a >> 12) & 1;
    a &= 4095;

    if (a >= 2048)
        a = 4095 - a;

    return neg ? -dt_sinq[a] : dt_sinq[a];
}

//-----------------------------------------------------------------------------
static inline fixed_t finecosine(unsigned a)
{
    return finesine(a + 2048);
}

//-----------------------------------------------------------------------------
// finetangent[i] and finetangent[4095 - i] are exact negatives, so only the
// positive half is stored
static inline fixed_t finetangent(unsigned i)
{
    i &= 4095;

    return (i >= 2048) ? dt_tanh[i - 2048] : -dt_tanh[2047 - i];
}

//-----------------------------------------------------------------------------
static inline int SlopeDiv(unsigned num, unsigned den)
{
    unsigned ans;

    if (den < 512)
        return SLOPERANGE;

    ans = (num << 3) / (den >> 8);

    return ans <= SLOPERANGE ? (int)ans : SLOPERANGE;
}

//-----------------------------------------------------------------------------
static inline const sector_t *seg_frontsector(const mapseg_t *seg)
{
    const maplinedef_t *line = &lines[seg->linedef];

    return &sectors[sides[line->sidenum[seg->side]].sector];
}

//-----------------------------------------------------------------------------
static inline const sector_t *seg_backsector(const mapseg_t *seg)
{
    const maplinedef_t *line = &lines[seg->linedef];
    uint16_t sn;

    if (!(line->flags & ML_TWOSIDED))
        return NULL;

    sn = line->sidenum[seg->side ^ 1];

    return (NO_TEXTURE == sn) ? NULL : &sectors[sides[sn].sector];
}

//-----------------------------------------------------------------------------
static inline const mapside_t *seg_sidedef(const mapseg_t *seg)
{
    return &sides[lines[seg->linedef].sidenum[seg->side]];
}

#endif // _DOOM_H_
