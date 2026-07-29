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
 * Visplanes - floors and ceilings.
 *
 * Walls are drawn as vertical columns, but a flat is only cheap to draw as
 * horizontal spans, so the two cannot be interleaved. Instead the wall pass
 * records, per screen column, which rows belong to which floor or ceiling; this
 * pass then walks those records and cuts them into spans. Planes that share a
 * height, texture and light level are merged, which is why a corridor floor
 * costs one plane and not one per subsector.
 */

/*- Includes ----------------------------------------------------------------*/
#include <string.h>
#include "doom.h"

/*- Variables ---------------------------------------------------------------*/
visplane_t *floorplane;
visplane_t *ceilingplane;

static visplane_t *lastvisplane;
int16_t *lastopening;

static fixed_t planeheight;
static const uint8_t *planezlight;
static fixed_t basexscale, baseyscale;

r_stats_t r_stats;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void R_ClearPlanes(void)
{
    angle_t angle;

    for (int i = 0; i < SCREENWIDTH; i++)
    {
        dm->floorclip[i] = SCREENHEIGHT;
        dm->ceilingclip[i] = -1;
    }

    lastvisplane = dm->visplanes;
    lastopening = dm->openings;

    memset(dm->cachedheight, 0, sizeof(dm->cachedheight));

    // The texture step along a screen row, for a plane one unit below the eye.
    // Every plane scales this by its own distance.
    angle = (viewangle - ANG90) >> ANGLETOFINESHIFT;
    basexscale = FixedDiv(finecosine(angle), centerxfrac);
    baseyscale = -FixedDiv(finesine(angle), centerxfrac);
}

//-----------------------------------------------------------------------------
visplane_t *R_FindPlane(fixed_t height, int picnum, int lightlevel)
{
    visplane_t *check;

    // Every sky ceiling is the same plane: the sky does not have a height and
    // is not lit
    if (SKY_FLAT == picnum)
    {
        height = 0;
        lightlevel = 0;
    }

    for (check = dm->visplanes; check < lastvisplane; check++)
    {
        if (height == check->height && picnum == check->picnum &&
            lightlevel == check->lightlevel)
            return check;
    }

    // Out of planes: reuse the last one rather than scribble past the array.
    // The frame is wrong in a corner, which beats a fault.
    if (lastvisplane - dm->visplanes == MAXVISPLANES)
        return lastvisplane - 1;

    check = lastvisplane++;

    if (lastvisplane - dm->visplanes > r_stats.visplanes)
        r_stats.visplanes = lastvisplane - dm->visplanes;

    check->height = height;
    check->picnum = picnum;
    check->lightlevel = lightlevel;
    check->minx = SCREENWIDTH;
    check->maxx = -1;

    // The original gets zeroed guards for free from .bss; this block lives in
    // raw SRAM, and a non-zero bottom guard makes R_MakeSpans emit rows that
    // are not on the screen
    check->topbuf[0] = check->topbuf[SCREENWIDTH + 1] = 0;
    check->bottombuf[0] = check->bottombuf[SCREENWIDTH + 1] = 0;

    memset(PL_TOP(check), 0xff, SCREENWIDTH);

    return check;
}

//-----------------------------------------------------------------------------
// Can this plane take the column range [start, stop], or does the range overlap
// rows it has already claimed? If it overlaps, the plane has to be split.
visplane_t *R_CheckPlane(visplane_t *pl, int start, int stop)
{
    int intrl, intrh, unionl, unionh, x;

    if (start < pl->minx)
    {
        intrl = pl->minx;
        unionl = start;
    }
    else
    {
        unionl = pl->minx;
        intrl = start;
    }

    if (stop > pl->maxx)
    {
        intrh = pl->maxx;
        unionh = stop;
    }
    else
    {
        unionh = pl->maxx;
        intrh = stop;
    }

    for (x = intrl; x <= intrh; x++)
    {
        if (PL_TOP(pl)[x] != 0xff)
            break;
    }

    if (x > intrh)
    {
        pl->minx = unionl;
        pl->maxx = unionh;

        return pl;
    }

    if (lastvisplane - dm->visplanes == MAXVISPLANES)
        return pl;

    lastvisplane->height = pl->height;
    lastvisplane->picnum = pl->picnum;
    lastvisplane->lightlevel = pl->lightlevel;

    pl = lastvisplane++;

    if (lastvisplane - dm->visplanes > r_stats.visplanes)
        r_stats.visplanes = lastvisplane - dm->visplanes;

    pl->minx = start;
    pl->maxx = stop;
    pl->topbuf[0] = pl->topbuf[SCREENWIDTH + 1] = 0;
    pl->bottombuf[0] = pl->bottombuf[SCREENWIDTH + 1] = 0;

    memset(PL_TOP(pl), 0xff, SCREENWIDTH);

    return pl;
}

//-----------------------------------------------------------------------------
// One row of a plane. The distance to a flat at a given screen row is constant
// across the row, so the texture step is too - that is the whole trick.
static void R_MapPlane(int y, int x1, int x2)
{
    angle_t angle;
    fixed_t distance, length;
    unsigned index;

    if (planeheight != dm->cachedheight[y])
    {
        dm->cachedheight[y] = planeheight;
        distance = dm->cacheddistance[y] = FixedMul(planeheight, dt_yslope[y]);
        ds_xstep = dm->cachedxstep[y] = FixedMul(distance, basexscale);
        ds_ystep = dm->cachedystep[y] = FixedMul(distance, baseyscale);
    }
    else
    {
        distance = dm->cacheddistance[y];
        ds_xstep = dm->cachedxstep[y];
        ds_ystep = dm->cachedystep[y];
    }

    length = FixedMul(distance, dt_distscale[x1]);
    angle = (viewangle + dt_xtoviewangle[x1]) >> ANGLETOFINESHIFT;

    ds_xfrac = viewx + FixedMul(finecosine(angle), length);
    ds_yfrac = -viewy - FixedMul(finesine(angle), length);

    index = distance >> LIGHTZSHIFT;

    if (index >= MAXLIGHTZ)
        index = MAXLIGHTZ - 1;

    ds_colormap = dt_colormap + (planezlight[index] << 8);

    ds_y = y;
    ds_x1 = x1;
    ds_x2 = x2;

    R_DrawSpan();
}

//-----------------------------------------------------------------------------
// Turn the difference between two adjacent columns' row ranges into spans:
// rows that end here are emitted, rows that start here are recorded
static void R_MakeSpans(int x, int t1, int b1, int t2, int b2)
{
    while (t1 < t2 && t1 <= b1)
    {
        R_MapPlane(t1, dm->spanstart[t1], x - 1);
        t1++;
    }

    while (b1 > b2 && b1 >= t1)
    {
        R_MapPlane(b1, dm->spanstart[b1], x - 1);
        b1--;
    }

    while (t2 < t1 && t2 <= b2)
    {
        dm->spanstart[t2] = x;
        t2++;
    }

    while (b2 > b1 && b2 >= t2)
    {
        dm->spanstart[b2] = x;
        b2--;
    }
}

//-----------------------------------------------------------------------------
static void R_DrawSky(visplane_t *pl)
{
    const texture_t *tex = &textures[skytexture];
    int shift = tex->shift;

    // The sky does not move with the player and is never shaded
    dc_colormap = dt_colormap;
    dc_iscale = FRACUNIT >> shift;
    dc_texturemid = (100 * FRACUNIT) >> shift;
    dc_texheight = tex->sheight;

    for (int x = pl->minx; x <= pl->maxx; x++)
    {
        dc_yl = PL_TOP(pl)[x];
        dc_yh = PL_BOTTOM(pl)[x];

        if (dc_yl <= dc_yh)
        {
            // ANGLETOSKYSHIFT maps a full turn onto four repeats of the texture
            unsigned angle = (viewangle + dt_xtoviewangle[x]) >> 22;

            dc_x = x;
            dc_source = w_column(
                texturecols[tex->firstcol + ((angle >> shift) & tex->widthmask)]);

            R_DrawColumn();
        }
    }
}

//-----------------------------------------------------------------------------
void R_DrawPlanes(void)
{
    visplane_t *pl;
    int light, stop;

    for (pl = dm->visplanes; pl < lastvisplane; pl++)
    {
        if (pl->minx > pl->maxx)
            continue;

        if (SKY_FLAT == pl->picnum)
        {
            R_DrawSky(pl);
            continue;
        }

        ds_source = flatdata + (pl->picnum << (12 - 2 * flatshift));

        planeheight = pl->height - viewz;

        if (planeheight < 0)
            planeheight = -planeheight;

        light = (pl->lightlevel >> LIGHTSEGSHIFT) + extralight;

        if (light >= LIGHTLEVELS)
            light = LIGHTLEVELS - 1;

        if (light < 0)
            light = 0;

        planezlight = dt_zlight + light * MAXLIGHTZ;

        // Sentinels so the sweep below closes the plane off at both ends
        PL_TOP(pl)[pl->maxx + 1] = 0xff;
        PL_TOP(pl)[pl->minx - 1] = 0xff;

        stop = pl->maxx + 1;

        for (int x = pl->minx; x <= stop; x++)
            R_MakeSpans(x, PL_TOP(pl)[x - 1], PL_BOTTOM(pl)[x - 1],
                        PL_TOP(pl)[x], PL_BOTTOM(pl)[x]);
    }
}
