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
 * Wall rendering.
 *
 * R_StoreWallRange takes a run of screen columns that a seg is known to be
 * visible in, works out the wall's perpendicular distance and the scale at both
 * ends, and then R_RenderSegLoop steps across the run drawing one column of
 * each tier - upper, middle, lower - and recording which rows above and below
 * belong to the floor and ceiling planes.
 *
 * The texture column is found by the tangent of the angle between the column
 * and the wall's normal, which is what makes the mapping a true perspective
 * one: the texture slides along the wall at exactly the rate the geometry does.
 */

/*- Includes ----------------------------------------------------------------*/
#include <string.h>
#include "doom.h"

/*- Definitions -------------------------------------------------------------*/
#define HEIGHTBITS          12
#define HEIGHTUNIT          (1 << HEIGHTBITS)
#define MAXSHORT            0x7fff

/*- Variables ---------------------------------------------------------------*/
angle_t rw_normalangle;
fixed_t rw_distance;

static bool segtextured;
static bool markfloor, markceiling;
static bool maskedtexture;
static int toptexture, bottomtexture, midtexture;

static int rw_x, rw_stopx;
static angle_t rw_centerangle;
static fixed_t rw_offset;
static fixed_t rw_scale, rw_scalestep;
static fixed_t rw_midtexturemid, rw_toptexturemid, rw_bottomtexturemid;

static int worldtop, worldbottom, worldhigh, worldlow;
static fixed_t pixhigh, pixlow, pixhighstep, pixlowstep;
static fixed_t topfrac, topstep, bottomfrac, bottomstep;

static const uint8_t *walllights;
static int16_t *maskedtexturecol;

extern int16_t *lastopening;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void R_InitSegs(void)
{
    for (int i = 0; i < SCREENWIDTH; i++)
    {
        dm->screenheightarray[i] = SCREENHEIGHT;
        dm->negonearray[i] = -1;
    }

    R_InitDrawFlats(flatshift);
}

//-----------------------------------------------------------------------------
// Point the column drawer at one texture column. The stored copy may be
// smaller than the logical texture, so the mapping constants are scaled down
// here instead of inside the inner loop.
static inline void set_column(int texnum, int col, fixed_t texturemid, fixed_t iscale)
{
    const texture_t *t = &textures[texnum];

    dc_source = w_column(texturecols[t->firstcol + ((col >> t->shift) & t->widthmask)]);
    dc_texturemid = texturemid >> t->shift;
    dc_iscale = iscale >> t->shift;
    dc_texheight = t->sheight;
}

//-----------------------------------------------------------------------------
static inline fixed_t texture_height(int texnum)
{
    return mapfix(textures[texnum].height);
}

//-----------------------------------------------------------------------------
static void R_RenderSegLoop(void)
{
    angle_t angle;
    unsigned index;
    int yl, yh, mid, top, bottom;
    fixed_t texturecolumn = 0;
    fixed_t base_iscale = 0;

    for (; rw_x < rw_stopx; rw_x++)
    {
        // Where the wall's top and bottom edges are on this column
        yl = (topfrac + HEIGHTUNIT - 1) >> HEIGHTBITS;

        if (yl < dm->ceilingclip[rw_x] + 1)
            yl = dm->ceilingclip[rw_x] + 1;

        if (markceiling)
        {
            top = dm->ceilingclip[rw_x] + 1;
            bottom = yl - 1;

            if (bottom >= dm->floorclip[rw_x])
                bottom = dm->floorclip[rw_x] - 1;

            if (top <= bottom)
            {
                PL_TOP(ceilingplane)[rw_x] = top;
                PL_BOTTOM(ceilingplane)[rw_x] = bottom;
            }
        }

        yh = bottomfrac >> HEIGHTBITS;

        if (yh >= dm->floorclip[rw_x])
            yh = dm->floorclip[rw_x] - 1;

        if (markfloor)
        {
            top = yh + 1;
            bottom = dm->floorclip[rw_x] - 1;

            if (top <= dm->ceilingclip[rw_x])
                top = dm->ceilingclip[rw_x] + 1;

            if (top <= bottom)
            {
                PL_TOP(floorplane)[rw_x] = top;
                PL_BOTTOM(floorplane)[rw_x] = bottom;
            }
        }

        if (segtextured)
        {
            // The tangent of the angle off the wall normal, scaled by the
            // distance, is the distance along the wall - a true perspective
            // mapping, one table lookup and one multiply
            angle = (rw_centerangle + dt_xtoviewangle[rw_x]) >> ANGLETOFINESHIFT;
            texturecolumn = rw_offset - FixedMul(finetangent(angle), rw_distance);
            texturecolumn >>= FRACBITS;

            index = rw_scale >> LIGHTSCALESHIFT;

            if (index >= MAXLIGHTSCALE)
                index = MAXLIGHTSCALE - 1;

            dc_colormap = dt_colormap + (walllights[index] << 8);
            dc_x = rw_x;
            base_iscale = 0xffffffffu / (unsigned)rw_scale;
        }

        if (midtexture >= 0)
        {
            // A solid wall: it covers the column completely
            dc_yl = yl;
            dc_yh = yh;
            set_column(midtexture, texturecolumn, rw_midtexturemid, base_iscale);
            R_DrawColumn();

            dm->ceilingclip[rw_x] = SCREENHEIGHT;
            dm->floorclip[rw_x] = -1;
        }
        else
        {
            if (toptexture >= 0)
            {
                mid = pixhigh >> HEIGHTBITS;
                pixhigh += pixhighstep;

                if (mid >= dm->floorclip[rw_x])
                    mid = dm->floorclip[rw_x] - 1;

                if (mid >= yl)
                {
                    dc_yl = yl;
                    dc_yh = mid;
                    set_column(toptexture, texturecolumn, rw_toptexturemid, base_iscale);
                    R_DrawColumn();

                    dm->ceilingclip[rw_x] = mid;
                }
                else
                {
                    dm->ceilingclip[rw_x] = yl - 1;
                }
            }
            else if (markceiling)
            {
                dm->ceilingclip[rw_x] = yl - 1;
            }

            if (bottomtexture >= 0)
            {
                mid = (pixlow + HEIGHTUNIT - 1) >> HEIGHTBITS;
                pixlow += pixlowstep;

                if (mid <= dm->ceilingclip[rw_x])
                    mid = dm->ceilingclip[rw_x] + 1;

                if (mid <= yh)
                {
                    dc_yl = mid;
                    dc_yh = yh;
                    set_column(bottomtexture, texturecolumn, rw_bottomtexturemid,
                               base_iscale);
                    R_DrawColumn();

                    dm->floorclip[rw_x] = mid;
                }
                else
                {
                    dm->floorclip[rw_x] = yh + 1;
                }
            }
            else if (markfloor)
            {
                dm->floorclip[rw_x] = yh + 1;
            }

            // A see-through middle texture is drawn later, back to front, so
            // only its column index is kept here
            if (maskedtexture)
                maskedtexturecol[rw_x] = texturecolumn;
        }

        rw_scale += rw_scalestep;
        topfrac += topstep;
        bottomfrac += bottomstep;
    }
}

//-----------------------------------------------------------------------------
static int openings_used(void)
{
    return (int)(lastopening - dm->openings);
}

//-----------------------------------------------------------------------------
void R_StoreWallRange(int start, int stop)
{
    fixed_t hyp, sineval, vtop;
    angle_t distangle, offsetangle;
    int lightnum;

    if (ds_p == &dm->drawsegs[MAXDRAWSEGS])
        return;

    // Perpendicular distance from the eye to the wall's line
    rw_normalangle = ((angle_t)(uint16_t)curline->angle << 16) + ANG90;

    offsetangle = rw_normalangle - rw_angle1;

    if (offsetangle > ANG180)
        offsetangle = (angle_t)(-(int32_t)offsetangle);

    if (offsetangle > ANG90)
        offsetangle = ANG90;

    distangle = ANG90 - offsetangle;
    hyp = R_PointToDist(mapfix(vertexes[curline->v1].x),
                        mapfix(vertexes[curline->v1].y));
    sineval = finesine(distangle >> ANGLETOFINESHIFT);
    rw_distance = FixedMul(hyp, sineval);

    ds_p->x1 = rw_x = start;
    ds_p->x2 = stop;
    ds_p->curline = (uint16_t)(curline - segs);
    rw_stopx = stop + 1;

    ds_p->scale1 = rw_scale = R_ScaleFromGlobalAngle(viewangle + dt_xtoviewangle[start]);

    if (stop > start)
    {
        ds_p->scale2 = R_ScaleFromGlobalAngle(viewangle + dt_xtoviewangle[stop]);
        ds_p->scalestep = rw_scalestep = (ds_p->scale2 - rw_scale) / (stop - start);
    }
    else
    {
        ds_p->scale2 = ds_p->scale1;
        rw_scalestep = 0;
        ds_p->scalestep = 0;
    }

    worldtop = frontsector->ceilingheight - viewz;
    worldbottom = frontsector->floorheight - viewz;

    midtexture = toptexture = bottomtexture = -1;
    maskedtexture = false;
    ds_p->maskedtexturecol = NULL;

    if (NULL == backsector)
    {
        // Single sided: one texture, and nothing behind it is visible
        midtexture = (NO_TEXTURE == sidedef->midtex) ? -1 : sidedef->midtex;
        markfloor = markceiling = true;

        if (linedef->flags & ML_DONTPEGBOTTOM)
        {
            // Texture pinned to the floor, so it grows upward
            vtop = frontsector->floorheight +
                (midtexture >= 0 ? texture_height(midtexture) : 0);
            rw_midtexturemid = vtop - viewz;
        }
        else
        {
            rw_midtexturemid = worldtop;
        }

        rw_midtexturemid += mapfix(sidedef->yoffset);

        ds_p->silhouette = SIL_BOTH;
        ds_p->sprtopclip = dm->screenheightarray;
        ds_p->sprbottomclip = dm->negonearray;
        ds_p->bsilheight = INT32_MAX;
        ds_p->tsilheight = INT32_MIN;
    }
    else
    {
        ds_p->sprtopclip = ds_p->sprbottomclip = NULL;
        ds_p->silhouette = 0;

        if (frontsector->floorheight > backsector->floorheight)
        {
            ds_p->silhouette = SIL_BOTTOM;
            ds_p->bsilheight = frontsector->floorheight;
        }
        else if (backsector->floorheight > viewz)
        {
            ds_p->silhouette = SIL_BOTTOM;
            ds_p->bsilheight = INT32_MAX;
        }

        if (frontsector->ceilingheight < backsector->ceilingheight)
        {
            ds_p->silhouette |= SIL_TOP;
            ds_p->tsilheight = frontsector->ceilingheight;
        }
        else if (backsector->ceilingheight < viewz)
        {
            ds_p->silhouette |= SIL_TOP;
            ds_p->tsilheight = INT32_MIN;
        }

        if (backsector->ceilingheight <= frontsector->floorheight)
        {
            ds_p->sprbottomclip = dm->negonearray;
            ds_p->bsilheight = INT32_MAX;
            ds_p->silhouette |= SIL_BOTTOM;
        }

        if (backsector->floorheight >= frontsector->ceilingheight)
        {
            ds_p->sprtopclip = dm->screenheightarray;
            ds_p->tsilheight = INT32_MIN;
            ds_p->silhouette |= SIL_TOP;
        }

        worldhigh = backsector->ceilingheight - viewz;
        worldlow = backsector->floorheight - viewz;

        // Outdoors on both sides: the sky has no height, so let the far
        // ceiling win and no upper texture appear
        if (SKY_FLAT == frontsector->ceilingpic && SKY_FLAT == backsector->ceilingpic)
            worldtop = worldhigh;

        markfloor = (worldlow != worldbottom ||
                     backsector->floorpic != frontsector->floorpic ||
                     backsector->lightlevel != frontsector->lightlevel);

        markceiling = (worldhigh != worldtop ||
                       backsector->ceilingpic != frontsector->ceilingpic ||
                       backsector->lightlevel != frontsector->lightlevel);

        if (backsector->ceilingheight <= frontsector->floorheight ||
            backsector->floorheight >= frontsector->ceilingheight)
            markceiling = markfloor = true;     // closed door

        if (worldhigh < worldtop && NO_TEXTURE != sidedef->toptex)
        {
            toptexture = sidedef->toptex;

            if (linedef->flags & ML_DONTPEGTOP)
            {
                rw_toptexturemid = worldtop;
            }
            else
            {
                vtop = backsector->ceilingheight + texture_height(toptexture);
                rw_toptexturemid = vtop - viewz;
            }
        }

        if (worldlow > worldbottom && NO_TEXTURE != sidedef->bottomtex)
        {
            bottomtexture = sidedef->bottomtex;

            rw_bottomtexturemid = (linedef->flags & ML_DONTPEGBOTTOM)
                ? worldtop : worldlow;
        }

        rw_toptexturemid += mapfix(sidedef->yoffset);
        rw_bottomtexturemid += mapfix(sidedef->yoffset);

        if (NO_TEXTURE != sidedef->midtex &&
            openings_used() + (rw_stopx - rw_x) <= MAXOPENINGS)
        {
            maskedtexture = true;
            ds_p->maskedtexturecol = maskedtexturecol = lastopening - rw_x;
            lastopening += rw_stopx - rw_x;
        }
    }

    segtextured = (midtexture >= 0) || (toptexture >= 0) || (bottomtexture >= 0) ||
                  maskedtexture;

    if (segtextured)
    {
        // Distance along the wall from its first vertex to the point the eye
        // projects onto it
        offsetangle = rw_normalangle - rw_angle1;

        if (offsetangle > ANG180)
            offsetangle = (angle_t)(-(int32_t)offsetangle);

        if (offsetangle > ANG90)
            offsetangle = ANG90;

        sineval = finesine(offsetangle >> ANGLETOFINESHIFT);
        rw_offset = FixedMul(hyp, sineval);

        if (rw_normalangle - rw_angle1 < ANG180)
            rw_offset = -rw_offset;

        rw_offset += (mapfix(sidedef->xoffset)) + (mapfix(curline->offset));
        rw_centerangle = ANG90 + viewangle - rw_normalangle;

        // Walls that run east-west are darkened and north-south ones
        // lightened, which is the original's stand-in for directional light
        lightnum = (frontsector->lightlevel >> LIGHTSEGSHIFT) + extralight;

        if (vertexes[curline->v1].y == vertexes[curline->v2].y)
            lightnum--;
        else if (vertexes[curline->v1].x == vertexes[curline->v2].x)
            lightnum++;

        if (lightnum < 0)
            lightnum = 0;
        else if (lightnum >= LIGHTLEVELS)
            lightnum = LIGHTLEVELS - 1;

        walllights = dt_scalelight + lightnum * MAXLIGHTSCALE;
    }

    // A plane on the far side of the eye cannot be seen, so do not mark it
    if (frontsector->floorheight >= viewz)
        markfloor = false;

    if (frontsector->ceilingheight <= viewz && SKY_FLAT != frontsector->ceilingpic)
        markceiling = false;

    // The edges are stepped in 1/16 of a pixel, which is enough resolution to
    // keep a long wall from drifting and still leaves headroom in 32 bits
    worldtop >>= 4;
    worldbottom >>= 4;

    topstep = -FixedMul(rw_scalestep, worldtop);
    topfrac = (centeryfrac >> 4) - FixedMul(worldtop, rw_scale);

    bottomstep = -FixedMul(rw_scalestep, worldbottom);
    bottomfrac = (centeryfrac >> 4) - FixedMul(worldbottom, rw_scale);

    if (backsector)
    {
        worldhigh >>= 4;
        worldlow >>= 4;

        if (worldhigh < worldtop)
        {
            pixhigh = (centeryfrac >> 4) - FixedMul(worldhigh, rw_scale);
            pixhighstep = -FixedMul(rw_scalestep, worldhigh);
        }

        if (worldlow > worldbottom)
        {
            pixlow = (centeryfrac >> 4) - FixedMul(worldlow, rw_scale);
            pixlowstep = -FixedMul(rw_scalestep, worldlow);
        }
    }

    if (markceiling)
        ceilingplane = R_CheckPlane(ceilingplane, rw_x, rw_stopx - 1);

    if (markfloor)
        floorplane = R_CheckPlane(floorplane, rw_x, rw_stopx - 1);

    R_RenderSegLoop();

    // Keep the silhouettes this wall cut, so what is drawn behind it later can
    // be clipped. When the opening store is exhausted the fallback has to be a
    // valid "clips nothing" array and not a null pointer: the masked pass walks
    // these unconditionally, and a null here is a fault rather than a glitch.
    if (((ds_p->silhouette & SIL_TOP) || maskedtexture) && !ds_p->sprtopclip)
    {
        if (openings_used() + (rw_stopx - start) <= MAXOPENINGS)
        {
            memcpy(lastopening, dm->ceilingclip + start, 2 * (rw_stopx - start));
            ds_p->sprtopclip = lastopening - start;
            lastopening += rw_stopx - start;
        }
        else
        {
            ds_p->sprtopclip = dm->negonearray;
        }
    }

    if (((ds_p->silhouette & SIL_BOTTOM) || maskedtexture) && !ds_p->sprbottomclip)
    {
        if (openings_used() + (rw_stopx - start) <= MAXOPENINGS)
        {
            memcpy(lastopening, dm->floorclip + start, 2 * (rw_stopx - start));
            ds_p->sprbottomclip = lastopening - start;
            lastopening += rw_stopx - start;
        }
        else
        {
            ds_p->sprbottomclip = dm->screenheightarray;
        }
    }

    if (maskedtexture && !(ds_p->silhouette & SIL_TOP))
    {
        ds_p->silhouette |= SIL_TOP;
        ds_p->tsilheight = INT32_MIN;
    }

    if (maskedtexture && !(ds_p->silhouette & SIL_BOTTOM))
    {
        ds_p->silhouette |= SIL_BOTTOM;
        ds_p->bsilheight = INT32_MAX;
    }

    if (ds_p - dm->drawsegs + 1 > r_stats.drawsegs)
        r_stats.drawsegs = ds_p - dm->drawsegs + 1;

    if (openings_used() > r_stats.openings)
        r_stats.openings = openings_used();

    ds_p++;
}

//-----------------------------------------------------------------------------
// The see-through middle textures, drawn after the solid geometry so that what
// is behind them is already on the screen
void R_RenderMaskedSegRange(drawseg_t *ds, int x1, int x2)
{
    unsigned index;
    int lightnum, texnum;
    const texture_t *tex;
    fixed_t texturemid, scale;

    curline = &segs[ds->curline];
    linedef = &lines[curline->linedef];
    sidedef = seg_sidedef(curline);
    frontsector = seg_frontsector(curline);
    backsector = seg_backsector(curline);

    if (NO_TEXTURE == sidedef->midtex || NULL == backsector ||
        NULL == ds->sprtopclip || NULL == ds->sprbottomclip)
        return;

    texnum = sidedef->midtex;
    tex = &textures[texnum];

    lightnum = (frontsector->lightlevel >> LIGHTSEGSHIFT) + extralight;

    if (vertexes[curline->v1].y == vertexes[curline->v2].y)
        lightnum--;
    else if (vertexes[curline->v1].x == vertexes[curline->v2].x)
        lightnum++;

    if (lightnum < 0)
        lightnum = 0;
    else if (lightnum >= LIGHTLEVELS)
        lightnum = LIGHTLEVELS - 1;

    walllights = dt_scalelight + lightnum * MAXLIGHTSCALE;

    maskedtexturecol = ds->maskedtexturecol;
    rw_scalestep = ds->scalestep;
    scale = ds->scale1 + (x1 - ds->x1) * rw_scalestep;
    mfloorclip = ds->sprbottomclip;
    mceilingclip = ds->sprtopclip;

    if (linedef->flags & ML_DONTPEGBOTTOM)
    {
        texturemid = frontsector->floorheight > backsector->floorheight
            ? frontsector->floorheight : backsector->floorheight;
        texturemid = texturemid + texture_height(texnum) - viewz;
    }
    else
    {
        texturemid = frontsector->ceilingheight < backsector->ceilingheight
            ? frontsector->ceilingheight : backsector->ceilingheight;
        texturemid = texturemid - viewz;
    }

    texturemid += mapfix(sidedef->yoffset);

    for (dc_x = x1; dc_x <= x2; dc_x++)
    {
        if (maskedtexturecol[dc_x] != MAXSHORT)
        {
            index = scale >> LIGHTSCALESHIFT;

            if (index >= MAXLIGHTSCALE)
                index = MAXLIGHTSCALE - 1;

            dc_colormap = dt_colormap + (walllights[index] << 8);

            // The post tops are stored-texture rows, so the drawer needs the
            // scale in screen pixels per stored row rather than per world unit
            spryscale = scale << tex->shift;
            dc_texturemid = texturemid >> tex->shift;
            dc_iscale = (0xffffffffu / (unsigned)scale) >> tex->shift;
            sprtopscreen = centeryfrac - FixedMul(dc_texturemid, spryscale);

            R_DrawMaskedColumn(w_column(texturecols[tex->firstcol +
                ((maskedtexturecol[dc_x] >> tex->shift) & tex->widthmask)]));

            maskedtexturecol[dc_x] = MAXSHORT;
            r_stats.maskedcols++;
        }

        scale += rw_scalestep;
    }
}

//-----------------------------------------------------------------------------
void R_DrawMasked(void)
{
    // Back to front: the drawsegs were recorded front to back
    for (drawseg_t *ds = ds_p - 1; ds >= dm->drawsegs; ds--)
    {
        if (ds->maskedtexturecol)
            R_RenderMaskedSegRange(ds, ds->x1, ds->x2);
    }
}
