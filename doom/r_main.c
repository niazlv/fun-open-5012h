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
 * Renderer core: the angle and projection math every other stage is built on.
 *
 * These are the original routines. R_PointToAngle resolves the octant by sign
 * and magnitude and then indexes tantoangle, which is why it costs one divide
 * instead of an atan2; R_ScaleFromGlobalAngle is the perspective divide that
 * turns a wall's angle relative to the view into a screen scale. Both are exact
 * in 16.16, and the rest of the renderer depends on that exactness - drifting
 * here shows up as walls that shear apart at the seams.
 */

/*- Includes ----------------------------------------------------------------*/
#include "doom.h"

/*- Variables ---------------------------------------------------------------*/
fixed_t viewx, viewy, viewz;
angle_t viewangle;
fixed_t viewcos, viewsin;
int extralight;

int centerx, centery;
fixed_t centerxfrac, centeryfrac, projection;
angle_t clipangle;

int validcount = 1;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void R_Init(void)
{
    centerx = SCREENWIDTH / 2;
    centery = SCREENHEIGHT / 2;
    centerxfrac = mapfix(centerx);
    centeryfrac = mapfix(centery);
    projection = centerxfrac;

    // The angle that maps to column 0 is the half-width of the view cone
    clipangle = dt_xtoviewangle[0];
}

//-----------------------------------------------------------------------------
// The binary angle from the view point to (x, y). Octant by octant so that a
// single table of atan over [0, 1] covers the whole circle.
angle_t R_PointToAngle(fixed_t x, fixed_t y)
{
    x -= viewx;
    y -= viewy;

    if (0 == x && 0 == y)
        return 0;

    if (x >= 0)
    {
        if (y >= 0)
        {
            if (x > y)
                return dt_tantoangle[SlopeDiv(y, x)];               // 0
            else
                return ANG90 - 1 - dt_tantoangle[SlopeDiv(x, y)];   // 1
        }
        else
        {
            y = -y;

            if (x > y)
                return (angle_t)(-(int32_t)dt_tantoangle[SlopeDiv(y, x)]); // 8
            else
                return ANG270 + dt_tantoangle[SlopeDiv(x, y)];      // 7
        }
    }
    else
    {
        x = -x;

        if (y >= 0)
        {
            if (x > y)
                return ANG180 - 1 - dt_tantoangle[SlopeDiv(y, x)];  // 3
            else
                return ANG90 + dt_tantoangle[SlopeDiv(x, y)];       // 2
        }
        else
        {
            y = -y;

            if (x > y)
                return ANG180 + dt_tantoangle[SlopeDiv(y, x)];      // 4
            else
                return ANG270 - 1 - dt_tantoangle[SlopeDiv(x, y)];  // 5
        }
    }
}

//-----------------------------------------------------------------------------
angle_t R_PointToAngle2(fixed_t x1, fixed_t y1, fixed_t x2, fixed_t y2)
{
    fixed_t sx = viewx, sy = viewy;
    angle_t result;

    viewx = x1;
    viewy = y1;
    result = R_PointToAngle(x2, y2);
    viewx = sx;
    viewy = sy;

    return result;
}

//-----------------------------------------------------------------------------
// Distance from the view point, via the angle table rather than a square root
fixed_t R_PointToDist(fixed_t x, fixed_t y)
{
    int angle;
    fixed_t dx, dy, temp;

    dx = x - viewx;
    dy = y - viewy;

    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;

    if (dy > dx)
    {
        temp = dx;
        dx = dy;
        dy = temp;
    }

    if (0 == dx)
        return 0;

    angle = (dt_tantoangle[FixedDiv(dy, dx) >> DBITS] + ANG90) >> ANGLETOFINESHIFT;

    return FixedDiv(dx, finesine(angle));
}

//-----------------------------------------------------------------------------
// The scale (screen pixels per world unit) of a wall seen at visangle, given
// the seg's distance and normal that R_StoreWallRange has already worked out
extern fixed_t rw_distance;
extern angle_t rw_normalangle;

fixed_t R_ScaleFromGlobalAngle(angle_t visangle)
{
    fixed_t scale, num, den;
    int sinea, sineb;

    sinea = finesine((ANG90 + (visangle - viewangle)) >> ANGLETOFINESHIFT);
    sineb = finesine((ANG90 + (visangle - rw_normalangle)) >> ANGLETOFINESHIFT);

    num = FixedMul(projection, sineb);
    den = FixedMul(rw_distance, sinea);

    if (den > num >> 16)
    {
        scale = FixedDiv(num, den);

        if (scale > 64 * FRACUNIT)
            scale = 64 * FRACUNIT;
        else if (scale < 256)
            scale = 256;
    }
    else
    {
        scale = 64 * FRACUNIT;
    }

    return scale;
}

//-----------------------------------------------------------------------------
// Which side of a BSP partition line a point is on. The sign-bit shortcut is
// the original's: when the four signs disagree the answer follows from them
// alone, and the two multiplies can be skipped.
int R_PointOnSide(fixed_t x, fixed_t y, const mapnode_t *node)
{
    fixed_t dx, dy, left, right;

    if (0 == node->dx)
    {
        if (x <= (mapfix(node->x)))
            return node->dy > 0;

        return node->dy < 0;
    }

    if (0 == node->dy)
    {
        if (y <= (mapfix(node->y)))
            return node->dx < 0;

        return node->dx > 0;
    }

    dx = x - (mapfix(node->x));
    dy = y - (mapfix(node->y));

    if (((int32_t)node->dy ^ (int32_t)node->dx ^ dx ^ dy) & 0x80000000)
        return (((int32_t)node->dy ^ dx) & 0x80000000) ? 1 : 0;

    // node->dx/dy are map units, which is exactly the fixed_t value shifted
    // down by FRACBITS - the form both multiplies want
    left = FixedMul(node->dy, dx);
    right = FixedMul(dy, node->dx);

    return (right < left) ? 0 : 1;
}

//-----------------------------------------------------------------------------
int R_PointOnSegSide(fixed_t x, fixed_t y, const mapseg_t *line)
{
    const mapvertex_t *v1 = &vertexes[line->v1];
    const mapvertex_t *v2 = &vertexes[line->v2];
    int ldx = v2->x - v1->x;
    int ldy = v2->y - v1->y;
    fixed_t dx, dy, left, right;

    if (0 == ldx)
    {
        if (x <= (mapfix(v1->x)))
            return ldy > 0;

        return ldy < 0;
    }

    if (0 == ldy)
    {
        if (y <= (mapfix(v1->y)))
            return ldx < 0;

        return ldx > 0;
    }

    dx = x - (mapfix(v1->x));
    dy = y - (mapfix(v1->y));

    if ((ldy ^ ldx ^ dx ^ dy) & 0x80000000)
        return ((ldy ^ dx) & 0x80000000) ? 1 : 0;

    left = FixedMul(ldy, dx);
    right = FixedMul(dy, ldx);

    return (right < left) ? 0 : 1;
}

//-----------------------------------------------------------------------------
int R_PointInSubsector(fixed_t x, fixed_t y)
{
    int nodenum;

    if (0 == numnodes)
        return 0;

    nodenum = numnodes - 1;

    while (!(nodenum & 0x8000))
        nodenum = nodes[nodenum].children[R_PointOnSide(x, y, &nodes[nodenum])];

    return nodenum & ~0x8000;
}

//-----------------------------------------------------------------------------
void R_RenderPlayerView(fixed_t x, fixed_t y, fixed_t z, angle_t angle)
{
    viewx = x;
    viewy = y;
    viewz = z;
    viewangle = angle;

    viewsin = finesine(angle >> ANGLETOFINESHIFT);
    viewcos = finecosine(angle >> ANGLETOFINESHIFT);

    validcount++;

    R_ClearClipSegs();
    R_ClearDrawSegs();
    R_ClearPlanes();

    R_RenderBSPNode(numnodes - 1);

    R_DrawPlanes();
    R_DrawMasked();
}
