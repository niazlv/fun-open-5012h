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
 * The two inner loops.
 *
 * Everything else in the renderer exists to set these up. A column walks down
 * the screen stepping through a texture column; a span walks across it stepping
 * through a flat. Both index a colormap on the way out, which is how DOOM does
 * distance shading without touching the pixel values.
 *
 * The frame buffer is 8-bit palette indices, exactly as the original: the
 * colormaps only work on indices, and it halves both the memory and the write
 * traffic compared with keeping RGB565 here.
 */

/*- Includes ----------------------------------------------------------------*/
#include "doom.h"

/*- Variables ---------------------------------------------------------------*/
int dc_x, dc_yl, dc_yh;
fixed_t dc_iscale, dc_texturemid;
const uint8_t *dc_source;
const uint8_t *dc_colormap;
int dc_texheight;

int ds_y, ds_x1, ds_x2;
fixed_t ds_xfrac, ds_yfrac, ds_xstep, ds_ystep;
const uint8_t *ds_source;
const uint8_t *ds_colormap;

// Masked drawing state, shared with the sprite and midtexture passes
fixed_t sprtopscreen, spryscale;
const int16_t *mfloorclip, *mceilingclip;

// Derived from the pack's flat size once, so the span loop stays branch free
static int flat_ushift = 16;
static int flat_umask = 63;
static int flat_vshift = 10;
static int flat_vmask = 63 * 64;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void R_InitDrawFlats(int shift)
{
    int size = 64 >> shift;         // flat is size x size
    int logsize = 6 - shift;

    flat_ushift = FRACBITS + shift;
    flat_umask = size - 1;
    flat_vshift = FRACBITS + shift - logsize;
    flat_vmask = (size - 1) * size;
}

//-----------------------------------------------------------------------------
void R_DrawColumn(void)
{
    int count = dc_yh - dc_yl;
    uint8_t *dest;
    const uint8_t *source = dc_source;
    const uint8_t *colormap = dc_colormap;
    fixed_t frac, fracstep;

    if (count < 0)
        return;

    dest = dm->screen + dc_yl * SCREENWIDTH + dc_x;
    fracstep = dc_iscale;
    frac = dc_texturemid + (dc_yl - centery) * fracstep;

    if (0 == (dc_texheight & (dc_texheight - 1)))
    {
        // Power of two, which almost every DOOM texture is
        int mask = dc_texheight - 1;

        do
        {
            *dest = colormap[source[(frac >> FRACBITS) & mask]];
            dest += SCREENWIDTH;
            frac += fracstep;
        }
        while (count--);
    }
    else
    {
        // The original wraps every texture at 128 rows, which is where the
        // tutti-frutti of a short texture comes from; wrapping at the real
        // height costs one modulo outside the loop and looks right
        int height = dc_texheight;
        int pos = (frac >> FRACBITS) % height;

        if (pos < 0)
            pos += height;

        do
        {
            *dest = colormap[source[pos]];
            dest += SCREENWIDTH;

            frac += fracstep;
            pos = (frac >> FRACBITS) % height;

            if (pos < 0)
                pos += height;
        }
        while (count--);
    }
}

//-----------------------------------------------------------------------------
// A column that has gaps: the post list says which runs of rows exist. The
// clip arrays are the wall silhouettes this column has to stay inside of.
void R_DrawMaskedColumn(const uint8_t *post)
{
    fixed_t basetexturemid = dc_texturemid;
    int64_t topscreen, bottomscreen;

    dc_texheight = 0;               // posts are bounded by the clip, not wrapped

    while (0xff != post[0])
    {
        int top = post[0];
        int length = post[1];

        // A post very close to the eye overflows 32 bits here; the original
        // wraps and draws a stripe of garbage, so keep the intermediate wide
        topscreen = (int64_t)sprtopscreen + (int64_t)spryscale * top;
        bottomscreen = topscreen + (int64_t)spryscale * length;

        dc_yl = (int)((topscreen + FRACUNIT - 1) >> FRACBITS);
        dc_yh = (int)((bottomscreen - 1) >> FRACBITS);

        if (dc_yl < 0)
            dc_yl = 0;

        if (dc_yh >= SCREENHEIGHT)
            dc_yh = SCREENHEIGHT - 1;

        if (dc_yh >= mfloorclip[dc_x])
            dc_yh = mfloorclip[dc_x] - 1;

        if (dc_yl <= mceilingclip[dc_x])
            dc_yl = mceilingclip[dc_x] + 1;

        if (dc_yl <= dc_yh)
        {
            dc_source = post + 2;
            dc_texturemid = basetexturemid - (top << FRACBITS);

            // The post is addressed from its own start, so no wrap applies
            {
                int count = dc_yh - dc_yl;
                uint8_t *dest = dm->screen + dc_yl * SCREENWIDTH + dc_x;
                const uint8_t *source = dc_source;
                const uint8_t *colormap = dc_colormap;
                fixed_t fracstep = dc_iscale;
                fixed_t frac = dc_texturemid + (dc_yl - centery) * fracstep;

                do
                {
                    int row = frac >> FRACBITS;

                    if (row < 0)
                        row = 0;
                    else if (row >= length)
                        row = length - 1;

                    *dest = colormap[source[row]];
                    dest += SCREENWIDTH;
                    frac += fracstep;
                }
                while (count--);
            }
        }

        post += length + 2;
    }

    dc_texturemid = basetexturemid;
}

//-----------------------------------------------------------------------------
void R_DrawSpan(void)
{
    int count = ds_x2 - ds_x1;
    uint8_t *dest;
    const uint8_t *source = ds_source;
    const uint8_t *colormap = ds_colormap;
    fixed_t xfrac = ds_xfrac, yfrac = ds_yfrac;
    fixed_t xstep = ds_xstep, ystep = ds_ystep;

    if (count < 0)
        return;

    dest = dm->screen + ds_y * SCREENWIDTH + ds_x1;

    do
    {
        int spot = ((yfrac >> flat_vshift) & flat_vmask) +
                   ((xfrac >> flat_ushift) & flat_umask);

        *dest++ = colormap[source[spot]];

        xfrac += xstep;
        yfrac += ystep;
    }
    while (count--);
}
