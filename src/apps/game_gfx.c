/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The drawing toolkit the games share - see game_gfx.h for what and why.
 *
 * This file is built with -Os (see make/Makefile). Its inner loops run 256
 * pixels behind a bit-banged panel that needs ~200 ns to accept one of them,
 * so unrolling them buys time that is not there to spend and costs kilobytes
 * of flash that are.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lcd.h"
#include "game_gfx.h"

/*- Variables ---------------------------------------------------------------*/
// One cell, composed here and blitted whole, so a half-drawn cell never
// reaches the panel. 768 bytes of TCM for every game together.
static uint16_t g_tile[GFX_TILE * GFX_TILE];
static uint8_t  g_shape[GFX_TILE * GFX_TILE];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The palette is shared by every sprite in the firmware, so a character means
// one colour wherever it appears and a sprite can be read as a picture in the
// source it is written in.
bool gfx_pixel_color(char c, uint16_t *color)
{
    switch (c)
    {
        case 'R': *color = LCD_COLOR(231,  71,  29); return true;  // fruit red
        case 'r': *color = LCD_COLOR(168,  32,  16); return true;  // its shadow
        case 'O': *color = LCD_COLOR(245, 140,  30); return true;  // orange, beak
        case 'o': *color = LCD_COLOR(190,  92,  10); return true;
        case 'Y': *color = LCD_COLOR(255, 214,  66); return true;  // gold, plumage
        case 'y': *color = LCD_COLOR(206, 148,  16); return true;
        case 'g': *color = LCD_COLOR( 76, 175,  50); return true;  // leaf, pipe
        case 'G': *color = LCD_COLOR( 40, 110,  30); return true;
        case 'l': *color = LCD_COLOR(140, 215,  90); return true;  // its highlight
        case 'k': *color = LCD_COLOR(110,  70,  30); return true;  // stem, soil
        case 'm': *color = LCD_COLOR(196, 154,  92); return true;  // ground
        case 't': *color = LCD_COLOR(232, 200, 140); return true;  // its highlight
        case 'h': *color = LCD_COLOR(255, 238, 150); return true;  // seed, glint
        case 'c': *color = LCD_COLOR(238, 246, 252); return true;  // cloud
        case 'C': *color = LCD_COLOR(206, 224, 240); return true;  // its underside
        case 'w': *color = LCD_COLOR(255, 255, 255); return true;
        case 'b': *color = LCD_COLOR( 30,  32,  40); return true;
        case 'B': *color = LCD_COLOR(150, 160, 175); return true;  // silver
        default:  return false;                                    // '.', space
    }
}

//-----------------------------------------------------------------------------
void gfx_fill(uint16_t color)
{
    for (int i = 0; i < GFX_TILE * GFX_TILE; i++)
        g_tile[i] = color;
}

//-----------------------------------------------------------------------------
void gfx_set(int x, int y, uint16_t color)
{
    if (x >= 0 && x < GFX_TILE && y >= 0 && y < GFX_TILE)
        g_tile[y * GFX_TILE + x] = color;
}

//-----------------------------------------------------------------------------
void gfx_rect(int x, int y, int w, int h, uint16_t color)
{
    for (int j = y; j < y + h; j++)
    {
        for (int i = x; i < x + w; i++)
            gfx_set(i, j, color);
    }
}

//-----------------------------------------------------------------------------
void gfx_sprite(const sprite_t *sprite)
{
    for (int y = 0; y < GFX_TILE; y++)
    {
        for (int x = 0; x < GFX_TILE; x++)
        {
            uint16_t color;

            if (gfx_pixel_color(sprite->rows[y][x], &color))
                g_tile[y * GFX_TILE + x] = color;
        }
    }
}

//-----------------------------------------------------------------------------
void gfx_blit(int px, int py)
{
    lcd_draw_buf(px, py, GFX_TILE, GFX_TILE, g_tile);
}

//-----------------------------------------------------------------------------
void gfx_draw_sprite(int px, int py, const sprite_t *sprite, uint16_t bg)
{
    gfx_fill(bg);
    gfx_sprite(sprite);
    gfx_blit(px, py);
}

//-----------------------------------------------------------------------------
void gfx_sprite_recolor(const sprite_t *sprite, uint16_t main, uint16_t shade)
{
    for (int y = 0; y < GFX_TILE; y++)
    {
        for (int x = 0; x < GFX_TILE; x++)
        {
            char ch = sprite->rows[y][x];
            uint16_t color;

            if ('Y' == ch)
                color = main;
            else if ('y' == ch)
                color = shade;
            else if (!gfx_pixel_color(ch, &color))
                continue;

            g_tile[y * GFX_TILE + x] = color;
        }
    }
}

//-----------------------------------------------------------------------------
// Pixel centres against a circle of radius r centred at (r, r), in halves so
// the whole test stays in integers
static bool inside_corner(int x, int y, int r)
{
    int dx = 2 * x + 1 - 2 * r;
    int dy = 2 * y + 1 - 2 * r;

    return dx * dx + dy * dy <= 4 * r * r;
}

//-----------------------------------------------------------------------------
// A core square inset by `margin`, with an arm out to each edge the shape
// continues through, and rounded corners wherever both of a corner's sides are
// free. With margin 0 the core already reaches every edge and the arms are
// empty, which is how a wall block tiles solid.
void gfx_shape_box(int mask, int margin, int radius)
{
    int lo = margin;
    int hi = GFX_TILE - 1 - margin;
    int n = hi - lo;

    memset(g_shape, 0, sizeof(g_shape));

    for (int y = lo; y <= hi; y++)
    {
        for (int x = lo; x <= hi; x++)
        {
            int lx = x - lo;
            int ly = y - lo;

            if (!(mask & GFX_UP) && !(mask & GFX_LEFT) &&
                lx < radius && ly < radius && !inside_corner(lx, ly, radius))
                continue;

            if (!(mask & GFX_UP) && !(mask & GFX_RIGHT) &&
                n - lx < radius && ly < radius &&
                !inside_corner(n - lx, ly, radius))
                continue;

            if (!(mask & GFX_DOWN) && !(mask & GFX_LEFT) &&
                lx < radius && n - ly < radius &&
                !inside_corner(lx, n - ly, radius))
                continue;

            if (!(mask & GFX_DOWN) && !(mask & GFX_RIGHT) &&
                n - lx < radius && n - ly < radius &&
                !inside_corner(n - lx, n - ly, radius))
                continue;

            g_shape[y * GFX_TILE + x] = 1;
        }
    }

    for (int i = 0; i < margin; i++)
    {
        for (int j = lo; j <= hi; j++)
        {
            if (mask & GFX_UP)    g_shape[i * GFX_TILE + j] = 1;
            if (mask & GFX_DOWN)  g_shape[(GFX_TILE - 1 - i) * GFX_TILE + j] = 1;
            if (mask & GFX_LEFT)  g_shape[j * GFX_TILE + i] = 1;
            if (mask & GFX_RIGHT) g_shape[j * GFX_TILE + GFX_TILE - 1 - i] = 1;
        }
    }
}

//-----------------------------------------------------------------------------
// Tapers from a full width joint at the connected edge to a two pixel tip at
// the other one. Square law rather than linear: a tail that draws in evenly
// across the whole cell reads as an arrowhead.
void gfx_shape_taper(int mask)
{
    memset(g_shape, 0, sizeof(g_shape));

    for (int i = 0; i < GFX_TILE; i++)
    {
        int half = 7 - (i * i * 5) / ((GFX_TILE - 1) * (GFX_TILE - 1));

        for (int j = 8 - half; j <= 7 + half; j++)
        {
            int x, y;

            switch (mask)
            {
                case GFX_LEFT:  x = i;                y = j;                break;
                case GFX_RIGHT: x = GFX_TILE - 1 - i; y = j;                break;
                case GFX_UP:    x = j;                y = i;                break;
                default:        x = j;                y = GFX_TILE - 1 - i; break;
            }

            g_shape[y * GFX_TILE + x] = 1;
        }
    }
}

//-----------------------------------------------------------------------------
// A row at the very edge of the cell is only filled where the shape continues
// into the next cell, so treating off-tile as filled is what keeps the seam
// between two neighbours unlit.
void gfx_shape_paint(uint16_t body, uint16_t top, uint16_t bottom)
{
    for (int y = 0; y < GFX_TILE; y++)
    {
        for (int x = 0; x < GFX_TILE; x++)
        {
            if (!g_shape[y * GFX_TILE + x])
                continue;

            if (y > 0 && !g_shape[(y - 1) * GFX_TILE + x])
                g_tile[y * GFX_TILE + x] = top;
            else if (y < GFX_TILE - 1 && !g_shape[(y + 1) * GFX_TILE + x])
                g_tile[y * GFX_TILE + x] = bottom;
            else
                g_tile[y * GFX_TILE + x] = body;
        }
    }
}

//-----------------------------------------------------------------------------
void gfx_text(int x, int y, const char *str, const Font *font,
    uint16_t fg, uint16_t bg)
{
    lcd_set_font(font);
    lcd_set_color(bg, fg);
    lcd_puts(x, y, str);
}

//-----------------------------------------------------------------------------
void gfx_text_centered(int cx, int y, const char *str, const Font *font,
    uint16_t fg, uint16_t bg)
{
    gfx_text(cx - (int)strlen(str) * font->width / 2, y, str, font, fg, bg);
}

//-----------------------------------------------------------------------------
int gfx_text_scaled_width(const char *str, int scale)
{
    return (int)strlen(str) * (FONT_LARGE)->width * scale;
}

//-----------------------------------------------------------------------------
// One glyph row is expanded into a line buffer and blitted `scale` times. The
// alternative - a fill_rect per scaled pixel - is 128 window setups a glyph.
void gfx_text_scaled(int x, int y, const char *str, int scale,
    uint16_t fg, uint16_t bg)
{
    const Font *font = FONT_LARGE;
    uint16_t line[8 * GFX_TEXT_MAX_SCALE];

    if (scale > GFX_TEXT_MAX_SCALE)
        scale = GFX_TEXT_MAX_SCALE;

    for (; *str; str++, x += font->width * scale)
    {
        char ch = *str;
        const uint8_t *bitmap;

        if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
            ch = '?';

        bitmap = font->data + (ch - FONT_FIRST_CHAR) * font->pitch;

        for (int gy = 0; gy < font->height; gy++)
        {
            for (int gx = 0; gx < font->width; gx++)
            {
                int i = gy * font->width + gx;
                uint16_t c = ((bitmap[i / 8] >> (i % 8)) & 1) ? fg : bg;

                for (int s = 0; s < scale; s++)
                    line[gx * scale + s] = c;
            }

            for (int s = 0; s < scale; s++)
                lcd_draw_buf(x, y + gy * scale + s, font->width * scale, 1, line);
        }
    }
}

//-----------------------------------------------------------------------------
// Glyph pixels only, as runs of set bits. Nothing is buffered: a run is one
// colour, so it is one fill_rect, and the gaps are simply never written.
void gfx_text_scaled_over(int x, int y, const char *str, int scale, uint16_t fg)
{
    const Font *font = FONT_LARGE;

    if (scale > GFX_TEXT_MAX_SCALE)
        scale = GFX_TEXT_MAX_SCALE;

    for (; *str; str++, x += font->width * scale)
    {
        char ch = *str;
        const uint8_t *bitmap;

        if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
            ch = '?';

        bitmap = font->data + (ch - FONT_FIRST_CHAR) * font->pitch;

        for (int gy = 0; gy < font->height; gy++)
        {
            int run = 0;

            for (int gx = 0; gx <= font->width; gx++)
            {
                int i = gy * font->width + gx;
                int on = (gx < font->width) &&
                    ((bitmap[i / 8] >> (i % 8)) & 1);

                if (on)
                {
                    run++;
                    continue;
                }

                if (run)
                    lcd_fill_rect(x + (gx - run) * scale, y + gy * scale,
                        run * scale, scale, fg);

                run = 0;
            }
        }
    }
}

//-----------------------------------------------------------------------------
// Rounded rectangle straight to the panel, a row of fill_rect at a time. Used
// where the shape is bigger than a tile - pipe lips, panels.
void gfx_round_rect(int x, int y, int w, int h, int radius, uint16_t color)
{
    if (radius > w / 2) radius = w / 2;
    if (radius > h / 2) radius = h / 2;

    lcd_fill_rect(x, y + radius, w, h - 2 * radius, color);

    for (int i = 0; i < radius; i++)
    {
        int inset = radius;

        while (inset > 0 && inside_corner(inset - 1, i, radius))
            inset--;

        lcd_fill_rect(x + inset, y + i, w - 2 * inset, 1, color);
        lcd_fill_rect(x + inset, y + h - 1 - i, w - 2 * inset, 1, color);
    }
}

//-----------------------------------------------------------------------------
void gfx_panel(int x, int y, int w, int h, uint16_t fill, uint16_t edge,
    uint16_t shadow)
{
    lcd_fill_rect(x + 4, y + 4, w, h, shadow);
    lcd_fill_rect(x, y, w, h, fill);
    lcd_draw_rect(x, y, w, h, edge);
    lcd_draw_rect(x + 1, y + 1, w - 2, h - 2, edge);
}
