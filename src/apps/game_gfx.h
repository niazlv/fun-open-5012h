/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The drawing toolkit the games share
 *
 * One 16x16 staging buffer, one text-art sprite format, one rounded-shape
 * builder, one scaled font and one panel. Every game wants all of it and only
 * one game runs at a time, so a second copy of any of it would be flash spent
 * twice and RAM reserved twice.
 *
 * Nothing here knows what a game is. It composes a cell and hands it to the
 * panel; where that cell goes and what it means is the caller's business.
 */

#ifndef _GAME_GFX_H_
#define _GAME_GFX_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "fonts.h"

/*- Definitions -------------------------------------------------------------*/
// The staging buffer, a sprite and a playfield cell are all this size. One
// number, because they are the same thing seen from three directions.
#define GFX_TILE            16

// Which sides of a cell the shape continues through. A corner is rounded only
// where both of its sides are free, which is what makes a straight run read as
// a tube, a turn read as a turn, and two neighbours join without a seam.
#define GFX_UP              (1 << 0)
#define GFX_DOWN            (1 << 1)
#define GFX_LEFT            (1 << 2)
#define GFX_RIGHT           (1 << 3)

#define GFX_TEXT_MAX_SCALE  3

/*- Types -------------------------------------------------------------------*/
// Text art: one character per pixel, '.' transparent, everything else a
// palette entry resolved by gfx_pixel_color(). 272 bytes an icon, editable in
// the source - the device has no filesystem, so an asset that is not compiled
// in does not exist, and one that needs a converter does not get edited.
//
// The rows keep their terminator. Sizing them at exactly GFX_TILE saves 16
// bytes an icon and costs more than it is worth: every compiler that has an
// opinion warns about a string literal with nowhere to put its NUL, and a row
// typed one character short then pads with NULs instead of failing a strlen.
typedef struct
{
    char rows[GFX_TILE][GFX_TILE + 1];
} sprite_t;

/*- Prototypes --------------------------------------------------------------*/
// Palette for text-art pixels. False means transparent, so a sprite composites
// over whatever is already in the tile.
bool gfx_pixel_color(char c, uint16_t *color);

// The staging tile
void gfx_fill(uint16_t color);
void gfx_set(int x, int y, uint16_t color);
void gfx_rect(int x, int y, int w, int h, uint16_t color);
void gfx_sprite(const sprite_t *sprite);
void gfx_blit(int px, int py);

// Fill, composite and blit in one go, for a sprite over a flat background
void gfx_draw_sprite(int px, int py, const sprite_t *sprite, uint16_t bg);

// The same sprite in another metal: 'Y' and 'y' take the two colours given,
// every other character keeps its palette entry. One icon and two arguments
// instead of one icon per colourway.
void gfx_sprite_recolor(const sprite_t *sprite, uint16_t main, uint16_t shade);

// Shapes are built into a coverage mask first and painted after, so that the
// paint can light every pixel with nothing above it and shade every pixel with
// nothing below it. That one rule gives any shape a rim without the shape
// having to know its own outline.
void gfx_shape_box(int mask, int margin, int radius);
void gfx_shape_taper(int mask);
void gfx_shape_paint(uint16_t body, uint16_t top, uint16_t bottom);

// Text. The scaled one expands the 8x16 font, which is the largest in the
// firmware and still too small for a title.
void gfx_text(int x, int y, const char *str, const Font *font,
    uint16_t fg, uint16_t bg);
void gfx_text_centered(int cx, int y, const char *str, const Font *font,
    uint16_t fg, uint16_t bg);
int  gfx_text_scaled_width(const char *str, int scale);
void gfx_text_scaled(int x, int y, const char *str, int scale,
    uint16_t fg, uint16_t bg);

// The same, painting only the glyph. Use it over a scene; the opaque one
// carries its own background, which is what erases the digit before it.
void gfx_text_scaled_over(int x, int y, const char *str, int scale,
    uint16_t fg);

// Straight to the panel, not through the tile
void gfx_round_rect(int x, int y, int w, int h, int radius, uint16_t color);
void gfx_panel(int x, int y, int w, int h, uint16_t fill, uint16_t edge,
    uint16_t shadow);

#endif // _GAME_GFX_H_
