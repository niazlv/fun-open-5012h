/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Snake artwork and levels
 *
 * Both are text art: one character per pixel for a sprite, one character per
 * cell for a level. A 16x16 icon costs the same 272 bytes of flash an indexed
 * bitmap would, and it stays editable in the source without a converter - the
 * device has no filesystem to load assets from, so anything not compiled in
 * does not exist.
 */

#ifndef _SNAKE_ASSETS_H_
#define _SNAKE_ASSETS_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
// The cell size divides the panel exactly: 20 x 13 cells of 16 px is 320 x 208,
// which is the display minus the 32 px status bar
#define SNAKE_CELL      16
#define SNAKE_GRID_W    20
#define SNAKE_GRID_H    13

/*- Types -------------------------------------------------------------------*/
// Sprite pixels. '.' is transparent, every other character is a palette entry
// resolved by snake_pixel_color().
typedef struct
{
    char rows[SNAKE_CELL][SNAKE_CELL + 1];
} sprite_t;

// A level is its map plus the two numbers that only mean anything next to it.
// Map characters:
//   ' '            empty
//   '#'            wall
//   '>' '<' '^' 'v'  where the snake starts, and which way it faces
typedef struct
{
    const char *name;
    int target;         // fruit to eat before the level is cleared
    int step_ms;        // step interval at the Normal speed setting
    char rows[SNAKE_GRID_H][SNAKE_GRID_W + 1];
} snake_level_t;

/*- Constants ---------------------------------------------------------------*/
// The three ordinary fruit are worth the same; which one spawns is chosen at
// random purely so the field does not look the same twice
extern const sprite_t snake_spr_apple;
extern const sprite_t snake_spr_orange;
extern const sprite_t snake_spr_berry;
extern const sprite_t snake_spr_gold;    // the timed bonus
extern const sprite_t snake_spr_skull;   // game over panel

extern const snake_level_t snake_levels[];
extern const int snake_level_count;

/*- Prototypes --------------------------------------------------------------*/
// Resolves one text-art pixel. Returns false for transparent characters, so a
// sprite composites over whatever is already in the tile.
bool snake_pixel_color(char c, uint16_t *color);

#endif // _SNAKE_ASSETS_H_
