/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Snake artwork and levels
 *
 * The sprite format and its palette are in game_gfx.h, shared with every other
 * game. What is here is only what is Snake's: five icons and eight maps.
 */

#ifndef _SNAKE_ASSETS_H_
#define _SNAKE_ASSETS_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "game_gfx.h"

/*- Definitions -------------------------------------------------------------*/
// The cell size divides the panel exactly: 20 x 13 cells of GFX_TILE is
// 320 x 208, which is the display minus the 32 px status bar
#define SNAKE_GRID_W    20
#define SNAKE_GRID_H    13

/*- Types -------------------------------------------------------------------*/
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

#endif // _SNAKE_ASSETS_H_
