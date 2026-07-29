/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Snake artwork and levels
 *
 * Everything here is const, so it lives in flash and costs no RAM. The sprite
 * format and its palette are in game_gfx.c, shared with the other games.
 */

/*- Includes ----------------------------------------------------------------*/
#include "snake_assets.h"

/*- Constants ---------------------------------------------------------------*/

//-----------------------------------------------------------------------------
const sprite_t snake_spr_apple =
{{
    "................",
    ".......kk.......",
    ".......kk.ggg...",
    "...RRR.kkgggggg.",
    "..RRRRRRkkgggg..",
    ".RRwRRRRRRRRrr..",
    ".RwwRRRRRRRRRrr.",
    ".RwRRRRRRRRRRrr.",
    ".RRRRRRRRRRRRrr.",
    ".RRRRRRRRRRRRrr.",
    ".RRRRRRRRRRRRrr.",
    "..RRRRRRRRRRrr..",
    "..RRRRRRRRRrrr..",
    "...RRRRRRRrrr...",
    "....RRRRRrrr....",
    "......RRrr......",
}};

//-----------------------------------------------------------------------------
const sprite_t snake_spr_orange =
{{
    "................",
    "................",
    ".......kk.......",
    "....OOOkkggg....",
    "..OOwOOOOOgg....",
    ".OOwwOOOOOOoo...",
    ".OwOOOOOOOOOoo..",
    ".OOOOOOOOOOOoo..",
    ".OOOOOOOOOOOOo..",
    ".OOOOOOOOOOOOo..",
    ".OOOOOOOOOOOOoo.",
    ".OOOOOOOOOOOoo..",
    "..OOOOOOOOOoo...",
    "...OOOOOOOoo....",
    ".....OOOoo......",
    "................",
}};

//-----------------------------------------------------------------------------
// A strawberry: the same red as the apple, told apart by the leafy crown and
// the seeds rather than by its outline, which is all a 16 px silhouette has
const sprite_t snake_spr_berry =
{{
    "................",
    "......ggg.......",
    "....gggggggg....",
    "...ggGGgGGggg...",
    "...RRRRRRRRRR...",
    "..RwRhRRhRRhRr..",
    ".RRwRRRhRRhRRrr.",
    ".RRRhRRRhRRhRrr.",
    ".RRRRhRRRhRRRrr.",
    "..RRRRhRRhRRrr..",
    "..RRhRRRhRRRrr..",
    "...RRRhRRhRrr...",
    "...RRRRRRRRrr...",
    "....RRRhRRrr....",
    ".....RRRRrr.....",
    "......RRrr......",
}};

//-----------------------------------------------------------------------------
const sprite_t snake_spr_gold =
{{
    "................",
    ".......kk.......",
    ".......kk.ggg...",
    "...YYY.kkgggggg.",
    "..YYYYYYkkgggg..",
    ".YYwYYYYYYYYyy..",
    ".YwwYYYYYYYYYyy.",
    ".YwYYYYYYYYYYyy.",
    ".YYYYYYYYYYYYyy.",
    ".YYYYYYYYYYwYyy.",
    ".YYYYYYYYYYYYyy.",
    "..YYYYYYYYYYyy..",
    "..YYYYYYYYYyyy..",
    "...YYYYYYYyyy...",
    "....YYYYYyyy....",
    "......YYyy......",
}};

//-----------------------------------------------------------------------------
const sprite_t snake_spr_skull =
{{
    "................",
    "....wwwwwwww....",
    "..wwwwwwwwwwww..",
    ".wwwwwwwwwwwwww.",
    ".wwbbbwwwwbbbww.",
    ".wwbbbwwwwbbbww.",
    ".wwwwwwwwwwwwww.",
    ".wwwwwwbbwwwwww.",
    ".wwwwwwbbwwwwww.",
    "..wwwwwwwwwwww..",
    "...wwwwwwwwww...",
    "....wwwwwwww....",
    "....wbwbwbww....",
    "....wwwwwwww....",
    "................",
    "................",
}};

//-----------------------------------------------------------------------------
// The campaign. Difficulty is carried by the map first and the clock second:
// the step interval only tightens by 10 ms a level, so what makes level 8 hard
// is that there is nowhere left to put a long snake.
//
// Every map leaves the start cell and the two cells behind it clear - that is
// where the three starting segments go - and every wall layout is reachable in
// full, so no fruit can spawn somewhere the snake cannot get to.
const snake_level_t snake_levels[] =
{
    {
        .name = "MEADOW", .target = 5, .step_ms = 170,
        .rows =
        {
            "                    ",
            "                    ",
            "                    ",
            "                    ",
            "                    ",
            "                    ",
            "    >               ",
            "                    ",
            "                    ",
            "                    ",
            "                    ",
            "                    ",
            "                    ",
        },
    },
    {
        .name = "THE PEN", .target = 6, .step_ms = 160,
        .rows =
        {
            "####################",
            "#                  #",
            "#                  #",
            "#                  #",
            "#                  #",
            "#                  #",
            "#   >              #",
            "#                  #",
            "#                  #",
            "#                  #",
            "#                  #",
            "#                  #",
            "####################",
        },
    },
    {
        .name = "FOUR CORNERS", .target = 7, .step_ms = 150,
        .rows =
        {
            "#####          #####",
            "#####          #####",
            "###              ###",
            "                    ",
            "                    ",
            "                    ",
            "    >               ",
            "                    ",
            "                    ",
            "                    ",
            "###              ###",
            "#####          #####",
            "#####          #####",
        },
    },
    {
        .name = "PILLARS", .target = 8, .step_ms = 140,
        .rows =
        {
            "                    ",
            "     ###    ###     ",
            "     ###    ###     ",
            "     ###    ###     ",
            "                    ",
            "                    ",
            "    >               ",
            "                    ",
            "                    ",
            "     ###    ###     ",
            "     ###    ###     ",
            "     ###    ###     ",
            "                    ",
        },
    },
    {
        .name = "LADDER", .target = 9, .step_ms = 130,
        .rows =
        {
            "                    ",
            "  ##############    ",
            "                    ",
            "                    ",
            "    ##############  ",
            "                    ",
            "   >                ",
            "                    ",
            "  ##############    ",
            "                    ",
            "                    ",
            "    ##############  ",
            "                    ",
        },
    },
    {
        .name = "DIAMOND", .target = 10, .step_ms = 120,
        .rows =
        {
            "                    ",
            "        ####        ",
            "       ##  ##       ",
            "      ##    ##      ",
            "     ##      ##     ",
            "    ##        ##    ",
            "   >                ",
            "    ##        ##    ",
            "     ##      ##     ",
            "      ##    ##      ",
            "       ##  ##       ",
            "        ####        ",
            "                    ",
        },
    },
    {
        .name = "ORCHARD", .target = 11, .step_ms = 110,
        .rows =
        {
            "                    ",
            "  ##   ##    ##  ## ",
            "                    ",
            " ##   ##    ##   ## ",
            "                    ",
            "   ##    ##    ##   ",
            "   >                ",
            "   ##    ##    ##   ",
            "                    ",
            " ##   ##    ##   ## ",
            "                    ",
            "  ##   ##    ##  ## ",
            "                    ",
        },
    },
    {
        .name = "THE VAULT", .target = 12, .step_ms = 100,
        .rows =
        {
            "####################",
            "#                  #",
            "#    ##########    #",
            "#                  #",
            "#   ####    ####   #",
            "#                  #",
            "#  >               #",
            "#                  #",
            "#   ####    ####   #",
            "#                  #",
            "#    ##########    #",
            "#                  #",
            "####################",
        },
    },
};

const int snake_level_count = (int)(sizeof(snake_levels) / sizeof(snake_levels[0]));
