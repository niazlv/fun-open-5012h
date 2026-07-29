/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Snake artwork and levels
 *
 * See snake_assets.h for the two formats. Everything here is const, so it
 * lives in flash and costs no RAM: five 16x16 sprites and eight maps come to
 * under 4 KB.
 */

/*- Includes ----------------------------------------------------------------*/
#include "lcd.h"
#include "snake_assets.h"

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The palette is shared by every sprite, so a character means the same colour
// wherever it appears and a sprite can be read as a picture in the source
bool snake_pixel_color(char c, uint16_t *color)
{
    switch (c)
    {
        case 'R': *color = LCD_COLOR(231,  71,  29); return true;  // fruit red
        case 'r': *color = LCD_COLOR(168,  32,  16); return true;  // its shadow
        case 'O': *color = LCD_COLOR(245, 140,  30); return true;  // orange
        case 'o': *color = LCD_COLOR(190,  92,  10); return true;
        case 'Y': *color = LCD_COLOR(255, 214,  66); return true;  // gold
        case 'y': *color = LCD_COLOR(206, 148,  16); return true;
        case 'g': *color = LCD_COLOR( 76, 175,  50); return true;  // leaf
        case 'G': *color = LCD_COLOR( 40, 110,  30); return true;
        case 'k': *color = LCD_COLOR(110,  70,  30); return true;  // stem
        case 'h': *color = LCD_COLOR(255, 238, 150); return true;  // seed
        case 'w': *color = LCD_COLOR(255, 255, 255); return true;  // highlight
        case 'b': *color = LCD_COLOR( 30,  32,  40); return true;
        default:  return false;                                    // '.', space
    }
}

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
