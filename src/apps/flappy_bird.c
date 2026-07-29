/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Flappy Bird
 *
 * Everything that draws comes from game_gfx: the sprite format, the 16x16
 * staging tile, the scaled font and the panel. What is left here is the scene -
 * a graded sky with clouds behind it, pipes with lipped ends, a scrolling
 * ground, and a bird composed over whatever it happens to be flying past.
 *
 * The scene is repainted by parts, never whole. restore_rect() is the primitive
 * the rest is built on: hand it a rectangle and it puts back the sky, the
 * clouds and the pipes that belong there, in that order. A frame is then the
 * strips the pipes vacated, the strips they moved into, the band the bird left
 * behind, and one 16x16 blit for the bird itself.
 *
 * Built with -Os (see make/Makefile): the panel takes ~200 ns a pixel and no
 * arithmetic queued behind that is worth unrolling.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "utils.h"
#include "config.h"
#include "ui.h"
#include "menu_widget.h"
#include "game_gfx.h"
#include "flappy_bird.h"

/*- Definitions -------------------------------------------------------------*/
#define FRAME_MS            40              // 25 fps

#define GROUND_H            28
#define SKY_H               (LCD_HEIGHT - GROUND_H)
#define GROUND_Y            SKY_H
#define STRIPE_H            7               // the scrolling band of the ground

#define BIRD                GFX_TILE
#define BIRD_X              56
#define BIRD_BODY           3               // the sprite's transparent margin

#define PIPE_W              32
#define PIPE_LIP_H          12
#define PIPE_LIP_OVER       5               // how far a collar overhangs the body
#define PIPE_SPACING        144
#define MAX_PIPES           4
#define GAP_MARGIN          26              // keep a gap clear of sky and ground

#define MAX_CLOUDS          3

#define GRAVITY             0.52f
#define FLAP_V              (-5.6f)
#define MAX_FALL            9.0f

// The gap closes and the pipes speed up as the score climbs, so one run keeps
// asking for more instead of settling into a rhythm
#define RAMP_EVERY          6               // points per step of the ramp
#define RAMP_STEPS          6               // steps before it stops tightening
#define RAMP_GAP            4               // px the gap loses per step
#define RAMP_SPEED_AT       3               // steps per +1 px/frame

#define DIFF_EASY           0
#define DIFF_NORMAL         1
#define DIFF_HARD           2

#define MEDAL_BRONZE        10
#define MEDAL_SILVER        25
#define MEDAL_GOLD          50

#define DEATH_FLASH_MS      120
#define OFFSCREEN           (-1000)

/*- Colors ------------------------------------------------------------------*/
// The sky is banded rather than flat: six fills cost what one costs on a
// bit-banged panel, and a flat 200 px of one blue is the thing that made the
// old scene look like a test pattern.
#define SKY_BANDS           6

#define C_GROUND            LCD_COLOR(222, 190, 128)
#define C_GROUND_DARK       LCD_COLOR(180, 148,  94)
#define C_GROUND_EDGE       LCD_COLOR( 96, 168,  62)
#define C_GROUND_EDGE_LO    LCD_COLOR( 66, 124,  40)
#define C_STRIPE            LCD_COLOR(238, 212, 158)

#define C_PIPE              LCD_COLOR( 92, 190,  60)
#define C_PIPE_HI           LCD_COLOR(150, 224, 110)
#define C_PIPE_LO           LCD_COLOR( 56, 132,  36)
#define C_PIPE_EDGE         LCD_COLOR( 34,  86,  24)

#define C_TEXT              LCD_COLOR(255, 255, 255)
#define C_TEXT_DIM          LCD_COLOR(196, 214, 232)
#define C_SHADE             LCD_COLOR( 40,  56,  76)
#define C_GOLD              LCD_COLOR(255, 214,  66)

#define C_PANEL             LCD_COLOR( 24,  40,  62)
#define C_PANEL_EDGE        LCD_COLOR(120, 172, 224)
#define C_PANEL_SHADOW      LCD_COLOR( 44,  84, 126)

#define C_WING              LCD_COLOR(206, 148,  16)
#define C_WING_HI           LCD_COLOR(255, 238, 150)
#define C_FLASH             LCD_COLOR(255, 255, 255)

/*- Types -------------------------------------------------------------------*/
typedef enum
{
    ST_TITLE = 0,
    ST_PLAY,
    ST_DYING,       // the bird falls to the ground before the panel comes up
    ST_OVER,
} state_t;

typedef struct
{
    int  x;
    int  drawn_x;
    int  gap_y;
    bool passed;
} pipe_t;

typedef struct
{
    int x, y;
} cloud_t;

/*- Artwork -----------------------------------------------------------------*/
// The bird carries no wing: the wing is three rectangles drawn over the body at
// one of three heights, which is 500 bytes less than three sprites of a bird
// that differ in six rows.
static const sprite_t g_spr_bird =
{{
    "................",
    "....bbbbbb......",
    "..bbYYYYYYbb....",
    ".bYYYYYYwwwwbb..",
    ".bYYYYYYwbbwYb..",
    "bYYYYYYYwbbwYYb.",
    "bYYYYYYYwwwwYYbb",
    "bYYYYYYYYYYYbOOb",
    "bYYYYYYYYYYYbOOb",
    "bYYYYYYYYYYYbOOb",
    ".bYYYYYYYYYYbbb.",
    ".bYYYYYYYYYYb...",
    "..bYYYYYYYYb....",
    "...bbYYYYYb.....",
    ".....bbbbb......",
    "................",
}};

static const sprite_t g_spr_cloud =
{{
    "................",
    "................",
    "......ccc.......",
    "....ccccccc.....",
    "...ccccccccc....",
    "..cccccccccccc..",
    ".cccccccccccccc.",
    "cccccccccccccccc",
    "CCCCCCCCCCCCCCCC",
    ".CCCCCCCCCCCCCC.",
    "................",
    "................",
    "................",
    "................",
    "................",
    "................",
}};

// One medal, recoloured at the two plumage characters for silver and bronze
static const sprite_t g_spr_medal =
{{
    "................",
    "................",
    "....bbbbbb......",
    "..bbYYYYYYbb....",
    ".bYYYYYYYYYYb...",
    ".bYYYYwwYYYYb...",
    "bYYYYwwwwYYYYb..",
    "bYYYwwwwwwYYYb..",
    "bYYYYwwwwYYYYb..",
    "bYYYYYwwYYYYYb..",
    ".bYYYyyyyYYYb...",
    ".bYYyyyyyyYYb...",
    "..bbyyyyyybb....",
    "....bbbbbb......",
    "................",
    "................",
}};

/*- Variables ---------------------------------------------------------------*/
static const uint16_t g_sky[SKY_BANDS] =
{
    LCD_COLOR( 74, 144, 226),
    LCD_COLOR( 92, 158, 232),
    LCD_COLOR(112, 174, 238),
    LCD_COLOR(134, 190, 242),
    LCD_COLOR(158, 206, 246),
    LCD_COLOR(184, 222, 250),
};

static pipe_t   g_pipes[MAX_PIPES];
static cloud_t  g_clouds[MAX_CLOUDS];

static float    g_bird_y;
static float    g_bird_v;
static int      g_bird_drawn_y;

static state_t  g_state;
static int      g_score;
static int      g_best;
static bool     g_new_best;
static int      g_ground_scroll;
static int      g_anim_ms;
static int      g_wing;             // 0 up, 1 level, 2 down

static int      g_frame_timer = TIMER_DISABLE;

static int      g_difficulty = DIFF_NORMAL;
static bool     g_ramp = true;

static const int g_base_gap[] = { 92, 76, 62 };
static const int g_base_speed[] = { 2, 3, 4 };

/*- Forward Declarations ----------------------------------------------------*/
static void draw_scene(void);
static void restore_rect(int x, int y, int w, int h);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Difficulty
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static int ramp_steps(void)
{
    int steps;

    if (!g_ramp)
        return 0;

    steps = g_score / RAMP_EVERY;

    return (steps > RAMP_STEPS) ? RAMP_STEPS : steps;
}

//-----------------------------------------------------------------------------
static int pipe_gap(void)
{
    int gap = g_base_gap[g_difficulty] - RAMP_GAP * ramp_steps();
    int floor_gap = BIRD + 22;

    return (gap < floor_gap) ? floor_gap : gap;
}

//-----------------------------------------------------------------------------
static int pipe_speed(void)
{
    return g_base_speed[g_difficulty] + ramp_steps() / RAMP_SPEED_AT;
}

//-----------------------------------------------------------------------------
// The scene, in layers
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void fill_sky(int x, int y, int w, int h)
{
    int band = SKY_H / SKY_BANDS;

    for (int i = 0; i < SKY_BANDS; i++)
    {
        int by = i * band;
        int bh = (SKY_BANDS - 1 == i) ? SKY_H - by : band;
        int top = (y > by) ? y : by;
        int bottom = (y + h < by + bh) ? y + h : by + bh;

        if (bottom > top)
            lcd_fill_rect(x, top, w, bottom - top, g_sky[i]);
    }
}

//-----------------------------------------------------------------------------
static bool overlaps(int ax, int aw, int bx, int bw)
{
    return ax < bx + bw && bx < ax + aw;
}

//-----------------------------------------------------------------------------
// Fills the staging tile with the sky the tile will sit against, row by row. A
// single band colour would show as a square of the wrong shade wherever the
// tile straddles two of them, which for a cloud is most of the time.
static void tile_sky_backdrop(int py)
{
    int band = SKY_H / SKY_BANDS;

    for (int j = 0; j < GFX_TILE; j++)
    {
        int sy = py + j;
        int i = (sy < 0) ? 0 : sy / band;

        if (i >= SKY_BANDS)
            i = SKY_BANDS - 1;

        gfx_rect(0, j, GFX_TILE, 1, g_sky[i]);
    }
}

//-----------------------------------------------------------------------------
static void draw_cloud(const cloud_t *c)
{
    tile_sky_backdrop(c->y);
    gfx_sprite(&g_spr_cloud);
    gfx_blit(c->x, c->y);
}

//-----------------------------------------------------------------------------
// Across a pipe or across its collar, the same rule: an edge either side, a
// bright band near the left, a dark one near the right. It is what makes a flat
// green rectangle read as a tube, and it is one function because the bird has
// to compose the very same colours behind itself.
static uint16_t tube_color(int i, int w)
{
    if (i <= 0 || i >= w - 1)           return C_PIPE_EDGE;
    if (i >= 3 && i <= 7)               return C_PIPE_HI;
    if (i >= w - 5 && i <= w - 3)       return C_PIPE_LO;

    return C_PIPE;
}

//-----------------------------------------------------------------------------
static void band(int x, int w, int y, int h, uint16_t color, int cx0, int cx1)
{
    int a = (x > cx0) ? x : cx0;
    int b = (x + w < cx1) ? x + w : cx1;

    if (b > a && h > 0)
        lcd_fill_rect(a, y, b - a, h, color);
}

//-----------------------------------------------------------------------------
// A shaded rectangle, emitted as runs of one colour. Scanning tube_color()
// rather than hard-coding the run boundaries keeps the pipes and the strip of
// pipe the bird draws behind itself from ever disagreeing.
static void draw_tube(int x, int w, int y, int h, int cx0, int cx1)
{
    int start = 0;

    if (h <= 0)
        return;

    for (int i = 1; i <= w; i++)
    {
        if (i < w && tube_color(i, w) == tube_color(start, w))
            continue;

        band(x + start, i - start, y, h, tube_color(start, w), cx0, cx1);
        start = i;
    }
}

//-----------------------------------------------------------------------------
// Sky back over a corner of a collar, so the mouth reads as moulded rather than
// sawn off. Clipped, because a collar can be half outside the strip being drawn.
static void cut_corner(int x, int w, int y, int cx0, int cx1)
{
    int a = (x > cx0) ? x : cx0;
    int b = (x + w < cx1) ? x + w : cx1;

    if (b > a)
        fill_sky(a, y, b - a, 1);
}

//-----------------------------------------------------------------------------
// A whole pipe, clipped to [cx0, cx1). Whole, and not the strip it scrolled
// into: shading a pipe across its width means no two of its columns are alike,
// so the usual "add a column at the front, drop one at the back" leaves the
// highlight standing where it was and trails a comb of stripes behind the pipe.
// All 38 columns is ~1.5 ms of panel time, against a 40 ms frame.
static void draw_pipe_part(const pipe_t *p, int cx0, int cx1)
{
    int gap = pipe_gap();
    int top_lip = p->gap_y - PIPE_LIP_H;
    int bot_lip = p->gap_y + gap;
    int lx = p->x - PIPE_LIP_OVER;
    int lw = PIPE_W + 2 * PIPE_LIP_OVER;

    draw_tube(p->x, PIPE_W, 0, top_lip, cx0, cx1);
    draw_tube(p->x, PIPE_W, bot_lip + PIPE_LIP_H,
        GROUND_Y - bot_lip - PIPE_LIP_H, cx0, cx1);

    draw_tube(lx, lw, top_lip, PIPE_LIP_H, cx0, cx1);
    draw_tube(lx, lw, bot_lip, PIPE_LIP_H, cx0, cx1);

    // The face of each mouth, and the seam where a collar meets its body -
    // without that line the collar reads as part of the pipe rather than a
    // fitting on the end of it
    band(lx, lw, p->gap_y - 1, 1, C_PIPE_EDGE, cx0, cx1);
    band(lx, lw, bot_lip, 1, C_PIPE_EDGE, cx0, cx1);
    band(lx, lw, top_lip, 1, C_PIPE_EDGE, cx0, cx1);
    band(lx, lw, bot_lip + PIPE_LIP_H - 1, 1, C_PIPE_EDGE, cx0, cx1);

    // ...and its four outer corners taken back off
    cut_corner(lx, 2, p->gap_y - 1, cx0, cx1);
    cut_corner(lx, 1, p->gap_y - 2, cx0, cx1);
    cut_corner(lx + lw - 2, 2, p->gap_y - 1, cx0, cx1);
    cut_corner(lx + lw - 1, 1, p->gap_y - 2, cx0, cx1);

    cut_corner(lx, 2, bot_lip, cx0, cx1);
    cut_corner(lx, 1, bot_lip + 1, cx0, cx1);
    cut_corner(lx + lw - 2, 2, bot_lip, cx0, cx1);
    cut_corner(lx + lw - 1, 1, bot_lip + 1, cx0, cx1);
}

//-----------------------------------------------------------------------------
static void draw_pipe(const pipe_t *p)
{
    draw_pipe_part(p, 0, LCD_WIDTH);
}

//-----------------------------------------------------------------------------
// Puts back everything that belongs in a rectangle of the sky, in the order it
// belongs in. Everything else in the file erases by calling this.
static void restore_rect(int x, int y, int w, int h)
{
    if (y + h > SKY_H)
        h = SKY_H - y;

    if (w <= 0 || h <= 0)
        return;

    fill_sky(x, y, w, h);

    for (int i = 0; i < MAX_CLOUDS; i++)
    {
        if (overlaps(x, w, g_clouds[i].x, GFX_TILE) &&
            overlaps(y, h, g_clouds[i].y, GFX_TILE))
        {
            // The whole cloud, not the sliver that was touched: it is one blit
            // either way, and the pipes are drawn after it
            draw_cloud(&g_clouds[i]);
        }
    }

    for (int i = 0; i < MAX_PIPES; i++)
    {
        if (overlaps(x, w, g_pipes[i].x - PIPE_LIP_OVER,
                PIPE_W + 2 * PIPE_LIP_OVER))
            draw_pipe_part(&g_pipes[i], x, x + w);
    }
}

//-----------------------------------------------------------------------------
static void draw_ground(void)
{
    lcd_fill_rect(0, GROUND_Y, LCD_WIDTH, 3, C_GROUND_EDGE);
    lcd_fill_rect(0, GROUND_Y + 3, LCD_WIDTH, 2, C_GROUND_EDGE_LO);
    lcd_fill_rect(0, GROUND_Y + 5, LCD_WIDTH, STRIPE_H, C_GROUND);
    lcd_fill_rect(0, GROUND_Y + 5 + STRIPE_H, LCD_WIDTH,
        GROUND_H - 5 - STRIPE_H, C_GROUND_DARK);
}

//-----------------------------------------------------------------------------
// Only the thin band scrolls. The whole 320x28 of ground repainted every frame
// is 1.8 ms of panel time for a motion cue that one stripe carries.
static void draw_ground_stripes(void)
{
    lcd_fill_rect(0, GROUND_Y + 5, LCD_WIDTH, STRIPE_H, C_GROUND);

    for (int x = -STRIPE_H; x < LCD_WIDTH; x += 16)
    {
        int sx = x + (g_ground_scroll % 16);

        // A parallelogram, drawn as a stack of one pixel rows, so the band
        // reads as moving rather than blinking
        for (int i = 0; i < STRIPE_H; i++)
            lcd_fill_rect(sx + STRIPE_H - i, GROUND_Y + 5 + i, 6, 1, C_STRIPE);
    }
}

//-----------------------------------------------------------------------------
// The bird, composed over whatever is behind it
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void draw_bird(void)
{
    static const int wing_y[] = { 5, 8, 10 };
    int y = (int)g_bird_y;
    int gap = pipe_gap();
    int wy = wing_y[g_wing];

    // Backdrop first: the sky it is against, then any pipe it is in front of.
    // Filling the tile with one flat colour would punch a hole through the pipe
    // the bird is passing, which is exactly when it is being looked at.
    tile_sky_backdrop(y);

    for (int p = 0; p < MAX_PIPES; p++)
    {
        for (int i = 0; i < GFX_TILE; i++)
        {
            int dx = BIRD_X + i - g_pipes[p].x;
            uint16_t color;

            if (dx < 0 || dx >= PIPE_W)
                continue;

            color = tube_color(dx, PIPE_W);

            for (int j = 0; j < GFX_TILE; j++)
            {
                int sy = y + j;

                if (sy < g_pipes[p].gap_y || sy >= g_pipes[p].gap_y + gap)
                    gfx_set(i, j, color);
            }
        }
    }

    gfx_sprite(&g_spr_bird);

    // The wing, at one of three heights: three rectangles rather than three
    // sprites of a bird that would differ in six rows
    gfx_rect(2, wy, 7, 4, C_WING);
    gfx_rect(3, wy + 1, 5, 1, C_WING_HI);
    gfx_set(2, wy, C_WING);

    gfx_blit(BIRD_X, y);
}

//-----------------------------------------------------------------------------
// The score, and the panels
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#define SCORE_Y             10
#define SCORE_H             34

static void draw_score(void)
{
    char buf[8];
    int x;

    snprintf(buf, sizeof(buf), "%d", g_score);
    x = (LCD_WIDTH - gfx_text_scaled_width(buf, 2)) / 2;

    // Outlined, not drop-shadowed. An offset copy of a digit falls inside the
    // counter of a 0 or an 8 and fills it in; an outline is what keeps white
    // digits legible against both a pale sky and a bright green pipe.
    for (int i = 0; i < 4; i++)
    {
        static const int ox[] = { -1, 1, 0, 0 };
        static const int oy[] = { 0, 0, -1, 1 };

        gfx_text_scaled_over(x + ox[i], SCORE_Y + oy[i], buf, 2, C_SHADE);
    }

    gfx_text_scaled_over(x, SCORE_Y, buf, 2, C_TEXT);
}

//-----------------------------------------------------------------------------
// Transparent text cannot erase the digit under it, and pipes scroll behind the
// score, so the patch goes back first. 96x34 of it is ~0.7 ms a frame.
static void repaint_score(void)
{
    restore_rect((LCD_WIDTH - 96) / 2, SCORE_Y - 2, 96, SCORE_H);
    draw_score();
}

//-----------------------------------------------------------------------------
static int medal_for(int score)
{
    if (score >= MEDAL_GOLD)   return 3;
    if (score >= MEDAL_SILVER) return 2;
    if (score >= MEDAL_BRONZE) return 1;

    return 0;
}

//-----------------------------------------------------------------------------
static void draw_panel(int w, int h, int *out_x, int *out_y)
{
    int x = (LCD_WIDTH - w) / 2;
    int y = (SKY_H - h) / 2;

    gfx_panel(x, y, w, h, C_PANEL, C_PANEL_EDGE, C_PANEL_SHADOW);

    *out_x = x;
    *out_y = y;
}

//-----------------------------------------------------------------------------
static void draw_title_panel(void)
{
    int x, y;

    draw_panel(264, 132, &x, &y);

    gfx_text_scaled(x + (264 - gfx_text_scaled_width("FLAPPY", 3)) / 2, y + 14,
        "FLAPPY", 3, C_TEXT, C_PANEL);

    gfx_draw_sprite(x + 24, y + 60, &g_spr_bird, C_PANEL);

    gfx_text(x + 52, y + 58, "MODE or UP  flap", FONT_SMALL, C_TEXT, C_PANEL);
    gfx_text(x + 52, y + 72, "MENU        settings", FONT_SMALL, C_TEXT_DIM,
        C_PANEL);
    gfx_text(x + 52, y + 86, "SHIFT+MENU  exit", FONT_SMALL, C_TEXT_DIM,
        C_PANEL);

    if (g_best > 0)
    {
        char buf[24];

        snprintf(buf, sizeof(buf), "BEST %d", g_best);
        gfx_text_centered(x + 132, y + 108, buf, FONT_SMALL, C_GOLD, C_PANEL);
    }
}

//-----------------------------------------------------------------------------
static void draw_over_panel(void)
{
    static const char *const g_medal_name[] =
        { "NO MEDAL", "BRONZE", "SILVER", "GOLD" };
    static const uint16_t g_medal_main[] =
    {
        LCD_COLOR(120, 128, 140), LCD_COLOR(198, 124,  58),
        LCD_COLOR(198, 206, 218), LCD_COLOR(255, 214,  66),
    };
    static const uint16_t g_medal_shade[] =
    {
        LCD_COLOR( 78,  86,  98), LCD_COLOR(140,  80,  32),
        LCD_COLOR(140, 148, 162), LCD_COLOR(206, 148,  16),
    };

    char buf[28];
    int medal = medal_for(g_score);
    int x, y;

    draw_panel(268, 146, &x, &y);

    gfx_text_scaled(x + (268 - gfx_text_scaled_width("GAME OVER", 2)) / 2,
        y + 14, "GAME OVER", 2, C_TEXT, C_PANEL);

    // One medal sprite, recoloured at its two plumage characters
    gfx_fill(C_PANEL);
    gfx_sprite_recolor(&g_spr_medal, g_medal_main[medal], g_medal_shade[medal]);
    gfx_blit(x + 26, y + 52);

    gfx_text(x + 52, y + 50, g_medal_name[medal], FONT_SMALL,
        medal ? C_GOLD : C_TEXT_DIM, C_PANEL);

    snprintf(buf, sizeof(buf), "SCORE %d", g_score);
    gfx_text(x + 52, y + 64, buf, FONT_SMALL, C_TEXT, C_PANEL);

    if (g_new_best)
        gfx_text(x + 52, y + 78, "NEW BEST", FONT_SMALL, C_GOLD, C_PANEL);
    else
    {
        snprintf(buf, sizeof(buf), "BEST %d", g_best);
        gfx_text(x + 52, y + 78, buf, FONT_SMALL, C_TEXT_DIM, C_PANEL);
    }

    gfx_text_centered(x + 134, y + 106, "MODE or UP   play again", FONT_SMALL,
        C_TEXT, C_PANEL);
    gfx_text_centered(x + 134, y + 120, "SHIFT+MENU   exit", FONT_SMALL,
        C_TEXT_DIM, C_PANEL);
}

//-----------------------------------------------------------------------------
static void draw_overlay(void)
{
    if (ST_TITLE == g_state)
        draw_title_panel();
    else if (ST_OVER == g_state)
        draw_over_panel();
}

//-----------------------------------------------------------------------------
static void draw_scene(void)
{
    fill_sky(0, 0, LCD_WIDTH, SKY_H);

    for (int i = 0; i < MAX_CLOUDS; i++)
        draw_cloud(&g_clouds[i]);

    for (int i = 0; i < MAX_PIPES; i++)
    {
        draw_pipe(&g_pipes[i]);
        g_pipes[i].drawn_x = g_pipes[i].x;
    }

    draw_ground();
    draw_ground_stripes();

    draw_bird();
    g_bird_drawn_y = (int)g_bird_y;

    draw_score();
    draw_overlay();
}

//-----------------------------------------------------------------------------
// The game
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void place_pipe(int index, int x)
{
    int span = SKY_H - pipe_gap() - 2 * GAP_MARGIN;

    g_pipes[index].x = x;
    g_pipes[index].gap_y = GAP_MARGIN + (span > 0 ? rand() % span : 0);
    g_pipes[index].passed = false;
}

//-----------------------------------------------------------------------------
static void reset_game(void)
{
    g_bird_y = SKY_H / 2 - BIRD / 2;
    g_bird_v = 0;
    g_bird_drawn_y = OFFSCREEN;

    g_score = 0;
    g_new_best = false;
    g_ground_scroll = 0;
    g_wing = 1;
    g_anim_ms = 0;

    for (int i = 0; i < MAX_PIPES; i++)
    {
        place_pipe(i, LCD_WIDTH + 80 + i * PIPE_SPACING);
        g_pipes[i].drawn_x = OFFSCREEN;
    }

    for (int i = 0; i < MAX_CLOUDS; i++)
    {
        g_clouds[i].x = 20 + rand() % (LCD_WIDTH - GFX_TILE - 40);
        g_clouds[i].y = 20 + rand() % (SKY_H / 2);
    }
}

//-----------------------------------------------------------------------------
static void record_best(void)
{
    if (g_score > g_best)
    {
        g_best = g_score;
        g_new_best = true;
    }
}

//-----------------------------------------------------------------------------
static void die(void)
{
    g_state = ST_DYING;
    g_anim_ms = 0;
    g_bird_v = FLAP_V / 2;      // one last bounce, then down

    record_best();

    lcd_fill_rect(0, 0, LCD_WIDTH, SKY_H, C_FLASH);
}

//-----------------------------------------------------------------------------
static bool bird_hits_pipe(void)
{
    int top = (int)g_bird_y + BIRD_BODY;
    int bottom = (int)g_bird_y + BIRD - BIRD_BODY;
    int left = BIRD_X + BIRD_BODY;
    int right = BIRD_X + BIRD - BIRD_BODY;
    int gap = pipe_gap();

    for (int i = 0; i < MAX_PIPES; i++)
    {
        if (!overlaps(left, right - left, g_pipes[i].x, PIPE_W))
            continue;

        if (top < g_pipes[i].gap_y || bottom > g_pipes[i].gap_y + gap)
            return true;
    }

    return false;
}

//-----------------------------------------------------------------------------
static void step_game(void)
{
    int speed = pipe_speed();
    int rightmost = g_pipes[0].x;
    int old_y = g_bird_drawn_y;
    int new_y;

    g_bird_v += GRAVITY;

    if (g_bird_v > MAX_FALL)
        g_bird_v = MAX_FALL;

    g_bird_y += g_bird_v;

    if (g_bird_y < 0)
    {
        g_bird_y = 0;
        g_bird_v = 0;
    }

    // The wing beats with the climb and holds out flat on the way down
    g_wing = (g_bird_v < -1.0f) ? 0 : ((g_bird_v > 3.0f) ? 2 : 1);

    for (int i = 1; i < MAX_PIPES; i++)
    {
        if (g_pipes[i].x > rightmost)
            rightmost = g_pipes[i].x;
    }

    for (int i = 0; i < MAX_PIPES; i++)
    {
        g_pipes[i].x -= speed;

        if (!g_pipes[i].passed && g_pipes[i].x + PIPE_W < BIRD_X)
        {
            g_pipes[i].passed = true;
            g_score++;
        }

        // One spacing behind the rightmost pipe, not at the screen edge: the
        // latter collapses the spacing to whatever order the pipes are in
        if (g_pipes[i].x + PIPE_W + PIPE_LIP_OVER < 0)
        {
            int spawn = rightmost + PIPE_SPACING;

            if (spawn < LCD_WIDTH)
                spawn = LCD_WIDTH;

            place_pipe(i, spawn);
            rightmost = spawn;
        }
    }

    if (g_bird_y > SKY_H - BIRD)
    {
        g_bird_y = SKY_H - BIRD;
        die();
        return;
    }

    if (bird_hits_pipe())
    {
        die();
        return;
    }

    // Repaint: what each pipe vacated, then the pipe, then the band the bird
    // left behind, then the bird.
    //
    // The whole pipe, not the strip it moved into. Shading a pipe across its
    // width is what makes it read as a tube, and it also means no two of its
    // columns are alike - so the usual scroll trick, "add a column at the front
    // and drop one at the back", leaves the highlight and the shadow standing
    // where they were. On screen that is a comb of stripes trailing the pipe.
    // Redrawing all 38 columns is ~1.5 ms of panel time per pipe, against a
    // 40 ms frame with at most three of them on screen.
    for (int i = 0; i < MAX_PIPES; i++)
    {
        pipe_t *p = &g_pipes[i];
        int width = PIPE_W + 2 * PIPE_LIP_OVER;

        if (p->drawn_x > OFFSCREEN)
        {
            int old_left = p->drawn_x - PIPE_LIP_OVER;
            int gone = p->drawn_x - p->x;

            if (gone <= 0 || gone > width)
                gone = width;   // a respawn, not a step: erase all of it

            restore_rect(old_left + width - gone, 0, gone, SKY_H);
        }

        draw_pipe(p);
        p->drawn_x = p->x;
    }

    new_y = (int)g_bird_y;

    if (old_y > OFFSCREEN && old_y != new_y)
    {
        int dy = new_y - old_y;

        if (dy >= BIRD || -dy >= BIRD)
            restore_rect(BIRD_X, old_y, BIRD, BIRD);
        else if (dy > 0)
            restore_rect(BIRD_X, old_y, BIRD, dy);
        else
            restore_rect(BIRD_X, new_y + BIRD, BIRD, -dy);
    }

    draw_bird();
    g_bird_drawn_y = new_y;

    g_ground_scroll -= speed;
    draw_ground_stripes();

    // Every frame, not only when it changed: pipes scroll behind the score and
    // punch holes through it
    repaint_score();
}

//-----------------------------------------------------------------------------
static void step_dying(void)
{
    int old_y = g_bird_drawn_y;
    int new_y;

    g_bird_v += GRAVITY;
    g_bird_y += g_bird_v;
    g_wing = 2;

    if (g_bird_y >= SKY_H - BIRD)
    {
        g_bird_y = SKY_H - BIRD;

        g_state = ST_OVER;

        if (g_best > config.flappy_high_score)
            config.flappy_high_score = g_best;
    }

    new_y = (int)g_bird_y;

    if (old_y > OFFSCREEN)
        restore_rect(BIRD_X, (old_y < new_y) ? old_y : new_y, BIRD,
            BIRD + ((old_y < new_y) ? new_y - old_y : old_y - new_y));

    draw_bird();
    g_bird_drawn_y = new_y;
    repaint_score();

    if (ST_OVER == g_state)
        draw_overlay();
}

//-----------------------------------------------------------------------------
static void flap(void)
{
    switch (g_state)
    {
        case ST_TITLE:
        case ST_OVER:
            reset_game();
            g_state = ST_PLAY;
            g_bird_v = FLAP_V;
            draw_scene();
            break;

        case ST_PLAY:
            g_bird_v = FLAP_V;
            g_wing = 0;
            break;

        default:
            break;
    }
}

//-----------------------------------------------------------------------------
// Application interface
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void flappy_bird_init(void)
{
    srand(timer_us()); // otherwise every run gets the same pipe layout

    g_best = config.flappy_high_score;

    if (g_best < 0 || g_best > 999999)
        g_best = 0;

    g_state = ST_TITLE;

    reset_game();

    timer_add(&g_frame_timer);
    g_frame_timer = FRAME_MS;

    draw_scene();
}

//-----------------------------------------------------------------------------
void flappy_bird_redraw(void)
{
    // A menu closed over the game. Handing a mid-flight bird back to someone
    // who was reading a menu is a run they did not get to lose themselves.
    if (ST_PLAY == g_state)
    {
        g_state = ST_TITLE;
        reset_game();
    }

    draw_scene();
}

//-----------------------------------------------------------------------------
void flappy_bird_task(void)
{
    if (0 != g_frame_timer)
        return;

    g_frame_timer = FRAME_MS;

    if (ST_PLAY == g_state)
    {
        step_game();
        return;
    }

    if (ST_DYING == g_state)
    {
        g_anim_ms += FRAME_MS;

        // The white flash of the hit holds for a moment before the fall
        if (g_anim_ms == DEATH_FLASH_MS)
            restore_rect(0, 0, LCD_WIDTH, SKY_H);

        if (g_anim_ms >= DEATH_FLASH_MS)
            step_dying();
    }
}

//-----------------------------------------------------------------------------
void flappy_bird_buttons_handler(int buttons)
{
    if ((buttons & (BTN_UP | BTN_MODE | BTN_STOP)) && !(buttons & BTN_REPEAT))
        flap();
}

//-----------------------------------------------------------------------------
void flappy_bird_cleanup(void)
{
    g_frame_timer = TIMER_DISABLE;
    timer_remove(&g_frame_timer);
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void settings_changed(void)
{
    g_state = ST_TITLE;
    reset_game();
}

//-----------------------------------------------------------------------------
static void action_restart(const void *arg)
{
    (void)arg;

    settings_changed();
    menu_close_popups(); // the game repaints itself when the popup closes
}

//-----------------------------------------------------------------------------
static void action_reset_best(const void *arg)
{
    (void)arg;

    g_best = 0;
    g_new_best = false;
    config.flappy_high_score = 0;
    menu_close_popups();
}

static const char *const g_diff_labels[] = { "Easy", "Normal", "Hard" };

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "Difficulty",
      .u.choice = { &g_difficulty, g_diff_labels, ARRAY_SIZE(g_diff_labels),
                    settings_changed } },
    { .kind = MI_TOGGLE, .label = "Tighten as you score",
      .u.toggle = { &g_ramp, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Restart",
      .u.action = { action_restart, NULL } },
    { .kind = MI_ACTION, .label = "Clear best score",
      .u.action = { action_reset_best, NULL } },
};

const menu_def_t flappy_bird_menu =
{
    .title = "Flappy Bird",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};

//-----------------------------------------------------------------------------
static const char *const g_help_lines[] =
{
    INFO_HEAD "CONTROLS",
    "MODE, UP or STOP - Flap / start / play again",
    "MENU             - This menu",
    "SHIFT+MENU       - Back to the launcher",
    "",
    INFO_HEAD "SCORING",
    "One point per pipe. Touching a pipe or the",
    "ground ends the run. The best score is kept",
    "in flash and survives a power cycle.",
    "",
    "Medals on the game over panel: bronze at 10,",
    "silver at 25, gold at 50.",
    "",
    INFO_HEAD "DIFFICULTY",
    "Easy, Normal and Hard set the gap the run",
    "starts with and how fast the pipes come.",
    "",
    "With \"Tighten as you score\" on, the gap",
    "closes by 4 px every 6 points and the pipes",
    "gain speed, up to six steps - so a good run",
    "keeps asking for more rather than settling",
    "into a rhythm. Turn it off for the plain",
    "game at a fixed difficulty.",
};

static const info_page_t g_help_page =
{
    .title = "Flappy Bird",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const menu_item_t g_help_items[] =
{
    { .kind = MI_ACTION, .label = "Controls and rules",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t flappy_bird_help_menu =
{
    .title = "Flappy Bird",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
