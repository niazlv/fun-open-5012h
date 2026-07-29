/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Snake
 *
 * The playfield is a 20x13 grid of 16 px cells over a checkerboard, and every
 * cell is composed in a 16x16 staging buffer before it is blitted. That is the
 * whole rendering strategy: one lcd_draw_buf() per cell costs the same as the
 * lcd_fill_rect() the old flat version used, but it buys rounded joints, a
 * shaded body, eyes that look where the snake is going and sprite fruit - none
 * of which can be drawn out of axis-aligned rectangles.
 *
 * A step touches four cells (vacated tail, new tail, the head that became a
 * neck, the new head), so the cost of a step is independent of the length of
 * the snake. A full repaint of the 320x240 bit-banged panel is ~15 ms, which is
 * why it only happens on a restart or when an overlay closed over the game.
 *
 * Artwork and levels are in snake_assets.c.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"   // utils.h reaches for __RBIT out of CMSIS
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "utils.h"
#include "config.h"
#include "ui.h"
#include "menu_widget.h"
#include "game_gfx.h"
#include "snake_assets.h"
#include "snake_game.h"

/*- Definitions -------------------------------------------------------------*/
#define CELL                GFX_TILE
#define GRID_W              SNAKE_GRID_W
#define GRID_H              SNAKE_GRID_H

#define HUD_H               32
#define FIELD_Y             HUD_H
#define FIELD_W             (GRID_W * CELL)     // 320
#define FIELD_H             (GRID_H * CELL)     // 208

#define MAX_SNAKE           (GRID_W * GRID_H)

// Housekeeping tick: countdowns, the bonus clock, the death flash. Short
// enough that none of them look stepped, long enough to cost nothing.
#define ANIM_MS             40

#define FRUIT_SCORE         10
#define BONUS_SCORE         50
#define BONUS_EVERY         4               // fruit between bonus spawns
#define BONUS_MS            9000
#define BONUS_WARN_MS       3000            // it starts blinking with this left
#define BONUS_BLINK_MS      200

#define READY_MS            600             // per countdown digit
#define DEATH_FLASHES       6
#define DEATH_FLASH_MS      90
#define CLEAR_MS            2000            // level banner, before it advances

// Speed setting, as a percentage of the level's own step interval
#define SPEED_SLOW          0
#define SPEED_NORMAL        1
#define SPEED_FAST          2
#define SPEED_INSANE        3

#define MODE_LEVELS         0
#define MODE_CLASSIC        1

#define CLASSIC_STEP_MS     150
#define MIN_STEP_MS         60

/*- Colors ------------------------------------------------------------------*/
#define C_FIELD_A           LCD_COLOR(170, 215,  81)
#define C_FIELD_B           LCD_COLOR(162, 209,  73)
#define C_HUD               LCD_COLOR( 74, 117,  44)
#define C_HUD_EDGE          LCD_COLOR( 52,  84,  30)
#define C_TEXT              LCD_COLOR(255, 255, 255)
#define C_TEXT_DIM          LCD_COLOR(198, 226, 160)

#define C_SNAKE             LCD_COLOR( 70, 117, 235)
#define C_SNAKE_TOP         LCD_COLOR(126, 168, 255)
#define C_SNAKE_BOT         LCD_COLOR( 38,  72, 176)
#define C_DEAD              LCD_COLOR(226,  62,  40)
#define C_DEAD_TOP          LCD_COLOR(255, 140, 120)
#define C_DEAD_BOT          LCD_COLOR(150,  24,  12)
#define C_EYE               LCD_COLOR(255, 255, 255)
#define C_PUPIL             LCD_COLOR( 24,  28,  44)

#define C_WALL              LCD_COLOR(104, 108, 118)
#define C_WALL_TOP          LCD_COLOR(154, 158, 168)
#define C_WALL_BOT          LCD_COLOR( 62,  66,  76)

#define C_PANEL             LCD_COLOR( 26,  44,  16)
#define C_PANEL_EDGE        LCD_COLOR(126, 176,  74)
#define C_SHADOW            LCD_COLOR( 92, 128,  44)
#define C_GOLD              LCD_COLOR(255, 214,  66)

/*- Types -------------------------------------------------------------------*/
typedef enum
{
    DIR_UP = 0,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT,
} direction_t;

typedef enum
{
    ST_TITLE = 0,   // the panel that comes up when the app is opened
    ST_READY,       // 3-2-1 before the snake starts moving
    ST_PLAY,
    ST_PAUSE,
    ST_DYING,       // the snake flashes before the game over panel
    ST_OVER,
    ST_CLEAR,       // level complete banner
} state_t;

enum
{
    CELL_EMPTY = 0,
    CELL_WALL,
    CELL_SNAKE,
    CELL_FRUIT,
    CELL_BONUS,
};

/*- Variables ---------------------------------------------------------------*/
static uint8_t  g_cell[GRID_H][GRID_W];
static uint16_t g_body[MAX_SNAKE];      // (y << 8) | x, index 0 is the head
static int      g_len;

static direction_t g_dir;               // the direction actually being travelled
static direction_t g_dir_queued;        // the last direction accepted into the queue
static uint8_t  g_dirq[4];
static int      g_dirq_count;

static state_t  g_state;
static int      g_score;
static int      g_best;
static int      g_level;                // index into snake_levels
static int      g_loop;                 // times the campaign has been round
static int      g_level_score;          // score when the level was entered
static int      g_eaten;                // fruit eaten in this level
static int      g_total_eaten;          // fruit since the last bonus
static bool     g_new_best;

static int      g_fruit_x, g_fruit_y;
static const sprite_t *g_fruit_spr;

static bool     g_bonus_on;
static bool     g_bonus_shown;          // blink state while it is expiring
static int      g_bonus_x, g_bonus_y;
static int      g_bonus_ms;

static int      g_anim_ms;              // time spent in the current animation
static int      g_anim_step;            // countdown digit / flash number
static bool     g_dying_bright;

// The rectangle the open overlay covers, so closing it repaints exactly that
static int      g_panel_x, g_panel_y, g_panel_w, g_panel_h;

static int      g_step_timer = TIMER_DISABLE;
static int      g_anim_timer = TIMER_DISABLE;

// Settings, offered in the application menu
static int      g_mode = MODE_LEVELS;
static int      g_speed = SPEED_NORMAL;
static bool     g_wrap = false;
static bool     g_bonus_enabled = true;

// Percentage of the level's own step interval, by speed setting
static const int g_speed_pct[] = { 150, 100, 78, 60 };

/*- Forward Declarations ----------------------------------------------------*/
static void paint_cell(int cx, int cy);
static void draw_field(void);
static void draw_hud(void);
static void draw_overlay(void);
static void start_run(void);
static void load_level(void);
static void spawn_fruit(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void tile_checker(int cx, int cy)
{
    gfx_fill(((cx + cy) & 1) ? C_FIELD_A : C_FIELD_B);
}

//-----------------------------------------------------------------------------
static void tile_eye(int x, int y, int px, int py, uint16_t body)
{
    gfx_rect(x, y, 4, 4, C_EYE);

    // Knocking the corners off is what makes a 4 px eye read as round
    gfx_set(x,     y,     body);
    gfx_set(x + 3, y,     body);
    gfx_set(x,     y + 3, body);
    gfx_set(x + 3, y + 3, body);

    gfx_rect(x + px, y + py, 2, 2, C_PUPIL);
}

//-----------------------------------------------------------------------------
static void tile_eyes(direction_t dir, uint16_t body)
{
    switch (dir)
    {
        // Set back one pixel from the snout: an eye against the rim of a
        // rounded head loses half of itself to the curve
        case DIR_RIGHT:
            tile_eye(8, 3, 2, 1, body);
            tile_eye(8, 9, 2, 1, body);
            break;

        case DIR_LEFT:
            tile_eye(4, 3, 0, 1, body);
            tile_eye(4, 9, 0, 1, body);
            break;

        case DIR_UP:
            tile_eye(3, 4, 1, 0, body);
            tile_eye(9, 4, 1, 0, body);
            break;

        default:
            tile_eye(3, 8, 1, 2, body);
            tile_eye(9, 8, 1, 2, body);
            break;
    }
}

//-----------------------------------------------------------------------------
// The model
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static inline int body_x(int i) { return g_body[i] & 0xff; }
static inline int body_y(int i) { return g_body[i] >> 8; }

//-----------------------------------------------------------------------------
static int body_index_at(int cx, int cy)
{
    uint16_t packed = (uint16_t)((cy << 8) | cx);

    for (int i = 0; i < g_len; i++)
    {
        if (g_body[i] == packed)
            return i;
    }

    return -1;
}

//-----------------------------------------------------------------------------
// Which side of cell a its neighbour b is on, wrapping if the walls do
static int link_mask(int ax, int ay, int bx, int by)
{
    int dx = bx - ax;
    int dy = by - ay;

    if (g_wrap)
    {
        if (dx > 1)  dx -= GRID_W;
        if (dx < -1) dx += GRID_W;
        if (dy > 1)  dy -= GRID_H;
        if (dy < -1) dy += GRID_H;
    }

    if (0 == dy && 1 == dx)  return GFX_RIGHT;
    if (0 == dy && -1 == dx) return GFX_LEFT;
    if (0 == dx && 1 == dy)  return GFX_DOWN;
    if (0 == dx && -1 == dy) return GFX_UP;

    return 0;
}

//-----------------------------------------------------------------------------
static int wall_mask(int cx, int cy)
{
    int mask = 0;

    if (cy > 0 && CELL_WALL == g_cell[cy - 1][cx])          mask |= GFX_UP;
    if (cy < GRID_H - 1 && CELL_WALL == g_cell[cy + 1][cx]) mask |= GFX_DOWN;
    if (cx > 0 && CELL_WALL == g_cell[cy][cx - 1])          mask |= GFX_LEFT;
    if (cx < GRID_W - 1 && CELL_WALL == g_cell[cy][cx + 1]) mask |= GFX_RIGHT;

    // A block on the edge of the panel is drawn square, so the wall reads as
    // continuing past the screen instead of stopping just short of it
    if (0 == cy) mask |= GFX_UP;
    if (GRID_H - 1 == cy) mask |= GFX_DOWN;
    if (0 == cx) mask |= GFX_LEFT;
    if (GRID_W - 1 == cx) mask |= GFX_RIGHT;

    return mask;
}

//-----------------------------------------------------------------------------
// Composes and blits one cell from the model, whatever is in it. Everything
// that changes the field goes through here, so the screen cannot disagree with
// the model about what a cell holds.
static void paint_cell(int cx, int cy)
{
    uint16_t body = (ST_DYING == g_state && g_dying_bright) ? C_DEAD : C_SNAKE;
    uint16_t top  = (ST_DYING == g_state && g_dying_bright) ? C_DEAD_TOP : C_SNAKE_TOP;
    uint16_t bot  = (ST_DYING == g_state && g_dying_bright) ? C_DEAD_BOT : C_SNAKE_BOT;

    tile_checker(cx, cy);

    switch (g_cell[cy][cx])
    {
        case CELL_WALL:
            gfx_shape_box(wall_mask(cx, cy), 0, 4);
            gfx_shape_paint(C_WALL, C_WALL_TOP, C_WALL_BOT);
            break;

        case CELL_FRUIT:
            gfx_sprite(g_fruit_spr);
            break;

        case CELL_BONUS:
            if (g_bonus_shown)
                gfx_sprite(&snake_spr_gold);
            break;

        case CELL_SNAKE:
        {
            int i = body_index_at(cx, cy);
            int mask = 0;

            if (i < 0)
                break;

            if (i > 0)
                mask |= link_mask(cx, cy, body_x(i - 1), body_y(i - 1));

            if (i < g_len - 1)
                mask |= link_mask(cx, cy, body_x(i + 1), body_y(i + 1));

            if (g_len > 1 && i == g_len - 1)
            {
                gfx_shape_taper(link_mask(cx, cy, body_x(i - 1), body_y(i - 1)));
                gfx_shape_paint(body, top, bot);
            }
            else if (0 == i)
            {
                gfx_shape_box(mask, 1, 6);
                gfx_shape_paint(body, top, bot);
                tile_eyes(g_dir, body);
            }
            else
            {
                gfx_shape_box(mask, 1, 5);
                gfx_shape_paint(body, top, bot);
            }
            break;
        }

        default:
            break;
    }

    gfx_blit(cx * CELL, FIELD_Y + cy * CELL);
}

//-----------------------------------------------------------------------------
static void draw_field(void)
{
    for (int cy = 0; cy < GRID_H; cy++)
    {
        for (int cx = 0; cx < GRID_W; cx++)
            paint_cell(cx, cy);
    }
}

//-----------------------------------------------------------------------------
// Repaints every cell a rectangle of the panel touches. Used to take an
// overlay back off the field without repainting all 260 cells for it.
static void repaint_region(int x, int y, int w, int h)
{
    int cx0 = x / CELL;
    int cx1 = (x + w - 1) / CELL;
    int cy0 = (y - FIELD_Y) / CELL;
    int cy1 = (y + h - 1 - FIELD_Y) / CELL;

    if (cx0 < 0) cx0 = 0;
    if (cy0 < 0) cy0 = 0;
    if (cx1 > GRID_W - 1) cx1 = GRID_W - 1;
    if (cy1 > GRID_H - 1) cy1 = GRID_H - 1;

    for (int cy = cy0; cy <= cy1; cy++)
    {
        for (int cx = cx0; cx <= cx1; cx++)
            paint_cell(cx, cy);
    }
}

//-----------------------------------------------------------------------------
// The status bar
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static const snake_level_t *current_level(void)
{
    // Classic is the campaign's own open field, played forever
    if (MODE_CLASSIC == g_mode)
        return &snake_levels[0];

    return &snake_levels[g_level];
}

//-----------------------------------------------------------------------------
static int level_target(void)
{
    if (MODE_CLASSIC == g_mode)
        return 0;

    return current_level()->target + 2 * g_loop;
}

//-----------------------------------------------------------------------------
static int step_interval(void)
{
    int ms = (MODE_CLASSIC == g_mode) ? CLASSIC_STEP_MS : current_level()->step_ms;

    ms = ms * g_speed_pct[g_speed] / 100;

    if (MODE_LEVELS == g_mode)
        ms -= 10 * g_loop;

    return (ms < MIN_STEP_MS) ? MIN_STEP_MS : ms;
}

//-----------------------------------------------------------------------------
static void draw_hud(void)
{
    char buf[48];

    lcd_fill_rect(0, 0, LCD_WIDTH, HUD_H, C_HUD);
    lcd_fill_rect(0, HUD_H - 2, LCD_WIDTH, 2, C_HUD_EDGE);

    gfx_draw_sprite(6, 8, &snake_spr_apple, C_HUD);

    snprintf(buf, sizeof(buf), "%d", g_score);
    gfx_text(28, 8, buf, FONT_LARGE, C_TEXT, C_HUD);

    if (MODE_CLASSIC == g_mode)
    {
        snprintf(buf, sizeof(buf), "CLASSIC - %s",
            g_wrap ? "WRAP" : "WALLS");
        gfx_text(112, 4, buf, FONT_SMALL, C_TEXT_DIM, C_HUD);

        snprintf(buf, sizeof(buf), "LENGTH %d", g_len);
        gfx_text(112, 16, buf, FONT_SMALL, C_TEXT_DIM, C_HUD);
    }
    else
    {
        snprintf(buf, sizeof(buf), "LEVEL %d - %s",
            g_level + 1, current_level()->name);
        gfx_text(112, 4, buf, FONT_SMALL, C_TEXT_DIM, C_HUD);

        snprintf(buf, sizeof(buf), "FRUIT %d/%d  LENGTH %d",
            g_eaten, level_target(), g_len);
        gfx_text(112, 16, buf, FONT_SMALL, C_TEXT_DIM, C_HUD);
    }

    snprintf(buf, sizeof(buf), "BEST %d", g_best);
    gfx_text(LCD_WIDTH - 4 - (int)strlen(buf) * (FONT_SMALL)->width, 10, buf,
        FONT_SMALL, C_GOLD, C_HUD);
}

//-----------------------------------------------------------------------------
// Overlay panels
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void panel_open(int w, int h)
{
    g_panel_w = w;
    g_panel_h = h;
    g_panel_x = (LCD_WIDTH - w) / 2;
    g_panel_y = FIELD_Y + (FIELD_H - h) / 2;

    gfx_panel(g_panel_x, g_panel_y, w, h, C_PANEL, C_PANEL_EDGE, C_SHADOW);
}

//-----------------------------------------------------------------------------
static void panel_close(void)
{
    if (0 == g_panel_w)
        return;

    repaint_region(g_panel_x, g_panel_y, g_panel_w + 4, g_panel_h + 4);
    g_panel_w = 0;
}

//-----------------------------------------------------------------------------
// The decorative snake on the title panel: the same cell renderer the game
// uses, so the title shows exactly what the player is about to steer
static void draw_title_snake(int px, int py)
{
    // Where each cell goes and which of its sides continue, tail first. The
    // cells carry their own column and row because the snake turns: the head
    // goes UNDER the cell that turns down, not one column past it, and a mask
    // that says two cells are joined does not make them touch on its own.
    static const struct { int8_t cx, cy, mask; } piece[] =
    {
        { 0, 0, GFX_RIGHT },              // tail
        { 1, 0, GFX_LEFT | GFX_RIGHT },
        { 2, 0, GFX_LEFT | GFX_RIGHT },
        { 3, 0, GFX_LEFT | GFX_DOWN },      // the turn
        { 3, 1, GFX_UP },                 // head, looking down at the apple
    };

    for (int i = 0; i < ARRAY_SIZE(piece); i++)
    {
        gfx_fill(C_PANEL);

        if (0 == i)
        {
            gfx_shape_taper(piece[i].mask);
            gfx_shape_paint(C_SNAKE, C_SNAKE_TOP, C_SNAKE_BOT);
        }
        else if (ARRAY_SIZE(piece) - 1 == i)
        {
            gfx_shape_box(piece[i].mask, 1, 6);
            gfx_shape_paint(C_SNAKE, C_SNAKE_TOP, C_SNAKE_BOT);
            tile_eyes(DIR_DOWN, C_SNAKE);
        }
        else
        {
            gfx_shape_box(piece[i].mask, 1, 5);
            gfx_shape_paint(C_SNAKE, C_SNAKE_TOP, C_SNAKE_BOT);
        }

        gfx_blit(px + piece[i].cx * CELL, py + piece[i].cy * CELL);
    }

    gfx_draw_sprite(px + 3 * CELL, py + 2 * CELL + 2, &snake_spr_apple, C_PANEL);
}

//-----------------------------------------------------------------------------
static void draw_title_panel(void)
{
    int x, y;

    panel_open(276, 168);
    x = g_panel_x;
    y = g_panel_y;

    gfx_text_scaled(x + (276 - gfx_text_scaled_width("SNAKE", 3)) / 2, y + 14,
        "SNAKE", 3, C_TEXT, C_PANEL);

    gfx_text_centered(x + 138, y + 66,
        (MODE_CLASSIC == g_mode) ? "CLASSIC - ENDLESS FIELD"
                                 : "CAMPAIGN - 8 LEVELS",
        FONT_SMALL, C_GOLD, C_PANEL);

    draw_title_snake(x + 24, y + 88);

    gfx_text(x + 132, y + 92, "MODE   start", FONT_SMALL, C_TEXT, C_PANEL);
    gfx_text(x + 132, y + 106, "arrows steer", FONT_SMALL, C_TEXT_DIM, C_PANEL);
    gfx_text(x + 132, y + 120, "MENU   settings", FONT_SMALL, C_TEXT_DIM, C_PANEL);
    gfx_text(x + 132, y + 134, "SHIFT+MENU exit", FONT_SMALL, C_TEXT_DIM, C_PANEL);

    if (g_best > 0)
    {
        char buf[32];

        snprintf(buf, sizeof(buf), "BEST %d", g_best);
        gfx_text_centered(x + 138, y + 150, buf, FONT_SMALL, C_GOLD, C_PANEL);
    }
}

//-----------------------------------------------------------------------------
static void draw_ready_panel(void)
{
    char buf[4];

    panel_open(96, 96);

    snprintf(buf, sizeof(buf), "%d", g_anim_step);
    gfx_text_scaled(g_panel_x + (96 - gfx_text_scaled_width(buf, 3)) / 2,
        g_panel_y + 12, buf, 3, C_GOLD, C_PANEL);

    gfx_text_centered(g_panel_x + 48, g_panel_y + 70, "GET READY", FONT_SMALL,
        C_TEXT, C_PANEL);
}

//-----------------------------------------------------------------------------
static void draw_pause_panel(void)
{
    panel_open(220, 96);

    gfx_text_scaled(g_panel_x + (220 - gfx_text_scaled_width("PAUSED", 2)) / 2,
        g_panel_y + 18, "PAUSED", 2, C_TEXT, C_PANEL);

    gfx_text_centered(g_panel_x + 110, g_panel_y + 58, "MODE  resume",
        FONT_SMALL, C_TEXT, C_PANEL);
    gfx_text_centered(g_panel_x + 110, g_panel_y + 72, "MENU  settings",
        FONT_SMALL, C_TEXT_DIM, C_PANEL);
}

//-----------------------------------------------------------------------------
static void draw_over_panel(void)
{
    char buf[40];
    int x, y;

    panel_open(268, 140);
    x = g_panel_x;
    y = g_panel_y;

    gfx_draw_sprite(x + 22, y + 20, &snake_spr_skull, C_PANEL);
    gfx_text_scaled(x + 52, y + 14, "GAME OVER", 2, C_TEXT, C_PANEL);

    snprintf(buf, sizeof(buf), "SCORE %d   LENGTH %d", g_score, g_len);
    gfx_text_centered(x + 134, y + 54, buf, FONT_SMALL, C_TEXT, C_PANEL);

    if (g_new_best)
        gfx_text_centered(x + 134, y + 70, "NEW BEST SCORE", FONT_SMALL,
            C_GOLD, C_PANEL);
    else
    {
        snprintf(buf, sizeof(buf), "BEST %d", g_best);
        gfx_text_centered(x + 134, y + 70, buf, FONT_SMALL, C_GOLD, C_PANEL);
    }

    if (MODE_LEVELS == g_mode)
    {
        snprintf(buf, sizeof(buf), "MODE  retry level %d", g_level + 1);
        gfx_text_centered(x + 134, y + 96, buf, FONT_SMALL, C_TEXT, C_PANEL);
        gfx_text_centered(x + 134, y + 110, "SHIFT+MODE  new run", FONT_SMALL,
            C_TEXT_DIM, C_PANEL);
    }
    else
    {
        gfx_text_centered(x + 134, y + 96, "MODE  play again", FONT_SMALL,
            C_TEXT, C_PANEL);
        gfx_text_centered(x + 134, y + 110, "SHIFT+MENU  exit", FONT_SMALL,
            C_TEXT_DIM, C_PANEL);
    }
}

//-----------------------------------------------------------------------------
static void draw_clear_panel(void)
{
    char buf[40];
    int next = (g_level + 1) % snake_level_count;

    panel_open(252, 112);

    snprintf(buf, sizeof(buf), "LEVEL %d", g_level + 1);
    gfx_text_scaled(g_panel_x + (252 - gfx_text_scaled_width(buf, 2)) / 2,
        g_panel_y + 14, buf, 2, C_TEXT, C_PANEL);

    gfx_text_centered(g_panel_x + 126, g_panel_y + 50, "CLEARED", FONT_SMALL,
        C_GOLD, C_PANEL);

    if (0 == next)
        snprintf(buf, sizeof(buf), "CAMPAIGN ROUND %d - FASTER", g_loop + 2);
    else
        snprintf(buf, sizeof(buf), "NEXT: %s", snake_levels[next].name);

    gfx_text_centered(g_panel_x + 126, g_panel_y + 72, buf, FONT_SMALL,
        C_TEXT, C_PANEL);
    gfx_text_centered(g_panel_x + 126, g_panel_y + 90, "MODE  continue",
        FONT_SMALL, C_TEXT_DIM, C_PANEL);
}

//-----------------------------------------------------------------------------
static void draw_overlay(void)
{
    switch (g_state)
    {
        case ST_TITLE: draw_title_panel(); break;
        case ST_READY: draw_ready_panel(); break;
        case ST_PAUSE: draw_pause_panel(); break;
        case ST_OVER:  draw_over_panel();  break;
        case ST_CLEAR: draw_clear_panel(); break;
        default: g_panel_w = 0; break;
    }
}

//-----------------------------------------------------------------------------
// Game logic
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void queue_direction(direction_t dir)
{
    static const direction_t opposite[] =
        { DIR_DOWN, DIR_UP, DIR_RIGHT, DIR_LEFT };

    // Validated against the last direction *queued*, not the one being
    // travelled: that is what lets a left-then-up flick inside a single step
    // come out as two turns instead of one turn and one ignored press
    if (dir == g_dir_queued || dir == opposite[g_dir_queued])
        return;

    if (g_dirq_count >= (int)ARRAY_SIZE(g_dirq))
        return;

    g_dirq[g_dirq_count++] = (uint8_t)dir;
    g_dir_queued = dir;
}

//-----------------------------------------------------------------------------
static direction_t take_direction(void)
{
    direction_t dir;

    if (0 == g_dirq_count)
        return g_dir;

    dir = (direction_t)g_dirq[0];

    for (int i = 1; i < g_dirq_count; i++)
        g_dirq[i - 1] = g_dirq[i];

    g_dirq_count--;

    return dir;
}

//-----------------------------------------------------------------------------
// Picks uniformly among the free cells rather than retrying random ones, so
// the last free cell of a nearly full field is found as reliably as the first
static bool free_cell(int *out_x, int *out_y)
{
    int free_count = 0;

    for (int cy = 0; cy < GRID_H; cy++)
    {
        for (int cx = 0; cx < GRID_W; cx++)
        {
            if (CELL_EMPTY == g_cell[cy][cx])
                free_count++;
        }
    }

    if (0 == free_count)
        return false;

    int pick = rand() % free_count;

    for (int cy = 0; cy < GRID_H; cy++)
    {
        for (int cx = 0; cx < GRID_W; cx++)
        {
            if (CELL_EMPTY != g_cell[cy][cx])
                continue;

            if (0 == pick--)
            {
                *out_x = cx;
                *out_y = cy;
                return true;
            }
        }
    }

    return false;
}

//-----------------------------------------------------------------------------
static void spawn_fruit(void)
{
    static const sprite_t *const fruit[] =
        { &snake_spr_apple, &snake_spr_orange, &snake_spr_berry };

    if (!free_cell(&g_fruit_x, &g_fruit_y))
        return;

    g_fruit_spr = fruit[rand() % ARRAY_SIZE(fruit)];
    g_cell[g_fruit_y][g_fruit_x] = CELL_FRUIT;
}

//-----------------------------------------------------------------------------
static void clear_bonus(void)
{
    if (!g_bonus_on)
        return;

    g_bonus_on = false;
    g_cell[g_bonus_y][g_bonus_x] = CELL_EMPTY;
    paint_cell(g_bonus_x, g_bonus_y);
}

//-----------------------------------------------------------------------------
static void spawn_bonus(void)
{
    if (!g_bonus_enabled || g_bonus_on)
        return;

    if (!free_cell(&g_bonus_x, &g_bonus_y))
        return;

    g_bonus_on = true;
    g_bonus_shown = true;
    g_bonus_ms = BONUS_MS;
    g_cell[g_bonus_y][g_bonus_x] = CELL_BONUS;
    paint_cell(g_bonus_x, g_bonus_y);
}

//-----------------------------------------------------------------------------
static void record_best(void)
{
    if (g_score <= g_best)
        return;

    g_best = g_score;
    g_new_best = true;
}

//-----------------------------------------------------------------------------
// Reads the map into the grid and puts the snake on its start cell. The body
// runs back from the head, so the map only has to keep the two cells behind
// the start marker clear.
static void load_level(void)
{
    const snake_level_t *level = current_level();
    int head_x = GRID_W / 2;
    int head_y = GRID_H / 2;

    g_dir = DIR_RIGHT;

    for (int cy = 0; cy < GRID_H; cy++)
    {
        for (int cx = 0; cx < GRID_W; cx++)
        {
            char c = level->rows[cy][cx];

            g_cell[cy][cx] = ('#' == c) ? CELL_WALL : CELL_EMPTY;

            switch (c)
            {
                case '>': g_dir = DIR_RIGHT; break;
                case '<': g_dir = DIR_LEFT;  break;
                case '^': g_dir = DIR_UP;    break;
                case 'v': g_dir = DIR_DOWN;  break;
                default: continue;
            }

            head_x = cx;
            head_y = cy;
        }
    }

    // If the border kills, the border is drawn. An open map with a lethal edge
    // is the one thing a player cannot learn from looking at the screen, and
    // the first version of this game died to it constantly; a map that already
    // has its own frame just gets the same cells set twice.
    if (!g_wrap)
    {
        for (int cx = 0; cx < GRID_W; cx++)
        {
            g_cell[0][cx] = CELL_WALL;
            g_cell[GRID_H - 1][cx] = CELL_WALL;
        }

        for (int cy = 0; cy < GRID_H; cy++)
        {
            g_cell[cy][0] = CELL_WALL;
            g_cell[cy][GRID_W - 1] = CELL_WALL;
        }
    }

    g_dir_queued = g_dir;
    g_dirq_count = 0;

    g_len = 3;

    for (int i = 0; i < g_len; i++)
    {
        int x = head_x;
        int y = head_y;

        switch (g_dir)
        {
            case DIR_RIGHT: x -= i; break;
            case DIR_LEFT:  x += i; break;
            case DIR_UP:    y += i; break;
            default:        y -= i; break;
        }

        if (x < 0) x += GRID_W;
        if (x >= GRID_W) x -= GRID_W;
        if (y < 0) y += GRID_H;
        if (y >= GRID_H) y -= GRID_H;

        g_body[i] = (uint16_t)((y << 8) | x);
        g_cell[y][x] = CELL_SNAKE;
    }

    g_eaten = 0;
    g_bonus_on = false;
    g_level_score = g_score;

    spawn_fruit();
}

//-----------------------------------------------------------------------------
static void enter_ready(void)
{
    g_state = ST_READY;
    g_anim_step = 3;
    g_anim_ms = 0;
    draw_overlay();
}

//-----------------------------------------------------------------------------
static void start_run(void)
{
    g_score = 0;
    g_level = 0;
    g_loop = 0;
    g_total_eaten = 0;
    g_new_best = false;

    load_level();
}

//-----------------------------------------------------------------------------
static void restart(bool whole_run)
{
    if (whole_run || MODE_CLASSIC == g_mode)
        start_run();
    else
    {
        g_score = g_level_score;
        load_level();
    }

    g_panel_w = 0;
    draw_hud();
    draw_field();
    enter_ready();
}

//-----------------------------------------------------------------------------
static void die(void)
{
    g_state = ST_DYING;
    g_anim_step = DEATH_FLASHES;
    g_anim_ms = 0;
    g_dying_bright = true;

    record_best();

    for (int i = 0; i < g_len; i++)
        paint_cell(body_x(i), body_y(i));
}

//-----------------------------------------------------------------------------
static void level_cleared(void)
{
    g_state = ST_CLEAR;
    g_anim_ms = 0;

    record_best();
    draw_overlay();
}

//-----------------------------------------------------------------------------
static void next_level(void)
{
    panel_close();

    g_level++;

    if (g_level >= snake_level_count)
    {
        g_level = 0;
        g_loop++;
    }

    load_level();
    draw_hud();
    draw_field();
    enter_ready();
}

//-----------------------------------------------------------------------------
// One step: move, test for a collision, and repaint only the cells that can
// have changed
static void step_game(void)
{
    int tail_x = body_x(g_len - 1);
    int tail_y = body_y(g_len - 1);
    int old_head_x = body_x(0);
    int old_head_y = body_y(0);
    int x = old_head_x;
    int y = old_head_y;
    bool ate_fruit, ate_bonus, grew;

    g_dir = take_direction();

    switch (g_dir)
    {
        case DIR_UP:    y--; break;
        case DIR_DOWN:  y++; break;
        case DIR_LEFT:  x--; break;
        default:        x++; break;
    }

    if (g_wrap)
    {
        if (x < 0) x = GRID_W - 1;
        if (x >= GRID_W) x = 0;
        if (y < 0) y = GRID_H - 1;
        if (y >= GRID_H) y = 0;
    }
    else if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
    {
        die();
        return;
    }

    if (CELL_WALL == g_cell[y][x])
    {
        die();
        return;
    }

    ate_fruit = (CELL_FRUIT == g_cell[y][x]);
    ate_bonus = (CELL_BONUS == g_cell[y][x]);
    grew = ate_fruit;

    // The tail cell is vacated by this very move, so running into it is legal -
    // unless this move also grows the snake, in which case it is not vacated
    if (CELL_SNAKE == g_cell[y][x] &&
        !(!grew && x == tail_x && y == tail_y))
    {
        die();
        return;
    }

    if (!grew)
    {
        g_cell[tail_y][tail_x] = CELL_EMPTY;
    }
    else if (g_len < MAX_SNAKE)
    {
        g_body[g_len] = g_body[g_len - 1];
        g_len++;
    }

    for (int i = g_len - 1; i > 0; i--)
        g_body[i] = g_body[i - 1];

    g_body[0] = (uint16_t)((y << 8) | x);
    g_cell[y][x] = CELL_SNAKE;

    if (ate_bonus)
    {
        g_bonus_on = false;
        g_score += BONUS_SCORE;
        draw_hud();
    }

    if (ate_fruit)
    {
        g_score += FRUIT_SCORE;
        g_eaten++;
        g_total_eaten++;

        spawn_fruit();
        paint_cell(g_fruit_x, g_fruit_y);
        draw_hud();

        if (0 == g_total_eaten % BONUS_EVERY)
            spawn_bonus();
    }

    if (!grew)
        paint_cell(tail_x, tail_y);

    paint_cell(body_x(g_len - 1), body_y(g_len - 1));
    paint_cell(old_head_x, old_head_y);
    paint_cell(x, y);

    if (MODE_LEVELS == g_mode && g_eaten >= level_target())
    {
        clear_bonus();
        level_cleared();
    }
}

//-----------------------------------------------------------------------------
static void anim_tick(void)
{
    g_anim_ms += ANIM_MS;

    switch (g_state)
    {
        case ST_READY:
            if (g_anim_ms < READY_MS)
                break;

            g_anim_ms = 0;
            g_anim_step--;

            if (g_anim_step > 0)
            {
                draw_ready_panel();
                break;
            }

            panel_close();
            g_state = ST_PLAY;
            g_step_timer = step_interval();
            break;

        case ST_DYING:
            if (g_anim_ms < DEATH_FLASH_MS)
                break;

            g_anim_ms = 0;
            g_anim_step--;
            g_dying_bright = !g_dying_bright;

            if (g_anim_step > 0)
            {
                for (int i = 0; i < g_len; i++)
                    paint_cell(body_x(i), body_y(i));
                break;
            }

            g_state = ST_OVER;

            if (g_best > config.snake_high_score)
                config.snake_high_score = g_best;

            draw_overlay();
            break;

        case ST_CLEAR:
            if (g_anim_ms >= CLEAR_MS)
                next_level();
            break;

        case ST_PLAY:
            if (!g_bonus_on)
                break;

            g_bonus_ms -= ANIM_MS;

            if (g_bonus_ms <= 0)
            {
                clear_bonus();
                break;
            }

            // It blinks out its last three seconds, so a bonus is something you
            // decide about rather than something that vanishes
            if (g_bonus_ms < BONUS_WARN_MS)
            {
                bool shown = ((g_bonus_ms / BONUS_BLINK_MS) & 1) != 0;

                if (shown != g_bonus_shown)
                {
                    g_bonus_shown = shown;
                    paint_cell(g_bonus_x, g_bonus_y);
                }
            }
            break;

        default:
            break;
    }
}

//-----------------------------------------------------------------------------
// Application interface
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void snake_game_init(void)
{
    srand(timer_us()); // otherwise every game gets the same fruit sequence

    g_best = config.snake_high_score;

    if (g_best < 0 || g_best > 999999)
        g_best = 0;

    g_state = ST_TITLE;
    g_panel_w = 0;

    start_run();

    timer_add(&g_step_timer);
    timer_add(&g_anim_timer);

    g_step_timer = step_interval();
    g_anim_timer = ANIM_MS;

    snake_game_redraw();
}

//-----------------------------------------------------------------------------
void snake_game_redraw(void)
{
    // Reached when a menu or a help page closed over the game. Whatever was on
    // screen is gone, and a player who was mid-run did not choose to be handed
    // back a moving snake, so the countdown comes back with it.
    if (ST_PLAY == g_state)
        enter_ready();

    g_panel_w = 0;

    draw_hud();
    draw_field();
    draw_overlay();
}

//-----------------------------------------------------------------------------
void snake_game_task(void)
{
    if (0 == g_anim_timer)
    {
        g_anim_timer = ANIM_MS;
        anim_tick();
    }

    if (0 == g_step_timer)
    {
        g_step_timer = step_interval();

        if (ST_PLAY == g_state)
            step_game();
    }
}

//-----------------------------------------------------------------------------
static void confirm_pressed(bool shift)
{
    switch (g_state)
    {
        case ST_TITLE:
            panel_close();
            enter_ready();
            break;

        case ST_PLAY:
            g_state = ST_PAUSE;
            draw_overlay();
            break;

        case ST_PAUSE:
            panel_close();
            enter_ready();
            break;

        case ST_OVER:
            restart(shift);
            break;

        case ST_CLEAR:
            next_level();
            break;

        default:
            break;
    }
}

//-----------------------------------------------------------------------------
void snake_game_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);

    if (buttons & BTN_UP)
        queue_direction(DIR_UP);
    else if (buttons & BTN_DOWN)
        queue_direction(DIR_DOWN);
    else if (buttons & BTN_LEFT)
        queue_direction(DIR_LEFT);
    else if (buttons & BTN_RIGHT)
        queue_direction(DIR_RIGHT);
    else if ((buttons & (BTN_MODE | BTN_STOP)) && !repeat)
        confirm_pressed(0 != (buttons & BTN_SHIFT));
}

//-----------------------------------------------------------------------------
void snake_game_cleanup(void)
{
    g_step_timer = TIMER_DISABLE;
    g_anim_timer = TIMER_DISABLE;
    timer_remove(&g_step_timer);
    timer_remove(&g_anim_timer);
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// A setting that changes the map or the rules cannot be applied to a run in
// progress, so it starts a new one. The popup repaints the game when it closes.
static void settings_changed(void)
{
    g_state = ST_TITLE;
    start_run();
}

//-----------------------------------------------------------------------------
static void action_restart(const void *arg)
{
    (void)arg;

    g_state = ST_TITLE;
    start_run();
    menu_close_popups();
}

//-----------------------------------------------------------------------------
static void action_reset_best(const void *arg)
{
    (void)arg;

    g_best = 0;
    g_new_best = false;
    config.snake_high_score = 0;
    menu_close_popups();
}

static const char *const g_mode_labels[] = { "Levels", "Classic" };
static const char *const g_speed_labels[] = { "Slow", "Normal", "Fast", "Insane" };

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "Game mode",
      .u.choice = { &g_mode, g_mode_labels, ARRAY_SIZE(g_mode_labels),
                    settings_changed } },
    { .kind = MI_CHOICE, .label = "Speed",
      .u.choice = { &g_speed, g_speed_labels, ARRAY_SIZE(g_speed_labels), NULL } },
    { .kind = MI_TOGGLE, .label = "Wrap at walls",
      .u.toggle = { &g_wrap, settings_changed } },
    { .kind = MI_TOGGLE, .label = "Bonus fruit",
      .u.toggle = { &g_bonus_enabled, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Restart",
      .u.action = { action_restart, NULL } },
    { .kind = MI_ACTION, .label = "Clear best score",
      .u.action = { action_reset_best, NULL } },
};

const menu_def_t snake_game_menu =
{
    .title = "Snake",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};

//-----------------------------------------------------------------------------
static const char *const g_help_lines[] =
{
    INFO_HEAD "CONTROLS",
    "UP/DOWN/LEFT/RIGHT - Steer. Presses are",
    "queued, so a two-turn flick between steps",
    "comes out as two turns.",
    "MODE (or STOP)     - Start / pause / retry",
    "SHIFT+MODE         - New run, from level 1",
    "MENU               - This menu",
    "SHIFT+MENU         - Back to the launcher",
    "",
    INFO_HEAD "SCORING",
    "Fruit is 10 points and one cell of length.",
    "The golden apple is 50, appears after every",
    "fourth fruit, and blinks out its last three",
    "seconds - it is a decision, not a gift.",
    "The best score is kept in flash and survives",
    "a power cycle.",
    "",
    INFO_HEAD "MODES",
    "Levels is a campaign of eight maps. Each one",
    "asks for a few more fruit than the last and",
    "runs 10 ms quicker; clearing the eighth",
    "starts the campaign again, faster, with",
    "higher targets.",
    "Classic is one open field forever, at a",
    "fixed speed.",
    "",
    INFO_HEAD "WALLS",
    "With Wrap off the border kills, so the game",
    "draws it: a ring of wall goes round the map",
    "whatever the map itself asks for. With Wrap",
    "on there is no ring and the snake comes back",
    "on the other side.",
    "",
    INFO_HEAD "SPEED",
    "The setting scales whatever the level asks",
    "for: Slow 150%, Normal 100%, Fast 78%,",
    "Insane 60% of its step interval.",
};

static const info_page_t g_help_page =
{
    .title = "Snake",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

// Read-only pages: the system menu shows them under Help, not among the
// settings above
static const menu_item_t g_help_items[] =
{
    { .kind = MI_ACTION, .label = "Controls and rules",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t snake_game_help_menu =
{
    .title = "Snake",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
