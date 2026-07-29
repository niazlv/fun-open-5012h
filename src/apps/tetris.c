/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Tetris
 *
 * The display is landscape and the well is not, which is what the layout is
 * built around: a 10x20 well of 11 px cells down the left, and the panel every
 * Tetris has - next, hold, score, level, lines - filling the width beside it.
 *
 * A piece is one 16-bit number: four rows of four bits in a box, one number per
 * rotation, 56 bytes for the whole set. Rotating is picking the next number,
 * and testing a position is walking sixteen bits. Nothing else in here needs to
 * know what an S looks like.
 *
 * Only the cells that changed are repainted. The board the player sees is the
 * locked cells plus the falling piece plus its ghost, composed into a shadow
 * and diffed against what is on the panel, so a frame costs the handful of
 * cells that moved rather than the 200 that did not.
 *
 * Built with -Os (see make/Makefile).
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
#include "tetris.h"

/*- Definitions -------------------------------------------------------------*/
#define COLS                10
#define ROWS                20
#define CELL                11

#define WELL_X              10
#define WELL_Y              9
#define WELL_W              (COLS * CELL)       // 110
#define WELL_H              (ROWS * CELL)       // 220

#define PANEL_X             (WELL_X + WELL_W + 16)

#define BOX                 4                   // a piece lives in a 4x4 box
#define PIECES              7

#define TICK_MS             20
#define LOCK_MS             400                 // grace before a landed piece sets
#define CLEAR_MS            60                  // per flash of a completed row
#define CLEAR_FLASHES       4

#define PREVIEW_CELL        9

/*- Colors ------------------------------------------------------------------*/
#define C_PAGE              LCD_COLOR( 16,  18,  28)
#define C_WELL              LCD_COLOR( 26,  30,  44)
#define C_GRID              LCD_COLOR( 38,  44,  62)
#define C_FRAME             LCD_COLOR( 92, 104, 140)
#define C_TEXT              LCD_COLOR(236, 240, 250)
#define C_DIM               LCD_COLOR(130, 142, 172)
#define C_GOLD              LCD_COLOR(255, 206,  84)

#define C_PANEL             LCD_COLOR( 30,  36,  56)
#define C_PANEL_EDGE        LCD_COLOR(120, 140, 190)
#define C_PANEL_SHADOW      LCD_COLOR( 10,  12,  20)

/*- Types -------------------------------------------------------------------*/
typedef enum
{
    ST_TITLE = 0,
    ST_PLAY,
    ST_CLEARING,    // completed rows flashing before they collapse
    ST_PAUSE,
    ST_OVER,
} state_t;

/*- Constants ---------------------------------------------------------------*/
// Bit i of a rotation is the cell (i % 4, i / 4) of the box. Four rotations a
// piece, in the order a clockwise turn visits them.
static const uint16_t g_shapes[PIECES][4] =
{
    { 0x00F0, 0x2222, 0x0F00, 0x4444 },     // I
    { 0x0066, 0x0066, 0x0066, 0x0066 },     // O
    { 0x0072, 0x0262, 0x0270, 0x0232 },     // T
    { 0x0036, 0x0462, 0x0360, 0x0231 },     // S
    { 0x0063, 0x0264, 0x0630, 0x0132 },     // Z
    { 0x0071, 0x0226, 0x0470, 0x0322 },     // J
    { 0x0074, 0x0622, 0x0170, 0x0223 },     // L
};

static const uint16_t g_piece_color[PIECES] =
{
    LCD_COLOR( 64, 200, 224),   // I, cyan
    LCD_COLOR(240, 208,  72),   // O, yellow
    LCD_COLOR(178, 102, 216),   // T, purple
    LCD_COLOR( 96, 204, 108),   // S, green
    LCD_COLOR(232,  84,  92),   // Z, red
    LCD_COLOR( 82, 122, 232),   // J, blue
    LCD_COLOR(240, 148,  64),   // L, orange
};

// A row is worth more the more of it goes at once, which is the whole game
static const int g_line_score[] = { 0, 100, 300, 500, 800 };

// Fall interval by level. Past the end of the table it stays where it is: at
// 60 ms a cell there is nothing left to take away.
static const int g_fall_table[] =
    { 800, 720, 630, 550, 470, 380, 300, 220, 160, 120, 100, 90, 80, 70, 60 };

/*- Variables ---------------------------------------------------------------*/
// The locked cells: 0 empty, otherwise the piece number plus one
static uint8_t  g_well[ROWS][COLS];
static uint8_t  g_shown[ROWS][COLS];    // what is on the panel
static bool     g_shown_valid;

static state_t  g_state;
static int      g_piece, g_rot, g_px, g_py;
static int      g_next, g_hold;
static bool     g_held_this_piece;

static uint8_t  g_bag[PIECES];
static int      g_bag_left;

static int      g_score, g_best, g_lines, g_level;
static bool     g_new_best;

static int      g_fall_owed;            // time owed to gravity
static int      g_lock_ms;              // time resting on the stack
static int      g_anim_ms;
static int      g_anim_step;
static uint8_t  g_full_row[ROWS];       // rows waiting to collapse

static int      g_tick_timer = TIMER_DISABLE;

static bool     g_ghost = true;
static int      g_start_level;

/*- Forward Declarations ----------------------------------------------------*/
static void draw_all(void);
static void draw_well(void);
static void draw_panel_side(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Blocks
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// One channel at a time, in RGB565. Two shades off one colour beats a table of
// three colours a piece, and it is the same bevel on the preview and the well.
static uint16_t shade(uint16_t c, int num, int den)
{
    int r = ((c >> 11) & 0x1f) * num / den;
    int g = ((c >> 5) & 0x3f) * num / den;
    int b = (c & 0x1f) * num / den;

    if (r > 31) r = 31;
    if (g > 63) g = 63;
    if (b > 31) b = 31;

    return (uint16_t)((r << 11) | (g << 5) | b);
}

//-----------------------------------------------------------------------------
// A block is a face with a lit top-left edge and a shaded bottom-right one.
// Two pixels of bevel on an 11 px cell is what stops a wall of one colour from
// reading as a single slab.
static void draw_block(int px, int py, int size, uint16_t color)
{
    uint16_t light = shade(color, 14, 10);
    uint16_t dark = shade(color, 6, 10);

    lcd_fill_rect(px, py, size, size, color);
    lcd_fill_rect(px, py, size, 2, light);
    lcd_fill_rect(px, py, 2, size, light);
    lcd_fill_rect(px, py + size - 2, size, 2, dark);
    lcd_fill_rect(px + size - 2, py, 2, size, dark);
}

//-----------------------------------------------------------------------------
static void draw_empty(int px, int py, int size)
{
    lcd_fill_rect(px, py, size, size, C_WELL);
    lcd_fill_rect(px, py, size, 1, C_GRID);
    lcd_fill_rect(px, py, 1, size, C_GRID);
}

//-----------------------------------------------------------------------------
// The ghost is the outline of where the piece would land. Without it a well
// this narrow is guesswork at the speeds the later levels run at.
static void draw_ghost(int px, int py, int size, uint16_t color)
{
    uint16_t c = shade(color, 5, 10);

    draw_empty(px, py, size);
    lcd_draw_rect(px + 1, py + 1, size - 2, size - 2, c);
}

//-----------------------------------------------------------------------------
// The model
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static bool piece_cell(int piece, int rot, int i)
{
    return 0 != (g_shapes[piece][rot & 3] & (1 << i));
}

//-----------------------------------------------------------------------------
// Would the piece sit here without leaving the well or touching anything
static bool fits(int piece, int rot, int px, int py)
{
    for (int i = 0; i < BOX * BOX; i++)
    {
        int x, y;

        if (!piece_cell(piece, rot, i))
            continue;

        x = px + i % BOX;
        y = py + i / BOX;

        if (x < 0 || x >= COLS || y >= ROWS)
            return false;

        // Above the ceiling is legal: a piece spawns partly off the top
        if (y >= 0 && g_well[y][x])
            return false;
    }

    return true;
}

//-----------------------------------------------------------------------------
// Seven pieces shuffled and dealt, rather than seven independent rolls. It is
// what stops a run being decided by an I that never came.
static int next_from_bag(void)
{
    if (0 == g_bag_left)
    {
        for (int i = 0; i < PIECES; i++)
            g_bag[i] = (uint8_t)i;

        for (int i = PIECES - 1; i > 0; i--)
        {
            int j = rand() % (i + 1);
            uint8_t t = g_bag[i];

            g_bag[i] = g_bag[j];
            g_bag[j] = t;
        }

        g_bag_left = PIECES;
    }

    return g_bag[--g_bag_left];
}

//-----------------------------------------------------------------------------
static int drop_row(void)
{
    int y = g_py;

    while (fits(g_piece, g_rot, g_px, y + 1))
        y++;

    return y;
}

//-----------------------------------------------------------------------------
static void spawn_piece(int piece)
{
    g_piece = piece;
    g_rot = 0;
    g_px = (COLS - BOX) / 2;
    g_py = -1;
    g_lock_ms = 0;

    // The spawn square is occupied, so there is nowhere left to put a piece
    if (!fits(g_piece, g_rot, g_px, g_py))
    {
        g_state = ST_OVER;

        if (g_score > g_best)
        {
            g_best = g_score;
            g_new_best = true;
        }

        if (g_best > config.tetris_high_score)
            config.tetris_high_score = g_best;
    }
}

//-----------------------------------------------------------------------------
static void take_next(void)
{
    int piece = g_next;

    g_next = next_from_bag();
    g_held_this_piece = false;
    spawn_piece(piece);
}

//-----------------------------------------------------------------------------
static int level_for(int lines)
{
    int level = g_start_level + lines / 10;

    return (level > 20) ? 20 : level;
}

//-----------------------------------------------------------------------------
static int fall_interval(void)
{
    int i = g_level;

    if (i >= ARRAY_SIZE(g_fall_table))
        i = ARRAY_SIZE(g_fall_table) - 1;

    return g_fall_table[i];
}

//-----------------------------------------------------------------------------
// Locks the piece into the well and marks any row it completed
static int lock_piece(void)
{
    int full = 0;

    for (int i = 0; i < BOX * BOX; i++)
    {
        int x, y;

        if (!piece_cell(g_piece, g_rot, i))
            continue;

        x = g_px + i % BOX;
        y = g_py + i / BOX;

        if (y >= 0 && y < ROWS && x >= 0 && x < COLS)
            g_well[y][x] = (uint8_t)(g_piece + 1);
    }

    memset(g_full_row, 0, sizeof(g_full_row));

    for (int y = 0; y < ROWS; y++)
    {
        int n = 0;

        for (int x = 0; x < COLS; x++)
        {
            if (g_well[y][x])
                n++;
        }

        if (COLS == n)
        {
            g_full_row[y] = 1;
            full++;
        }
    }

    return full;
}

//-----------------------------------------------------------------------------
static void collapse_rows(void)
{
    int write = ROWS - 1;

    for (int y = ROWS - 1; y >= 0; y--)
    {
        if (g_full_row[y])
            continue;

        if (write != y)
            memcpy(g_well[write], g_well[y], COLS);

        write--;
    }

    while (write >= 0)
        memset(g_well[write--], 0, COLS);
}

//-----------------------------------------------------------------------------
static void new_game(void)
{
    memset(g_well, 0, sizeof(g_well));

    g_score = 0;
    g_lines = 0;
    g_level = g_start_level;
    g_new_best = false;
    g_hold = -1;
    g_bag_left = 0;
    g_fall_owed = 0;
    g_anim_ms = 0;
    g_shown_valid = false;

    g_next = next_from_bag();
    take_next();
}

//-----------------------------------------------------------------------------
// Drawing
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// What the player should be looking at: the stack, the piece in the air, and
// the ghost under it. Composed into one grid so the panel can be diffed
// against the last one rather than repainted.
//
// A cell holds the piece number plus one, and the ghost adds 0x80 on top, so
// one byte carries both what colour it is and how it should be drawn.
static void compose(uint8_t *out)
{
    memcpy(out, g_well, ROWS * COLS);

    if (ST_PLAY != g_state && ST_PAUSE != g_state)
        return;

    if (g_ghost)
    {
        int gy = drop_row();

        for (int i = 0; i < BOX * BOX; i++)
        {
            int x = g_px + i % BOX;
            int y = gy + i / BOX;

            if (!piece_cell(g_piece, g_rot, i))
                continue;

            if (y >= 0 && y < ROWS && x >= 0 && x < COLS && !out[y * COLS + x])
                out[y * COLS + x] = (uint8_t)(0x80 | (g_piece + 1));
        }
    }

    for (int i = 0; i < BOX * BOX; i++)
    {
        int x = g_px + i % BOX;
        int y = g_py + i / BOX;

        if (!piece_cell(g_piece, g_rot, i))
            continue;

        if (y >= 0 && y < ROWS && x >= 0 && x < COLS)
            out[y * COLS + x] = (uint8_t)(g_piece + 1);
    }
}

//-----------------------------------------------------------------------------
static void draw_grid_cell(int x, int y, uint8_t v)
{
    int px = WELL_X + x * CELL;
    int py = WELL_Y + y * CELL;

    if (0 == v)
        draw_empty(px, py, CELL);
    else if (v & 0x80)
        draw_ghost(px, py, CELL, g_piece_color[(v & 0x7f) - 1]);
    else
        draw_block(px, py, CELL, g_piece_color[v - 1]);
}

//-----------------------------------------------------------------------------
static void draw_well(void)
{
    uint8_t want[ROWS * COLS];

    compose(want);

    if (!g_shown_valid)
    {
        lcd_draw_rect(WELL_X - 2, WELL_Y - 2, WELL_W + 4, WELL_H + 4, C_FRAME);
        memset(g_shown, 0xff, sizeof(g_shown));
    }

    for (int y = 0; y < ROWS; y++)
    {
        for (int x = 0; x < COLS; x++)
        {
            if (g_shown[y][x] == want[y * COLS + x])
                continue;

            draw_grid_cell(x, y, want[y * COLS + x]);
            g_shown[y][x] = want[y * COLS + x];
        }
    }

    g_shown_valid = true;
}

//-----------------------------------------------------------------------------
// A piece drawn small, centred in a box. Used for next and for hold, which is
// the only reason either of them costs anything.
static void draw_preview(int px, int py, int w, int h, int piece)
{
    int lo = BOX, hi = -1, top = BOX, bottom = -1;
    int ox, oy;

    lcd_fill_rect(px, py, w, h, C_WELL);
    lcd_draw_rect(px, py, w, h, C_GRID);

    if (piece < 0)
        return;

    for (int i = 0; i < BOX * BOX; i++)
    {
        int x = i % BOX;
        int y = i / BOX;

        if (!piece_cell(piece, 0, i))
            continue;

        if (x < lo) lo = x;
        if (x > hi) hi = x;
        if (y < top) top = y;
        if (y > bottom) bottom = y;
    }

    ox = px + (w - (hi - lo + 1) * PREVIEW_CELL) / 2 - lo * PREVIEW_CELL;
    oy = py + (h - (bottom - top + 1) * PREVIEW_CELL) / 2 - top * PREVIEW_CELL;

    for (int i = 0; i < BOX * BOX; i++)
    {
        if (piece_cell(piece, 0, i))
            draw_block(ox + (i % BOX) * PREVIEW_CELL,
                oy + (i / BOX) * PREVIEW_CELL, PREVIEW_CELL,
                g_piece_color[piece]);
    }
}

//-----------------------------------------------------------------------------
static void draw_stat(int y, const char *label, int value)
{
    char buf[16];

    gfx_text(PANEL_X, y, label, FONT_SMALL, C_DIM, C_PAGE);

    snprintf(buf, sizeof(buf), "%d", value);
    lcd_fill_rect(PANEL_X, y + 11, 96, 16, C_PAGE);
    gfx_text(PANEL_X, y + 11, buf, FONT_LARGE, C_TEXT, C_PAGE);
}

//-----------------------------------------------------------------------------
static void draw_panel_side(void)
{
    char buf[24];

    gfx_text_scaled(PANEL_X, 8, "TETRIS", 2, C_TEXT, C_PAGE);

    gfx_text(PANEL_X, 46, "NEXT", FONT_SMALL, C_DIM, C_PAGE);
    draw_preview(PANEL_X, 58, 52, 44, g_next);

    gfx_text(PANEL_X + 68, 46, "HOLD", FONT_SMALL, C_DIM, C_PAGE);
    draw_preview(PANEL_X + 68, 58, 52, 44, g_hold);

    draw_stat(114, "SCORE", g_score);
    draw_stat(152, "LEVEL", g_level);
    draw_stat(190, "LINES", g_lines);

    snprintf(buf, sizeof(buf), "BEST %d", g_best);
    lcd_fill_rect(PANEL_X, 224, 140, 8, C_PAGE);
    gfx_text(PANEL_X, 224, buf, FONT_SMALL, C_GOLD, C_PAGE);
}

//-----------------------------------------------------------------------------
static void draw_message(const char *title, const char *l1, const char *l2)
{
    int w = 176;
    int h = 104;
    int x = PANEL_X - 26;
    int y = (LCD_HEIGHT - h) / 2;

    gfx_panel(x, y, w, h, C_PANEL, C_PANEL_EDGE, C_PANEL_SHADOW);

    gfx_text_scaled(x + (w - gfx_text_scaled_width(title, 2)) / 2, y + 14,
        title, 2, C_TEXT, C_PANEL);

    gfx_text_centered(x + w / 2, y + 54, l1, FONT_SMALL, C_TEXT, C_PANEL);

    if (l2)
        gfx_text_centered(x + w / 2, y + 70, l2, FONT_SMALL, C_GOLD, C_PANEL);

    gfx_text_centered(x + w / 2, y + 86, "MENU  settings", FONT_SMALL, C_DIM,
        C_PANEL);
}

//-----------------------------------------------------------------------------
static void draw_overlay(void)
{
    char buf[24];

    switch (g_state)
    {
        case ST_TITLE:
            // Not "TETRIS" again: the panel beside it already says so
            draw_message("READY?", "MODE  start", "arrows steer");
            break;

        case ST_PAUSE:
            draw_message("PAUSED", "STOP  resume", NULL);
            break;

        case ST_OVER:
            snprintf(buf, sizeof(buf), "SCORE %d", g_score);
            draw_message("GAME OVER", buf,
                g_new_best ? "NEW BEST" : "MODE  play again");
            break;

        default:
            break;
    }
}

//-----------------------------------------------------------------------------
static void draw_all(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, C_PAGE);

    g_shown_valid = false;

    draw_well();
    draw_panel_side();
    draw_overlay();
}

//-----------------------------------------------------------------------------
// Play
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void land_piece(void)
{
    int full = lock_piece();

    if (full > 0)
    {
        g_lines += full;
        g_score += g_line_score[full] * (g_level + 1);
        g_level = level_for(g_lines);

        g_state = ST_CLEARING;
        g_anim_ms = 0;
        g_anim_step = 0;

        // The rows that go are shown going: without the flash a four line
        // clear and a single look exactly the same
        g_shown_valid = false;
        draw_well();
        draw_panel_side();
        return;
    }

    take_next();
    draw_well();
}

//-----------------------------------------------------------------------------
static void step_fall(bool soft)
{
    if (fits(g_piece, g_rot, g_px, g_py + 1))
    {
        g_py++;
        g_lock_ms = 0;

        if (soft)
            g_score++;

        draw_well();
        return;
    }

    // Resting on the stack is not landing yet: a piece can still be slid or
    // spun for a moment, which is what makes a tuck under an overhang possible
    if (g_lock_ms >= LOCK_MS)
        land_piece();
}

//-----------------------------------------------------------------------------
static void try_move(int dx)
{
    if (fits(g_piece, g_rot, g_px + dx, g_py))
    {
        g_px += dx;
        g_lock_ms = 0;
        draw_well();
    }
}

//-----------------------------------------------------------------------------
// Rotation, with a nudge sideways if the turn does not fit where it stands.
// Without the kicks a piece against a wall simply refuses to turn, which reads
// as the game ignoring the button.
static void try_rotate(void)
{
    static const int kick[] = { 0, -1, 1, -2, 2 };
    int rot = (g_rot + 1) & 3;

    for (int i = 0; i < ARRAY_SIZE(kick); i++)
    {
        if (!fits(g_piece, rot, g_px + kick[i], g_py))
            continue;

        g_rot = rot;
        g_px += kick[i];
        g_lock_ms = 0;
        draw_well();
        return;
    }
}

//-----------------------------------------------------------------------------
static void hard_drop(void)
{
    int y = drop_row();

    g_score += 2 * (y - g_py);
    g_py = y;

    land_piece();
    draw_panel_side();
}

//-----------------------------------------------------------------------------
// Puts the falling piece aside and brings back whatever was there. Once per
// piece, or it is a way to stall forever.
static void hold_piece(void)
{
    int piece = g_piece;

    if (g_held_this_piece)
        return;

    if (g_hold < 0)
    {
        g_hold = piece;
        take_next();
    }
    else
    {
        int swap = g_hold;

        g_hold = piece;
        spawn_piece(swap);
    }

    g_held_this_piece = true;

    g_shown_valid = false;
    draw_well();
    draw_panel_side();
}

//-----------------------------------------------------------------------------
static void start_game(void)
{
    new_game();
    g_state = ST_PLAY;
    draw_all();
}

//-----------------------------------------------------------------------------
// Application interface
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void tetris_init(void)
{
    srand(timer_us());

    g_best = config.tetris_high_score;

    if (g_best < 0 || g_best > 9999999)
        g_best = 0;

    g_state = ST_TITLE;
    new_game();

    timer_add(&g_tick_timer);
    g_tick_timer = TICK_MS;

    draw_all();
}

//-----------------------------------------------------------------------------
void tetris_redraw(void)
{
    // A menu closed over the game. Handing back a piece already halfway down
    // is a run the player did not get to lose themselves.
    if (ST_PLAY == g_state)
        g_state = ST_PAUSE;

    draw_all();
}

//-----------------------------------------------------------------------------
void tetris_task(void)
{
    if (0 != g_tick_timer)
        return;

    g_tick_timer = TICK_MS;

    if (ST_CLEARING == g_state)
    {
        g_anim_ms += TICK_MS;

        if (g_anim_ms < CLEAR_MS)
            return;

        g_anim_ms = 0;
        g_anim_step++;

        if (g_anim_step <= CLEAR_FLASHES)
        {
            bool on = (g_anim_step & 1);

            for (int y = 0; y < ROWS; y++)
            {
                if (!g_full_row[y])
                    continue;

                for (int x = 0; x < COLS; x++)
                {
                    if (on)
                        lcd_fill_rect(WELL_X + x * CELL, WELL_Y + y * CELL,
                            CELL, CELL, C_TEXT);
                    else
                        draw_grid_cell(x, y, g_well[y][x]);
                }
            }

            return;
        }

        collapse_rows();
        g_shown_valid = false;
        take_next();
        draw_well();

        if (ST_OVER == g_state)
            draw_overlay();

        g_state = (ST_OVER == g_state) ? ST_OVER : ST_PLAY;
        return;
    }

    if (ST_PLAY != g_state)
        return;

    g_lock_ms += TICK_MS;
    g_fall_owed += TICK_MS;

    if (g_fall_owed < fall_interval())
        return;

    g_fall_owed = 0;
    step_fall(false);

    if (ST_OVER == g_state)
        draw_overlay();
}

//-----------------------------------------------------------------------------
void tetris_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);

    if ((buttons & BTN_STOP) && !repeat)
    {
        if (ST_PLAY == g_state)
        {
            g_state = ST_PAUSE;
            draw_overlay();
        }
        else if (ST_PAUSE == g_state)
        {
            g_state = ST_PLAY;
            draw_all();
        }

        return;
    }

    if ((buttons & BTN_MODE) && !repeat)
    {
        if (ST_PLAY == g_state)
            hard_drop();
        else if (ST_TITLE == g_state || ST_OVER == g_state)
            start_game();

        return;
    }

    if (ST_PLAY != g_state)
        return;

    if ((buttons & BTN_F1) && !repeat)
        hold_piece();
    else if (buttons & BTN_LEFT)
        try_move(-1);
    else if (buttons & BTN_RIGHT)
        try_move(1);
    else if ((buttons & BTN_UP) && !repeat)
        try_rotate();
    else if (buttons & BTN_DOWN)
    {
        g_fall_owed = 0;
        step_fall(true);
        draw_panel_side();
    }
}

//-----------------------------------------------------------------------------
void tetris_cleanup(void)
{
    g_tick_timer = TIMER_DISABLE;
    timer_remove(&g_tick_timer);
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void action_new_game(const void *arg)
{
    (void)arg;

    new_game();
    g_state = ST_PLAY;
    menu_close_popups(); // the game repaints itself when the popup closes
}

//-----------------------------------------------------------------------------
static void action_clear_best(const void *arg)
{
    (void)arg;

    g_best = 0;
    g_new_best = false;
    config.tetris_high_score = 0;
    menu_close_popups();
}

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_NUMBER, .label = "Start level",
      .u.number = { &g_start_level, 0, 14, 1, 1, NULL, NULL } },
    { .kind = MI_TOGGLE, .label = "Ghost piece",
      .u.toggle = { &g_ghost, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "New game",
      .u.action = { action_new_game, NULL } },
    { .kind = MI_ACTION, .label = "Clear best score",
      .u.action = { action_clear_best, NULL } },
};

const menu_def_t tetris_menu =
{
    .title = "Tetris",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};

//-----------------------------------------------------------------------------
static const char *const g_help_lines[] =
{
    INFO_HEAD "CONTROLS",
    "LEFT/RIGHT - Move",
    "UP         - Rotate",
    "DOWN       - Soft drop, 1 point a cell",
    "MODE       - Hard drop, 2 points a cell",
    "F1         - Hold, once per piece",
    "STOP       - Pause",
    "MENU       - This menu",
    "SHIFT+MENU - Back to the launcher",
    "",
    INFO_HEAD "SCORING",
    "One row is 100, two 300, three 500 and four",
    "800, each multiplied by the level. Four rows",
    "at once are worth twice what four singles",
    "are, which is the whole game.",
    "",
    "The level goes up every ten rows and the",
    "pieces fall quicker with it. The best score",
    "is kept in flash and survives a power cycle.",
    "",
    INFO_HEAD "THE PIECES",
    "They come seven at a time, shuffled and",
    "dealt, so a run is never decided by an I",
    "that never came.",
    "",
    "A piece that has landed can still be slid or",
    "spun for a moment before it sets, which is",
    "what makes a tuck under an overhang",
    "possible. A turn that does not fit is nudged",
    "up to two cells sideways before it is",
    "refused.",
    "",
    "The outline under the piece is where it will",
    "land. Menu > Ghost piece turns it off.",
};

static const info_page_t g_help_page =
{
    .title = "Tetris",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const menu_item_t g_help_items[] =
{
    { .kind = MI_ACTION, .label = "Controls and rules",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t tetris_help_menu =
{
    .title = "Tetris",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
