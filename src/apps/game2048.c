/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 2048, on a board of 3x3 to 6x6
 *
 * Two things keep this small. The board holds exponents, not values - a cell is
 * 0 or the n of 2^n, which fits a byte, indexes the colour table directly and
 * turns "do these merge" into an integer compare. And there is one slide, not
 * four: a direction is a way of walking the board, so cell_at() maps (line,
 * step) to a square and every move is that same loop over a different walk.
 *
 * The tiles are drawn from the shared toolkit in game_gfx, so the whole
 * application carries no artwork of its own.
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
#include "game2048.h"

/*- Definitions -------------------------------------------------------------*/
#define MIN_SIZE            3
#define MAX_SIZE            6
#define CELLS               (MAX_SIZE * MAX_SIZE)

#define HUD_H               32
#define BOARD_PX            200                         // the panel it lives in
#define BOARD_X             ((LCD_WIDTH - BOARD_PX) / 2)
#define BOARD_Y             (HUD_H + (LCD_HEIGHT - HUD_H - BOARD_PX) / 2)

#define WIN_EXP             11                          // 2^11 = 2048
#define MAX_EXP             17                          // 131072, past the table

// A move is animated in two beats: the tiles slide to where they ended up, and
// then whatever was made there swells and settles. Both are short - the board
// is 200x200 of bit-banged panel, and a slide frame that repaints the lines
// that moved costs a few milliseconds, so the animation IS the wait.
#define TICK_MS             20
#define SLIDE_STEPS         5
#define POP_STEPS           3

#define DIR_LEFT            0
#define DIR_RIGHT           1
#define DIR_UP              2
#define DIR_DOWN            3

/*- Colors ------------------------------------------------------------------*/
#define C_PAGE              LCD_COLOR(250, 248, 239)
#define C_BOARD             LCD_COLOR(187, 173, 160)
#define C_EMPTY             LCD_COLOR(205, 193, 180)
#define C_INK               LCD_COLOR(119, 110, 101)
#define C_INK_LIGHT         LCD_COLOR(249, 246, 242)
#define C_CHIP              LCD_COLOR(143, 122, 102)

#define C_PANEL             LCD_COLOR(120, 106,  94)
#define C_PANEL_EDGE        LCD_COLOR(237, 194,  46)
#define C_PANEL_SHADOW      LCD_COLOR(160, 148, 136)

/*- Types -------------------------------------------------------------------*/
// Where one tile came from and where it ended up. Every tile on the board gets
// one of these, a tile that did not move included, because the slide repaints
// whole lines and has to put the still ones back too.
typedef struct
{
    uint8_t exp;            // what it looked like while it was moving
    uint8_t line;           // the row it slid along, or the column
    int8_t  fx, fy;
    int8_t  tx, ty;
} slide_t;

/*- Variables ---------------------------------------------------------------*/
// A cell is the exponent, so 0 is empty and 11 is the 2048 tile
static uint8_t  g_board[CELLS];
static uint8_t  g_undo_board[CELLS];
static uint8_t  g_drawn[CELLS];     // what is on the panel, to repaint the rest
static bool     g_drawn_valid;

static int      g_size = 4;
static int      g_score;
static int      g_undo_score;
static bool     g_can_undo;
static int      g_best;
static bool     g_won;              // the banner has been shown once
static bool     g_over;

static slide_t  g_slide[CELLS];
static int      g_slide_count;
static uint8_t  g_line_moved[MAX_SIZE];
static uint8_t  g_pop[CELLS];       // squares that swell when the slide lands
static int      g_pop_cell = -1;    // the tile that has just appeared
static int      g_anim_dir;
static int      g_anim_phase;       // 0 nothing, 1 sliding, 2 settling
static int      g_anim_step;
static bool     g_show_win;         // the 2048 banner, owed until the move lands

static int      g_tick_timer = TIMER_DISABLE;

/*- Forward Declarations ----------------------------------------------------*/
static void draw_board(void);
static void draw_hud(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Geometry
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// The gap between tiles shrinks with the board, so a 6x6 does not lose half its
// area to grout
static int cell_gap(void)
{
    return 11 - g_size;
}

//-----------------------------------------------------------------------------
static int cell_size(void)
{
    return (BOARD_PX - (g_size + 1) * cell_gap()) / g_size;
}

//-----------------------------------------------------------------------------
static int cell_origin(int i)
{
    int gap = cell_gap();
    int size = cell_size();
    int used = g_size * size + (g_size + 1) * gap;

    // Whatever the division left over is split either side, so the grid stays
    // centred in its panel at every board size
    return (BOARD_PX - used) / 2 + gap + i * (size + gap);
}

//-----------------------------------------------------------------------------
// The model
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Step `i` along line `l` in direction `d`, as an index into the board. Sliding
// left walks a row from its left end; sliding up walks a column from the top;
// right and down are the same walks reversed. One mapping, and the slide below
// never learns which way it is going.
static int cell_index(int d, int l, int i)
{
    int x, y;

    switch (d)
    {
        case DIR_LEFT:  x = i;              y = l;              break;
        case DIR_RIGHT: x = g_size - 1 - i; y = l;              break;
        case DIR_UP:    x = l;              y = i;              break;
        default:        x = l;              y = g_size - 1 - i; break;
    }

    return y * g_size + x;
}

//-----------------------------------------------------------------------------
static uint8_t *cell_at(int d, int l, int i)
{
    return &g_board[cell_index(d, l, i)];
}

//-----------------------------------------------------------------------------
// Where a tile started and where it is going, for the slide to draw. Recorded
// for every tile, still ones included: the slide repaints whole lines, so a
// tile it does not know about is a tile that disappears for the length of it.
static void record_slide(int d, int l, int from, int to, uint8_t exp)
{
    int fi = cell_index(d, l, from);
    int ti = cell_index(d, l, to);
    slide_t *m;

    if (g_slide_count >= CELLS)
        return;

    m = &g_slide[g_slide_count++];

    m->exp = exp;
    m->line = (uint8_t)l;
    m->fx = (int8_t)(fi % g_size);
    m->fy = (int8_t)(fi / g_size);
    m->tx = (int8_t)(ti % g_size);
    m->ty = (int8_t)(ti / g_size);
}

//-----------------------------------------------------------------------------
// Packs one line towards step 0 and merges equal neighbours, at most once each
// per move. Returns true if anything moved.
static bool slide_line(int d, int l)
{
    int write = 0;
    int merged = -1;
    bool moved = false;

    for (int read = 0; read < g_size; read++)
    {
        uint8_t *src = cell_at(d, l, read);
        uint8_t v = *src;

        if (0 == v)
            continue;

        *src = 0;

        if (write > 0 && merged != write - 1 && *cell_at(d, l, write - 1) == v)
        {
            int di = cell_index(d, l, write - 1);

            g_board[di] = (uint8_t)(v + 1);
            merged = write - 1;
            g_score += 1 << (v + 1);
            moved = true;

            // Both halves of the pair slide onto the same square, and that
            // square is what swells afterwards
            record_slide(d, l, read, write - 1, v);
            g_pop[di] = 1;
            continue;
        }

        *cell_at(d, l, write) = v;
        record_slide(d, l, read, write, v);

        if (write != read)
            moved = true;

        write++;
    }

    return moved;
}

//-----------------------------------------------------------------------------
static int free_cells(void)
{
    int n = 0;

    for (int i = 0; i < g_size * g_size; i++)
    {
        if (0 == g_board[i])
            n++;
    }

    return n;
}

//-----------------------------------------------------------------------------
// Uniformly among the empty squares, rather than by retrying random ones, so a
// nearly full board is served as reliably as an empty one
static int spawn_tile(void)
{
    int n = free_cells();
    int pick;

    if (0 == n)
        return -1;

    pick = rand() % n;

    for (int i = 0; i < g_size * g_size; i++)
    {
        if (0 != g_board[i])
            continue;

        if (0 == pick--)
        {
            // One in ten is a 4, as in the original
            g_board[i] = (rand() % 10) ? 1 : 2;
            return i;
        }
    }

    return -1;
}

//-----------------------------------------------------------------------------
static bool moves_left(void)
{
    if (free_cells() > 0)
        return true;

    for (int y = 0; y < g_size; y++)
    {
        for (int x = 0; x < g_size; x++)
        {
            uint8_t v = g_board[y * g_size + x];

            if (x + 1 < g_size && g_board[y * g_size + x + 1] == v)
                return true;

            if (y + 1 < g_size && g_board[(y + 1) * g_size + x] == v)
                return true;
        }
    }

    return false;
}

//-----------------------------------------------------------------------------
static bool board_has(int exp)
{
    for (int i = 0; i < g_size * g_size; i++)
    {
        if (g_board[i] >= exp)
            return true;
    }

    return false;
}

//-----------------------------------------------------------------------------
static void new_game(void)
{
    memset(g_board, 0, sizeof(g_board));

    g_score = 0;
    g_can_undo = false;
    g_won = false;
    g_over = false;
    g_show_win = false;
    g_pop_cell = -1;
    g_anim_phase = 0;
    g_slide_count = 0;
    g_drawn_valid = false;
    memset(g_pop, 0, sizeof(g_pop));

    spawn_tile();
    spawn_tile();
}

//-----------------------------------------------------------------------------
// Drawing
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static uint16_t tile_color(int exp)
{
    static const uint16_t table[] =
    {
        LCD_COLOR(238, 228, 218),   // 2
        LCD_COLOR(237, 224, 200),   // 4
        LCD_COLOR(242, 177, 121),   // 8
        LCD_COLOR(245, 149,  99),   // 16
        LCD_COLOR(246, 124,  95),   // 32
        LCD_COLOR(246,  94,  59),   // 64
        LCD_COLOR(237, 207, 114),   // 128
        LCD_COLOR(237, 204,  97),   // 256
        LCD_COLOR(237, 200,  80),   // 512
        LCD_COLOR(237, 197,  63),   // 1024
        LCD_COLOR(237, 194,  46),   // 2048
    };

    if (exp < 1)
        return C_EMPTY;

    if (exp - 1 >= ARRAY_SIZE(table))
        return LCD_COLOR(60, 58, 50);   // past 2048, one colour for all of them

    return table[exp - 1];
}

//-----------------------------------------------------------------------------
// 2 and 4 are pale enough to need dark ink; everything above them is not
static uint16_t tile_ink(int exp)
{
    return (exp <= 2) ? C_INK : C_INK_LIGHT;
}

//-----------------------------------------------------------------------------
// A 6x6 board has no room for "131072", so anything that will not fit is shown
// in thousands. The tile colour and the position carry the rest.
static void format_tile(int exp, char *buf, int size)
{
    uint32_t value = 1u << exp;

    if (value >= 10000)
        snprintf(buf, size, "%uk", (unsigned)(value / 1024));
    else
        snprintf(buf, size, "%u", (unsigned)value);
}

//-----------------------------------------------------------------------------
static void draw_tile_at(int px, int py, int size, int exp)
{
    uint16_t color = tile_color(exp);
    char buf[10];
    int w;

    gfx_round_rect(px, py, size, size, size / 6, color);

    if (exp < 1)
        return;

    format_tile(exp, buf, sizeof(buf));
    w = (int)strlen(buf);

    // The biggest that fits, in this order: the scaled font, then the two the
    // firmware carries. A tile is small and a number in it has to be read at a
    // glance, so it is worth the three cases.
    if (w * 16 <= size - 8 && 32 <= size - 8)
    {
        gfx_text_scaled(px + (size - gfx_text_scaled_width(buf, 2)) / 2,
            py + (size - 32) / 2, buf, 2, tile_ink(exp), color);
    }
    else if (w * (FONT_LARGE)->width <= size - 6)
    {
        gfx_text(px + (size - w * (FONT_LARGE)->width) / 2,
            py + (size - (FONT_LARGE)->height) / 2, buf, FONT_LARGE,
            tile_ink(exp), color);
    }
    else
    {
        gfx_text(px + (size - w * (FONT_SMALL)->width) / 2,
            py + (size - (FONT_SMALL)->height) / 2, buf, FONT_SMALL,
            tile_ink(exp), color);
    }
}

//-----------------------------------------------------------------------------
static void draw_cell(int x, int y, int exp)
{
    draw_tile_at(BOARD_X + cell_origin(x), BOARD_Y + cell_origin(y),
        cell_size(), exp);
}

//-----------------------------------------------------------------------------
// The band one line of the board occupies, put back empty. Exactly the cells'
// own rows or columns and no more: a tile slides along its line and never
// leaves it, so this never has to touch the rounded corners of the panel.
static void fill_line_band(int d, int l)
{
    bool horizontal = (DIR_LEFT == d || DIR_RIGHT == d);
    int size = cell_size();
    int a = cell_origin(0);
    int b = cell_origin(g_size - 1) + size - a;
    int c = cell_origin(l);

    if (horizontal)
        lcd_fill_rect(BOARD_X + a, BOARD_Y + c, b, size, C_BOARD);
    else
        lcd_fill_rect(BOARD_X + c, BOARD_Y + a, size, b, C_BOARD);

    for (int i = 0; i < g_size; i++)
        draw_cell(horizontal ? i : l, horizontal ? l : i, 0);
}

//-----------------------------------------------------------------------------
// One frame of the slide. Eased out, so the tiles leave quickly and arrive
// gently, which is what makes a grid of squares look like it has weight.
static void draw_slide_frame(int step)
{
    int den = SLIDE_STEPS;
    int num = step + 1;
    int eased = den * den - (den - num) * (den - num);
    int size = cell_size();

    for (int l = 0; l < g_size; l++)
    {
        if (g_line_moved[l])
            fill_line_band(g_anim_dir, l);
    }

    for (int i = 0; i < g_slide_count; i++)
    {
        const slide_t *m = &g_slide[i];
        int fx, fy, tx, ty;

        if (!g_line_moved[m->line])
            continue;

        fx = cell_origin(m->fx);
        fy = cell_origin(m->fy);
        tx = cell_origin(m->tx);
        ty = cell_origin(m->ty);

        draw_tile_at(BOARD_X + fx + (tx - fx) * eased / (den * den),
            BOARD_Y + fy + (ty - fy) * eased / (den * den), size, m->exp);
    }
}

//-----------------------------------------------------------------------------
// The beat after the slide: what was made swells past its square and settles,
// and the tile that has just appeared grows into its own. The overshoot is
// capped at half the grout, so it can never reach a neighbour.
static void draw_pop_frame(int step)
{
    static const int shape[POP_STEPS] = { 2, 1, 0 };
    int size = cell_size();
    int gap = cell_gap();
    int bump = gap / 2 - 1;
    int grow;

    if (bump < 1)
        bump = 1;

    grow = bump * shape[step] / shape[0];

    for (int i = 0; i < g_size * g_size; i++)
    {
        int x = i % g_size;
        int y = i / g_size;
        int px = BOARD_X + cell_origin(x);
        int py = BOARD_Y + cell_origin(y);
        int d;

        if (!g_pop[i] && i != g_pop_cell)
            continue;

        // A square that has just appeared comes up out of nothing; one that was
        // merged into overshoots and comes back
        d = (i == g_pop_cell) ? -(size / 3) * shape[step] / shape[0] : grow;

        lcd_fill_rect(px - bump, py - bump, size + 2 * bump, size + 2 * bump,
            C_BOARD);
        draw_tile_at(px, py, size, 0);
        draw_tile_at(px - d, py - d, size + 2 * d, g_board[i]);
    }
}

//-----------------------------------------------------------------------------
// Only the squares that changed. A move touches a handful of them, and the
// whole board is 200x200 of bit-banged panel - 40 ms if it is repainted for
// every arrow press, which is a game that feels slow for no reason.
static void draw_board(void)
{
    if (!g_drawn_valid)
    {
        gfx_round_rect(BOARD_X, BOARD_Y, BOARD_PX, BOARD_PX, 8, C_BOARD);

        for (int i = 0; i < g_size * g_size; i++)
            g_drawn[i] = 0xff;
    }

    for (int y = 0; y < g_size; y++)
    {
        for (int x = 0; x < g_size; x++)
        {
            int i = y * g_size + x;

            if (g_drawn_valid && g_drawn[i] == g_board[i])
                continue;

            draw_cell(x, y, g_board[i]);
            g_drawn[i] = g_board[i];
        }
    }

    g_drawn_valid = true;
}

//-----------------------------------------------------------------------------
static void draw_hud(void)
{
    char buf[32];

    lcd_fill_rect(0, 0, LCD_WIDTH, HUD_H, C_PAGE);

    gfx_text(6, 8, "2048", FONT_LARGE, C_INK, C_PAGE);

    snprintf(buf, sizeof(buf), "%dx%d", g_size, g_size);
    gfx_text(46, 12, buf, FONT_SMALL, C_CHIP, C_PAGE);

    snprintf(buf, sizeof(buf), "SCORE %d", g_score);
    gfx_text(LCD_WIDTH - 4 - (int)strlen(buf) * (FONT_SMALL)->width, 4, buf,
        FONT_SMALL, C_INK, C_PAGE);

    snprintf(buf, sizeof(buf), "BEST %d", g_best);
    gfx_text(LCD_WIDTH - 4 - (int)strlen(buf) * (FONT_SMALL)->width, 18, buf,
        FONT_SMALL, C_CHIP, C_PAGE);

    if (g_can_undo)
        gfx_text(96, 12, "MODE undo", FONT_SMALL, C_CHIP, C_PAGE);
}

//-----------------------------------------------------------------------------
static void draw_panel(const char *title, const char *l1, const char *l2)
{
    int w = 240;
    int h = 108;
    int x = (LCD_WIDTH - w) / 2;
    int y = BOARD_Y + (BOARD_PX - h) / 2;

    gfx_panel(x, y, w, h, C_PANEL, C_PANEL_EDGE, C_PANEL_SHADOW);

    gfx_text_scaled(x + (w - gfx_text_scaled_width(title, 2)) / 2, y + 16,
        title, 2, C_INK_LIGHT, C_PANEL);

    gfx_text_centered(x + w / 2, y + 58, l1, FONT_SMALL, C_INK_LIGHT, C_PANEL);

    if (l2)
        gfx_text_centered(x + w / 2, y + 76, l2, FONT_SMALL, C_PANEL_EDGE,
            C_PANEL);

    // The panel covered part of the board, so what it covered has to be
    // repainted when it goes
    g_drawn_valid = false;
}

//-----------------------------------------------------------------------------
static void draw_overlay(void)
{
    char buf[32];

    if (g_over)
    {
        snprintf(buf, sizeof(buf), "SCORE %d", g_score);
        draw_panel("NO MOVES", buf, "STOP  new game");
    }
}

//-----------------------------------------------------------------------------
static void draw_all(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, C_PAGE);

    g_drawn_valid = false;

    draw_hud();
    draw_board();
    draw_overlay();
}

//-----------------------------------------------------------------------------
// Play
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void record_best(void)
{
    if (g_score > g_best)
    {
        g_best = g_score;

        if (g_best > config.g2048_high_score)
            config.g2048_high_score = g_best;
    }
}

//-----------------------------------------------------------------------------
// The board is already final when the animation starts - the slide is drawn
// from the record of where each tile came from, not from the board - so this
// only has to put the shadow back in step and hand over any owed panel.
static void anim_finish(void)
{
    if (0 == g_anim_phase)
        return;

    g_anim_phase = 0;
    g_pop_cell = -1;
    memset(g_pop, 0, sizeof(g_pop));

    // The last frames of the animation left the board looking exactly like the
    // model, so the diff shadow can simply be told so
    memcpy(g_drawn, g_board, sizeof(g_drawn));
    g_drawn_valid = true;

    if (g_show_win)
    {
        g_show_win = false;
        draw_panel("2048", "you got there", "any arrow  keep going");
        return;
    }

    draw_overlay();
}

//-----------------------------------------------------------------------------
static void do_move(int dir)
{
    uint8_t saved[CELLS];
    int saved_score = g_score;
    bool moved = false;

    if (g_over)
        return;

    // An arrow during an animation lands on a settled board rather than
    // interrupting one: holding a direction stays responsive
    anim_finish();

    memcpy(saved, g_board, sizeof(saved));

    g_slide_count = 0;
    g_anim_dir = dir;
    memset(g_pop, 0, sizeof(g_pop));
    memset(g_line_moved, 0, sizeof(g_line_moved));

    for (int l = 0; l < g_size; l++)
    {
        if (slide_line(dir, l))
        {
            g_line_moved[l] = 1;
            moved = true;
        }
    }

    if (!moved)
    {
        // Nothing shifted and nothing merged, so it was not a move: no tile
        // appears, and the undo still points at the position before the last
        // real one
        g_score = saved_score;
        g_slide_count = 0;
        return;
    }

    memcpy(g_undo_board, saved, sizeof(g_undo_board));
    g_undo_score = saved_score;
    g_can_undo = true;

    g_pop_cell = spawn_tile();

    record_best();

    if (!g_won && board_has(WIN_EXP))
    {
        g_won = true;
        g_show_win = true;
    }

    if (!moves_left())
        g_over = true;

    draw_hud();

    g_anim_phase = 1;
    g_anim_step = 0;
    draw_slide_frame(0);
}

//-----------------------------------------------------------------------------
static void undo_move(void)
{
    if (!g_can_undo)
        return;

    anim_finish();

    memcpy(g_board, g_undo_board, sizeof(g_board));
    g_score = g_undo_score;
    g_can_undo = false;
    g_over = false;
    g_pop_cell = -1;

    draw_all();
}

//-----------------------------------------------------------------------------
// Application interface
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void game2048_init(void)
{
    srand(timer_us());

    g_best = config.g2048_high_score;

    if (g_best < 0 || g_best > 9999999)
        g_best = 0;

    new_game();

    timer_add(&g_tick_timer);
    g_tick_timer = TICK_MS;

    draw_all();
}

//-----------------------------------------------------------------------------
void game2048_redraw(void)
{
    draw_all();
}

//-----------------------------------------------------------------------------
void game2048_task(void)
{
    if (0 != g_tick_timer)
        return;

    g_tick_timer = TICK_MS;

    if (1 == g_anim_phase)
    {
        if (++g_anim_step < SLIDE_STEPS)
        {
            draw_slide_frame(g_anim_step);
            return;
        }

        g_anim_phase = 2;
        g_anim_step = 0;
        draw_pop_frame(0);
        return;
    }

    if (2 == g_anim_phase)
    {
        if (++g_anim_step < POP_STEPS)
        {
            draw_pop_frame(g_anim_step);
            return;
        }

        anim_finish();
    }
}

//-----------------------------------------------------------------------------
void game2048_buttons_handler(int buttons)
{
    if (buttons & BTN_REPEAT)
        return;

    if (buttons & BTN_STOP)
    {
        new_game();
        draw_all();
        return;
    }

    if (buttons & BTN_MODE)
    {
        undo_move();
        return;
    }

    if (buttons & BTN_LEFT)       do_move(DIR_LEFT);
    else if (buttons & BTN_RIGHT) do_move(DIR_RIGHT);
    else if (buttons & BTN_UP)    do_move(DIR_UP);
    else if (buttons & BTN_DOWN)  do_move(DIR_DOWN);
}

//-----------------------------------------------------------------------------
void game2048_cleanup(void)
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
    menu_close_popups(); // the game repaints itself when the popup closes
}

//-----------------------------------------------------------------------------
static void action_clear_best(const void *arg)
{
    (void)arg;

    g_best = 0;
    config.g2048_high_score = 0;
    menu_close_popups();
}

//-----------------------------------------------------------------------------
// The board size is an index into the labels, so the setting and the number
// stay in one place
static int g_size_choice = 1;

static void size_changed(void)
{
    g_size = MIN_SIZE + g_size_choice;
    new_game();
}

static const char *const g_size_labels[] = { "3x3", "4x4", "5x5", "6x6" };

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "Board",
      .u.choice = { &g_size_choice, g_size_labels, ARRAY_SIZE(g_size_labels),
                    size_changed } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "New game",
      .u.action = { action_new_game, NULL } },
    { .kind = MI_ACTION, .label = "Clear best score",
      .u.action = { action_clear_best, NULL } },
};

const menu_def_t game2048_menu =
{
    .title = "2048",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};

//-----------------------------------------------------------------------------
static const char *const g_help_lines[] =
{
    INFO_HEAD "CONTROLS",
    "Arrows     - Slide the whole board",
    "MODE       - Undo the last move",
    "STOP       - New game",
    "MENU       - This menu",
    "SHIFT+MENU - Back to the launcher",
    "",
    INFO_HEAD "RULES",
    "Every arrow slides all the tiles as far as",
    "they go. Two equal tiles that meet become",
    "one of twice the value, and that value is",
    "what it scores. A tile can only merge once",
    "in a move, so a row of four 2s becomes two",
    "4s, never one 8.",
    "",
    "After a move that changed something, one new",
    "tile appears - a 2, or a 4 one time in ten.",
    "The run ends when the board is full and no",
    "two neighbours match.",
    "",
    INFO_HEAD "BOARD",
    "Menu > Board sets it from 3x3 to 6x6, and",
    "starts a new game. A small board fills up",
    "before it can be planned; a large one asks",
    "for a lot more merges to reach the same",
    "tile. The best score is kept in flash and is",
    "shared by every board size.",
};

static const info_page_t g_help_page =
{
    .title = "2048",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const menu_item_t g_help_items[] =
{
    { .kind = MI_ACTION, .label = "Controls and rules",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t game2048_help_menu =
{
    .title = "2048",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
