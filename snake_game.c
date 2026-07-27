/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Simple Snake Game
 *
 * The playfield is drawn incrementally: on every step only the new head, the
 * old head and the vacated tail cell are touched. A full repaint of a
 * 320x240 bit-banged display costs more than the whole game tick, so it only
 * happens on a restart or after a menu closed over the game.
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
#include "ui.h"
#include "menu_widget.h"
#include "snake_game.h"

/*- Definitions -------------------------------------------------------------*/
#define GRID_SIZE           10
#define STATUS_H            20
#define FIELD_Y             STATUS_H
#define GRID_WIDTH          (LCD_WIDTH / GRID_SIZE)
#define GRID_HEIGHT         ((LCD_HEIGHT - FIELD_Y) / GRID_SIZE)
#define MAX_SNAKE_LENGTH    (GRID_WIDTH * GRID_HEIGHT)

// Step interval in ms, indexed by g_speed
#define SPEED_SLOW          0
#define SPEED_NORMAL        1
#define SPEED_FAST          2

// Colors
#define BG_COLOR            LCD_COLOR(0, 0, 0)        // Black
#define STATUS_BG           LCD_COLOR(20, 20, 40)
#define SNAKE_COLOR         LCD_COLOR(0, 200, 0)      // Green
#define SNAKE_HEAD_COLOR    LCD_COLOR(120, 255, 120)  // Light green
#define FOOD_COLOR          LCD_COLOR(255, 0, 0)      // Red
#define BORDER_COLOR        LCD_COLOR(80, 80, 80)
#define TEXT_COLOR          LCD_COLOR(255, 255, 255)  // White

/*- Types -------------------------------------------------------------------*/
typedef enum {
    DIR_UP = 0,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT
} direction_t;

typedef struct {
    int x, y;
} point_t;

typedef struct {
    point_t segments[MAX_SNAKE_LENGTH];
    int length;
    direction_t direction;
    direction_t next_direction;
} snake_t;

typedef struct {
    snake_t snake;
    point_t food;
    int score;
    bool game_over;
    bool game_started;
    bool paused;
} game_state_t;

/*- Variables ---------------------------------------------------------------*/
static game_state_t g_game;
static int g_game_timer = TIMER_DISABLE;
static int g_speed = SPEED_NORMAL;
static bool g_wrap_walls = false;
static int g_high_score = 0;

static const int g_step_ms[] = { 250, 150, 90 };

/*- Forward Declarations ----------------------------------------------------*/
static void reset_game(void);
static void spawn_food(void);
static void draw_status(void);
static void draw_field(void);
static void draw_overlay(void);
static bool is_free_cell(int x, int y);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void draw_cell(int x, int y, uint16_t color)
{
    lcd_fill_rect(x * GRID_SIZE + 1, FIELD_Y + y * GRID_SIZE + 1,
        GRID_SIZE - 2, GRID_SIZE - 2, color);
}

//-----------------------------------------------------------------------------
static void clear_cell(int x, int y)
{
    lcd_fill_rect(x * GRID_SIZE, FIELD_Y + y * GRID_SIZE,
        GRID_SIZE, GRID_SIZE, BG_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_head(int x, int y)
{
    draw_cell(x, y, SNAKE_HEAD_COLOR);

    lcd_draw_pixel(x * GRID_SIZE + 3, FIELD_Y + y * GRID_SIZE + 3, BG_COLOR);
    lcd_draw_pixel(x * GRID_SIZE + GRID_SIZE - 4, FIELD_Y + y * GRID_SIZE + 3, BG_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_food(void)
{
    draw_cell(g_game.food.x, g_game.food.y, FOOD_COLOR);
    lcd_draw_pixel(g_game.food.x * GRID_SIZE + GRID_SIZE / 2,
        FIELD_Y + g_game.food.y * GRID_SIZE + GRID_SIZE / 2, LCD_COLOR(255, 255, 0));
}

//-----------------------------------------------------------------------------
static void reset_game(void)
{
    g_game.snake.length = 3;
    g_game.snake.direction = DIR_RIGHT;
    g_game.snake.next_direction = DIR_RIGHT;

    int start_x = GRID_WIDTH / 2;
    int start_y = GRID_HEIGHT / 2;

    for (int i = 0; i < g_game.snake.length; i++) {
        g_game.snake.segments[i].x = start_x - i;
        g_game.snake.segments[i].y = start_y;
    }

    g_game.score = 0;
    g_game.game_over = false;
    g_game.game_started = false;
    g_game.paused = false;

    spawn_food();
}

//-----------------------------------------------------------------------------
static bool is_free_cell(int x, int y)
{
    if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT)
        return false;

    for (int i = 0; i < g_game.snake.length; i++) {
        if (g_game.snake.segments[i].x == x && g_game.snake.segments[i].y == y)
            return false;
    }

    return true;
}

//-----------------------------------------------------------------------------
static void spawn_food(void)
{
    int attempts = 0;

    do {
        g_game.food.x = rand() % GRID_WIDTH;
        g_game.food.y = rand() % GRID_HEIGHT;
        attempts++;
    } while (!is_free_cell(g_game.food.x, g_game.food.y) && attempts < 200);
}

//-----------------------------------------------------------------------------
// One step: move, test for a collision, and repaint only the three cells that
// can have changed
static void step_game(void)
{
    point_t tail = g_game.snake.segments[g_game.snake.length - 1];
    point_t head = g_game.snake.segments[0];

    g_game.snake.direction = g_game.snake.next_direction;

    switch (g_game.snake.direction) {
        case DIR_UP:    head.y--; break;
        case DIR_DOWN:  head.y++; break;
        case DIR_LEFT:  head.x--; break;
        case DIR_RIGHT: head.x++; break;
    }

    if (g_wrap_walls) {
        if (head.x < 0) head.x = GRID_WIDTH - 1;
        if (head.x >= GRID_WIDTH) head.x = 0;
        if (head.y < 0) head.y = GRID_HEIGHT - 1;
        if (head.y >= GRID_HEIGHT) head.y = 0;
    }
    else if (head.x < 0 || head.x >= GRID_WIDTH ||
             head.y < 0 || head.y >= GRID_HEIGHT) {
        g_game.game_over = true;
        draw_overlay();
        return;
    }

    // The tail cell is vacated by this very move, so running into it is legal
    for (int i = 0; i < g_game.snake.length - 1; i++) {
        if (g_game.snake.segments[i].x == head.x &&
            g_game.snake.segments[i].y == head.y) {
            g_game.game_over = true;
            draw_overlay();
            return;
        }
    }

    point_t prev_head = g_game.snake.segments[0];
    bool ate = (head.x == g_game.food.x && head.y == g_game.food.y);

    for (int i = g_game.snake.length - 1; i > 0; i--)
        g_game.snake.segments[i] = g_game.snake.segments[i - 1];

    g_game.snake.segments[0] = head;

    if (ate) {
        // Grow by putting the tail cell back on the end
        if (g_game.snake.length < MAX_SNAKE_LENGTH) {
            g_game.snake.segments[g_game.snake.length] = tail;
            g_game.snake.length++;
        }

        g_game.score += 10;

        if (g_game.score > g_high_score)
            g_high_score = g_game.score;

        spawn_food();
        draw_food();
        draw_status();
    }
    else {
        clear_cell(tail.x, tail.y);
    }

    if (g_game.snake.length > 1)
        draw_cell(prev_head.x, prev_head.y, SNAKE_COLOR);

    draw_head(head.x, head.y);
}

//-----------------------------------------------------------------------------
static void draw_status(void)
{
    char buf[48];

    lcd_fill_rect(0, 0, LCD_WIDTH, STATUS_H, STATUS_BG);
    lcd_fill_rect(0, STATUS_H - 1, LCD_WIDTH, 1, BORDER_COLOR);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(STATUS_BG, TEXT_COLOR);

    // The left string reaches 32 characters (192 px) at four-digit scores, so
    // the mode label starts well clear of it
    snprintf(buf, sizeof(buf), "Score %d   Best %d   Len %d",
        g_game.score, g_high_score, g_game.snake.length);
    lcd_puts(5, 6, buf);

    lcd_puts(LCD_WIDTH - 40, 6, g_wrap_walls ? "WRAP" : "WALLS");
}

//-----------------------------------------------------------------------------
static void draw_field(void)
{
    lcd_fill_rect(0, FIELD_Y, LCD_WIDTH, LCD_HEIGHT - FIELD_Y, BG_COLOR);

    if (!g_game.game_started)
        return;

    for (int i = g_game.snake.length - 1; i > 0; i--) {
        draw_cell(g_game.snake.segments[i].x, g_game.snake.segments[i].y,
            SNAKE_COLOR);
    }

    draw_head(g_game.snake.segments[0].x, g_game.snake.segments[0].y);
    draw_food();
}

//-----------------------------------------------------------------------------
// Centered message box, used for the title, pause and game over states
static void draw_box(const char *title, const char *l1, const char *l2)
{
    int w = 240;
    int h = 80;
    int x = (LCD_WIDTH - w) / 2;
    int y = FIELD_Y + (LCD_HEIGHT - FIELD_Y - h) / 2;

    lcd_fill_rect(x, y, w, h, STATUS_BG);
    lcd_draw_rect(x, y, w - 1, h - 1, BORDER_COLOR);

    lcd_set_font(FONT_LARGE);
    lcd_set_color(STATUS_BG, TEXT_COLOR);
    lcd_puts(x + (w - (int)strlen(title) * 8) / 2, y + 12, title);

    lcd_set_font(FONT_SMALL);

    if (l1)
        lcd_puts(x + (w - (int)strlen(l1) * 6) / 2, y + 40, l1);

    if (l2)
        lcd_puts(x + (w - (int)strlen(l2) * 6) / 2, y + 55, l2);
}

//-----------------------------------------------------------------------------
static void draw_overlay(void)
{
    if (!g_game.game_started)
        draw_box("SNAKE", "MODE: start   arrows: steer",
            "MENU: settings and help");
    else if (g_game.game_over)
        draw_box("GAME OVER", "MODE: play again", "SHIFT+MENU: back to the launcher");
    else if (g_game.paused)
        draw_box("PAUSED", "MODE: resume", NULL);
}

//-----------------------------------------------------------------------------
static void start_or_toggle(void)
{
    if (!g_game.game_started) {
        g_game.game_started = true;
        draw_field();
        return;
    }

    if (g_game.game_over) {
        reset_game();
        g_game.game_started = true;
        draw_status();
        draw_field();
        return;
    }

    g_game.paused = !g_game.paused;

    if (g_game.paused)
        draw_overlay();
    else
        draw_field();
}

//-----------------------------------------------------------------------------
void snake_game_init(void)
{
    srand(timer_ms()); // otherwise every game gets the same food sequence

    reset_game();

    timer_add(&g_game_timer);
    g_game_timer = g_step_ms[g_speed];

    snake_game_redraw();
}

//-----------------------------------------------------------------------------
void snake_game_redraw(void)
{
    draw_status();
    draw_field();
    draw_overlay();
}

//-----------------------------------------------------------------------------
void snake_game_task(void)
{
    if (g_game_timer != 0)
        return;

    g_game_timer = g_step_ms[g_speed];

    if (g_game.game_started && !g_game.game_over && !g_game.paused)
        step_game();
}

//-----------------------------------------------------------------------------
void snake_game_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    direction_t dir = g_game.snake.direction;

    // A 180 degree turn would run the head straight into the neck
    if (buttons & BTN_UP) {
        if (dir != DIR_DOWN)
            g_game.snake.next_direction = DIR_UP;
    }
    else if (buttons & BTN_DOWN) {
        if (dir != DIR_UP)
            g_game.snake.next_direction = DIR_DOWN;
    }
    else if (buttons & BTN_LEFT) {
        if (dir != DIR_RIGHT)
            g_game.snake.next_direction = DIR_LEFT;
    }
    else if (buttons & BTN_RIGHT) {
        if (dir != DIR_LEFT)
            g_game.snake.next_direction = DIR_RIGHT;
    }
    else if (buttons & (BTN_MODE | BTN_STOP)) {
        if (!repeat)
            start_or_toggle();
    }
}

//-----------------------------------------------------------------------------
void snake_game_cleanup(void)
{
    g_game_timer = TIMER_DISABLE;
    timer_remove(&g_game_timer);
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void action_restart(const void *arg)
{
    (void)arg;

    reset_game();
    g_game.game_started = true;
    menu_close_popups(); // the game repaints itself when the popup closes
}

static const char *const g_help_lines[] =
{
    "UP/DOWN/LEFT/RIGHT - Steer",
    "MODE (or STOP)     - Start / pause / retry",
    "MENU               - This menu",
    "SHIFT+MENU         - Back to the launcher",
    "",
    "Eating the food scores 10 points and grows",
    "the snake by one cell.",
    "",
    "With Walls on, the border is lethal; with",
    "Wrap on, the snake comes back on the other",
    "side. Either way, running into yourself",
    "ends the game.",
};

static const info_page_t g_help_page =
{
    .title = "Snake",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const char *const g_speed_labels[] = { "Slow", "Normal", "Fast" };

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "Speed",
      .u.choice = { &g_speed, g_speed_labels, ARRAY_SIZE(g_speed_labels), NULL } },
    { .kind = MI_TOGGLE, .label = "Wrap at walls",
      .u.toggle = { &g_wrap_walls, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Restart",
      .u.action = { action_restart, NULL } },
};

const menu_def_t snake_game_menu =
{
    .title = "Snake",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};

// Read-only pages: the system menu shows them under Help, not among the
// settings above
static const menu_item_t g_help_items[] =
{
    { .kind = MI_ACTION, .label = "Controls",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t snake_game_help_menu =
{
    .title = "Snake",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
