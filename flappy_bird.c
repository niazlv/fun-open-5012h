/*
 * Simple Flappy Bird Game
 *
 * Only what moved is repainted: the strip each pipe vacated, the pipe bodies
 * themselves and the bird's old and new rectangles. Clearing the whole 320x240
 * panel every frame, as the first version did, is both slow on a bit-banged
 * display and visibly flickery.
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
#include "flappy_bird.h"

/*- Definitions -------------------------------------------------------------*/
#define BIRD_SIZE           10
#define BIRD_X              50
#define PIPE_WIDTH          24
#define PIPE_LIP_H          6           // darker band framing the gap
#define PIPE_SPACING        150         // horizontal distance between pipes
#define MAX_PIPES           4
#define GRAVITY             0.55f
#define JUMP_STRENGTH       -6.5f
#define FRAME_MS            50          // 20 FPS

#define GROUND_HEIGHT       20
#define SKY_HEIGHT          (LCD_HEIGHT - GROUND_HEIGHT)
#define GAP_MARGIN          25          // keep the gap away from sky/ground

#define SCORE_X             8
#define SCORE_Y             8
#define SCORE_W             130
#define SCORE_H             16

// Difficulty, index into g_gap_size / g_pipe_speed
#define DIFF_EASY           0
#define DIFF_NORMAL         1
#define DIFF_HARD           2

// Colors
#define SKY_COLOR           LCD_COLOR(135, 206, 235)  // Sky blue
#define GROUND_COLOR        LCD_COLOR(222, 184, 135)  // Tan
#define GROUND_LINE_COLOR   LCD_COLOR(139, 69, 19)
#define BIRD_COLOR          LCD_COLOR(255, 215, 0)    // Yellow
#define BIRD_EDGE_COLOR     LCD_COLOR(255, 140, 0)
#define PIPE_COLOR          LCD_COLOR(0, 150, 0)      // Green
#define PIPE_LIP_COLOR      LCD_COLOR(0, 100, 0)
#define TEXT_COLOR          LCD_COLOR(255, 255, 255)  // White
#define PANEL_COLOR         LCD_COLOR(20, 20, 40)

/*- Types -------------------------------------------------------------------*/
typedef struct {
    float y;
    float velocity_y;
    int drawn_y;      // where the bird was last painted, -1 if nowhere
} bird_t;

typedef struct {
    int x;
    int drawn_x;      // where the pipe was last painted, INT_MIN if nowhere
    int gap_y;
    bool passed;
} pipe_t;

typedef struct {
    bird_t bird;
    pipe_t pipes[MAX_PIPES];
    int score;
    int drawn_score;
    bool game_over;
    bool game_started;
} game_state_t;

/*- Variables ---------------------------------------------------------------*/
static game_state_t g_game;
static int g_game_timer = TIMER_DISABLE;
static int g_difficulty = DIFF_NORMAL;
static int g_high_score = 0;

static const int g_gap_size[] = { 85, 65, 50 };
static const int g_pipe_speed[] = { 2, 3, 4 };

#define OFFSCREEN           (-1000)

/*- Forward Declarations ----------------------------------------------------*/
static void reset_game(void);
static void draw_overlay(void);
static void draw_score(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static int pipe_gap(void)
{
    return g_gap_size[g_difficulty];
}

//-----------------------------------------------------------------------------
static void place_pipe(int index, int x)
{
    int span = SKY_HEIGHT - pipe_gap() - 2 * GAP_MARGIN;

    g_game.pipes[index].x = x;
    g_game.pipes[index].gap_y = GAP_MARGIN + (span > 0 ? rand() % span : 0);
    g_game.pipes[index].passed = false;
}

//-----------------------------------------------------------------------------
static void reset_game(void)
{
    g_game.bird.y = LCD_HEIGHT / 2;
    g_game.bird.velocity_y = 0;
    g_game.bird.drawn_y = OFFSCREEN;

    g_game.score = 0;
    g_game.drawn_score = -1;
    g_game.game_over = false;
    g_game.game_started = false;

    for (int i = 0; i < MAX_PIPES; i++) {
        place_pipe(i, LCD_WIDTH + 60 + i * PIPE_SPACING);
        g_game.pipes[i].drawn_x = OFFSCREEN;
    }
}

//-----------------------------------------------------------------------------
static void draw_background(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, SKY_HEIGHT, SKY_COLOR);
    lcd_fill_rect(0, SKY_HEIGHT, LCD_WIDTH, GROUND_HEIGHT, GROUND_COLOR);
    lcd_fill_rect(0, SKY_HEIGHT, LCD_WIDTH, 2, GROUND_LINE_COLOR);
}

//-----------------------------------------------------------------------------
// Column band [x, x+w) of a pipe: body above the gap, both lips, body below
static void draw_pipe_band(const pipe_t *p, int x, int w)
{
    int bottom_y = p->gap_y + pipe_gap();

    if (w <= 0)
        return;

    lcd_fill_rect(x, 0, w, p->gap_y - PIPE_LIP_H, PIPE_COLOR);
    lcd_fill_rect(x, p->gap_y - PIPE_LIP_H, w, PIPE_LIP_H, PIPE_LIP_COLOR);
    lcd_fill_rect(x, bottom_y, w, PIPE_LIP_H, PIPE_LIP_COLOR);
    lcd_fill_rect(x, bottom_y + PIPE_LIP_H, w,
        SKY_HEIGHT - bottom_y - PIPE_LIP_H, PIPE_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_pipe(const pipe_t *p)
{
    if (p->x >= LCD_WIDTH || p->x + PIPE_WIDTH <= 0)
        return;

    draw_pipe_band(p, p->x, PIPE_WIDTH);
}

//-----------------------------------------------------------------------------
// Move a pipe on screen by repainting only the two column strips that
// changed: the one it vacated on the right and the one it now covers on the
// left. A discontinuity (first frame, respawn) falls back to erase + redraw.
static void update_pipe_pixels(pipe_t *p, int speed)
{
    if (p->drawn_x <= OFFSCREEN || p->drawn_x - p->x != speed) {
        if (p->drawn_x > OFFSCREEN)
            lcd_fill_rect(p->drawn_x, 0, PIPE_WIDTH, SKY_HEIGHT, SKY_COLOR);

        draw_pipe(p);
        p->drawn_x = p->x;
        return;
    }

    lcd_fill_rect(p->x + PIPE_WIDTH, 0, speed, SKY_HEIGHT, SKY_COLOR);

    if (p->x + PIPE_WIDTH > 0 && p->x < LCD_WIDTH)
        draw_pipe_band(p, p->x, speed);

    p->drawn_x = p->x;
}

//-----------------------------------------------------------------------------
static bool pipe_at_bird(const pipe_t *p)
{
    return (p->x < BIRD_X + BIRD_SIZE) && (p->x + PIPE_WIDTH > BIRD_X);
}

//-----------------------------------------------------------------------------
static void draw_bird(void)
{
    int x = BIRD_X;
    int y = (int)g_game.bird.y;

    lcd_fill_rect(x, y, BIRD_SIZE, BIRD_SIZE, BIRD_COLOR);
    lcd_draw_rect(x, y, BIRD_SIZE, BIRD_SIZE, BIRD_EDGE_COLOR);
    lcd_fill_rect(x + BIRD_SIZE - 4, y + 2, 2, 2, LCD_BLACK_COLOR);
}

//-----------------------------------------------------------------------------
// Erase only the band the bird moved out of. Whatever pipe shares the bird's
// column is repainted by the caller afterwards.
static void erase_bird(void)
{
    int old_y = g_game.bird.drawn_y;
    int new_y = (int)g_game.bird.y;
    int dy = new_y - old_y;

    if (old_y <= OFFSCREEN || 0 == dy)
        return;

    if (dy >= BIRD_SIZE || -dy >= BIRD_SIZE)
        lcd_fill_rect(BIRD_X, old_y, BIRD_SIZE, BIRD_SIZE, SKY_COLOR);
    else if (dy > 0)
        lcd_fill_rect(BIRD_X, old_y, BIRD_SIZE, dy, SKY_COLOR);
    else
        lcd_fill_rect(BIRD_X, new_y + BIRD_SIZE, BIRD_SIZE, -dy, SKY_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_score(void)
{
    char buf[24];

    lcd_fill_rect(SCORE_X - 3, SCORE_Y - 3, SCORE_W, SCORE_H, PANEL_COLOR);
    lcd_set_font(FONT_SMALL);
    lcd_set_color(PANEL_COLOR, TEXT_COLOR);

    snprintf(buf, sizeof(buf), "Score %d  Best %d", g_game.score, g_high_score);
    lcd_puts(SCORE_X, SCORE_Y, buf);

    g_game.drawn_score = g_game.score;
}

//-----------------------------------------------------------------------------
// Centered message box, used for the title and game over states
static void draw_box(const char *title, const char *l1, const char *l2)
{
    int w = 250;
    int h = 80;
    int x = (LCD_WIDTH - w) / 2;
    int y = (SKY_HEIGHT - h) / 2;

    lcd_fill_rect(x, y, w, h, PANEL_COLOR);
    lcd_draw_rect(x, y, w, h, TEXT_COLOR);

    lcd_set_font(FONT_LARGE);
    lcd_set_color(PANEL_COLOR, TEXT_COLOR);
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
        draw_box("FLAPPY BIRD", "MODE or UP: flap and start",
            "MENU: settings and help");
    else if (g_game.game_over)
        draw_box("GAME OVER", "MODE or UP: play again",
            "SHIFT+MENU: back to the launcher");
}

//-----------------------------------------------------------------------------
static void draw_scene(void)
{
    draw_background();

    for (int i = 0; i < MAX_PIPES; i++) {
        draw_pipe(&g_game.pipes[i]);
        g_game.pipes[i].drawn_x = g_game.pipes[i].x;
    }

    draw_bird();
    g_game.bird.drawn_y = (int)g_game.bird.y;

    draw_score();
    draw_overlay();
}

//-----------------------------------------------------------------------------
static void update_bird(void)
{
    g_game.bird.velocity_y += GRAVITY;
    g_game.bird.y += g_game.bird.velocity_y;

    if (g_game.bird.y < 0) {
        g_game.bird.y = 0;
        g_game.bird.velocity_y = 0;
    }

    if (g_game.bird.y > SKY_HEIGHT - BIRD_SIZE) {
        g_game.bird.y = SKY_HEIGHT - BIRD_SIZE;
        g_game.game_over = true;
    }
}

//-----------------------------------------------------------------------------
static void update_pipes(void)
{
    int speed = g_pipe_speed[g_difficulty];
    int rightmost = g_game.pipes[0].x;

    for (int i = 1; i < MAX_PIPES; i++) {
        if (g_game.pipes[i].x > rightmost)
            rightmost = g_game.pipes[i].x;
    }

    for (int i = 0; i < MAX_PIPES; i++) {
        g_game.pipes[i].x -= speed;

        if (!g_game.pipes[i].passed && g_game.pipes[i].x + PIPE_WIDTH < BIRD_X) {
            g_game.pipes[i].passed = true;
            g_game.score++;

            if (g_game.score > g_high_score)
                g_high_score = g_game.score;
        }

        // Respawn one spacing behind the rightmost pipe. Respawning at
        // LCD_WIDTH, as the first version did, collapsed the spacing to
        // whatever order the pipes happened to be in.
        if (g_game.pipes[i].x + PIPE_WIDTH < 0) {
            int spawn_x = rightmost + PIPE_SPACING;

            if (spawn_x < LCD_WIDTH)
                spawn_x = LCD_WIDTH;

            place_pipe(i, spawn_x);
            rightmost = spawn_x;
        }
    }
}

//-----------------------------------------------------------------------------
static void check_collisions(void)
{
    int bird_top = (int)g_game.bird.y;
    int bird_bottom = bird_top + BIRD_SIZE;
    int gap = pipe_gap();

    for (int i = 0; i < MAX_PIPES; i++) {
        if (BIRD_X + BIRD_SIZE <= g_game.pipes[i].x ||
            BIRD_X >= g_game.pipes[i].x + PIPE_WIDTH)
            continue;

        if (bird_top < g_game.pipes[i].gap_y ||
            bird_bottom > g_game.pipes[i].gap_y + gap)
            g_game.game_over = true;
    }
}

//-----------------------------------------------------------------------------
void flappy_bird_init(void)
{
    srand(timer_ms()); // otherwise every run gets the same pipe layout

    reset_game();

    timer_add(&g_game_timer);
    g_game_timer = FRAME_MS;

    draw_scene();
}

//-----------------------------------------------------------------------------
void flappy_bird_redraw(void)
{
    draw_scene();
}

//-----------------------------------------------------------------------------
void flappy_bird_task(void)
{
    if (g_game_timer != 0)
        return;

    g_game_timer = FRAME_MS;

    if (!g_game.game_started || g_game.game_over)
        return;

    update_bird();
    update_pipes();
    check_collisions();

    // Repaint what moved: each pipe's two changed column strips, the band the
    // bird vacated, and any pipe that band cut a hole in
    for (int i = 0; i < MAX_PIPES; i++)
        update_pipe_pixels(&g_game.pipes[i], g_pipe_speed[g_difficulty]);

    erase_bird();

    for (int i = 0; i < MAX_PIPES; i++) {
        if (pipe_at_bird(&g_game.pipes[i]))
            draw_pipe(&g_game.pipes[i]);
    }

    draw_bird();
    g_game.bird.drawn_y = (int)g_game.bird.y;

    // Unconditionally, not only when the score changed: the panel sits inside
    // the play area, so every pipe that scrolls past it and the bird itself
    // punch holes through the text
    draw_score();

    if (g_game.game_over)
        draw_overlay();
}

//-----------------------------------------------------------------------------
static void flap(void)
{
    if (!g_game.game_started) {
        g_game.game_started = true;
        g_game.bird.velocity_y = JUMP_STRENGTH;
        draw_scene(); // clears the title box
        return;
    }

    if (g_game.game_over) {
        reset_game();
        g_game.game_started = true;
        g_game.bird.velocity_y = JUMP_STRENGTH;
        draw_scene();
        return;
    }

    g_game.bird.velocity_y = JUMP_STRENGTH;
}

//-----------------------------------------------------------------------------
void flappy_bird_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);

    if (buttons & (BTN_UP | BTN_MODE)) {
        if (!repeat)
            flap();
    }
}

//-----------------------------------------------------------------------------
void flappy_bird_cleanup(void)
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
    menu_close_popups(); // the game repaints itself when the popup closes
}

static const char *const g_help_lines[] =
{
    "MODE (or UP) - Flap / start / play again",
    "MENU         - This menu",
    "SHIFT+MENU   - Back to the launcher",
    "",
    "Fly through the gaps. Touching a pipe or",
    "the ground ends the run; one point per",
    "pipe passed.",
    "",
    "Difficulty sets both the gap size and the",
    "scroll speed: Easy 85 px at 2 px/frame,",
    "Normal 65 at 3, Hard 50 at 4.",
};

static const info_page_t g_help_page =
{
    .title = "Flappy Bird",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

//-----------------------------------------------------------------------------
// The gap size changes with the difficulty, so a gap chosen for the old one
// can push the bottom pipe past the ground once the new one is applied
static void difficulty_changed(void)
{
    int span = SKY_HEIGHT - pipe_gap() - 2 * GAP_MARGIN;
    int max_y = GAP_MARGIN + (span > 0 ? span : 0);

    for (int i = 0; i < MAX_PIPES; i++) {
        if (g_game.pipes[i].gap_y < GAP_MARGIN)
            g_game.pipes[i].gap_y = GAP_MARGIN;

        if (g_game.pipes[i].gap_y > max_y)
            g_game.pipes[i].gap_y = max_y;

        // Force a full repaint of the pipe: its outline changed shape
        g_game.pipes[i].drawn_x = OFFSCREEN;
    }
}

static const char *const g_difficulty_labels[] = { "Easy", "Normal", "Hard" };

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "Difficulty",
      .u.choice = { &g_difficulty, g_difficulty_labels,
                    ARRAY_SIZE(g_difficulty_labels), difficulty_changed } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Restart",
      .u.action = { action_restart, NULL } },
    { .kind = MI_ACTION, .label = "Help",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t flappy_bird_menu =
{
    .title = "Flappy Bird",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};
