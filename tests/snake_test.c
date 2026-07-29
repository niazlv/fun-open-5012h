/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for the Snake application.
 *
 * snake_game.c is #included rather than linked, so the tests can see the model
 * behind the screen: the grid, the body and the state machine. What they drive
 * is still the public interface - buttons in, timer ticks forward - so the
 * state machine under test is the one that runs on the device.
 *
 * The display is a real framebuffer here, not a no-op. That matters because
 * lcd_draw_buf() does not clip on the hardware: a blit that runs off an edge
 * is not invisible, the controller wraps the address counter and paints the
 * overflow somewhere else. The stub below fails the test instead, which turns
 * a display corruption into something a build catches.
 *
 * Build & run (no hardware needed), from the repository root:
 *   make test
 *
 * With a path argument it also writes that frame out as a .ppm, which is how
 * the artwork gets looked at without flashing a board:
 *   ./tests/build/snake_test /tmp/snake.ppm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// utils.h cannot be compiled for the host - delay_ms() is inline ARM assembly
// and rbit8() is a CMSIS intrinsic - and the only thing snake_game.c wants out
// of it is ARRAY_SIZE. Claiming its include guard leaves the file inert without
// a stub tree that would have to be kept in step with it. The vendor CMSIS
// header, which the game includes only because utils.h assumes it, comes from
// tests/hoststub - listed last on the include path, so the real lcd.h and
// timer.h still win over the stubs sitting beside it.
#define _UTILS_H_
#define ARRAY_SIZE(x)  ((int)(sizeof(x) / sizeof(0[x])))

#include "config.h"
#include "buttons.h"
#include "lcd.h"

/*- The display -------------------------------------------------------------*/
static uint16_t g_fb[LCD_HEIGHT][LCD_WIDTH];
static int g_bg_color, g_fg_color;
static const Font *g_font;
static int g_failures;
static int g_offscreen_blits;

static void fb_set(int x, int y, uint16_t color)
{
    if (x >= 0 && x < LCD_WIDTH && y >= 0 && y < LCD_HEIGHT)
        g_fb[y][x] = color;
}

void lcd_draw_buf(int x, int y, int w, int h, const uint16_t *buf)
{
    // The hardware would wrap this into the middle of the picture
    if (x < 0 || y < 0 || x + w > LCD_WIDTH || y + h > LCD_HEIGHT)
    {
        if (0 == g_offscreen_blits)
            printf("  FAIL blit off the panel: %d,%d %dx%d\n", x, y, w, h);

        g_offscreen_blits++;
        return;
    }

    for (int j = 0; j < h; j++)
    {
        for (int i = 0; i < w; i++)
            g_fb[y + j][x + i] = buf[j * w + i];
    }
}

void lcd_fill_rect(int x, int y, int w, int h, int color)
{
    for (int j = y; j < y + h; j++)
    {
        for (int i = x; i < x + w; i++)
            fb_set(i, j, (uint16_t)color);
    }
}

void lcd_draw_rect(int x, int y, int w, int h, int color)
{
    w -= 1;
    h -= 1;

    for (int i = 0; i <= w; i++)
    {
        fb_set(x + i, y, (uint16_t)color);
        fb_set(x + i, y + h, (uint16_t)color);
    }

    for (int j = 0; j <= h; j++)
    {
        fb_set(x, y + j, (uint16_t)color);
        fb_set(x + w, y + j, (uint16_t)color);
    }
}

void lcd_set_font(const Font *font) { g_font = font; }
void lcd_set_color(int bg, int fg) { g_bg_color = bg; g_fg_color = fg; }

void lcd_putc(int x, int y, char ch)
{
    const uint8_t *bitmap;
    int size = g_font->width * g_font->height;

    // Same rule as the firmware: a glyph that does not fit is dropped
    if (x < 0 || y < 0 || x + g_font->width > LCD_WIDTH ||
        y + g_font->height > LCD_HEIGHT)
        return;

    if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
        ch = '?';

    bitmap = g_font->data + (ch - FONT_FIRST_CHAR) * g_font->pitch;

    for (int i = 0; i < size; i++)
    {
        int pixel = (bitmap[i / 8] >> (i % 8)) & 1;

        fb_set(x + i % g_font->width, y + i / g_font->width,
            (uint16_t)(pixel ? g_fg_color : g_bg_color));
    }
}

void lcd_puts(int x, int y, const char *str)
{
    while (*str)
    {
        lcd_putc(x, y, *str++);
        x += g_font->width;
    }
}

/*- Everything else the application reaches for -----------------------------*/
Config config;

static uint32_t g_now_us = 1234567;
static int *g_timers[8];
static int g_timer_count;

uint32_t timer_us(void) { return g_now_us; }
uint32_t timer_ms(void) { return g_now_us / 1000; }

void timer_add(int *timer)
{
    if (g_timer_count < (int)ARRAY_SIZE(g_timers))
        g_timers[g_timer_count++] = timer;
}

void timer_remove(int *timer)
{
    for (int i = 0; i < g_timer_count; i++)
    {
        if (g_timers[i] == timer)
        {
            g_timers[i] = g_timers[--g_timer_count];
            return;
        }
    }
}

void menu_close_popups(void) {}
void menu_action_info(const void *arg) { (void)arg; }

#include "../src/apps/snake_game.c"

/*- Test plumbing -----------------------------------------------------------*/

static void check(const char *name, long got, long want)
{
    if (got == want)
    {
        printf("  PASS %-46s %ld\n", name, got);
    }
    else
    {
        printf("  FAIL %-46s got %ld  want %ld\n", name, got, want);
        g_failures++;
    }
}

static void check_true(const char *name, bool ok)
{
    check(name, ok ? 1 : 0, 1);
}

// One millisecond of firmware time: the timers the application registered come
// down by one, and its task runs, exactly as the main loop would do it
static void advance(int ms)
{
    for (int i = 0; i < ms; i++)
    {
        for (int t = 0; t < g_timer_count; t++)
        {
            if (*g_timers[t] > 0)
                (*g_timers[t])--;
        }

        g_now_us += 1000;
        snake_game_task();
    }
}

static void restart_app(void)
{
    snake_game_cleanup();
    snake_game_init();
}

/*- Invariants --------------------------------------------------------------*/

// Everything that must be true of the model between any two steps. The grid is
// a second copy of where the snake is, kept for O(1) collision tests, and a
// grid that has drifted from the body array is the failure mode that costs a
// player a run for no visible reason.
static int t_check_model(const char *where)
{
    int snake_cells = 0;
    int fruit_cells = 0;
    int bonus_cells = 0;
    int bad = 0;

    for (int cy = 0; cy < GRID_H; cy++)
    {
        for (int cx = 0; cx < GRID_W; cx++)
        {
            switch (g_cell[cy][cx])
            {
                case CELL_SNAKE: snake_cells++; break;
                case CELL_FRUIT: fruit_cells++; break;
                case CELL_BONUS: bonus_cells++; break;
                default: break;
            }
        }
    }

    if (snake_cells != g_len)
        bad++;

    if (fruit_cells != 1)
        bad++;

    if (bonus_cells != (g_bonus_on ? 1 : 0))
        bad++;

    for (int i = 0; i < g_len; i++)
    {
        int x = body_x(i);
        int y = body_y(i);

        if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
        {
            bad++;
            continue;
        }

        if (CELL_SNAKE != g_cell[y][x])
            bad++;

        // Every pair of neighbours in the array has to be a pair of neighbours
        // on the grid, or the drawing code has no shape to join
        if (i > 0 && 0 == link_mask(x, y, body_x(i - 1), body_y(i - 1)))
            bad++;

        for (int j = i + 1; j < g_len; j++)
        {
            if (g_body[i] == g_body[j])
                bad++;
        }
    }

    if (bad)
        printf("  FAIL model broken at %s (%d problems)\n", where, bad);

    return bad;
}

/*- Map checks --------------------------------------------------------------*/

static void t_maps(void)
{
    printf("levels:\n");

    for (int i = 0; i < snake_level_count; i++)
    {
        const snake_level_t *level = &snake_levels[i];
        char name[64];
        int starts = 0;
        int short_rows = 0;

        for (int cy = 0; cy < GRID_H; cy++)
        {
            if ((int)strlen(level->rows[cy]) != GRID_W)
                short_rows++;

            for (int cx = 0; cx < GRID_W; cx++)
            {
                char c = level->rows[cy][cx];

                if ('>' == c || '<' == c || '^' == c || 'v' == c)
                    starts++;
            }
        }

        snprintf(name, sizeof(name), "%s: rows are %d cells", level->name, GRID_W);
        check(name, short_rows, 0);

        snprintf(name, sizeof(name), "%s: exactly one start", level->name);
        check(name, starts, 1);
    }
}

// The three starting segments have to land on clear cells, including the wall
// ring the game adds when the border is lethal, and every empty cell has to be
// reachable from the snake - a fruit spawns on any of them, and one behind a
// wall is a run that cannot be finished.
static void t_reachable(bool wrap)
{
    printf("reachability, %s:\n", wrap ? "wrap" : "walls");

    g_wrap = wrap;
    g_mode = MODE_LEVELS;

    for (int i = 0; i < snake_level_count; i++)
    {
        static uint8_t seen[GRID_H][GRID_W];
        static uint16_t queue[GRID_H * GRID_W];
        char name[64];
        int head = 0, tail = 0;
        int empty = 0, found = 0;
        int on_wall = 0;

        g_level = i;
        g_score = 0;
        load_level();

        for (int s = 0; s < g_len; s++)
        {
            if (CELL_WALL == g_cell[body_y(s)][body_x(s)])
                on_wall++;
        }

        snprintf(name, sizeof(name), "%s: start is clear of walls",
            snake_levels[i].name);
        check(name, on_wall, 0);

        memset(seen, 0, sizeof(seen));

        for (int cy = 0; cy < GRID_H; cy++)
        {
            for (int cx = 0; cx < GRID_W; cx++)
            {
                if (CELL_WALL != g_cell[cy][cx])
                    empty++;
            }
        }

        queue[tail++] = g_body[0];
        seen[body_y(0)][body_x(0)] = 1;

        while (head < tail)
        {
            int x = queue[head] & 0xff;
            int y = queue[head] >> 8;
            static const int dx[] = { 0, 0, -1, 1 };
            static const int dy[] = { -1, 1, 0, 0 };

            head++;
            found++;

            for (int d = 0; d < 4; d++)
            {
                int nx = x + dx[d];
                int ny = y + dy[d];

                if (wrap)
                {
                    if (nx < 0) nx = GRID_W - 1;
                    if (nx >= GRID_W) nx = 0;
                    if (ny < 0) ny = GRID_H - 1;
                    if (ny >= GRID_H) ny = 0;
                }
                else if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H)
                    continue;

                if (seen[ny][nx] || CELL_WALL == g_cell[ny][nx])
                    continue;

                seen[ny][nx] = 1;
                queue[tail++] = (uint16_t)((ny << 8) | nx);
            }
        }

        snprintf(name, sizeof(name), "%s: every free cell is reachable",
            snake_levels[i].name);
        check(name, found, empty);
    }
}

/*- A player ----------------------------------------------------------------*/

// Breadth-first from the head to the fruit, over cells that are free or about
// to be. Good enough to keep a snake alive for a few hundred fruit, which is
// all this is for: it is a way to exercise the game, not a way to win it.
static int t_best_direction(void)
{
    static int16_t from[GRID_H][GRID_W];
    static uint16_t queue[GRID_H * GRID_W];
    int head = 0, tail = 0;
    static const int dx[] = { 0, 0, -1, 1 };
    static const int dy[] = { -1, 1, 0, 0 };
    static const int as_dir[] = { DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT };

    memset(from, -1, sizeof(from));

    queue[tail++] = g_body[0];
    from[body_y(0)][body_x(0)] = -2;

    while (head < tail)
    {
        int x = queue[head] & 0xff;
        int y = queue[head] >> 8;

        head++;

        for (int d = 0; d < 4; d++)
        {
            int nx = x + dx[d];
            int ny = y + dy[d];

            if (g_wrap)
            {
                if (nx < 0) nx = GRID_W - 1;
                if (nx >= GRID_W) nx = 0;
                if (ny < 0) ny = GRID_H - 1;
                if (ny >= GRID_H) ny = 0;
            }
            else if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H)
                continue;

            if (from[ny][nx] != -1)
                continue;

            if (CELL_WALL == g_cell[ny][nx] || CELL_SNAKE == g_cell[ny][nx])
                continue;

            from[ny][nx] = (int16_t)((-2 == from[y][x]) ? as_dir[d] : from[y][x]);
            queue[tail++] = (uint16_t)((ny << 8) | nx);
        }
    }

    if (from[g_fruit_y][g_fruit_x] >= 0)
        return from[g_fruit_y][g_fruit_x];

    // No route to the fruit: keep moving into anything that is not a wall
    for (int d = 0; d < 4; d++)
    {
        int nx = body_x(0) + dx[d];
        int ny = body_y(0) + dy[d];

        if (g_wrap)
        {
            if (nx < 0) nx = GRID_W - 1;
            if (nx >= GRID_W) nx = 0;
            if (ny < 0) ny = GRID_H - 1;
            if (ny >= GRID_H) ny = 0;
        }
        else if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H)
            continue;

        if (CELL_EMPTY == g_cell[ny][nx] || CELL_FRUIT == g_cell[ny][nx] ||
            CELL_BONUS == g_cell[ny][nx])
            return as_dir[d];
    }

    return g_dir;
}

static void t_press(int button)
{
    snake_game_buttons_handler(button);
}

// Plays until the snake has eaten `fruit_wanted`, it dies, or the clock runs
// out, checking the model after every step it takes
static int t_play(int fruit_wanted, int *deaths, int *model_faults)
{
    int eaten = 0;
    int last_len = g_len;
    int ms = 0;

    while (eaten < fruit_wanted && ms < 400000)
    {
        if (ST_PLAY == g_state)
        {
            switch (t_best_direction())
            {
                case DIR_UP:    t_press(BTN_UP); break;
                case DIR_DOWN:  t_press(BTN_DOWN); break;
                case DIR_LEFT:  t_press(BTN_LEFT); break;
                default:        t_press(BTN_RIGHT); break;
            }
        }
        else if (ST_OVER == g_state)
        {
            (*deaths)++;
            t_press(BTN_MODE);
        }
        else if (ST_TITLE == g_state || ST_PAUSE == g_state ||
                 ST_CLEAR == g_state)
        {
            t_press(BTN_MODE);
        }

        advance(1);
        ms++;

        if (g_len != last_len)
        {
            if (g_len > last_len)
                eaten++;

            last_len = g_len;
        }

        if (ST_PLAY == g_state)
            *model_faults += t_check_model("play");
    }

    return eaten;
}

/*- Hand-built situations ---------------------------------------------------*/

// Replaces the snake with an explicit one. The cells are head first, and have
// to be a chain of neighbours - the same thing the model asks of itself.
static void t_place_snake(const int *cells, int count)
{
    for (int cy = 0; cy < GRID_H; cy++)
    {
        for (int cx = 0; cx < GRID_W; cx++)
        {
            if (CELL_SNAKE == g_cell[cy][cx])
                g_cell[cy][cx] = CELL_EMPTY;
        }
    }

    g_len = count;

    for (int i = 0; i < count; i++)
    {
        int x = cells[i * 2];
        int y = cells[i * 2 + 1];

        g_body[i] = (uint16_t)((y << 8) | x);
        g_cell[y][x] = CELL_SNAKE;
    }
}

static void t_park_fruit(int x, int y)
{
    if (CELL_FRUIT == g_cell[g_fruit_y][g_fruit_x])
        g_cell[g_fruit_y][g_fruit_x] = CELL_EMPTY;

    g_fruit_x = x;
    g_fruit_y = y;
    g_cell[y][x] = CELL_FRUIT;
}

static void t_rules(void)
{
    // A square of four, head top-left, with the tail directly below it
    static const int square[] = { 5, 5,  6, 5,  6, 6,  5, 6 };

    printf("rules:\n");

    g_mode = MODE_CLASSIC;
    g_wrap = false;
    restart_app();
    g_state = ST_PLAY;

    t_place_snake(square, 4);
    t_park_fruit(12, 8);
    g_dir = DIR_UP;
    g_dir_queued = DIR_UP;
    g_dirq_count = 0;
    queue_direction(DIR_LEFT);
    step_game();
    check_true("a turn out of the square is not a collision",
        ST_PLAY == g_state);

    t_place_snake(square, 4);
    t_park_fruit(12, 8);
    g_dir = DIR_RIGHT;
    g_dir_queued = DIR_RIGHT;
    g_dirq_count = 0;
    step_game();
    check_true("running into your own neck ends the run", ST_DYING == g_state);

    // The tail vacates the cell on the very move the head enters it
    g_state = ST_PLAY;
    t_place_snake(square, 4);
    t_park_fruit(12, 8);
    g_dir = DIR_DOWN;
    g_dir_queued = DIR_DOWN;
    g_dirq_count = 0;
    step_game();
    check_true("chasing your own tail is legal", ST_PLAY == g_state);
    check("and the snake is no longer for it", g_len, 4);

    // ... but not when that move also grows it, because then it does not move
    g_state = ST_PLAY;
    t_place_snake(square, 4);
    t_park_fruit(5, 4);
    g_dir = DIR_UP;
    g_dir_queued = DIR_UP;
    g_dirq_count = 0;
    step_game();
    check("eating grows the snake by one", g_len, 5);
    check("and scores", g_score, FRUIT_SCORE);

    // Two turns between two steps: the queue is what makes both of them count
    g_state = ST_PLAY;
    t_place_snake(square, 4);
    t_park_fruit(12, 8);
    g_dir = DIR_UP;
    g_dir_queued = DIR_UP;
    g_dirq_count = 0;
    queue_direction(DIR_LEFT);
    queue_direction(DIR_DOWN);
    check("both presses of a flick are queued", g_dirq_count, 2);
    step_game();
    check_true("the first one is taken", DIR_LEFT == g_dir);
    step_game();
    check_true("and the second one on the next step", DIR_DOWN == g_dir);

    // A reversal would drive the head straight into the neck
    g_dir = DIR_RIGHT;
    g_dir_queued = DIR_RIGHT;
    g_dirq_count = 0;
    queue_direction(DIR_LEFT);
    check("a 180 is not queued", g_dirq_count, 0);
}

static void t_wrap(void)
{
    static const int row[] = { 1, 6,  2, 6,  3, 6 };

    printf("wrap:\n");

    g_mode = MODE_CLASSIC;
    g_wrap = true;
    restart_app();
    g_state = ST_PLAY;

    t_place_snake(row, 3);
    t_park_fruit(10, 2);
    g_dir = DIR_LEFT;
    g_dir_queued = DIR_LEFT;
    g_dirq_count = 0;

    step_game();
    check("crossing the left edge lands on column 0", body_x(0), 0);
    step_game();
    check("and the next step comes back on the right",
        body_x(0), GRID_W - 1);
    check_true("without ending the run", ST_PLAY == g_state);
    check("no wall ring is laid down when the snake wraps",
        g_cell[0][0], CELL_EMPTY);

    g_wrap = false;
    restart_app();
    check("the ring is there when the border kills", g_cell[0][0], CELL_WALL);
}

static void t_campaign(void)
{
    int deaths = 0, faults = 0;

    printf("campaign:\n");

    g_mode = MODE_LEVELS;
    g_wrap = false;
    g_speed = SPEED_NORMAL;
    g_bonus_enabled = false;    // so the score below is only fruit
    restart_app();

    t_press(BTN_MODE);          // leave the title panel
    t_play(snake_levels[0].target, &deaths, &faults);

    // The last fruit of a level takes the state to the banner, and the banner
    // runs itself out into the next level
    advance(CLEAR_MS + READY_MS * 4);

    check("clearing level 1 opens level 2", g_level, 1);
    check("the score carries into it", g_level_score, snake_levels[0].target * FRUIT_SCORE);
    check("fruit eaten resets with the level", g_eaten, 0);

    // Straight to the end of the campaign, to see it come round again
    g_level = snake_level_count - 1;
    g_eaten = level_target();
    g_state = ST_CLEAR;
    next_level();

    check("the campaign comes round", g_level, 0);
    check("on a second lap", g_loop, 1);
    check_true("which asks for more fruit",
        level_target() > snake_levels[0].target);
    check_true("and runs quicker", step_interval() < snake_levels[0].step_ms);

    g_bonus_enabled = true;
}

static void t_bonus(void)
{
    printf("bonus:\n");

    g_mode = MODE_CLASSIC;
    g_wrap = false;
    g_bonus_enabled = true;
    restart_app();
    g_state = ST_PLAY;

    spawn_bonus();
    check_true("a bonus spawns", g_bonus_on);
    check("and takes a cell of its own", g_cell[g_bonus_y][g_bonus_x],
        CELL_BONUS);

    int bx = g_bonus_x;
    int by = g_bonus_y;

    for (int t = 0; t <= BONUS_MS; t += ANIM_MS)
        anim_tick();

    check_true("it does not stay forever", !g_bonus_on);
    check("and gives its cell back", g_cell[by][bx], CELL_EMPTY);

    // Eating one: parked next to the head, which then steps onto it
    static const int row[] = { 5, 6,  4, 6,  3, 6 };

    t_place_snake(row, 3);
    t_park_fruit(12, 2);
    g_score = 0;
    g_bonus_on = true;
    g_bonus_shown = true;
    g_bonus_x = 6;
    g_bonus_y = 6;
    g_cell[6][6] = CELL_BONUS;
    g_dir = DIR_RIGHT;
    g_dir_queued = DIR_RIGHT;
    g_dirq_count = 0;
    step_game();

    check("eating one scores", g_score, BONUS_SCORE);
    check_true("and is taken off the field", !g_bonus_on);
    check("without growing the snake", g_len, 3);

    g_bonus_enabled = false;
    g_bonus_on = false;
    spawn_bonus();
    check_true("the setting turns them off", !g_bonus_on);
    g_bonus_enabled = true;
}

static void t_persistence(void)
{
    printf("best score:\n");

    config.snake_high_score = 0;
    g_mode = MODE_CLASSIC;
    g_wrap = false;
    restart_app();

    g_state = ST_PLAY;
    g_score = 320;
    die();

    check("dying banks the score", g_best, 320);

    // The death animation is what carries the state into the game over panel,
    // and that is where the score reaches the settings store
    advance(2000);

    check_true("the game over panel is up", ST_OVER == g_state);
    check("and the best score went to flash", config.snake_high_score, 320);

    config.snake_high_score = 0;
}

/*- Rendering ---------------------------------------------------------------*/

static void write_ppm(const char *path)
{
    FILE *f = fopen(path, "wb");

    if (!f)
        return;

    fprintf(f, "P6\n%d %d\n255\n", LCD_WIDTH, LCD_HEIGHT);

    for (int y = 0; y < LCD_HEIGHT; y++)
    {
        for (int x = 0; x < LCD_WIDTH; x++)
        {
            uint16_t v = g_fb[y][x];
            uint8_t rgb[3];

            rgb[0] = (uint8_t)(((v >> 11) & 0x1f) * 255 / 31);
            rgb[1] = (uint8_t)(((v >> 5) & 0x3f) * 255 / 63);
            rgb[2] = (uint8_t)((v & 0x1f) * 255 / 31);

            fwrite(rgb, 1, 3, f);
        }
    }

    fclose(f);
    printf("wrote %s\n", path);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    int deaths = 0, faults = 0, eaten = 0;

    printf("snake\n\n");

    t_maps();
    printf("\n");

    t_reachable(false);
    printf("\n");
    t_reachable(true);
    printf("\n");

    t_rules();
    printf("\n");

    t_wrap();
    printf("\n");

    t_campaign();
    printf("\n");

    t_bonus();
    printf("\n");

    t_persistence();
    printf("\n");

    // A long run over every map, with the model checked after every step
    printf("play:\n");
    g_mode = MODE_LEVELS;
    g_wrap = false;
    g_speed = SPEED_INSANE;
    restart_app();
    t_press(BTN_MODE);
    eaten = t_play(120, &deaths, &faults);

    check_true("the player gets somewhere", eaten >= 40);
    check("the model holds all the way through", faults, 0);
    check("nothing was drawn off the panel", g_offscreen_blits, 0);

    // A frame to look at: the field as it stands, mid-run, with whatever the
    // player has grown into. draw_field() rather than snake_game_redraw(),
    // which would put the countdown panel over the thing worth seeing.
    if (argc > 1)
    {
        while (ST_PLAY != g_state)
        {
            t_press(BTN_MODE);
            advance(1);
        }

        t_play(14, &deaths, &faults);   // long enough to have corners in it

        while (ST_PLAY != g_state)
        {
            t_press(BTN_MODE);
            advance(1);
        }

        draw_hud();
        draw_field();
        write_ppm(argv[1]);
    }

    printf("\n%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED",
        g_failures);

    return g_failures ? 1 : 0;
}
