/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * What an application needs around it to run on the host: a display it can
 * really draw into, timers the test drives by hand, and the few symbols the
 * launcher and the settings store would otherwise provide.
 *
 * The display is a real framebuffer, not a no-op. lcd_draw_buf() does not clip
 * on the hardware - a blit that runs off an edge is not invisible, the
 * controller wraps its address counter and paints the overflow somewhere else -
 * so the stub counts those and the test fails on them. That turns a class of
 * display corruption into something a build catches.
 *
 * Included by a test before it includes the application's .c file, so the
 * checks can see the model behind the screen while still driving the same
 * public interface the firmware does.
 */

#ifndef _GAMESTUB_H_
#define _GAMESTUB_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// utils.h cannot be compiled for the host - delay_ms() is inline ARM assembly
// and rbit8() is a CMSIS intrinsic - and the only thing the games want out of
// it is ARRAY_SIZE. Claiming its include guard leaves the file inert without a
// stub tree that would have to be kept in step with it. The vendor CMSIS
// header, which the games include only because utils.h assumes it, comes from
// tests/hoststub - listed last on the include path, so the real lcd.h and
// timer.h still win over the stubs sitting beside it.
#define _UTILS_H_
#define ARRAY_SIZE(x)  ((int)(sizeof(x) / sizeof(0[x])))

#include "config.h"
#include "buttons.h"
#include "lcd.h"
#include "game_gfx.h"

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

/*
 * The games' own dice, replacing the host's.
 *
 * Every game seeds with srand(timer_us()), and timer_us() below starts from a
 * constant precisely so that a run is repeatable. That is only half of it: the
 * same seed does not give the same sequence on two libcs, and glibc's rand()
 * and macOS's disagree from the first call. The pipe layout in Flappy Bird
 * came out different on Linux, the autopilot that clears five pipes on a Mac
 * scored zero there, and the test called that a regression - on a build where
 * nothing in the firmware had changed.
 *
 * So the tests bring dice of their own: the LCG from the C standard's own
 * example, defined here to win over libc for anything linked into a test. A
 * failure is then a property of the code, and reproducing one means running
 * the same test, not running it on the same laptop.
 */
static unsigned long g_rand_state = 1;

void srand(unsigned int seed) { g_rand_state = seed; }

int rand(void)
{
    g_rand_state = g_rand_state * 1103515245 + 12345;
    return (int)((g_rand_state >> 16) & 0x7fff);
}

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

// One millisecond of firmware time, without the application's task: the caller
// runs that, because only it knows which one
static void tick_timers(void)
{
    for (int t = 0; t < g_timer_count; t++)
    {
        if (*g_timers[t] > 0)
            (*g_timers[t])--;
    }

    g_now_us += 1000;
}

/*- Reporting ---------------------------------------------------------------*/
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

/*- Artwork -----------------------------------------------------------------*/
// A row typed one character short is not a compile error - it just comes out
// with a transparent column nobody drew. This is what catches a miscounted row
// of text art, and an icon that uses a character the palette does not know.
static void check_sprites(const sprite_t *const *sprites, const char *const *names,
    int count)
{
    for (int s = 0; s < count; s++)
    {
        int short_rows = 0;
        int unknown = 0;
        char label[64];

        for (int y = 0; y < GFX_TILE; y++)
        {
            if ((int)strlen(sprites[s]->rows[y]) != GFX_TILE)
            {
                short_rows++;
                continue;
            }

            for (int x = 0; x < GFX_TILE; x++)
            {
                char c = sprites[s]->rows[y][x];
                uint16_t color;

                if ('.' != c && !gfx_pixel_color(c, &color))
                    unknown++;
            }
        }

        snprintf(label, sizeof(label), "%s: every row is %d characters",
            names[s], GFX_TILE);
        check(label, short_rows, 0);

        snprintf(label, sizeof(label), "%s: every pixel is in the palette",
            names[s]);
        check(label, unknown, 0);
    }
}

#endif // _GAMESTUB_H_
