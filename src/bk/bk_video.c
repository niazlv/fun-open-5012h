/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The screen.
 *
 * A BK's video memory is 16 KB read straight out by the controller: 256 lines
 * of 64 bytes, two bits a dot, the leftmost dot in the lowest bits. There is no
 * attribute plane, no character generator and no sprite - what is in memory is
 * what is on the tube, which is why every game on this machine is a program
 * that writes bytes into 040000 as fast as it can.
 *
 * Getting it onto this panel is two problems.
 *
 * The first is that 256 lines do not fit in 240 rows. Neither answer is right
 * for everything, so both are here: crop, which keeps the scale and loses the
 * border, and squash, which drops one line in sixteen.
 *
 * The second is the panel itself. It is bit-banged, about 200 ns a pixel, so a
 * full frame is 12 ms whatever this file does - a quarter of the time a 50 Hz
 * machine has between refreshes, spent before a single instruction is
 * emulated. What makes the emulator playable is not painting the rows that did
 * not change: the write path marks the lines a program touched, and a frame of
 * a game that is moving a few sprites around costs a tenth of a full one.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lcd.h"
#include "bk_config.h"
#include "bk_video.h"
#include "bk_mem.h"
#include "bk_io.h"

/*- Definitions -------------------------------------------------------------*/
#define VIEW_H_FULL             240
#define STATUS_H                16

#define COLOR_W                 256
#define COLOR_X                 ((LCD_WIDTH - COLOR_W) / 2)

#define MONO_W                  LCD_WIDTH   // a 320 dot window of the 512
#define MONO_X                  0

/*
 * How many panel rows are converted before they are pushed.
 *
 * One call per row would spend more time setting up a transfer window than
 * sending it; one call for the whole screen would need 76 KB of buffer, and
 * the machine being emulated has taken 64 KB of the 120 already. Sixteen rows
 * is 5 KB and fifteen transfers for a full repaint, and the setup is then
 * under a percent of the time.
 *
 * Not a static array, though. Everything in this firmware's .bss lands in the
 * 64 KB of tightly coupled memory, which is 83% full before this file is
 * compiled at all - so the caller supplies the buffer out of the main SRAM the
 * capture ring is not using, which is where the emulated machine already is.
 */
#define BLIT_ROWS               16

/*- Variables ---------------------------------------------------------------*/
/*
 * Palette zero of a BK-0010: black, blue, green, red. The other fifteen are a
 * BK-0011 feature, and software written for one is on the same chip as
 * software written for the other, so the whole table is here.
 *
 * Taken from the values every emulator of this machine agrees on, converted to
 * the panel's 5-6-5 at build time.
 */
#if BK_VIDEO_PALETTES
static const uint16_t g_palettes[16][4] =
{
    { LCD_COLOR(0,0,0), LCD_COLOR(0,0,255),   LCD_COLOR(0,255,0),   LCD_COLOR(255,0,0)     },
    { LCD_COLOR(0,0,0), LCD_COLOR(255,255,0), LCD_COLOR(255,0,255), LCD_COLOR(255,0,0)     },
    { LCD_COLOR(0,0,0), LCD_COLOR(0,255,255), LCD_COLOR(0,0,255),   LCD_COLOR(255,0,255)   },
    { LCD_COLOR(0,0,0), LCD_COLOR(0,255,0),   LCD_COLOR(0,255,255), LCD_COLOR(255,255,0)   },
    { LCD_COLOR(0,0,0), LCD_COLOR(255,0,255), LCD_COLOR(0,255,255), LCD_COLOR(255,255,255) },
    { LCD_COLOR(0,0,0), LCD_COLOR(255,255,255), LCD_COLOR(255,255,255), LCD_COLOR(255,255,255) },
    { LCD_COLOR(0,0,0), LCD_COLOR(192,0,0),   LCD_COLOR(142,0,0),   LCD_COLOR(255,0,0)     },
    { LCD_COLOR(0,0,0), LCD_COLOR(192,255,0), LCD_COLOR(142,255,0), LCD_COLOR(255,255,0)   },
    { LCD_COLOR(0,0,0), LCD_COLOR(192,0,255), LCD_COLOR(142,0,255), LCD_COLOR(255,0,255)   },
    { LCD_COLOR(0,0,0), LCD_COLOR(142,255,0), LCD_COLOR(142,0,255), LCD_COLOR(142,0,0)     },
    { LCD_COLOR(0,0,0), LCD_COLOR(192,255,0), LCD_COLOR(192,0,255), LCD_COLOR(192,0,0)     },
    { LCD_COLOR(0,0,0), LCD_COLOR(0,255,255), LCD_COLOR(255,255,0), LCD_COLOR(255,0,0)     },
    { LCD_COLOR(0,0,0), LCD_COLOR(255,0,0),   LCD_COLOR(0,255,0),   LCD_COLOR(0,255,255)   },
    { LCD_COLOR(0,0,0), LCD_COLOR(0,255,255), LCD_COLOR(255,255,0), LCD_COLOR(255,255,255) },
    { LCD_COLOR(0,0,0), LCD_COLOR(255,255,0), LCD_COLOR(0,255,0),   LCD_COLOR(255,255,255) },
    { LCD_COLOR(0,0,0), LCD_COLOR(0,255,255), LCD_COLOR(0,255,0),   LCD_COLOR(255,255,255) },
};
#else
static const uint16_t g_palettes[1][4] =
{
    { LCD_COLOR(0,0,0), LCD_COLOR(0,0,255), LCD_COLOR(0,255,0), LCD_COLOR(255,0,0) },
};
#endif

static const uint16_t g_mono[4] =
{
    LCD_COLOR(0,0,0), LCD_COLOR(255,255,255),
    LCD_COLOR(0,0,0), LCD_COLOR(255,255,255),
};

static bk_screen_t g_screen = BK_SCREEN_COLOR;
static bk_fit_t g_fit = BK_FIT_CROP;
static int g_top = 8;               // crop: which line lands on panel row 0
static int g_pan;                   // mono: which dot lands on panel column 0
static bool g_status;

static uint8_t *g_buf;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void bk_video_init(void)
{
    bk_vram_mark_all();
}

//-----------------------------------------------------------------------------
void bk_video_set_buffer(uint8_t *buf)
{
    g_buf = buf;
}

//-----------------------------------------------------------------------------
void bk_video_set_screen(bk_screen_t screen)
{
    if (screen != g_screen)
    {
        g_screen = screen;
        lcd_fill_rect(0, 0, LCD_WIDTH, VIEW_H_FULL, LCD_BLACK_COLOR);
        bk_vram_mark_all();
    }
}

//-----------------------------------------------------------------------------
void bk_video_set_fit(bk_fit_t fit)
{
    g_fit = fit;
    bk_vram_mark_all();
}

//-----------------------------------------------------------------------------
void bk_video_set_top(int line)
{
    g_top = (line < 0) ? 0 : (line > 16 ? 16 : line);
    bk_vram_mark_all();
}

//-----------------------------------------------------------------------------
void bk_video_set_pan(int dot)
{
    g_pan = (dot < 0) ? 0 : (dot > 512 - MONO_W ? 512 - MONO_W : dot);
    bk_vram_mark_all();
}

//-----------------------------------------------------------------------------
void bk_video_set_status(bool on)
{
    if (on != g_status)
    {
        g_status = on;
        lcd_fill_rect(0, 0, LCD_WIDTH, VIEW_H_FULL, LCD_BLACK_COLOR);
        bk_vram_mark_all();
    }
}

//-----------------------------------------------------------------------------
int bk_video_bottom(void)
{
    return g_status ? (VIEW_H_FULL - STATUS_H) : VIEW_H_FULL;
}

//-----------------------------------------------------------------------------
const uint16_t *bk_video_palette(void)
{
    if (BK_SCREEN_MONO == g_screen)
        return g_mono;

    return g_palettes[bk_io_palette() % (sizeof(g_palettes) / sizeof(g_palettes[0]))];
}

//-----------------------------------------------------------------------------
// Which of the machine's 256 lines a panel row shows, before the scroll
// register is applied.
//
// Squash uses 256/240 rather than "drop every sixteenth", which comes to the
// same set of dropped lines but spreads them evenly instead of leaving a seam
// every sixteen rows.
static inline unsigned source_line(int row, int view_h)
{
    if (BK_FIT_SQUASH == g_fit)
        return (unsigned)(row * BK_VRAM_LINES / view_h);

    return (unsigned)(row + g_top);
}

//-----------------------------------------------------------------------------
#if BK_VIDEO_COLOR
// 64 bytes into 256 indices, four to a byte, lowest bits leftmost.
static void expand_color(const uint8_t *src, uint8_t *dst)
{
    for (unsigned i = 0; i < BK_VRAM_STRIDE; i++)
    {
        uint8_t b = src[i];

        dst[0] = (uint8_t)(b & 3);
        dst[1] = (uint8_t)((b >> 2) & 3);
        dst[2] = (uint8_t)((b >> 4) & 3);
        dst[3] = (uint8_t)(b >> 6);
        dst += 4;
    }
}
#endif

//-----------------------------------------------------------------------------
#if BK_VIDEO_MONO
// A 320 dot window of the line's 512, one bit a dot, lowest bit leftmost.
static void expand_mono(const uint8_t *src, uint8_t *dst, int pan)
{
    int bit = pan & 7;

    src += pan >> 3;

    for (int x = 0; x < MONO_W; x++)
    {
        *dst++ = (uint8_t)((*src >> bit) & 1);

        if (8 == ++bit)
        {
            bit = 0;
            src++;
        }
    }
}
#endif

//-----------------------------------------------------------------------------
// One run of consecutive panel rows, converted and pushed in a single transfer
static void blit(int row0, int count, int x, int w)
{
    lcd_draw_indexed(x, row0, w, count, g_buf, bk_video_palette());
}

//-----------------------------------------------------------------------------
int bk_video_draw(bool full)
{
    const uint8_t *vram = bk_mem + BK_VRAM_BASE;
    uint16_t scroll = bk_io_scroll();
    unsigned offset;
    int view_h = bk_video_bottom();
    int lines_shown;
    int x, w;
    int run = 0, run_start = 0;
    int painted = 0;

    if (NULL == bk_mem || NULL == g_buf)
        return 0;

#if BK_VIDEO_MONO
    if (BK_SCREEN_MONO == g_screen)
    {
        x = MONO_X;
        w = MONO_W;
    }
    else
#endif
    {
        x = COLOR_X;
        w = COLOR_W;
    }

    /*
     * The scroll register holds the number of the line that lands at the top,
     * counted from 0330 - which is the value the monitor leaves behind and the
     * one that means "no scroll". Anything below it has wrapped round the
     * bottom of the 256, hence the 050.
     */
    offset = scroll & BK_SCROLL_LINE;
    offset = (offset >= BK_SCROLL_HOME) ? offset - BK_SCROLL_HOME : offset + 050u;

    /*
     * Bit 9 clear is the small screen: the controller shows 64 lines and
     * blanks the rest. Software uses it to hide a screen it is building.
     */
    lines_shown = (scroll & BK_SCROLL_FULL) ? BK_VRAM_LINES : 64;

    for (int row = 0; row <= view_h; row++)
    {
        bool want = false;
        unsigned line = 0;

        if (row < view_h)
        {
            unsigned src = source_line(row, view_h);

            if (src < (unsigned)lines_shown)
            {
                line = (src + offset) & 0377u;

#if BK_VIDEO_DIRTY
                want = full ||
                    (0 != (bk_vram_dirty[line >> 5] & (1u << (line & 31))));
#else
                (void)full;
                want = true;
#endif
            }
            else if (full)
            {
                // Blanked by the small screen mode, and only worth clearing
                // when everything is being repainted anyway
                lcd_fill_rect(x, row, w, 1, LCD_BLACK_COLOR);
            }
        }

        if (want)
        {
            uint8_t *dst = g_buf + (size_t)run * w;
            const uint8_t *src = vram + line * BK_VRAM_STRIDE;

            if (0 == run)
                run_start = row;

#if BK_VIDEO_MONO
            if (BK_SCREEN_MONO == g_screen)
                expand_mono(src, dst, g_pan);
            else
#endif
#if BK_VIDEO_COLOR
                expand_color(src, dst);
#else
                memset(dst, 0, w);
#endif

            if (++run < BLIT_ROWS)
                continue;
        }

        // Either the run ended or it filled the buffer
        if (run > 0)
        {
            blit(run_start, run, x, w);
            painted += run;
            run = 0;
        }
    }

#if BK_VIDEO_DIRTY
    memset(bk_vram_dirty, 0, sizeof(bk_vram_dirty));
#endif

    return painted;
}
