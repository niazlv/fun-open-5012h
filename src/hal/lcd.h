/*
 * Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _LCD_H_
#define _LCD_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "fonts.h"
#include "images.h"

/*- Definitions -------------------------------------------------------------*/
#define LCD_WIDTH      320
#define LCD_HEIGHT     240

#define LCD_COLOR(r, g, b) \
    ((((r) << 8) & 0xf800) | (((g) << 3) & 0x07e0) | (((b) >> 3) & 0x001f))

#define LCD_BLACK_COLOR        LCD_COLOR(0, 0, 0)
#define LCD_WHITE_COLOR        LCD_COLOR(255, 255, 255)
#define LCD_RED_COLOR          LCD_COLOR(255, 0, 0)
#define LCD_GREEN_COLOR        LCD_COLOR(0, 255, 0)
#define LCD_BLUE_COLOR         LCD_COLOR(0, 0, 255)

/*
 * Auto-dim on inactivity. The backlight is the largest single draw on the
 * battery, and a screen nobody has touched for minutes is spending it on
 * nothing. Dimmed is DIM, never off: the image stays readable, and the panel
 * is never put to sleep - what is on it is still an instrument's reading.
 *
 * How long is config.lcd_dim_timeout, an index into lcd_dim_labels where 0
 * is Off. The count is a macro because the menu table that offers these is a
 * const initialiser and needs it at compile time.
 */
#define LCD_DIM_COUNT  6
extern const char *const lcd_dim_labels[LCD_DIM_COUNT];

/*- Prototypes --------------------------------------------------------------*/
void lcd_init(void);
void lcd_set_backlight_level(int level);

// Start the inactivity clock. After config_init(), which is where the timeout
// comes from, and after the first lcd_set_backlight_level().
void lcd_backlight_init(void);

// The user is here: full brightness, and the wait starts again. Called for
// every button event, repeats included.
void lcd_backlight_activity(void);

// One pass of the inactivity clock. may_dim says whether what is on screen
// right now is allowed to be dimmed - the caller knows that, this does not.
void lcd_backlight_task(bool may_dim);
void lcd_draw_pixel(int x, int y, int color);
void lcd_draw_buf(int x, int y, int w, int h, const uint16_t *buf);
void lcd_draw_indexed(int x, int y, int w, int h, const uint8_t *pix,
    const uint16_t *palette);
void lcd_draw_image(int x, int y, const Image *image);
void lcd_draw_rect(int x, int y, int w, int h, int color);
void lcd_fill_rect(int x, int y, int w, int h, int color);
void lcd_hline(int x0, int x1, int y, int color);
void lcd_vline(int x, int y0, int y1, int color);
void lcd_set_font(const Font *font);

/*
 * Draw every glyph pixel this many times over, in both directions: scale 2 turns
 * the 8x16 font into a 16x32 one. Stays set until it is set back, like the font
 * and the colours do.
 *
 * Doubling rather than a third font, because the alternative is ~3 KB of flash
 * for ninety-five glyphs at a size that is only used by the two bars and only at
 * the large text setting. What it costs instead is a division per pixel column
 * inside lcd_putc, on a bus that spends ~200 ns a pixel regardless.
 */
void lcd_set_text_scale(int scale);
int lcd_text_scale(void);

// Pixel width and height of one glyph as it will be drawn, scale included: the
// callers that lay text out by hand should ask rather than multiply by 6 or 8.
int lcd_glyph_w(void);
int lcd_glyph_h(void);
int lcd_text_w(const char *str);

void lcd_set_color(int bg, int fg);
void lcd_putc(int x, int y, char ch);
void lcd_puts(int x, int y, const char *str);

#endif // _LCD_H_

