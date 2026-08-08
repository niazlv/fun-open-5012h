/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Host test stub: drawing is a no-op, the tests are about the ring */
#ifndef _HOSTSTUB_LCD_H_
#define _HOSTSTUB_LCD_H_

#include <stdint.h>

/* The geometry and the RGB565 packing below are copies of src/hal/lcd.h -
 * nothing checks they agree. If the panel size or the pixel format ever
 * changes there, change it here too, or the coredump test renders into a
 * screen that no longer exists. */
#define LCD_WIDTH   320
#define LCD_HEIGHT  240

#define LCD_COLOR(r, g, b) \
    ((((r) << 8) & 0xf800) | (((g) << 3) & 0x07e0) | (((b) >> 3) & 0x001f))

#define LCD_BLACK_COLOR  LCD_COLOR(0, 0, 0)
#define LCD_WHITE_COLOR  LCD_COLOR(255, 255, 255)

typedef int Font;
extern const Font *FONT_SMALL;
extern const Font *FONT_LARGE;

void lcd_fill_rect(int x, int y, int w, int h, int color);
void lcd_draw_rect(int x, int y, int w, int h, int color);
void lcd_set_font(const Font *font);
void lcd_set_color(int bg, int fg);
void lcd_puts(int x, int y, const char *str);

#endif
