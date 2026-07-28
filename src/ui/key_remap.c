/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Key remap editor
 *
 * One row per remappable key, reading "the key on the left does what the key
 * on the right does", plus the switch that turns the whole translation on and
 * a row that puts every key back to itself. The table it edits lives in
 * input.c, which owns both the list of keys and the translation; this file
 * only draws it and takes the presses.
 *
 * The screen is driven with system keys alone - arrows, MODE, MENU - and not
 * with any key it can remap. That is not a style choice: a mapping under
 * edit is live, so a row driven by TRIG_UP would stop answering the moment
 * TRIG_UP was pointed somewhere else, and the way out of a mapping would
 * depend on the mapping. It also costs LEFT its usual "back", which is why
 * MENU says so in the footer.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include "buttons.h"
#include "config.h"
#include "ui.h"
#include "input.h"
#include "key_remap.h"

/*- Definitions -------------------------------------------------------------*/
// The info page palette, so the editor reads as part of the same firmware
#define KR_BG            LCD_COLOR(240, 240, 240)
#define KR_FG            LCD_COLOR(0, 0, 0)
#define KR_TITLE         LCD_COLOR(0, 70, 150)
#define KR_DIM           LCD_COLOR(110, 110, 110)
#define KR_RULE          LCD_COLOR(190, 190, 190)
#define KR_SEL_BG        LCD_COLOR(0, 120, 215)
#define KR_SEL_FG        LCD_COLOR(255, 255, 255)
#define KR_MOVED         LCD_COLOR(190, 60, 0)   // a key that no longer means itself
#define KR_OFF           LCD_COLOR(150, 150, 150)
#define KR_CAPTURE_BG    LCD_COLOR(255, 200, 0)

#define KR_X             20
#define KR_TITLE_Y       20
#define KR_ROW0_Y        44
#define KR_ROW_H         12
#define KR_BAND_H        11
#define KR_NAME_X        (KR_X + 4)
#define KR_ARROW_X       (KR_NAME_X + 8 * FW_SMALL)
#define KR_VALUE_X       (KR_ARROW_X + 3 * FW_SMALL)
#define KR_TEXT_R        (LCD_WIDTH - 24)
#define KR_RULE_Y        202
#define KR_FOOT_Y        207
#define KR_FOOT2_Y       217

#define FW_SMALL         6 // font_6x8 glyph width
#define FW_LARGE         8 // terminus_8x16 glyph width

// Row 0 is the enable switch and the last row restores the defaults; the
// keys themselves are everything in between
#define ROW_ENABLE       0
#define ROW_FIRST_KEY    1

/*- Variables ---------------------------------------------------------------*/
static struct
{
  int sel;
  bool capture;  // waiting for the key the selected row should act as
} g_kr;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static int row_count(void)
{
  return ROW_FIRST_KEY + key_remap_count() + 1;
}

//-----------------------------------------------------------------------------
static int row_reset(void)
{
  return row_count() - 1;
}

//-----------------------------------------------------------------------------
// The key a row edits, -1 for the two rows that are not a key
static int row_key(int row)
{
  if (row < ROW_FIRST_KEY || row >= row_reset())
    return -1;

  return row - ROW_FIRST_KEY;
}

//-----------------------------------------------------------------------------
static int row_y(int row)
{
  return KR_ROW0_Y + row * KR_ROW_H;
}

//-----------------------------------------------------------------------------
static void draw_row(int row)
{
  int key = row_key(row);
  bool selected = (row == g_kr.sel);
  int y = row_y(row);
  uint16_t bg = selected ? KR_SEL_BG : KR_BG;
  uint16_t fg = selected ? KR_SEL_FG : KR_FG;

  lcd_fill_rect(14, y - 2, LCD_WIDTH - 28, KR_BAND_H, bg);
  lcd_set_font(FONT_SMALL);
  lcd_set_color(bg, fg);

  if (ROW_ENABLE == row)
  {
    lcd_puts(KR_NAME_X, y, "Remapping");
    lcd_puts(KR_VALUE_X, y, config.key_remapping_enabled ? "ON" : "OFF");
    return;
  }

  if (key < 0)
  {
    lcd_puts(KR_NAME_X, y, "Every key back to itself");
    return;
  }

  lcd_puts(KR_NAME_X, y, key_remap_name(key));

  uint32_t target = key_remap_target(key);
  int index = key_remap_index_of(target);
  const char *value = (index < 0) ? "(off)" : key_remap_name(index);

  // The arrow is dimmed on a row that still says itself, so the rows that
  // were changed are the ones the eye lands on
  lcd_set_color(bg, selected ? KR_SEL_FG : KR_RULE);
  lcd_puts(KR_ARROW_X, y, "->");

  if (!selected)
  {
    if (index < 0)
      fg = KR_OFF;
    else if (index != key)
      fg = KR_MOVED;
  }

  lcd_set_color(bg, fg);
  lcd_puts(KR_VALUE_X, y, value);
}

//-----------------------------------------------------------------------------
// Beside the title, because the state of the switch decides whether anything
// on the screen below it is in effect at all
static void draw_state(void)
{
  const char *text = config.key_remapping_enabled ? "ON" : "OFF - not in effect";
  int x = KR_TEXT_R - (int)strlen(text) * FW_SMALL;

  lcd_set_font(FONT_SMALL);
  lcd_set_color(KR_BG, config.key_remapping_enabled ? KR_TITLE : KR_DIM);

  // The whole line height, not the font's: the long form of this text is
  // replaced by the short one in place, and an erase that stopped at the top
  // of the glyphs left the tails of the old words behind
  lcd_fill_rect(140, KR_TITLE_Y + 2, KR_TEXT_R - 140, 11, KR_BG);
  lcd_puts(x, KR_TITLE_Y + 4, text);
}

//-----------------------------------------------------------------------------
static void draw_footer(void)
{
  lcd_fill_rect(11, KR_FOOT_Y - 2, LCD_WIDTH - 22, 26, KR_BG);
  lcd_set_font(FONT_SMALL);

  if (g_kr.capture)
  {
    int key = row_key(g_kr.sel);
    char buf[48];

    snprintf(buf, sizeof(buf), "Press the key %s should act as",
        key_remap_name(key));

    lcd_fill_rect(11, KR_FOOT_Y - 2, LCD_WIDTH - 22, 13, KR_CAPTURE_BG);
    lcd_set_color(KR_CAPTURE_BG, KR_FG);
    lcd_puts(KR_X, KR_FOOT_Y, buf);
    lcd_set_color(KR_BG, KR_DIM);
    lcd_puts(KR_X, KR_FOOT2_Y, "Any other key cancels");
    return;
  }

  lcd_set_color(KR_BG, KR_DIM);
  lcd_puts(KR_X, KR_FOOT_Y, "U/D row  L/R set  MODE press a key");
  lcd_puts(KR_X, KR_FOOT2_Y, "MENU back        (off) = key does nothing");
}

//-----------------------------------------------------------------------------
static void key_remap_draw(void *ctx, bool full)
{
  (void)ctx;

  if (full)
  {
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, KR_BG);
    lcd_draw_rect(10, 10, LCD_WIDTH - 21, LCD_HEIGHT - 21, KR_FG);

    lcd_set_font(FONT_LARGE);
    lcd_set_color(KR_BG, KR_TITLE);
    lcd_puts(KR_X, KR_TITLE_Y, "Key Remap");

    lcd_fill_rect(14, KR_RULE_Y, LCD_WIDTH - 28, 1, KR_RULE);
  }

  draw_state();

  for (int i = 0; i < row_count(); i++)
    draw_row(i);

  // Hairlines in the gaps the rows leave, marking off the switch at the top
  // and the reset row at the bottom from the keys between them
  lcd_fill_rect(14, row_y(ROW_FIRST_KEY) - 3, LCD_WIDTH - 28, 1, KR_RULE);
  lcd_fill_rect(14, row_y(row_reset()) - 3, LCD_WIDTH - 28, 1, KR_RULE);

  draw_footer();
}

//-----------------------------------------------------------------------------
static void move_sel(int dir)
{
  int sel = g_kr.sel + dir;

  if (sel < 0 || sel >= row_count())
    return;

  g_kr.sel = sel;
  ui_request_redraw();
}

//-----------------------------------------------------------------------------
// Through the keys in the order the table lists them and then off, wrapping.
// Cycling is what makes a dead key repairable: a capture cannot ask for a
// press the hardware can no longer produce, and pointing a key that still
// works at the one that does not is the whole repair.
static void cycle_target(int key, int dir)
{
  int count = key_remap_count();
  int off = count; // one past the keys
  uint32_t target = key_remap_target(key);
  int value = (0 == target) ? off : key_remap_index_of(target);

  value += dir;
  value = ((value % (off + 1)) + (off + 1)) % (off + 1);

  key_remap_set(key, (off == value) ? 0 : key_remap_button(value));
  ui_request_redraw();
}

//-----------------------------------------------------------------------------
static void capture_set(bool enable)
{
  g_kr.capture = enable;
  input_capture_set(enable);
  ui_request_redraw();
}

//-----------------------------------------------------------------------------
// In capture the buttons arrive untranslated, so what is matched here is the
// key that was physically pressed. It has to be that one key and nothing
// else: a chord is not a key this screen can point a row at, and neither is
// a system key, so both land in the same place and cancel.
static bool capture_input(int buttons)
{
  int key = row_key(g_kr.sel);
  int index;

  if (0 == buttons || (buttons & BTN_REPEAT))
    return true; // the release of the key that opened the capture, or a repeat

  index = key_remap_index_of((uint32_t)buttons);

  if (index >= 0 && key >= 0)
    key_remap_set(key, key_remap_button(index));

  capture_set(false);

  return true;
}

//-----------------------------------------------------------------------------
static bool key_remap_input(void *ctx, int buttons)
{
  bool repeat = (buttons & BTN_REPEAT);
  int key;

  (void)ctx;

  if (g_kr.capture)
    return capture_input(buttons);

  if (0 == buttons)
    return true;

  key = row_key(g_kr.sel);

  if (buttons & (BTN_UP | BTN_DOWN))
  {
    move_sel((buttons & BTN_UP) ? -1 : 1);
    return true;
  }

  if (buttons & (BTN_LEFT | BTN_RIGHT))
  {
    if (repeat)
      return true;

    if (ROW_ENABLE == g_kr.sel)
    {
      config.key_remapping_enabled = !config.key_remapping_enabled;
      ui_request_redraw();
    }
    else if (key >= 0)
    {
      cycle_target(key, (buttons & BTN_RIGHT) ? 1 : -1);
    }

    return true;
  }

  if (buttons & BTN_MODE)
  {
    if (repeat)
      return true;

    if (ROW_ENABLE == g_kr.sel)
    {
      config.key_remapping_enabled = !config.key_remapping_enabled;
      ui_request_redraw();
    }
    else if (key >= 0)
    {
      capture_set(true);
    }
    else
    {
      key_remap_reset();
      ui_request_redraw();
    }

    return true;
  }

  if (buttons & BTN_MENU)
  {
    if (!repeat)
      ui_pop();

    return true;
  }

  // A remappable key pressed here would arrive already translated and mean
  // whatever the table now says. Swallowed, so it cannot reach the menus
  // underneath while its own row is on screen.
  return true;
}

//-----------------------------------------------------------------------------
static void key_remap_leave(void *ctx)
{
  (void)ctx;

  // The passthrough belongs to this screen and to nothing else
  input_capture_set(false);
  g_kr.capture = false;
}

static const ui_screen_t key_remap_screen =
{
  .leave  = key_remap_leave,
  .draw   = key_remap_draw,
  .input  = key_remap_input,
  .opaque = true,
};

//-----------------------------------------------------------------------------
void key_remap_open(void)
{
  g_kr.sel = ROW_ENABLE;
  g_kr.capture = false;

  ui_push(&key_remap_screen, NULL);
}
