/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DOOM, in a build that does not have it.
 *
 * The renderer is ~23 KB of code and its asset pack is 208 KB of the 384 KB the
 * firmware may use, so `make` leaves both out and `make DOOM=1` puts them in.
 * What is left in either case is this: the same application entry, the same
 * five entry points doom_port.c exports, and a screen that says what to build.
 *
 * The alternative was to drop the launcher row with an #ifdef, which reads on
 * the device as a feature that quietly disappeared. A kilobyte is worth not
 * having to explain that.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "utils.h"
#include "buttons.h"
#include "menu_widget.h"
#include "doom_port.h"

/*- Definitions -------------------------------------------------------------*/
#define STUB_FG             LCD_COLOR(200, 200, 200)
#define STUB_HI             LCD_COLOR(255, 190, 60)
#define STUB_TITLE          LCD_COLOR(220, 60, 40)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void draw_screen(void)
{
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);

  lcd_set_font(FONT_LARGE);
  lcd_set_color(LCD_BLACK_COLOR, STUB_TITLE);
  lcd_puts(84, 20, "NO DOOM HERE");

  lcd_set_font(FONT_SMALL);
  lcd_set_color(LCD_BLACK_COLOR, STUB_FG);
  lcd_puts(12, 58, "This firmware was built without DOOM.");
  lcd_puts(12, 76, "The asset pack is 208 KB of the 384 KB the");
  lcd_puts(12, 88, "firmware has, and the renderer another 23,");
  lcd_puts(12, 100, "so builds leave them out unless asked.");

  lcd_puts(12, 124, "Pack an IWAD and build with DOOM=1:");

  lcd_set_color(LCD_BLACK_COLOR, STUB_HI);
  lcd_puts(12, 144, "python3 tools/wadpack.py doom1.wad \\");
  lcd_puts(12, 156, "        -o doom/doom_assets.bin");
  lcd_puts(12, 168, "cd make && make DOOM=1");

  lcd_set_color(LCD_BLACK_COLOR, STUB_FG);
  lcd_puts(12, 188, "Any DOOM IWAD works, Freedoom included.");

  lcd_puts(12, 216, "SHIFT+MENU returns to the launcher.");

  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
void doom_port_init(void)
{
  draw_screen();
}

//-----------------------------------------------------------------------------
void doom_port_task(void)
{
  // Nothing moves on this screen, so there is nothing to do per frame
}

//-----------------------------------------------------------------------------
void doom_port_buttons_handler(int buttons)
{
  (void)buttons;
}

//-----------------------------------------------------------------------------
void doom_port_cleanup(void)
{
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
}

//-----------------------------------------------------------------------------
void doom_port_redraw(void)
{
  draw_screen();
}

/*- Variables ---------------------------------------------------------------*/
// The same text as the screen, so the answer is also where a user looks for it:
// the application's own section of the system menu
static const char *const g_help_lines[] =
{
  "The DOOM renderer and its asset pack are",
  "not in this build. The pack alone is 208 KB",
  "of the 384 KB the firmware may use, so it",
  "is opt-in.",
  "",
  "Build a pack from any DOOM IWAD (Freedoom",
  "included) and rebuild with DOOM=1:",
  "",
  "  python3 tools/wadpack.py doom1.wad \\",
  "          -o doom/doom_assets.bin",
  "  cd make && make DOOM=1",
  "",
  "Everything else in the firmware is the",
  "same either way.",
};

static const info_page_t g_help_page =
{
  .title = "DOOM is not in this build",
  .lines = g_help_lines,
  .count = ARRAY_SIZE(g_help_lines),
};

// The stub has nothing to configure: everything it has to say is the page
// below, and pages belong to the system menu's Help section
const menu_def_t doom_port_menu =
{
  .title = "DOOM",
  .items = NULL,
  .count = 0,
};

static const menu_item_t g_help_items[] =
{
  { .kind = MI_ACTION, .label = "How to build it in",
    .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t doom_port_help_menu =
{
  .title = "DOOM",
  .items = g_help_items,
  .count = ARRAY_SIZE(g_help_items),
};
