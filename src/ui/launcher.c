/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Application launcher
 */

/*- Includes ----------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "buttons.h"
#include "utils.h"
#include "ui.h"
#include "menu_widget.h"
#include "launcher.h"
#include "scope.h"
#include "capture.h"
#include "cube3d.h"
#include "engine3d.h"
#include "raytrace_test.h"
#include "flappy_bird.h"
#include "snake_game.h"
#include "game2048.h"
#include "tetris.h"
#include "doom_port.h"
#include "bk_port.h"
#include "debug_coredump.h"
#include "flash_viewer.h"
#include "spi_tool.h"
#include "tcm_borrow.h"

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  const char *name;
  const char *desc;
  void (*init)(void);
  void (*task)(void);
  void (*buttons)(int buttons);
  void (*cleanup)(void);
  void (*redraw)(void);       // full repaint after an overlay closes
  const menu_def_t *menu;     // application section of the system menu
  const menu_def_t *help;     // its pages in the system menu's Help section

  // May the main loop sleep the core when a pass of this application finds
  // nothing to do (see launcher_app_may_idle)? It qualifies when it paints a
  // whole frame in one pass and then waits for its next tick: the games, the
  // cube, the two viewers.
  //
  // The incremental renderers do not. The oscilloscope paints one column of
  // the sweep per pass, the ray tracer one line, the 3D engine a slice of
  // lines, and DOOM renders flat out with no tick at all - for those a pass
  // IS the frame budget, and a millisecond of sleep between passes would
  // divide the frame rate by the number of passes a frame takes.
  //
  // Every row below answers this explicitly, and one left unset reads as
  // false, which is the safe way round for anything added here later.
  bool idle;
} app_desc_t;

/*- Variables ---------------------------------------------------------------*/
static const app_desc_t *g_running = NULL;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void scope_init_wrapper(void)
{
  // Back to the state of a boot before the instrument rebuilds itself: while
  // an application was on screen its memory was somebody else's, and
  // scope_init() is written against zeroed .bss, not against whatever a game
  // left there. See tcm_borrow.h.
  tcm_borrow_return();

  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
  scope_init(false); // Normal mode, not calibration
}

//-----------------------------------------------------------------------------
static void scope_cleanup_wrapper(void)
{
  capture_stop();

  // capture_stop() ends the acquisition, not the converter. The encode clocks
  // TIMER0 puts on PA8/PA9 keep the AD9288 sampling at 62.5 MHz a channel for
  // as long as the device is on afterwards - tens of milliamps spent digitising
  // a front end nobody is looking at, in a game or in this menu.
  //
  // Safe to stop here and only here: the acquisition above is what uses them,
  // the DMA it feeds is already stopped, and no other application touches
  // capture at all. scope_init() puts them back on its way in, through
  // update_sample_rate() -> capture_set_horizontal_parameters(), which is the
  // one sequence that establishes the phase between the encode edge and the
  // DMA's read of the data bus - so the clocks never restart any other way.
  capture_disable_clock();

  // The instrument's 28 KB of TCM are now an application's to use. Poisoned
  // rather than left alone, so that anything still reading instrument state
  // from here reads a pattern instead of numbers that look reasonable.
  tcm_borrow_poison();

  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
}

/*- Variables ---------------------------------------------------------------*/
// Every application supplies a redraw(): the system menu is a popup that only
// covers part of the screen, and the launcher repaints whatever it uncovered
// by calling redraw() on the application below it
static const app_desc_t g_apps[] =
{
  { "Oscilloscope", "Digital oscilloscope with signal analysis",
    scope_init_wrapper, scope_task, scope_buttons_handler, scope_cleanup_wrapper,
    scope_redraw_all, &scope_menu, &scope_help_menu, false },
  { "3D Graphics", "Rotating 3D cube demo",
    cube3d_init, cube3d_task, cube3d_buttons_handler, cube3d_cleanup,
    cube3d_redraw, &cube3d_menu, &cube3d_help_menu, true },
  { "3D Engine", "Advanced 3D engine with RayCast/RayTrace",
    engine3d_init, engine3d_task, engine3d_buttons_handler, engine3d_cleanup,
    engine3d_redraw, &engine3d_menu, &engine3d_help_menu, false },
  { "Ray Trace Test", "Simple ray tracing demo",
    raytrace_test_init, raytrace_test_task, raytrace_test_buttons_handler,
    raytrace_test_cleanup, raytrace_test_redraw, &raytrace_test_menu,
    &raytrace_test_help_menu, false },
  { "Flappy Bird", "Side-scrolling bird game",
    flappy_bird_init, flappy_bird_task, flappy_bird_buttons_handler,
    flappy_bird_cleanup, flappy_bird_redraw, &flappy_bird_menu,
    &flappy_bird_help_menu, true },
  { "Snake Game", "Classic snake game",
    snake_game_init, snake_game_task, snake_game_buttons_handler,
    snake_game_cleanup, snake_game_redraw, &snake_game_menu,
    &snake_game_help_menu, true },
  { "2048", "Slide and merge, on a board from 3x3 to 6x6",
    game2048_init, game2048_task, game2048_buttons_handler,
    game2048_cleanup, game2048_redraw, &game2048_menu, &game2048_help_menu,
    true },
  { "Tetris", "Ten by twenty, with next, hold and a ghost",
    tetris_init, tetris_task, tetris_buttons_handler,
    tetris_cleanup, tetris_redraw, &tetris_menu, &tetris_help_menu, true },
  { "DOOM", "id Software's renderer on a real WAD level",
    doom_port_init, doom_port_task, doom_port_buttons_handler,
    doom_port_cleanup, doom_port_redraw, &doom_port_menu, &doom_port_help_menu,
    false },
  { "BK-0010-01", "Electronika BK-0010-01, a PDP-11 from 1985",
    bk_port_init, bk_port_task, bk_port_buttons_handler,
    bk_port_cleanup, bk_port_redraw, &bk_port_menu, &bk_port_help_menu,
    false },
  { "CoreDump Viewer", "View crash dumps and stack traces",
    coredump_app_init, coredump_app_task, coredump_app_buttons_handler,
    coredump_app_cleanup, coredump_app_redraw, &coredump_menu,
    &coredump_help_menu, true },
  { "Flash Viewer", "Explore flash memory structure",
    flash_viewer_init, flash_viewer_task, flash_viewer_buttons_handler,
    flash_viewer_cleanup, flash_viewer_redraw, &flash_viewer_menu,
    &flash_viewer_help_menu, true },
  // Deliberately last, and deliberately an application: the SPI part is only
  // writable while this screen is open
  { "SPI Flash Loader", "Take data from a PC onto the 8 MB chip",
    spi_tool_init, spi_tool_task, spi_tool_buttons_handler,
    spi_tool_cleanup, spi_tool_redraw, NULL, NULL, true },
};

static menu_item_t g_app_items[ARRAY_SIZE(g_apps)];
static menu_def_t g_launcher_def;

//-----------------------------------------------------------------------------
// Application screen: one app runs at a time on top of the launcher
//-----------------------------------------------------------------------------
static void app_enter(void *ctx)
{
  const app_desc_t *app = (const app_desc_t *)ctx;

  g_running = app;

  if (app->init)
    app->init();
}

//-----------------------------------------------------------------------------
static void app_leave(void *ctx)
{
  const app_desc_t *app = (const app_desc_t *)ctx;

  g_running = NULL;

  if (app->cleanup)
    app->cleanup();
}

//-----------------------------------------------------------------------------
static void app_draw(void *ctx, bool full)
{
  const app_desc_t *app = (const app_desc_t *)ctx;

  // Called with full=true right after launch (init has already painted,
  // redraw is idempotent) and when an overlay above has closed
  if (full && app->redraw)
    app->redraw();
}

//-----------------------------------------------------------------------------
static void app_tick(void *ctx)
{
  const app_desc_t *app = (const app_desc_t *)ctx;

  if (app->task)
    app->task();
}

//-----------------------------------------------------------------------------
static bool app_input(void *ctx, int buttons)
{
  const app_desc_t *app = (const app_desc_t *)ctx;

  if (buttons & BTN_MENU)
  {
    // SHIFT+MENU leaves the application from anywhere; a plain MENU press is
    // left unconsumed so the caller opens the system menu over the app
    if ((buttons & BTN_SHIFT) && !(buttons & BTN_REPEAT))
    {
      launcher_exit_app();
      return true;
    }

    return false;
  }

  if (app->buttons)
    app->buttons(buttons);

  return true;
}

static const ui_screen_t app_screen =
{
  .enter  = app_enter,
  .leave  = app_leave,
  .draw   = app_draw,
  .tick   = app_tick,
  .input  = app_input,
  .opaque = true,
};

//-----------------------------------------------------------------------------
static void action_launch(const void *arg)
{
  ui_push(&app_screen, (void *)(uintptr_t)arg);
}

//-----------------------------------------------------------------------------
void launcher_start(void)
{
  for (int i = 0; i < ARRAY_SIZE(g_apps); i++)
  {
    g_app_items[i] = (menu_item_t)
    {
      .kind = MI_ACTION,
      .label = g_apps[i].name,
      .desc = g_apps[i].desc,
      .u.action = { action_launch, &g_apps[i] },
    };
  }

  g_launcher_def = (menu_def_t)
  {
    .title = "SELECT APPLICATION",
    .items = g_app_items,
    .count = ARRAY_SIZE(g_apps),
  };

  menu_open_fullscreen(&g_launcher_def);
}

//-----------------------------------------------------------------------------
// Open an application without going through the list, for the boot that
// follows a crash
bool launcher_start_app(const char *name)
{
  for (int i = 0; i < ARRAY_SIZE(g_apps); i++)
  {
    if (0 == strcmp(g_apps[i].name, name))
    {
      ui_push(&app_screen, (void *)&g_apps[i]);
      return true;
    }
  }

  return false;
}

//-----------------------------------------------------------------------------
void launcher_exit_app(void)
{
  ui_pop_to_root();
}

//-----------------------------------------------------------------------------
const menu_def_t *launcher_app_menu(void)
{
  return g_running ? g_running->menu : NULL;
}

//-----------------------------------------------------------------------------
const menu_def_t *launcher_app_help(void)
{
  return g_running ? g_running->help : NULL;
}

//-----------------------------------------------------------------------------
const char *launcher_app_name(void)
{
  return g_running ? g_running->name : NULL;
}

//-----------------------------------------------------------------------------
bool launcher_app_running(void)
{
  return NULL != g_running;
}

//-----------------------------------------------------------------------------
// May the main loop sleep the core when a pass finds nothing to do? The
// launcher's own menu may - it is drawn on input and does nothing in between -
// and an application may when it says so (app_desc_t::idle).
//
// It is the running APPLICATION that decides, not the top screen: g_running
// still names the oscilloscope while a menu is open over it, which is what
// keeps the sweep at full speed there as well.
bool launcher_app_may_idle(void)
{
  return NULL == g_running || g_running->idle;
}

//-----------------------------------------------------------------------------
// May the backlight dim while this application is on screen?
//
// A different question from the one above, and it has a different answer:
// that one is about who can afford to be woken a millisecond late, this one
// is about whether nobody having pressed a key means nobody is looking. For
// the oscilloscope it does not - a trace watched without a key being touched
// is the instrument doing its job, and dimming it mid-measurement would be a
// fault, not a feature. Everything else here is watched by someone who is
// also pressing something.
bool launcher_app_may_dim(void)
{
  return NULL == g_running || scope_task != g_running->task;
}
