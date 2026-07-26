/*
 * Application launcher
 */

/*- Includes ----------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
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
#include "doom_port.h"
#include "debug_coredump.h"
#include "flash_viewer.h"

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  const char *name;
  const char *desc;
  void (*init)(void);
  void (*task)(void);
  void (*buttons)(int buttons);
  void (*cleanup)(void);
  void (*redraw)(void); // full repaint after an overlay closes; NULL if task() repaints anyway
} app_desc_t;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void scope_init_wrapper(void)
{
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
  scope_init(false); // Normal mode, not calibration
}

//-----------------------------------------------------------------------------
static void scope_cleanup_wrapper(void)
{
  capture_stop();
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
}

/*- Variables ---------------------------------------------------------------*/
static const app_desc_t g_apps[] =
{
  { "Oscilloscope", "Digital oscilloscope with signal analysis",
    scope_init_wrapper, scope_task, scope_buttons_handler, scope_cleanup_wrapper,
    scope_redraw_all },
  { "3D Graphics", "Rotating 3D cube demo",
    cube3d_init, cube3d_task, cube3d_buttons_handler, cube3d_cleanup, NULL },
  { "3D Engine", "Advanced 3D engine with RayCast/RayTrace",
    engine3d_init, engine3d_task, engine3d_buttons_handler, engine3d_cleanup, NULL },
  { "Ray Trace Test", "Simple ray tracing demo",
    raytrace_test_init, raytrace_test_task, raytrace_test_buttons_handler,
    raytrace_test_cleanup, NULL },
  { "Flappy Bird", "Side-scrolling bird game",
    flappy_bird_init, flappy_bird_task, flappy_bird_buttons_handler,
    flappy_bird_cleanup, NULL },
  { "Snake Game", "Classic snake game",
    snake_game_init, snake_game_task, snake_game_buttons_handler,
    snake_game_cleanup, NULL },
  { "DOOM Port", "Classic DOOM FPS game",
    doom_port_init, doom_port_task, doom_port_buttons_handler,
    doom_port_cleanup, NULL },
  { "CoreDump Viewer", "View crash dumps and stack traces",
    coredump_app_init, coredump_app_task, coredump_app_buttons_handler,
    coredump_app_cleanup, NULL },
  { "Flash Viewer", "Explore flash memory structure",
    flash_viewer_init, flash_viewer_task, flash_viewer_buttons_handler,
    flash_viewer_cleanup, NULL },
};

static menu_item_t g_app_items[ARRAY_SIZE(g_apps)];
static menu_def_t g_launcher_def;

//-----------------------------------------------------------------------------
// Application screen: one app runs at a time on top of the launcher
//-----------------------------------------------------------------------------
static void app_enter(void *ctx)
{
  const app_desc_t *app = (const app_desc_t *)ctx;

  if (app->init)
    app->init();
}

//-----------------------------------------------------------------------------
static void app_leave(void *ctx)
{
  const app_desc_t *app = (const app_desc_t *)ctx;

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
    return false; // let the caller open the system menu

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
void launcher_exit_app(void)
{
  ui_pop_to_root();
}
