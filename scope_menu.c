/*
 * Oscilloscope application menu
 *
 * The scope's section of the system menu plus its help pages. Everything in
 * here is scope specific, which is exactly why it does not live in
 * system_menu.c: the system menu shows this table only while the scope is the
 * running application.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "config.h"
#include "common.h"
#include "utils.h"
#include "capture.h"
#include "ui.h"
#include "menu_widget.h"
#include "scope.h"

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Trigger settings: values live in config, hardware follows immediately.
// The scope screen underneath repaints in full when the menu closes.
//-----------------------------------------------------------------------------
static void trigger_mode_changed(void)
{
  capture_set_trigger_mode(config.trigger_mode);
  capture_start(); // arms SINGLE, resumes after a stopped capture
}

//-----------------------------------------------------------------------------
static void trigger_edge_changed(void)
{
  capture_set_trigger_edge(config.trigger_edge);
}

//-----------------------------------------------------------------------------
static void trigger_level_apply(int value)
{
  (void)value; // the widget already stored it in config.trigger_level
  scope_apply_trigger_level();
}

//-----------------------------------------------------------------------------
static void action_trigger_50p(const void *arg)
{
  (void)arg;
  scope_trigger_50_percent();
  ui_request_redraw(); // the Level row shows the new value
}

/*- Variables ---------------------------------------------------------------*/
static const char *const g_trigger_mode_labels[] = { "Auto", "Normal", "Single" };
static const char *const g_trigger_edge_labels[] = { "Rise", "Fall", "Both" };

static const char *const g_measure_view_labels[] = { "Panel", "Rotate" };

static const char *const g_decoder_proto_labels[] =
{
  "Auto", "UART", "1-Wire", "WS2812", "NEC IR", "Raw",
};

//-----------------------------------------------------------------------------
// Help pages
//-----------------------------------------------------------------------------
static const char *const g_scope_help_lines[] =
{
  "MODE        - Measurements on/off:",
  "  [M] Vpp+freq  [R] Vrms+duty  [S] type+THD",
  "SHIFT+MODE  - FFT spectrum view",
  "  see the Spectrum (FFT) page for details",
  "SHIFT+EDGE  - Decoder: UART/1-Wire/",
  "  WS2812/NEC/raw; TRIG_UP/DN jump bytes",
  "AUTO        - Auto-setup (scale/trigger)",
  "50%         - Trigger level to mid-signal",
  "STOP        - Freeze: pan/zoom/measure/",
  "  decode/spectrum on the frozen record",
  "TRIG / EDGE - Trigger mode / edge",
  "TRIG_UP/DN  - Trigger level",
  "MENU        - System menu, SHIFT+MENU exits",
};

static const info_page_t g_page_scope_help =
{
  .title = "Scope Functions",
  .lines = g_scope_help_lines,
  .count = ARRAY_SIZE(g_scope_help_lines),
};

// The spectrum view has its own key map and its own trap (resolution comes
// from the timebase, not from the sample rate), so it gets its own page
static const char *const g_spectrum_help_lines[] =
{
  "SHIFT+MODE - enter/leave   MODE - panel",
  "LEFT/RIGHT - cursor   UP/DOWN - peaks",
  "TRIG_UP/DN - wider span / finer df",
  "SHIFT+L/R - timebase  SHIFT+U/D - volts",
  "",
  "The whole record is transformed, so the",
  "resolution df = 1/record time and the",
  "TIMEBASE sets it, not the sample rate.",
  "df is on the panel: mains 50/100 Hz",
  "needs 5 ms/div or slower to separate.",
  "",
  "F0 comes from the harmonic comb, not",
  "from the tallest peak. h1..hN mark its",
  "harmonics, '-' marks peaks that belong",
  "to something else: interference, noise.",
};

static const info_page_t g_page_spectrum_help =
{
  .title = "Spectrum (FFT)",
  .lines = g_spectrum_help_lines,
  .count = ARRAY_SIZE(g_spectrum_help_lines),
};

//-----------------------------------------------------------------------------
// Menu tables (const, in flash). config changes are persisted automatically
// by config_task once the struct CRC goes stale.
//-----------------------------------------------------------------------------
static const menu_item_t g_trigger_items[] =
{
  { .kind = MI_CHOICE, .label = "Mode",
    .u.choice = { &config.trigger_mode, g_trigger_mode_labels, 3, trigger_mode_changed } },
  { .kind = MI_CHOICE, .label = "Edge",
    .u.choice = { &config.trigger_edge, g_trigger_edge_labels, 3, trigger_edge_changed } },
  { .kind = MI_NUMBER, .label = "Level",
    .u.number = { &config.trigger_level, -100, 100, 1, 5, "px", trigger_level_apply } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "Level to 50%",
    .u.action = { action_trigger_50p, NULL } },
};

// Measurements display: values live in config; the scope picks them up on
// its next tick, no callbacks needed
static const menu_item_t g_measure_items[] =
{
  { .kind = MI_TOGGLE, .label = "Show (MODE)",
    .u.toggle = { &config.measure_display, NULL } },
  { .kind = MI_CHOICE, .label = "View",
    .u.choice = { &config.measure_panel_mode, g_measure_view_labels, 2, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_TOGGLE, .label = "Vpp",
    .u.toggle = { &config.show_vpp, NULL } },
  { .kind = MI_TOGGLE, .label = "Frequency",
    .u.toggle = { &config.show_freq, NULL } },
  { .kind = MI_TOGGLE, .label = "Duty cycle",
    .u.toggle = { &config.show_duty, NULL } },
  { .kind = MI_TOGGLE, .label = "Vrms",
    .u.toggle = { &config.show_vrms, NULL } },
  { .kind = MI_TOGGLE, .label = "Vavg",
    .u.toggle = { &config.show_vavg, NULL } },
  { .kind = MI_TOGGLE, .label = "Signal type",
    .u.toggle = { &config.show_type, NULL } },
  { .kind = MI_TOGGLE, .label = "THD",
    .u.toggle = { &config.show_thd, NULL } },
};

static const menu_item_t g_decoder_items[] =
{
  { .kind = MI_CHOICE, .label = "Protocol",
    .u.choice = { &config.decoder_proto, g_decoder_proto_labels, 6, NULL } },
  { .kind = MI_TOGGLE, .label = "Stop on frames",
    .u.toggle = { &config.decoder_stop, NULL } },
};

static const menu_item_t g_scope_items[] =
{
  { .kind = MI_SUBMENU, .label = "Trigger Settings",
    .u.submenu = { g_trigger_items, ARRAY_SIZE(g_trigger_items) } },
  { .kind = MI_SUBMENU, .label = "Measurements",
    .u.submenu = { g_measure_items, ARRAY_SIZE(g_measure_items) } },
  { .kind = MI_SUBMENU, .label = "Decoder",
    .u.submenu = { g_decoder_items, ARRAY_SIZE(g_decoder_items) } },
  { .kind = MI_ACTION, .label = "Scope Functions",
    .u.action = { menu_action_info, &g_page_scope_help } },
  { .kind = MI_ACTION, .label = "Spectrum (FFT)",
    .u.action = { menu_action_info, &g_page_spectrum_help } },
};

const menu_def_t scope_menu =
{
  .title = "Oscilloscope",
  .items = g_scope_items,
  .count = ARRAY_SIZE(g_scope_items),
};
