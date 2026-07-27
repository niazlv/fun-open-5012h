/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
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

static const char *const g_measure_panel_labels[] = { "On", "Off" };

// What a status-line slot shows. Indexed by the MEASURE_* metric numbers, so
// the order here is the order in config.h. "Off" hands the space back to the
// trigger readouts, which is what an empty slot means.
static const char *const g_measure_slot_labels[] =
{
  "Off", "Vpp", "Frequency", "Duty", "Vrms", "Vavg", "Type", "THD",
};

static const char *const g_decoder_proto_labels[] =
{
  "Auto", "UART", "1-Wire", "WS2812", "NEC IR", "Raw",
};

//-----------------------------------------------------------------------------
// Help pages
//-----------------------------------------------------------------------------
static const char *const g_scope_help_lines[] =
{
  "MODE        - Measurements on/off; which",
  "  ones and where: Menu > Measurements",
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

// The panel: a set of metrics in a small font, composited over the trace
static const menu_item_t g_panel_items[] =
{
  { .kind = MI_CHOICE, .label = "Panel",
    .u.choice = { &config.measure_panel_mode, g_measure_panel_labels, 2, NULL } },
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

// The status line: exactly two values in the large font, and the user says
// which. Set both to Off and the line goes back to the trigger edge, the
// trigger level and the horizontal position.
static const menu_item_t g_line_items[] =
{
  { .kind = MI_CHOICE, .label = "Left",
    .desc = "Large readout at x=140",
    .u.choice = { &config.measure_line[0], g_measure_slot_labels,
        MEASURE_COUNT, NULL } },
  { .kind = MI_CHOICE, .label = "Right",
    .desc = "Large readout at x=228",
    .u.choice = { &config.measure_line[1], g_measure_slot_labels,
        MEASURE_COUNT, NULL } },
};

// Measurements display: values live in config; the scope picks them up on
// its next tick, no callbacks needed
static const menu_item_t g_measure_items[] =
{
  { .kind = MI_TOGGLE, .label = "Show (MODE)",
    .u.toggle = { &config.measure_display, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_SUBMENU, .label = "Panel over the trace",
    .u.submenu = { g_panel_items, ARRAY_SIZE(g_panel_items) } },
  { .kind = MI_SUBMENU, .label = "Status line",
    .u.submenu = { g_line_items, ARRAY_SIZE(g_line_items) } },
};

static const menu_item_t g_decoder_items[] =
{
  { .kind = MI_CHOICE, .label = "Protocol",
    .u.choice = { &config.decoder_proto, g_decoder_proto_labels, 6, NULL } },
  { .kind = MI_TOGGLE, .label = "Stop on frames",
    .u.toggle = { &config.decoder_stop, NULL } },
};

// The four parameters the calibration screen edits, and the order the
// procedure wants them in. Z and D are the two that decide how quiet an
// unterminated input looks: Z centres the single ADC, D matches the second
// one to it. S and O are per vertical range and have to be redone on each.
static const char *const g_calib_help_lines[] =
{
  "TRIG_UP/DN   - change the value",
  "SHIFT+TRIG_x - next parameter",
  "SHIFT+L/R    - timebase, SHIFT+U/D - volts",
  "AUTO         - auto-calibrate Z, D and O",
  "MODE         - hide the hint (back next step)",
  "",
  "Z zero    - offset DAC centre",
  "D delta   - ADC B against ADC A",
  "S scale   - gain, PER volts/div range",
  "O offset  - DAC step, PER volts/div range",
  "",
  "SHORT the input for Z and D - an open",
  "1 MOhm input is an antenna, and the noise",
  "it picks up is what makes the numbers",
  "impossible to aim.",
  "",
  "Z: single channel. Mid screen is 0 V, which",
  "is a raw ADC reading of 0x80 = 128. Aim the",
  "avg there. D: dual channel, make A and B",
  "read the same - their mismatch IS the",
  "sample-to-sample noise on the trace.",
  "The readout turns RED when the parameter",
  "does not belong to the current mode.",
  "",
  "S: apply a known level, ideally with the",
  "raw data in 0xd0-0xf0, and adjust until",
  "min and max match it. S and O are per",
  "vertical range; Z and D are shared.",
};

static const info_page_t g_page_calib_help =
{
  .title = "Calibration",
  .lines = g_calib_help_lines,
  .count = ARRAY_SIZE(g_calib_help_lines),
};

//-----------------------------------------------------------------------------
// Numeric calibration entry.
//
// The scope screen edits these with the trigger keys, blind and one step at a
// time; a gain correction of a few percent is hundreds of presses there. Here
// the value is a number you can see and hold a key on. Scale and DAC step are
// per vertical range, and a const menu table cannot point into a live index,
// so they go through proxies that the opening action loads and the apply
// callbacks write back.
//-----------------------------------------------------------------------------
static int g_calib_scale_proxy;
static int g_calib_dac_proxy;

static const char *const g_calib_range_labels[] =
{
  "50 mV", "100 mV", "200 mV", "500 mV", "1 V", "2 V", "5 V", "10 V",
};

//-----------------------------------------------------------------------------
static void calib_load_proxies(void)
{
  g_calib_scale_proxy = config.calib_vs_mult[config.vertical_scale];
  g_calib_dac_proxy   = config.calib_dac_mult[config.vertical_scale];
}

//-----------------------------------------------------------------------------
static void calib_range_changed(void)
{
  scope_set_vertical_scale(config.vertical_scale);
  calib_load_proxies();
  ui_request_redraw(); // the two per-range rows now show different numbers
}

//-----------------------------------------------------------------------------
static void calib_zero_apply(int value)
{
  (void)value;
  scope_calib_apply(true); // the offset DAC moved
}

//-----------------------------------------------------------------------------
static void calib_delta_apply(int value)
{
  (void)value;
  scope_calib_apply(false); // software only, no acquisition restart
}

//-----------------------------------------------------------------------------
static void calib_scale_apply(int value)
{
  config.calib_vs_mult[config.vertical_scale] = value;
  scope_calib_apply(false);
}

//-----------------------------------------------------------------------------
static void calib_dac_apply(int value)
{
  config.calib_dac_mult[config.vertical_scale] = value;
  scope_calib_apply(true);
}

//-----------------------------------------------------------------------------
static void action_autocal(const void *arg)
{
  (void)arg;
  // It runs on the scope screen and paints its progress there, so the whole
  // popup chain has to go - not ui_pop_to_root(), which would leave the app
  menu_close_popups();
  scope_autocal_start();
}

static const menu_item_t g_calib_value_items[] =
{
  { .kind = MI_CHOICE, .label = "Range",
    .desc = "Scale and DAC step below are for THIS range",
    .u.choice = { &config.vertical_scale, g_calib_range_labels,
        ARRAY_SIZE(g_calib_range_labels), calib_range_changed } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_NUMBER, .label = "Z  zero",
    .desc = "Offset DAC centre, input open -> 0 V",
    .u.number = { &config.calib_dac_zero, 1900, 2200, 1, 10, NULL,
        calib_zero_apply } },
  { .kind = MI_NUMBER, .label = "D  channel delta",
    .desc = "ADC A against ADC B, input open",
    .u.number = { &config.calib_channel_delta, -64, 64, 1, 4, NULL,
        calib_delta_apply } },
  { .kind = MI_NUMBER, .label = "S  scale",
    .desc = "Gain: raise to read higher",
    .u.number = { &g_calib_scale_proxy, 1, 4000000, 1, 200, NULL,
        calib_scale_apply } },
  { .kind = MI_NUMBER, .label = "O  DAC step",
    .desc = "1 position pixel = 1 screen pixel",
    .u.number = { &g_calib_dac_proxy, 0, 100000, 1, 100, NULL,
        calib_dac_apply } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_NUMBER, .label = "Reference, mV",
    .desc = "Level the gain step asks you to apply",
    .u.number = { &config.calib_ref_mv, 50, 40000, 10, 250, "mV", NULL } },
};

static const menu_def_t g_calib_values =
{
  .title = "Calibration Values",
  .items = g_calib_value_items,
  .count = ARRAY_SIZE(g_calib_value_items),
};

//-----------------------------------------------------------------------------
static void action_calib_values(const void *arg)
{
  (void)arg;
  calib_load_proxies(); // a const table cannot follow the range on its own
  menu_open_dialog(&g_calib_values); // MENU or LEFT backs out
}

static const menu_item_t g_calib_items[] =
{
  { .kind = MI_ACTION, .label = "Auto-calibrate",
    .desc = "Disconnect the probe first",
    .u.action = { action_autocal, NULL } },
  { .kind = MI_ACTION, .label = "Enter values",
    .desc = "Type Z, D, S and O per range",
    .u.action = { action_calib_values, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_TOGGLE, .label = "Calibration mode",
    .u.toggle = { &scope_calibration_mode, scope_calibration_changed } },
  { .kind = MI_ACTION, .label = "How to calibrate",
    .u.action = { menu_action_info, &g_page_calib_help } },
};

static const menu_item_t g_scope_items[] =
{
  { .kind = MI_SUBMENU, .label = "Trigger Settings",
    .u.submenu = { g_trigger_items, ARRAY_SIZE(g_trigger_items) } },
  { .kind = MI_SUBMENU, .label = "Measurements",
    .u.submenu = { g_measure_items, ARRAY_SIZE(g_measure_items) } },
  { .kind = MI_SUBMENU, .label = "Decoder",
    .u.submenu = { g_decoder_items, ARRAY_SIZE(g_decoder_items) } },
  { .kind = MI_SUBMENU, .label = "Calibration",
    .u.submenu = { g_calib_items, ARRAY_SIZE(g_calib_items) } },
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
