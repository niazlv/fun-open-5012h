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

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gd32f4xx.h"
#include "hal_gpio.h"
#include "utils.h"
#include "lcd.h"
#include "timer.h"
#include "images.h"
#include "common.h"
#include "config.h"
#include "buttons.h"
#include "battery.h"
#include "capture.h"
#include "fft.h"
#include "classify.h"
#include "logic_decode.h"
#include "scope.h"

/*- Definitions -------------------------------------------------------------*/
#define ZERO_POINT             0x80

#define GRID_CENTER_X          160
#define GRID_CENTER_Y          120
#define GRID_LEFT              10
#define GRID_RIGHT             310
#define GRID_TOP               20
#define GRID_BOTTOM            220
#define GRID_WIDTH             300
#define GRID_HEIGHT            200
#define GRID_DIV_PX            25
#define GRID_DIVS_H            12
#define GRID_DIVS_V            10

#define STATUS_LINE_Y          223
#define STATUS_LINE_HEIGHT     16

#define MINIVIEW_WIDTH         160

#define CALIB_AREA_LEFT        140
#define CALIB_AREA_WIDTH       (LCD_WIDTH - CALIB_AREA_LEFT)

#define MAX_SAMPLE_RATE_LIMIT  13

#define TOAST_TIMEOUT          1500
#define TOAST_COLOR            LCD_COLOR(255, 255, 0)

#define WAIT_STATE_HOLDOFF     100

#define BG_COLOR               LCD_COLOR(0, 0, 0)
#define TRACE_COLOR            LCD_COLOR(255, 255, 0)
#define TRACE_FILLED_COLOR     LCD_COLOR(0, 255, 0)
#define TRACE_CLIP_COLOR       LCD_COLOR(255, 0, 0)
#define TRACE_INVALID_COLOR    LCD_COLOR(255, 0, 0)
#define GRID_BG_COLOR          LCD_COLOR(0, 0, 0)
#define GRID_FG_COLOR          LCD_COLOR(200, 200, 200)
#define MV_FRAME_COLOR         LCD_COLOR(230, 230, 230)

#define HSCALE_COLOR           LCD_COLOR(255, 255, 255)
#define HPOS_COLOR             LCD_COLOR(255, 180, 50)
#define VSCALE_COLOR           LCD_COLOR(255, 255, 0)

#define TRIGGER_LEVEL_COLOR    LCD_COLOR(50, 255, 255)
#define TRIGGER_MODE_COLOR     LCD_COLOR(0, 255, 0)

#define MEASURE_MODE_COLOR     LCD_COLOR(50, 255, 255)
#define MEASURE_VOLTAGE_COLOR  LCD_COLOR(255, 255, 0)
#define MEASURE_FREQ_COLOR     LCD_COLOR(255, 255, 255)

#define CAPTURE_STOP_COLOR     LCD_COLOR(255, 0, 0)
#define CAPTURE_WAIT_COLOR     LCD_COLOR(255, 180, 50)
#define CAPTURE_TRIG_COLOR     LCD_COLOR(0, 255, 0)

#define SR_LIMIT_COLOR         LCD_COLOR(250, 50, 50)
#define SR_COLOR               LCD_COLOR(0, 230, 0)

#define MIN_TRIGGER_LEVEL      -100 // px
#define MAX_TRIGGER_LEVEL       100 // px

#define MIN_HORIZONTAL_POSITION -2000000000 // 2 seconds
#define MAX_HORIZONTAL_POSITION  2000000000 // 2 seconds

#define MIN_VERTICAL_POSITION  (-10 * GRID_DIV_PX)
#define MAX_VERTICAL_POSITION  ( 10 * GRID_DIV_PX)

#define MEASURE_UPDATE_TIMEOUT 100
#define MEASURE_PAGE_TICKS     20    // measure page flips every 2 s
#define MEASURE_PAGES          3     // [M] Vpp+freq, [R] Vrms+duty, [S] type+THD

// The decode panel is a hole in the trace area: columns it covers only
// paint their part BELOW it, so neither grid nor trace ever touches the
// panel pixels and it needs no repainting while it is on screen.
// Aligned to the column grid: col N sits at screen x = GRID_LEFT+1+N.
#define DECODE_PANEL_COL0      2
#define DECODE_PANEL_W         180
#define DECODE_PANEL_COL1      (DECODE_PANEL_COL0 + DECODE_PANEL_W - 1)
#define DECODE_PANEL_X         (GRID_LEFT + 1 + DECODE_PANEL_COL0)
#define DECODE_PANEL_Y         (GRID_TOP + 1)
#define DECODE_PANEL_H         52

// Translucent measurements panel: composited INTO the trace columns (trace
// dimmed to 50%, text overlaid), sitting just above the status line
#define MPANEL_H               26
#define MPANEL_ROW0            (GRID_HEIGHT - 1 - MPANEL_H)
#define MPANEL_TEXT_MAX        50    // chars per line at 6 px/char
#define MPANEL_HEARTBEAT       4     // forced repaints: every 4th update (~2 s)
#define MPANEL_DIM(p)          (((p) >> 1) & 0x7BEF) // RGB565 half brightness

#define FFT_DB_RANGE           60.0f // spectrum display dynamic range

// Spectrum rendering: a continuous curve over a heat-gradient fill (the
// classic analyzer look — quiet bins glow blue-green, strong peaks run
// through yellow into red). Detected peaks carry a floating tick in their
// class colour, the cursor is a hairline standing on the curve.
#define FFT_LINE_COLOR         LCD_COLOR(255, 255, 210)
#define FFT_PEAK_H_COLOR       LCD_COLOR(255, 255, 255) // harmonic of F0
#define FFT_PEAK_X_COLOR       LCD_COLOR(255, 128, 32)  // foreign peak
#define FFT_CURSOR_COLOR       LCD_COLOR(0, 190, 190)

// Spectrum analysis panel: same "hole in the sweep" trick as the decode
// panel, parked on the right where the high-frequency end of a spectrum is
// almost always empty. Eight rows of the small font plus margins.
#define FFT_PANEL_W            168   // 26 chars of the small font + margins
#define FFT_PANEL_COL0         (GRID_WIDTH - 2 - FFT_PANEL_W)
#define FFT_PANEL_COL1         (FFT_PANEL_COL0 + FFT_PANEL_W - 1)
#define FFT_PANEL_X            (GRID_LEFT + 1 + FFT_PANEL_COL0)
#define FFT_PANEL_Y            (GRID_TOP + 1)
#define FFT_PANEL_H            80
#define FFT_PANEL_ROWS         5     // peak rows

#define FFT_UPDATE_TIMEOUT     100   // recompute the spectrum at most at 10 Hz
#define FFT_AUTO_BAND_MARGIN   6     // keep the measured frequency this far
                                     // below the analysis Nyquist

#define AUTOSET_TIMEOUT        8000  // give up on auto-setup after 8 s
#define AUTOSET_SURVEY_HS      HS_5_ms   // ~100 ms record: >= 5 periods of 50 Hz
#define AUTOSET_FAST_HS        HS_10_us  // maximum sample rate, ~800 us record
#define AUTOSET_SLOW_HS        HS_50_ms  // ~800 ms record: down to a few Hz
#define AUTOSET_FAST_MIN_HZ    50000 // below this the survey record knows better
#define AUTOSET_TARGET_PX      (6 * GRID_DIV_PX) // fill 6 of the 8 divisions
#define AUTOSET_VIEW_PERIODS   25    // periods across the screen, x10
#define AUTOSET_CENTER_TOL     8     // counts of DC offset we accept as centered
#define AUTOSET_VERT_TRIES     10    // vertical actions before we settle
#define AUTOSET_REFINE_TRIES   4
#define AUTOSET_FREQ_TOL_PCT   30    // estimate vs re-measurement agreement
#define AUTOSET_MIN_PERIODS    3     // periods a believable frequency needs
#define AUTOSET_MIN_GOOD_PCT   60    // ... of that regularity
#define AUTOSET_MIN_LEVEL_PCT  65    // ... or, when irregular, this digital
#define AUTOSET_MIN_SIGNAL_MV  100   // less than this at 50 mV/div is noise
#define AUTOSET_NO_ERR         0x7fffffff

enum
{
  CALIB_ZERO,
  CALIB_DELTA,
  CALIB_SCALE,
  CALIB_OFFSET,
};

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  uint8_t  min[GRID_WIDTH];
  uint8_t  max[GRID_WIDTH];
  uint8_t  flags[GRID_WIDTH];
} DisplayBuffer;

/*- Constants ---------------------------------------------------------------*/
static const char *hs_str[HS_COUNT] =
{
                            " 50\x01ns", // ns
  "100\x01ns", "200\x01ns", "500\x01ns",
  "  1\x01us", "  2\x01us", "  5\x01us", // us
  " 10\x01us", " 20\x01us", " 50\x01us",
  "100\x01us", "200\x01us", "500\x01us",
  "  1\x01ms", "  2\x01ms", "  5\x01ms", // ms
  " 10\x01ms", " 20\x01ms", " 50\x01ms",
  "100\x01ms", "200\x01ms", "500\x01ms",
};

static const int hs_div_value[HS_COUNT] =
{
  50, 100, 200, 500, // ns
  1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, // us
  1000000, 2000000, 5000000, 10000000, 20000000, 50000000, 100000000, 200000000, 500000000, // ms
};

static const int hs_px_value[HS_COUNT] = // in ns
{
  2, 4, 8, 20, // ns
  40, 80, 200, 400, 800, 2000, 4000, 8000, 20000, // us
  40000, 80000, 200000, 400000, 800000, 2000000, 4000000, 8000000, 20000000, // ms
};

static const char *vs_str[VS_COUNT] =
{
  " 50\x01mV", "100\x01mV", "200\x01mV", "500\x01mV", "  1\x01V ", "  2\x01V ", "  5\x01V ", " 10\x01V ",
};

static const int vs_px_value[VS_COUNT] =
{
  2, 4, 8, 20, 40, 80, 200, 400,
};

/*- Variables ---------------------------------------------------------------*/
// Five distinct grid column patterns; g_grid_index picks one per column
// (a byte index table instead of 300 pointers saves ~1.3 KB of scarce TCM)
static uint16_t g_grid_column_0[GRID_HEIGHT];
static uint16_t g_grid_column_1[GRID_HEIGHT];
static uint16_t g_grid_column_2[GRID_HEIGHT];
static uint16_t g_grid_column_3[GRID_HEIGHT];
static uint16_t g_grid_column_4[GRID_HEIGHT];

static uint16_t *const g_grid_columns[5] =
{
  g_grid_column_0, g_grid_column_1, g_grid_column_2,
  g_grid_column_3, g_grid_column_4,
};

static uint8_t g_grid_index[GRID_WIDTH];

static DataBuffer g_data_buffer;
static DisplayBuffer g_display_buffer;

static int g_trace_column = (GRID_WIDTH-1);

// Shadow of what each trace column currently shows on the LCD. Columns whose
// data and overlay did not change since the last sweep are skipped entirely,
// so a static trace costs ~300 compares instead of ~11 ms of LCD writes.
static uint8_t g_shadow_min[GRID_WIDTH];
static uint8_t g_shadow_max[GRID_WIDTH];
static uint8_t g_shadow_flags[GRID_WIDTH];
static bool g_shadow_valid = false;
static int g_shadow_marker_px = 0x7fffffff;
static bool g_sweep_force = true;

typedef struct
{
  int  x, y, w, h;
  bool valid;
} MarkerRect;

static MarkerRect g_vpos_marker = { 0, 0, 0, 0, false };
static MarkerRect g_trig_marker = { 0, 0, 0, 0, false };

static bool g_toast_active = false;
static int g_toast_timer = TIMER_DISABLE;

static int g_state = -1;
static int g_state_timer = TIMER_DISABLE;

static bool g_calibration_mode = false;
static bool g_calibration_dual_channel = false;
static int g_calibration_parameter = CALIB_ZERO;

static int g_measure_timer = TIMER_DISABLE;
static int g_measure_page = 0;
static int g_measure_page_tick = 0;

static bool g_fft_mode = false;
static int g_fft_peak_hz = 0;
static uint8_t g_fft_samples[FFT_SIZE];
static float g_fft_mag[FFT_BINS];

// Spectrum view state. The analysis always spans the whole record decimated
// by `max >> band`: band 0 is the full record (best resolution, narrowest
// span), every step up doubles the span and halves the resolution.
static FftAnalysis g_fft_an;
static int g_fft_band = 0;
static bool g_fft_band_auto = true;
static int64_t g_fft_auto_span = 0; // record time the auto band was picked for
static int g_fft_cursor = -1;       // trace column, -1 when the cursor is off
static int g_fft_cursor_bin = 0;
static bool g_fft_panel_pending = false;
static bool g_fft_panel_on = true;  // MODE toggles the breakdown panel
static uint16_t g_fft_grad[GRID_HEIGHT]; // heat ramp, indexed by curve height
static bool g_fft_grad_ready = false;
static int g_fft_timer = TIMER_DISABLE;
static bool g_fft_have = false;

static bool g_autoset_active = false;
static int g_autoset_timer = TIMER_DISABLE;

// Auto-setup phases, in the order autoset_step() walks them
enum
{
  AS_SURVEY,  // park on the long record every slow signal is visible in
  AS_VERT,    // converge the vertical there (amplitude survives undersampling)
  AS_FAST,    // ask the maximum sample rate whether something faster exists
  AS_SLOW,    // one last look through an eight times longer record
  AS_REFINE,  // re-measure on the timebase the estimate asked for
};

// Auto-setup run state, reset in autoset_start()
static int g_autoset_phase = AS_SURVEY;
static int g_autoset_saved_hs = 0;      // settings to restore if nothing is found
static int g_autoset_saved_vs = 0;
static int g_autoset_saved_vpos = 0;
static int g_autoset_zoom_floor = 0;    // no zoom-in below this scale (anti ping-pong)
static int g_autoset_prev_scale = -1;   // scale the last zoom step left behind
static int g_autoset_cent_dir = 1;      // DC-centering direction, flips on divergence
static int g_autoset_cent_prev = AUTOSET_NO_ERR;
static int g_autoset_vert_tries = 0;
static int g_autoset_refine_tries = 0;
static int g_autoset_freq = 0;          // current estimate
static int g_autoset_vamp_mv = 0;       // last measured swing

// Held-key coalescing: while a pan key auto-repeats, the DMA restart and
// miniview/sample-rate redraw run only every Nth tick; this timer fires
// once after the key is released to settle the final state
static int g_pan_settle_timer = TIMER_DISABLE;
static int g_pan_repeat_count = 0;

// Displayed-frames-per-second, surfaced in System Info for diagnosing
// "the scope feels slow" reports with a number
static int g_fps_counter = 0;
static int g_fps_value = 0;
static int g_fps_timer = TIMER_DISABLE;

static SignalClass g_signal_class = { SIG_UNKNOWN, -1 };

// Measurements panel state: 1bpp text mask, one bit per trace-area pixel
static bool g_mpanel_active = false;
static bool g_mpanel_force = false;   // rebuild the panel text on the next tick
static uint8_t g_mpanel_mask[MPANEL_H][(GRID_WIDTH + 7) / 8];
static char g_mpanel_line[2][MPANEL_TEXT_MAX + 2];

// Logic decoder view; the run tables live in the spare main-SRAM block
static bool g_decode_mode = false;
static bool g_decode_panel_pending = false;
static bool g_decode_force = false;   // run the next decode past the throttle
static bool g_logic_have = false;
static LogicResult g_logic;
static int g_decode_sel = 0;              // selected byte (jump target)
static int g_decode_period_ns = 0;        // record metrics the decode ran on
static int g_decode_trig_pos = 0;
static int g_decode_size = 0;             // record length the decode ran on

// Last miniview geometry so it can repaint without a full update_sample_rate
static int g_mv_trigger_px = 0;
static int g_mv_window_px = -1;
static int g_mv_width_px = 3;

_Static_assert(sizeof(LogicScratch) <= CAPTURE_SPARE_RAM_SIZE,
    "decoder scratch must fit the spare SRAM block");

// Decoded bytes are marked in the miniview (the whole-record map at the top
// of the screen): alternating tints per byte, bright for the selected one
#define DSTRIP_EVEN    LCD_COLOR(0, 150, 200)
#define DSTRIP_ODD     LCD_COLOR(0, 90, 130)
#define DSTRIP_SEL     LCD_COLOR(255, 255, 0)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void grid_init(void)
{
  // Setup column data
  for (int y = 0; y < 200; y++)
    g_grid_column_0[y] = GRID_BG_COLOR;

  for (int y = 0; y < 200; y++)
    g_grid_column_1[y] = (4 == (y % 5)) ? GRID_FG_COLOR : GRID_BG_COLOR;

  for (int y = 0; y < 200; y++)
    g_grid_column_2[y] = (24 == (y % 25)) ? GRID_FG_COLOR : GRID_BG_COLOR;
  g_grid_column_2[0]   = GRID_FG_COLOR;
  g_grid_column_2[1]   = GRID_FG_COLOR;
  g_grid_column_2[98]  = GRID_FG_COLOR;
  g_grid_column_2[100] = GRID_FG_COLOR;
  g_grid_column_2[197] = GRID_FG_COLOR;
  g_grid_column_2[198] = GRID_FG_COLOR;

  for (int y = 0; y < 200; y++)
    g_grid_column_3[y] = (4 == (y % 5)) ? GRID_FG_COLOR : GRID_BG_COLOR;
  g_grid_column_3[0]   = GRID_FG_COLOR;
  g_grid_column_3[1]   = GRID_FG_COLOR;
  g_grid_column_3[2]   = GRID_FG_COLOR;
  g_grid_column_3[98]  = GRID_FG_COLOR;
  g_grid_column_3[100] = GRID_FG_COLOR;
  g_grid_column_3[196] = GRID_FG_COLOR;
  g_grid_column_3[197] = GRID_FG_COLOR;
  g_grid_column_3[198] = GRID_FG_COLOR;

  for (int y = 0; y < 200; y++)
    g_grid_column_4[y] = (24 == (y % 25)) ? GRID_FG_COLOR : GRID_BG_COLOR;

  // Assign columns
  for (int x = 0; x < 299; x++)
    g_grid_index[x] = 0;

  for (int x = 4; x < 299; x += 5)
    g_grid_index[x] = 2;

  for (int x = 24; x < 299; x += 25)
    g_grid_index[x] = 3;

  g_grid_index[0]   = 1;
  g_grid_index[1]   = 1;
  g_grid_index[2]   = 4;
  g_grid_index[148] = 1;
  g_grid_index[150] = 1;
  g_grid_index[296] = 4;
  g_grid_index[297] = 1;
  g_grid_index[298] = 1;
}

//-----------------------------------------------------------------------------
static void toast_show(void)
{
  if (!g_toast_active)
    lcd_fill_rect(GRID_LEFT, GRID_BOTTOM+1, GRID_WIDTH+1, STATUS_LINE_HEIGHT, BG_COLOR);

  lcd_set_color(BG_COLOR, TOAST_COLOR);
  g_toast_active = true;
  g_toast_timer = TOAST_TIMEOUT;
}

//-----------------------------------------------------------------------------
static bool trace_ready(void)
{
  return (g_trace_column == (GRID_WIDTH-1));
}

//-----------------------------------------------------------------------------
static void redraw_trace(void)
{
  if (trace_ready())
  {
    g_trace_column = 0;
    // The trigger-position overlay is composited into the columns, so a
    // moved marker invalidates the whole sweep, not just changed data
    g_sweep_force = !g_shadow_valid || (config.horizontal_position_px != g_shadow_marker_px);
    g_shadow_marker_px = config.horizontal_position_px;
  }
}

//-----------------------------------------------------------------------------
static void update_column_from_image(int index, uint16_t *column, int x, int y, const Image *image)
{
  int image_col = index - x + image->ox + 1;

  if (image_col < 0 || image_col > (image->width-1))
    return;

  for (int i = 0; i < image->height; i++)
  {
    int color = image->data[i * image->width + image_col];

    if (color)
      column[y + i] = color;
  }
}

//-----------------------------------------------------------------------------
static void update_from_display_buffer(uint16_t *column, DisplayBuffer *db)
{
  bool clip_h = db->flags[g_trace_column] & SAMPLE_FLAG_CLIP_H;
  bool clip_l = db->flags[g_trace_column] & SAMPLE_FLAG_CLIP_L;
  int color;

  if (db->flags[g_trace_column] & SAMPLE_FLAG_VALID)
  {
    if (clip_h || clip_l)
      color = TRACE_CLIP_COLOR;
    else if (db->flags[g_trace_column] & SAMPLE_FLAG_FILLED)
      color = TRACE_FILLED_COLOR;
    else
      color =  TRACE_COLOR;

    for (int y = db->min[g_trace_column]; y <= db->max[g_trace_column]; y++)
      column[y] = color;
  }
  else
  {
    column[GRID_HEIGHT/2-1] = TRACE_INVALID_COLOR;
  }

  if (clip_h)
  {
    for (int i = 0; i < 3; i++)
      column[i] = TRACE_CLIP_COLOR;
  }

  if (clip_l)
  {
    for (int i = 0; i < 3; i++)
      column[GRID_HEIGHT-2 - i] = TRACE_CLIP_COLOR;
  }
}

//-----------------------------------------------------------------------------
// Heat ramp for the area under the spectrum curve, indexed by height above
// the baseline: deep blue at the floor, through teal and yellow, red at the
// top of the scale. Colour then encodes level on its own — a strong peak
// grows a red core, a noise bump never leaves the blues.
static void fft_build_gradient(void)
{
  static const uint8_t stop[4][3] =
  {
    { 0, 30, 90 }, { 0, 150, 130 }, { 210, 200, 0 }, { 255, 60, 0 },
  };

  for (int i = 0; i < GRID_HEIGHT; i++)
  {
    float t = (float)i * 3.0f / (GRID_HEIGHT - 2);
    int seg = (t >= 2.0f) ? 2 : (int)t;
    float frac = t - seg;
    int r = stop[seg][0] + (int)((stop[seg+1][0] - stop[seg][0]) * frac);
    int g = stop[seg][1] + (int)((stop[seg+1][1] - stop[seg][1]) * frac);
    int b = stop[seg][2] + (int)((stop[seg+1][2] - stop[seg][2]) * frac);

    g_fft_grad[i] = LCD_COLOR(r, g, b);
  }

  g_fft_grad_ready = true;
}

//-----------------------------------------------------------------------------
// Spectrum column: connected curve (min..max span, like the waveform trace)
// over the gradient fill. Peaks and the cursor ride the flags bits (set in
// fft_update), so the dirty-column compare repaints exactly the columns
// whose marker state changed.
static void update_from_spectrum(uint16_t *column, DisplayBuffer *db)
{
  int c = g_trace_column;
  int flags = db->flags[c];
  int top = db->min[c];
  int bottom = db->max[c];

  // Cursor: a hairline dropping from the top of the grid onto the curve
  if (flags & SAMPLE_FLAG_FILLED)
  {
    for (int y = 0; y < top; y++)
      column[y] = FFT_CURSOR_COLOR;
  }

  // Heat-gradient fill under the curve
  for (int y = bottom + 1; y <= GRID_HEIGHT-2; y++)
    column[y] = g_fft_grad[GRID_HEIGHT-2 - y];

  // The curve itself, one extra pixel below the span for weight
  for (int y = top; y <= bottom + 1 && y <= GRID_HEIGHT-2; y++)
    column[y] = FFT_LINE_COLOR;

  // Peak marker: a tick floating above the curve
  if (flags & (SAMPLE_FLAG_CLIP_H | SAMPLE_FLAG_CLIP_L))
  {
    int color = (flags & SAMPLE_FLAG_CLIP_H) ? FFT_PEAK_H_COLOR : FFT_PEAK_X_COLOR;

    for (int y = top - 6; y <= top - 3; y++)
    {
      if (y >= 0)
        column[y] = color;
    }
  }
}

//-----------------------------------------------------------------------------
// Compose the full pixel column c: grid, trace, overlays, translucent
// measurements panel and the decoder byte strip
static void build_trace_column(int c, uint16_t *column)
{
  for (int i = 0; i < GRID_HEIGHT; i++)
    column[i] = g_grid_columns[g_grid_index[c]][i];

  if (g_fft_mode)
    update_from_spectrum(column, &g_display_buffer);
  else
    update_from_display_buffer(column, &g_display_buffer);

  // The trigger-position overlay is meaningless on the spectrum view
  if (!g_fft_mode)
  {
    if (config.horizontal_position_px < -(GRID_WIDTH/2-1))
    {
      update_column_from_image(c, column, GRID_WIDTH-1, 1, &image_trigger_offset_right);
    }
    else if (config.horizontal_position_px > (GRID_WIDTH/2-1))
    {
      update_column_from_image(c, column, 1, 1, &image_trigger_offset_left);
    }
    else
    {
      int pos = GRID_WIDTH/2 - config.horizontal_position_px;
      update_column_from_image(c, column, pos, 0, &image_trigger_offset);
    }
  }

  // Translucent measurements panel: dim the trace pixels and overlay the
  // text mask, all inside the column buffer, so the panel is part of the
  // normal sweep and can never flicker or be painted over
  if (g_mpanel_active)
  {
    for (int y = 0; y < MPANEL_H; y++)
    {
      int row = MPANEL_ROW0 + y;

      column[row] = MPANEL_DIM(column[row]);

      if (g_mpanel_mask[y][c / 8] & (1 << (c % 8)))
        column[row] = LCD_WHITE_COLOR;
    }
  }

}

//-----------------------------------------------------------------------------
static void draw_trace(void)
{
  uint16_t column[GRID_HEIGHT];
  int c = g_trace_column;

  if (trace_ready())
    return;

  if (!g_sweep_force &&
      g_display_buffer.min[c] == g_shadow_min[c] &&
      g_display_buffer.max[c] == g_shadow_max[c] &&
      g_display_buffer.flags[c] == g_shadow_flags[c])
  {
    g_trace_column++;

    if (g_trace_column == (GRID_WIDTH-1))
      g_shadow_valid = true;

    return;
  }

  build_trace_column(c, column);

  // An opaque panel is a hole in the sweep: columns it covers paint only
  // the part below it, so the panel pixels are never touched
  int hole = 0;

  if (g_decode_mode && c >= DECODE_PANEL_COL0 && c <= DECODE_PANEL_COL1)
    hole = DECODE_PANEL_H;
  else if (g_fft_mode && g_fft_panel_on && c >= FFT_PANEL_COL0 && c <= FFT_PANEL_COL1)
    hole = FFT_PANEL_H;

  if (hole)
    lcd_draw_buf(GRID_LEFT+1 + c, GRID_TOP+1 + hole, 1,
        GRID_HEIGHT-1 - hole, &column[hole]);
  else
    lcd_draw_buf(GRID_LEFT+1 + c, GRID_TOP+1, 1, GRID_HEIGHT-1, column);

  g_shadow_min[c]   = g_display_buffer.min[c];
  g_shadow_max[c]   = g_display_buffer.max[c];
  g_shadow_flags[c] = g_display_buffer.flags[c];

  g_trace_column++;

  if (g_trace_column == (GRID_WIDTH-1))
    g_shadow_valid = true;
}

//-----------------------------------------------------------------------------
static void draw_grid_frame(void)
{
  lcd_vline(10, 20, 220, GRID_FG_COLOR);
  lcd_vline(310, 20, 220, GRID_FG_COLOR);
  lcd_hline(10, 310, 20, GRID_FG_COLOR);
  lcd_hline(10, 310, 220, GRID_FG_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_ac_dc(void)
{
  if (g_toast_active)
    return;

  lcd_draw_image(54, STATUS_LINE_Y, config.ac_coupling ? &image_ac : &image_dc);
}

//-----------------------------------------------------------------------------
static void draw_horizontal_scale(void)
{
  if (g_toast_active)
    return;

  lcd_set_color(BG_COLOR, HSCALE_COLOR);
  lcd_puts(82, STATUS_LINE_Y, hs_str[config.horizontal_scale]);
}

//-----------------------------------------------------------------------------
static void draw_horizontal_position(void)
{
  char *str;

  if (g_toast_active || g_calibration_mode || config.measure_display || g_fft_mode)
    return;

  str = format_time(config.horizontal_position, true);
  lcd_set_color(BG_COLOR, HPOS_COLOR);
  lcd_puts(236, STATUS_LINE_Y, str);
}

//-----------------------------------------------------------------------------
// Erase only the previous marker's bounding box instead of clearing the whole
// 209-px gutter on every key repeat
static void draw_marker_image(MarkerRect *prev, int x, int y, const Image *image)
{
  int nx = x - image->ox;
  int ny = y - image->oy;

  if (prev->valid && (prev->x != nx || prev->y != ny ||
      prev->w != image->width || prev->h != image->height))
    lcd_fill_rect(prev->x, prev->y, prev->w, prev->h, BG_COLOR);

  lcd_draw_image(x, y, image);

  prev->x = nx;
  prev->y = ny;
  prev->w = image->width;
  prev->h = image->height;
  prev->valid = true;
}

//-----------------------------------------------------------------------------
static void draw_vertical_position(bool toast)
{
  int y = GRID_CENTER_Y - config.vertical_position;

  if (y < GRID_TOP)
    draw_marker_image(&g_vpos_marker, GRID_LEFT-5, GRID_TOP, &image_reference_level_up);
  else if (y > GRID_BOTTOM)
    draw_marker_image(&g_vpos_marker, GRID_LEFT-5, GRID_BOTTOM, &image_reference_level_down);
  else
    draw_marker_image(&g_vpos_marker, GRID_LEFT-2, y, &image_reference_level);

  if (toast)
  {
    char *str;

    toast_show();

    lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Vertical position");

    str = format_divisions((config.vertical_position * 100) / GRID_DIV_PX, true);
    lcd_puts(GRID_LEFT + 140, STATUS_LINE_Y, str);

    str = format_voltage(config.vertical_position_mv, true);
    lcd_puts(GRID_LEFT + 226, STATUS_LINE_Y, str);
  }
}

//-----------------------------------------------------------------------------
static void draw_vertical_scale(void)
{
  if (g_toast_active)
    return;

  lcd_set_color(BG_COLOR, VSCALE_COLOR);
  lcd_puts(10, STATUS_LINE_Y, vs_str[config.vertical_scale]);
}

//-----------------------------------------------------------------------------
static void draw_trigger_level(void)
{
  char *str;

  draw_marker_image(&g_trig_marker, GRID_RIGHT+2,
      GRID_CENTER_Y - config.trigger_level, &image_trigger_level);

  if (g_toast_active || g_calibration_mode || config.measure_display || g_fft_mode)
    return;

  str = format_voltage(config.trigger_level_mv - config.vertical_position_mv, true);
  lcd_set_color(BG_COLOR, TRIGGER_LEVEL_COLOR);
  lcd_puts(148, STATUS_LINE_Y, str);
}

//-----------------------------------------------------------------------------
static void draw_trigger_edge(void)
{
  if (g_toast_active || g_calibration_mode || config.measure_display || g_fft_mode)
    return;

  if (TRIGGER_EDGE_RISE == config.trigger_edge)
    lcd_draw_image(140, STATUS_LINE_Y, &image_trigger_edge_rise);
  else if (TRIGGER_EDGE_FALL == config.trigger_edge)
    lcd_draw_image(140, STATUS_LINE_Y, &image_trigger_edge_fall);
  else
    lcd_draw_image(140, STATUS_LINE_Y, &image_trigger_edge_both);
}

//-----------------------------------------------------------------------------
static void draw_trigger_mode(void)
{
  char *str;

  if (TRIGGER_MODE_AUTO == config.trigger_mode)
    str = "AUTO";
  else if (TRIGGER_MODE_NORMAL == config.trigger_mode)
    str = "NORM";
  else
    str = "SNGL";

  lcd_set_color(BG_COLOR, TRIGGER_MODE_COLOR);
  lcd_puts(10, 4, str);
}

//-----------------------------------------------------------------------------
static char *format_duty(int duty_x10)
{
  static char str[10];

  if (duty_x10 < 0)
    return "d --.-%";

  snprintf(str, sizeof(str), "d%3d.%d%%", duty_x10 / 10, duty_x10 % 10);

  return str;
}

//-----------------------------------------------------------------------------
// Classify the current record (type + THD), cached per acquisition. Runs the
// FFT on a non-consuming sample copy so the display pipeline is unaffected.
static void signal_info_update(void)
{
  static uint32_t cached_gen = 0xffffffff;
  uint32_t gen = capture_get_generation();
  Measure m;
  int period_ns = 0;

  if (gen == cached_gen)
    return;

  if (!capture_get_raw_measure(&m))
    return;

  if (capture_read_samples(g_fft_samples, FFT_SIZE, &period_ns, false) == FFT_SIZE)
  {
    int fund_bin = (int)(((int64_t)m.frequency * FFT_SIZE * period_ns) / 1000000000ll);

    fft_spectrum(g_fft_samples, FFT_SIZE, 0, g_fft_mag);
    classify_signal(&m, g_fft_mag, FFT_BINS, fund_bin, &g_signal_class);
  }
  else
  {
    classify_signal(&m, NULL, 0, 0, &g_signal_class);
  }

  cached_gen = gen;
}

//-----------------------------------------------------------------------------
// Render a text line into the measurements panel mask (1 bit per pixel)
static void mpanel_render_text(int row, const char *str)
{
  const Font *font = FONT_SMALL;
  int x = 4;
  int y = 3 + row * 10;

  for (; *str; str++)
  {
    char ch = *str;

    if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
      ch = ' ';

    const uint8_t *bitmap = font->data + (ch - FONT_FIRST_CHAR) * font->pitch;
    int size = font->width * font->height;

    for (int i = 0; i < size; i++)
    {
      if ((bitmap[i / 8] >> (i % 8)) & 1)
      {
        int px = x + i % font->width;
        int py = y + i / font->width;

        if (px < GRID_WIDTH && py < MPANEL_H)
          g_mpanel_mask[py][px / 8] |= (uint8_t)(1 << (px % 8));
      }
    }

    x += font->width;

    if (x > GRID_WIDTH - font->width)
      break;
  }
}

//-----------------------------------------------------------------------------
// Repaint just a horizontal band of the trace area, immediately and without
// touching the column shadows: build_trace_column() composes from current
// state, so a later sweep of the same column produces identical pixels.
// ~10x cheaper than forcing a full 300-column sweep for an overlay change.
static void overlay_repaint_region(int row0, int rows)
{
  uint16_t column[GRID_HEIGHT];

  for (int c = 0; c < GRID_WIDTH - 1; c++)
  {
    // Never paint into the opaque decode panel hole
    if (g_decode_mode && c >= DECODE_PANEL_COL0 && c <= DECODE_PANEL_COL1 &&
        row0 < DECODE_PANEL_H)
      continue;

    build_trace_column(c, column);
    lcd_draw_buf(GRID_LEFT+1 + c, GRID_TOP+1 + row0, 1, rows, &column[row0]);
  }
}

//-----------------------------------------------------------------------------
// Forget the panel's rendered state so the next update rebuilds the text and
// repaints the band even if the numbers happen to come out identical. Needed
// after anything that leaves the bottom of the screen in an unknown state:
// the panel is only repainted when its TEXT changes, so a band that was
// cleared by something else would otherwise stay blank indefinitely.
static void mpanel_invalidate(void)
{
  g_mpanel_line[0][0] = '\x01'; // never matches a freshly built line
  g_mpanel_line[1][0] = 0;
  g_mpanel_force = true;        // and do it on the next tick, not in 500 ms

  if (config.measure_display)
    g_measure_timer = 0;
}

//-----------------------------------------------------------------------------
static void mpanel_set_active(bool active)
{
  if (active == g_mpanel_active)
    return;

  g_mpanel_active = active;
  mpanel_invalidate();
  overlay_repaint_region(MPANEL_ROW0, MPANEL_H);
}

//-----------------------------------------------------------------------------
// Rebuild the panel text from the selected metrics; on change, re-render the
// mask and invalidate the affected columns
static void mpanel_update(void)
{
  ScopeMeasure sm;
  char items[7][22];
  char line[2][MPANEL_TEXT_MAX + 2];
  int n = 0;
  static int throttle = 0;
  static int heartbeat = 0;

  // Live values jitter in their last digit; rebuilding the text (and
  // repainting the band) 10x a second serialized the main loop behind the
  // LCD bus and made buttons feel mushy. 2 Hz is plenty for reading.
  if (!g_mpanel_force && ++throttle < 5)
    return;

  g_mpanel_force = false;
  throttle = 0;

  // Unthrottled: this runs at 2 Hz anyway, and the 10 Hz recompute throttle
  // behind capture_get_measurements() can only ever hand back an older frame
  if (!capture_get_measurements_fresh(&sm))
    return;

  // A config saved before these flags existed reads all-false: default set
  bool any = config.show_vpp || config.show_freq || config.show_duty ||
      config.show_vrms || config.show_vavg || config.show_type || config.show_thd;

  bool s_vpp  = any ? config.show_vpp  : true;
  bool s_freq = any ? config.show_freq : true;
  bool s_duty = any ? config.show_duty : true;
  bool s_vrms = any ? config.show_vrms : true;
  bool s_vavg = any && config.show_vavg;
  bool s_type = any && config.show_type;
  bool s_thd  = any && config.show_thd;

  if (s_vpp)
    snprintf(items[n++], 22, "Vpp%s", format_voltage(sm.vpp_mv, false));

  if (s_freq)
    snprintf(items[n++], 22, "f %s", format_frequency(sm.frequency));

  if (s_duty)
    snprintf(items[n++], 22, "%s", format_duty(sm.duty_x10));

  if (s_vrms)
    snprintf(items[n++], 22, "rms%s", format_voltage(sm.vrms_mv, false));

  if (s_vavg)
    snprintf(items[n++], 22, "avg%s", format_voltage(sm.vavg_mv, false));

  if (s_type || s_thd)
  {
    signal_info_update();

    if (s_type)
      snprintf(items[n++], 22, "%s", classify_name(g_signal_class.type));

    if (s_thd && g_signal_class.thd_x10 >= 0)
      snprintf(items[n++], 22, "t%d.%d%%",
          g_signal_class.thd_x10 / 10, g_signal_class.thd_x10 % 10);
  }

  // Flow the items across the two lines
  line[0][0] = 0;
  line[1][0] = 0;

  for (int i = 0, li = 0; i < n && li < 2; i++)
  {
    if (strlen(line[li]) + strlen(items[i]) + 2 > MPANEL_TEXT_MAX)
    {
      if (++li >= 2)
        break;
    }

    if (line[li][0])
      strcat(line[li], "  ");

    strcat(line[li], items[i]);
  }

  if (0 == strcmp(line[0], g_mpanel_line[0]) &&
      0 == strcmp(line[1], g_mpanel_line[1]))
  {
    // Same text, so the mask is still right — but repaint it every few
    // seconds anyway. The band lives inside the trace area, and anything
    // that paints there without going through build_trace_column() leaves a
    // hole the sweep will not heal (it skips columns whose trace data has
    // not changed). A steady signal is exactly when that hole would stay.
    if (++heartbeat < MPANEL_HEARTBEAT)
      return;

    heartbeat = 0;
    overlay_repaint_region(MPANEL_ROW0, MPANEL_H);
    return;
  }

  heartbeat = 0;
  strcpy(g_mpanel_line[0], line[0]);
  strcpy(g_mpanel_line[1], line[1]);

  memset(g_mpanel_mask, 0, sizeof(g_mpanel_mask));
  mpanel_render_text(0, line[0]);
  mpanel_render_text(1, line[1]);

  overlay_repaint_region(MPANEL_ROW0, MPANEL_H);
}

//-----------------------------------------------------------------------------
// Measurements come from the full capture record (measure.c): mid-level
// crossing frequency with adaptive hysteresis, true RMS, duty cycle. Two
// pages alternate: [M] Vpp + frequency, [R] Vrms + duty. In FFT mode the
// slot shows [F] Vpp + spectrum peak frequency instead.
static void fft_format_hz(char *buf, int size, float hz);
static void fft_format_db(char *buf, int size, float db);

static void draw_measure(void)
{
  ScopeMeasure sm;
  char *str;

  if (g_toast_active || g_calibration_mode || g_mpanel_active)
    return;

  if (!config.measure_display && !g_fft_mode)
    return;

  if (g_fft_mode)
  {
    // The spectrum readout: F0 and the resolution, or the cursor values
    // while the cursor is up. Fixed-width fields so a shorter value always
    // overwrites the previous one completely.
    char line[64], n1[20], n2[20];

    lcd_set_color(BG_COLOR, MEASURE_MODE_COLOR);
    lcd_putc(140, STATUS_LINE_Y, 'F');

    lcd_set_font(FONT_SMALL);

    if (!g_fft_have)
    {
      lcd_set_color(BG_COLOR, MEASURE_FREQ_COLOR);
      lcd_puts(152, STATUS_LINE_Y + 4, "waiting for a record    ");
    }
    else if (g_fft_cursor >= 0)
    {
      float m = g_fft_mag[g_fft_cursor_bin];
      float db = (g_fft_an.count > 0 && m > 0.0f) ?
          20.0f * log10f(m / g_fft_an.peak[0].mag) : -99.9f;

      fft_format_hz(n1, sizeof(n1), g_fft_cursor_bin * g_fft_an.bin_hz);
      fft_format_db(n2, sizeof(n2), db);
      snprintf(line, sizeof(line), "> %-9s %-9s  ", n1, n2);
      lcd_set_color(BG_COLOR, TRIGGER_LEVEL_COLOR);
      lcd_puts(152, STATUS_LINE_Y + 4, line);
    }
    else
    {
      fft_format_hz(n1, sizeof(n1), g_fft_an.fundamental);
      fft_format_hz(n2, sizeof(n2), g_fft_an.bin_hz);
      snprintf(line, sizeof(line), "F0 %-9s df %-8s", n1, n2);
      lcd_set_color(BG_COLOR, MEASURE_FREQ_COLOR);
      lcd_puts(152, STATUS_LINE_Y + 4, line);
    }

    lcd_set_font(FONT_LARGE);
    return;
  }

  if (!capture_get_measurements(&sm))
    return;

  lcd_set_color(BG_COLOR, MEASURE_MODE_COLOR);

  if (g_measure_page == 0)
  {
    lcd_putc(140, STATUS_LINE_Y, 'M');

    str = format_voltage(sm.vpp_mv, false);
    lcd_set_color(BG_COLOR, MEASURE_VOLTAGE_COLOR);
    lcd_puts(148, STATUS_LINE_Y, str);

    str = format_frequency(sm.frequency);
    lcd_set_color(BG_COLOR, MEASURE_FREQ_COLOR);
    lcd_puts(236, STATUS_LINE_Y, str);
  }
  else if (g_measure_page == 1)
  {
    lcd_putc(140, STATUS_LINE_Y, 'R');

    str = format_voltage(sm.vrms_mv, false);
    lcd_set_color(BG_COLOR, MEASURE_VOLTAGE_COLOR);
    lcd_puts(148, STATUS_LINE_Y, str);

    lcd_set_color(BG_COLOR, MEASURE_FREQ_COLOR);
    lcd_puts(236, STATUS_LINE_Y, format_duty(sm.duty_x10));
  }
  else
  {
    char thd[10];

    signal_info_update();

    lcd_putc(140, STATUS_LINE_Y, 'S');

    lcd_set_color(BG_COLOR, MEASURE_VOLTAGE_COLOR);
    lcd_puts(148, STATUS_LINE_Y, classify_name(g_signal_class.type));

    if (g_signal_class.thd_x10 >= 0)
    {
      snprintf(thd, sizeof(thd), "t%3d.%d%%",
          g_signal_class.thd_x10 / 10, g_signal_class.thd_x10 % 10);
      lcd_set_color(BG_COLOR, MEASURE_FREQ_COLOR);
      lcd_puts(236, STATUS_LINE_Y, thd);
    }
  }
}

//-----------------------------------------------------------------------------
static void draw_capture_state(void)
{
  int state = capture_get_state();
  int color = BG_COLOR;
  char *str = "";

  if (g_state == state)
    return;

  if (CAPTURE_STATE_STOP == state)
  {
    color = CAPTURE_STOP_COLOR;
    str = "STOP";
  }
  else if (CAPTURE_STATE_WAIT == state)
  {
    color = CAPTURE_WAIT_COLOR;
    str = "WAIT";
  }
  else if (CAPTURE_STATE_TRIG == state)
  {
    color = CAPTURE_TRIG_COLOR;
    str = "TRIG";
  }

  lcd_set_color(BG_COLOR, color);
  lcd_puts(46, 4, str);

  g_state = state;
}

//-----------------------------------------------------------------------------
// x runs [-MINIVIEW_WIDTH/2+1, MINIVIEW_WIDTH/2), i.e. -79..79 => 159 columns
#define MINIVIEW_COLS  (MINIVIEW_WIDTH - 1)
#define MINIVIEW_ROWS  8

// Which decoded byte covers this miniview column? The miniview maps the
// whole record linearly onto its width, so record position p sits at
// x = (p/size - 1/2) * MINIVIEW_WIDTH.
static int miniview_byte_at(int x)
{
  if (!g_decode_mode || !g_logic_have || g_logic.count == 0 || g_decode_size <= 0)
    return -1;

  int64_t p = ((int64_t)x * g_decode_size) / MINIVIEW_WIDTH + g_decode_size / 2;

  for (int b = 0; b < g_logic.count; b++)
  {
    if (p >= g_logic.pos[b] && p < g_logic.end[b])
      return b;
  }

  return -1;
}

//-----------------------------------------------------------------------------
static void draw_miniview(int trigger_offset, int window_offset, int window_width)
{
  static const uint8_t wave_pattern[8] = { 1, 0, 0, 1, 2, 3, 3, 2 };
  // Rendered row-major and sent as one blit: the 158 separate one-pixel-wide
  // lcd_draw_buf calls this replaces each paid ~40% window-setup overhead
  static uint16_t buf[MINIVIEW_ROWS * MINIVIEW_COLS];

  for (int x = -MINIVIEW_WIDTH/2+1; x < MINIVIEW_WIDTH/2; x++)
  {
    bool inside = ((x > window_offset) && (x < (window_offset + window_width)));
    bool edge = ((x == window_offset) || (x == (window_offset + window_width - 1)));
    int cx = x + MINIVIEW_WIDTH/2 - 1;
    // C truncates toward zero, so a bare x % 8 is negative for the left half
    // of the miniview; that indexed wave_pattern out of bounds and the value
    // read there was then used as a write index into this column
    int phase = ((x % (int)sizeof(wave_pattern)) + sizeof(wave_pattern)) % (int)sizeof(wave_pattern);
    int byte_here = miniview_byte_at(x);
    uint16_t col[MINIVIEW_ROWS];

    if (edge)
    {
      for (int i = 0; i < MINIVIEW_ROWS; i++)
        col[i] = MV_FRAME_COLOR;
    }
    else
    {
      for (int i = 0; i < MINIVIEW_ROWS; i++)
        col[i] = BG_COLOR;

      if (byte_here >= 0)
      {
        // Decoder view: the record map shows where the decoded bytes sit
        // instead of the decorative wave
        uint16_t color = (byte_here == g_decode_sel) ? DSTRIP_SEL :
            ((byte_here & 1) ? DSTRIP_ODD : DSTRIP_EVEN);

        for (int i = 2; i <= 5; i++)
          col[i] = color;
      }
      else
      {
        col[2 + wave_pattern[phase]] = inside ? TRACE_COLOR : MV_FRAME_COLOR;
      }

      if (inside)
      {
        col[0] = MV_FRAME_COLOR;
        col[MINIVIEW_ROWS-1] = MV_FRAME_COLOR;
      }
    }

    for (int i = 0; i < MINIVIEW_ROWS; i++)
      buf[i * MINIVIEW_COLS + cx] = col[i];
  }

  lcd_draw_buf(GRID_CENTER_X - MINIVIEW_WIDTH/2 + 1, 7, MINIVIEW_COLS, MINIVIEW_ROWS, buf);

#define LEFT   (GRID_CENTER_X - MINIVIEW_WIDTH/2)
#define RIGHT  (GRID_CENTER_X + MINIVIEW_WIDTH/2)

  lcd_fill_rect(LEFT - image_trigger_mv.width/2, 1, MINIVIEW_WIDTH + image_trigger_mv.width,
      image_trigger_mv.height, BG_COLOR);
  lcd_draw_image(GRID_CENTER_X + trigger_offset, 5, &image_trigger_mv);

  lcd_vline(LEFT, 6, 15, MV_FRAME_COLOR);
  lcd_hline(LEFT, LEFT+2, 6, MV_FRAME_COLOR);
  lcd_hline(LEFT, LEFT+2, 15, MV_FRAME_COLOR);

  lcd_vline(RIGHT, 6, 15, MV_FRAME_COLOR);
  lcd_hline(RIGHT-2, RIGHT, 6, MV_FRAME_COLOR);
  lcd_hline(RIGHT-2, RIGHT, 15, MV_FRAME_COLOR);

#undef LEFT
#undef RIGHT
}

//-----------------------------------------------------------------------------
static void draw_sample_rates(int sample_rate_limit, int sample_rate)
{
  char *str;

  lcd_set_font(FONT_SMALL);

  str = format_sps(sample_rate_limit);
  lcd_set_color(BG_COLOR, SR_LIMIT_COLOR);
  lcd_puts(252, 2, str);

  str = format_sps(sample_rate);
  lcd_set_color(BG_COLOR, SR_COLOR);
  lcd_puts(252, 10, str);

  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
static void update_sample_rate(void)
{
  int64_t hp_abs = (config.horizontal_position < 0) ? -config.horizontal_position : config.horizontal_position;
  int64_t window_time = (int64_t)hs_div_value[config.horizontal_scale] * GRID_DIVS_H;
  int64_t period = BASE_SAMPLE_PERIOD;
  int64_t trigger_margin, trigger_offset;
  int64_t buffer_time, required_time;
  int64_t window_offset;
  int64_t denom;
  int sample_rate = BASE_SAMPLE_RATE;
  int sample_rate_limit;
  int trigger_offset_px, window_offset_px, window_width_px;
  int sr_divider = config.sample_rate_limit;

  for (int i = 0; i < config.sample_rate_limit; i++)
  {
    period *= 2;
    sample_rate /= 2;
  }

  sample_rate_limit = sample_rate;

  while (1)
  {
    trigger_margin = period * TRIGGER_MARGIN_SAMPLES;
    required_time = trigger_margin + hp_abs + window_time/2;
    buffer_time = (int64_t)CAPTURE_BUFFER_SIZE * period;

    if (required_time < window_time)
      required_time = window_time;

    if (required_time < buffer_time)
      break;

    sr_divider++;
    period *= 2;
    sample_rate /= 2;
  }

  g_calibration_dual_channel = (sr_divider == 0);

  denom = buffer_time - window_time/2 - trigger_margin;
  trigger_offset = -config.horizontal_position * (buffer_time/2 - trigger_margin) / denom;
  window_offset = trigger_offset + config.horizontal_position;

  capture_set_horizontal_parameters(sr_divider, CAPTURE_BUFFER_SIZE/2 + trigger_offset / period);

  denom = period * CAPTURE_BUFFER_SIZE;

  trigger_offset_px = (trigger_offset * MINIVIEW_WIDTH) / denom;
  window_offset_px  = ((window_offset - window_time/2) * MINIVIEW_WIDTH) / denom;
  window_width_px   = (window_time * MINIVIEW_WIDTH) / denom;

  if (window_offset_px == 0)
    window_offset_px = -1;

  if (window_width_px < 3)
    window_width_px = 3;

  g_mv_trigger_px = trigger_offset_px;
  g_mv_window_px = window_offset_px;
  g_mv_width_px = window_width_px;

  draw_miniview(trigger_offset_px, window_offset_px, window_width_px);
  draw_sample_rates(sample_rate_limit, sample_rate);
}

//-----------------------------------------------------------------------------
// Repaint the miniview with its last geometry (decode results changed, the
// selection moved) without the DMA restart update_sample_rate() would do
static void redraw_miniview(void)
{
  draw_miniview(g_mv_trigger_px, g_mv_window_px, g_mv_width_px);
}

//-----------------------------------------------------------------------------
static int clip_for_display(int value)
{
  value = GRID_HEIGHT/2-1 - value;

  if (value > (GRID_HEIGHT-2))
    value = (GRID_HEIGHT-2);
  else if (value < 0)
    value = 0;

  return value;
}

//---------------------------------------------------------------------
static void close_gaps(DisplayBuffer *db)
{
  for (int i = 0; i < GRID_WIDTH-1; i++)
  {
    if ((db->flags[i] & SAMPLE_FLAG_VALID) && (db->flags[i+1] & SAMPLE_FLAG_VALID))
    {
      if (db->max[i] < (db->min[i+1]-1))
      {
        int avg = (db->max[i] + db->min[i+1]) / 2;
        db->max[i] = avg;
        db->min[i+1] = avg+1;
      }
      else if (db->min[i] > (db->max[i+1] + 1))
      {
        int avg = (db->min[i] + db->max[i+1]) / 2;
        db->min[i] = avg;
        db->max[i+1] = avg-1;
      }
    }
  }
}

//-----------------------------------------------------------------------------
static void update_display(void)
{
  int scale = vs_px_value[config.vertical_scale];

  g_data_buffer.size = GRID_WIDTH;
  capture_get_data(&g_data_buffer);

  for (int i = 0; i < GRID_WIDTH; i++)
  {
    int min = (g_data_buffer.min[i] - g_data_buffer.vertical_position) / scale + config.vertical_position;
    int max = (g_data_buffer.max[i] - g_data_buffer.vertical_position) / scale + config.vertical_position;

    g_display_buffer.min[i]   = clip_for_display(max);
    g_display_buffer.max[i]   = clip_for_display(min);
    g_display_buffer.flags[i] = g_data_buffer.flags[i];
  }

  close_gaps(&g_display_buffer);
  redraw_trace();
}

//-----------------------------------------------------------------------------
// How many bands (span/resolution steps) a record of this size offers:
// decimation runs from the whole record down to raw samples by halves
static int fft_band_count(int size)
{
  int max = fft_max_decimation(size);
  int n = 1;

  while ((max >> n) >= 1)
    n++;

  return n;
}

//-----------------------------------------------------------------------------
static int fft_decimation(int size)
{
  int decim = fft_max_decimation(size) >> g_fft_band;

  return (decim < 1) ? 1 : decim;
}

//-----------------------------------------------------------------------------
// Automatic band: the narrowest span (so the finest resolution) that still
// keeps the measured frequency well inside the spectrum. A mains record ends
// up on the whole-record transform, a 10 MHz square wave on the raw one.
// Returns false when there is no measurement to base the choice on, so the
// caller keeps asking instead of latching the fallback.
static bool fft_pick_auto_band(int size, int period_ns)
{
  ScopeMeasure sm;
  int max = fft_max_decimation(size);
  int band = 0;
  bool known = (period_ns > 0 && capture_get_measurements(&sm) &&
      sm.frequency > 0);

  if (known)
  {
    // nyquist = 1e9 / (2 * period * decim) >= MARGIN * frequency
    int64_t limit = 1000000000ll / ((int64_t)2 * FFT_AUTO_BAND_MARGIN *
        sm.frequency * period_ns);

    if (limit < 1)
      limit = 1;

    while ((max >> band) > limit && (max >> (band + 1)) >= 1)
      band++;
  }

  g_fft_band = band;

  return known;
}

//-----------------------------------------------------------------------------
static void fft_column_bins(int c, int *b0, int *b1)
{
  *b0 = 1 + c * (FFT_BINS - 1) / GRID_WIDTH;
  *b1 = 1 + (c + 1) * (FFT_BINS - 1) / GRID_WIDTH;

  if (*b1 >= FFT_BINS)
    *b1 = FFT_BINS - 1;
}

//-----------------------------------------------------------------------------
// Spectrum view: transform the record and render it through the same column
// pipeline as the trace (bars = min..max spans), so the dirty-column
// machinery keeps working. Display is normalized to the strongest bin over
// an FFT_DB_RANGE dynamic range.
//
// The transform spans the WHOLE record (FFT_SIZE points taken `decim`
// samples apart), not the newest 512 samples: frequency resolution is
// 1/(record time), so a 100 ms record resolves 10 Hz and separates mains
// 50 Hz from its harmonics. Transforming 512 raw samples of that same
// record covered 0.5 ms of it, i.e. 2 kHz bins, and could only ever report
// one number for a signal that is a sum of several.
static void fft_update(void)
{
  const uint8_t *data;
  int size, offset, period_ns, trigger_timepos;
  int decim;
  float maxm = 1e-9f;

  if (!g_fft_grad_ready)
    fft_build_gradient();

  if (!capture_get_record(&data, &size, &offset, &period_ns, &trigger_timepos))
    return;

  // Re-picking the automatic band on every frame would make it hunt between
  // two neighbouring choices whenever the measured frequency sits on the
  // boundary. Pick it once per record geometry instead — and note that the
  // record TIME is what matters, so a run/stop transition (which swaps a
  // 24 K record at 4x the period for a 96 K one) does not disturb it.
  if (g_fft_band_auto)
  {
    int64_t span = (int64_t)size * period_ns;

    if (span != g_fft_auto_span)
    {
      if (fft_pick_auto_band(size, period_ns))
        g_fft_auto_span = span;
    }
  }
  else if (g_fft_band >= fft_band_count(size))
    g_fft_band = fft_band_count(size) - 1; // record got shorter under us

  decim = fft_decimation(size);

  fft_spectrum_decim(data, size, offset, decim, g_fft_mag);
  fft_analyze(g_fft_mag, period_ns * decim, &g_fft_an);

  g_fft_peak_hz = (int)(g_fft_an.fundamental + 0.5f);
  g_fft_have = true;

  for (int i = 1; i < FFT_BINS; i++)
  {
    if (g_fft_mag[i] > maxm)
      maxm = g_fft_mag[i];
  }

  for (int c = 0; c < GRID_WIDTH; c++)
  {
    int b0, b1;
    float m = 0.0f;
    int h = 0;

    fft_column_bins(c, &b0, &b1);

    for (int b = b0; b <= b1; b++)
    {
      if (g_fft_mag[b] > m)
        m = g_fft_mag[b];
    }

    if (m > 0.0f)
    {
      float db = 20.0f * log10f(m / maxm); // 0 at the peak, negative below

      h = (int)((db + FFT_DB_RANGE) * (GRID_HEIGHT - 2) / FFT_DB_RANGE);

      if (h < 0)
        h = 0;
      else if (h > GRID_HEIGHT - 2)
        h = GRID_HEIGHT - 2;
    }

    g_display_buffer.min[c] = (GRID_HEIGHT - 2) - h;
    g_display_buffer.max[c] = (GRID_HEIGHT - 2) - h;
    g_display_buffer.flags[c] = SAMPLE_FLAG_VALID;
  }

  // Stitch neighbouring points into a continuous curve, exactly like the
  // waveform trace does
  close_gaps(&g_display_buffer);

  // Mark peak and cursor columns in the flags; update_from_spectrum turns
  // them into the floating ticks and the cursor hairline
  for (int i = 0; i < g_fft_an.count; i++)
  {
    int c = (g_fft_an.peak[i].bin - 1) * GRID_WIDTH / (FFT_BINS - 1);

    g_display_buffer.flags[c] |= (g_fft_an.peak[i].harmonic > 0) ?
        SAMPLE_FLAG_CLIP_H : SAMPLE_FLAG_CLIP_L;
  }

  if (g_fft_cursor >= 0)
  {
    int b0, b1;
    float m = -1.0f;

    fft_column_bins(g_fft_cursor, &b0, &b1);
    g_fft_cursor_bin = b0;

    for (int b = b0; b <= b1; b++)
    {
      if (g_fft_mag[b] > m)
      {
        m = g_fft_mag[b];
        g_fft_cursor_bin = b;
      }
    }

    g_display_buffer.flags[g_fft_cursor] |= SAMPLE_FLAG_FILLED;
  }

  g_fft_panel_pending = true;

  redraw_trace();
  draw_measure();
}

//-----------------------------------------------------------------------------
// Called on every new acquisition: the frame must be released even when the
// spectrum is not recomputed, or the DMA never refills the storage buffer
static void fft_tick(void)
{
  if (g_fft_timer == TIMER_DISABLE || g_fft_timer == 0)
  {
    g_fft_timer = FFT_UPDATE_TIMEOUT;
    fft_update();
  }

  capture_consume_frame();
}

//-----------------------------------------------------------------------------
// Repaint whatever the trace area currently shows. Vertical and timebase
// changes are meaningful in the spectrum view too (they set the ADC range
// and the record length), but they must not paint a waveform over it.
static void refresh_view(void)
{
  if (g_fft_mode)
    fft_update();
  else
    update_display();
}

//-----------------------------------------------------------------------------
// Spectrum cursor, in trace columns. The first press parks it on the
// strongest peak rather than on the screen edge.
static void fft_move_cursor(int delta)
{
  if (g_fft_cursor < 0)
  {
    int bin = (g_fft_an.count > 0) ? g_fft_an.peak[0].bin : 1;

    g_fft_cursor = (bin - 1) * GRID_WIDTH / (FFT_BINS - 1); // inverse mapping
  }
  else
  {
    g_fft_cursor += delta;
  }

  if (g_fft_cursor < 0)
    g_fft_cursor = 0;
  else if (g_fft_cursor > GRID_WIDTH - 1)
    g_fft_cursor = GRID_WIDTH - 1;

  fft_update();
}

//-----------------------------------------------------------------------------
// Span/resolution step: +1 doubles the analysed bandwidth and halves the
// frequency resolution, -1 the other way. Once stepped by hand the band
// stays put (the panel marks it with a '*') until the spectrum view is
// re-entered, so a mains record cannot silently jump back to a wide span.
static void fft_change_band(int delta)
{
  const uint8_t *data;
  int size, offset, period_ns, trigger_timepos, band;

  if (!capture_get_record(&data, &size, &offset, &period_ns, &trigger_timepos))
    return;

  band = g_fft_band + delta;

  if (band < 0 || band >= fft_band_count(size))
    return;

  g_fft_band = band;
  g_fft_band_auto = false;

  fft_update();
}

//-----------------------------------------------------------------------------
static void change_horizontal_scale(int delta)
{
  if ((delta < 0 && config.horizontal_scale == 0) ||
      (delta > 0 && config.horizontal_scale == HS_LAST))
    return;

  config.horizontal_scale += delta;
  config.horizontal_position_px = config.horizontal_position / hs_px_value[config.horizontal_scale];
  config.horizontal_period = hs_px_value[config.horizontal_scale];

  draw_horizontal_scale();
  draw_horizontal_position();
  update_sample_rate();
  refresh_view();
}

//-----------------------------------------------------------------------------
static void change_horizontal_position_ex(int delta, bool repeat)
{
  int div = hs_px_value[config.horizontal_scale];

  if ((delta < 0 && config.horizontal_position <= MIN_HORIZONTAL_POSITION) ||
      (delta > 0 && config.horizontal_position >= MAX_HORIZONTAL_POSITION))
    return;

  config.horizontal_position_px = config.horizontal_position / div + delta;
  config.horizontal_position = (int64_t)config.horizontal_position_px * div;

  draw_horizontal_position();

  // While the key is held, restarting the DMA and repainting the miniview
  // on every 10-50 ms repeat tick is what makes panning feel sticky: do the
  // heavy part every 4th tick and settle once after release
  if (repeat)
  {
    g_pan_settle_timer = 200;

    if (++g_pan_repeat_count % 4 == 0)
      update_sample_rate();
  }
  else
  {
    g_pan_repeat_count = 0;
    update_sample_rate();
  }

  refresh_view();
}

//-----------------------------------------------------------------------------
static void change_horizontal_position(int delta)
{
  change_horizontal_position_ex(delta, false);
}

//-----------------------------------------------------------------------------
static void change_vertical_scale(int delta)
{
  // Clamped, not just guarded at the ends: the auto-setup jumps straight to
  // the scale it computed, so |delta| is not always 1
  int scale = config.vertical_scale + delta;

  if (scale < 0)
    scale = 0;
  else if (scale > VS_LAST)
    scale = VS_LAST;

  if (scale == config.vertical_scale)
    return;

  config.vertical_scale = scale;
  config.vertical_mult = config.calib_vs_mult[config.vertical_scale];
  config.vertical_position_mv = config.vertical_position * vs_px_value[config.vertical_scale];

  config.trigger_level_mv = config.trigger_level * vs_px_value[config.vertical_scale];

  capture_set_vertical_parameters();
  capture_set_trigger_level(config.trigger_level_mv);
  draw_vertical_scale();
  draw_trigger_level();
  refresh_view();
}

//-----------------------------------------------------------------------------
static void change_vertical_position_ex(int delta, bool repeat)
{
  if ((delta < 0 && config.vertical_position <= MIN_VERTICAL_POSITION) ||
      (delta > 0 && config.vertical_position >= MAX_VERTICAL_POSITION))
    return;

  config.vertical_position += delta;
  config.vertical_position_mv = config.vertical_position * vs_px_value[config.vertical_scale];

  // Same held-key coalescing as horizontal pan: the DAC/DMA restart runs
  // every 4th repeat tick, the settle timer finalizes after release
  if (repeat)
  {
    g_pan_settle_timer = 200;

    if (++g_pan_repeat_count % 4 == 0)
      capture_set_vertical_parameters();
  }
  else
  {
    g_pan_repeat_count = 0;
    capture_set_vertical_parameters();
  }

  draw_vertical_position(true);
  draw_trigger_level();
  refresh_view();
}

//-----------------------------------------------------------------------------
static void change_vertical_position(int delta)
{
  change_vertical_position_ex(delta, false);
}

//-----------------------------------------------------------------------------
// UP/DOWN in the spectrum view: hop the cursor between detected peaks in
// frequency order (with wrap-around)
static void fft_jump_peak(int dir)
{
  int cols[FFT_MAX_PEAKS];
  int n = 0;

  if (!g_fft_have || g_fft_an.count == 0)
    return;

  for (int i = 0; i < g_fft_an.count; i++)
  {
    int c = (g_fft_an.peak[i].bin - 1) * GRID_WIDTH / (FFT_BINS - 1);
    int pos = n;

    while (pos > 0 && cols[pos - 1] > c)
    {
      cols[pos] = cols[pos - 1];
      pos--;
    }

    cols[pos] = c;
    n++;
  }

  if (n == 0)
    return;

  if (g_fft_cursor < 0)
  {
    g_fft_cursor = (dir > 0) ? cols[0] : cols[n - 1];
  }
  else if (dir > 0)
  {
    int next = cols[0]; // wrap-around default

    for (int i = n - 1; i >= 0; i--)
    {
      if (cols[i] > g_fft_cursor)
        next = cols[i];
    }

    g_fft_cursor = next;
  }
  else
  {
    int next = cols[n - 1];

    for (int i = 0; i < n; i++)
    {
      if (cols[i] < g_fft_cursor)
        next = cols[i];
    }

    g_fft_cursor = next;
  }

  fft_update();
}

//-----------------------------------------------------------------------------
// The spectrum view owns the whole key map: nothing may leak through to the
// waveform bindings (trigger level, panning) — those keys silently mutating
// acquisition state under a spectrum is what makes a mode feel bolted-on.
// Returns false only for the keys that keep their global meaning
// (SHIFT+MODE = leave, STOP = run/stop, AC/DC = coupling).
static bool fft_buttons(int buttons, bool shift, bool repeat)
{
  if ((buttons & BTN_LEFT) && (buttons & BTN_RIGHT))
  {
    g_fft_cursor = -1;
    fft_update();
  }
  else if (buttons & BTN_LEFT)
  {
    if (shift)
      change_horizontal_scale(-1); // record length -> resolution
    else
      fft_move_cursor(-1);
  }
  else if (buttons & BTN_RIGHT)
  {
    if (shift)
      change_horizontal_scale(1);
    else
      fft_move_cursor(1);
  }
  else if (buttons & BTN_UP)
  {
    if (shift)
      change_vertical_scale(1); // ADC range
    else
      fft_jump_peak(1);
  }
  else if (buttons & BTN_DOWN)
  {
    if (shift)
      change_vertical_scale(-1);
    else
      fft_jump_peak(-1);
  }
  else if (buttons & BTN_TRIG_UP)
  {
    if (!repeat)
      fft_change_band(1);
  }
  else if (buttons & BTN_TRIG_DOWN)
  {
    if (!repeat)
      fft_change_band(-1);
  }
  else if (buttons & BTN_MODE)
  {
    if (shift)
      return false; // SHIFT+MODE leaves the spectrum view

    if (!repeat)
    {
      g_fft_panel_on = !g_fft_panel_on;
      g_shadow_valid = false;
      g_sweep_force = true;

      if (g_fft_panel_on)
        g_fft_panel_pending = true;

      fft_update();
    }
  }
  else if (buttons & BTN_50P)
  {
    if (!repeat)
    {
      g_fft_cursor = -1;
      fft_move_cursor(0); // park on the strongest peak
    }
  }
  else if ((buttons & BTN_STOP) || (buttons & BTN_AC_DC))
  {
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// Push config.trigger_level (px) to hardware. Deliberately draw-free: also
// called from the system menu, which overlays the marker gutter (the screen
// is repainted in full when the menu closes).
void scope_apply_trigger_level(void)
{
  if (config.trigger_level < MIN_TRIGGER_LEVEL)
    config.trigger_level = MIN_TRIGGER_LEVEL;
  else if (config.trigger_level > MAX_TRIGGER_LEVEL)
    config.trigger_level = MAX_TRIGGER_LEVEL;

  config.trigger_level_mv = config.trigger_level * vs_px_value[config.vertical_scale];

  capture_set_trigger_level(config.trigger_level_mv);
}

//-----------------------------------------------------------------------------
static void change_trigger_level(int delta)
{
  if ((delta < 0 && config.trigger_level == MIN_TRIGGER_LEVEL) ||
      (delta > 0 && config.trigger_level == MAX_TRIGGER_LEVEL))
    return;

  config.trigger_level += delta;
  scope_apply_trigger_level();
  draw_trigger_level();
}

//-----------------------------------------------------------------------------
static void change_sample_rate_limit(int delta)
{
  if ((delta < 0 && config.sample_rate_limit == 0) ||
      (delta > 0 && config.sample_rate_limit == MAX_SAMPLE_RATE_LIMIT))
    return;

  config.sample_rate_limit += delta;

  update_sample_rate();
}

//-----------------------------------------------------------------------------
// Map every trace column to the decoded byte its center sample belongs to
//-----------------------------------------------------------------------------
// Run the protocol auto-decoder over the current record; flag the panel for
// a repaint only when the result actually changed. Optionally freeze the
// capture the moment a structured protocol is caught ("trigger on decode").
static void decode_update(void)
{
  const uint8_t *data;
  int size, offset, period_ns, trig_pos;
  LogicResult res;
  static uint32_t last_ms = 0;

  // Decoding a 24K record (worse: the whole auto-detect cascade) on every
  // acquisition starves the button path at fast frame rates; 4 Hz reading
  // of a serial console is indistinguishable from instant. The throttle is
  // on time alone: keying it off "have a result" let a decoder that never
  // matches anything run flat out, which is exactly the case that hurts.
  if (!g_decode_force && timer_ms() - last_ms < 250)
    return;

  g_decode_force = false;
  last_ms = timer_ms();

  if (!capture_get_record(&data, &size, &offset, &period_ns, &trig_pos))
    return;

  // Sticky protocol: once something matched, try it alone first and only
  // fall back to the full auto cascade when it stops matching
  proto_t forced = (proto_t)config.decoder_proto;

  if (forced == PROTO_AUTO && g_logic_have && g_logic.proto != PROTO_NONE &&
      g_logic.count > 0)
  {
    logic_decode(data, size, offset, period_ns, g_logic.proto,
        (LogicScratch *)CAPTURE_SPARE_RAM, &res);

    if (res.count == 0)
      logic_decode(data, size, offset, period_ns, PROTO_AUTO,
          (LogicScratch *)CAPTURE_SPARE_RAM, &res);
  }
  else
  {
    logic_decode(data, size, offset, period_ns, forced,
        (LogicScratch *)CAPTURE_SPARE_RAM, &res);
  }

  g_decode_period_ns = period_ns;
  g_decode_trig_pos = trig_pos;
  g_decode_size = size;

  bool changed = !g_logic_have || res.count != g_logic.count ||
      res.proto != g_logic.proto || res.rate != g_logic.rate ||
      memcmp(res.bytes, g_logic.bytes, (size_t)res.count) != 0;

  if (changed)
  {
    g_logic = res;
    g_logic_have = true;
    g_decode_panel_pending = true;

    if (g_decode_sel >= res.count)
      g_decode_sel = (res.count > 0) ? res.count - 1 : 0;

    // Stop-on-decode: RAW matches nearly anything, so it only counts as a
    // catch when the user explicitly selected it
    if (config.decoder_stop && res.count > 0 &&
        (res.proto != PROTO_RAW || (proto_t)config.decoder_proto == PROTO_RAW) &&
        capture_get_state() != CAPTURE_STATE_STOP)
      capture_stop();
  }

  // The byte map lives in the miniview: repaint it when the result changed
  if (changed)
    redraw_miniview();
}

//-----------------------------------------------------------------------------
// Center the display on the selected decoded byte (LEFT/RIGHT-style pan,
// but in whole bytes)
static void decode_jump_to_selected(void)
{
  if (!g_logic_have || g_logic.count == 0 || g_decode_period_ns <= 0)
    return;

  int64_t n_center = ((int64_t)g_logic.pos[g_decode_sel] +
      g_logic.end[g_decode_sel]) / 2 - g_decode_trig_pos;
  int64_t hpos = n_center * g_decode_period_ns;
  int div = hs_px_value[config.horizontal_scale];

  if (hpos < MIN_HORIZONTAL_POSITION)
    hpos = MIN_HORIZONTAL_POSITION;
  else if (hpos > MAX_HORIZONTAL_POSITION)
    hpos = MAX_HORIZONTAL_POSITION;

  config.horizontal_position_px = (int)(hpos / div);
  config.horizontal_position = (int64_t)config.horizontal_position_px * div;

  draw_horizontal_position();
  update_sample_rate(); // also repaints the miniview with the new selection
  update_display();

  g_decode_panel_pending = true; // the selection highlight moved
}

//-----------------------------------------------------------------------------
// Decoded-bytes overlay, drawn after a trace sweep completes so the sweep
// does not immediately paint over it (unchanged columns are skipped by the
// dirty-column logic, which keeps the panel intact on a stable signal)
static void draw_decode_panel(void)
{
  char line[36];

  g_decode_panel_pending = false;

  lcd_fill_rect(DECODE_PANEL_X, DECODE_PANEL_Y, DECODE_PANEL_W, DECODE_PANEL_H, BG_COLOR);
  lcd_draw_rect(DECODE_PANEL_X, DECODE_PANEL_Y, DECODE_PANEL_W, DECODE_PANEL_H, GRID_FG_COLOR);

  lcd_set_font(FONT_SMALL);
  lcd_set_color(BG_COLOR, TOAST_COLOR);

  if (!g_logic_have || g_logic.count == 0)
  {
    lcd_puts(DECODE_PANEL_X + 4, DECODE_PANEL_Y + 4, "Decoder: no frames");
  }
  else
  {
    // The 16-byte window follows the selected byte
    int show = (g_logic.count < 16) ? g_logic.count : 16;
    int base = g_decode_sel - 8;

    if (base > g_logic.count - show)
      base = g_logic.count - show;

    if (base < 0)
      base = 0;

    lcd_puts(DECODE_PANEL_X + 4, DECODE_PANEL_Y + 4, g_logic.info);

    for (int row = 0; row < 2; row++)
    {
      for (int i = 0; i < 8; i++)
      {
        int idx = base + row * 8 + i;

        if (idx >= g_logic.count)
          break;

        snprintf(line, sizeof(line), "%02X", g_logic.bytes[idx]);
        lcd_set_color(BG_COLOR,
            (idx == g_decode_sel) ? DSTRIP_SEL : LCD_WHITE_COLOR);
        lcd_puts(DECODE_PANEL_X + 4 + i * 18, DECODE_PANEL_Y + 14 + row * 9, line);
      }
    }

    char *p = line;

    for (int i = 0; i < show && p < line + 32; i++)
    {
      uint8_t ch = g_logic.bytes[base + i];
      *p++ = (ch >= 32 && ch < 127) ? (char)ch : '.';
    }

    *p = 0;
    lcd_set_color(BG_COLOR, MEASURE_FREQ_COLOR);
    lcd_puts(DECODE_PANEL_X + 4, DECODE_PANEL_Y + 34, line);
  }

  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
// "49.9Hz" / "1.23kHz" / "12.34MHz" into a caller-supplied buffer: a panel
// line holds two numbers and format_frequency()'s buffer is one deep.
// Rounding is done once, on the printed digit, so 49.97 Hz is "50.0Hz" and
// never "49.0Hz".
static void fft_format_hz(char *buf, int size, float hz)
{
  int v;

  if (hz < 0.0f)
    hz = 0.0f;

  if (hz < 1000.0f)
  {
    v = (int)(hz * 10.0f + 0.5f);
    snprintf(buf, size, "%d.%dHz", v / 10, v % 10);
  }
  else if (hz < 1000000.0f)
  {
    v = (int)(hz / 10.0f + 0.5f);
    snprintf(buf, size, "%d.%02dkHz", v / 100, v % 100);
  }
  else
  {
    v = (int)(hz / 10000.0f + 0.5f);
    snprintf(buf, size, "%d.%02dMHz", v / 100, v % 100);
  }
}

//-----------------------------------------------------------------------------
static void fft_format_db(char *buf, int size, float db)
{
  int v;

  if (db > 0.0f)
    db = 0.0f;
  else if (db < -99.9f)
    db = -99.9f;

  v = (int)(-db * 10.0f + 0.5f);

  if (v == 0)
    snprintf(buf, size, "0.0dB");
  else
    snprintf(buf, size, "-%d.%ddB", v / 10, v % 10);
}

//-----------------------------------------------------------------------------
static void fft_panel_row(int row, int color, const char *str)
{
  lcd_set_color(BG_COLOR, color);
  lcd_puts(FFT_PANEL_X + 4, FFT_PANEL_Y + 4 + row * 9, str);
}

//-----------------------------------------------------------------------------
// The spectrum breakdown: which frequencies the record is actually made of.
// Peaks that belong to the fundamental's harmonic comb are labelled h1..hN,
// everything else (interference, a second unrelated source, the noise
// floor poking through) is left unlabelled — that distinction is the whole
// point of the panel on a mains-plus-noise record.
static void draw_fft_panel(void)
{
  char line[80], num1[20], num2[20];
  int row = 0;

  g_fft_panel_pending = false;

  lcd_fill_rect(FFT_PANEL_X, FFT_PANEL_Y, FFT_PANEL_W, FFT_PANEL_H, BG_COLOR);
  lcd_draw_rect(FFT_PANEL_X, FFT_PANEL_Y, FFT_PANEL_W, FFT_PANEL_H, GRID_FG_COLOR);

  lcd_set_font(FONT_SMALL);

  if (!g_fft_have || g_fft_an.count == 0)
  {
    fft_panel_row(0, TOAST_COLOR, "Spectrum: no peaks");
    fft_panel_row(1, MEASURE_FREQ_COLOR, "(only noise found)");
    lcd_set_font(FONT_LARGE);
    return;
  }

  fft_format_hz(num1, sizeof(num1), g_fft_an.fundamental);

  if (g_fft_an.thd_x10 >= 0)
    snprintf(line, sizeof(line), "F0 %-9s THD%3d.%d%%", num1,
        g_fft_an.thd_x10 / 10, g_fft_an.thd_x10 % 10);
  else
    snprintf(line, sizeof(line), "F0 %s", num1);

  fft_panel_row(row++, TOAST_COLOR, line);

  fft_format_hz(num1, sizeof(num1), g_fft_an.bin_hz);
  fft_format_hz(num2, sizeof(num2), g_fft_an.nyquist_hz);
  snprintf(line, sizeof(line), "df %-9s BW %s%s", num1, num2,
      g_fft_band_auto ? "" : "*");
  fft_panel_row(row++, MEASURE_FREQ_COLOR, line);

  for (int i = 0; i < g_fft_an.count && i < FFT_PANEL_ROWS; i++)
  {
    const FftPeak *p = &g_fft_an.peak[i];
    char tag[16];

    if (p->harmonic > 0)
      snprintf(tag, sizeof(tag), "h%d", p->harmonic);
    else
      snprintf(tag, sizeof(tag), "-");

    fft_format_hz(num1, sizeof(num1), p->freq);
    fft_format_db(num2, sizeof(num2), p->rel_db);
    snprintf(line, sizeof(line), "%-3s%9s %8s", tag, num1, num2);

    fft_panel_row(row++, (p->harmonic > 0) ? LCD_WHITE_COLOR : HPOS_COLOR, line);
  }

  row = 2 + FFT_PANEL_ROWS;

  if (g_fft_cursor >= 0)
  {
    float m = g_fft_mag[g_fft_cursor_bin];
    float db = (m > 0.0f) ? 20.0f * log10f(m / g_fft_an.peak[0].mag) : -99.9f;

    fft_format_hz(num1, sizeof(num1), g_fft_cursor_bin * g_fft_an.bin_hz);
    fft_format_db(num2, sizeof(num2), db);
    snprintf(line, sizeof(line), "%-3s%9s %8s", ">", num1, num2);
    fft_panel_row(row, TRIGGER_LEVEL_COLOR, line);
  }
  else
  {
    fft_format_db(num1, sizeof(num1), g_fft_an.noise_db);
    snprintf(line, sizeof(line), "noise %s  <> cursor", num1);
    fft_panel_row(row, MEASURE_FREQ_COLOR, line);
  }

  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
// Put the trigger level at the mid point between the measured signal
// extremes (the classic 50% button). Draw-free public form for the system
// menu; the scope's own button path wraps it with the marker redraw.
void scope_trigger_50_percent(void)
{
  ScopeMeasure sm;

  if (!capture_get_measurements_fresh(&sm))
    return;

  config.trigger_level = sm.vmid_mv / vs_px_value[config.vertical_scale];

  scope_apply_trigger_level();
}

//-----------------------------------------------------------------------------
static void trigger_set_50_percent(void)
{
  scope_trigger_50_percent();
  draw_trigger_level();
}

//-----------------------------------------------------------------------------
// Auto-setup.
//
// Amplitude and frequency want opposite records. A frequency needs several
// periods inside the record; an amplitude only needs the envelope, which
// undersampling leaves intact. So the run is split into phases, each on the
// record that suits what it measures:
//
//   SURVEY - park on a ~100 ms record: mains hum and up shows several
//            periods there, and the envelope of a signal far above that
//            sample rate is still correct
//   VERT   - converge the vertical on that record: step out of clipping,
//            center the DC, then jump straight to the scale the measured
//            voltage asks for
//   FAST   - the maximum sample rate answers "is there something far too
//            fast for the survey record?", by crossings first and by the
//            spectrum of the raw ring second
//   REFINE - park on the timebase the estimate asks for and re-measure. An
//            aliased estimate cannot survive this: the new record samples
//            the signal thousands of times per period.
//
// Doing the vertical on a record that held a fraction of one period was the
// old bug: 50 Hz looked like a few counts of drift, the loop zoomed into the
// millivolt ranges and locked onto the noise floor.
//-----------------------------------------------------------------------------
// Spectral frequency estimate from the full-rate ring: crossing counting
// dies at ~10-20 MHz on the decimated record, the FFT peak survives up to
// the Nyquist edge of the raw ring (~62 MHz)
static int autoset_fft_probe(void)
{
  int period_ns = 0;

  if (capture_read_fast_samples(g_fft_samples, FFT_SIZE, &period_ns) < FFT_SIZE)
    return 0;

  if (period_ns <= 0)
    return 0;

  fft_spectrum(g_fft_samples, FFT_SIZE, 0, g_fft_mag);

  return fft_peak_frequency(g_fft_mag, period_ns);
}

//-----------------------------------------------------------------------------
static void autoset_set_horizontal(int hs)
{
  if (hs < 0)
    hs = 0;
  else if (hs > HS_LAST)
    hs = HS_LAST;

  config.horizontal_scale = hs;
  config.horizontal_position = 0;
  config.horizontal_position_px = 0;
  config.horizontal_period = hs_px_value[hs];

  draw_horizontal_scale();
  draw_horizontal_position();
  update_sample_rate();
}

//-----------------------------------------------------------------------------
// Timebase that puts periods_x10/10 whole periods across the screen
static void autoset_set_timebase(int freq, int periods_x10)
{
  int64_t need = (100000000ll * periods_x10) / freq / GRID_DIVS_H;
  int hs = 0;

  while (hs < HS_LAST && hs_div_value[hs] < need)
    hs++;

  autoset_set_horizontal(hs);
}

//-----------------------------------------------------------------------------
// A frequency is only believable when the record held several similar
// periods: noise crosses the mid level all the time and a bare "frequency"
// reading cannot tell the two apart
static bool autoset_freq_ok(const ScopeMeasure *sm)
{
  if (sm->frequency <= 0 || sm->periods < AUTOSET_MIN_PERIODS)
    return false;

  // Regular repetition is the strong case: a waveform generator, a clock,
  // mains hum.
  if (sm->period_good_pct >= AUTOSET_MIN_GOOD_PCT)
    return true;

  // But most things worth probing do NOT repeat evenly — UART frames, bus
  // traffic, an SWD line, a burst of packets. Their intervals are as
  // irregular as noise, and demanding regularity made the auto-setup walk
  // straight past them. What noise cannot fake is sitting AT two levels:
  // a logic line does it ~90% of the time, noise a few %. So an irregular
  // record still counts when it is unmistakably digital, and the median
  // interval is then a rough but usable timebase for it.
  return sm->level_pct >= AUTOSET_MIN_LEVEL_PCT;
}

//-----------------------------------------------------------------------------
// Position pixels that cancel a DC error of mid_err ADC counts. The offset
// DAC is calibrated so one position pixel moves the trace exactly one pixel.
static int autoset_center_step(int mid_err)
{
  int64_t mv = (int64_t)mid_err * config.calib_vs_mult[config.vertical_scale] /
      CALIB_MULTIPLIER;
  int step = (int)(mv / vs_px_value[config.vertical_scale]);

  if (step == 0)
    step = (mid_err > 0) ? 1 : -1;

  return step;
}

//-----------------------------------------------------------------------------
// The most sensitive scale that still keeps the whole trace on screen.
// Deliberately in PIXELS, not ADC counts: the 50 and 100 mV ranges share one
// analog gain and differ only in display zoom, so a counts-based target
// silently refuses to zoom at the sensitive end.
static int autoset_target_scale(int vamp_mv)
{
  int s;

  for (s = 0; s < VS_LAST; s++)
  {
    if (vamp_mv / vs_px_value[s] <= AUTOSET_TARGET_PX)
      break;
  }

  if (s < g_autoset_zoom_floor)
    s = g_autoset_zoom_floor;

  return s;
}

//-----------------------------------------------------------------------------
static void autoset_set_scale(int scale)
{
  change_vertical_scale(scale - config.vertical_scale); // clamps and redraws
}

//-----------------------------------------------------------------------------
// One vertical convergence step. Returns true when there is nothing left
// worth doing; every other path consumes the frame through update_display().
static bool autoset_vertical(const ScopeMeasure *sm)
{
  bool clip_hi = (sm->top_raw >= 254);
  bool clip_lo = (sm->base_raw <= 1);
  bool center_blocked = false;
  int mid_err = (sm->top_raw + sm->base_raw) / 2 - ZERO_POINT;
  int abs_err = (mid_err < 0) ? -mid_err : mid_err;
  int target;

  if (g_autoset_vert_tries >= AUTOSET_VERT_TRIES)
    return true; // budget spent: show what we have rather than hunt forever

  // 1) Off-center DC. Clipping on ONE rail is an offset problem, not a gain
  //    problem, so this runs before any sensitivity is given away.
  if (abs_err > AUTOSET_CENTER_TOL && !(clip_hi && clip_lo))
  {
    int abs_prev = (g_autoset_cent_prev < 0) ? -g_autoset_cent_prev :
        g_autoset_cent_prev;
    int want;

    // The step assumes the offset DAC pushes the reading up; if the error
    // grew instead, this analog chain works the other way round
    if (g_autoset_cent_prev != AUTOSET_NO_ERR && abs_err > abs_prev + 2)
      g_autoset_cent_dir = -g_autoset_cent_dir;

    want = config.vertical_position -
        g_autoset_cent_dir * autoset_center_step(mid_err);

    // Past the end of the offset range the DAC has nothing left to give at
    // this gain (a few volts of DC at 50 mV/div): the fix is coarser, not
    // more clamped steps that burn the whole budget without moving
    if (want >= MIN_VERTICAL_POSITION && want <= MAX_VERTICAL_POSITION)
    {
      config.vertical_position = want;
      config.vertical_position_mv =
          config.vertical_position * vs_px_value[config.vertical_scale];

      g_autoset_cent_prev = mid_err;
      g_autoset_vert_tries++;

      capture_set_vertical_parameters();
      draw_vertical_position(false);
      update_display();
      return false;
    }

    center_blocked = true;
  }

  // 2) Too much gain: both rails pinned, or the offset DAC out of range.
  //    Never zoom back into a scale that clipped during this run.
  if ((clip_hi || clip_lo || center_blocked) && config.vertical_scale < VS_LAST)
  {
    g_autoset_zoom_floor = config.vertical_scale + 1;
    g_autoset_cent_prev = AUTOSET_NO_ERR;
    g_autoset_vert_tries++;
    change_vertical_scale(1);
    return false;
  }

  if (clip_hi || clip_lo)
    return true; // pinned at the coarsest scale: nothing else to try

  // 3) The swing is a voltage, so the scale that displays it well can be
  //    computed outright instead of felt out one step per acquisition
  target = autoset_target_scale(sm->vamp_mv);

  if (target != config.vertical_scale)
  {
    // A swing that lands right on the boundary between two ranges maps back
    // to the one we just left; settle for the current one instead of
    // alternating until the budget runs out
    if (target == g_autoset_prev_scale)
      return true;

    g_autoset_prev_scale = config.vertical_scale;
    g_autoset_cent_prev = AUTOSET_NO_ERR; // a gain change moves the DC too
    g_autoset_vert_tries++;
    autoset_set_scale(target);
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
static void autoset_end(void)
{
  g_autoset_active = false;
  g_autoset_timer = TIMER_DISABLE;

  // The hunt rewrote the scale, the position, the timebase and the trigger
  // while a toast was up, and a toast suppresses every status-line element.
  // The measurements panel is the one part that is not toast-gated, so put
  // it back straight away; the rest follows when the toast expires.
  mpanel_invalidate();
}

//-----------------------------------------------------------------------------
static void autoset_finish(int freq)
{
  char str[32];

  if (freq > 0)
    autoset_set_timebase(freq, AUTOSET_VIEW_PERIODS);

  trigger_set_50_percent();
  draw_vertical_position(false);
  update_display();
  autoset_end();

  toast_show();

  if (freq >= 1000000)
    snprintf(str, sizeof(str), "Auto-setup: %d.%02d MHz", freq / 1000000,
        freq % 1000000 / 10000);
  else if (freq >= 1000)
    snprintf(str, sizeof(str), "Auto-setup: %d.%02d kHz", freq / 1000,
        freq % 1000 / 10);
  else
    snprintf(str, sizeof(str), "Auto-setup: %d Hz", freq);

  lcd_puts(GRID_LEFT, STATUS_LINE_Y, str);
}

//-----------------------------------------------------------------------------
// Nothing periodic at any sample rate. Leaving the scope parked on whatever
// the noise floor looked like is worse than useless, so the user's own
// settings come back.
static void autoset_abort(void)
{
  bool bottomed = (config.vertical_scale == 0) &&
      (g_autoset_vamp_mv < AUTOSET_MIN_SIGNAL_MV);

  autoset_set_horizontal(g_autoset_saved_hs);

  // A DC level or a one-shot is still worth showing at the scale we found;
  // a scale that bottomed out on a few counts of noise is not
  if (bottomed)
  {
    config.vertical_position = g_autoset_saved_vpos;
    config.vertical_scale = g_autoset_saved_vs;
    config.vertical_mult = config.calib_vs_mult[config.vertical_scale];
    config.vertical_position_mv =
        config.vertical_position * vs_px_value[config.vertical_scale];
    config.trigger_level_mv =
        config.trigger_level * vs_px_value[config.vertical_scale];

    capture_set_vertical_parameters();
    capture_set_trigger_level(config.trigger_level_mv);
    draw_vertical_scale();
    draw_vertical_position(false);
    draw_trigger_level();
  }

  update_display();
  autoset_end();

  toast_show();
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Auto-setup: no signal found");
}

//-----------------------------------------------------------------------------
// One auto-setup iteration, run on each fresh acquisition while active.
// Every path consumes the frame (update_display() directly or through
// change_*) so the next acquisition keeps flowing.
static void autoset_step(void)
{
  ScopeMeasure sm;

  // Fresh (unthrottled) measurements: this is a feedback loop, and steering
  // on a reading cached from before the previous step's own change is what
  // made it oscillate between vertical scales
  if (!capture_get_measurements_fresh(&sm))
  {
    update_display();
    return;
  }

  g_autoset_vamp_mv = sm.vamp_mv;

  switch (g_autoset_phase)
  {
    case AS_SURVEY:
      // ~100 ms of record: >= 5 periods of mains hum, and the envelope of
      // anything faster is still intact here
      if (config.horizontal_scale != AUTOSET_SURVEY_HS)
      {
        autoset_set_horizontal(AUTOSET_SURVEY_HS);
        update_display();
        return;
      }

      g_autoset_phase = AS_VERT;
      /* fall through: the record in hand is already a survey record */

    case AS_VERT:
      if (!autoset_vertical(&sm))
        return;

      // The long record is the honest witness: it holds many periods, its
      // crossings are gated by 20% of the DOMINANT swing (so small fast
      // junk riding on a big slow signal cannot claim the answer), and its
      // sample rate still resolves everything below ~100 kHz.
      if (autoset_freq_ok(&sm))
      {
        g_autoset_freq = sm.frequency;
        g_autoset_phase = AS_REFINE;
        autoset_set_timebase(g_autoset_freq, AUTOSET_VIEW_PERIODS);
        update_display();
        return;
      }

      // Nothing regular in 100 ms: whatever is out there is too fast for
      // this record, so go and ask the maximum sample rate
      g_autoset_phase = AS_FAST;
      autoset_set_horizontal(AUTOSET_FAST_HS);
      update_display();
      return;

    case AS_FAST:
    {
      // Maximum sample rate. A slow signal is a fraction of one period here,
      // so only a fast result counts: anything under the floor would have
      // been seen by the survey record, and was not, so it is junk.
      int freq = 0;

      if (autoset_freq_ok(&sm) && sm.frequency >= AUTOSET_FAST_MIN_HZ)
        freq = sm.frequency;

      if (freq == 0)
      {
        // Crossings die once a period is only a couple of samples wide; the
        // spectrum of the raw ring survives to the Nyquist edge
        int spectral = autoset_fft_probe();

        if (spectral >= AUTOSET_FAST_MIN_HZ)
          freq = spectral;
      }

      if (freq == 0)
      {
        // Not fast, and the survey record saw nothing either: it may simply
        // be slower than 100 ms of record can resolve
        g_autoset_phase = AS_SLOW;
        autoset_set_horizontal(AUTOSET_SLOW_HS);
        update_display();
        return;
      }

      g_autoset_freq = freq;
      g_autoset_phase = AS_REFINE;
      autoset_set_timebase(freq, AUTOSET_VIEW_PERIODS);
      update_display();
      return;
    }

    case AS_SLOW:
      // Eight times the record: the last chance for anything below the
      // ~30 Hz the survey record can resolve
      if (autoset_freq_ok(&sm))
      {
        g_autoset_freq = sm.frequency;
        g_autoset_phase = AS_REFINE;
        autoset_set_timebase(g_autoset_freq, AUTOSET_VIEW_PERIODS);
        update_display();
        return;
      }

      autoset_abort();
      return;

    case AS_REFINE:
    default:
    {
      // The timebase now matches the estimate, so this record samples the
      // signal thousands of times per period. If the estimate came from an
      // aliased record, the disagreement shows up right here.
      int freq = sm.frequency;
      int tol = (int)((int64_t)g_autoset_freq * AUTOSET_FREQ_TOL_PCT / 100);

      if (tol < 1)
        tol = 1;

      if (!autoset_freq_ok(&sm))
      {
        // Too few periods left on screen to re-measure: the estimate that
        // picked this timebase stands
        autoset_finish(g_autoset_freq);
        return;
      }

      if (g_autoset_refine_tries >= AUTOSET_REFINE_TRIES ||
          (freq >= g_autoset_freq - tol && freq <= g_autoset_freq + tol))
      {
        autoset_finish(freq);
        return;
      }

      g_autoset_refine_tries++;
      g_autoset_freq = freq;
      autoset_set_timebase(freq, AUTOSET_VIEW_PERIODS);
      update_display();
      return;
    }
  }
}

//-----------------------------------------------------------------------------
static void autoset_start(void)
{
  g_autoset_active = true;
  g_autoset_timer = AUTOSET_TIMEOUT;

  g_autoset_phase = AS_SURVEY;
  g_autoset_saved_hs = config.horizontal_scale;
  g_autoset_saved_vs = config.vertical_scale;
  g_autoset_saved_vpos = config.vertical_position;
  g_autoset_zoom_floor = 0;
  g_autoset_prev_scale = -1;
  g_autoset_cent_dir = 1;
  g_autoset_cent_prev = AUTOSET_NO_ERR;
  g_autoset_vert_tries = 0;
  g_autoset_refine_tries = 0;
  g_autoset_freq = 0;
  g_autoset_vamp_mv = 0;

  // AUTO trigger mode guarantees a steady stream of frames to iterate on
  if (config.trigger_mode != TRIGGER_MODE_AUTO)
  {
    config.trigger_mode = TRIGGER_MODE_AUTO;
    capture_set_trigger_mode(config.trigger_mode);
    draw_trigger_mode();
  }

  capture_start();

  toast_show();
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Auto-setup...");
}

//-----------------------------------------------------------------------------
static void change_calibration_value(int delta, bool shift)
{
  if (shift)
  {
    if (g_calibration_parameter == CALIB_OFFSET)
      g_calibration_parameter = CALIB_ZERO;
    else
      g_calibration_parameter++;

    lcd_fill_rect(CALIB_AREA_LEFT, GRID_BOTTOM+1, CALIB_AREA_WIDTH, STATUS_LINE_HEIGHT, BG_COLOR);

    return;
  }

  if (g_calibration_parameter == CALIB_ZERO)
  {
    config.calib_dac_zero += delta;

    if (config.calib_dac_zero < 1900)
      config.calib_dac_zero = 1900;
    else if (config.calib_dac_zero > 2200)
      config.calib_dac_zero = 2200;
  }
  else if (g_calibration_parameter == CALIB_DELTA)
  {
    config.calib_channel_delta += delta;

    if (config.calib_channel_delta < -64)
      config.calib_channel_delta = -64;
    else if (config.calib_channel_delta > 64)
      config.calib_channel_delta = 64;
  }
  else if (g_calibration_parameter == CALIB_SCALE)
  {
    config.calib_vs_mult[config.vertical_scale] += delta;

    if (config.calib_vs_mult[config.vertical_scale] < 0)
      config.calib_vs_mult[config.vertical_scale] = 0;
    else if (config.calib_vs_mult[config.vertical_scale] > 4000000)
      config.calib_vs_mult[config.vertical_scale] = 4000000;
  }
  else if (g_calibration_parameter == CALIB_OFFSET)
  {
    config.calib_dac_mult[config.vertical_scale] += delta;

    if (config.calib_dac_mult[config.vertical_scale] < 0)
      config.calib_dac_mult[config.vertical_scale] = 0;
    else if (config.calib_dac_mult[config.vertical_scale] > 100000)
      config.calib_dac_mult[config.vertical_scale] = 100000;
  }

  capture_set_vertical_parameters();
}

//-----------------------------------------------------------------------------
static void draw_calibration_info(void)
{
  static const char *labels[] = { "Z", "D", "S", "O" };
  char *str;

  lcd_set_color(BG_COLOR, LCD_WHITE_COLOR);

  if (!g_toast_active)
    lcd_puts(CALIB_AREA_LEFT, STATUS_LINE_Y, labels[g_calibration_parameter]);

  g_data_buffer.size = GRID_WIDTH;

  if (g_calibration_parameter == CALIB_ZERO || g_calibration_parameter == CALIB_DELTA)
  {
    if ((g_calibration_parameter == CALIB_ZERO) == g_calibration_dual_channel)
      lcd_set_color(BG_COLOR, LCD_RED_COLOR);

    capture_get_raw_data(g_data_buffer.max, GRID_WIDTH/2);

    if (g_calibration_dual_channel)
    {
      for (int i = 1; i < GRID_WIDTH/2; i += 2)
        g_data_buffer.max[i] = rbit8(g_data_buffer.max[i]) + config.calib_channel_delta;
    }

    if (!g_toast_active)
    {
      str = format_raw_data(g_data_buffer.max, 4);
      lcd_puts(CALIB_AREA_LEFT + 24, STATUS_LINE_Y, str);
    }

    #define PIXEL_SIZE 15
    for (int i = 0; i < GRID_WIDTH/2; i++)
    {
      g_data_buffer.max[i] -= ZERO_POINT;
      g_data_buffer.min[i] = g_data_buffer.max[i];

      if (0 == i % PIXEL_SIZE)
      {
        g_data_buffer.max[GRID_WIDTH/2 + i] = 0;
        g_data_buffer.min[GRID_WIDTH/2 + i] = 0;
      }
      else
      {
        int value = g_data_buffer.max[i / PIXEL_SIZE] * PIXEL_SIZE;
        g_data_buffer.max[GRID_WIDTH/2 + i] = value + PIXEL_SIZE/2;
        g_data_buffer.min[GRID_WIDTH/2 + i] = value - PIXEL_SIZE/2;
      }
    }

    for (int i = 0; i < GRID_WIDTH; i++)
    {
      g_display_buffer.min[i]   = clip_for_display(g_data_buffer.max[i]);
      g_display_buffer.max[i]   = clip_for_display(g_data_buffer.min[i]);
      g_display_buffer.flags[i] = SAMPLE_FLAG_VALID;
    }

    redraw_trace();
  }
  else // if (g_calibration_parameter == CALIB_SCALE || g_calibration_parameter == CALIB_OFFSET)
  {
    update_display();

    int vmin = g_data_buffer.min_value;
    int vmax = g_data_buffer.max_value;
    int vpos = g_data_buffer.vertical_position;

    if (!g_toast_active)
    {
      str = format_voltage(vmin - vpos, true);
      lcd_puts(CALIB_AREA_LEFT + 12, STATUS_LINE_Y, str);

      str = format_voltage(vmax - vpos, true);
      lcd_puts(CALIB_AREA_LEFT + 96, STATUS_LINE_Y, str);
    }
  }
}

//-----------------------------------------------------------------------------
static void draw_status_line(void)
{
  lcd_fill_rect(GRID_LEFT, GRID_BOTTOM+1, GRID_WIDTH+1, STATUS_LINE_HEIGHT, BG_COLOR);

  draw_vertical_scale();
  draw_ac_dc();
  draw_horizontal_scale();
  draw_horizontal_position();
  draw_trigger_level();
  draw_trigger_edge();
  draw_measure();
}

//-----------------------------------------------------------------------------
void scope_buttons_handler(int buttons)
{
  bool shift  = (buttons & BTN_SHIFT);
  bool repeat = (buttons & BTN_REPEAT);

  if (g_fft_mode && !g_calibration_mode)
  {
    if (fft_buttons(buttons, shift, repeat))
      return;
  }

  if ((buttons & BTN_UP) && (buttons & BTN_DOWN))
  {
    config.vertical_position = 0;
    config.vertical_position_mv = 0;

    capture_set_vertical_parameters();
    draw_vertical_position(true);
    refresh_view();
  }
  else if (buttons & BTN_UP)
  {
    if (shift)
      change_vertical_scale(1);
    else
      change_vertical_position_ex(1, repeat);
  }
  else if (buttons & BTN_DOWN)
  {
    if (shift)
      change_vertical_scale(-1);
    else
      change_vertical_position_ex(-1, repeat);
  }

  else if ((buttons & BTN_LEFT) && (buttons & BTN_RIGHT))
  {
    config.horizontal_position = 0;
    config.horizontal_position_px = 0;

    draw_horizontal_position();
    update_sample_rate();
    update_display();
  }
  else if (buttons & BTN_LEFT)
  {
    if (shift)
      change_horizontal_scale(-1);
    else
      change_horizontal_position_ex(1, repeat);
  }
  else if (buttons & BTN_RIGHT)
  {
    if (shift)
      change_horizontal_scale(1);
    else
      change_horizontal_position_ex(-1, repeat);
  }

  else if (buttons & BTN_TRIG)
  {
    if (repeat || g_calibration_mode)
      return;

    if (config.trigger_mode == TRIGGER_MODE_SINGLE)
      config.trigger_mode = TRIGGER_MODE_AUTO;
    else
      config.trigger_mode++;

    capture_set_trigger_mode(config.trigger_mode);
    capture_start();
    draw_trigger_mode();
  }
  else if (buttons & BTN_EDGE)
  {
    if (repeat || g_calibration_mode)
      return;

    if (shift)
    {
      // SHIFT+EDGE: toggle the protocol decoder view
      g_decode_mode = !g_decode_mode;

      if (g_decode_mode)
      {
        g_fft_mode = false;
        g_logic_have = false;
        g_decode_sel = 0;
        g_decode_force = true; // show something without waiting for the throttle
        trigger_set_50_percent();
        decode_update();
        g_decode_panel_pending = true;
        g_shadow_valid = false;
        g_sweep_force = true;
        update_display();
      }
      else
      {
        g_shadow_valid = false;
        g_sweep_force = true;
        update_display();
        redraw_miniview(); // restore the decorative wave over the byte map
      }

      draw_status_line();
      return;
    }

    if (config.trigger_edge == TRIGGER_EDGE_BOTH)
      config.trigger_edge = TRIGGER_EDGE_RISE;
    else
      config.trigger_edge++;

    capture_set_trigger_edge(config.trigger_edge);
    draw_trigger_edge();
  }
  else if (buttons & BTN_TRIG_UP)
  {
    if (g_calibration_mode)
      change_calibration_value(1, shift);
    else if (shift)
      change_sample_rate_limit(1);
    else if (g_decode_mode && g_logic_have && g_logic.count > 0)
    {
      // In decoder view these keys step through the decoded bytes
      if (g_decode_sel + 1 < g_logic.count)
      {
        g_decode_sel++;
        decode_jump_to_selected();
      }
    }
    else
      change_trigger_level(1);
  }
  else if (buttons & BTN_TRIG_DOWN)
  {
    if (g_calibration_mode)
      change_calibration_value(-1, shift);
    else if (shift)
      change_sample_rate_limit(-1);
    else if (g_decode_mode && g_logic_have && g_logic.count > 0)
    {
      if (g_decode_sel > 0)
      {
        g_decode_sel--;
        decode_jump_to_selected();
      }
    }
    else
      change_trigger_level(-1);
  }

  else if (buttons & BTN_AC_DC)
  {
    if (repeat)
      return;

    config.ac_coupling = !config.ac_coupling;

    capture_set_vertical_parameters();
    draw_ac_dc();
  }

  else if (buttons & BTN_MODE)
  {
    if (repeat)
      return;

    if (shift && !g_calibration_mode)
    {
      // SHIFT+MODE: toggle the FFT spectrum view
      g_fft_mode = !g_fft_mode;
      g_decode_mode = false;
      g_shadow_valid = false;
      g_sweep_force = true;
      draw_status_line();

      if (g_fft_mode)
      {
        g_fft_cursor = -1;
        g_fft_band_auto = true;
        g_fft_auto_span = 0;
        g_fft_have = false;
        g_fft_timer = TIMER_DISABLE;
        g_fft_panel_on = true;
        g_fft_panel_pending = true; // draw it even if no record exists yet
        fft_update();
      }
      else
      {
        update_display();
      }
    }
    else
    {
      config.measure_display = !config.measure_display;
      g_measure_timer = config.measure_display ? MEASURE_UPDATE_TIMEOUT : TIMER_DISABLE;
      draw_status_line();
    }
  }

  else if (buttons & BTN_AUTO)
  {
    if (repeat || g_calibration_mode || g_fft_mode)
      return;

    autoset_start();
  }

  else if (buttons & BTN_50P)
  {
    if (repeat || g_calibration_mode)
      return;

    trigger_set_50_percent();
  }

  else if (buttons & BTN_SAVE)
  {
  }

  else if (buttons & BTN_STOP)
  {
    if (capture_get_state() == CAPTURE_STATE_STOP)
      capture_start();
    else
      capture_stop();
  }
}

//-----------------------------------------------------------------------------
void scope_init(bool calibration_mode)
{
  g_calibration_mode = calibration_mode;

  config.horizontal_period = hs_px_value[config.horizontal_scale];
  config.vertical_mult = config.calib_vs_mult[config.vertical_scale];

  grid_init();

  // The screen was just cleared: previous-frame caches are stale
  g_shadow_valid = false;
  g_sweep_force = true;
  g_vpos_marker.valid = false;
  g_trig_marker.valid = false;
  draw_grid_frame();
  draw_vertical_position(false);
  draw_trigger_mode();
  draw_capture_state();
  draw_status_line();
  redraw_trace();

  if (g_calibration_mode)
  {
    capture_set_trigger_edge(TRIGGER_EDGE_RISE);
    capture_set_trigger_mode(TRIGGER_MODE_AUTO);
    capture_set_trigger_level(0);
  }
  else
  {
    capture_set_trigger_edge(config.trigger_edge);
    capture_set_trigger_mode(config.trigger_mode);
    capture_set_trigger_level(config.trigger_level_mv);
  }

  timer_add(&g_toast_timer);
  timer_add(&g_state_timer);
  timer_add(&g_measure_timer);
  timer_add(&g_autoset_timer);
  timer_add(&g_pan_settle_timer);
  timer_add(&g_fps_timer);
  timer_add(&g_fft_timer);
  g_fps_timer = 1000;

  g_measure_timer = config.measure_display ? MEASURE_UPDATE_TIMEOUT : TIMER_DISABLE;

  update_sample_rate();
  capture_set_vertical_parameters();
  capture_start();
}

//-----------------------------------------------------------------------------
// Repaint everything the scope owns. Menus and modal dialogs paint over the
// whole screen and clear it on close, so the incremental renderer has to be
// told that what it thinks is on screen is gone.
void scope_redraw_all(void)
{
  g_shadow_valid = false;
  g_sweep_force = true;
  g_vpos_marker.valid = false;
  g_trig_marker.valid = false;
  g_state = -1;
  g_toast_active = false;
  g_toast_timer = TIMER_DISABLE;

  lcd_set_font(FONT_LARGE);
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_COLOR);

  draw_grid_frame();
  draw_vertical_position(false);
  draw_trigger_mode();
  draw_capture_state();
  draw_status_line();
  battery_redraw();
  update_sample_rate();
  redraw_trace();

  if (g_decode_mode)
    g_decode_panel_pending = true;

  if (g_fft_mode)
    g_fft_panel_pending = true;
}

//-----------------------------------------------------------------------------
int scope_get_fps(void)
{
  return g_fps_value;
}

//-----------------------------------------------------------------------------
void scope_task(void)
{
  if (trace_ready())
  {
    if (capture_buffer_updated())
    {
      g_fps_counter++;

      if (g_calibration_mode)
        draw_calibration_info();
      else if (g_autoset_active)
        autoset_step();
      else if (g_fft_mode)
        fft_tick();
      else
      {
        update_display();

        if (g_decode_mode)
          decode_update();
      }
    }
  }

  // A RUN/STOP transition switches the active record (decimated storage vs
  // full-rate capture buffer): the decode and its positions must follow.
  // Only STOP matters here — capture_get_state() also flips WAIT<->TRIG on
  // every single acquisition, and reacting to that ran the whole decoder
  // cascade dozens of times a second with the throttle bypassed.
  if (g_decode_mode)
  {
    static int prev_stopped = -1;
    int stopped = (capture_get_state() == CAPTURE_STATE_STOP);

    if (stopped != prev_stopped)
    {
      prev_stopped = stopped;
      g_decode_force = true;
      decode_update();
    }
  }

  // Same for the spectrum: STOP is where the analysis matters most. A
  // stopped acquisition delivers no more frames, so nothing else would ever
  // re-run it — and the record it freezes is the full-rate one, four times
  // longer than the storage record the running view analyzes.
  if (g_fft_mode)
  {
    static int prev_stopped = -1;
    int stopped = (capture_get_state() == CAPTURE_STATE_STOP);

    if (stopped != prev_stopped)
    {
      prev_stopped = stopped;
      g_fft_timer = TIMER_DISABLE;
      fft_update();
    }
  }

  if (g_autoset_active && g_autoset_timer == 0)
    autoset_abort(); // ran out of time: put the user's settings back

  if (g_fps_timer == 0)
  {
    g_fps_timer = 1000;
    g_fps_value = g_fps_counter;
    g_fps_counter = 0;
  }

  // A held pan key has been released: apply the coalesced heavy updates
  if (g_pan_settle_timer == 0)
  {
    g_pan_settle_timer = TIMER_DISABLE;
    g_pan_repeat_count = 0;
    capture_set_vertical_parameters();
    update_sample_rate();
    refresh_view();
  }

  draw_trace();

  // Sweeps no longer touch the panel area, so a redraw is only needed when
  // the decoded content itself changed — and can happen mid-sweep
  if (g_decode_mode && g_decode_panel_pending)
    draw_decode_panel();

  if (g_fft_mode && g_fft_panel_on && g_fft_panel_pending)
    draw_fft_panel();

  if (CAPTURE_STATE_WAIT == capture_get_state())
  {
    if (g_state_timer == TIMER_DISABLE)
    {
      g_state_timer = WAIT_STATE_HOLDOFF;
    }
    else if (g_state_timer == 0)
    {
      g_state_timer = TIMER_DISABLE;
      draw_capture_state();
    }
  }
  else
  {
    g_state_timer = TIMER_DISABLE;
    draw_capture_state();
  }

  if (g_toast_active)
  {
    if (g_toast_timer == 0)
    {
      g_toast_timer = TIMER_DISABLE;
      g_toast_active = false;

      // Every status-line element returns early while a toast is up, so the
      // bottom of the screen has been frozen for as long as it was shown:
      // rebuild all of it, panel included
      draw_status_line();
      mpanel_invalidate();
    }
  }

  mpanel_set_active(config.measure_display && !g_fft_mode &&
      !g_calibration_mode && config.measure_panel_mode == 0);

  // measure_display can also be flipped from the system menu, which does not
  // know about our timer: keep the timer state in sync here
  if (config.measure_display && TIMER_DISABLE == g_measure_timer)
    g_measure_timer = 0;
  else if (!config.measure_display && TIMER_DISABLE != g_measure_timer)
    g_measure_timer = TIMER_DISABLE;

  if (config.measure_display && !g_fft_mode)
  {
    if (g_measure_timer == 0)
    {
      g_measure_timer = MEASURE_UPDATE_TIMEOUT;

      if (g_mpanel_active)
      {
        mpanel_update();
      }
      else if (!g_toast_active)
      {
        if (++g_measure_page_tick >= MEASURE_PAGE_TICKS)
        {
          g_measure_page_tick = 0;
          g_measure_page = (g_measure_page + 1) % MEASURE_PAGES;
          // Page layouts differ; wipe the slot so no stale glyphs survive
          lcd_fill_rect(140, STATUS_LINE_Y, GRID_RIGHT + 1 - 140, STATUS_LINE_HEIGHT, BG_COLOR);
        }

        draw_measure();
      }
    }
  }
}

