/*
 * Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
 * Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
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
#include <limits.h>
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
#include "alias.h"
#include "classify.h"
#include "logic_decode.h"
#include "trend.h"
#include "scope.h"
#include "scope_internal.h"

/*- Variables: the screen geometry ------------------------------------------*/
/*
 * Normal: what this instrument has always looked like. 8 divisions down; the
 * settings and the two readings share the bottom bar, the trigger mode and the
 * capture state sit in the top one beside the record map.
 *
 * Large: what only the READINGS being big looks like, which is the arrangement
 * the stock firmware uses and the reason it is legible at arm's length:
 *
 *   - every SETTING moves into the top bar as its own compact field: the probe
 *     ratio, volts per division, the capture state, time per division, the
 *     coupling, the trigger mode. Still 8x16. They are what you set, not what
 *     you read, and a word like AUTO does not become more informative at 32 px -
 *     it just eats the room the numbers need. (That was the first attempt: the
 *     state words were doubled and the settings left small. It read as a size
 *     applied to whatever happened to have room.)
 *   - the bottom bar holds nothing but the two readings, and inside a reading
 *     only the DIGITS are doubled - the name before them and the unit after
 *     stay 8x16, exactly as "UPP 7.84 U" does on the stock display. A padded
 *     unit at 16 px costs 24 px that the number wants.
 *   - which leaves the graticule 175 px, seven divisions instead of eight. ONE
 *     division, where doubling everything cost two.
 *
 * The record map is the one thing that does not fit: the settings row needs the
 * width it was using. mv_y = -1 turns it off, and draw_miniview returns.
 */
static const ScopeGeom g_geom_normal =
{
  .grid_top = 20, .grid_h = 200, .status_y = 223, .status_h = 16,
  .status_row2 = 0,
  .slot0_x = 140, .slot1_x = 228, .slot_w = 88,
  .vscale_x = 10, .vscale_y = 223,
  .acdc_x = 54, .acdc_y = 223,
  .hscale_x = 82, .hscale_y = 223,
  .mode_x = 10, .mode_y = 4,
  .state_x = 46, .state_y = 4,
  .mv_y = 1, .bar_scale = 1,
};

static const ScopeGeom g_geom_large =
{
  .grid_top = 20, .grid_h = 175, .status_y = 202, .status_h = 44,
  .status_row2 = 16,
  .slot0_x = 2, .slot1_x = 162, .slot_w = 158,
  .vscale_x = 30, .vscale_y = 2,
  .acdc_x = 176, .acdc_y = 2,
  .hscale_x = 126, .hscale_y = 2,
  .mode_x = 208, .mode_y = 2,
  .state_x = 88, .state_y = 2,
  .mv_y = -1, .bar_scale = 2,
};

ScopeGeom g_geom =
{
  .grid_top = 20, .grid_h = 200, .status_y = 223, .status_h = 16,
  .slot0_x = 140, .slot1_x = 228, .slot_w = 88,
  .vscale_x = 10, .vscale_y = 223,
  .acdc_x = 54, .acdc_y = 223,
  .hscale_x = 82, .hscale_y = 223,
  .mode_x = 10, .mode_y = 4,
  .state_x = 46, .state_y = 4,
  .mv_y = 1, .bar_scale = 1,
};


/*- Variables ---------------------------------------------------------------*/
// Five distinct grid column patterns; g_grid_index picks one per column
// (a byte index table instead of 300 pointers saves ~1.3 KB of scarce TCM)
static uint16_t g_grid_column_0[GRID_HEIGHT_MAX];
static uint16_t g_grid_column_1[GRID_HEIGHT_MAX];
static uint16_t g_grid_column_2[GRID_HEIGHT_MAX];
static uint16_t g_grid_column_3[GRID_HEIGHT_MAX];
static uint16_t g_grid_column_4[GRID_HEIGHT_MAX];

static uint16_t *const g_grid_columns[5] =
{
  g_grid_column_0, g_grid_column_1, g_grid_column_2,
  g_grid_column_3, g_grid_column_4,
};

static uint8_t g_grid_index[GRID_WIDTH];

DataBuffer g_data_buffer;
DisplayBuffer g_display_buffer;

int g_trace_column = (GRID_WIDTH-1);

// Shadow of what each trace column currently shows on the LCD. Columns whose
// data and overlay did not change since the last sweep are skipped entirely,
// so a static trace costs ~300 compares instead of ~11 ms of LCD writes.
static uint8_t g_shadow_min[GRID_WIDTH];
static uint8_t g_shadow_max[GRID_WIDTH];
static uint8_t g_shadow_flags[GRID_WIDTH];
bool g_shadow_valid = false;
static int g_shadow_marker_px = 0x7fffffff;
bool g_sweep_force = true;
// Set when the record has fewer than RECON_MIN_SAMPLES samples per period, so
// the reconstruction drawn between them is past what it can honestly do.
// Recomputed once per display rebuild, read by every column.
static bool g_recon_strained = false;
#define RECON_MIN_SAMPLES 3


// Persistence and averaging state, all in display (column) space: persist is
// the accumulated envelope, avg the per-column EMA of the trace midpoint in
// px*64 fixed point. Both forget everything on any pan/zoom/scale change -
// the column-to-time mapping they accumulated under is gone.
static uint8_t  g_persist_min[GRID_WIDTH];
static uint8_t  g_persist_max[GRID_WIDTH];
// How brightly this column's envelope still glows, 0 = nothing there. Was a
// plain have/have-not flag; a byte was already being spent on it, so the
// decay mode costs no RAM at all - infinite persistence simply pins it full.
static uint8_t  g_persist_lvl[GRID_WIDTH];
static uint16_t g_persist_ramp[PERSIST_RAMP_STEPS];
static uint32_t g_persist_stamp;
static uint16_t g_avg_acc[GRID_WIDTH];
static uint8_t  g_avg_have[GRID_WIDTH];
static uint32_t g_avg_gen;

// Measurement cursors: SAVE cycles Off -> T1 -> T2 -> V1 -> V2 -> Off. Time
// cursors live in trigger-relative ns and voltage cursors in absolute mV, so
// both stay glued to the signal through pan and zoom exactly like the
// trigger marker does.
int     g_cursor_sel = 0;
static int64_t g_cursor_t[2];
static int     g_cursor_v[2];
static bool    g_cursor_dirty = false;

// Trend view (SHIFT+SAVE): the 1 Hz logger runs regardless; the view plots
// one metric of its ring. EDGE cycles the metric, MODE clears the ring.
bool g_trend_mode = false;

// Roll view: no button turns it on, the timebase does. Past the point where a
// triggered sweep would leave the screen unchanged for seconds at a time, the
// trace becomes a strip chart - the input's peak envelope over one column's
// worth of time, appended at the right, everything else one column older.
//
// The history is kept here rather than in g_display_buffer, and the 900 bytes
// of TCM that costs buy exactly one thing: the strip survives a look at
// something else. The spectrum, the trend and the decoder all own the trace
// columns and write their own picture into them, so a plot that lived there
// was gone the moment any of them was opened - ten minutes of recording lost
// to a glance at the spectrum and back.
//
// Rows, not millivolts, because that is what a column of the display buffer
// is and this is copied straight into one. What that costs is spelled out in
// roll_rescale(), which is why the vertical keys remap the stored rows
// instead of dropping them.
uint8_t g_roll_row_min[GRID_WIDTH];
uint8_t g_roll_row_max[GRID_WIDTH];
uint8_t g_roll_row_flags[GRID_WIDTH];

bool g_roll_active = false;

static int cursor_t_col(int64_t t_ns);
static int cursor_v_row(int mv);

// One Vpp/DC toggle per hold of the AC/DC key, not one per auto-repeat tick
static bool g_acdc_held = false;


// Not static: the scope's menu toggles it directly (see scope_menu.c). The
// full calibration UI has been in here since the base project, but nothing
// could ever reach it — scope_init() is only ever called with false.
bool scope_calibration_mode = false;
bool g_calibration_dual_channel = false;

bool g_line_owner = false; // measurements hold the status line's end

bool g_fft_mode = false;


                                   // frequencies and the hold restarts

bool g_autoset_active = false;
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


// The linearity sweep's accumulators. Sums rather than a table of points: the
// fit is three coefficients, and three coefficients need six sums, not 33
// samples kept around in a TCM that has none to spare.

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


/*
 * Measurements panel state: what the band SAYS, as characters.
 *
 * It used to be a 1bpp mask of the whole band - one bit per trace-area pixel,
 * ~1 KB of it - rebuilt whenever the text changed and read a bit at a time by
 * the column sweep. Storing the text instead and picking the glyph column out
 * of the font while compositing costs the same arithmetic per column, drops the
 * kilobyte, and takes the panel's height out of a struct size: a taller band
 * for the 8x16 font is free, where a taller mask was not (see the linker
 * script's TCM assert).
 */

//-----------------------------------------------------------------------------
// The panel's font and the geometry that follows from it. Everything that
// touches the band asks these rather than reading a constant: the band is
// taller in the 8x16 font, and what stacks on top of it (the decoder's byte
// strip) has to follow.
const Font *mpanel_font(void)
{
  return (PANEL_FONT_SMALL == config.measure_panel_font) ?
      FONT_SMALL : FONT_LARGE;
}

// ...and how many times over each of its pixels is drawn. There is no 16x32 font
// in the image, so the biggest band is the 8x16 one doubled - see PANEL_FONT_HUGE.
int mpanel_scale(void)
{
  return (PANEL_FONT_HUGE == config.measure_panel_font) ? 2 : 1;
}

// One glyph cell as drawn, which is what the band measures itself in
static int mpanel_cell_w(void)
{
  return mpanel_font()->width * mpanel_scale();
}

// How many characters of the chosen font fit across the band: 48 in the 6x8,
// 36 in the 8x16, 18 doubled
int mpanel_chars(void)
{
  return (GRID_WIDTH - 2 * MPANEL_PAD_X) / mpanel_cell_w();
}

int mpanel_row_h(void)
{
  return mpanel_font()->height * mpanel_scale() + MPANEL_GAP_Y;
}

// Two rows of readings, or one at the doubled size: two would be 72 px of the
// 200 the trace area has, and a band that owns a third of the screen is not a
// band any more. One row of 16x32 is 36 px - the same share the 8x16 band has.
int mpanel_rows(void)
{
  return (PANEL_FONT_HUGE == config.measure_panel_font) ? 1 : MPANEL_ROWS;
}

// Whether the readings are in the band at the bottom or placed as widgets.
// Free text always takes the band: it is borrowed to say something, and a
// sentence has nowhere else to go.
static bool mpanel_band_mode(void)
{
  return g_mpanel_is_text ||
      PANEL_LAYOUT_WIDGETS != config.measure_layout_mode;
}

// 24 px in the small font, 40 in the large: two rows of glyphs, the gap between
// them, and a margin above and below. Zero in the widget layout, where there is
// no band at all - which is also what gives the decoder's byte strip the whole
// height back (DBAND_BOTTOM follows this).
static int mpanel_h(void)
{
  if (!mpanel_band_mode())
    return 0;

  return 2 * MPANEL_PAD_Y + mpanel_rows() * mpanel_row_h() - MPANEL_GAP_Y;
}

int mpanel_row0(void)
{
  return GRID_HEIGHT - 1 - mpanel_h();
}


_Static_assert(sizeof(LogicScratch) <= CAPTURE_SPARE_RAM_SIZE,
    "decoder scratch must fit the spare SRAM block");


// ...and on the trace itself, in a band the sweep composites like the
// measurements panel: where each byte begins and ends, and its value written
// in where the byte is wide enough to hold the text. A byte list tells you
// what was said; this tells you where on the waveform it was said.
//
// Two rows where the zoom allows it - the number, and under it what the
// number means as text. Side by side in one row they run together into
// something that has to be parsed rather than read.
//
// The band is bottom-aligned and only as tall as it needs to be: one row
// where the bytes are too narrow to name, none at all where they are too
// narrow even for the number. A band of empty pixels is trace it has taken
// away for nothing.

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
static bool trace_ready(void)
{
  return (g_trace_column == (GRID_WIDTH-1));
}

//-----------------------------------------------------------------------------
void redraw_trace(void)
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
// The phosphor ramp: PERSIST_GLOW_COLOR down to black, in equal steps.
// Built rather than written out because it is indexed by a brightness that
// falls linearly with time, so the steps have to be linear too - a hand
// picked set of dim yellows would decay in visible jumps.
static void persist_build_ramp(void)
{
  for (int i = 0; i < PERSIST_RAMP_STEPS; i++)
  {
    // i = 0 is the last step before nothing at all, so it is not black: a
    // glow that reaches zero is dropped, not drawn
    int n = i + 1;

    g_persist_ramp[i] = LCD_COLOR(PERSIST_GLOW_R * n / PERSIST_RAMP_STEPS,
        PERSIST_GLOW_G * n / PERSIST_RAMP_STEPS,
        PERSIST_GLOW_B * n / PERSIST_RAMP_STEPS);
  }
}

//-----------------------------------------------------------------------------
// One edge of the decaying envelope, in this column, drawn as a piece of a
// CURVE rather than as a dot.
//
// The two edges are where the beam has been, so they are what a phosphor
// leaves behind - filling everything between them turns a trail into a solid
// olive polygon, which is what the first version did and what nobody wants to
// look at. But a single pixel per column is not a curve either: on a steep
// edge the envelope's top moves a long way from one column to the next, and
// the marks come out as a row of dots with gaps between them.
//
// So each column reaches halfway to whichever neighbours still glow - the
// same halves-meet-in-the-middle trick close_gaps() plays on the trace, and
// it makes the two ends of the join land on the same pixel.
static void persist_edge(uint16_t *column, const uint8_t *edge, int c,
    uint16_t glow)
{
  int top = edge[c], bot = edge[c];

  for (int n = -1; n <= 1; n += 2)
  {
    int i = c + n, mid;

    if (i < 0 || i >= GRID_WIDTH || !g_persist_lvl[i])
      continue; // nothing glowing there to join up with

    mid = (edge[c] + edge[i]) / 2;

    if (mid < top)
      top = mid;

    if (mid > bot)
      bot = mid;
  }

  for (int y = top; y <= bot; y++)
    column[y] = glow;
}

//-----------------------------------------------------------------------------
static void update_from_display_buffer(uint16_t *column, DisplayBuffer *db)
{
  bool clip_h = db->flags[g_trace_column] & SAMPLE_FLAG_CLIP_H;
  bool clip_l = db->flags[g_trace_column] & SAMPLE_FLAG_CLIP_L;
  int color;

  // The accumulated envelope first, so the live trace paints over it. This
  // is how a runt that fired once stays visible: the envelope holds every
  // level any frame ever reached in this column.
  if (config.persist_mode != PERSIST_OFF && g_persist_lvl[g_trace_column])
  {
    int c = g_trace_column;

    if (config.persist_mode == PERSIST_DECAY)
    {
      // The trail: two fading curves, the highest and the lowest the beam
      // has lately been. Not the area between them - see persist_edge.
      uint16_t glow = g_persist_ramp[g_persist_lvl[c] * PERSIST_RAMP_STEPS / 256];

      persist_edge(column, g_persist_min, c, glow);
      persist_edge(column, g_persist_max, c, glow);
    }
    else
    {
      // Infinite is a different question and keeps its answer: everywhere
      // the signal has EVER been, as an area, so a runt that fired once is
      // a mark you cannot miss rather than a hairline
      for (int y = g_persist_min[c]; y <= g_persist_max[c]; y++)
        column[y] = TRACE_PERSIST_COLOR;
    }
  }

  if (db->flags[g_trace_column] & SAMPLE_FLAG_VALID)
  {
    if (clip_h || clip_l)
      color = TRACE_CLIP_COLOR;
    else if (db->flags[g_trace_column] & SAMPLE_FLAG_FILLED)
      color = g_recon_strained ? TRACE_RECON_WARN_COLOR : TRACE_FILLED_COLOR;
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
// One character column of one glyph into the column buffer: `bit` says which of
// the glyph's columns this is, and the caller has already worked out that this
// trace column falls inside the character. Anything outside the font - the
// half-space the value formatters use before a unit - paints nothing, which is
// what a blank is: the layout has already reserved its width.
/*
 * The three sizes a placed reading comes in, out of the two bits its flags keep
 * them in: the 6x8 font, the 8x16 font, and the 8x16 font doubled to 16x32.
 *
 * A 16x32 reading is four glyphs' worth of screen for one, and it exists for one
 * reason: it is bigger than anything the top and bottom bars can hold. Those are
 * 20 and 16 pixels tall and packed end to end, so 8x16 is their ceiling forever -
 * and the trace area is 300x200 with a compositor that can put text anywhere in
 * it. "The bar values are too small" is answered here, not in the bars.
 */
int widget_size_of(uint8_t flags)
{
  if (flags & PW_HUGE)
    return 2;

  return (flags & PW_LARGE) ? 1 : 0;
}

const Font *widget_font(int size)
{
  return size ? FONT_LARGE : FONT_SMALL;
}

int widget_scale(int size)
{
  return (size > 1) ? 2 : 1;
}

/*
 * Vertical distances are stored against the BIGGEST trace area there is, not
 * against whatever one is on screen, and these two are the conversion.
 *
 * The area is 200 px at the normal text size and 175 at the large one, so a
 * distance kept in plain pixels means a different fraction of the screen in each
 * - and a layout arranged in one of them arrives in the other with its spacing
 * wrong. What that looks like is two readings, one measured from the top and one
 * from the bottom, closing 25 px on each other and ending up on top of one
 * another: the pair did not move WITH the area, so the area moved through them.
 *
 * Normalised, the pair keeps its share of the height and its shape. Layouts
 * saved before this need no migration: they were written on the 200 px area,
 * which is exactly what they are now read against.
 *
 * Horizontally there is nothing to do - the trace area is 300 px wide at every
 * size, and the bars stretch rather than the graticule.
 */
static int widget_y_px(int stored)
{
  return stored * PANEL_WIDGET_STEP * GRID_HEIGHT / GRID_HEIGHT_MAX;
}

static int widget_y_store(int px)
{
  return px * GRID_HEIGHT_MAX / GRID_HEIGHT / PANEL_WIDGET_STEP;
}

//-----------------------------------------------------------------------------
// Where a stored widget actually IS, given how big its text came out.
//
// The stored position is a distance from a CORNER (see PanelWidget), so this is
// where the two meet: the size is known here and nowhere else, and a right- or
// bottom-anchored reading is placed by its own right or bottom edge. Which is
// what keeps a reading that grew from moving, and a reading in the bottom right
// corner from growing off the screen. Clamped, because a layout arranged at one
// size and read back at another can name a position that no longer exists.
void widget_pos(const PanelWidget *w, int wide, int gh, int *px, int *py)
{
  int max_x = GRID_WIDTH - 2 - wide;
  int max_y = GRID_HEIGHT - 2 - gh;
  int x = (w->flags & PW_ANCHOR_RIGHT) ?
      (GRID_WIDTH - 2 - wide - w->x * PANEL_WIDGET_STEP) :
      (w->x * PANEL_WIDGET_STEP);
  int y = (w->flags & PW_ANCHOR_BOTTOM) ?
      (GRID_HEIGHT - 2 - gh - widget_y_px(w->y)) :
      widget_y_px(w->y);

  *px = (x < 0) ? 0 : ((x > max_x) ? ((max_x < 0) ? 0 : max_x) : x);
  *py = (y < 0) ? 0 : ((y > max_y) ? ((max_y < 0) ? 0 : max_y) : y);
}

//-----------------------------------------------------------------------------
// ...and the way back: store an absolute top-left, measured from whichever
// corner the reading is nearest. Nothing asks the user which corner - the corner
// is where they put it, so it follows from the position on every move.
void widget_set_pos(PanelWidget *w, int x, int y, int wide, int gh)
{
  bool right  = (x + wide / 2) > GRID_WIDTH / 2;
  bool bottom = (y + gh / 2) > GRID_HEIGHT / 2;
  int dx = right ? (GRID_WIDTH - 2 - wide - x) : x;
  int dy = bottom ? (GRID_HEIGHT - 2 - gh - y) : y;

  if (dx < 0)
    dx = 0;
  if (dy < 0)
    dy = 0;

  w->flags &= (uint8_t)~PW_ANCHOR_MASK;
  w->flags |= (uint8_t)((right ? PW_ANCHOR_RIGHT : 0) |
      (bottom ? PW_ANCHOR_BOTTOM : 0));
  w->x = (uint8_t)(dx / PANEL_WIDGET_STEP);
  w->y = (uint8_t)widget_y_store(dy);
}

//-----------------------------------------------------------------------------
// `scale` repeats each pixel row that many times, which is how a widget gets a
// 16x32 glyph out of an 8x16 font. Doubling and not a second font: there is no
// 16x32 font in the image and one would cost 3 KB of flash for the ninety-five
// characters, where this costs a multiply. It is a chunky glyph read from a
// bench, not typography - and it is the only size the two bars cannot hold, so
// it is the whole reason the readings can leave them.
__attribute__((noinline))
static void mpanel_glyph_column(uint16_t *column, int row0, const Font *font,
    char ch, int bit, uint16_t color, int scale)
{
  const uint8_t *bitmap;

  if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
    return;

  bitmap = font->data + (ch - FONT_FIRST_CHAR) * font->pitch;

  for (int y = 0; y < font->height; y++)
  {
    int i = y * font->width + bit;

    if (!((bitmap[i / 8] >> (i % 8)) & 1))
      continue;

    for (int s = 0; s < scale; s++)
      column[row0 + y * scale + s] = color;
  }
}

//-----------------------------------------------------------------------------
// Everything the band says, for ONE trace column. Two shapes: the grid of
// reading cells, or two lines of free text. Both are stored as characters and
// turned into pixels here, inside the column buffer, so the panel is composed
// by the same sweep that draws the trace and can never flicker against it.
typedef struct
{
  char text[MPANEL_CHARS_MAX + 1]; // "name value", nothing else
  uint8_t len;
  uint8_t label_len;               // leading characters drawn dimmer: the name
  uint8_t x;                       // first character column of the band's row
  uint8_t row;
  uint16_t color;
} MPanelCell;

bool g_mpanel_active = false;
static bool g_mpanel_force = false;   // rebuild the panel text on the next tick
static uint32_t g_mpanel_builds = 0;  // texts built / bands repainted, for
static uint32_t g_mpanel_paints = 0;  // the System Info diagnostic
static MPanelCell g_mpanel_cell[MPANEL_CELLS_MAX];
static int g_mpanel_cells = 0;
// ...or two lines of free text instead of the grid, for the things the band is
// borrowed to SAY rather than to measure (calibration hints, the
// auto-calibration's report). Sentences, so they keep the small font.
static char g_mpanel_text[2][MPANEL_TEXT_MAX + 2];
bool g_mpanel_is_text = false;

/*
 * ...or readings placed where the user put them, which is the other layout.
 *
 * Same idea as a band cell and the same painter, with the position in pixels
 * instead of character columns and the font per widget rather than per panel.
 * Built from config.measure_widget[] at the same 2 Hz the band is.
 */

PanelPlaced g_placed[PANEL_WIDGETS_MAX];
int g_placed_n = 0;

// The layout editor. It owns the screen and the keyboard while it is up: the
// trace is a mock waveform and the readings are mock values, so that arranging
// them is not a moving target and every one of them has something to show
// whatever is on the probe.
bool g_layout_edit = false;

// Logic decoder view; the run tables live in the spare main-SRAM block
bool g_decode_mode = false;
bool g_logic_have = false;
// The frames on the panel are older than the record on the screen: kept
// because the records since then had nothing in them (see decode_update)
bool g_decode_held = false;
LogicResult g_logic;
int g_decode_sel = 0;              // selected byte (jump target)
int g_decode_size = 0;             // record length the decode ran on

// UART rate table shared with the decoder's menu entry. Index 0 is auto: the
// decoder works the rate out of the record, which is right nearly always and
// wrong exactly when the record holds too few frames to pin it down.
const char *const decoder_baud_labels[DECODER_BAUD_COUNT] =
{
  "Auto", "9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600",
};

const int decoder_baud_values[DECODER_BAUD_COUNT] =
{
  0, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
};

//-----------------------------------------------------------------------------
// The placed readings, for one trace column: each widget's own background over
// its own box, then its text. Same painter as the band, and for the same
// reason - a reading composited by the sweep cannot flicker against the trace.
//
// The selection frame belongs to the editor and is drawn here too, because here
// is where this column's pixels are.
__attribute__((noinline))
static void widgets_paint_column(int c, uint16_t *column)
{
  for (int i = 0; i < g_placed_n; i++)
  {
    const PanelPlaced *w = &g_placed[i];
    const Font *font = widget_font(w->size);
    int sc = widget_scale(w->size);
    int gw = font->width * sc;      // one glyph cell, as drawn
    int gh = font->height * sc;
    int at = c - w->x;
    int wide = w->len * gw;
    bool sel = g_layout_edit && i == g_layout_sel;
    int ch_i, bit;
    uint16_t color;

    // One pixel of air around the text, which is also what the frame and the
    // solid background cover
    if (at < -1 || at > wide)
      continue;

    for (int y = w->y - 1; y <= w->y + gh; y++)
    {
      if (y < 0 || y >= GRID_HEIGHT - 1)
        continue;

      if (PANEL_BG_SOLID == config.measure_panel_bg || sel)
        column[y] = BG_COLOR;
      else if (PANEL_BG_OFF != config.measure_panel_bg)
        column[y] = MPANEL_DIM(column[y]);
    }

    // The frame: the box's edge, so a grabbed widget reads as held
    if (sel)
    {
      uint16_t edge = g_layout_grab ? MEASURE_VOLTAGE_COLOR : MPANEL_EDGE_COLOR;
      int top = w->y - 1;
      int bot = w->y + gh;

      if (-1 == at || at == wide)
      {
        for (int y = top; y <= bot; y++)
        {
          if (y >= 0 && y < GRID_HEIGHT - 1)
            column[y] = edge;
        }
      }
      else
      {
        if (top >= 0)
          column[top] = edge;

        if (bot < GRID_HEIGHT - 1)
          column[bot] = edge;
      }
    }

    if (at < 0 || at >= wide)
      continue;

    ch_i = at / gw;
    bit = (at % gw) / sc;
    color = (ch_i < w->label_len) ? MPANEL_DIM(w->color) : w->color;

    mpanel_glyph_column(column, w->y, font, w->text[ch_i], bit, color, sc);
  }
}

__attribute__((noinline))
static void mpanel_paint_column(int c, uint16_t *column)
{
  int row0 = mpanel_row0() + MPANEL_PAD_Y;
  int x = c - MPANEL_PAD_X;

  if (x < 0)
    return;

  if (g_mpanel_is_text)
  {
    const Font *font = FONT_SMALL;
    int ch_i = x / font->width;
    int bit = x % font->width;

    if (ch_i >= MPANEL_TEXT_MAX)
      return;

    for (int r = 0; r < 2; r++)
      mpanel_glyph_column(column, row0 + r * (font->height + MPANEL_GAP_Y),
          font, g_mpanel_text[r][ch_i], bit, LCD_WHITE_COLOR, 1);

    return;
  }

  {
    const Font *font = mpanel_font();
    int sc = mpanel_scale();
    int cw = font->width * sc;
    int ch_i = x / cw;
    int bit = (x % cw) / sc;

    // At most one pair per row covers this character, and there are never more
    // than a handful of them - a scan is cheaper than a map to look it up in
    for (int i = 0; i < g_mpanel_cells; i++)
    {
      const MPanelCell *cell = &g_mpanel_cell[i];
      int at = ch_i - cell->x;
      uint16_t color;

      if (at < 0 || at >= cell->len)
        continue;

      // The name dimmer than the number it names: the reading is what the eye
      // should land on, and a label at full brightness competes with it
      color = (at < cell->label_len) ? MPANEL_DIM(cell->color) : cell->color;

      mpanel_glyph_column(column, row0 + cell->row * mpanel_row_h(), font,
          cell->text[at], bit, color, sc);
    }
  }
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
void build_trace_column(int c, uint16_t *column)
{
  // The background the rest of this composites over: this column's share of
  // the graticule, or pattern 0 - the one with no ruling in it, which every
  // column between the divisions already uses - when the grid is switched off.
  // Nothing below has to know which it got, and a blank screen costs exactly
  // what a ruled one does.
  const uint16_t *grid = g_grid_columns[config.grid_mode ? 0 : g_grid_index[c]];

  for (int i = 0; i < GRID_HEIGHT; i++)
    column[i] = grid[i];

  if (g_fft_mode)
    update_from_spectrum(column, &g_display_buffer);
  else
    update_from_display_buffer(column, &g_display_buffer);

  // The trigger-position overlay is meaningless on the spectrum, the trend
  // and the roll strip - none of them is drawn around a trigger
  if (!g_fft_mode && !g_trend_mode && !g_roll_active)
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

  // Measurement cursors ride the sweep like every other overlay: the pair of
  // time hairlines and the pair of voltage lines, the active one solid, its
  // partner dashed
  if (g_cursor_sel && !g_fft_mode && !g_trend_mode && !g_roll_active)
  {
    for (int k = 0; k < 2; k++)
    {
      int row = cursor_v_row(g_cursor_v[k]);

      if ((g_cursor_sel == 3 + k) || (c & 3) < 2)
        column[row] = CURSOR_V_COLOR;
    }

    for (int k = 0; k < 2; k++)
    {
      if (c == cursor_t_col(g_cursor_t[k]))
      {
        bool solid = (g_cursor_sel == 1 + k);

        for (int y = 0; y < GRID_HEIGHT-1; y++)
        {
          if (solid || (y & 3) < 2)
            column[y] = CURSOR_T_COLOR;
        }
      }
    }
  }

  // Bit grid: a hairline straight across at every bit boundary, and the bit's
  // number in the cell. Over everything, waveform included - a boundary is a
  // fact about that column, and a line broken wherever the trace crosses it
  // is a line the eye cannot follow to the edge it is meant to line up with.
  if (g_dbit_on && g_dbit_bot >= g_dbit_top)
  {
    if ((DBIT->edge[c / 8] >> (c % 8)) & 1)
    {
      for (int y = g_dbit_top; y <= g_dbit_bot; y++)
        column[y] = DBIT_BLEND(column[y], DBIT_TICK_COLOR);
    }

    for (int y = 0; y < 8; y++)
    {
      if ((DBIT->glyph[c] >> y) & 1)
        column[g_dbit_text_row + y] = DBIT_TEXT_COLOR;
    }
  }

  // Decoder band: the same trick, a couple of rows of it. Under every column
  // that a decoded byte covers the trace is dimmed and tinted in the byte's
  // colour, the column where a frame starts gets a full-height tick, and the
  // value is written in wherever it fits.
  //
  // In two steps, and the order is the whole point: everything that marks
  // where a byte IS goes down first, and the text goes over it. A tick and a
  // glyph want the same pixel often enough that whichever is written second
  // wins, and the one worth keeping is never the tick.
  if (g_dband_rows && g_dband_byte[c] >= 0)
  {
    int b = g_dband_byte[c];
    bool grouped = (g_dband_sel_len > 1 && b >= g_dband_sel_start &&
        b < g_dband_sel_start + g_dband_sel_len);
    uint16_t tint = (b == g_decode_sel) ? DSTRIP_SEL :
        grouped ? DSTRIP_GROUP :
        ((b & 1) ? DSTRIP_ODD : DSTRIP_EVEN);
    bool edge = (g_dband_edge[c / 8] >> (c % 8)) & 1;
    bool gedge = (g_dband_gedge[c / 8] >> (c % 8)) & 1;
    bool gap = (g_dband_gap[c / 8] >> (c % 8)) & 1;

    for (int y = 0; y < g_dband_rows; y++)
    {
      int row = g_dband_row0 + y;

      // A filled gap is the group's business alone. Above the group's row the
      // column is left exactly as it was before anything was filled - no byte
      // covers it, so no byte claims it.
      if (gap && y < g_dband_group_y)
        continue;
      // Byte boundaries divide the rows that show bytes; the row that shows
      // what a whole group came to is divided by the group's edges alone
      bool tick = (y < g_dband_group_y) ? edge : gedge;

      if (tick || y == g_dband_rows - 1)
        column[row] = tint;                    // boundary / baseline
      else
        column[row] = MPANEL_DIM(column[row]); // room for the text to read

      // The colour itself, beside the hex that names it
      if (dband_sw_at(c) && y >= g_dband_group_y &&
          y < g_dband_group_y + DBAND_SW_H)
      {
        bool frame = !dband_sw_at(c - 1) || !dband_sw_at(c + 1) ||
            y == g_dband_group_y || y == g_dband_group_y + DBAND_SW_H - 1;
        uint16_t swc;

        if (frame || !decode_group_color(b, &swc))
          swc = DBAND_SW_EDGE;

        column[row] = swc;
      }
    }
  }

  // ...and the text over all of it, whether or not a byte covers this column.
  // Bytes do not touch: a WS2812 byte ends on its last bit's high half and
  // the next begins after the low tail, so a group's value - written once
  // across every byte it took - runs over columns no byte covers. Gating the
  // whole band on coverage took the glyphs in those columns out with the
  // tint, and the value came out sliced at each byte boundary.
  if (g_dband_rows)
  {
    for (int y = 0; y < g_dband_rows; y++)
    {
      if (g_dband_mask[y][c / 8] & (1 << (c % 8)))
        column[g_dband_row0 + y] = LCD_WHITE_COLOR;
    }
  }

  // The measurements panel: its background, then its text, all inside the
  // column buffer, so the panel is part of the normal sweep and can never
  // flicker or be painted over.
  //
  // Which background is the user's call. Dimming the trace keeps the waveform
  // visible through the readings and is what this has always done; a solid band
  // is the one that stays legible over a bright, busy trace, which is exactly
  // when the numbers matter; and nothing at all is for reading the waveform
  // under them with the numbers still there.
  if (g_mpanel_active)
  {
    if (mpanel_band_mode())
    {
      int h = mpanel_h();
      int row0 = mpanel_row0();

      for (int y = 0; y < h; y++)
      {
        uint16_t *px = &column[row0 + y];

        if (PANEL_BG_SOLID == config.measure_panel_bg)
          *px = (0 == y) ? MPANEL_EDGE_COLOR : BG_COLOR;
        else if (PANEL_BG_OFF != config.measure_panel_bg)
          *px = MPANEL_DIM(*px);
      }

      mpanel_paint_column(c, column);
    }
    else
    {
      widgets_paint_column(c, column);
    }
  }
}

//-----------------------------------------------------------------------------
static void draw_trace(void)
{
  uint16_t column[GRID_HEIGHT_MAX];
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
  else if (g_snap_tag && c >= SNAP_TAG_COL0 && c <= SNAP_TAG_COL1)
    hole = SNAP_TAG_H;

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
// Compose one pair: the name, one space, the value. The value keeps the leading
// spaces its formatter padded it with - that padding is what holds a metric's
// width constant, and a constant width is why nothing in the band moves when a
// reading changes.
static void mpanel_cell_set(MPanelCell *cell, const char *label,
    const char *value, uint16_t color)
{
  int n = snprintf(cell->text, sizeof(cell->text), "%s%s%s",
      label, label[0] ? " " : "", value);

  if (n < 0)
    n = 0;
  else if (n > (int)sizeof(cell->text) - 1)
    n = (int)sizeof(cell->text) - 1;

  cell->len = (uint8_t)n;
  cell->label_len = (uint8_t)strlen(label);
  cell->color = color;
  cell->x = 0;
  cell->row = 0;
}

//-----------------------------------------------------------------------------
// Forget the panel's rendered state so the next update rebuilds the text and
// repaints the band even if the numbers happen to come out identical. Needed
// after anything that leaves the bottom of the screen in an unknown state:
// the panel is only repainted when its TEXT changes, so a band that was
// cleared by something else would otherwise stay blank indefinitely.
void mpanel_invalidate(void)
{
  g_mpanel_cells = -1;          // matches neither a grid nor free text
  g_mpanel_text[0][0] = '\x01'; // ...and never a freshly built line either
  g_mpanel_text[1][0] = 0;
  g_mpanel_force = true;        // and do it on the next tick, not in 500 ms

  if (config.measure_display)
    g_measure_timer = 0;
}

//-----------------------------------------------------------------------------
// Whether the translucent panel is the right place for the measurements: in
// panel view, with the measurements on, and only where the trace area is ours
// to composite into (the spectrum and the calibration screen draw their own)
bool mpanel_wanted(void)
{
  // The editor IS the readings on screen; without them there would be nothing
  // to arrange
  if (g_layout_edit)
    return true;

  // The calibration screen draws its own trace but goes through the same
  // sweep, so the band is available there too - and that is where a hint is
  // worth more than a measurement
  if (g_autocal_active)
    return true; // it is talking to the user through the band

  if (scope_calibration_mode)
    return g_calib_hint;

  return config.measure_display && !g_fft_mode && !g_trend_mode &&
      config.measure_panel_mode == 0;
}

//-----------------------------------------------------------------------------
void mpanel_set_active(bool active)
{
  if (active == g_mpanel_active)
    return;

  g_mpanel_active = active;
  mpanel_invalidate();
  overlay_repaint_region(mpanel_row0(), mpanel_h());
}

//-----------------------------------------------------------------------------
// Put fixed text in the panel instead of the reading grid. mpanel_update()
// builds cells out of what was measured; this is for text that is chosen rather
// than measured, like the calibration hint, and it rebuilds only when the text
// actually changes.
void mpanel_set_lines(const char *l0, const char *l1)
{
  if (g_mpanel_is_text &&
      0 == strcmp(l0, g_mpanel_text[0]) && 0 == strcmp(l1, g_mpanel_text[1]))
    return;

  strncpy(g_mpanel_text[0], l0, MPANEL_TEXT_MAX);
  strncpy(g_mpanel_text[1], l1, MPANEL_TEXT_MAX);
  g_mpanel_text[0][MPANEL_TEXT_MAX] = 0;
  g_mpanel_text[1][MPANEL_TEXT_MAX] = 0;
  g_mpanel_is_text = true;

  g_mpanel_builds++;
  g_mpanel_paints++;
  overlay_repaint_region(mpanel_row0(), mpanel_h());
}

//-----------------------------------------------------------------------------
// Build the placed readings from config.measure_widget[]. Slot for slot, not
// compacted: the editor selects by slot, and a hole in the middle of the array
// is a slot the user emptied rather than a reason to renumber the rest.
void widgets_update(const ScopeMeasure *sm)
{
  PanelPlaced next[PANEL_WIDGETS_MAX];
  static int heartbeat = 0;

  memset(next, 0, sizeof(next));

  for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
  {
    const PanelWidget *w = &config.measure_widget[i];
    PanelPlaced *p = &next[i];
    MeasureItem item;
    int len;

    if (MEASURE_NONE == w->metric || w->metric >= MEASURE_COUNT)
      continue;

    if (!measure_format(w->metric, sm, &item))
      continue;

    len = snprintf(p->text, sizeof(p->text), "%s %s", item.label, item.value);

    if (len < 0)
      len = 0;
    else if (len > (int)sizeof(p->text) - 1)
      len = (int)sizeof(p->text) - 1;

    p->len       = (uint8_t)len;
    p->label_len = (uint8_t)strlen(item.label);
    p->size      = (uint8_t)widget_size_of(w->flags);
    p->color     = measure_item_color(&item);

    // The text's own size decides where a corner-anchored reading starts, and
    // the text is only known now
    {
      const Font *font = widget_font(p->size);
      int sc = widget_scale(p->size);
      int x, y;

      widget_pos(w, len * font->width * sc, font->height * sc, &x, &y);
      p->x = (int16_t)x;
      p->y = (int16_t)y;
    }
  }

  if (g_placed_n == PANEL_WIDGETS_MAX &&
      0 == memcmp(next, g_placed, sizeof(next)))
  {
    // Same readings in the same places - but heal the area anyway every few
    // seconds, for the reason the band does (see mpanel_update)
    if (++heartbeat < MPANEL_HEARTBEAT)
      return;

    heartbeat = 0;
    g_mpanel_paints++;
    overlay_repaint_region(0, GRID_HEIGHT - 1);
    return;
  }

  heartbeat = 0;
  g_mpanel_builds++;
  memcpy(g_placed, next, sizeof(next));
  g_placed_n = PANEL_WIDGETS_MAX;

  // The whole trace area, because a widget can be anywhere in it. At 2 Hz that
  // is a fraction of what the sweep itself costs.
  g_mpanel_paints++;
  overlay_repaint_region(0, GRID_HEIGHT - 1);
}

//-----------------------------------------------------------------------------
// noinline deliberately: it is called once per scope tick from one place, and
// welded into scope_task() -O3 unrolls the metric scan and the cell building
// into three kilobytes of straight-line code. A call costs nothing at 2 Hz.
__attribute__((noinline))
static void mpanel_update(void)
{
  ScopeMeasure sm;
  MeasureItem items[MEASURE_ITEMS_MAX];
  MPanelCell next[MPANEL_CELLS_MAX];
  int n, cells = 0;
  static int throttle = 0;
  static int heartbeat = 0;

  // Live values jitter in their last digit; rebuilding the text (and
  // repainting the band) 10x a second serialized the main loop behind the
  // LCD bus and made buttons feel mushy. 2 Hz is plenty for reading.
  if (!g_mpanel_force && ++throttle < 5)
    return;

  throttle = 0;

  // Unthrottled: this runs at 2 Hz anyway, and the 10 Hz recompute throttle
  // behind capture_get_measurements() can only ever hand back an older frame.
  // The editor arranges against mock values instead - see layout_mock_measure.
  if (g_layout_edit)
    layout_mock_measure(&sm);
  else if (!capture_get_measurements_fresh(&sm))
    return;

  // Only now: nothing was rebuilt, so a request made before the first
  // acquisition landed - which is every entry into the scope - still stands.
  // Consuming it here used to cost the panel up to half a second of the
  // throttle before it first appeared.
  g_mpanel_force = false;

  // The other layout: readings where the user put them, not a band
  if (!mpanel_band_mode())
  {
    widgets_update(&sm);
    return;
  }

  n = measure_build_items(&sm, items);

  // memcmp decides below whether anything changed, so the holes the compiler
  // leaves between these fields have to be a known value
  memset(next, 0, sizeof(next));

  // Everything that fits - and if something does not, the last cell says how
  // many were left out instead of the band simply ending. Running out of room
  // silently reads exactly like "there is nothing more to show".
  {
    int chars = mpanel_chars();
    int x = 0, row = 0;

    for (int i = 0; i < n && cells < MPANEL_CELLS_MAX; i++)
    {
      MPanelCell *cell = &next[cells];

      mpanel_cell_set(cell, items[i].label, items[i].value,
          measure_item_color(&items[i]));

      // Wrap to the second row when this pair would run off the first, and
      // stop when there is no third row to wrap into
      if (x + cell->len > chars)
      {
        if (++row >= mpanel_rows())
          break;

        x = 0;
      }

      cell->x = (uint8_t)x;
      cell->row = (uint8_t)row;
      x += cell->len + MPANEL_GUTTER;
      cells++;
    }

    // What did not fit says so, rather than the band simply ending: running out
    // of room silently reads exactly like "there is nothing more to show". The
    // marker needs room of its own, so the last pair placed gives way to it.
    if (cells < n)
    {
      char more[12];
      int hidden = n - cells;
      int at = cells;

      snprintf(more, sizeof(more), "+%d more", hidden);

      // ...but it says it in two characters rather than eating the only reading
      // on the band. At 16x32 a row is eighteen characters and one reading fills
      // twelve of them, so "+4 more" does not fit and "+4" does - and a band
      // showing nothing but "+4 more" (which is what this did) has replaced the
      // measurement with a note about the measurements.
      if (x + (int)strlen(more) > chars)
        snprintf(more, sizeof(more), "+%d", hidden);

      if (x + (int)strlen(more) > chars && at > 0)
      {
        at--;                     // no room left: the marker takes its place
        x = next[at].x;
        row = next[at].row;
        hidden++;
        snprintf(more, sizeof(more), "+%d more", hidden);

        if (x + (int)strlen(more) > chars)
          snprintf(more, sizeof(more), "+%d", hidden);
      }
      else
      {
        row = (at > 0) ? next[at - 1].row : 0;
      }

      mpanel_cell_set(&next[at], "", more, MPANEL_NONE_COLOR);
      next[at].x = (uint8_t)x;
      next[at].row = (uint8_t)row;
      cells = at + 1;
    }
  }

  if (!g_mpanel_is_text && cells == g_mpanel_cells &&
      0 == memcmp(next, g_mpanel_cell, sizeof(next)))
  {
    // Same readings, so the band is still right — but repaint it every few
    // seconds anyway. The band lives inside the trace area, and anything
    // that paints there without going through build_trace_column() leaves a
    // hole the sweep will not heal (it skips columns whose trace data has
    // not changed). A steady signal is exactly when that hole would stay.
    if (++heartbeat < MPANEL_HEARTBEAT)
      return;

    heartbeat = 0;
    g_mpanel_paints++;
    overlay_repaint_region(mpanel_row0(), mpanel_h());
    return;
  }

  heartbeat = 0;
  g_mpanel_builds++;
  memcpy(g_mpanel_cell, next, sizeof(next));
  g_mpanel_cells = cells;
  g_mpanel_is_text = false;

  g_mpanel_paints++;
  overlay_repaint_region(mpanel_row0(), mpanel_h());
}

/*- The layout editor -------------------------------------------------------*/
// Both live further down, and the editor is here because it belongs beside the
// panel it edits
void refresh_view(void);

/*
 * A screen for arranging the readings, because a menu cannot do it: where a
 * number should sit is a question about the picture, and a list of coordinates
 * is not a picture. So the trace area stays the trace area, the readings stay
 * on it, and the arrows move them.
 *
 * Two things are mocked while it is up. The trace is a synthetic square wave,
 * and the readings are the fixed set in layout_mock_measure(): a layout arranged
 * against numbers that jump - or against a probe with nothing on it, where half
 * the readings are grey dashes - is a layout arranged blind. What is NOT mocked
 * is the drawing: the widgets go through the same column compositor they will be
 * drawn by afterwards, so the editor cannot flatter the result.
 */

//-----------------------------------------------------------------------------
// The selected slot, and the moves the arrows make on it
PanelWidget *layout_selected(void)
{
  if (g_layout_sel < 0 || g_layout_sel >= PANEL_WIDGETS_MAX)
    g_layout_sel = 0;

  return &config.measure_widget[g_layout_sel];
}

int layout_used(void)
{
  int n = 0;

  for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
  {
    if (MEASURE_NONE != config.measure_widget[i].metric)
      n++;
  }

  return n;
}

// A reading the layout is not already showing, so that adding one twice takes
// two deliberate presses of the metric key rather than being the default
int layout_unused_metric(void)
{
  for (int m = MEASURE_NONE + 1; m < MEASURE_COUNT; m++)
  {
    bool taken = false;

    for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
    {
      if (config.measure_widget[i].metric == m)
        taken = true;
    }

    if (!taken)
      return m;
  }

  return MEASURE_VPP;
}

//-----------------------------------------------------------------------------
/*
 * The signal behind the strip: the record's own min/max envelope, one column at
 * a time, instead of the zigzag that used to stand in for it.
 *
 * One byte per column - the envelope's top row in the high nibble, its bottom
 * row in the low one, MV_ENV_NONE where no record reaches. Display ROWS and not
 * the raw counts they came from, which costs nothing: the mapping from counts to
 * rows depends on the vertical scale and position, and both of those already
 * invalidate this through scope_display_settings_changed(). It buys back 159
 * bytes of the borrowed TCM block, which is a region with a floor rather than
 * room to spare - see the linker script.
 *
 * WHY THE RECORD AND NOT THE RING, when the strip's axis is the ring's whole
 * 786 us. Because the ring cannot be read as a record once acquisition is
 * running. A sweep is one full lap of it (capture.c: the post-trigger count is
 * CAPTURE_BUFFER_SIZE - trigger_offset), and dma_finish() hands straight over
 * to dma_start(), which begins overwriting from ring index 0 again. The ring is
 * a coherent, trigger-anchored record for exactly as long as it takes the next
 * sweep to start, which at 125 MS/s is 786 us - tens of laps between two
 * repaints of this strip. Whatever the main loop finds there afterwards is a
 * different sweep, and the trigger position saved from the last dma_finish does
 * not describe it. The 24 KB storage record is the ONLY trigger-anchored
 * snapshot that survives, and it is what the trace is drawn from - so it is
 * also the only thing that can be drawn here without the frame below lying
 * about which signal it is framing.
 *
 * At slow timebases the record IS the whole ring, decimated 4:1, so the strip
 * fills end to end. Past ~16 us/div it becomes a 24 KB full-rate WINDOW of the
 * ring (update_storage_window) and covers about a quarter of the strip; the
 * rest stays empty, which is the true statement - there is no record there, and
 * panning into it moves the window rather than revealing anything.
 */


//-----------------------------------------------------------------------------
int clip_for_display(int value)
{
  value = GRID_HEIGHT/2-1 - value;

  if (value > (GRID_HEIGHT-2))
    value = (GRID_HEIGHT-2);
  else if (value < 0)
    value = 0;

  return value;
}

//---------------------------------------------------------------------
void close_gaps(DisplayBuffer *db)
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

/*- Display post-processing, cursors and the trend view ---------------------*/
void update_display(void);

//-----------------------------------------------------------------------------
// Forget the accumulated envelope and averages: called whenever the mapping
// from column to time or voltage changes (pan, zoom, scale, coupling) and
// when the menu toggles the features themselves
void scope_display_settings_changed(void)
{
  memset(g_persist_lvl, 0, sizeof(g_persist_lvl));
  g_persist_stamp = timer_ms();
  memset(g_avg_have, 0, sizeof(g_avg_have));
  g_shadow_valid = false;

  // The record map's envelope is stored as display rows, so it was built
  // through the vertical scale and position - which is exactly what just moved
  g_mv_env_valid = false;
}

//-----------------------------------------------------------------------------
// The probe attenuation changed - from the 1X/10X key or from the menu. Every
// millivolt on this screen is derived from one of the two multipliers below, so
// both have to be recomputed together; the trace itself does not move, because
// the mV-per-count and the mV-per-pixel moved by the same factor.
void scope_probe_changed(void)
{
  if (scope_calibration_mode)
    return;   // it is forced off there, see scope_init

  config.vertical_mult = config.calib_vs_mult[config.vertical_scale] *
      config_probe_mult();
  config.vertical_position_mv = config.vertical_position *
      vs_mv_px(config.vertical_scale);
  config.trigger_level_mv = config.trigger_level * vs_mv_px(config.vertical_scale);

  // The hardware trigger is a converter code, and the level it is derived from
  // has just changed units
  capture_set_trigger_level(config.trigger_level_mv);

  mpanel_invalidate();
  draw_status_line();
  refresh_view();
}

//-----------------------------------------------------------------------------
/*
 * The text size changed, so the screen is divided up differently: see ScopeGeom.
 *
 * Everything that is positioned against the grid asks g_geom, so setting it is
 * most of the work. What is left is the state that CACHES a position: the
 * per-column shadow of what is on the panel (rows moved), the grid patterns
 * (fewer rows to rule), the trace's own history (a column's pixel row means a
 * different voltage now), and the state slot's "nothing has changed" latch.
 *
 * The vertical position and the trigger level are in PIXELS from the middle of
 * the grid, and the middle stays the middle - but a smaller grid puts both
 * nearer its edge, so they are clamped back inside it here rather than being
 * quietly off-screen with a marker pinned to the frame.
 */
void scope_ui_scale_changed(void)
{
  int half;

  g_geom = (UI_SCALE_LARGE == config.ui_scale) ? g_geom_large : g_geom_normal;

  half = GRID_HEIGHT / 2 - 1;

  if (config.vertical_position > half)
    config.vertical_position = half;
  else if (config.vertical_position < -half)
    config.vertical_position = -half;

  if (config.trigger_level > half)
    config.trigger_level = half;
  else if (config.trigger_level < -half)
    config.trigger_level = -half;

  config.vertical_position_mv = config.vertical_position *
      vs_mv_px(config.vertical_scale);
  config.trigger_level_mv = config.trigger_level * vs_mv_px(config.vertical_scale);

  capture_set_vertical_parameters();
  capture_set_trigger_level(config.trigger_level_mv);

  g_state = -1;              // the capture-state slot repaints itself
  g_vpos_marker.valid = false;
  g_trig_marker.valid = false;
  g_shadow_valid = false;
  memset(g_persist_lvl, 0, sizeof(g_persist_lvl));
  memset(g_avg_have, 0, sizeof(g_avg_have));
  grid_init();
  memset(g_mpanel_cell, 0, sizeof(g_mpanel_cell));
  g_mpanel_is_text = false;
  mpanel_invalidate();
}

//-----------------------------------------------------------------------------
void scope_measure_panel_changed(void)
{
  // The cells were composed to the old font's cell width, so they go; the band
  // rebuilds at the new one on the scope's next tick. The trace area itself is
  // repainted in full when the menu closes over it, which is what covers the
  // rows the band gives back when it shrinks.
  memset(g_mpanel_cell, 0, sizeof(g_mpanel_cell));
  g_mpanel_is_text = false;
  mpanel_invalidate();
}

//-----------------------------------------------------------------------------
// Acquisition averaging, display space: an EMA of each column's midpoint
// across trigger-aligned frames, drawn as a thin trace instead of the
// min/max band. At fast timebases a column IS one sample, so this is
// textbook average mode there (~sqrt(N) noise reduction); at slow timebases
// it steadies the centre of the band. Only NEW frames advance the average -
// a pan redraw of the same record must not multiply-count it.
static void display_average(void)
{
  uint32_t gen = capture_get_generation();
  int shift = config.average_mode + 1; // N = 2 << mode
  bool fresh = (gen != g_avg_gen);

  g_avg_gen = gen;

  if (config.average_mode <= 0)
    return;

  for (int c = 0; c < GRID_WIDTH; c++)
  {
    int px;

    if (!(g_display_buffer.flags[c] & SAMPLE_FLAG_VALID))
    {
      g_avg_have[c] = 0;
      continue;
    }

    px = (g_display_buffer.min[c] + g_display_buffer.max[c]) * 32; // mid * 64

    if (!g_avg_have[c])
    {
      g_avg_acc[c] = px;
      g_avg_have[c] = 1;
    }
    else if (fresh)
    {
      int acc = g_avg_acc[c];

      acc += (px - acc) >> shift;
      g_avg_acc[c] = acc;
    }

    px = (g_avg_acc[c] + 32) / 64;

    g_display_buffer.min[c] = px;
    g_display_buffer.max[c] = px;
  }
}

//-----------------------------------------------------------------------------
// Persistence: fold what is about to be displayed into the envelope.
//
// Infinite mode is the plain union of every frame, pinned at full brightness
// - the runt that fired once an hour ago is still there, which is the whole
// point of it.
//
// Decay mode is the same envelope with a brightness that runs out, and one
// rule makes it behave like a phosphor: the glow is refreshed only where the
// envelope GROWS, not merely where the trace is. A steady trace therefore
// stops refreshing its own column immediately and its envelope fades to
// nothing underneath it, while the one excursion that reached further lights
// up and then dies away over PERSIST_DECAY_MS. Refreshing wherever the trace
// happened to be would pin every column at full brightness forever, which is
// infinite persistence again with extra steps.
//
// A column whose glow has run out drops back to the live band, and by then
// it is black, so nothing is seen to snap.
static void display_persist_accum(void)
{
  uint32_t now = timer_ms();
  int drop = 0;

  if (config.persist_mode == PERSIST_OFF)
    return;

  if (config.persist_mode == PERSIST_DECAY)
  {
    drop = (int)(((now - g_persist_stamp) * 255) / PERSIST_DECAY_MS);

    // Only when it came to something: at a few hundred frames a second the
    // elapsed time per frame rounds to no decay at all, and advancing the
    // stamp anyway would throw that time away and leave the glow immortal
    if (drop > 0)
      g_persist_stamp = now;
  }

  for (int c = 0; c < GRID_WIDTH; c++)
  {
    int lo = g_display_buffer.min[c], hi = g_display_buffer.max[c];
    bool grew_top = false, grew_bot = false;

    if (!(g_display_buffer.flags[c] & SAMPLE_FLAG_VALID))
      continue;

    if (!g_persist_lvl[c])
    {
      g_persist_min[c] = lo;
      g_persist_max[c] = hi;
      grew_top = grew_bot = true;
    }
    else
    {
      if (lo < g_persist_min[c])
      {
        g_persist_min[c] = lo;
        grew_top = true;
      }

      if (hi > g_persist_max[c])
      {
        g_persist_max[c] = hi;
        grew_bot = true;
      }
    }

    if (config.persist_mode == PERSIST_INFINITE)
    {
      g_persist_lvl[c] = 255;
      continue;
    }

    if (grew_top || grew_bot)
      g_persist_lvl[c] = 255;
    else
      g_persist_lvl[c] = (g_persist_lvl[c] > drop) ? (g_persist_lvl[c] - drop) : 0;

    // ...and the glow pulls back toward the live trace as it fades.
    //
    // Dimming alone is not enough, because it is per column and a column that
    // grows at all is pinned bright - which is most of them on any steep
    // edge, where a pixel of trigger jitter moves the band every frame. That
    // pinned the WHOLE accumulated band, so an excursion from a minute ago
    // stayed at full brightness underneath a live edge that had nothing to do
    // with it. Each end therefore retreats on its own, and only the end that
    // did not just grow: an edge column re-lights the side the beam is on and
    // sheds the side it left.
    //
    // Exponential, with a floor of one pixel so the last few always close.
    if (drop > 0 && !grew_top)
    {
      int step = ((lo - g_persist_min[c]) * drop) / 255;

      g_persist_min[c] += (step > 1) ? step : 1;

      if (g_persist_min[c] > lo)
        g_persist_min[c] = lo;
    }

    if (drop > 0 && !grew_bot)
    {
      int step = ((g_persist_max[c] - hi) * drop) / 255;

      g_persist_max[c] -= (step > 1) ? step : 1;

      if (g_persist_max[c] < hi)
        g_persist_max[c] = hi;
    }
  }
}

//-----------------------------------------------------------------------------
// Measurement cursors. Both mappings are exactly the arithmetic the trace
// itself is drawn with, so the cursors track pan and zoom.
static int cursor_t_col(int64_t t_ns)
{
  return GRID_WIDTH/2 + (int)((t_ns - config.horizontal_position) /
      hs_px_value[config.horizontal_scale]);
}

//-----------------------------------------------------------------------------
static int cursor_v_row(int mv)
{
  int px = (mv - config.vertical_position_mv) / vs_mv_px(config.vertical_scale) +
      config.vertical_position;

  return clip_for_display(px);
}

//-----------------------------------------------------------------------------
// The whole status line while cursors are up: both deltas and the implied
// frequency, with the active cursor named so the arrows have an obvious
// target
void cursor_readout(void)
{
  static const char *const names[] = { "", "T1", "T2", "V1", "V2" };
  char dt[14], f[14], dv[14], buf[64];
  int64_t d = g_cursor_t[1] - g_cursor_t[0];

  if (g_toast_active)
    return;

  format_ns(d, dt, sizeof(dt));
  format_mv(g_cursor_v[0] - g_cursor_v[1], dv, sizeof(dv));

  if (d != 0)
  {
    int64_t ad = (d < 0) ? -d : d;

    format_hz_mhz(1000000000000ll / ad, f, sizeof(f));
  }
  else
  {
    snprintf(f, sizeof(f), "-");
  }

  lcd_fill_rect(0, GRID_BOTTOM+1, LCD_WIDTH, STATUS_LINE_HEIGHT, BG_COLOR);
  lcd_set_color(BG_COLOR, CURSOR_T_COLOR);
  snprintf(buf, sizeof(buf), "%s dT %s 1/dT %s dV %s",
      names[g_cursor_sel], dt, f, dv);
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, buf);
}

//-----------------------------------------------------------------------------
// Arrow keys while a cursor is active: LEFT/RIGHT drag a time cursor,
// UP/DOWN drag a voltage cursor, one pixel per press, x10 with SHIFT.
// Everything else falls through to the normal handlers.
static bool cursor_buttons(int buttons, bool shift, bool repeat)
{
  int mul = shift ? 10 : 1;

  if (repeat)
    mul *= 4;

  if (g_cursor_sel == 1 || g_cursor_sel == 2)
  {
    int64_t step = (int64_t)hs_px_value[config.horizontal_scale] * mul;
    int64_t *t = &g_cursor_t[g_cursor_sel - 1];

    if (buttons & BTN_LEFT)
      *t -= step;
    else if (buttons & BTN_RIGHT)
      *t += step;
    else
      return false;
  }
  else
  {
    int step = vs_mv_px(config.vertical_scale) * mul;
    int *v = &g_cursor_v[g_cursor_sel - 3];

    if (buttons & BTN_UP)
      *v += step;
    else if (buttons & BTN_DOWN)
      *v -= step;
    else
      return false;
  }

  g_cursor_dirty = true;
  cursor_readout();

  return true;
}

//-----------------------------------------------------------------------------
// Roll view
//
// A triggered sweep shows a record, and a record only exists once it is
// whole: at 500 ms/div that is six seconds of a frozen screen before six
// seconds of new signal replace all of it at once. Past that the useful
// picture is not a waveform at all - it is the input plotted against wall
// clock, appended a column at a time, which is what the stock firmware turns
// into above 500 ms/div and what this does.
//
// Nothing here waits for the acquisition. The ring is folded into a peak pair
// every pass of the main loop (capture_fold_samples), the pair accumulates
// into the column being written, and when that column has had its share of
// time the whole screen shifts left by one. The trigger takes no part - the
// acquisition is held in AUTO for the duration so the ring keeps turning and
// the measurements keep landing, whatever the trigger is set to.
//-----------------------------------------------------------------------------
void update_display(void)
{
  int scale = vs_mv_px(config.vertical_scale);

  // The display buffer is the trend plot while that view is up: a frame
  // landing (or a pan-settle refresh) must not overwrite it with the trace
  if (g_trend_mode)
  {
    trend_view_update();
    return;
  }

  // ...and it is the roll strip while THAT view is up, which the columns of
  // a record would wipe out. Nothing to rebuild: roll_task() keeps the strip
  // in its own store and this only puts it back on screen.
  if (g_roll_active)
  {
    roll_publish();
    return;
  }

  g_data_buffer.size = GRID_WIDTH;
  capture_get_data(&g_data_buffer);

  // Whether what was just drawn between the samples is still worth believing.
  // Only the reconstruction can overstate itself this way - straight lines
  // never claimed to be the signal - so the strain is only reported when it
  // is switched on.
  {
    ScopeMeasure sm;
    const uint8_t *data;
    int size, offset, period_ns, trig;

    g_recon_strained = false;

    if (config.draw_mode == DRAW_SINC &&
        capture_get_record(&data, &size, &offset, &period_ns, &trig) &&
        period_ns > 0 && capture_get_measurements(&sm) && sm.frequency > 0)
    {
      g_recon_strained =
          (sm.period_med_ns < RECON_MIN_SAMPLES * period_ns);
    }
  }

  for (int i = 0; i < GRID_WIDTH; i++)
  {
    int min = (g_data_buffer.min[i] - g_data_buffer.vertical_position) / scale + config.vertical_position;
    int max = (g_data_buffer.max[i] - g_data_buffer.vertical_position) / scale + config.vertical_position;

    g_display_buffer.min[i]   = clip_for_display(max);
    g_display_buffer.max[i]   = clip_for_display(min);
    g_display_buffer.flags[i] = g_data_buffer.flags[i];
  }

  display_average();            // thin running-mean trace instead of the band
  close_gaps(&g_display_buffer);
  display_persist_accum();      // envelope of what is actually displayed

  dbit_span_update();           // the bit grid is as tall as the trace is

  redraw_trace();
}

//-----------------------------------------------------------------------------
// Repaint whatever the trace area currently shows. Vertical and timebase
// changes are meaningful in the spectrum view too (they set the ADC range
// and the record length), but they must not paint a waveform over it.
void refresh_view(void)
{
  if (g_fft_mode)
    fft_update();
  else
    update_display();
}

//-----------------------------------------------------------------------------
void change_horizontal_scale(int delta)
{
  if ((delta < 0 && config.horizontal_scale == 0) ||
      (delta > 0 && config.horizontal_scale == HS_LAST))
    return;

  config.horizontal_scale += delta;
  config.horizontal_period = hs_px_value[config.horizontal_scale];

  // Roll is a property of the timebase and this is the only key that changes
  // it. Ahead of the pan being rescaled, because entering roll zeroes it.
  roll_set_active(roll_wanted());

  config.horizontal_position_px = config.horizontal_position / hs_px_value[config.horizontal_scale];

  draw_horizontal_scale();
  draw_horizontal_position();
  update_sample_rate();
  refresh_view();
}

//-----------------------------------------------------------------------------
static void change_horizontal_position_ex(int delta, bool repeat)
{
  int div = hs_px_value[config.horizontal_scale];

  // Rolling: the right-hand column is now and there is nothing either side of
  // the screen to walk into - the record a pan moves through is the very
  // thing roll does without
  if (g_roll_active)
    return;

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
void change_vertical_scale(int delta)
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

  int old_scale = vs_mv_px(config.vertical_scale);

  config.vertical_scale = scale;
  config.vertical_mult = config.calib_vs_mult[config.vertical_scale] * config_probe_mult();
  config.vertical_position_mv = config.vertical_position * vs_mv_px(config.vertical_scale);

  config.trigger_level_mv = config.trigger_level * vs_mv_px(config.vertical_scale);

  scope_display_settings_changed(); // px-to-mV mapping changed under them
  roll_rescale(old_scale, config.vertical_position); // ...including the plot's
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

  int old_vpos = config.vertical_position;

  config.vertical_position += delta;
  config.vertical_position_mv = config.vertical_position * vs_mv_px(config.vertical_scale);

  scope_display_settings_changed(); // every column moved on screen
  roll_rescale(vs_mv_px(config.vertical_scale), old_vpos); // the plot too

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
// Push config.trigger_level (px) to hardware. Deliberately draw-free: also
// called from the system menu, which overlays the marker gutter (the screen
// is repainted in full when the menu closes).
void scope_apply_trigger_level(void)
{
  if (config.trigger_level < MIN_TRIGGER_LEVEL)
    config.trigger_level = MIN_TRIGGER_LEVEL;
  else if (config.trigger_level > MAX_TRIGGER_LEVEL)
    config.trigger_level = MAX_TRIGGER_LEVEL;

  config.trigger_level_mv = config.trigger_level * vs_mv_px(config.vertical_scale);

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
// Decoded-bytes overlay, drawn after a trace sweep completes so the sweep
// does not immediately paint over it (unchanged columns are skipped by the
// dirty-column logic, which keeps the panel intact on a stable signal)

//-----------------------------------------------------------------------------
// SHIFT+50P: find the narrowest pulse anywhere in the record and pan to it.
//
// The screen shows 300 of 98304 samples; a runt that fires once per record
// is practically invisible on the live trace and an edge trigger never
// singles it out. Scanning the whole record after the fact does. Runs on the
// STOPPED record - on a live one the glitch would be overwritten before the
// pan lands on it - so it stops acquisition first, exactly like STOP.
static void find_glitch(void)
{
  Measure m;
  const uint8_t *data;
  int size, offset, period_ns, trigger_timepos;
  int width = 0, count = 0, pos, mid, hyst;
  int64_t t_ns;
  char msg[48], wtxt[12];
  bool high = false;

  if (capture_get_state() != CAPTURE_STATE_STOP)
  {
    capture_stop();
    draw_capture_state();
  }

  if (!capture_get_raw_measure_fresh(&m) || !capture_get_record(&data, &size,
      &offset, &period_ns, &trigger_timepos))
    return;

  if ((m.pk_hi - m.pk_lo) < 6)
  {
    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, "No signal to search");
    return;
  }

  // The frequency counter's own Schmitt pair, so "a pulse" means the same
  // thing here as it does there
  mid = (m.pk_lo + m.pk_hi) / 2;
  hyst = (m.pk_hi - m.pk_lo) * 20 / 100;

  if (hyst < 2)
    hyst = 2;

  pos = measure_find_min_pulse(data, size, offset, mid - hyst, mid + hyst,
      &width, &high, &count);

  if (pos < 0)
  {
    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, "No pulses in the record");
    return;
  }

  // Pan the view onto the pulse: its center in trigger-relative time is
  // exactly what horizontal_position means
  t_ns = (int64_t)(pos - trigger_timepos) * period_ns;

  if (t_ns < MIN_HORIZONTAL_POSITION)
    t_ns = MIN_HORIZONTAL_POSITION;
  else if (t_ns > MAX_HORIZONTAL_POSITION)
    t_ns = MAX_HORIZONTAL_POSITION;

  config.horizontal_position = t_ns;
  config.horizontal_position_px =
      (int)(config.horizontal_position / hs_px_value[config.horizontal_scale]);

  draw_horizontal_position();
  update_sample_rate();
  refresh_view();

  {
    int64_t w_ps = (int64_t)width * period_ns * 1000;

    format_ps((w_ps > 0x7fffffff) ? 0x7fffffff : (int)w_ps, wtxt, sizeof(wtxt));
  }

  // The tie count is the verdict: "x1" is a genuine outlier worth staring
  // at, "x2481" is the signal's own half-periods quantized by the sample
  // clock - nothing anomalous in the record at all
  if (count > 1)
    snprintf(msg, sizeof(msg), "Min %s pulse: %s x%d (STOP resumes)",
        high ? "high" : "low", wtxt, count);
  else
    snprintf(msg, sizeof(msg), "Min %s pulse: %s (STOP resumes)",
        high ? "high" : "low", wtxt);

  toast_show();
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
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

  config.trigger_level = sm.vmid_mv / vs_mv_px(config.vertical_scale);

  scope_apply_trigger_level();
}

//-----------------------------------------------------------------------------
void trigger_set_50_percent(void)
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
void autoset_set_horizontal(int hs)
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
// Timebase that puts periods_x10/10 whole periods across the screen. Sweep
// timebases only - the hunt found a signal by measuring records, and landing
// it in a view that draws none would throw that away.
static void autoset_set_timebase(int freq, int periods_x10)
{
  int64_t need = (100000000ll * periods_x10) / freq / GRID_DIVS_H;
  int hs = 0;

  while (hs < HS_SWEEP_LAST && hs_div_value[hs] < need)
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
  int step = (int)(mv / vs_mv_px(config.vertical_scale));

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
    if (vamp_mv / vs_mv_px(s) <= AUTOSET_TARGET_PX)
      break;
  }

  if (s < g_autoset_zoom_floor)
    s = g_autoset_zoom_floor;

  return s;
}

//-----------------------------------------------------------------------------
void autoset_set_scale(int scale)
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
          config.vertical_position * vs_mv_px(config.vertical_scale);

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

  // An abort restored a timebase that may have been a roll one
  roll_sync();

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

  // A toast owns the whole status line and every element on it returns early
  // while one is up — including the measurements. Announcing a frequency the
  // measurements are about to show anyway is not worth blanking them for a
  // second and a half, so the result is only announced when there is nothing
  // else down there.
  if (config.measure_display)
    return;

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
    config.vertical_mult = config.calib_vs_mult[config.vertical_scale] * config_probe_mult();
    config.vertical_position_mv =
        config.vertical_position * vs_mv_px(config.vertical_scale);
    config.trigger_level_mv =
        config.trigger_level * vs_mv_px(config.vertical_scale);

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

  // The hunt steers on records, so it cannot run inside a view that has none.
  // It ends on a swept timebase whatever it finds, and "no signal found" puts
  // the roll timebase - and the roll view with it - back.
  roll_sync();

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
// Auto-calibration against an OPEN input.
//
// Three of the four calibration parameters are derivable with nothing
// connected, because each one is a statement about the instrument rather than
// about a signal:
//
//   Z (calib_dac_zero)     the resting level has to read ZERO_POINT
//   D (calib_channel_delta) the two converters have to agree on that level
//   O (calib_dac_mult)     one position pixel has to move the trace one pixel
//
// The fourth, S (calib_vs_mult), is a volts-per-count claim and cannot be
// checked without a reference amplitude - there is no calibrator output on
// this board (doc/Hardware.md), so it stays manual.
//
// Every phase changes hardware and then has to wait for the change to reach a
// record: capture_get_*_fresh() bypasses the display throttle but still hands
// back whatever acquisition last completed, which for the first frames after
// a DAC or attenuator change is the state from before it.
//-----------------------------------------------------------------------------
// Set the vertical range from the menu, so calibration values can be edited
// per range without leaving it
void scope_set_vertical_scale(int scale)
{
  change_vertical_scale(scale - config.vertical_scale);
}

//-----------------------------------------------------------------------------
void scope_buttons_handler(int buttons)
{
  bool shift  = (buttons & BTN_SHIFT);
  bool repeat = (buttons & BTN_REPEAT);

  // The AC/DC hold is armed again only once the key is let go - the release
  // arrives as an event with no keys in it, which is exactly what this reads
  if (0 == (buttons & BTN_AC_DC))
    g_acdc_held = false;

  // ...and so does the layout editor, for a simpler reason: every key on this
  // screen means something else than it does over a live trace
  if (g_layout_edit)
  {
    if (!repeat || (buttons & (BTN_LEFT | BTN_RIGHT | BTN_UP | BTN_DOWN)))
      layout_edit_keys(buttons);

    return;
  }

  // While auto-calibration is up it owns the keyboard: it is rewriting the
  // vertical settings underneath, so letting anything else change them
  // mid-run would silently corrupt the values it is deriving.
  if (g_autocal_active)
  {
    // The driver reports a release as an event with no keys in it, and the
    // "MODE closes" message at the end is dismissed by ANY event - so a STOP
    // that ends the run put the message up on the press and took it away again
    // on the release, about sixty milliseconds later. Nothing else here reads
    // a bare event as a keypress, so the guard belongs at the door.
    if (0 == (buttons & ~(BTN_SHIFT | BTN_REPEAT | BTN_SHIFT_TAP)))
      return;

    // The gain pass is the exception, and only at its prompt: the expected
    // level and the range are the two things the user is holding, so those
    // four keys keep their normal meaning and their auto-repeat. Everything
    // else in a run is a one-shot answer to a question that was asked.
    if (ACAL_ASK_REF == g_autocal_phase)
    {
      if (buttons & (BTN_LEFT | BTN_RIGHT))
      {
        gain_ref_step((buttons & BTN_RIGHT) ? 1 : -1, repeat);
        return;
      }

      if (shift && (buttons & (BTN_UP | BTN_DOWN)))
      {
        gain_range_step((buttons & BTN_UP) ? 1 : -1);
        return;
      }
    }

    if (repeat)
      return;

    if (ACAL_DONE == g_autocal_phase)
      autocal_close();
    else if (buttons & BTN_STOP)
    {
      // Stopping at the gain prompt is not a failure: zero, delta and the DAC
      // step are already found and worth keeping
      if (ACAL_ASK_REF == g_autocal_phase)
        autocal_finish(true);
      else
        autocal_finish(false);
    }
    else if (buttons & BTN_MODE)
    {
      if (ACAL_ASK == g_autocal_phase)
        autocal_begin();
      else if (ACAL_ASK_REF == g_autocal_phase)
      {
        g_autocal_phase = ACAL_SCALE;
        g_autocal_timer = AUTOCAL_TIMEOUT;
        g_gain_note_until = 0;
        autocal_mark();
        autocal_say(g_autocal_gain_only ? "Measuring this range" :
            "Step 5 of 5: gain", "Hold the level steady");
      }
    }

    return;
  }

  if (g_fft_mode && !scope_calibration_mode)
  {
    if (fft_buttons(buttons, shift, repeat))
      return;
  }

  if (g_trend_mode && !scope_calibration_mode)
  {
    if (trend_buttons(buttons, shift, repeat))
      return;
  }

  if (g_cursor_sel && !g_fft_mode && !g_decode_mode && !g_trend_mode &&
      !g_roll_active && !scope_calibration_mode)
  {
    if (cursor_buttons(buttons, shift, repeat))
      return;
  }

  if ((buttons & BTN_UP) && (buttons & BTN_DOWN))
  {
    int old_vpos = config.vertical_position;

    config.vertical_position = 0;
    config.vertical_position_mv = 0;

    capture_set_vertical_parameters();
    roll_rescale(vs_mv_px(config.vertical_scale), old_vpos);
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
    if (g_roll_active)
      return; // already zero, and nothing to re-time

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
    if (repeat || scope_calibration_mode)
      return;

    if (config.trigger_mode == TRIGGER_MODE_SINGLE)
      config.trigger_mode = TRIGGER_MODE_AUTO;
    else
      config.trigger_mode++;

    // Set here, applied on the way out of roll: the strip is drawn from the
    // ring and holding the acquisition in AUTO is what keeps the ring turning
    if (!g_roll_active)
    {
      capture_set_trigger_mode(config.trigger_mode);
      capture_start();
    }

    draw_trigger_mode();
  }
  else if (buttons & BTN_EDGE)
  {
    if (repeat || scope_calibration_mode)
      return;

    if (shift)
    {
      // SHIFT+EDGE: toggle the protocol decoder view
      if (!g_decode_mode)
      {
        decode_mode_enter();
      }
      else
      {
        g_decode_mode = false;
        g_decode_hunt = false;
        decode_trigger_restore();
        g_shadow_valid = false;
        g_sweep_force = true;
        roll_sync();
        update_display();
        redraw_miniview();   // restore the decorative wave over the byte map
        decode_band_build(); // and clear the byte marks off the trace
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
    if (scope_calibration_mode)
      change_calibration_value(1, shift, repeat);
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
    if (scope_calibration_mode)
      change_calibration_value(-1, shift, repeat);
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
    // HELD: the other thing this key is about. Coupling decides whether there
    // is a DC level at all, so the switch for "show it in the bar instead of a
    // peak-to-peak of noise" belongs on the same key rather than four rows deep
    // in a menu - and it only does anything on the side of the key that has a
    // level to show. One toast, because a setting toggled by a long press with
    // no visible result is a setting nobody trusts.
    if (repeat)
    {
      char msg[48];

      if (g_acdc_held)
        return; // one toggle per hold, not one per repeat tick

      g_acdc_held = true;

      // The press that began this hold has already flipped the coupling. Put it
      // back: a hold means the OTHER thing on this key, not both things, and
      // the user who wants the coupling changed has not lifted their finger yet
      // to say so.
      config.ac_coupling = !config.ac_coupling;
      scope_display_settings_changed();
      capture_set_vertical_parameters();
      draw_ac_dc();

      config.vpp_dc_off = !config.vpp_dc_off;

      snprintf(msg, sizeof(msg), "Flat DC in the bar: %s%s",
          config.vpp_dc_off ? "OFF" : "ON",
          config.ac_coupling ? "  (DC coupling only)" : "");
      toast_show();
      lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
      return;
    }

    config.ac_coupling = !config.ac_coupling;

    scope_display_settings_changed(); // the DC level just moved
    capture_set_vertical_parameters();
    draw_ac_dc();
  }

  else if (buttons & BTN_MODE)
  {
    if (repeat)
      return;

    if (scope_calibration_mode)
    {
      // MODE hides the hint band. It comes back on the next parameter -
      // see g_calib_hint.
      g_calib_hint = !g_calib_hint;
      g_calib_hint_param = -1;
      calib_hint_update();
      g_shadow_valid = false;
      g_sweep_force = true;
      return;
    }

    if (shift)
    {
      // SHIFT+MODE: toggle the FFT spectrum view
      g_fft_mode = !g_fft_mode;
      g_decode_mode = false;
      decode_trigger_restore(); // no-op unless the decoder view was on
      g_trend_mode = false;
      g_shadow_valid = false;
      g_sweep_force = true;
      roll_sync(); // a spectrum has no time axis to roll along
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

  // The legend on the shift key. A tap of it - no chord, no hold - is the one
  // press that was free to mean 1X/10X, see BTN_SHIFT_TAP.
  else if (buttons & BTN_SHIFT_TAP)
  {
    if (scope_calibration_mode)
      return;   // calibrating through an attenuator is not a calibration

    // Switching a 1x probe on would do nothing at all, and this key's legend
    // promises 10x: an instrument nobody has told about its probe gets the one
    // nearly everybody has, and the menu is where an odd ratio is said.
    if (!config.x10 && 1 == config_probe_ratio())
      config.probe_ratio = PROBE_RATIO_10X;

    config.x10 = !config.x10;
    scope_probe_changed();

    // Said out loud, because every reading on screen just changed by that
    // factor and the tag in the top bar is easy to miss on a key that used to
    // do nothing
    {
      char msg[40];

      snprintf(msg, sizeof(msg), config.x10 ?
          "Probe %s - readings scaled" : "Probe off - readings direct",
          probe_ratio_labels[((unsigned)config.probe_ratio < PROBE_RATIO_COUNT) ?
              config.probe_ratio : 0]);
      toast_show();
      lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
    }
  }

  // Straight to what the two large readouts show, one key each. Which two
  // measurements are worth the big font changes while probing, and reaching it
  // through Menu > Measurements > Status line > Left was four levels of menu
  // for a question asked that often.
  else if (buttons & (BTN_F1 | BTN_F2))
  {
    if (repeat || scope_calibration_mode)
      return;

    scope_menu_open_slot((buttons & BTN_F1) ? 0 : 1);
  }

  else if (buttons & BTN_AUTO)
  {
    if (repeat || g_fft_mode)
      return;

    // Same promise as everywhere else on this key - "work it out yourself" -
    // and in calibration mode the thing to work out is the calibration.
    // Gain (S) is the one parameter it cannot do; the hint says so.
    if (scope_calibration_mode)
    {
      scope_autocal_start();
      return;
    }

    autoset_start();
  }

  else if (buttons & BTN_50P)
  {
    if (repeat || scope_calibration_mode)
      return;

    // Both work on a record: the glitch finder scans one, and a 50% level is
    // a trigger setting for a sweep that is not running here
    if (g_roll_active)
    {
      toast_show();
      lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Not while rolling - zoom in first");
      return;
    }

    if (shift)
      find_glitch();
    else
      trigger_set_50_percent();
  }

  else if (buttons & BTN_SAVE)
  {
    if (repeat || scope_calibration_mode)
      return;

    if (shift)
    {
      trend_toggle();
      return;
    }

    // Cursors live on the waveform view only
    if (g_fft_mode || g_decode_mode || g_trend_mode)
      return;

    if (g_cursor_sel == 0 && g_cursor_t[0] == g_cursor_t[1])
    {
      // Never used yet: park the pairs two divisions around the view centre
      int64_t div_ns = hs_div_value[config.horizontal_scale];
      int div_mv = vs_mv_px(config.vertical_scale) * GRID_DIV_PX;

      g_cursor_t[0] = config.horizontal_position - 2 * div_ns;
      g_cursor_t[1] = config.horizontal_position + 2 * div_ns;
      g_cursor_v[0] = config.vertical_position_mv + 2 * div_mv;
      g_cursor_v[1] = config.vertical_position_mv - 2 * div_mv;
    }

    g_cursor_sel = (g_cursor_sel + 1) % 5;
    g_cursor_dirty = true;
    draw_status_line(); // hands the line to the readout - or back
  }

  else if (buttons & BTN_STOP)
  {
    if (capture_get_state() == CAPTURE_STATE_STOP)
      capture_start();
    else
      capture_stop_view(); // freeze what the display shows, not the live ring
  }
}

//-----------------------------------------------------------------------------
void scope_init(bool calibration_mode)
{
  scope_calibration_mode = calibration_mode;

  config.horizontal_period = hs_px_value[config.horizontal_scale];
  config.vertical_mult = config.calib_vs_mult[config.vertical_scale] * config_probe_mult();

  // How the screen is divided up, before anything is laid out against it
  scope_ui_scale_changed();

  grid_init();
  persist_build_ramp();

  // The screen was just cleared: previous-frame caches are stale
  g_shadow_valid = false;
  g_sweep_force = true;
  g_vpos_marker.valid = false;
  g_trig_marker.valid = false;

  // The stored timebase decides the view, and update_sample_rate() at the
  // bottom of this has to already know which one it is timing the ADC for.
  // Not roll_sync(): nothing is on screen yet for it to compare against.
  g_roll_active = false;
  roll_set_active(roll_wanted());

  // Settle the panel before anything paints. It decides who owns the status
  // line, and it is a static that outlives the application: entering the scope
  // a second time painted that line for the panel state of the previous
  // session. The cells go with it - the readings in them are the old session's,
  // and they would be composited into the first sweep.
  g_mpanel_active = mpanel_wanted();
  g_line_owner = measure_owns_status_line();
  memset(g_mpanel_cell, 0, sizeof(g_mpanel_cell));
  g_mpanel_is_text = false;
  mpanel_invalidate();

  draw_grid_frame();
  draw_vertical_position(false);
  draw_trigger_mode();
  draw_trigger_edge();
  draw_capture_state();
  draw_status_line();
  redraw_trace();

  if (scope_calibration_mode)
  {
    capture_set_trigger_edge(TRIGGER_EDGE_RISE);
    capture_set_trigger_mode(TRIGGER_MODE_AUTO);
    capture_set_trigger_level(0);
  }
  else
  {
    capture_set_trigger_edge(config.trigger_edge);
    // AUTO while rolling whatever the stored mode says - see roll_set_active
    capture_set_trigger_mode(g_roll_active ? TRIGGER_MODE_AUTO : config.trigger_mode);
    capture_set_trigger_level(config.trigger_level_mv);
  }

  timer_add(&g_toast_timer);
  timer_add(&g_state_timer);
  timer_add(&g_measure_timer);
  timer_add(&g_autoset_timer);
  timer_add(&g_autocal_timer);
  timer_add(&g_pan_settle_timer);
  timer_add(&g_fps_timer);
  timer_add(&g_fft_timer);
  timer_add(&g_trend_timer);
  timer_add(&g_mv_timer);
  g_fps_timer = 1000;
  g_trend_timer = TREND_INTERVAL_MS;
  g_mv_timer = MINIVIEW_UPDATE_TIMEOUT;

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
  // The menu that was up may have been the one that changes the text size, and
  // that decides how the screen is divided before any of it is drawn
  if (g_geom.bar_scale != ((UI_SCALE_LARGE == config.ui_scale) ? 2 : 1))
    scope_ui_scale_changed();

  g_shadow_valid = false;
  g_sweep_force = true;
  g_vpos_marker.valid = false;
  g_trig_marker.valid = false;
  g_state = -1;
  g_toast_active = false;
  g_toast_timer = TIMER_DISABLE;

  lcd_set_font(FONT_LARGE);
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_COLOR);

  // The menu that was up may have moved the roll threshold under the current
  // timebase, and update_sample_rate() below times the ADC for one view or
  // the other
  roll_set_active(roll_wanted());

  draw_grid_frame();
  draw_vertical_position(false);
  draw_trigger_mode();
  draw_trigger_edge();
  draw_capture_state();
  draw_status_line();
  battery_redraw();
  update_sample_rate();
  redraw_trace();

  /*
   * ...and run the sweep out here, instead of leaving it to paint a column a
   * main-loop pass.
   *
   * What this function repaints is a screen something else covered, and the
   * usual something else is a menu - which is on top of the scope, so the
   * scope is not ticked at all while it is up. A trace left to the sweep is
   * then not a trace that arrives a moment later: it is a hole in the display
   * that stays there until the last menu closes. It showed as a black grid
   * behind an open menu every time a submenu was closed.
   *
   * It is the same three hundred columns either way, and the sweep that
   * follows redraws none of them: this leaves the shadows describing exactly
   * what was painted, which is what tells it there is nothing to do.
   */
  g_trace_column = 0;

  while (!trace_ready())
    draw_trace();

  // The editor's backdrop is its own, and this has just cleared the screen and
  // re-armed the sweep - which is exactly the path taken when the menu that
  // opened the editor closes over it
  if (g_layout_edit)
    layout_edit_refresh();

  // The panels are holes in the sweep, so what is under a closed menu comes
  // back only when they are painted - here, for the same reason as the trace.
  // The snapshot tag is normally put back by scope_task() when it sees a sweep
  // finish; the sweep above finished before it could see it start.
  if (g_snap_tag)
    snap_tag_paint();

  if (g_decode_mode)
    draw_decode_panel();

  if (g_fft_mode)
  {
    g_fft_panel_pending = true;

    if (g_fft_panel_on)
      draw_fft_panel();
  }
}

//-----------------------------------------------------------------------------
int scope_get_fps(void)
{
  return g_fps_value;
}

//-----------------------------------------------------------------------------
// Why the measurements panel is (or is not) on screen. Read this straight
// after the panel goes blank: `act` says whether the sweep composites it at
// all, `bld`/`pnt` say whether it is still being rebuilt and repainted, and
// `cell` is how many readings it last composed - zero with `act1` is the
// combination that means "the band is being drawn, and it is empty".
void scope_get_panel_state(char *buf, int size)
{
  snprintf(buf, (size_t)size, "d%d m%d act%d fft%d t%d bld%u pnt%u cell%d%s",
      config.measure_display ? 1 : 0, config.measure_panel_mode,
      g_mpanel_active ? 1 : 0, g_fft_mode ? 1 : 0, g_measure_timer,
      (unsigned)g_mpanel_builds, (unsigned)g_mpanel_paints,
      g_mpanel_cells, g_mpanel_is_text ? " text" : "");
}

//-----------------------------------------------------------------------------
void scope_task(void)
{
  if (trace_ready())
  {
    if (capture_buffer_updated())
    {
      g_fps_counter++;

      // The editor's trace is its own: a mock waveform where update_display()
      // would put whatever is on the probe. Re-laid every frame rather than
      // once, because a sweep that began before the editor opened paints its
      // first columns from the buffer as it was then - which showed as half a
      // screen of real trace beside half a screen of mock. The frame still has
      // to be consumed, or acquisition stalls behind it.
      if (g_layout_edit)
      {
        capture_consume_frame();
        layout_mock_trace();
        g_sweep_force = true;
      }
      else if (g_autocal_active)
        autocal_step();
      else if (scope_calibration_mode)
        draw_calibration_info();
      else if (g_autoset_active)
        autoset_step();
      else if (g_fft_mode)
        fft_tick();
      else if (g_trend_mode || g_roll_active)
        capture_consume_frame(); // keep acquisition flowing under the plot
      else
      {
        update_display();

        if (g_decode_mode)
          decode_update();
      }
    }
  }

  // The strip advances on wall clock, not on records: every pass, so a
  // column is closed within a main-loop pass of the time it is due
  if (g_roll_active)
    roll_task();

  // A cursor moved: repaint the sweep once the current one is done. The
  // overlay is composited into the columns, so the shadow cache is stale.
  if (g_cursor_dirty && trace_ready() && !g_fft_mode && !g_trend_mode)
  {
    g_cursor_dirty = false;
    g_shadow_valid = false;
    update_display();
  }

  // The trend logger ticks whether or not its view is open. A stopped scope
  // logs nothing: its record is frozen and the "measurements" would flatline.
  if (g_trend_timer == 0)
  {
    ScopeMeasure sm;

    g_trend_timer = TREND_INTERVAL_MS;

    if (capture_get_state() != CAPTURE_STATE_STOP && !scope_calibration_mode &&
        !g_autocal_active && capture_get_measurements(&sm))
    {
      trend_push(sm.frequency, sm.vrms_mv, sm.duty_x10);

      if (g_trend_mode)
      {
        trend_view_update();
        trend_readout();
      }
    }
  }

  // A RUN/STOP transition switches the active record (decimated storage vs
  // full-rate capture buffer): the decode and its positions must follow.
  // Only STOP matters here — capture_get_state() also flips WAIT<->TRIG on
  // every single acquisition, and reacting to that ran the whole decoder
  // cascade dozens of times a second with the throttle bypassed.
  //
  // Asked for from the menu, done here: the menu is a screen over the scope
  // and the scope does not draw while it is open
  if (g_spi_clock_request)
  {
    Measure m;
    char msg[48];

    g_spi_clock_request = false;

    // From the RAW measurement, not from the decoder: the probe is on a
    // clock line, a frequency counter is what reads clocks, and the decoder
    // has nothing to say about a signal nobody pointed it at.
    if (capture_get_raw_measure(&m) && m.frequency > 0)
    {
      config.spi_clock_hz = m.frequency;
      snprintf(msg, sizeof(msg), "SPI clock stored: %d Hz", m.frequency);
    }
    else
    {
      snprintf(msg, sizeof(msg), "SPI clock: nothing periodic to measure");
    }

    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
    mpanel_invalidate();
  }

  if (g_swd_clock_request)
  {
    Measure m;
    char msg[48];

    g_swd_clock_request = false;

    if (capture_get_raw_measure(&m) && m.frequency > 0)
    {
      config.swd_clock_hz = m.frequency;
      snprintf(msg, sizeof(msg), "SWD clock stored: %d Hz", m.frequency);
    }
    else
    {
      snprintf(msg, sizeof(msg), "SWD clock: nothing periodic to measure");
    }

    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
    mpanel_invalidate();
  }

  if (g_decode_hunt_request)
  {
    g_decode_hunt_request = false;

    if (!g_decode_mode)
    {
      decode_mode_enter(); // which arms the hunt itself when it can
      draw_status_line();
    }

    if (!g_decode_hunt)
      decode_arm_hunt();
  }

  // A stop may swap the record under the frozen trace (an AUTO stop
  // publishes the live ring, up to one sweep newer than the drawn frame;
  // see capture_stop_view): repaint from the actual frozen record NOW,
  // so the first pan/zoom does not silently morph the trace into it.
  {
    static int prev_stopped = -1;
    int stopped = (capture_get_state() == CAPTURE_STATE_STOP);

    if (stopped != prev_stopped && trace_ready())
    {
      prev_stopped = stopped;

      if (stopped && !g_fft_mode && !g_trend_mode && !scope_calibration_mode)
      {
        g_shadow_valid = false;
        update_display();
      }
    }
  }

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
  // re-run it — and the record it freezes may have swapped (an AUTO stop
  // publishes the raw ring, four times longer than the storage record the
  // running view analyzes; a NORMAL/SINGLE one keeps the snapshot).
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

  // The snapshot tag's lifecycle. On/off transitions cover stop/start from
  // every path (the button, the decode hunt, the glitch finder); the
  // sweep-completion repaint self-heals after anything that repainted the
  // whole grid over it (a view switch, the menu closing) - any such repaint
  // runs a sweep, and the sweep's hole spares the tag columns from then on.
  {
    static bool was_ready = true;
    bool ready = trace_ready();
    bool want = capture_stopped_on_snapshot() && !g_fft_mode &&
        !g_trend_mode && !g_roll_active && !scope_calibration_mode;

    if (want != g_snap_tag)
    {
      g_snap_tag = want;

      if (want)
        snap_tag_paint();
      else
        overlay_repaint_region(0, SNAP_TAG_H); // put the trace back
    }
    else if (want && ready && !was_ready)
      snap_tag_paint();

    was_ready = ready;
  }

  if (g_autoset_active && g_autoset_timer == 0)
    autoset_abort(); // ran out of time: put the user's settings back

  if (g_autocal_active && g_autocal_timer == 0)
    autocal_finish(false); // same, and say so rather than claim success

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

  // A filling record delivers no frames while it fills, so fft_tick() - which
  // is what normally repaints the panel - does not run at all, and the
  // progress line would sit frozen at whatever it read when the timebase
  // moved. Tick that ONE row here instead, off the wall clock rather than off
  // frames, and leave the rest of the panel where it is.
  if (g_fft_mode && g_fft_panel_on && !g_fft_panel_pending && fft_fill_pending())
  {
    static uint32_t last_ms = 0;

    if ((timer_ms() - last_ms) >= FFT_FILL_TICK_MS)
    {
      last_ms = timer_ms();
      lcd_set_font(FONT_SMALL);
      fft_fill_row_draw();
      lcd_set_font(FONT_LARGE);
    }
  }

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

  mpanel_set_active(mpanel_wanted());

  // The view, the metric selection and the spectrum key can all hand the
  // right-hand end of the status line from the trigger readouts to the
  // measurements and back. Neither side repaints on its own, so watch the
  // ownership itself: whoever holds it now has to paint it, and the loser's
  // glyphs have to go. A toast owns the whole line while it is up and
  // rebuilds it from scratch when it expires.
  if (measure_owns_status_line() != g_line_owner)
  {
    g_line_owner = measure_owns_status_line();

    if (!g_toast_active)
      draw_status_line();
  }

  // Put the current record in the record map. Only when the acquisition has
  // actually moved on: the strip costs a 24 KB scan and four blits, and at a
  // fast timebase this timer would otherwise pay both to redraw the same
  // picture. MV_Y < 0 is the Large layout, where there is no strip at all.
  if (g_mv_timer == 0)
  {
    g_mv_timer = MINIVIEW_UPDATE_TIMEOUT;

    // Rolling reads the on-screen strip rather than a record, and that strip
    // advances on the fold's schedule with nothing in the acquisition
    // generation to mark it: there the tick itself is the trigger to repaint
    if (MV_Y >= 0 && (g_roll_active || capture_get_generation() != g_mv_env_gen))
    {
      g_mv_env_valid = false;
      redraw_miniview();
    }
  }

  // measure_display can also be flipped from the system menu, which does not
  // know about our timer: keep the timer state in sync here
  if (config.measure_display && TIMER_DISABLE == g_measure_timer)
    g_measure_timer = 0;
  else if (!config.measure_display && TIMER_DISABLE != g_measure_timer)
    g_measure_timer = TIMER_DISABLE;

  // In calibration mode the panel band carries the hint instead, so the
  // measurement builder must not overwrite it
  if (config.measure_display && !g_fft_mode && !scope_calibration_mode &&
      !g_autocal_active)
  {
    if (g_measure_timer == 0)
    {
      g_measure_timer = MEASURE_UPDATE_TIMEOUT;

      // The two views are independent and can both be on: the panel lists a
      // set of metrics compactly, the status line puts two of them in the
      // large font. Repeating a value between them is the user's call.
      if (g_mpanel_active)
        mpanel_update();

      // ...but not over the editor's hint line, which is the only thing on
      // screen saying what the keys do there
      if (measure_owns_status_line() && !g_toast_active && !g_layout_edit)
        draw_measure();
    }
  }
}

