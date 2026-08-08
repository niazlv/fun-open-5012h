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

/*- Constants ---------------------------------------------------------------*/
const char *const hs_str[HS_COUNT] =
{
                            " 50\x01ns", // ns
  "100\x01ns", "200\x01ns", "500\x01ns",
  "  1\x01us", "  2\x01us", "  5\x01us", // us
  " 10\x01us", " 20\x01us", " 50\x01us",
  "100\x01us", "200\x01us", "500\x01us",
  "  1\x01ms", "  2\x01ms", "  5\x01ms", // ms
  " 10\x01ms", " 20\x01ms", " 50\x01ms",
  "100\x01ms", "200\x01ms", "500\x01ms",
  "  1\x01s ", "  2\x01s ", "  5\x01s ", // s - roll
  " 10\x01s ", " 20\x01s ", " 50\x01s ",
};
// 64-bit because of the roll end: 5 s/div is 5e9 ns and int stops at 2.1e9
const int64_t hs_div_value[HS_COUNT] =
{
  50, 100, 200, 500, // ns
  1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, // us
  1000000, 2000000, 5000000, 10000000, 20000000, 50000000, 100000000, 200000000, 500000000, // ms
  1000000000ll, 2000000000ll, 5000000000ll, // s
  10000000000ll, 20000000000ll, 50000000000ll,
};
const int hs_px_value[HS_COUNT] = // in ns
{
  2, 4, 8, 20, // ns
  40, 80, 200, 400, 800, 2000, 4000, 8000, 20000, // us
  40000, 80000, 200000, 400000, 800000, 2000000, 4000000, 8000000, 20000000, // ms
  40000000, 80000000, 200000000, // s
  400000000, 800000000, 2000000000,
};
const char *const vs_str[VS_COUNT] =
{
  " 50\x01mV", "100\x01mV", "200\x01mV", "500\x01mV", "  1\x01V ", "  2\x01V ", "  5\x01V ", " 10\x01V ",
};
const int vs_px_value[VS_COUNT] =
{
  2, 4, 8, 20, 40, 80, 200, 400,
};
//-----------------------------------------------------------------------------
// Millivolts per pixel at a scale, with the probe in it. Everything that maps
// between the screen and volts goes through here rather than reading the table,
// so a 10x probe cannot end up applied to half of the arithmetic - see
// config_probe_mult() for why that would shrink the trace instead of relabelling
// the axis.
int vs_mv_px(int scale)
{
  if (scale < 0 || scale >= VS_COUNT)
    scale = 0;

  return vs_px_value[scale] * config_probe_mult();
}

//-----------------------------------------------------------------------------
// The scale label, computed rather than looked up, because a probe may multiply
// it by anything in the list: 500 mV/div through a 1000x probe is 500 V/div, and
// no table of eight strings covers that. Reproduces vs_str[] exactly at 1x -
// same three digits, same half space, same two-character unit, same width - so
// nothing on this line moves when a probe is switched in.
const char *vs_label(int scale)
{
  static char buf[10];
  int mv_div = vs_mv_px(scale) * GRID_DIV_PX;

  if (mv_div < 1000)
    snprintf(buf, sizeof(buf), "%3d\x01mV", mv_div);
  else if (mv_div < 1000000)
    snprintf(buf, sizeof(buf), "%3d\x01V ", mv_div / 1000);
  else
    snprintf(buf, sizeof(buf), "%3d\x01kV", mv_div / 1000000);

  return buf;
}

// The stopped-on-a-snapshot tag is on screen (and its columns are a hole in
// the sweep). Owned by the state machine in scope_task().
bool g_snap_tag = false;
MarkerRect g_vpos_marker = { 0, 0, 0, 0, false };
MarkerRect g_trig_marker = { 0, 0, 0, 0, false };
bool g_toast_active = false;
int g_toast_timer = TIMER_DISABLE;
int g_state = -1;
int g_state_timer = TIMER_DISABLE;
// The rate the acquisition is running at, as the top bar last drew it.
// update_sample_rate() derives it from the timebase and the record length and
// nothing kept it afterwards; MEASURE_SRATE needs to be able to ask.
int g_sample_rate = 0;
// Last miniview geometry so it can repaint without a full update_sample_rate
static int g_mv_trigger_px = 0;
static int g_mv_window_px = -1;
static int g_mv_width_px = 3;
// Time the whole strip spans, ns. The strip's x axis is the capture RING, not
// the record: update_sample_rate() derives every offset above from
// period * CAPTURE_BUFFER_SIZE, and the trigger sits inside that at
// g_mv_trigger_px. Kept here so the envelope below can put its samples on the
// same axis the window frame is already drawn on.
static int64_t g_mv_span_ns = 0;
// Nothing else repaints the strip when a frame lands - it is redrawn from
// settings changes - so the signal in it would otherwise be whatever the record
// held the last time a key was pressed
int g_mv_timer = TIMER_DISABLE;
//-----------------------------------------------------------------------------
void toast_show(void)
{
  if (!g_toast_active)
    lcd_fill_rect(0, GRID_BOTTOM+1, LCD_WIDTH, STATUS_LINE_HEIGHT, BG_COLOR);

  lcd_set_color(BG_COLOR, TOAST_COLOR);
  g_toast_active = true;
  g_toast_timer = TOAST_TIMEOUT;
}

//-----------------------------------------------------------------------------
void draw_grid_frame(void)
{
  lcd_vline(10, 20, 220, GRID_FG_COLOR);
  lcd_vline(310, 20, 220, GRID_FG_COLOR);
  lcd_hline(10, 310, 20, GRID_FG_COLOR);
  lcd_hline(10, 310, 220, GRID_FG_COLOR);
}

//-----------------------------------------------------------------------------
void draw_ac_dc(void)
{
  if (g_toast_active)
    return;

  lcd_draw_image(g_geom.acdc_x, g_geom.acdc_y, config.ac_coupling ? &image_ac : &image_dc);
}

//-----------------------------------------------------------------------------
void draw_horizontal_scale(void)
{
  if (g_toast_active)
    return;

  lcd_set_color(BG_COLOR, HSCALE_COLOR);
  lcd_puts(g_geom.hscale_x, g_geom.hscale_y, hs_str[config.horizontal_scale]);
}

//-----------------------------------------------------------------------------
void draw_horizontal_position(void)
{
  char *str;

  if (g_toast_active || scope_calibration_mode || measure_owns_status_line())
    return;

  // The pan offset is always zero while rolling. What the same field is
  // worth there is how much time the screen holds - the one number a strip
  // chart has to be read against.
  str = format_time(g_roll_active ? roll_screen_ns() : config.horizontal_position,
      !g_roll_active);
  // Through the reading painter, in the second reading's slot: what stands here
  // when the measurements are off has to read the same as what stands here when
  // they are on
  measure_slot(1, MEASURE_SLOT_1_X, " ", str, HPOS_COLOR);
}

//-----------------------------------------------------------------------------
// Erase only the previous marker's bounding box instead of clearing the whole
// 209-px gutter on every key repeat
void draw_marker_image(MarkerRect *prev, int x, int y, const Image *image)
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
void draw_vertical_position(bool toast)
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
void draw_vertical_scale(void)
{
  const char *ratio = probe_ratio_labels[
      ((unsigned)config.probe_ratio < PROBE_RATIO_COUNT) ? config.probe_ratio : 0];

  if (g_toast_active)
    return;

  /*
   * It matters that the probe ratio is on screen and not just in the menu: the
   * label alone cannot distinguish 5 V/div from 500 mV/div through a 10x probe,
   * and with 25x and 100x probes in the list the difference is not guessable.
   *
   * Where it goes is the one thing the two layouts disagree about. At the large
   * size this field is in the TOP bar with the other settings, and the ratio is
   * a field of its own to the left of it - the arrangement the stock firmware
   * uses, and there is room for it there. At the normal size the top bar has
   * none at all (the trigger mode ends at 41, the capture state runs 46..77, the
   * record map starts at 81), so the ratio goes above the label in the small
   * font, two rows of 6x8 in the sixteen pixels the status line has.
   */
  if (g_geom.bar_scale > 1)
  {
    lcd_fill_rect(g_geom.vscale_x - 26, g_geom.vscale_y, VSCALE_FIELD_W + 26,
        16, BG_COLOR);

    if (config.x10)
    {
      lcd_set_color(BG_COLOR, PROBE_TAG_COLOR);
      lcd_puts(g_geom.vscale_x - 26, g_geom.vscale_y, ratio);
    }

    lcd_set_color(BG_COLOR, VSCALE_COLOR);
    lcd_puts(g_geom.vscale_x, g_geom.vscale_y, vs_label(config.vertical_scale));
    return;
  }

  lcd_fill_rect(g_geom.vscale_x, g_geom.vscale_y, VSCALE_FIELD_W, 16, BG_COLOR);

  if (config.x10)
  {
    lcd_set_font(FONT_SMALL);
    lcd_set_color(BG_COLOR, PROBE_TAG_COLOR);
    lcd_puts(g_geom.vscale_x, g_geom.vscale_y, ratio);
    lcd_puts(g_geom.vscale_x, g_geom.vscale_y + 8, vs_label(config.vertical_scale));
    lcd_set_font(FONT_LARGE);
    return;
  }

  lcd_set_color(BG_COLOR, VSCALE_COLOR);
  lcd_puts(g_geom.vscale_x, g_geom.vscale_y, vs_label(config.vertical_scale));
}

//-----------------------------------------------------------------------------
void draw_trigger_level(void)
{
  char *str;

  draw_marker_image(&g_trig_marker, GRID_RIGHT+2,
      GRID_CENTER_Y - config.trigger_level, &image_trigger_level);

  if (g_toast_active || scope_calibration_mode || measure_owns_status_line())
    return;

  str = format_voltage(config.trigger_level_mv - config.vertical_position_mv, true);
  // The edge icon used to sit at 140 and this readout began after it. With the
  // icon in the top bar the reading starts the field itself, on the same left
  // edge the first measurement slot uses.
  measure_slot(0, MEASURE_SLOT_0_X, " ", str, TRIGGER_LEVEL_COLOR);
}

//-----------------------------------------------------------------------------
// The icon lives in the top bar, not on the status line. On the status line it
// shared the space from x=140 with the measurements, the toasts and the
// calibration readout, so the setting the EDGE key changes was invisible for
// as long as any of them were up - which, with the measurements panel on, is
// all the time. Nothing else claims the top bar, so here it is always on.
//
// Calibration is the one exception, and not because of space: it drives the
// trigger itself (rise, auto, zero level) and leaves config.trigger_edge
// alone, so the icon would be naming an edge the hardware is not using. It
// clears the cell instead of lying about it.
void draw_trigger_edge(void)
{
  if (scope_calibration_mode)
  {
    lcd_fill_rect(TRIGGER_EDGE_X, TRIGGER_EDGE_Y, image_trigger_edge_both.width,
        image_trigger_edge_both.height, BG_COLOR);
    return;
  }

  if (TRIGGER_EDGE_RISE == config.trigger_edge)
    lcd_draw_image(TRIGGER_EDGE_X, TRIGGER_EDGE_Y, &image_trigger_edge_rise);
  else if (TRIGGER_EDGE_FALL == config.trigger_edge)
    lcd_draw_image(TRIGGER_EDGE_X, TRIGGER_EDGE_Y, &image_trigger_edge_fall);
  else
    lcd_draw_image(TRIGGER_EDGE_X, TRIGGER_EDGE_Y, &image_trigger_edge_both);
}

//-----------------------------------------------------------------------------
void draw_trigger_mode(void)
{
  char *str;

  if (TRIGGER_MODE_AUTO == config.trigger_mode)
    str = "AUTO";
  else if (TRIGGER_MODE_NORMAL == config.trigger_mode)
    str = "NORM";
  else
    str = "SNGL";

  lcd_set_color(BG_COLOR, TRIGGER_MODE_COLOR);
  lcd_puts(g_geom.mode_x, g_geom.mode_y, str);
}

//-----------------------------------------------------------------------------
// Bare value, no name: the two views label it themselves, and both need every
// reading to come out the same width so a shorter one erases the tail of the
// longer one it replaces
char *format_duty(int duty_x10)
{
  static char str[10];

  if (duty_x10 < 0)
    return " --.-%";

  snprintf(str, sizeof(str), "%3d.%d%%", duty_x10 / 10, duty_x10 % 10);

  return str;
}

//-----------------------------------------------------------------------------
// Picoseconds into the shortest sensible unit. Jitter spans six orders of
// magnitude here: 31 ps resolution at the fast end, milliseconds of spread
// on a noisy slow record.
// Five characters, always: two of these and a tilde are the jitter reading,
// and a figure that changes width as the signal settles used to drag the rest
// of the panel sideways with it. Three significant digits fit in five
// characters at every scale except one - 10..99 ns loses its tenth, which at
// that magnitude is under the record's own resolution anyway.
void format_ps(int ps, char *out, int size)
{
  char buf[12];

  if (ps < 0)
    snprintf(buf, sizeof(buf), "--");
  else if (ps < 1000)
    snprintf(buf, sizeof(buf), "%dps", ps);
  else if (ps < 10000)
    snprintf(buf, sizeof(buf), "%d.%dns", ps / 1000, (ps % 1000) / 100);
  else if (ps < 1000000)
    snprintf(buf, sizeof(buf), "%dns", ps / 1000);
  else if (ps < 1000000000)
    snprintf(buf, sizeof(buf), "%dus", ps / 1000000);
  else
    snprintf(buf, sizeof(buf), "%dms", ps / 1000000000);

  // An int of picoseconds runs out at 2.1 ms, so the widest this can be is
  // "999ps" / "999ns" / "999us" - five characters, which is the width above
  snprintf(out, size, "%*s", FORMAT_PS_W, buf);
}

//-----------------------------------------------------------------------------
// Repaint just a horizontal band of the trace area, immediately and without
// touching the column shadows: build_trace_column() composes from current
// state, so a later sweep of the same column produces identical pixels.
// ~10x cheaper than forcing a full 300-column sweep for an overlay change.
void overlay_repaint_region(int row0, int rows)
{
  uint16_t column[GRID_HEIGHT_MAX];
  int saved = g_trace_column;

  for (int c = 0; c < GRID_WIDTH - 1; c++)
  {
    // Never paint into the opaque decode panel or snapshot tag holes
    if (g_decode_mode && c >= DECODE_PANEL_COL0 && c <= DECODE_PANEL_COL1 &&
        row0 < DECODE_PANEL_H)
      continue;

    if (g_snap_tag && c >= SNAP_TAG_COL0 && c <= SNAP_TAG_COL1 &&
        row0 < SNAP_TAG_H)
      continue;

    // The trace half of a column is indexed by g_trace_column, not by the
    // argument - the sweep sets one and passes the other, and they are the same
    // number there. Here they are not, so every column would be composed over
    // the trace pixels of whichever column the sweep last drew. It went
    // unnoticed while this only ever repainted the panel band: those rows are
    // below the waveform most of the time, and where they were not, the smear
    // was under a dimmed overlay. The widget layout put readings anywhere in
    // the trace area and made it a flat line across the screen.
    g_trace_column = c;
    build_trace_column(c, column);
    lcd_draw_buf(GRID_LEFT+1 + c, GRID_TOP+1 + row0, 1, rows, &column[row0]);
  }

  g_trace_column = saved;
}

//-----------------------------------------------------------------------------
// The frozen record is only the storage snapshot: a small dim "rec <span>"
// in the trace corner says how much record there is to zoom into. Its
// columns are a hole in the sweep (see draw_trace), so painting once at
// activation - and again after anything that repaints the whole grid -
// keeps it on screen without per-frame cost.
void snap_tag_paint(void)
{
  const uint8_t *data;
  int size, offset, period_ns, trigger_timepos;
  int64_t span;
  char str[12];

  if (!capture_get_record(&data, &size, &offset, &period_ns, &trigger_timepos))
    return;

  span = (int64_t)size * period_ns;

  if (span < 1000000)
    snprintf(str, sizeof(str), "rec %dus", (int)(span / 1000));
  else if (span < 1000000000)
    snprintf(str, sizeof(str), "rec %dms", (int)(span / 1000000));
  else
    snprintf(str, sizeof(str), "rec %ds", (int)(span / 1000000000));

  lcd_fill_rect(GRID_LEFT+1 + SNAP_TAG_COL0, GRID_TOP+1, SNAP_TAG_W,
      SNAP_TAG_H, BG_COLOR);

  lcd_set_font(FONT_SMALL);
  lcd_set_color(BG_COLOR, SNAP_TAG_COLOR);
  lcd_puts(GRID_LEFT+1 + SNAP_TAG_COL1 - 1 - 6 * (int)strlen(str), GRID_TOP+2, str);
  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
// How many bytes the UTF-8 character starting here takes, 0 if this byte does
// not start a valid one (a continuation byte, a bad lead, or a sequence the
// record cut short). Only UTF-8: it is what serial ports carry, and it is the
// one encoding that can be RECOGNISED rather than guessed - its lead and
// continuation bytes are marked as such, so a valid sequence is evidence and
// not a hunch. UTF-16 or a code page would have to be inferred from a byte
// histogram, and being told the wrong language is worse than being told none.
//
// Nothing gets rendered from this - the font is 6x8 ASCII and always will be.
// But "these two bytes are one character, U+0416" is most of what a reader
// needs: it says the line is text in another language rather than a decode
// that has gone wrong, which is exactly what a screen full of dots looks like.
int utf8_len(const uint8_t *b, int n)
{
  int len;

  if (b[0] < 0x80)
    return 1;
  else if ((b[0] & 0xE0) == 0xC0)
    len = 2;
  else if ((b[0] & 0xF0) == 0xE0)
    len = 3;
  else if ((b[0] & 0xF8) == 0xF0)
    len = 4;
  else
    return 0; // a continuation byte, or 0xFE/0xFF which UTF-8 never uses

  if (len > n)
    return 0; // the rest of it is past the end of what was decoded

  for (int i = 1; i < len; i++)
  {
    if ((b[i] & 0xC0) != 0x80)
      return 0;
  }

  return len;
}

//-----------------------------------------------------------------------------
uint32_t utf8_code_point(const uint8_t *b, int len)
{
  static const uint8_t lead_mask[5] = { 0, 0x7F, 0x1F, 0x0F, 0x07 };
  uint32_t cp = b[0] & lead_mask[len];

  for (int i = 1; i < len; i++)
    cp = (cp << 6) | (b[i] & 0x3F);

  return cp;
}

//-----------------------------------------------------------------------------
// How many characters these bytes are. Fewer than there are bytes means the
// text is not plain ASCII - which is the honest way to say it about a mixed
// message: "UTF-8" alone claims the whole line is something other than ASCII
// when nearly all of it usually is, and says nothing about how much is not.
int utf8_char_count(const uint8_t *bytes, int count)
{
  int chars = 0;

  for (int i = 0; i < count; chars++)
  {
    int len = utf8_len(&bytes[i], count - i);

    i += (len > 0) ? len : 1;
  }

  return chars;
}

//-----------------------------------------------------------------------------
// Which character does byte `idx` belong to: where that character starts and
// how many bytes it takes. A byte that is part of no valid sequence is a
// character of its own, so every byte has an answer.
void utf8_group_at(const uint8_t *bytes, int count, int idx,
    int *start, int *len)
{
  *start = idx;
  *len = 1;

  for (int i = 0; i < count; )
  {
    int n = utf8_len(&bytes[i], count - i);

    if (n < 1)
      n = 1;

    if (idx < i + n)
    {
      *start = i;
      *len = n;
      return;
    }

    i += n;
  }
}

//-----------------------------------------------------------------------------
// Pixel width of a string in the current font: the half-space is half a glyph,
// so counting characters is not the same thing
int text_width(const char *str)
{
  int glyph = (FONT_LARGE)->width; // FONT_LARGE is &font, so -> needs the parens
  int w = 0;

  for (; *str; str++)
    w += (FONT_HALF_SPACE == *str) ? glyph / 2 : glyph;

  return w;
}

//-----------------------------------------------------------------------------
void draw_capture_state(void)
{
  int state = capture_get_state();
  int color = BG_COLOR;
  char *str = "";

  // Rolling has no trigger to wait for or catch, so WAIT/TRIG here would be
  // reporting on an acquisition the screen is not showing. This slot is the
  // only place the mode names itself.
  if (g_roll_active && CAPTURE_STATE_STOP != state)
    state = CAPTURE_STATE_ROLL;

  if (g_state == state)
    return;

  if (CAPTURE_STATE_ROLL == state)
  {
    color = ROLL_STATE_COLOR;
    str = "ROLL";
  }
  else if (CAPTURE_STATE_STOP == state)
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
  lcd_puts(g_geom.state_x, g_geom.state_y, str);

  g_state = state;
}

//-----------------------------------------------------------------------------
// x runs [-MINIVIEW_WIDTH/2+1, MINIVIEW_WIDTH/2), i.e. -79..79 => 159 columns
#define MINIVIEW_COLS  (MINIVIEW_WIDTH - 1)
#define MINIVIEW_ROWS  8
// Columns per blit. 40 splits the 159-wide strip into four, which costs four
// window set-ups a frame instead of one and holds 640 bytes instead of 2544.
#define MINIVIEW_CHUNK 40

// Rows the trace gets. 0 and MINIVIEW_ROWS-1 are the window frame's rails, and
// they are drawn after the trace, so the envelope stays off them: a signal that
// reached the rail row would read as the frame extending.
#define MINIVIEW_WAVE_TOP  1
#define MINIVIEW_WAVE_ROWS (MINIVIEW_ROWS - 2)

// Rebuild the envelope at 10 Hz at most. A sweep is 786 us at the fast end, so
// the acquisition generation changes far quicker than anyone can read a 159 px
// strip, and the scan below is ~24 KB of compares.

// Which decoded byte covers this miniview column? The miniview maps the
// whole record linearly onto its width, so record position p sits at
// x = (p/size - 1/2) * MINIVIEW_WIDTH.
static int miniview_byte_at(int x)
{
  // Held frames came out of a record that has already been overwritten, so
  // their positions mean nothing on this one: the panel keeps them, the map
  // does not
  if (!g_decode_mode || !g_logic_have || g_decode_held ||
      g_logic.count == 0 || g_decode_size <= 0)
    return -1;

  int64_t p = ((int64_t)x * g_decode_size) / MINIVIEW_WIDTH + g_decode_size / 2;

  for (int b = 0; b < g_logic.count; b++)
  {
    if (p >= g_logic.pos[b] && p < g_logic.end[b])
      return b;
  }

  return -1;
}

uint8_t g_mv_env[MINIVIEW_COLS];
bool g_mv_env_valid = false;
uint32_t g_mv_env_gen = 0;
//-----------------------------------------------------------------------------
// Strip column edge x to a sample index in the record. Both go through
// trigger-relative time, which is the one axis the strip, the window frame and
// the record all already agree on.
static int miniview_record_index(int x, int period_ns, int trigger_timepos)
{
  int64_t t = ((int64_t)(x - g_mv_trigger_px) * g_mv_span_ns) / MINIVIEW_WIDTH;
  int64_t j = t / period_ns;

  // Floor, not truncate: C rounds toward zero, and on the left half of the
  // strip that would round two adjacent column edges to the same index and
  // leave the column between them empty
  if (t < 0 && (t % period_ns) != 0)
    j -= 1;

  return (int)(j + trigger_timepos);
}

//-----------------------------------------------------------------------------
// A graticule row to a row inside the strip. Everything that lands in the strip
// goes through the display first, so the strip is scaled and positioned exactly
// like the trace under it, and a signal driven off screen pins to the strip's
// edge instead of being quietly rescaled to fit.
static int miniview_row_of(int display_row)
{
  int row = (display_row * MINIVIEW_WAVE_ROWS) / (GRID_HEIGHT - 1);

  if (row < 0)
    row = 0;
  else if (row >= MINIVIEW_WAVE_ROWS)
    row = MINIVIEW_WAVE_ROWS - 1;

  return MINIVIEW_WAVE_TOP + row;
}

//-----------------------------------------------------------------------------
// ...and a raw ADC count, by the route the swept trace takes to get there
static int miniview_row(int raw)
{
  return miniview_row_of(roll_row(raw));
}

//-----------------------------------------------------------------------------
/*
 * Rolling: the screen IS the record. roll_commit() keeps exactly GRID_WIDTH
 * columns and drops the oldest one out of the left of them, so there is no
 * history behind the screen for a record map to reveal - and the window frame
 * is pinned open across the whole strip to say so.
 *
 * So this draws the screen, squeezed. The redundancy is the message: the
 * miniature matching the trace column for column is what makes the full-width
 * frame legible as "the window covers everything" rather than as an empty box.
 * Columns the roll has not reached yet stay empty, so the strip fills in from
 * the right exactly as the trace does.
 */
static void miniview_envelope_roll(void)
{
  for (int cx = 0; cx < MINIVIEW_COLS; cx++)
  {
    int c0 = (cx * GRID_WIDTH) / MINIVIEW_COLS;
    int c1 = ((cx + 1) * GRID_WIDTH) / MINIVIEW_COLS;
    int top = GRID_HEIGHT, bot = -1;

    for (int c = c0; c < c1; c++)
    {
      if (!(g_roll_row_flags[c] & SAMPLE_FLAG_VALID))
        continue;

      // Named for the counts they came from, not the rows they hold: a bigger
      // count is a HIGHER pixel, so _min is the top row (see roll_commit)
      if (g_roll_row_min[c] < top)
        top = g_roll_row_min[c];

      if (g_roll_row_max[c] > bot)
        bot = g_roll_row_max[c];
    }

    if (bot >= 0)
      g_mv_env[cx] = (uint8_t)((miniview_row_of(top) << 4) | miniview_row_of(bot));
  }
}

//-----------------------------------------------------------------------------
static void miniview_envelope_build(void)
{
  const uint8_t *data;
  int size, offset, period_ns, trigger_timepos;

  memset(g_mv_env, MV_ENV_NONE, sizeof(g_mv_env));

  g_mv_env_valid = true;
  g_mv_env_gen = capture_get_generation();

  if (g_roll_active)
  {
    miniview_envelope_roll();
    return;
  }

  if (g_mv_span_ns <= 0)
    return;

  if (!capture_get_record(&data, &size, &offset, &period_ns, &trigger_timepos) ||
      period_ns <= 0 || size <= 0)
    return;

  for (int cx = 0; cx < MINIVIEW_COLS; cx++)
  {
    int x = cx - MINIVIEW_WIDTH/2 + 1;
    int j0 = miniview_record_index(x, period_ns, trigger_timepos);
    int j1 = miniview_record_index(x + 1, period_ns, trigger_timepos);
    int vmin = 255, vmax = 0;
    int idx;

    if (j0 < 0)
      j0 = 0;

    if (j1 > size)
      j1 = size;

    if (j0 >= j1)
      continue;   // no record under this column

    idx = offset + j0;

    if (idx >= size)
      idx -= size;

    for (int j = j0; j < j1; j++)
    {
      int v = data[idx];

      if (v < vmin)
        vmin = v;

      if (v > vmax)
        vmax = v;

      if (++idx == size)
        idx = 0;
    }

    // Bigger count is higher on screen, i.e. the SMALLER row number
    g_mv_env[cx] = (uint8_t)((miniview_row(vmax) << 4) | miniview_row(vmin));
  }
}

//-----------------------------------------------------------------------------
void draw_miniview(int trigger_offset, int window_offset, int window_width)
{
  // No row for it at the large size: the settings the stock layout puts in the
  // top bar need the width this was using. See ScopeGeom.
  if (MV_Y < 0)
    return;

  // Cheap when it is already current: a decoder selection moving repaints this
  // strip without the record under it having changed
  if (!g_mv_env_valid)
    miniview_envelope_build();

  // Rendered row-major in column chunks, one blit each.
  //
  // It started as 158 separate one-pixel-wide lcd_draw_buf calls, each paying
  // ~40% window-setup overhead, and became one buffer for the whole strip -
  // which fixed that and then stood in 2.5 KB of TCM for the 99% of the time
  // the miniview is not being drawn. Four blits keep essentially all of the
  // saving (four window set-ups against 158) and give back four fifths of the
  // memory. The chunk is a width and not a height because the fill is
  // column-major: each column's eight pixels are computed together, and
  // splitting by ROWS would mean walking every column eight times.
  static uint16_t buf[MINIVIEW_ROWS * MINIVIEW_CHUNK];

  for (int base = 0; base < MINIVIEW_COLS; base += MINIVIEW_CHUNK)
  {
    // The last chunk is short, and its width is the buffer's STRIDE for the
    // blit as well as its loop bound - the two cannot drift apart
    int w = MINIVIEW_COLS - base;

    if (w > MINIVIEW_CHUNK)
      w = MINIVIEW_CHUNK;

    for (int k = 0; k < w; k++)
    {
      int cx = base + k;
      int x = cx - MINIVIEW_WIDTH/2 + 1;
      bool inside = ((x > window_offset) && (x < (window_offset + window_width)));
      bool edge = ((x == window_offset) || (x == (window_offset + window_width - 1)));
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
          // instead of the signal
          uint16_t color = (byte_here == g_decode_sel) ? DSTRIP_SEL :
              ((byte_here & 1) ? DSTRIP_ODD : DSTRIP_EVEN);

          for (int i = 2; i <= 5; i++)
            col[i] = color;
        }
        else if (g_mv_env[cx] != MV_ENV_NONE)
        {
          // The record's own envelope for this column, min to max, on the same
          // vertical scale as the trace. Same colour rule the zigzag had: lit
          // inside the window, frame-grey outside, so the strip still reads as
          // "this part is on screen" at a glance.
          uint16_t color = inside ? TRACE_COLOR : MV_FRAME_COLOR;
          int bot = g_mv_env[cx] & 0x0f;

          for (int i = g_mv_env[cx] >> 4; i <= bot; i++)
            col[i] = color;
        }

        if (inside)
        {
          col[0] = MV_FRAME_COLOR;
          col[MINIVIEW_ROWS-1] = MV_FRAME_COLOR;
        }
      }

      for (int i = 0; i < MINIVIEW_ROWS; i++)
        buf[i * w + k] = col[i];
    }

    lcd_draw_buf(GRID_CENTER_X - MINIVIEW_WIDTH/2 + 1 + base, MV_Y + 6, w,
        MINIVIEW_ROWS, buf);
  }

#define LEFT   (GRID_CENTER_X - MINIVIEW_WIDTH/2)
#define RIGHT  (GRID_CENTER_X + MINIVIEW_WIDTH/2)

  lcd_fill_rect(LEFT - image_trigger_mv.width/2, MV_Y, MINIVIEW_WIDTH + image_trigger_mv.width,
      image_trigger_mv.height, BG_COLOR);
  lcd_draw_image(GRID_CENTER_X + trigger_offset, MV_Y + 4, &image_trigger_mv);

  lcd_vline(LEFT, MV_Y + 5, MV_Y + 14, MV_FRAME_COLOR);
  lcd_hline(LEFT, LEFT+2, MV_Y + 5, MV_FRAME_COLOR);
  lcd_hline(LEFT, LEFT+2, MV_Y + 14, MV_FRAME_COLOR);

  lcd_vline(RIGHT, MV_Y + 5, MV_Y + 14, MV_FRAME_COLOR);
  lcd_hline(RIGHT-2, RIGHT, MV_Y + 5, MV_FRAME_COLOR);
  lcd_hline(RIGHT-2, RIGHT, MV_Y + 14, MV_FRAME_COLOR);

#undef LEFT
#undef RIGHT
}

//-----------------------------------------------------------------------------
void draw_sample_rates(int sample_rate_limit, int sample_rate)
{
  char *str;

  // Kept for MEASURE_SRATE: the rate is derived here, from the timebase and the
  // record, and nothing else holds on to it afterwards
  g_sample_rate = sample_rate;

  lcd_set_font(FONT_SMALL);

  str = format_sps(sample_rate_limit);
  lcd_set_color(BG_COLOR, SR_LIMIT_COLOR);
  lcd_puts(SAMPLE_RATE_X, 2, str);

  str = format_sps(sample_rate);
  lcd_set_color(BG_COLOR, SR_COLOR);
  lcd_puts(SAMPLE_RATE_X, 10, str);

  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
void update_sample_rate(void)
{
  int64_t hp_abs = (config.horizontal_position < 0) ? -config.horizontal_position : config.horizontal_position;

  // Every path that re-times the acquisition or moves the view comes through
  // here: whatever the envelope and the averages accumulated no longer maps
  // onto these columns
  scope_display_settings_changed();

  // Rolling: the screen is not a record and the rate cannot be derived from
  // it. Asking for one would not just be pointless, it would be wrong - the
  // loop below stretches the sample period until a record spans the screen,
  // and a minute-wide screen walks the divider straight past the 16 bits the
  // sampling timer's prescaler has. Pin the rate instead (see ROLL_SR_DIVIDER)
  // and let the whole strip read as the window it is.
  if (g_roll_active)
  {
    int sr = (config.sample_rate_limit > ROLL_SR_DIVIDER) ?
        config.sample_rate_limit : ROLL_SR_DIVIDER;
    int rate = (int)(BASE_SAMPLE_RATE / (1 << sr));

    capture_set_horizontal_parameters(sr, CAPTURE_BUFFER_SIZE/2);

    g_mv_trigger_px = 0;
    g_mv_window_px = -MINIVIEW_WIDTH/2;
    g_mv_width_px = MINIVIEW_WIDTH;
    g_mv_span_ns = 0;   // no record behind the strip; the envelope stays empty
    g_mv_env_valid = false;
    draw_miniview(g_mv_trigger_px, g_mv_window_px, g_mv_width_px);

    // The lower readout is the ring's own rate, undecimated: the fold reads
    // the ring itself, not the storage record every other view goes through
    draw_sample_rates((int)(BASE_SAMPLE_RATE / (1 << config.sample_rate_limit)),
        rate);
    return;
  }

  int64_t window_time = (int64_t)hs_div_value[config.horizontal_scale] * GRID_DIVS_H;
  int64_t period = BASE_SAMPLE_PERIOD;
  int64_t trigger_margin, trigger_offset;
  int64_t buffer_time, required_time;
  int64_t window_offset;
  int64_t denom;
  int sample_rate = BASE_SAMPLE_RATE;
  int sample_rate_limit, record_period;
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

    // ...and the prescaler runs out before the slow timebases do. Past here
    // the record simply spans less than the screen asks for, which is a fact
    // about the instrument; stretching the period past the register would be
    // a rate nothing on screen reports (see MAX_SR_DIVIDER).
    if (sr_divider >= MAX_SR_DIVIDER)
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

  // Same denom the three offsets above were scaled by: the envelope has to land
  // on the axis they were placed on, or the signal and the frame around it
  // would be drawn to different rulers
  g_mv_span_ns = denom;
  g_mv_env_valid = false;

  draw_miniview(trigger_offset_px, window_offset_px, window_width_px);

  // Report the rate the record is actually captured at, not the ring's: the
  // two differ by 4x whenever the screen needs more time than the full-rate
  // window spans (see update_storage_window in capture.c), and that is the
  // rate that decides whether an input aliases
  record_period = capture_get_pending_record_period();

  if (record_period > 0)
    sample_rate = (int)(1000000000ll / record_period);

  draw_sample_rates(sample_rate_limit, sample_rate);

  // Every pan and every timebase step lands here, and both move the decoded
  // bytes across the screen: the band is anchored in time, not in columns
  decode_band_build();
}

//-----------------------------------------------------------------------------
// Repaint the miniview with its last geometry (decode results changed, the
// selection moved) without the DMA restart update_sample_rate() would do
void redraw_miniview(void)
{
  draw_miniview(g_mv_trigger_px, g_mv_window_px, g_mv_width_px);
}

//-----------------------------------------------------------------------------
void format_ns(int64_t ns, char *out, int size)
{
  int64_t a = (ns < 0) ? -ns : ns;
  const char *sign = (ns < 0) ? "-" : "";

  if (a < 1000)
    snprintf(out, size, "%s%dns", sign, (int)a);
  else if (a < 1000000)
    snprintf(out, size, "%s%d.%02dus", sign, (int)(a / 1000), (int)(a % 1000) / 10);
  else if (a < 1000000000)
    snprintf(out, size, "%s%d.%02dms", sign, (int)(a / 1000000), (int)(a % 1000000) / 10000);
  else
    snprintf(out, size, "%s%d.%02ds", sign, (int)(a / 1000000000),
        (int)((a % 1000000000) / 10000000));
}

//-----------------------------------------------------------------------------
// Frequency from milli-hertz: cursor spans run from sub-Hz (dt = 2 s) up to
// tens of MHz (dt = 50 ns), so the input must not lose the fractional Hz
void format_hz_mhz(int64_t mhz, char *out, int size)
{
  if (mhz >= 1000000000)
    snprintf(out, size, "%d.%03dM", (int)(mhz / 1000000000),
        (int)(mhz % 1000000000) / 1000000);
  else if (mhz >= 1000000)
    snprintf(out, size, "%d.%02dk", (int)(mhz / 1000000), (int)(mhz % 1000000) / 10000);
  else
    snprintf(out, size, "%d.%02d", (int)(mhz / 1000), (int)(mhz % 1000) / 10);
}

//-----------------------------------------------------------------------------
void format_mv(int mv, char *out, int size)
{
  int a = (mv < 0) ? -mv : mv;
  const char *sign = (mv < 0) ? "-" : "";

  if (a < 1000)
    snprintf(out, size, "%s%dmV", sign, a);
  else
    snprintf(out, size, "%s%d.%02dV", sign, a / 1000, (a % 1000) / 10);
}

//-----------------------------------------------------------------------------
void draw_status_line(void)
{
  // The layout editor owns the line too, and for the same reason: what the keys
  // do there is not what they do here
  if (g_layout_edit)
  {
    layout_edit_footer();
    return;
  }

  // The trend view and the cursors each own the whole line while they are
  // up; both repaint it themselves, toast expiry included
  if (g_trend_mode)
  {
    trend_readout();
    return;
  }

  if (g_cursor_sel && !g_fft_mode && !g_decode_mode)
  {
    cursor_readout();
    return;
  }

  lcd_fill_rect(0, GRID_BOTTOM+1, LCD_WIDTH, STATUS_LINE_HEIGHT, BG_COLOR);

  draw_vertical_scale();
  draw_ac_dc();
  draw_horizontal_scale();
  draw_horizontal_position();
  draw_trigger_level();
  draw_measure();
}

