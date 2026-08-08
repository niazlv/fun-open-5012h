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

/*- Variables and implementations -------------------------------------------*/
int g_layout_sel = 0;      // which slot the arrows act on
bool g_layout_grab = false; // ...and whether they move it or the choice
//-----------------------------------------------------------------------------
// Plausible readings for the editor. Arranging a layout against numbers that
// jump - or against a probe with nothing on it, where half the metrics read
// "--" and the grey makes them hard to see - is arranging blind. A 10 kHz,
// 2 Vpp, 50% square is what the mock trace behind them draws.
void layout_mock_measure(ScopeMeasure *sm)
{
  memset(sm, 0, sizeof(*sm));

  sm->vpp_mv  = 2000;
  sm->vamp_mv = 1980;
  sm->vtop_mv = 1000;
  sm->vbase_mv = -1000;
  sm->vrms_mv = 1000;
  sm->vavg_mv = 0;
  sm->vmax_mv = 1000;
  sm->vmin_mv = -1000;
  sm->vp_mv   = 1000;
  sm->vmid_mv = 0;
  sm->frequency     = 10000;
  sm->duty_x10      = 500;
  sm->period_med_ns = 100000;
  sm->period_min_ns = 99800;
  sm->period_max_ns = 100200;
  sm->width_pos_ns  = 50000;
  sm->width_neg_ns  = 50000;
  sm->jitter_rms_ps = 1200;
  sm->jitter_pp_ps  = 8000;
  sm->periods       = 80;
  sm->period_good_pct = 100;
  sm->level_pct     = 90;
}

//-----------------------------------------------------------------------------
// A 10 kHz-looking square at 50% into the display buffer, which is what the
// mock readings describe. Pixel space, so it needs no acquisition and no
// timebase: this is a backdrop to arrange against, not a measurement.
#define LAYOUT_MOCK_PERIOD   60
#define LAYOUT_MOCK_AMP      45

void layout_mock_trace(void)
{
  for (int c = 0; c < GRID_WIDTH; c++)
  {
    int phase = c % LAYOUT_MOCK_PERIOD;
    bool high = phase < LAYOUT_MOCK_PERIOD / 2;
    bool edge = (0 == phase) || (LAYOUT_MOCK_PERIOD / 2 == phase);
    int top = GRID_CENTER_Y - LAYOUT_MOCK_AMP;
    int bottom = GRID_CENTER_Y + LAYOUT_MOCK_AMP;

    // An edge column spans both levels, which is how a real record draws a
    // transition too
    g_display_buffer.min[c] = (uint8_t)(edge ? top : (high ? top : bottom));
    g_display_buffer.max[c] = (uint8_t)(edge ? bottom : (high ? top : bottom));
    g_display_buffer.flags[c] = SAMPLE_FLAG_VALID;
  }
}

//-----------------------------------------------------------------------------
// Lay the band's own readings out as widgets, in the places the band had them.
// That is the layout the user is already looking at, so the editor opens on
// something familiar rather than on an empty screen - and AUTO brings it back
// when an arrangement has gone wrong.
static void layout_seed_from_band(void)
{
  ScopeMeasure sm;
  MeasureItem items[MEASURE_ITEMS_MAX];
  const Font *font = mpanel_font();
  int sc = mpanel_scale();
  int cell = font->width * sc;
  int gh = font->height * sc;
  int chars = mpanel_chars();
  int row_h = mpanel_row_h();
  int rows = mpanel_rows();
  int band_y = GRID_HEIGHT - 1 - (2 * MPANEL_PAD_Y + rows * row_h -
      MPANEL_GAP_Y) + MPANEL_PAD_Y;
  uint8_t size_flags = (PANEL_FONT_HUGE == config.measure_panel_font) ? PW_HUGE :
      ((PANEL_FONT_LARGE == config.measure_panel_font) ? PW_LARGE : 0);
  int n, placed = 0, x = 0, row = 0;

  layout_mock_measure(&sm);
  n = measure_build_items(&sm, items);

  memset(config.measure_widget, 0, sizeof(config.measure_widget));

  for (int i = 0; i < n && placed < PANEL_WIDGETS_MAX; i++)
  {
    PanelWidget *w = &config.measure_widget[placed];
    int len = (int)strlen(items[i].label) + 1 + (int)strlen(items[i].value);

    if (x + len > chars)
    {
      if (++row >= rows)
        break;

      x = 0;
    }

    w->metric = (uint8_t)items[i].metric;
    w->flags = size_flags;

    // Through the anchor, so that the row the band put along the bottom stays
    // along the bottom - and so that a reading in the right half is measured
    // from the right edge, which is what it will grow towards later
    widget_set_pos(w, MPANEL_PAD_X + x * cell, band_y + row * row_h,
        len * cell, gh);

    x += len + MPANEL_GUTTER;
    placed++;
  }
}

//-----------------------------------------------------------------------------
// The next slot that holds something, wrapping, so the selection walks the
// readings rather than the array's holes
static void layout_select_step(int dir)
{
  for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
  {
    g_layout_sel = (g_layout_sel + dir + PANEL_WIDGETS_MAX) % PANEL_WIDGETS_MAX;

    if (MEASURE_NONE != config.measure_widget[g_layout_sel].metric)
      return;
  }
}

//-----------------------------------------------------------------------------
// Which reading a widget shows, stepped through the metrics. MEASURE_NONE is
// skipped: a widget showing nothing would be an invisible thing to be holding.
static void layout_metric_step(PanelWidget *w, int dir)
{
  int m = w->metric + dir;

  if (m <= MEASURE_NONE)
    m = MEASURE_COUNT - 1;
  else if (m >= MEASURE_COUNT)
    m = MEASURE_NONE + 1;

  w->metric = (uint8_t)m;
}

//-----------------------------------------------------------------------------
// Move the selected widget, clamped so it cannot be pushed off the screen and
// lost. The text's own width decides the right-hand limit, which is why this
// asks g_placed rather than the config: that is where the composed width is.
static void layout_move(int dx, int dy)
{
  PanelWidget *w = layout_selected();
  const PanelPlaced *p = &g_placed[g_layout_sel];
  int size = widget_size_of(w->flags);
  const Font *font = widget_font(size);
  int sc = widget_scale(size);
  int wide = (p->len ? p->len : 8) * font->width * sc;
  int gh = font->height * sc;
  int max_x = GRID_WIDTH - 2 - wide;
  int max_y = GRID_HEIGHT - 2 - gh;
  int x, y;

  // In screen pixels, not in the stored units: the arrows move a reading across
  // the screen, and which corner that ends up being measured from is decided
  // afterwards, by where it lands
  widget_pos(w, wide, gh, &x, &y);

  x += dx * PANEL_WIDGET_STEP;
  y += dy * PANEL_WIDGET_STEP;

  if (max_x < 0)
    max_x = 0;

  if (max_y < 0)
    max_y = 0;

  if (x < 0)
    x = 0;
  else if (x > max_x)
    x = max_x;

  if (y < 0)
    y = 0;
  else if (y > max_y)
    y = max_y;

  widget_set_pos(w, x, y, wide, gh);
}

//-----------------------------------------------------------------------------
// One line of key hints where the status line normally is. The editor owns that
// line: what the arrows do here changes with whether a widget is held, and a
// screen full of readings with no way to know that is a puzzle.
void layout_edit_footer(void)
{
  char buf[64];

  // Two rows of the 6x8 font in the sixteen pixels the status line has, which
  // is the only way all of the keys fit: 51 characters to a row here.
  //
  // Exactly the rectangle draw_status_line() clears, and no wider: a footer
  // that reached x=0 left its first letters standing in the margin the status
  // line does not own when the editor closed.
  lcd_fill_rect(0, GRID_BOTTOM + 1, LCD_WIDTH, STATUS_LINE_HEIGHT, BG_COLOR);
  lcd_set_font(FONT_SMALL);
  lcd_set_color(BG_COLOR, MEASURE_MODE_COLOR);

  if (g_layout_grab)
  {
    snprintf(buf, sizeof(buf), "HOLDING %s", g_placed[g_layout_sel].text);
    lcd_puts(GRID_LEFT + 2, STATUS_LINE_Y, buf);
    lcd_puts(GRID_LEFT + 2, STATUS_LINE_Y + 8,
        "arrows move it   MODE put it down");
  }
  else
  {
    lcd_puts(GRID_LEFT + 2, STATUS_LINE_Y,
        "L/R pick   U/D metric   MODE take   F1 size");
    lcd_puts(GRID_LEFT + 2, STATUS_LINE_Y + 8,
        "EDGE add   50% delete   AUTO tidy   SAVE done");
  }

  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
// Redraw everything the editor owns: the mock trace under the widgets, the
// widgets themselves (built from the mock readings), and the hint line.
void layout_edit_refresh(void)
{
  ScopeMeasure sm;

  layout_mock_measure(&sm);
  layout_mock_trace();
  g_placed_n = 0;            // force the rebuild past its own compare
  widgets_update(&sm);
  layout_edit_footer();
}

//-----------------------------------------------------------------------------
void scope_layout_edit_start(void)
{
  // An empty layout would be an empty screen to arrange nothing on, so the
  // band's own arrangement is the starting point
  if (0 == layout_used())
    layout_seed_from_band();

  config.measure_layout_mode = PANEL_LAYOUT_WIDGETS;
  g_layout_edit = true;
  g_layout_grab = false;
  g_layout_sel = 0;

  if (MEASURE_NONE == config.measure_widget[0].metric)
    layout_select_step(1);

  g_mpanel_is_text = false;
  g_mpanel_active = true;
  mpanel_invalidate();
  layout_edit_refresh();
}

//-----------------------------------------------------------------------------
static void layout_edit_finish(void)
{
  g_layout_edit = false;
  g_layout_grab = false;

  // Back to the real record and the real readings. The layout itself stays as
  // it was arranged - it is in config, so the store writes it out on its own.
  mpanel_invalidate();
  refresh_view();
  draw_status_line();
}

//-----------------------------------------------------------------------------
// The editor owns the keyboard while it is up, the way auto-calibration does:
// every key here means something else than it does over a live trace.
void layout_edit_keys(int buttons)
{
  PanelWidget *w = layout_selected();

  // SAVE and not MENU: the launcher takes MENU for the system menu before an
  // application sees it (a plain press opens the menu over whatever is on
  // screen), so the editor never gets to hear about it. Closing that menu
  // repaints straight back into the editor, which is at least consistent.
  if (buttons & BTN_SAVE)
  {
    layout_edit_finish();
    return;
  }

  if (buttons & BTN_MODE)
  {
    g_layout_grab = !g_layout_grab;
  }
  else if (buttons & BTN_F1)
  {
    // 6x8 -> 8x16 -> 16x32 -> 6x8. Three steps where there used to be a toggle,
    // because the third one is the point of the screen: it is the only size that
    // is bigger than anything the two bars can hold.
    int size = (widget_size_of(w->flags) + 1) % 3;

    w->flags &= (uint8_t)~PW_SIZE_MASK;
    w->flags |= (1 == size) ? PW_LARGE : ((2 == size) ? PW_HUGE : 0);

    // Growing one can push it off the right edge or the bottom; the clamp lives
    // in the move, so ask for a move of nothing
    layout_move(0, 0);
  }
  else if (buttons & BTN_EDGE)
  {
    // A new reading in the first free slot, at the top left corner - which is
    // empty in every layout that came from the band - and already held, so the
    // arrows carry it away from there without another keypress. Anywhere near
    // the widget it was added from would land on top of something.
    for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
    {
      if (MEASURE_NONE != config.measure_widget[i].metric)
        continue;

      config.measure_widget[i].metric = (uint8_t)layout_unused_metric();
      config.measure_widget[i].x = 1;
      config.measure_widget[i].y = 1;
      // The size of the one it was added from, but never its corner: x and y
      // here mean the top left one, and inheriting a bottom-right anchor would
      // put the new reading in the opposite corner from where it says it is
      config.measure_widget[i].flags = w->flags & PW_SIZE_MASK;
      g_layout_sel = i;
      g_layout_grab = true;
      break;
    }
  }
  else if (buttons & BTN_50P)
  {
    // ...and never the last one: an empty layout is a screen with no readings
    // and no way back to them except the menu
    if (layout_used() > 1)
    {
      w->metric = MEASURE_NONE;
      layout_select_step(1);
    }
  }
  else if (buttons & BTN_AUTO)
  {
    layout_seed_from_band();
    g_layout_sel = 0;
    g_layout_grab = false;
  }
  else if (g_layout_grab)
  {
    if (buttons & BTN_LEFT)
      layout_move(-1, 0);
    else if (buttons & BTN_RIGHT)
      layout_move(1, 0);
    else if (buttons & BTN_UP)
      layout_move(0, -1);
    else if (buttons & BTN_DOWN)
      layout_move(0, 1);
  }
  else
  {
    if (buttons & BTN_LEFT)
      layout_select_step(-1);
    else if (buttons & BTN_RIGHT)
      layout_select_step(1);
    else if (buttons & BTN_UP)
      layout_metric_step(w, 1);
    else if (buttons & BTN_DOWN)
      layout_metric_step(w, -1);
  }

  layout_edit_refresh();
}

