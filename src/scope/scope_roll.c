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
static int  g_roll_col_us = 0;    // time one display column covers
static int  g_roll_acc_us = 0;    // ...and how much of it is in the open one
static uint32_t g_roll_stamp = 0; // timer_us() at the previous fold
static int  g_roll_min = 255;     // open column's envelope, raw ADC counts
static int  g_roll_max = 0;
//-----------------------------------------------------------------------------

// The timebase the user's roll threshold starts at. Index 0 is the default
// and means "only where a sweep cannot work at all", which is what the stock
// scope does; the rest pull roll down into timebases a sweep can still show,
// where it trades the trigger for a screen that keeps moving.
static int roll_threshold_hs(void)
{
  static const uint8_t from[] =
      { HS_ROLL_FIRST, HS_500_ms, HS_200_ms, HS_100_ms };

  int sel = config.roll_from;

  if (sel < 0 || sel >= (int)ARRAY_SIZE(from))
    sel = 0;

  return from[sel];
}

//-----------------------------------------------------------------------------
// Roll is a property of the timebase, not a mode with a key. The other views
// own the whole trace area and say what it means themselves, so none of them
// rolls: the spectrum has no time axis, the trend plots its own log, and a
// decoder needs the record roll refuses to wait for.
bool roll_wanted(void)
{
  if (g_fft_mode || g_trend_mode || g_decode_mode || scope_calibration_mode ||
      g_autocal_active || g_autoset_active)
    return false;

  return config.horizontal_scale >= roll_threshold_hs();
}

//-----------------------------------------------------------------------------
// One display column's worth of time. Column, not division: 25 px per
// division is the whole resolution of the plot, and a column is what the
// envelope is accumulated into.
static int roll_column_us(void)
{
  int64_t us = hs_div_value[config.horizontal_scale] / (GRID_DIV_PX * 1000);

  return (us < 1) ? 1 : (int)us;
}

//-----------------------------------------------------------------------------
// Raw ADC count to display row, by the same route the swept trace takes:
// counts to millivolts through the range's calibration, millivolts to pixels
// through the volts/div, then the user's vertical position.
int roll_row(int raw)
{
  int mv = ((raw - ZERO_POINT) * config.vertical_mult + config.vertical_mult/2) /
      CALIB_MULTIPLIER;

  return clip_for_display((mv - config.vertical_position_mv) /
      vs_mv_px(config.vertical_scale) + config.vertical_position);
}

//-----------------------------------------------------------------------------
// The strip onto the screen. The columns are already in display space, so
// this is a copy - and it is a copy rather than the plot living there in the
// first place because the display buffer belongs to whatever view is up.
void roll_publish(void)
{
  if (!g_roll_active)
    return; // another view owns the columns; the strip waits its turn

  memcpy(g_display_buffer.min, g_roll_row_min, GRID_WIDTH);
  memcpy(g_display_buffer.max, g_roll_row_max, GRID_WIDTH);
  memcpy(g_display_buffer.flags, g_roll_row_flags, GRID_WIDTH);

  g_shadow_valid = false;
  redraw_trace();
}

//-----------------------------------------------------------------------------
// Start the open column over and put the whole strip back on screen. Every
// path that rewrites the stored columns ends here.
//
// Deliberately does NOT restamp the clock. The strip is anchored to wall
// clock, and time that passed while the view was elsewhere is time the input
// was not watched: roll_task shifts blank columns in for it, so a break in
// the recording is drawn as a break rather than closed up.
static void roll_reopen(void)
{
  g_roll_min   = 255;
  g_roll_max   = 0;
  g_roll_acc_us = 0;
  roll_publish();
}

//-----------------------------------------------------------------------------
// Forget the plotted history. Only where there is nothing left to carry over:
// the first entry since boot, and coming back after longer than the strip
// spans. Restamps, because a blank strip has no gap left to draw.
static void roll_clear(void)
{
  memset(g_roll_row_min, GRID_HEIGHT/2-1, GRID_WIDTH);
  memset(g_roll_row_max, GRID_HEIGHT/2-1, GRID_WIDTH);
  memset(g_roll_row_flags, SAMPLE_FLAG_NONE, GRID_WIDTH);

  g_roll_stamp = timer_us();
  roll_reopen();
}

//-----------------------------------------------------------------------------
// A timebase step INSIDE the view: rescale the strip along time instead of
// dropping it.
//
// The sample rate does not move with the roll timebase - it is pinned, see
// ROLL_SR_DIVIDER - so a column means the same thing at every roll timebase:
// the peak envelope of a stretch of wall clock. Only the length of the
// stretch changes. Slower, and a new column is the union of the old ones its
// span covers; faster, and one old column feeds the several new ones that
// fall inside it.
//
// That second direction cannot un-blur anything, and does not pretend to: a
// column recorded over 400 ms knows only its own two extremes, and the four
// 100 ms columns it becomes all carry them. It is the envelope at the
// resolution it was RECORDED at - which is a true statement about the signal,
// where throwing ten minutes of history away over one keypress is not.
//
// In place, because a second copy of the strip is another 900 bytes of TCM.
// The two directions need opposite orders and the arithmetic is what
// guarantees it: compressing, a new column never reads anything to the right
// of itself, so the sweep goes right to left; expanding, never anything to
// the left, so it goes the other way. Either way a column is written only
// after everything that reads it has been.
static void roll_retime(int old_us, int new_us)
{
  int step = (new_us > old_us) ? -1 : 1;
  int j = (step < 0) ? GRID_WIDTH-1 : 0;

  for (int n = 0; n < GRID_WIDTH; n++, j += step)
  {
    // Age of this column's edges, in us behind the newest edge of the strip.
    // Column c of either scale covers [(W-1-c)*us, (W-c)*us).
    int lo = (GRID_WIDTH-1 - j) * new_us;
    int hi = lo + new_us - 1;
    int first = GRID_WIDTH-1 - hi / old_us; // oldest old column it touches
    int last  = GRID_WIDTH-1 - lo / old_us; // ...and the newest
    int min = GRID_HEIGHT, max = -1, flags = SAMPLE_FLAG_NONE;

    if (first < 0)
      first = 0;

    if (last > GRID_WIDTH-1)
      last = GRID_WIDTH-1;

    for (int i = first; i <= last; i++)
    {
      if (!(g_roll_row_flags[i] & SAMPLE_FLAG_VALID))
        continue;

      if (g_roll_row_min[i] < min)
        min = g_roll_row_min[i];

      if (g_roll_row_max[i] > max)
        max = g_roll_row_max[i];

      flags |= g_roll_row_flags[i];
    }

    // Nothing was ever recorded this far back: the strip now spans more time
    // than it has been running for, and those columns are honestly empty
    if (max < 0)
    {
      g_roll_row_min[j]   = GRID_HEIGHT/2-1;
      g_roll_row_max[j]   = GRID_HEIGHT/2-1;
      g_roll_row_flags[j] = SAMPLE_FLAG_NONE;
    }
    else
    {
      g_roll_row_min[j]   = min;
      g_roll_row_max[j]   = max;
      g_roll_row_flags[j] = flags;
    }
  }

  roll_reopen();
}

//-----------------------------------------------------------------------------
// A vertical change moves every stored column, and clearing an hour of
// history because the range was nudged is not an answer. The rows carry
// enough to move them: a row is the input's millivolts divided by the
// volts/div and offset by the vertical position, so the old settings give the
// millivolts back and the new ones put them somewhere else.
//
// What that cannot recover is a column that was already against the top or
// the bottom of the grid - clip_for_display() threw the rest of it away when
// the column was written, exactly as it does for the live trace. Zooming out
// therefore un-clips nothing; it only stops NEW columns from clipping.
//
// Runs whether or not the view is up. The strip outlives a look at the
// spectrum now, and a volts/div change made while looking at it would
// otherwise leave every stored row meaning something the mapping no longer
// says.
void roll_rescale(int old_scale, int old_vpos)
{
  int scale = vs_mv_px(config.vertical_scale);

  for (int c = 0; c < GRID_WIDTH; c++)
  {
    if (!(g_roll_row_flags[c] & SAMPLE_FLAG_VALID))
      continue;

    // clip_for_display() is row = GRID_HEIGHT/2-1 - value, so this runs it
    // backwards into a value, into millivolts, and forwards again
    int lo = (GRID_HEIGHT/2-1 - g_roll_row_min[c] - old_vpos) * old_scale;
    int hi = (GRID_HEIGHT/2-1 - g_roll_row_max[c] - old_vpos) * old_scale;

    g_roll_row_min[c] = clip_for_display(lo / scale + config.vertical_position);
    g_roll_row_max[c] = clip_for_display(hi / scale + config.vertical_position);
  }

  roll_publish();
}

//-----------------------------------------------------------------------------
// Close the open column: everything one place to the left, the accumulated
// envelope into the column that just came free at the right.
void roll_commit(void)
{
  int flags = SAMPLE_FLAG_NONE;

  memmove(&g_roll_row_min[0], &g_roll_row_min[1], GRID_WIDTH-1);
  memmove(&g_roll_row_max[0], &g_roll_row_max[1], GRID_WIDTH-1);
  memmove(&g_roll_row_flags[0], &g_roll_row_flags[1], GRID_WIDTH-1);

  if (g_roll_min <= g_roll_max)
  {
    // min/max are counts here and rows on the way out, and the two run
    // opposite ways: a bigger count is a HIGHER pixel, i.e. a smaller row
    g_roll_row_min[GRID_WIDTH-1] = roll_row(g_roll_max);
    g_roll_row_max[GRID_WIDTH-1] = roll_row(g_roll_min);

    flags = SAMPLE_FLAG_VALID;

    if (g_roll_min == 0)
      flags |= SAMPLE_FLAG_CLIP_L;

    if (g_roll_max == 255)
      flags |= SAMPLE_FLAG_CLIP_H;
  }
  else
  {
    g_roll_row_min[GRID_WIDTH-1] = GRID_HEIGHT/2-1;
    g_roll_row_max[GRID_WIDTH-1] = GRID_HEIGHT/2-1;
  }

  g_roll_row_flags[GRID_WIDTH-1] = flags;

  g_roll_min = 255;
  g_roll_max = 0;
}

//-----------------------------------------------------------------------------
// Called every pass of the main loop while the view is up
void roll_task(void)
{
  uint32_t now = timer_us();
  int vmin, vmax, elapsed, columns = 0;

  if (g_roll_col_us <= 0)
    return; // never entered the view properly; nothing to divide time into

  if (capture_get_state() == CAPTURE_STATE_STOP)
  {
    // Frozen: the plot is a picture of the last minutes and the time spent
    // stopped is not part of it. Restart the clock so resuming appends the
    // next column rather than a screenful of blank ones.
    g_roll_stamp = now;
    return;
  }

  if (capture_fold_samples(&vmin, &vmax) > 0)
  {
    if (vmin < g_roll_min)
      g_roll_min = vmin;

    if (vmax > g_roll_max)
      g_roll_max = vmax;
  }

  elapsed = (int)(now - g_roll_stamp); // wrap-safe: 1 MHz into 32 bits
  g_roll_stamp = now;

  // More than a screen's worth of time since the last look. The scope does
  // not run while a menu is open, so this is mostly "the menu was up for a
  // while" - and a negative reading is the same thing after 35 minutes of it,
  // which is where a 1 MHz counter overruns what an int can hold. Either way
  // every column on screen is older than the screen, and shifting 300 empty
  // ones through one at a time only takes longer to say so.
  if (elapsed < 0 || elapsed > GRID_WIDTH * g_roll_col_us)
  {
    roll_clear();
    return;
  }

  g_roll_acc_us += elapsed;

  while (g_roll_acc_us >= g_roll_col_us)
  {
    g_roll_acc_us -= g_roll_col_us;
    roll_commit();
    columns++;
  }

  if (columns > 0)
  {
    // The sweep paints one column per pass and everything moved, so this is
    // a full repaint - ~11 ms of LCD writes spread over the next 300 passes,
    // which is why the roll threshold stops where it does
    roll_publish();
  }
}

//-----------------------------------------------------------------------------
// Enter or leave the view. Only the timebase decides this, so it is called
// from wherever the timebase or a view flag can have changed rather than from
// a key.
void roll_set_active(bool active)
{
  if (active == g_roll_active)
  {
    // Same view, different timebase: a column covers a new amount of time,
    // and what is already plotted gets rescaled onto it
    if (active && g_roll_col_us != roll_column_us())
    {
      int old_us = g_roll_col_us;

      g_roll_col_us = roll_column_us();
      roll_retime(old_us, g_roll_col_us);
    }

    return;
  }

  g_roll_active = active;
  g_state = -1; // the state slot says ROLL or does not

  if (active)
  {
    int old_us = g_roll_col_us;
    int stale_min, stale_max;

    // Panning has nothing to pan: the newest column is the present, and
    // there is no record either side of the screen to walk into
    config.horizontal_position = 0;
    config.horizontal_position_px = 0;

    g_roll_col_us = roll_column_us();

    // The ring has been turning the whole time this view was not up, and the
    // fold's read point with it. Take that stretch and throw it away, or the
    // first column back would carry up to a ring of signal in it.
    capture_fold_samples(&stale_min, &stale_max);

    // Whatever is stored is still a strip of the same input at the same
    // sample rate; it is only OLDER, and roll_task shifts blank columns in
    // for the time spent away (or clears it, past a screenful). A timebase
    // moved while another view was up gets the same rescaling as one moved
    // in here - the stored columns just cover a different amount of time.
    if (old_us > 0 && old_us != g_roll_col_us)
      roll_retime(old_us, g_roll_col_us);
    else if (old_us <= 0)
      roll_clear(); // first entry since boot: there is no strip yet
    else
      roll_publish();
  }

  // AUTO while rolling whatever the trigger says, and the user's mode back
  // on the way out. A NORMAL sweep that never triggers would leave the
  // measurements frozen on whatever record was last published, and SINGLE
  // stops the ring outright - which would look exactly like a hung scope.
  //
  // Not capture_start(): a stop is the user's, and a timebase key is not a
  // request to undo one. Rolling into a stopped acquisition shows the frozen
  // strip and STOP in the state slot, and RUN starts it moving.
  capture_set_trigger_mode(active ? TRIGGER_MODE_AUTO : config.trigger_mode);
}

//-----------------------------------------------------------------------------
// Re-decide the view after something OTHER than the timebase changed what it
// should be - a view toggle, the threshold setting. The timebase key does
// this itself, in the middle of its own rescaling; everyone else calls here.
void roll_sync(void)
{
  bool want = roll_wanted();

  if (want == g_roll_active)
    return;

  roll_set_active(want);
  update_sample_rate(); // the acquisition is timed differently either way
}

//-----------------------------------------------------------------------------
// How much time the screen holds, for the readout that shows the pan offset
// on a swept trace - there is no pan here, and the span is what a strip chart
// is read against
int64_t roll_screen_ns(void)
{
  return hs_div_value[config.horizontal_scale] * GRID_DIVS_H;
}

