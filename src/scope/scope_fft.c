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

/*- Definitions -------------------------------------------------------------*/
// Spectrum hold: EDGE cycles Off -> Max -> Avg. Max-hold pins anything that
// ever appeared (intermittent interference, drifting tones); the exponential
// average (1/8 per frame) sinks the noise floor ~9 dB and steadies weak
// lines. The combined spectrum REPLACES g_fft_mag, so analysis, peaks and
// the cursor all describe the picture actually on screen.
enum { FFT_HOLD_OFF, FFT_HOLD_MAX, FFT_HOLD_AVG };

/*- Variables and implementations -------------------------------------------*/
static int g_fft_peak_hz = 0;
uint8_t g_fft_samples[FFT_SIZE];
float g_fft_mag[FFT_BINS];
// Spectrum view state. The analysis always spans the whole record decimated
// by `max >> band`: band 0 is the full record (best resolution, narrowest
// span), every step up doubles the span and halves the resolution.
FftAnalysis g_fft_an;
// Above-nyquist reasoning for that analysis. Only ever filled on the
// undecimated spectrum: everywhere else the band edge is the decimation's
// own low-pass rather than the frontend, and alias.c reasons about the
// frontend. The auto band already parks anything fast on the raw transform,
// so this appears exactly where the ADC is the thing doing the folding.
AliasAnalysis g_fft_alias;
static int g_fft_band = 0;
bool g_fft_band_auto = true;
int64_t g_fft_auto_span = 0; // record time the auto band was picked for
int g_fft_cursor = -1;       // trace column, -1 when the cursor is off
int g_fft_cursor_bin = 0;
bool g_fft_panel_pending = false;
bool g_fft_panel_on = true;  // MODE toggles the breakdown panel
uint16_t g_fft_grad[GRID_HEIGHT_MAX]; // heat ramp, indexed by curve height
static bool g_fft_grad_ready = false;
static int g_fft_hold_mode = FFT_HOLD_OFF;
static bool g_fft_hold_primed = false;
static float g_fft_hold[FFT_BINS];
static int g_fft_hold_decim = 0;   // bin scale the held data was taken at:
static int g_fft_hold_period = 0;  // a change makes the bins mean new
int g_fft_timer = TIMER_DISABLE;
bool g_fft_have = false;
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
void fft_update(void)
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

  if (g_fft_hold_mode != FFT_HOLD_OFF)
  {
    if (decim != g_fft_hold_decim || period_ns != g_fft_hold_period)
      g_fft_hold_primed = false; // bins mean new frequencies now

    if (!g_fft_hold_primed)
    {
      for (int i = 0; i < FFT_BINS; i++)
        g_fft_hold[i] = g_fft_mag[i];

      g_fft_hold_primed = true;
      g_fft_hold_decim = decim;
      g_fft_hold_period = period_ns;
    }
    else if (g_fft_hold_mode == FFT_HOLD_MAX)
    {
      for (int i = 0; i < FFT_BINS; i++)
      {
        if (g_fft_mag[i] > g_fft_hold[i])
          g_fft_hold[i] = g_fft_mag[i];
      }
    }
    else
    {
      for (int i = 0; i < FFT_BINS; i++)
        g_fft_hold[i] += (g_fft_mag[i] - g_fft_hold[i]) * 0.125f;
    }

    for (int i = 0; i < FFT_BINS; i++)
      g_fft_mag[i] = g_fft_hold[i];
  }

  fft_analyze(g_fft_mag, period_ns * decim, &g_fft_an);

  if (1 == decim)
  {
    alias_check(g_fft_mag, &g_fft_an, ALIAS_FRONTEND_HZ, &g_fft_alias);
  }
  else
  {
    g_fft_alias.verdict = ALIAS_NO_EVIDENCE;
    g_fft_alias.count = 0;
  }

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
void fft_tick(void)
{
  if (g_fft_timer == TIMER_DISABLE || g_fft_timer == 0)
  {
    g_fft_timer = FFT_UPDATE_TIMEOUT;
    fft_update();
  }

  capture_consume_frame();
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
bool fft_buttons(int buttons, bool shift, bool repeat)
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
  else if (buttons & BTN_EDGE)
  {
    // Off -> Max -> Avg. Restarting the accumulator on every switch is the
    // point: stale max-hold from before a probing change is a lie.
    if (!repeat)
    {
      g_fft_hold_mode = (g_fft_hold_mode + 1) % 3;
      g_fft_hold_primed = false;
      g_fft_panel_pending = true;
      fft_update();
    }
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
// "49.9Hz" / "1.23kHz" / "12.34MHz" into a caller-supplied buffer: a panel
// line holds two numbers and format_frequency()'s buffer is one deep.
// Rounding is done once, on the printed digit, so 49.97 Hz is "50.0Hz" and
// never "49.0Hz".
void fft_format_hz(char *buf, int size, float hz)
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
void fft_format_db(char *buf, int size, float db)
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
// Is the record still filling, by enough to be worth saying? Below a second
// the wait is shorter than the eye, and every pan tick re-times the
// acquisition - a readout that flickered on all of them would be noise.
bool fft_fill_pending(void)
{
  int done, total;

  capture_record_fill(&done, &total);

  return (total >= FFT_FILL_MIN_MS && done < total);
}

//-----------------------------------------------------------------------------
// The progress row, always row 2 - under the two header rows and above
// whatever the panel has room for below it. Drawn on its own rather than by
// repainting the panel around it: this ticks twice a second, and clearing 168
// by 80 pixels of a bit-banged display that often is a panel that blinks.
//
// Expects FONT_SMALL, like fft_panel_row - it is called from the middle of
// the panel's own rendering, and setting the font back to FONT_LARGE there
// drew every row under it in the wrong one, off the edge of the panel.
#define FFT_FILL_ROW  2

void fft_fill_row_draw(void)
{
  char line[40];
  int done, total;

  capture_record_fill(&done, &total);

  snprintf(line, sizeof(line), "rec %d.%d/%d.%d s filling",
      done / 1000, (done % 1000) / 100, total / 1000, (total % 1000) / 100);

  lcd_fill_rect(FFT_PANEL_X + 1, FFT_PANEL_Y + 4 + FFT_FILL_ROW * 9,
      FFT_PANEL_W - 2, 9, BG_COLOR);
  fft_panel_row(FFT_FILL_ROW, CAPTURE_WAIT_COLOR, line);
}

//-----------------------------------------------------------------------------
// The spectrum breakdown: which frequencies the record is actually made of.
// Peaks that belong to the fundamental's harmonic comb are labelled h1..hN,
// everything else (interference, a second unrelated source, the noise
// floor poking through) is left unlabelled — that distinction is the whole
// point of the panel on a mains-plus-noise record.
void draw_fft_panel(void)
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

    // "No peaks" on a record that is a tenth full is not a finding
    if (fft_fill_pending())
      fft_fill_row_draw();

    lcd_set_font(FONT_LARGE);
    return;
  }

  fft_format_hz(num1, sizeof(num1), g_fft_an.fundamental);

  // A record with no harmonic content cannot say whether F0 is the signal or
  // the fold of one from above nyquist, so the number gets a question mark
  // rather than the confidence of the plain reading. The alternatives are
  // spelled out under the peak list.
  bool aliasable = (ALIAS_POSSIBLE == g_fft_alias.verdict);

  if (g_fft_an.thd_x10 >= 0)
    snprintf(line, sizeof(line), "F0 %-9s THD%3d.%d%%%s", num1,
        g_fft_an.thd_x10 / 10, g_fft_an.thd_x10 % 10, aliasable ? " ?" : "");
  else
    snprintf(line, sizeof(line), "F0 %s%s", num1, aliasable ? " ?" : "");

  fft_panel_row(row++, aliasable ? CAPTURE_WAIT_COLOR : TOAST_COLOR, line);

  fft_format_hz(num1, sizeof(num1), g_fft_an.bin_hz);
  fft_format_hz(num2, sizeof(num2), g_fft_an.nyquist_hz);
  snprintf(line, sizeof(line), "df %-9s BW %s%s%s", num1, num2,
      g_fft_band_auto ? "" : "*",
      (g_fft_hold_mode == FFT_HOLD_MAX) ? " M" :
      (g_fft_hold_mode == FFT_HOLD_AVG) ? " A" : "");
  fft_panel_row(row++, MEASURE_FREQ_COLOR, line);

  // A record still filling means every number above it was measured on the
  // one from BEFORE the timebase moved. At 5 ms/div that is over in 100 ms
  // and saying so would be noise; at 10 s/div it is 51 seconds of a spectrum
  // that looks finished and answers the wrong question, which is exactly the
  // trap this whole slow end of the timebase sets.
  if (fft_fill_pending())
  {
    fft_fill_row_draw(); // draws at FFT_FILL_ROW, which is this row
    row++;
  }

  // Peaks take what is left between the header and the footer row
  for (int i = 0; i < g_fft_an.count && row < 2 + FFT_PANEL_ROWS; i++)
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

  // The frequencies that would have produced this same spectrum from above
  // nyquist, listed under the peaks in the same columns. Room is never the
  // problem: what gets here is a bare tone, and a bare tone leaves most of
  // the peak rows empty by definition.
  if (aliasable && row < 2 + FFT_PANEL_ROWS)
  {
    fft_panel_row(row++, CAPTURE_WAIT_COLOR, "no h3: sine, or alias of");

    for (int i = 1; i < g_fft_alias.count && row < 2 + FFT_PANEL_ROWS; i++)
    {
      fft_format_hz(num1, sizeof(num1), g_fft_alias.cand[i].freq);
      snprintf(line, sizeof(line), "?%-2d%9s", g_fft_alias.cand[i].order, num1);
      fft_panel_row(row++, CAPTURE_WAIT_COLOR, line);
    }
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

