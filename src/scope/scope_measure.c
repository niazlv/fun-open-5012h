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
static int  g_trend_metric = 0;   // 0 = frequency, 1 = Vrms, 2 = duty
int  g_trend_timer = TIMER_DISABLE;
int g_measure_timer = TIMER_DISABLE;
static SignalClass g_signal_class = { SIG_UNKNOWN, -1 };
// The spectrum's frequency for the record g_signal_class was built from; 0
// when no record or no peak stood out of the floor
static int g_spectrum_hz = 0;
// The lowest above-nyquist frequency that would have folded to g_spectrum_hz,
// for the same record; 0 when the record rules that reading out or cannot
// raise the question. Kept apart from g_fft_alias, which belongs to the
// SPECTRUM view and is only refreshed while that view is on screen - this one
// has to answer in the ordinary sweep too. Scratch for it is static rather
// than automatic: FftAnalysis and AliasAnalysis together are ~240 bytes, and
// the TCM stack is the one pool with no room to spare.
static int g_alias_hz = 0;
static FftAnalysis g_alias_an;
static AliasAnalysis g_alias_res;
//-----------------------------------------------------------------------------
// Who owns the right-hand end of the status line (x >= 140): the measurements
// when at least one slot asks for a metric, the trigger edge, trigger level
// and horizontal position otherwise.
//
// This used to ask config.measure_display alone, which handed the slot to a
// readout that draw_measure() never painted in panel view: whatever glyphs
// happened to be there froze (the reading from the moment the scope was
// entered), and anything that cleared the line — a toast, an auto-setup —
// left it empty for good, because the only element allowed to repaint it
// returned immediately.
bool measure_owns_status_line(void)
{
  if (g_fft_mode)
    return true; // the spectrum readout lives there

  /*
   * MODE is not asked about here, and that is the point of the base view: with
   * the overlay off, this screen IS the instrument - the settings along the top,
   * two readings along the bottom, nothing over the trace. Asking MODE would
   * empty the bottom bar of the only two numbers on it and hand the space to the
   * trigger readouts, so the cleanest view was the one that said least.
   *
   * MODE means the OVERLAY - the band or the placed widgets, the thing that
   * covers the trace (mpanel_wanted). A slot that should say nothing is set to
   * Off in the menu; both of them Off hands the space back to the trigger level
   * and the horizontal position, exactly as before.
   */
  if (scope_calibration_mode)
    return false;

  for (int i = 0; i < MEASURE_LINE_SLOTS; i++)
  {
    if (config.measure_line[i] != MEASURE_NONE)
      return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
// One reading of the status line: see the definition for the two forms it takes
void measure_slot(int slot, int x, const char *tag, const char *value,
    int color);

//-----------------------------------------------------------------------------
// Decimation that puts the measured fundamental somewhere the harmonics can be
// counted from.
//
// This is what stood between the THD reading and ever showing a number. The
// transform is FFT_SIZE points; taken raw off the record they span
// FFT_SIZE * period, which at any ordinary timebase is a sliver of a
// microsecond-scale window - a 1 kHz signal has a hundredth of a cycle in it,
// the fundamental lands in bin zero, and classify.c refuses (rightly: there is
// nothing there to call a fundamental). The spectrum VIEW never had the problem
// because it decimates to a band; this had no band to pick.
//
// So pick one: aim the fundamental at bin FFT_SIZE/32 and let the clamps do the
// rest. That leaves the fifth harmonic at bin 80 of 256 - inside the spectrum,
// which is what the THD sum needs - and a whole number of the signal's periods
// in the window either way.
static int signal_info_decim(int freq, int size, int period_ns)
{
  int64_t want;
  int max = fft_max_decimation(size);

  if (freq <= 0 || period_ns <= 0)
    return 1;

  want = (int64_t)(FFT_SIZE / 32) * 1000000000ll /
      ((int64_t)freq * FFT_SIZE * period_ns);

  if (want < 1)
    return 1;

  return (want > max) ? max : (int)want;
}

//-----------------------------------------------------------------------------
// Classify the current record (type + THD), cached per acquisition. Reads the
// record in place rather than copying it, so the display pipeline is
// unaffected.
//
// The same transform also yields the spectrum's own reading of the frequency,
// which MEASURE_FFT_FREQ puts next to the counter's. It costs nothing extra:
// the peak is picked out of a spectrum that was already computed, and the
// parabolic interpolation in fft_peak_frequency() lands well inside one bin.
void signal_info_update(void)
{
  static uint32_t cached_gen = 0xffffffff;
  uint32_t gen = capture_get_generation();
  const uint8_t *data;
  Measure m;
  int size, offset, period_ns, tpos;

  if (gen == cached_gen)
    return;

  if (!capture_get_raw_measure(&m))
    return;

  if (capture_get_record(&data, &size, &offset, &period_ns, &tpos) &&
      size >= FFT_SIZE && period_ns > 0)
  {
    int decim = signal_info_decim(m.frequency, size, period_ns);
    int64_t bin_period = (int64_t)period_ns * decim;
    int fund_bin = (int)(((int64_t)m.frequency * FFT_SIZE * bin_period) /
        1000000000ll);

    fft_spectrum_decim(data, size, offset, decim, g_fft_mag);
    classify_signal(&m, g_fft_mag, FFT_BINS, fund_bin, &g_signal_class);
    g_spectrum_hz = fft_peak_frequency(g_fft_mag, (int)bin_period);

    // The full harmonic analysis, which is where the THD reading comes from:
    // the same function, on the same kind of spectrum, as the number the
    // spectrum view puts in its panel. It also feeds the alias test below.
    fft_analyze(g_fft_mag, (int)bin_period, &g_alias_an);

    // Same argument as the peak above: the transform is already paid for, and
    // this reads an answer out of it. But only when the spectrum is UNDECIMATED
    // - that is the one condition alias.c reasons under, since past it the band
    // edge is the decimation's low-pass rather than the analog frontend. No
    // loss: a signal that needed decimation to be seen at all is nowhere near
    // the Nyquist rate this question is about.
    //
    // Only ALIAS_POSSIBLE produces a number. IN_BAND means the harmonics were
    // found where only an in-band source can put them, and printing "could be
    // 108 MHz" beside a reading the record just PROVED honest would be worse
    // than printing nothing; NO_EVIDENCE means the test was refused rather
    // than passed, and a refusal is not a candidate either.
    if (1 == decim)
    {
      alias_check(g_fft_mag, &g_alias_an, ALIAS_FRONTEND_HZ, &g_alias_res);

      g_alias_hz = (ALIAS_POSSIBLE == g_alias_res.verdict && g_alias_res.count > 1) ?
          (int)(g_alias_res.cand[1].freq + 0.5f) : 0;
    }
    else
    {
      g_alias_res.verdict = ALIAS_NO_EVIDENCE;
      g_alias_hz = 0;
    }
  }
  else
  {
    classify_signal(&m, NULL, 0, 0, &g_signal_class);
    g_spectrum_hz = 0;
    g_alias_hz = 0;
  }

  cached_gen = gen;
}

//-----------------------------------------------------------------------------
// One metric, formatted for both views at once: the panel puts `label` and
// `value` in a cell of its grid, the status line prints `tag` and `value` in
// the large one. Every metric formats to a constant width, so a shorter
// reading always covers the longer one it replaces - and in the panel, so a
// changing reading cannot move the ones beside it.
//
// `kind` and `present` are for the panel's colours: what the number measures,
// and whether there is a number at all. Both default to the common case here
// (a voltage that is being reported) and each metric overrides what differs.
//
/*
 * What a metric is worth on THIS record.
 *
 * Every rule here is about the record and the instrument, never about the
 * signal - see MeasureQuality. Each one is a fact the firmware can check:
 *
 *   clipped        the converter hit an end. Every voltage taken off that
 *                  record is a lower bound wearing a measurement's clothes.
 *   under a division  the reading is worth fewer than 25 ADC codes, so one
 *                  code is 4% of it. True, and coarse, and the fix is one key.
 *   periods disagree  the "period" the timing readings are built on is the
 *                  median of a set that has no middle. Frequency, duty and
 *                  jitter are then arithmetic on noise.
 *   under 3 samples per period  the counter's own trap: at 2.5 samples the
 *                  Schmitt arming misses every second crossing and 50 MHz
 *                  reads exactly 25 MHz, with nothing else on screen to say so.
 *   at the jitter floor  an edge lands between two samples and interpolates to
 *                  about a hundredth of one, so below that the number is the
 *                  timebase and not the signal.
 */
static int metric_mv(int metric, const ScopeMeasure *sm)
{
  switch (metric)
  {
    case MEASURE_VPP:  return sm->vpp_mv;
    case MEASURE_VRMS: return sm->vrms_mv;
    case MEASURE_VAVG: return sm->vavg_mv;
    case MEASURE_VP:   return sm->vp_mv;
    case MEASURE_VMAX: return sm->vmax_mv;
    case MEASURE_VMIN: return sm->vmin_mv;
    case MEASURE_VAMP: return sm->vtop_mv - sm->vbase_mv;
    default:           return 0;
  }
}

static int measure_quality(int metric, const ScopeMeasure *sm)
{
  int per = capture_get_record_period();

  switch (metric)
  {
    case MEASURE_VPP:
    case MEASURE_VRMS:
    case MEASURE_VAVG:
    case MEASURE_VP:
    case MEASURE_VMAX:
    case MEASURE_VMIN:
    case MEASURE_VAMP:
    {
      int mv_code = config.vertical_mult / CALIB_MULTIPLIER;
      int mv = metric_mv(metric, sm);
      int codes, swing;

      if (sm->vmax_raw >= 254 || sm->vmin_raw <= 1)
        return MQ_BAD;

      if (mv_code <= 0)
        return MQ_OK;

      if (mv < 0)
        mv = -mv;

      // How many codes the RECORD uses, not how many this particular reading
      // is worth. Judging a reading by its own magnitude flags avg = 0 mV on a
      // full-screen symmetric signal - which is a perfectly good measurement
      // of zero, and its relative error is a meaningless quantity. What is
      // actually coarse is a trace that lives in three codes; then every
      // number off it is coarse, including the large ones.
      codes = mv / mv_code;
      swing = sm->vmax_raw - sm->vmin_raw;

      if (swing > codes)
        codes = swing;

      return (codes < 25) ? MQ_WEAK : MQ_OK;
    }

    case MEASURE_FREQ:
    case MEASURE_FFT_FREQ:
    case MEASURE_PERIOD:
    case MEASURE_WIDTH_POS:
    case MEASURE_WIDTH_NEG:
    case MEASURE_DUTY:
    case MEASURE_JITTER:
    {
      int spp;

      // Nothing to judge: those readings are already blank, and a colour on a
      // row of dashes is a statement about nothing
      if (sm->periods <= 0 || per <= 0 || sm->period_med_ns <= 0)
        return MQ_OK;

      if (sm->period_good_pct < 60)
        return MQ_BAD;

      spp = sm->period_med_ns / per;

      if (spp < 3)
        return MQ_BAD;

      // Jitter is a STATISTIC over periods, so how many there were is the
      // whole question: a sigma over four of them is not a sigma. And its
      // floor is about a hundredth of a sample period, in picoseconds.
      if (MEASURE_JITTER == metric)
      {
        if (sm->periods < 5)
          return MQ_WEAK;

        if (sm->jitter_rms_ps >= 0 && sm->jitter_rms_ps <= per * 10)
          return MQ_WEAK;

        return MQ_OK;
      }

      // Frequency, period, duty and the widths do NOT get their precision from
      // the number of cycles: they get it from the time between two
      // interpolated crossings, which one clean period already gives to a
      // fraction of a percent. Three cycles of mains on the screen is a normal
      // way to look at mains, not a doubtful measurement - what would be
      // doubtful is a record that holds less than a whole cycle, or an edge
      // sampled so coarsely that the crossing has nowhere to land.
      return (sm->periods < 2 || spp < 10) ? MQ_WEAK : MQ_OK;
    }

    default:
      // Settings (V/div, s/div, trigger level, sample rate) are not
      // measurements and have no record to be judged against; the signal type
      // and the alias candidate say their own uncertainty in words already.
      return MQ_OK;
  }
}

//-----------------------------------------------------------------------------
// format_*() all hand back the same static buffer, so the value is copied into
// the item before anything else is formatted.
bool measure_format(int metric, const ScopeMeasure *sm, MeasureItem *it)
{
  const char *tag = "";
  const char *label = "";
  const char *value = NULL;
  MeasureKind kind = MK_VOLT;
  bool present = true;
  char scratch[16];

  switch (metric)
  {
    case MEASURE_VPP:
      tag = "V"; label = "Vpp"; value = format_voltage(sm->vpp_mv, false);
      break;

    case MEASURE_FREQ:
      tag = "f"; label = "f"; kind = MK_TIME;
      value = format_frequency(sm->frequency);
      present = sm->frequency > 0;
      break;

    case MEASURE_FFT_FREQ:
      // The counter's own answer is one line away, which is the point of
      // showing this one: they are independent instruments, and the record
      // where they disagree is the record worth looking at twice
      signal_info_update();
      tag = "F"; label = "fft"; kind = MK_TIME;
      value = g_spectrum_hz > 0 ? format_frequency(g_spectrum_hz) : "  --.--  ";
      present = g_spectrum_hz > 0;
      break;

    case MEASURE_ALIAS:
      // '?' rather than a letter, and the same '?' the spectrum panel marks
      // an unresolvable F0 with: this is not a measurement of anything, it is
      // the other frequency the same record would look like. Blank most of
      // the time, and that blank is the normal state - see MEASURE_ALIAS in
      // config.h for what it means.
      signal_info_update();
      tag = "?"; label = "?"; kind = MK_OTHER;
      value = g_alias_hz > 0 ? format_frequency(g_alias_hz) : "  --.--  ";
      present = g_alias_hz > 0;
      break;

    case MEASURE_DUTY:
      tag = "d"; label = "duty"; kind = MK_OTHER;
      value = format_duty(sm->duty_x10);
      present = sm->duty_x10 >= 0;
      break;

    case MEASURE_VRMS:
      tag = "R"; label = "rms"; value = format_voltage(sm->vrms_mv, false);
      break;

    case MEASURE_VP:
      // Peak from GROUND, not half of Vpp: on a 0..3.3 V logic line the two
      // differ by a factor of two, and the one that answers "how much voltage
      // is on this wire" is this one. See vp_mv in capture.h.
      tag = "p"; label = "Vp"; value = format_voltage(sm->vp_mv, false);
      break;

    case MEASURE_VMAX:
      tag = "M"; label = "max"; value = format_voltage(sm->vmax_mv, true);
      break;

    case MEASURE_VMIN:
      // Both extremes carry their sign - which side of ground a peak is on is
      // half of what they are for
      tag = "m"; label = "min"; value = format_voltage(sm->vmin_mv, true);
      break;

    case MEASURE_VAMP:
      // The swing between the flat LEVELS, which is the amplitude of a square
      // wave as a datasheet means it: the overshoot and the ringing that Vpp
      // reports are outside the percentiles this comes from. vamp_mv is not
      // this number - it trims spikes, not distribution tails, and it belongs
      // to the auto-setup, which needs a short burst to still have a swing.
      tag = "a"; label = "amp";
      value = format_voltage(sm->vtop_mv - sm->vbase_mv, false);
      break;

    case MEASURE_PERIOD:
      // What the frequency reading is 1/x of, and the number a datasheet
      // timing diagram is written in. Blank rather than "0.00ns" when there is
      // no periodic signal: a period of zero is not a measurement.
      tag = "P"; label = "T"; kind = MK_TIME;
      present = sm->period_med_ns > 0;
      value = present ? format_time(sm->period_med_ns, false) : PERIOD_NONE;
      break;

    case MEASURE_WIDTH_POS:
      tag = "H"; label = "T+"; kind = MK_TIME;
      present = sm->width_pos_ns >= 0;
      value = present ? format_time(sm->width_pos_ns, false) : PERIOD_NONE;
      break;

    case MEASURE_WIDTH_NEG:
      tag = "L"; label = "T-"; kind = MK_TIME;
      present = sm->width_neg_ns >= 0;
      value = present ? format_time(sm->width_neg_ns, false) : PERIOD_NONE;
      break;

    case MEASURE_VAVG:
      tag = "A"; label = "avg"; value = format_voltage(sm->vavg_mv, false);
      break;

    case MEASURE_TYPE:
      signal_info_update();
      tag = "S"; label = "type"; kind = MK_OTHER;
      // Padded to the longest name there is ("Sawtooth"), because a value that
      // changes width is a value that moves whatever the panel put after it
      snprintf(scratch, sizeof(scratch), "%-8s", classify_name(g_signal_class.type));
      value = scratch;
      present = SIG_UNKNOWN != g_signal_class.type;
      break;

    case MEASURE_JITTER:
    {
      char sig[FORMAT_PS_W + 4], pp[FORMAT_PS_W + 4];

      // sigma answers "how stable", p-p answers "how bad was the worst one";
      // a runt cycle lives in p-p and would be invisible in sigma alone. Both
      // come out five characters wide whatever the scale, so the pair is
      // eleven and the panel's widest cell holds it without wobbling.
      format_ps(sm->jitter_rms_ps, sig, sizeof(sig));
      format_ps(sm->jitter_pp_ps, pp, sizeof(pp));

      tag = "J"; label = "jit"; kind = MK_TIME;
      present = sm->jitter_rms_ps >= 0;
      snprintf(scratch, sizeof(scratch), "%s~%s", sig, pp);
      value = scratch;
      break;
    }

    /*
     * The bars' own numbers, so that they can be placed where they can be read.
     * Formatted exactly as the bar formats them - same helper, same width - and
     * they come from config, so they have a value with no signal on the probe
     * and keep it while the acquisition is stopped.
     */
    case MEASURE_VDIV:
      tag = "v"; label = "V/div";
      value = vs_label(config.vertical_scale);
      break;

    case MEASURE_TDIV:
      tag = "t"; label = "s/div"; kind = MK_TIME;
      value = hs_str[(unsigned)config.horizontal_scale < HS_COUNT ?
          config.horizontal_scale : 0];
      break;

    case MEASURE_TRIG:
      // Referred to the vertical position, like the status line's own readout:
      // what the bar says and what a widget says have to be the same number
      tag = "l"; label = "trig";
      value = format_voltage(config.trigger_level_mv -
          config.vertical_position_mv, true);
      break;

    case MEASURE_SRATE:
      // With the unit spelled out, unlike the top bar's own field: there the
      // number sits under the rate LIMIT in a corner that says what it is, and
      // a widget dropped in the middle of the grid has no such neighbour
      tag = "r"; label = "rate"; kind = MK_TIME;
      present = g_sample_rate > 0;
      snprintf(scratch, sizeof(scratch), "%sS/s", format_sps(g_sample_rate));
      value = scratch;
      break;

    case MEASURE_THD:
      signal_info_update();
      // "THD", not the lone "t" it used to be. A one-letter name over a field
      // that is blank most of the time is a puzzle, not a label.
      //
      // From the same analysis the spectrum view prints, not from the
      // classifier's own smaller sum: two readings called THD on one
      // instrument have to be the same number, and the one with twelve folded
      // harmonics behind it is the better of the two to keep.
      tag = "T"; label = "THD"; kind = MK_OTHER;
      present = g_alias_an.thd_x10 >= 0;

      // A spectrum is not always there to take it from, and a slot the user
      // asked for says so rather than going blank or keeping a stale number
      if (!present)
        snprintf(scratch, sizeof(scratch), " --.-%%");
      else
        snprintf(scratch, sizeof(scratch), "%3d.%d%%",
            g_alias_an.thd_x10 / 10, g_alias_an.thd_x10 % 10);

      value = scratch;
      break;

    default:
      return false;
  }

  snprintf(it->label, sizeof(it->label), "%s", label);
  snprintf(it->tag, sizeof(it->tag), "%s", tag);
  snprintf(it->value, sizeof(it->value), "%s", value);
  it->metric = (uint8_t)metric;
  it->kind = (uint8_t)kind;
  it->present = present;
  it->quality = present ? (uint8_t)measure_quality(metric, sm) : MQ_OK;

  return true;
}

//-----------------------------------------------------------------------------
// The set of metrics the panel lists, in menu order
int measure_build_items(const ScopeMeasure *sm, MeasureItem *it)
{
  // A config saved before these flags existed reads all-false: default set
  bool any = config.show_vpp || config.show_freq || config.show_duty ||
      config.show_vrms || config.show_vavg || config.show_type || config.show_thd || config.show_jitter ||
      config.show_fft_freq || config.show_alias ||
      config.show_vp || config.show_vmax || config.show_vmin ||
      config.show_vamp || config.show_period || config.show_width_pos ||
      config.show_width_neg;

  const bool shown[MEASURE_COUNT] =
  {
    [MEASURE_VPP]  = any ? config.show_vpp  : true,
    [MEASURE_FREQ] = any ? config.show_freq : true,
    [MEASURE_DUTY] = any ? config.show_duty : true,
    [MEASURE_VRMS] = any ? config.show_vrms : true,
    [MEASURE_VAVG] = any && config.show_vavg,
    [MEASURE_TYPE] = any && config.show_type,
    [MEASURE_THD]  = any && config.show_thd,
    [MEASURE_JITTER] = any && config.show_jitter,
    [MEASURE_FFT_FREQ] = any && config.show_fft_freq,
    [MEASURE_ALIAS] = any && config.show_alias,
    [MEASURE_VP] = any && config.show_vp,
    [MEASURE_VMAX] = any && config.show_vmax,
    [MEASURE_VMIN] = any && config.show_vmin,
    [MEASURE_VAMP] = any && config.show_vamp,
    [MEASURE_PERIOD] = any && config.show_period,
    [MEASURE_WIDTH_POS] = any && config.show_width_pos,
    [MEASURE_WIDTH_NEG] = any && config.show_width_neg,
  };

  int n = 0;

  for (int m = MEASURE_NONE + 1; m < MEASURE_COUNT; m++)
  {
    if (shown[m] && measure_format(m, sm, &it[n]))
      n++;
  }

  // Never end up with nothing to draw: an empty band compares equal to itself
  // and is therefore never repainted again, and it is what a blank panel would
  // look like for any other reason.
  if (n == 0 && measure_format(MEASURE_VPP, sm, &it[0]))
    n = 1;

  return n;
}

//-----------------------------------------------------------------------------
// Rebuild the panel's cells from the selected metrics; on change, invalidate
// the columns they cover.
//
// What colour a reading is drawn in, wherever it is drawn: by what it measures,
// or grey when there is nothing to report
uint16_t measure_item_color(const MeasureItem *it)
{
  static const uint16_t kind_color[] =
  {
    [MK_VOLT]  = MEASURE_VOLTAGE_COLOR,
    [MK_TIME]  = MEASURE_FREQ_COLOR,
    [MK_OTHER] = MEASURE_MODE_COLOR,
  };

  if (!it->present)
    return MPANEL_NONE_COLOR;

  if (MQ_BAD == it->quality)
    return MEASURE_BAD_COLOR;

  if (MQ_WEAK == it->quality)
    return MEASURE_WEAK_COLOR;

  return kind_color[it->kind < ARRAY_SIZE(kind_color) ? it->kind : MK_OTHER];
}

//-----------------------------------------------------------------------------
// One status-line slot: the metric's one-character tag, then its value. The
// widths are remembered per slot so a shorter value erases the tail of the
// longer one it replaces - the metrics on show are the user's choice now, and
// they change under the slot without waiting for a page flip to wipe it.
void measure_slot(int slot, int x, const char *tag, const char *value,
    int color)
{
  static int prev_w[MEASURE_LINE_SLOTS];
  int w, h;

  if (1 == BAR_SCALE)
  {
    lcd_set_color(BG_COLOR, MEASURE_MODE_COLOR);
    lcd_puts(x, STATUS_LINE_Y, tag);

    lcd_set_color(BG_COLOR, color);
    lcd_puts(x + MEASURE_TAG_W, STATUS_LINE_Y, value);

    w = text_width(value);

    if (prev_w[slot] > w)
      lcd_fill_rect(x + MEASURE_TAG_W + w, STATUS_LINE_Y, prev_w[slot] - w,
          STATUS_LINE_HEIGHT, BG_COLOR);

    prev_w[slot] = w;
    return;
  }

  /*
   * The large form, which is the stock display's and is the whole point of the
   * setting: the NAME small, the DIGITS at 16x32, the UNIT small again -
   * "UPP 7.84 U". Doubling the unit and the padding with the number would cost
   * 40 px a reading and buy nothing: nobody squints at "mV", they read the
   * figure and glance at what it is in.
   *
   * Every formatter here writes [padded number][half space][unit] - see
   * format_number() - so the half space is the split, and a value that has none
   * (a duty cycle, a signal type) is all number, which is also right.
   *
   * The number is left-trimmed rather than padded, because at 16 px a character
   * the padding IS the layout: a seven-character field is 112 px of a 158 px
   * slot. What the padding was for - a shorter reading covering the longer one
   * it replaces - is done by erasing the tail instead (prev_w).
   */
  const char *unit = strchr(value, FONT_HALF_SPACE);
  const char *num = value;
  char digits[16];
  int n = 0;

  while (' ' == *num)
    num++;

  while (num[n] && (!unit || num + n < unit) && n < (int)sizeof(digits) - 1)
  {
    digits[n] = num[n];
    n++;
  }

  digits[n] = 0;

  // The name is as wide as it is - one character for the trigger readouts, three
  // or four for a metric - and the digits start after it
  int name_w = text_width(tag) + 4;

  lcd_set_color(BG_COLOR, MEASURE_MODE_COLOR);
  lcd_puts(x, STATUS_LINE_Y + STATUS_ROW2_Y_OFS, tag);

  lcd_set_text_scale(2);
  lcd_set_color(BG_COLOR, color);
  lcd_puts(x + name_w, STATUS_LINE_Y, digits);
  w = name_w + lcd_text_w(digits);
  h = lcd_glyph_h();
  lcd_set_text_scale(1);

  /*
   * Everything to the right of the digits goes, and only then does the unit go
   * back on it.
   *
   * The order matters, and getting it wrong is visible: the number is trimmed
   * rather than padded now, so losing a digit (105.00 mV -> 81.00 mV) slides the
   * unit one 16 px glyph to the left. Erasing after the unit - which is what a
   * fixed-width value needed - left the four pixels between the digits and the
   * unit holding the left edge of the digit that used to be there, and a sliver
   * of a figure beside "mV" reads as an extra zero.
   */
  if (prev_w[slot] > w)
    lcd_fill_rect(x + w, STATUS_LINE_Y, prev_w[slot] - w, h, BG_COLOR);

  // The unit sits on the digits' baseline, not their top: it is read as part of
  // the number below it, and a unit floating level with the tops of the figures
  // reads as a separate word
  if (unit && unit[1])
  {
    lcd_set_color(BG_COLOR, MEASURE_MODE_COLOR);
    lcd_puts(x + w + 4, STATUS_LINE_Y + STATUS_ROW2_Y_OFS, unit + 1);
    w += 4 + text_width(unit + 1);
  }

  prev_w[slot] = w;
}

//-----------------------------------------------------------------------------
// Measurements come from the full capture record (measure.c): mid-level
// crossing frequency with adaptive hysteresis, true RMS, duty cycle. The
// status line has room for two of them in the large font, and which two is
// set per slot in the Measurements menu - nothing rotates, a slot shows the
// metric it was given. In FFT mode the same space shows the spectrum readout.

// Is the record a level rather than a signal? In ADC CODES, not millivolts and
// not percent: the question is whether the trace is a LINE on the screen, and a
// code is about a pixel. Six of them is a quarter of a division - under that,
// what a peak-to-peak reading reports is the noise band whatever the range;
// over it there is a waveform to measure.
//
// Hysteresis, because what this decides is a WORD on the bar. A signal sitting
// on the threshold would otherwise flicker between two names, and a flickering
// label is worse than either of the two things it cannot choose between.
#define BAR_DC_ENTER   6
#define BAR_DC_LEAVE   10

static bool measure_bar_is_flat(const ScopeMeasure *sm)
{
  static bool flat = false;
  int swing = sm->vmax_raw - sm->vmin_raw;

  // Only with the input DC coupled, and that is not a detail: in AC the DC is
  // taken out by the hardware before the converter ever sees it, so there is no
  // level to report and a flat trace means "nothing is happening" rather than
  // "a steady voltage is here". Vpp is the honest reading there and it stays.
  if (config.vpp_dc_off || config.ac_coupling)
    return false;

  flat = flat ? (swing <= BAR_DC_LEAVE) : (swing <= BAR_DC_ENTER);

  return flat;
}

void draw_measure(void)
{
  ScopeMeasure sm;
  bool dc;
  // Not static: where the two slots are depends on the text size now
  const int slot_x[MEASURE_LINE_SLOTS] =
      { MEASURE_SLOT_0_X, MEASURE_SLOT_1_X };
  static const int slot_color[MEASURE_LINE_SLOTS] =
      { MEASURE_VOLTAGE_COLOR, MEASURE_FREQ_COLOR };

  if (g_toast_active || scope_calibration_mode)
    return;

  if (!measure_owns_status_line())
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

  dc = measure_bar_is_flat(&sm);

  for (int i = 0; i < MEASURE_LINE_SLOTS; i++)
  {
    MeasureItem item;
    int metric = config.measure_line[i];
    bool swap = dc && (MEASURE_VPP == metric);

    // A flat trace has no peak-to-peak worth reading, and what Vpp reports on
    // one is the noise band: a true answer to a question nobody asked. What
    // somebody looking at a flat line wants is the level it sits at, so the bar
    // hands its slot over and renames itself while it does - "Vdc" rather than
    // a number under a word that no longer describes it. The panel and the
    // placed widgets do NOT do this: those are lists of metrics somebody chose
    // one by one, and a list whose entries change meaning is not a list.
    if (swap)
      metric = MEASURE_VAVG;

    // A slot set to Off draws itself blank: the trigger readouts do not move
    // in to share the space, so leaving the old glyphs there would be a
    // reading that quietly stopped updating
    // The metric's NAME at the large size, its one-character tag at the normal
    // one: "Vpp 2.03 V" is what the stock display puts there and there is room
    // for it once the unit and the padding stop being 16 px wide, where at 8 px
    // a character the same name would take a third of the slot.
    if (measure_format(metric, &sm, &item))
    {
      if (swap)
      {
        // The tag keeps the V and gains the direct-current sign under it - one
        // glyph, because the tag column is one glyph, and the reading is still
        // volts. At the large size there is room for the word instead.
        snprintf(item.tag, sizeof(item.tag), "%c", FONT_DC_MARK);
        snprintf(item.label, sizeof(item.label), "Vdc");
      }

      // The bar colours by slot, not by metric - the two big readouts are a
      // fixed pair of colours so the eye finds them without reading. A verdict
      // on the reading overrides that, because it is the more urgent thing the
      // colour can be carrying.
      measure_slot(i, slot_x[i], (BAR_SCALE > 1) ? item.label : item.tag,
          item.value, (MQ_OK == item.quality) ? slot_color[i] :
          measure_item_color(&item));
    }
    else
    {
      measure_slot(i, slot_x[i], " ", "", slot_color[i]);
    }
  }
}

//-----------------------------------------------------------------------------
// Trend view: the logger ring plotted right-aligned (newest at the right
// edge), auto-scaled to its own min..max with headroom. One metric at a
// time; EDGE cycles them, MODE clears the ring.
static int trend_value(const TrendPoint *p)
{
  if (g_trend_metric == 0)
    return p->freq;

  if (g_trend_metric == 1)
    return p->vrms;

  return p->duty;
}

//-----------------------------------------------------------------------------
void trend_view_update(void)
{
  const TrendPoint *pts = trend_data();
  int n = trend_count();
  int lo = INT_MAX, hi = INT_MIN;

  for (int c = 0; c < GRID_WIDTH; c++)
    g_display_buffer.flags[c] = SAMPLE_FLAG_NONE;

  for (int i = 0; i < n; i++)
  {
    int v = trend_value(&pts[i]);

    if (v < lo)
      lo = v;

    if (v > hi)
      hi = v;
  }

  if (n > 0)
  {
    int span = hi - lo;
    int y0 = TREND_MARGIN_PX, y1 = GRID_HEIGHT - 2 - TREND_MARGIN_PX;

    for (int i = 0; i < n; i++)
    {
      int c = GRID_WIDTH - n + i;
      int v = trend_value(&pts[i]);
      int y = (span > 0) ?
          y1 - (int)((int64_t)(v - lo) * (y1 - y0) / span) : (y0 + y1) / 2;

      g_display_buffer.min[c]   = y;
      g_display_buffer.max[c]   = y;
      g_display_buffer.flags[c] = SAMPLE_FLAG_VALID;
    }

    close_gaps(&g_display_buffer);
  }

  g_shadow_valid = false;
  redraw_trace();
}

//-----------------------------------------------------------------------------
void trend_readout(void)
{
  static const char *const metric_names[] = { "freq", "Vrms", "duty" };
  const TrendPoint *pts = trend_data();
  int n = trend_count();
  char lo_s[14], hi_s[14], span_s[14], buf[64];
  int lo = INT_MAX, hi = INT_MIN, secs;

  if (g_toast_active)
    return;

  lcd_fill_rect(0, GRID_BOTTOM+1, LCD_WIDTH, STATUS_LINE_HEIGHT, BG_COLOR);
  lcd_set_color(BG_COLOR, SR_COLOR);

  if (n == 0)
  {
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Trend: no data yet (logs at 1 Hz)");
    return;
  }

  for (int i = 0; i < n; i++)
  {
    int v = trend_value(&pts[i]);

    if (v < lo)
      lo = v;

    if (v > hi)
      hi = v;
  }

  if (g_trend_metric == 0)
  {
    format_hz_mhz((int64_t)lo * 1000, lo_s, sizeof(lo_s));
    format_hz_mhz((int64_t)hi * 1000, hi_s, sizeof(hi_s));
  }
  else if (g_trend_metric == 1)
  {
    format_mv(lo, lo_s, sizeof(lo_s));
    format_mv(hi, hi_s, sizeof(hi_s));
  }
  else
  {
    snprintf(lo_s, sizeof(lo_s), "%d.%d%%", lo / 10, (lo < 0) ? 0 : lo % 10);
    snprintf(hi_s, sizeof(hi_s), "%d.%d%%", hi / 10, (hi < 0) ? 0 : hi % 10);
  }

  secs = n * trend_seconds_per_point();

  if (secs < 60)
    snprintf(span_s, sizeof(span_s), "%ds", secs);
  else if (secs < 3600)
    snprintf(span_s, sizeof(span_s), "%dm%02ds", secs / 60, secs % 60);
  else
    snprintf(span_s, sizeof(span_s), "%dh%02dm", secs / 3600, (secs % 3600) / 60);

  snprintf(buf, sizeof(buf), "%s %s..%s  %s",
      metric_names[g_trend_metric], lo_s, hi_s, span_s);
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, buf);
}

//-----------------------------------------------------------------------------
void trend_toggle(void)
{
  g_trend_mode = !g_trend_mode;
  g_decode_mode = false;
  g_fft_mode = false;
  g_shadow_valid = false;
  g_sweep_force = true;
  roll_sync(); // the trend plots its own log, not the input

  if (g_trend_mode)
    trend_view_update();
  else
    update_display();

  draw_status_line();
}

//-----------------------------------------------------------------------------
bool trend_buttons(int buttons, bool shift, bool repeat)
{
  if (repeat)
    return true;

  if ((buttons & BTN_EDGE) && !shift)
  {
    g_trend_metric = (g_trend_metric + 1) % 3;
    trend_view_update();
    trend_readout();
    return true;
  }

  if ((buttons & BTN_MODE) && !shift)
  {
    trend_reset();
    trend_view_update();
    trend_readout();
    return true;
  }

  // Arrows, AUTO and 50% have no meaning here; swallowing them beats having
  // them silently retune the scope underneath the plot
  if (buttons & (BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT | BTN_AUTO | BTN_50P))
    return true;

  return false; // SAVE, STOP and the SHIFT view switches fall through
}

