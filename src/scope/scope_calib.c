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

#define ACAL_LIN_M       16       // points either side of the middle
#define ACAL_LIN_N       (2*ACAL_LIN_M + 1)
#define ACAL_LIN_SPAN    110      // ADC codes the sweep aims to cover, +-
#define ACAL_LIN_MIN_SPAN 60      // ...and the least it may actually cover

/*- Variables and implementations -------------------------------------------*/
static int g_calibration_parameter = CALIB_ZERO;
// Calibration hint: on by default for every parameter, MODE hides it. It
// comes back on the next parameter, because the point of it is saying what
// THIS step wants - a hint dismissed once should not stay dismissed through
// three more steps the user has not seen yet.
bool g_calib_hint = true;
int g_calib_hint_param = -1; // parameter the current text was built for
bool g_autocal_active = false;
int g_autocal_timer = TIMER_DISABLE;
int g_autocal_phase = ACAL_ZERO;
static int g_autocal_lo = 0, g_autocal_hi = 0; // ACAL_ZERO bisection bracket
static int g_autocal_range = 0;    // vertical range ACAL_DAC is working on
static int g_autocal_sub = 0;      // sub-step inside a phase
static int g_autocal_ref = 0;      // reading carried between sub-steps
static uint32_t g_autocal_gen = 0; // acquisition generation at the last change
static int g_autocal_saved_hs = 0; // user settings, restored when it finishes
static int g_autocal_saved_vs = 0;
static int g_autocal_saved_vpos = 0;
static int g_autocal_saved_srl = 0;
static int g_autocal_saved_tmode = 0;
static bool g_autocal_did_gain = false; // whether the gain step actually ran
static int g_lin_i = 0;           // -M..+M, the step being measured
static int g_lin_step = 0;        // DAC counts between steps
static int g_lin_dac0 = 0;        // DAC code the sweep is centred on
static int g_lin_n[2] = { 0, 0 }; // points that landed inside the window
static int64_t g_lin_sx[2], g_lin_sxi[2], g_lin_sxi2[2], g_lin_sxx[2];
static int64_t g_lin_si[2], g_lin_si2[2], g_lin_si3[2], g_lin_si4[2];
// The gain pass on its own: no shorted-input preamble, and it does not stop
// after one range. Zero, delta and the DAC step are statements about the
// instrument and are found once; gain is a volts-per-count claim and there is
// one per range, so the only way to have all eight right is to visit them.
bool g_autocal_gain_only = false;
static int  g_autocal_gain_done = 0;    // ranges trimmed in this pass
//-----------------------------------------------------------------------------
// The scope menu flipped scope_calibration_mode. Calibration needs a free
// running acquisition to iterate on, so it takes the trigger over exactly
// like scope_init() does when it starts in that mode; leaving gives it back
// to the stored settings. The screen itself repaints when the menu closes.
void scope_calibration_changed(void)
{
  g_calibration_parameter = CALIB_ZERO;
  g_calib_hint = true;      // a fresh entry starts explained
  g_calib_hint_param = -1;

  if (scope_calibration_mode)
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

  capture_start(); // a stopped scope has nothing to calibrate against

  // Both the measurements panel and the FFT view step aside in calibration
  // mode, so their claim on the status line has to be recomputed
  g_mpanel_active = mpanel_wanted();
  g_line_owner = measure_owns_status_line();
}

//-----------------------------------------------------------------------------
void autocal_mark(void)
{
  g_autocal_gen = capture_get_generation();
}

//-----------------------------------------------------------------------------
// The reference level the gain step aims at. Configs saved before the field
// existed read zero; treat that as "never set" and offer a round number
// rather than refusing to run.
int scope_calib_ref_mv(void)
{
  if (config.calib_ref_mv < CALIB_REF_MIN_MV ||
      config.calib_ref_mv > CALIB_REF_MAX_MV)
    return CALIB_REF_DEFAULT_MV;

  return config.calib_ref_mv;
}

//-----------------------------------------------------------------------------
// Say what is happening and what it needs. A calibration that silently
// rearranges four numbers and then declares victory is worth nothing: the
// user has no way to tell a good run from a broken one, or to know what it
// did NOT do.
void autocal_say(const char *l0, const char *l1)
{
  // The panel band, not the status line: two lines of fifty characters is
  // enough to say what to DO, and the status line is not. mpanel_wanted()
  // hands the band over for the duration of a run.
  mpanel_set_active(true);
  mpanel_set_lines(l0, l1);
}

//-----------------------------------------------------------------------------
static bool autocal_ready(void)
{
  // The linearity sweep waits on a DAC write and nothing else - no relay, no
  // sample rate, no attenuator - and capture_get_channel_means() reads the raw
  // ring rather than a decimated snapshot, so one whole record already holds
  // none of the old level. Two, and its thirty-three steps take six seconds
  // instead of eighteen; the six the other phases use are for a switched
  // attenuator, which is a different kind of settling.
  if (ACAL_LIN == g_autocal_phase)
    return (capture_get_generation() - g_autocal_gen) >= 2;

  return (capture_get_generation() - g_autocal_gen) >= AUTOCAL_SETTLE;
}

//-----------------------------------------------------------------------------
static void autocal_set_position(int px)
{
  config.vertical_position = px;
  config.vertical_position_mv = px * vs_mv_px(config.vertical_scale);

  capture_set_vertical_parameters();
  autocal_mark();
}

/*
 * Gain, one range at a time, against a level the user already trusts.
 *
 * calib_vs_mult is a volts-per-ADC-count claim, and there are eight of them
 * because there are eight attenuator settings - a divider network and an
 * amplifier per range, each with its own tolerance. One trim cannot speak for
 * the others: 3% out at 500 mV/div says nothing about 50 mV/div, which is why
 * the gain step at the end of an auto-calibration run had to say "this range
 * only". This is that step made repeatable, so the eight can be walked in one
 * sitting with whatever references are to hand.
 *
 * The reference is STORED at the BNC, because calib_vs_mult is about the
 * instrument and knows nothing about what is clipped to the front. It is SHOWN
 * and typed at the probe tip, because that is what the meter beside it reads
 * and what the screen already displays. With a 10x probe the two differ by
 * exactly the factor the reading is multiplied by, so the round trip is exact.
 */
static int gain_ref_tip_mv(void)
{
  return scope_calib_ref_mv() * config_probe_mult();
}

//-----------------------------------------------------------------------------
static void gain_ref_set_tip(int tip_mv)
{
  int mult = config_probe_mult();
  int bnc = (tip_mv + mult / 2) / mult;

  if (bnc < CALIB_REF_MIN_MV)
    bnc = CALIB_REF_MIN_MV;
  else if (bnc > CALIB_REF_MAX_MV)
    bnc = CALIB_REF_MAX_MV;

  config.calib_ref_mv = bnc;
}

//-----------------------------------------------------------------------------
// The formatters pad to a fixed width so that a shorter reading covers the
// longer one it replaces on the status line. In a sentence that padding is
// just a hole, and there is no room for holes in fifty characters.
static const char *gain_trim(const char *s)
{
  while (' ' == *s)
    s++;

  return s;
}

//-----------------------------------------------------------------------------
// What the band says while the pass waits. A note (the result of a trim, or a
// refusal) holds the line for a couple of seconds and then it goes back to the
// live comparison - which after a successful trim is the same statement made
// by the instrument itself.
uint32_t g_gain_note_until = 0;

static void gain_prompt(void)
{
  // Generous, and clipped to the band by mpanel_set_lines(): a sentence that
  // does not fit is a sentence to shorten, not a buffer to police here
  char l0[80];
  char want[16], reads[16];
  ScopeMeasure sm;
  int ref = gain_ref_tip_mv();

  if (g_gain_note_until && (int32_t)(g_gain_note_until - timer_ms()) > 0)
    return;

  g_gain_note_until = 0;
  snprintf(want, sizeof(want), "%s", gain_trim(format_voltage(ref, false)));

  // A tenth of the expected level, not merely "positive": at this prompt in a
  // full calibration run the input is still shorted, and "reads 3.00 mV, want
  // 1.00 V, -99.7%" is a true sentence that tells nobody anything.
  if (capture_get_measurements(&sm) && sm.vavg_mv > ref / 10)
  {
    // Percent, because that is the shape of the error: a gain is a multiplier,
    // and 3.5% out at 5 V/div is 3.5% out everywhere on that range
    int err = (int)((int64_t)(sm.vavg_mv - ref) * 1000 / ref);
    char sign = (err < 0) ? '-' : '+';

    if (err < 0)
      err = -err;

    if (err > 9999)
      err = 9999;

    snprintf(reads, sizeof(reads), "%s", gain_trim(format_voltage(sm.vavg_mv, false)));
    snprintf(l0, sizeof(l0), "%s/div reads %s, want %s  %c%d.%d%%",
        gain_trim(vs_label(config.vertical_scale)), reads, want, sign,
        err / 10, err % 10);
  }
  else
  {
    snprintf(l0, sizeof(l0), "%s/div: apply %s DC to the input",
        gain_trim(vs_label(config.vertical_scale)), want);
  }

  autocal_say(l0, "MODE trims  L/R level  SHIFT+U/D range  STOP done");
}

//-----------------------------------------------------------------------------
static void gain_note(const char *msg, int ms)
{
  g_gain_note_until = timer_ms() + ms;
  autocal_say(msg, "MODE trims  L/R level  SHIFT+U/D range  STOP done");
}

//-----------------------------------------------------------------------------
// A measurement the gain step will not use. In the pass this is a note and the
// prompt comes back, because the fix is usually one keypress away - a coarser
// range, or the level that was actually applied. In a full auto-calibration
// run there is nothing left to come back to, so it ends there.
static void gain_refuse(const char *note, const char *l0, const char *l1)
{
  if (g_autocal_gain_only)
  {
    gain_note(note, 2500);
    g_autocal_phase = ACAL_ASK_REF;
    g_autocal_timer = TIMER_DISABLE;
    autocal_mark();
    return;
  }

  autocal_say(l0, l1);
  g_autocal_phase = ACAL_DONE;
}

//-----------------------------------------------------------------------------
// The expected level, on a round grid: snap first, then move, so that a value
// walked up with the fast step still lands exactly on what the meter says
// rather than 3 mV beside it.
void gain_ref_step(int dir, bool repeat)
{
  int step = repeat ? 200 : 10;
  int tip = (gain_ref_tip_mv() / step) * step;

  g_gain_note_until = 0;
  gain_ref_set_tip(tip + dir * step);
  gain_prompt();
}

//-----------------------------------------------------------------------------
void gain_range_step(int delta)
{
  int before = config.vertical_scale;

  change_vertical_scale(delta);

  if (config.vertical_scale == before)
    return;

  // The pass leaves the instrument on the range it was last working on, not
  // on the one the menu was opened from: the user picked this one, and having
  // it snap back would hide which range the last trim landed on
  g_autocal_saved_vs = config.vertical_scale;
  g_gain_note_until = 0;

  autocal_set_position(0); // the reading is taken against the centre line
  gain_prompt();
}

/*
 * Linearity: the instrument measuring its own bend, with nothing plugged in.
 *
 * The trick is that the reference is already on the board. The offset DAC is
 * twelve bits and it moves the signal across the converter's window; the ADC is
 * eight. Step the DAC in equal jumps with the input shorted, write down where
 * the trace lands each time, and any departure from a straight line is the
 * chain's - because the DAC's own nonlinearity, at +-2 LSB of 4096, is 0.06 of
 * an 8-bit code. A ruler eight times finer than the thing being measured is a
 * ruler.
 *
 * What comes out is fitted as x = a + b*i + c*i^2 over the step number i, and
 * only c is kept: a is the zero (calibrated already, and by a step that owns
 * it) and b is the gain (calibrated per range, later, and by a step that owns
 * that). The curvature is the part nothing else can express - and it is the
 * part that made a single-point gain trim fix 9.18 V and break 5.28 V.
 *
 * Both converters are measured in the same sweep: the ring interleaves them,
 * capture_get_channel_means() splits them, and they do not have the same curve.
 */
static void lin_reset(void)
{
  for (int c = 0; c < 2; c++)
  {
    g_lin_n[c] = 0;
    g_lin_sx[c] = g_lin_sxi[c] = g_lin_sxi2[c] = g_lin_sxx[c] = 0;
    g_lin_si[c] = g_lin_si2[c] = g_lin_si3[c] = g_lin_si4[c] = 0;
  }
}

//-----------------------------------------------------------------------------
static void lin_accumulate(int c, int mean_x100)
{
  // Outside this the amplifier is on its rail and the point is a lie about a
  // straight line rather than a measurement of a bent one
  if (mean_x100 < 1200 || mean_x100 > 24300)
    return;

  {
    int64_t x = mean_x100 - ZERO_POINT * 100;
    int64_t i = g_lin_i;

    g_lin_n[c]++;
    g_lin_sx[c]   += x;
    g_lin_sxi[c]  += i * x;
    g_lin_sxi2[c] += i * i * x;
    g_lin_sxx[c]  += x * x;
    g_lin_si[c]   += i;
    g_lin_si2[c]  += i * i;
    g_lin_si3[c]  += i * i * i;
    g_lin_si4[c]  += i * i * i * i;
  }
}

//-----------------------------------------------------------------------------
static void lin_set_dac(void)
{
  capture_set_dac_raw(g_lin_dac0 + g_lin_i * g_lin_step);
  autocal_mark();
}

//-----------------------------------------------------------------------------
// Least squares through the three sums, with the step number normalised to
// -1..1 first: i^4 over 33 steps is a quarter of a million, and a float has
// seven digits to spend. Normalised, every term is order n, and the ratio the
// correction is made of - c/b^2 - is invariant under the scaling anyway.
//
// Returns false and says why when the sweep is not something to calibrate
// against: too little of the window covered (a DAC that could not move it), or
// a scatter too wide to be a curve (an input that was not quiet).
static bool lin_fit(int c, int *q_out, int *resid_x100, const char **why)
{
  const float M = ACAL_LIN_M;
  float n   = (float)g_lin_n[c];
  float s1  = (float)g_lin_si[c]  / M;
  float s2  = (float)g_lin_si2[c] / (M*M);
  float s3  = (float)g_lin_si3[c] / (M*M*M);
  float s4  = (float)g_lin_si4[c] / (M*M*M*M);
  float y0  = (float)g_lin_sx[c]   / 100.0f;
  float y1  = (float)g_lin_sxi[c]  / (100.0f*M);
  float y2  = (float)g_lin_sxi2[c] / (100.0f*M*M);
  float yy  = (float)g_lin_sxx[c]  / 10000.0f;
  float det, a, b, q, rss;

  if (g_lin_n[c] < ACAL_LIN_N / 2)
  {
    *why = "too few usable points - was the input shorted?";
    return false;
  }

  det = n*(s2*s4 - s3*s3) - s1*(s1*s4 - s3*s2) + s2*(s1*s3 - s2*s2);

  if (det > -1e-3f && det < 1e-3f)
  {
    *why = "the sweep did not move - check the offset DAC";
    return false;
  }

  a = (y0*(s2*s4 - s3*s3) - s1*(y1*s4 - s3*y2) + s2*(y1*s3 - s2*y2)) / det;
  b = (n*(y1*s4 - s3*y2) - y0*(s1*s4 - s3*s2) + s2*(s1*y2 - y1*s2)) / det;
  q = (n*(s2*y2 - y1*s3) - s1*(s1*y2 - y1*s2) + y0*(s1*s3 - s2*s2)) / det;

  // b is half the span in codes, by construction: i runs -1..1
  if (b < ACAL_LIN_MIN_SPAN/2 && b > -ACAL_LIN_MIN_SPAN/2)
  {
    *why = "the sweep covered too little of the window";
    return false;
  }

  // What the fit did not explain, in hundredths of a code. Not noise: the mean
  // of two thousand samples has hundredths of a code of that. It is the
  // converter's own code-to-code error, the part of the curve no quadratic
  // reaches, and a converter with a whole code of it is still a normal
  // converter. Past that, what is being fitted is not a curve - it is a
  // signal on the input, a lead that came off, or mains humming into an
  // unterminated 1 MOhm, and none of those may be averaged into a constant.
  rss = yy - a*y0 - b*y1 - q*y2;

  if (rss < 0.0f)
    rss = 0.0f;

  *resid_x100 = (int)isqrt64((uint64_t)(rss / n * 10000.0f));

  if (*resid_x100 > 150)
  {
    *why = "the input is not quiet - short it and try again";
    return false;
  }

  // The curvature over the slope squared IS the coefficient: a chain that
  // reads x + k*x^2 fits as b*i + k*b^2*i^2, so c/b^2 is k whatever the sweep
  // was scaled by - which is why neither the DAC step nor the range it ran on
  // has to be known accurately for this to come out right.
  q = q / (b * b) * 4194304.0f; // 2^22, the scale capture.c corrects with

  if (q > 3000.0f || q < -3000.0f)
  {
    *why = "the bend is too big to be this instrument's own";
    return false;
  }

  *q_out = (int)(q + (q < 0 ? -0.5f : 0.5f));

  return true;
}

//-----------------------------------------------------------------------------
// The bend, said in the only unit that means anything from the front: how much
// the correction is worth at the top of the screen, as a percentage of it.
// x = 100 codes is four divisions on every range.
static int lin_percent_x100(int q)
{
  // q*100^2 / 2^22 codes of correction at x = 100, over 100 codes, in
  // hundredths of a percent: q * 10^6 / 2^22
  return (int)((int64_t)q * 1000000 / 4194304);
}

//-----------------------------------------------------------------------------
static void lin_start(void)
{
  int r = config.vertical_scale;
  int64_t codes_per_dac_x1000;
  int step;

  lin_reset();
  g_lin_dac0 = config.calib_dac_zero;

  // What one DAC count is worth in ADC codes: a screen pixel is
  // 1024/calib_dac_mult DAC counts and vs_px_value millivolts, and a code is
  // calib_vs_mult/1024 millivolts. Both halves of that were measured by the two
  // steps before this one, which is why this one runs after them - it only
  // needs them to pick a step size, but a wrong step size is a sweep that
  // either misses the window or spends itself on a quarter of it.
  codes_per_dac_x1000 = (int64_t)vs_px_value[r] * 1048576 * 1000 /
      ((int64_t)config.calib_dac_mult[r] * config.calib_vs_mult[r]);

  if (codes_per_dac_x1000 < 20)
    codes_per_dac_x1000 = 20;

  step = (int)((int64_t)ACAL_LIN_SPAN * 1000 /
      (ACAL_LIN_M * codes_per_dac_x1000));

  if (step < 2)
    step = 2;
  else if (step > 200)
    step = 200;

  // ...and the whole sweep has to stay inside the DAC's twelve bits, or the
  // ends of it are the same point measured twice
  while (step > 2 && (g_lin_dac0 - ACAL_LIN_M * step < 0 ||
      g_lin_dac0 + ACAL_LIN_M * step > 4095))
    step--;

  g_lin_step = step;
  g_lin_i = -ACAL_LIN_M;

  g_autocal_phase = ACAL_LIN;
  g_autocal_timer = AUTOCAL_TIMEOUT;

  autocal_say("Step 4 of 4: linearity", "Keep the input shorted");
  lin_set_dac();
}

//-----------------------------------------------------------------------------
// The sweep is over: fit both converters, keep the pair only if BOTH of them
// came out of a measurement worth keeping, and hand over to the gain prompt
// with the result still on the line - the next thing this run does depends on
// this having worked, and a number that scrolled past is a number nobody read.
static void lin_done(void)
{
  int q[2] = { 0, 0 }, resid[2] = { 0, 0 };
  const char *why = "no measurement";
  char l0[80];
  bool ok;

  ok = lin_fit(0, &q[0], &resid[0], &why) && lin_fit(1, &q[1], &resid[1], &why);

  if (ok)
  {
    int pb = lin_percent_x100(q[0]);
    int pa = lin_percent_x100(q[1]);

    config.calib_nl2[0] = (int16_t)q[0];
    config.calib_nl2[1] = (int16_t)q[1];

    snprintf(l0, sizeof(l0), "Bend: B %s%d.%02d%%  A %s%d.%02d%% at 4 divisions",
        pb < 0 ? "-" : "+", (pb < 0 ? -pb : pb) / 100, (pb < 0 ? -pb : pb) % 100,
        pa < 0 ? "-" : "+", (pa < 0 ? -pa : pa) / 100, (pa < 0 ? -pa : pa) % 100);
  }
  else
  {
    // Deliberately leaves whatever was stored alone. A failed sweep is not a
    // statement that the chain is straight, and zeroing it here would turn one
    // noisy minute into the loss of a good measurement.
    snprintf(l0, sizeof(l0), "Linearity: %s", why);
    // The scatter goes on the line whatever happened: it is the one number
    // that says whether the sweep was measuring the instrument or the room
    if (resid[0] > 0 || resid[1] > 0)
    {
      int r = (resid[0] > resid[1]) ? resid[0] : resid[1];

      snprintf(l0 + strlen(l0), sizeof(l0) - strlen(l0), " (%d.%02d)",
          r / 100, r % 100);
    }
  }

  // Gain is per range, so it belongs on the range the user chose before
  // starting - not on whichever one the sweeps above ended at. Position back to
  // zero: the reading is taken against the centre line.
  autoset_set_scale(g_autocal_saved_vs);
  autocal_set_position(0);

  g_autocal_phase = ACAL_ASK_REF;
  g_autocal_timer = TIMER_DISABLE; // connecting a source takes as long as it takes

  // Held for six seconds and then the gain prompt takes the line: what the
  // sweep found is worth reading, and what comes next is worth doing, and the
  // one thing that must not happen is the number going by unread.
  gain_note(l0, 6000);
}

//-----------------------------------------------------------------------------
// Restore the user's settings and report. It stays "active" so the band and
// the keyboard remain ours until the result has actually been read - a
// message that vanishes on its own is a message nobody saw.
void autocal_finish(bool ok)
{
  g_autocal_phase = ACAL_DONE;
  g_autocal_timer = TIMER_DISABLE;

  autoset_set_horizontal(g_autocal_saved_hs);
  autoset_set_scale(g_autocal_saved_vs);
  autocal_set_position(g_autocal_saved_vpos);

  config.sample_rate_limit = g_autocal_saved_srl;
  config.trigger_mode = g_autocal_saved_tmode;

  capture_set_trigger_mode(config.trigger_mode);
  update_sample_rate();
  draw_trigger_mode();
  refresh_view();
  mpanel_invalidate();

  g_gain_note_until = 0;

  // The gain pass answers a different question, so it reports a different
  // number: how many of the eight ranges now have a measured gain rather than
  // the factory's. The ones it did not visit are not wrong, they are just
  // still the numbers they came with.
  if (g_autocal_gain_only)
  {
    char msg[MPANEL_TEXT_MAX + 2];

    g_autocal_gain_only = false;

    // ok is false only on the watchdog here - STOP is how this one is meant to
    // end - and a measurement that never arrived is worth saying out loud
    if (!ok)
    {
      autocal_say("Timed out waiting for a record.",
          "Whatever was trimmed before it is kept.  MODE closes");
      return;
    }

    if (0 == g_autocal_gain_done)
    {
      autocal_say("Nothing was changed.", "MODE closes");
      return;
    }

    snprintf(msg, sizeof(msg), "Gain set on %d of %d ranges",
        g_autocal_gain_done, VS_COUNT);
    autocal_say(msg, "The others keep the gain they had.  MODE closes");
    return;
  }

  // Name what is still manual. "Calibrated" on its own would be a lie: the
  // gain is untouched, and it is the one the readings actually depend on.
  if (ok)
    autocal_say(g_autocal_did_gain ?
        "Done: zero, delta, DAC step and gain are set" :
        "Done: zero, delta and DAC step are set",
        g_autocal_did_gain ? "Gain applies to this range only.  MODE closes" :
        "Gain (S) was skipped.  MODE closes");
  else
    autocal_say("Stopped. Settings put back unchanged.",
        "MODE closes");
}

//-----------------------------------------------------------------------------
// The result has been read: give the band and the keyboard back
void autocal_close(void)
{
  g_autocal_active = false;

  mpanel_set_active(mpanel_wanted());
  mpanel_invalidate();
  refresh_view();
}

//-----------------------------------------------------------------------------
static void autocal_advance(void)
{
  Measure m;

  if (ACAL_ZERO == g_autocal_phase)
  {
    int err;

    // Bisect the offset DAC's zero point. Raw counts rise with it, so the
    // bracket halves cleanly, and single-channel means this is ADC B on its
    // own - bringing ADC A to the same level is the next phase's job.
    if (!capture_get_raw_measure_fresh(&m))
      return;

    err = (m.mean_c100 < 0) ? -m.mean_c100 : m.mean_c100;

    if (err <= AUTOCAL_ZERO_TOL || (g_autocal_hi - g_autocal_lo) <= 1)
    {
      g_autocal_phase = ACAL_DELTA;
      config.sample_rate_limit = 0; // dual channel: both converters visible
      update_sample_rate();
      autocal_mark();
      autocal_say("Step 2 of 4: matching the two ADCs",
          "Keep the input shorted");
      return;
    }

    if (m.mean_c100 > 0)
      g_autocal_hi = config.calib_dac_zero;
    else
      g_autocal_lo = config.calib_dac_zero;

    config.calib_dac_zero = (g_autocal_lo + g_autocal_hi) / 2;
    capture_set_vertical_parameters();
    autocal_mark();
    return;
  }

  if (ACAL_DELTA == g_autocal_phase)
  {
    int a, b;

    // The means come back RAW, with no delta applied, so the correction is
    // absolute rather than incremental. buffer_reverse adds the delta to
    // ADC A, so it has to make up A's shortfall against B.
    if (capture_get_channel_means(&a, &b))
    {
      int delta = (b - a + ((b > a) ? 50 : -50)) / 100;

      if (delta < -64)
        delta = -64;
      else if (delta > 64)
        delta = 64;

      config.calib_channel_delta = delta;
    }

    g_autocal_phase = ACAL_DAC;
    g_autocal_range = 0;
    g_autocal_sub = 0;
    autoset_set_scale(0);
    autocal_set_position(0);
    autocal_say("Step 3 of 4: DAC step, range 1 of 8",
        "Keep the input shorted");
    return;
  }

  if (ACAL_DAC == g_autocal_phase)
  {
    int target_x100, observed_x100;

    if (!capture_get_raw_measure_fresh(&m))
      return;

    if (g_autocal_sub == 0)
    {
      g_autocal_ref = m.mean_c100;
      g_autocal_sub = 1;
      autocal_set_position(AUTOCAL_DAC_PX);
      return;
    }

    // One position pixel must move the trace one screen pixel, and a screen
    // pixel is vs_px_value mV, which at this range is this many ADC counts
    target_x100 = vs_mv_px(g_autocal_range) * CALIB_MULTIPLIER * 100 /
        config.calib_vs_mult[g_autocal_range];
    observed_x100 = (m.mean_c100 - g_autocal_ref) / AUTOCAL_DAC_PX;

    // The DAC is linear, so a single proportional correction is exact
    if (observed_x100 > 0)
    {
      int64_t mult = (int64_t)config.calib_dac_mult[g_autocal_range] *
          target_x100 / observed_x100;

      if (mult < 0)
        mult = 0;
      else if (mult > 100000)
        mult = 100000;

      config.calib_dac_mult[g_autocal_range] = (int)mult;
    }

    g_autocal_range++;
    g_autocal_sub = 0;

    if (g_autocal_range >= VS_COUNT)
    {
      // Linearity next, and on the LEAST sensitive range: the prompt allows an
      // unplugged probe as well as a shorted one, and whatever an open 1 MOhm
      // input picks up out of the room is worth eighty times less in codes at
      // 10 V/div than at 50 mV/div. The bend being measured lives past the
      // attenuator, so the range it is measured on does not otherwise matter.
      autoset_set_scale(VS_LAST);
      autocal_set_position(0);
      lin_start();
      return;
    }

    {
      char msg[40];

      snprintf(msg, sizeof(msg), "Step 3 of 4: DAC step, range %d of 8",
          g_autocal_range + 1);
      autocal_say(msg, "Keep the input shorted");
    }

    autoset_set_scale(g_autocal_range);
    autocal_set_position(0);
    return;
  }

  if (ACAL_LIN == g_autocal_phase)
  {
    int a_x100, b_x100;

    // Both converters at once, which is the only reason the sweep is worth the
    // seconds it takes: they do not have the same curve, and separating them
    // afterwards from an interleaved record is impossible.
    if (!capture_get_channel_means(&a_x100, &b_x100))
    {
      autoset_set_scale(g_autocal_saved_vs);
      autocal_set_position(0);
      g_autocal_phase = ACAL_ASK_REF;
      g_autocal_timer = TIMER_DISABLE;
      gain_note("Linearity: both converters have to be running", 4000);
      return;
    }

    lin_accumulate(0, b_x100);
    lin_accumulate(1, a_x100);

    // Per step rather than for the whole sweep: thirty-three of them at six
    // records each is longer than the twenty seconds the rest of the run is
    // allowed, and what the watchdog is for is a step that never lands.
    g_autocal_timer = AUTOCAL_TIMEOUT;
    g_lin_i++;

    if (g_lin_i > ACAL_LIN_M)
    {
      lin_done();
      return;
    }

    if (0 == ((g_lin_i + ACAL_LIN_M) & 7))
    {
      char msg[52];

      snprintf(msg, sizeof(msg), "Step 4 of 4: linearity, %d%%",
          (g_lin_i + ACAL_LIN_M) * 100 / ACAL_LIN_N);
      autocal_say(msg, "Keep the input shorted");
    }

    lin_set_dac();
    return;
  }

  if (ACAL_SCALE == g_autocal_phase)
  {
    int ref_mv = scope_calib_ref_mv();
    int counts_x100, reading_mv;
    int64_t mult;

    if (!capture_get_raw_measure_fresh(&m))
      return;

    // The mean with the measured bend already taken out of it: gain is a
    // straight multiplier and can only be trimmed against a straight chain.
    // Calibrated against a bent one it would be right at the level used and
    // wrong either side of it - which is exactly the failure that made the
    // linearity step above worth having.
    counts_x100 = capture_measure_mean_x100(&m);

    // The reference has to actually be there, and it has to fit on the ADC.
    // Calibrating gain against a clipped or near-zero reading would scale the
    // whole range by a made-up number and quietly ruin it.
    if (m.vmax >= 254 || m.vmin <= 1)
    {
      gain_refuse("Clipping - this range is too fine for that level",
          "Signal is clipping - pick a coarser V/div", "MODE closes");
      return;
    }

    if (counts_x100 < CALIB_REF_MIN_COUNTS * 100)
    {
      gain_refuse("Reading too small or wrong polarity - not trimmed",
          "Reading too small or wrong polarity",
          "Check the level and the range.  MODE closes");
      return;
    }

    reading_mv = (int)((int64_t)counts_x100 *
        config.calib_vs_mult[config.vertical_scale] / CALIB_MULTIPLIER / 100);

    if (reading_mv <= 0)
    {
      gain_refuse("Reading too small or wrong polarity - not trimmed",
          "Reading too small or wrong polarity",
          "Check the level and the range.  MODE closes");
      return;
    }

    // Gain is a straight multiplier, so one proportional step lands exactly
    mult = (int64_t)config.calib_vs_mult[config.vertical_scale] * ref_mv /
        reading_mv;

    if (mult < 1)
      mult = 1;
    else if (mult > 4000000)
      mult = 4000000;

    {
      // Report the size of the correction rather than the multiplier itself.
      // 20449 to 19764 means nothing to anyone; -3.4% is the number that says
      // whether this was a trim or a mistake, and it is the same number that
      // was on the line before MODE was pressed.
      int64_t was = config.calib_vs_mult[config.vertical_scale];
      int err = (int)((was - mult) * 1000 / (was > 0 ? was : 1));

      // A trim is a few percent. Half a range is not a trim, it is the wrong
      // range or the wrong number typed, and the ADC has nothing to say about
      // it: 400 mV at 100 mV/div is four divisions up, off the top of the
      // GRID but nowhere near the end of the converter's window, so the
      // clipping guard above lets it through. Committed, it would multiply
      // that range by 2.5 and every reading on it afterwards.
      if (err > GAIN_TRIM_LIMIT || err < -GAIN_TRIM_LIMIT)
      {
        char msg[80];
        char sign = (err > 0) ? '-' : '+';

        if (err < 0)
          err = -err;

        snprintf(msg, sizeof(msg), "%c%d%% is not a trim - wrong range or level?",
            sign, err / 10);
        gain_refuse(msg, "Correction too large to be a calibration",
            "Check the level and the range.  MODE closes");
        return;
      }

      config.calib_vs_mult[config.vertical_scale] = (int)mult;
      config.vertical_mult = (int)mult;
      g_autocal_did_gain = true;

      if (g_autocal_gain_only)
      {
        char msg[80];
        char sign = (err > 0) ? '-' : '+';

        if (err < 0)
          err = -err;

        g_autocal_gain_done++;

        snprintf(msg, sizeof(msg), "%s/div gain %c%d.%d%% - now reads %s",
            gain_trim(vs_label(config.vertical_scale)), sign, err / 10,
            err % 10, gain_trim(format_voltage(gain_ref_tip_mv(), false)));
        gain_note(msg, 2500);

        g_autocal_phase = ACAL_ASK_REF;
        g_autocal_timer = TIMER_DISABLE;
        autocal_mark();
        return;
      }
    }

    autocal_finish(true);
    return;
  }

  autocal_finish(true);
}

//-----------------------------------------------------------------------------
void autocal_step(void)
{
  // Both ends of the run are conversations, not measurements
  if (ACAL_ASK == g_autocal_phase || ACAL_ASK_REF == g_autocal_phase ||
      ACAL_DONE == g_autocal_phase)
  {
    // ...except the gain pass, where the conversation IS a measurement: what
    // it says is the live reading against the expected level, so the user can
    // see the error settle before committing to it. mpanel_set_lines() drops a
    // line identical to the one already up, so a steady reading is not a
    // repaint.
    if (ACAL_ASK_REF == g_autocal_phase)
      gain_prompt();

    update_display(); // just keep the trace alive under the message
    return;
  }

  if (autocal_ready())
    autocal_advance();

  // EVERY path has to release the frame. update_storage_buffer() refills the
  // storage record only once the previous one has been consumed, so a step
  // that merely reads a measurement and returns freezes acquisition on that
  // one frame - and the generation counter this machine waits on stops with
  // it, deadlocking until the timeout. autocal_finish() repaints on its own.
  if (g_autocal_active)
    update_display();
}

//-----------------------------------------------------------------------------
void scope_autocal_start(void)
{
  g_autocal_saved_hs    = config.horizontal_scale;
  g_autocal_saved_vs    = config.vertical_scale;
  g_autocal_saved_vpos  = config.vertical_position;
  g_autocal_saved_srl   = config.sample_rate_limit;
  g_autocal_saved_tmode = config.trigger_mode;

  g_autocal_active = true;
  g_autocal_phase  = ACAL_ASK;
  g_autocal_did_gain = false;

  // Ask first, touch nothing yet. autocal_begin() sets up the run itself,
  // once there is something worth running against. Every phase below measures the instrument
  // against itself, and all three of them are only meaningful with a quiet
  // input - so the one thing this cannot do without is the user shorting it.
  // No timer while it waits: fetching a wire takes as long as it takes.
  g_autocal_timer = TIMER_DISABLE;
  g_autocal_gain_only = false;
  autocal_say("Short the BNC to ground (or unplug the probe)",
      "MODE starts   STOP cancels");
}

//-----------------------------------------------------------------------------
// The gain pass on its own. Nothing here needs a shorted input - it needs the
// opposite - so it starts where the auto-calibration run ends, and it stays
// there: trim, change the range or the source, trim again, until the ranges
// that matter have been done.
void scope_calib_gain_start(void)
{
  g_autocal_saved_hs    = config.horizontal_scale;
  g_autocal_saved_vs    = config.vertical_scale;
  g_autocal_saved_vpos  = config.vertical_position;
  g_autocal_saved_srl   = config.sample_rate_limit;
  g_autocal_saved_tmode = config.trigger_mode;

  g_autocal_active    = true;
  g_autocal_gain_only = true;
  g_autocal_gain_done = 0;
  g_autocal_did_gain  = false;
  g_autocal_phase     = ACAL_ASK_REF;
  g_autocal_timer     = TIMER_DISABLE; // finding a reference takes as long as it takes
  g_gain_note_until   = 0;

  // A DC level never crosses a trigger threshold, so a normal-mode instrument
  // would sit here showing the last frame it caught and calibrate against it.
  // AUTO sweeps regardless, which is the whole reason the auto-calibration run
  // uses it too.
  config.trigger_mode = TRIGGER_MODE_AUTO;
  capture_set_trigger_mode(config.trigger_mode);
  draw_trigger_mode();

  // The reading is the mean against the centre line, so the trace has to be
  // on it: a panned trace would have the position folded into the gain.
  autocal_set_position(0);
  gain_prompt();
}

//-----------------------------------------------------------------------------
// The user confirmed: from here on it drives the hardware itself
void autocal_begin(void)
{
  g_autocal_timer = AUTOCAL_TIMEOUT;
  g_autocal_phase = ACAL_ZERO;
  g_autocal_sub = 0;
  g_autocal_lo = CALIB_DAC_ZERO_MIN;
  g_autocal_hi = CALIB_DAC_ZERO_MAX;

  // Free-running acquisition on a short record, trace centred, and the single
  // converter first. AUTO trigger guarantees frames even with a flat input,
  // which every phase here depends on.
  config.trigger_mode = TRIGGER_MODE_AUTO;
  config.sample_rate_limit = 1; // single channel: ADC B alone
  config.calib_dac_zero = (g_autocal_lo + g_autocal_hi) / 2;

  capture_set_trigger_mode(config.trigger_mode);
  autoset_set_horizontal(AUTOCAL_HS);
  autoset_set_scale(0);
  autocal_set_position(0);
  update_sample_rate();
  capture_start();

  draw_trigger_mode();
  autocal_say("Step 1 of 4: zeroing the offset DAC",
      "Keep the input shorted");
}

//-----------------------------------------------------------------------------
void change_calibration_value(int delta, bool shift, bool repeat)
{
  // Gain and DAC step live in the thousands, so a fixed step of one turns a
  // 4% correction into two hundred keypresses. Accelerate on auto-repeat,
  // and only there: a single press still has to be able to land on an exact
  // value. Zero and delta have small ranges and stay fine-grained.
  if (repeat)
  {
    if (CALIB_SCALE == g_calibration_parameter ||
        CALIB_OFFSET == g_calibration_parameter)
      delta *= 16;
    else if (CALIB_ZERO == g_calibration_parameter)
      delta *= 4;
  }

  if (shift)
  {
    if (repeat)
      return; // holding SHIFT+TRIG would otherwise race through all four

    g_calib_hint = true; // every step explains itself, even if the last was hidden

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

    if (config.calib_dac_zero < CALIB_DAC_ZERO_MIN)
      config.calib_dac_zero = CALIB_DAC_ZERO_MIN;
    else if (config.calib_dac_zero > CALIB_DAC_ZERO_MAX)
      config.calib_dac_zero = CALIB_DAC_ZERO_MAX;
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

  scope_calib_apply(CALIB_ZERO == g_calibration_parameter ||
      CALIB_OFFSET == g_calibration_parameter);
}

//-----------------------------------------------------------------------------
// Re-apply a changed calibration value.
//
// Only the two parameters that feed the offset DAC need the hardware touched,
// and touching it means dma_stop() / dma_start(). Doing that on every
// auto-repeat tick restarts acquisition faster than a frame can complete,
// which is why holding a key froze the screen - including on the gain, which
// is a pure display multiplier and needs no hardware at all.
void scope_calib_apply(bool touch_dac)
{
  config.vertical_mult = config.calib_vs_mult[config.vertical_scale] * config_probe_mult();

  // The trigger level is stored in mV and converted through the gain, so it
  // has to follow it. This one does not restart the DMA.
  capture_set_trigger_level(config.trigger_level_mv);

  if (touch_dac)
    capture_set_vertical_parameters();
}

//-----------------------------------------------------------------------------
// What each calibration step actually wants, in the panel band over the
// trace. Line 0 is the parameter and its precondition, line 1 is the same key
// legend everywhere - 50 characters at 6 px is exactly the grid width, so
// neither line may grow.
void calib_hint_update(void)
{
  static const char *const hint[] =
  {
    "Z: SHORT the input, 1 channel, aim avg at 128",
    "D: SHORT the input, 2 channels, make A = B",
    "S: min and max to a known level, raw 0xd0-0xf0",
    "O: 1 position px = 1 screen px, per range",
  };

  mpanel_set_active(mpanel_wanted());

  if (!g_calib_hint)
  {
    g_calib_hint_param = -1;
    return;
  }

  if (g_calib_hint_param == g_calibration_parameter)
    return;

  g_calib_hint_param = g_calibration_parameter;
  mpanel_set_lines(hint[g_calibration_parameter],
      "TRIG_U/D set  SHIFT+TRIG next  AUTO all  MODE off");
}

//-----------------------------------------------------------------------------
void draw_calibration_info(void)
{
  static const char *labels[] = { "Z", "D", "S", "O" };
  char *str;

  calib_hint_update();

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

    // Four instantaneous samples of an untriggered ring slice is the worst
    // possible readout to aim a value with: it is raw noise, taken from a
    // different part of the ring every frame, which is why the number used to
    // jump about at random. The per-converter MEANS are what the procedure
    // actually asks for - 0x80 = 128 for zero, and equal to each other for
    // delta - and averaging thousands of samples makes them hold still.
    if (!g_toast_active)
    {
      char buf[20];
      int a, b;

      buf[0] = 0;

      if (capture_get_channel_means(&a, &b))
      {
        // Show A as the record sees it, i.e. with the delta already applied:
        // the goal is the two reading the same
        snprintf(buf, sizeof(buf), "A%4d B%4d",
            (a + 50) / 100 + config.calib_channel_delta, (b + 50) / 100);
      }
      else
      {
        Measure raw;

        if (capture_get_raw_measure(&raw))
          snprintf(buf, sizeof(buf), "avg%4d", raw.mean_c100 / 100 + ZERO_POINT);
      }

      if (buf[0])
        lcd_puts(CALIB_AREA_LEFT + 24, STATUS_LINE_Y, buf);
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

    // Minimum and maximum, as the original procedure documents: "adjust until
    // the minimum and maximum values in the status bar match the measured
    // value". They only converge on a quiet instrument, though - a single
    // sampling glitch drags them hundreds of millivolts apart - so the mean
    // goes in the hint band underneath, where it can be aimed when they do
    // not agree.
    if (!g_toast_active)
    {
      str = format_voltage(vmin - vpos, true);
      lcd_puts(CALIB_AREA_LEFT + 12, STATUS_LINE_Y, str);

      str = format_voltage(vmax - vpos, true);
      lcd_puts(CALIB_AREA_LEFT + 96, STATUS_LINE_Y, str);
    }
  }
}

