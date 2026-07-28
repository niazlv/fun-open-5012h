/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Signal type classification from measurement statistics and (optionally)
 * an FFT magnitude spectrum.
 *
 * Pure C, no hardware dependencies: this module is also compiled and tested
 * on the host against synthetic waveforms (see tests/host_test.c).
 *
 * Decision tree (all thresholds validated by the host tests):
 *   - peak-to-peak below noise floor            -> DC
 *   - no periodicity found                      -> NOISE
 *   - duty cycle far from 50%                   -> PULSE
 *   - AC-RMS / (Vpp/2) ratio ("crest" inverse):
 *       square wave ~1.00, sine ~0.707, triangle/saw ~0.577
 *   - triangle vs sawtooth: a sawtooth spends almost all its samples on one
 *     slope (rising-vs-falling balance from measure_run), confirmed by the
 *     strong 2nd harmonic when the spectrum resolves it
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "measure.h"
#include "classify.h"

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint32_t isqrt32(uint32_t value)
{
  uint32_t result = 0;
  uint32_t bit = 1ul << 30;

  while (bit > value)
    bit >>= 2;

  while (bit)
  {
    if (value >= result + bit)
    {
      value -= result + bit;
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }

    bit >>= 2;
  }

  return result;
}

//-----------------------------------------------------------------------------
const char *classify_name(SignalType type)
{
  switch (type)
  {
    case SIG_DC:       return "DC";
    case SIG_NOISE:    return "Noise";
    case SIG_SINE:     return "Sine";
    case SIG_SQUARE:   return "Square";
    case SIG_PULSE:    return "Pulse";
    case SIG_TRIANGLE: return "Triangle";
    case SIG_SAW:      return "Sawtooth";
    default:           return "----";
  }
}

//-----------------------------------------------------------------------------
void classify_signal(const Measure *m, const float *mag, int bins,
    int fund_bin, SignalClass *out)
{
  out->type = SIG_UNKNOWN;
  out->thd_x10 = -1;

  // Robust amplitude: histogram percentiles, so glitch spikes, overshoot
  // and noise peaks do not inflate the swing
  int vpp = m->top - m->base;

  if (vpp < 6)
  {
    out->type = SIG_DC;
    return;
  }

  if (m->frequency == 0)
  {
    out->type = SIG_NOISE;
    return;
  }

  // A real periodic signal repeats regularly: most periods sit near the
  // median. Random noise fails this badly, while a few glitch spikes on an
  // otherwise clean signal only dent the percentage.
  //
  // Data-carrying digital lines fail it too — UART frames and bus traffic
  // are irregular by nature — so an unmistakably two-level record is called
  // a pulse train rather than noise. Noise never rests at its extremes.
  if (m->periods >= 3 && m->period_good_pct < 60)
  {
    out->type = (m->level_pct >= 65) ? SIG_PULSE : SIG_NOISE;
    return;
  }

  // THD from the harmonics when the spectrum resolves them
  if (mag != NULL && fund_bin >= 3 && fund_bin * 2 < bins)
  {
    float fund = mag[fund_bin];

    // The true fundamental may sit one bin off the predicted one
    if (fund_bin + 1 < bins && mag[fund_bin + 1] > fund) fund = mag[fund_bin + 1];
    if (mag[fund_bin - 1] > fund) fund = mag[fund_bin - 1];

    if (fund > 0.0f)
    {
      float sum_sq = 0.0f;

      for (int h = 2; h <= 5; h++)
      {
        int b = fund_bin * h;

        if (b >= bins - 1)
          break;

        float hm = mag[b];

        if (mag[b - 1] > hm) hm = mag[b - 1];
        if (mag[b + 1] > hm) hm = mag[b + 1];

        sum_sq += hm * hm;
      }

      float thd = sum_sq / (fund * fund); // squared ratio
      // integer sqrt keeps the module float-light: thd_x10 = 1000*sqrt(thd)
      out->thd_x10 = (int)isqrt32((uint32_t)(thd * 1000000.0f));
    }
  }

  // Pulse trains: mid-level duty far from 50%
  if (m->duty_x10 >= 0 && (m->duty_x10 < 400 || m->duty_x10 > 600))
  {
    out->type = SIG_PULSE;
    return;
  }

  // AC RMS (about the mean, not the zero point), counts*100
  int64_t rms2 = (int64_t)m->rms_c100 * m->rms_c100 -
      (int64_t)m->mean_c100 * m->mean_c100;

  if (rms2 < 0)
    rms2 = 0;

  int ac_rms_c100 = (int)isqrt32((uint32_t)rms2);
  int vpp_c100 = vpp * 100;

  // ratio_x1000 = AC_RMS / (Vpp/2), scaled by 1000
  int ratio = (int)((int64_t)ac_rms_c100 * 2000 / vpp_c100);

  if (ratio > 850)
  {
    out->type = SIG_SQUARE;
    return;
  }

  // Slope balance: fraction of moving samples that are rising
  int moving = m->slope_pos + m->slope_neg;

  if (moving > 0)
  {
    int rising_x100 = m->slope_pos * 100 / moving;

    if (rising_x100 > 75 || rising_x100 < 25)
    {
      out->type = SIG_SAW;
      return;
    }
  }

  out->type = (ratio >= 640) ? SIG_SINE : SIG_TRIANGLE;
}
