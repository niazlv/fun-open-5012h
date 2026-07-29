/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Above-nyquist reasoning for the spectrum. See alias.h for what the two
 * entry points promise and, more to the point, what they refuse to promise.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "alias.h"

/*- Definitions -------------------------------------------------------------*/
// Bins either side of a predicted position that are searched for its line,
// and how close two positions may be before they are the same place. Both
// match the tolerance fft_analyze() walks its harmonic comb with.
#define SLOT_HALF_WIDTH   2

// A line closer than this to bin 0 is inside the window's DC leakage
#define DC_GUARD_BINS     3

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Tallest bin within SLOT_HALF_WIDTH of `center`
static float slot_magnitude(const float *mag, int center)
{
  float m = 0.0f;

  for (int b = center - SLOT_HALF_WIDTH; b <= center + SLOT_HALF_WIDTH; b++)
  {
    if (b > 0 && b < FFT_BINS && mag[b] > m)
      m = mag[b];
  }

  return m;
}

//-----------------------------------------------------------------------------
// Spectrum floor measured where the signal is NOT: the mean of every bin
// that is not part of a detected peak or of a harmonic slot about to be
// read.
//
// fft_analyze's own floor is the mean of ALL bins, which is the right figure
// for scaling a display and the wrong one here. A clean square puts three or
// four tall lines into 256 bins and drags that mean up by 30 dB or more - a
// floor that then sits above the level this module decides at, and every
// honest square gets answered "cannot tell" (measured: a synthetic 17 MHz
// square, floor -32 dB by the mean and -70 dB with the lines taken out).
//
// Returns -1 when the exclusions leave too little spectrum to average, so
// the caller can fall back to the figure fft_analyze already carries.
static float clear_floor(const float *mag, const FftAnalysis *an,
    const int *used, int used_n, int f0_bin)
{
  float sum = 0.0f;
  int n = 0;

  for (int b = 2; b < FFT_BINS; b++)
  {
    bool taken = (abs(b - f0_bin) <= SLOT_HALF_WIDTH + 2);

    for (int u = 0; u < used_n && !taken; u++)
      taken = (abs(b - used[u]) <= SLOT_HALF_WIDTH + 2);

    for (int p = 0; p < an->count && !taken; p++)
      taken = (abs(b - an->peak[p].bin) <= SLOT_HALF_WIDTH + 2);

    if (!taken)
    {
      sum += mag[b];
      n++;
    }
  }

  return (n >= 16) ? (sum / (float)n) : -1.0f;
}

//-----------------------------------------------------------------------------
int alias_candidates(float observed_hz, float nyquist_hz, float limit_hz,
    AliasCandidate *out, int max)
{
  float fs = 2.0f * nyquist_hz;
  int n = 0;

  if (!out || max <= 0 || observed_hz <= 0.0f || nyquist_hz <= 0.0f)
    return 0;

  out[n].freq = observed_hz;
  out[n].order = 0;
  n++;

  for (int k = 1; n < max; k++)
  {
    float lo = (float)k * fs - observed_hz;
    float hi = (float)k * fs + observed_hz;

    if (lo > limit_hz)
      break;

    out[n].freq = lo;
    out[n].order = k;
    n++;

    if (n >= max || hi > limit_hz)
      break;

    out[n].freq = hi;
    out[n].order = k;
    n++;
  }

  return n;
}

//-----------------------------------------------------------------------------
void alias_check(const float *mag, const FftAnalysis *an, float bandwidth_hz,
    AliasAnalysis *out)
{
  float f0, nyquist, bin_hz, floor_mag, h1, best = 0.0f;
  int used[ALIAS_HARM_LAST];
  int used_n = 0, f0_bin;
  bool past_frontend = false;

  out->verdict  = ALIAS_NO_EVIDENCE;
  out->slots    = 0;
  out->harm_db  = 0.0f;
  out->floor_db = 0.0f;
  out->count    = 0;
  out->reason   = "no peak to reason about";

  if (!mag || !an || an->count == 0 || an->fundamental <= 0.0f ||
      an->bin_hz <= 0.0f)
    return;

  f0      = an->fundamental;
  nyquist = an->nyquist_hz;
  bin_hz  = an->bin_hz;
  f0_bin  = (int)(f0 / bin_hz + 0.5f);

  // The ladder does not depend on any of the reasoning below and is worth
  // having even when the verdict comes out ALIAS_NO_EVIDENCE: it is the
  // complete list either way.
  out->count = alias_candidates(f0, nyquist,
      (float)ALIAS_LADDER_MULT * bandwidth_hz, out->cand, ALIAS_MAX_CANDIDATES);

  h1 = slot_magnitude(mag, f0_bin);

  if (h1 <= 0.0f)
    return;

  for (int n = 3; n <= ALIAS_HARM_LAST; n += 2)
  {
    float true_hz = (float)n * f0;
    int center;
    bool dup = false;

    // Past the frontend the line would be missing whether the reading is
    // honest or not, so its absence says nothing. Harmonics only climb from
    // here, so nothing below can qualify either.
    if (true_hz > bandwidth_hz)
    {
      past_frontend = (3 == n);
      break;
    }

    center = (int)(fft_fold_hz(true_hz, nyquist) / bin_hz + 0.5f);

    // Folded onto DC, onto the fundamental, or onto a slot already counted:
    // there is nothing at that bin that could be told apart from what is
    // already there
    if (center < DC_GUARD_BINS || center >= FFT_BINS - 1)
      continue;

    if (abs(center - f0_bin) <= SLOT_HALF_WIDTH)
      continue;

    for (int u = 0; u < used_n; u++)
    {
      if (abs(center - used[u]) <= SLOT_HALF_WIDTH)
        dup = true;
    }

    if (dup)
      continue;

    used[used_n++] = center;
    out->slots++;
  }

  for (int u = 0; u < used_n; u++)
  {
    float m = slot_magnitude(mag, used[u]);

    if (m > best)
      best = m;
  }

  floor_mag = clear_floor(mag, an, used, used_n, f0_bin);

  out->harm_db  = (best > 0.0f) ? 20.0f * log10f(best / h1) : -999.0f;

  // ALIAS_MIN_SNR in dB, spelled out rather than computed: a log10f() of a
  // constant is not worth the libm call on a part with no flash left.
  out->floor_db = (floor_mag > 0.0f) ?
      (20.0f * log10f(floor_mag / h1) + ALIAS_MIN_SNR_DB) :
      (an->noise_db + ALIAS_MIN_SNR_DB);

  if (0 == out->slots)
  {
    out->reason = past_frontend ? "h3 is past the frontend" :
        "harmonics fold onto DC or F0";
    return;
  }

  // A floor above the decision level means "no h3" was never observable in
  // the first place - the record simply has no dynamic range left to say it
  // with.
  if (out->floor_db > ALIAS_HARM_MIN_DB)
  {
    out->reason = "noise floor hides h3";
    return;
  }

  if (out->harm_db >= ALIAS_HARM_MIN_DB)
  {
    out->verdict = ALIAS_IN_BAND;
    out->reason  = "harmonics stand where only an in-band source puts them";
  }
  else
  {
    out->verdict = ALIAS_POSSIBLE;
    out->reason  = "no harmonics: a sine, or edges from above nyquist";
  }
}
