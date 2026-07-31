/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Amplitude-modulation detection. See am.h for what am_analyze() promises
 * and, more to the point, for the three things it refuses to.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "am.h"

/*- Definitions -------------------------------------------------------------*/
// Bins either side of a line that belong to it rather than to the floor:
// the Hann window spreads a tone over three, and its skirt over the next
// few. Matches the guard the peak finder in fft.c keeps.
#define LINE_HALF_WIDTH   3

// Fraction of the envelope's span that still counts as sitting AT one of
// its two levels, when deciding whether keying explains the record
#define LEVEL_BAND        0.1f

// Bins the floor is averaged over before it is trusted. Under this the
// exclusions have eaten the spectrum and the figure means nothing.
#define FLOOR_MIN_BINS    16

/*- Variables ---------------------------------------------------------------*/
// The envelope, the same series requantised for fft_spectrum(), and its
// spectrum. Static rather than automatic for the reason fft.c's own buffers
// are: this runs on a part where a 3.5 KB stack frame at panel cadence is
// not on offer.
static float   g_env[AM_ENV_POINTS];
static uint8_t g_env8[AM_ENV_POINTS];
static float   g_mag[FFT_BINS];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// RMS of one block about its OWN mean, so a DC offset on the record never
// reaches the envelope. (A DC-coupled carrier sitting off centre would
// otherwise add a constant to every envelope point and read as a shallower
// modulation than it is.)
//
// The sums are integer and therefore exact; the one cancelling subtraction
// happens once per block in double, where 53 bits of mantissa leave the
// result of ~16400 - 16384 with digits to spare. In float it would leave
// three, and a 1% depth would be noise.
static float block_rms(const uint8_t *data, int size, int start, int n)
{
  uint32_t sum = 0;
  uint64_t sqsum = 0;
  int idx = start;
  double mean, var;

  for (int i = 0; i < n; i++)
  {
    uint32_t v = data[idx];

    sum += v;
    sqsum += (uint64_t)v * v;

    if (++idx >= size)
      idx = 0;
  }

  mean = (double)sum / (double)n;
  var  = ((double)sqsum - (double)sum * mean) / (double)n;

  return (var > 0.0) ? (float)sqrt(var) : 0.0f;
}

//-----------------------------------------------------------------------------
// Cut the record into AM_ENV_POINTS blocks and take the RMS of each.
// Returns the envelope's mean, which is the A0 every depth is divided by.
//
// The block-RMS estimator has a known error, and it is worth naming because
// two of the gates below exist to stay clear of it. The RMS of a sinusoid
// over a window that does not hold a whole number of cycles depends on where
// in the cycle the window opened: over a block of length Tb the mean of
// A^2*cos^2(2*pi*fa*t + phi) is A^2/2 plus a term swinging at 2*fa with the
// block's centre, suppressed by sinc(2*fa*Tb), i.e. about 1/(4*pi*cycles).
// Sampled once per block, that term arrives folded somewhere into the
// envelope band - as a clean line, at a plausible rate, on a carrier that is
// not modulated at all.
static float build_envelope(const uint8_t *data, int size, int offset,
    int block, float *env, float *min_out, float *max_out)
{
  float emin = 255.0f, emax = 0.0f, sum = 0.0f;
  int idx = offset % size;

  for (int i = 0; i < AM_ENV_POINTS; i++)
  {
    float e = block_rms(data, size, idx, block);

    env[i] = e;
    sum += e;

    if (e < emin) emin = e;
    if (e > emax) emax = e;

    idx += block;

    if (idx >= size)
      idx -= size;
  }

  if (min_out) *min_out = emin;
  if (max_out) *max_out = emax;

  return sum / (float)AM_ENV_POINTS;
}

//-----------------------------------------------------------------------------
// What the block average does to an envelope component at `rate_hz`.
//
// Averaging over a block IS a boxcar filter, so the envelope arrives
// low-passed: a component at fm comes out scaled by sinc(fm*Tb), which is
// 0.98 a tenth of the way up the envelope band and 0.64 at the top of it.
// Measured before this was divided back out: -4% at 150 kHz, -17% at 300 kHz
// and -41% at 450 kHz on a 488 kHz band.
//
// It is a known response rather than a limit, and dividing it back out is
// bounded: sinc does not reach its first zero until twice the envelope's
// nyquist, so the correction never exceeds 1.57x and cannot run away.
static float boxcar_gain(float rate_hz, double block_period_s)
{
  float x = (float)(M_PI * (double)rate_hz * block_period_s);

  return (x > 1e-6f) ? (sinf(x) / x) : 1.0f;
}

//-----------------------------------------------------------------------------
// Tallest bin within LINE_HALF_WIDTH of `center`
static float slot_magnitude(const float *mag, int center)
{
  float m = 0.0f;

  for (int b = center - LINE_HALF_WIDTH; b <= center + LINE_HALF_WIDTH; b++)
  {
    if (b > 0 && b < FFT_BINS && mag[b] > m)
      m = mag[b];
  }

  return m;
}

//-----------------------------------------------------------------------------
// Spectrum floor measured where the lines are not - the same lesson alias.c
// learned: a mean over ALL bins is dragged up by the very lines it is being
// compared against, and a deep clean modulation would then fail its own SNR
// gate. Returns -1 when the exclusions leave too little to average.
static float line_floor(const float *mag, const FftAnalysis *an)
{
  float sum = 0.0f;
  int n = 0;

  for (int b = 2; b < FFT_BINS; b++)
  {
    bool taken = false;

    for (int p = 0; p < an->count && !taken; p++)
      taken = (abs(b - an->peak[p].bin) <= LINE_HALF_WIDTH + 1);

    if (!taken)
    {
      sum += mag[b];
      n++;
    }
  }

  return (n >= FLOOR_MIN_BINS) ? (sum / (float)n) : -1.0f;
}

//-----------------------------------------------------------------------------
// Amplitude of the component at `bins` (fractional) of a mean-removed,
// Hann-windowed series, by Goertzel.
//
// Deliberately NOT read off the FFT magnitude: the transform's bins are
// whole numbers and a modulation almost never lands on one, so its energy
// splits between neighbours and the peak reads low by up to 1.4 dB. Goertzel
// evaluates at the interpolated position instead, which costs one pass and
// removes the scalloping entirely - and it leaves the depth reading
// independent of whatever scaling fft_spectrum() happens to apply.
//
// The window is generated in the loop rather than tabulated because a table
// would be another 2 KB of a part that has none to spare, and 512 cosf()
// calls at panel cadence do not show. Normalising by the window's own sum
// makes the result exact for any window, Hann or none.
static float line_amplitude(const float *x, int n, float mean, float bins)
{
  float w = 2.0f * (float)M_PI * bins / (float)n;
  float cw = cosf(w), sw = sinf(w), coeff = 2.0f * cw;
  float s1 = 0.0f, s2 = 0.0f, wsum = 0.0f;
  float re, im;

  for (int i = 0; i < n; i++)
  {
    float win = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i /
        (float)(n - 1)));
    float s0 = (x[i] - mean) * win + coeff * s1 - s2;

    wsum += win;
    s2 = s1;
    s1 = s0;
  }

  re = s1 - s2 * cw;
  im = s2 * sw;

  return (wsum > 0.0f) ? (2.0f * sqrtf(re * re + im * im) / wsum) : 0.0f;
}

//-----------------------------------------------------------------------------
const char *am_verdict_name(AmVerdict v)
{
  switch (v)
  {
    case AM_FLAT:      return "Flat";
    case AM_MODULATED: return "AM";
    case AM_BURST:     return "Burst";
    case AM_ERRATIC:   return "Erratic";
    default:           return "-";
  }
}

//-----------------------------------------------------------------------------
void am_analyze(const uint8_t *data, int size, int offset, int period_ns,
    float carrier_hz, AmAnalysis *out)
{
  FftAnalysis an;
  int64_t env_period_ns;
  float emin, emax, emean, span, floor_mag, peak_mag, min_depth, rate;
  int block, lo_n = 0, hi_n = 0, rate_bin;
  bool line;

  out->verdict          = AM_NO_EVIDENCE;
  out->depth            = 0.0f;
  out->depth_pp         = 0.0f;
  out->rate_hz          = 0.0f;
  out->snr_db           = 0.0f;
  out->carrier_hz       = carrier_hz;
  out->env_bin_hz       = 0.0f;
  out->env_nyquist_hz   = 0.0f;
  out->cycles_per_block = 0.0f;
  out->block            = 0;
  out->artifact         = false;
  out->reason           = "no carrier to measure an envelope on";

  if (!data || !out || size <= 0 || period_ns <= 0 || carrier_hz <= 0.0f)
    return;

  // The block is what makes the envelope span the whole record, so its
  // resolution is 1/record_time whatever the sample rate was
  block = size / AM_ENV_POINTS;

  if (block < 2)
  {
    out->reason = "record too short to cut into blocks";
    return;
  }

  env_period_ns = (int64_t)block * period_ns;

  if (env_period_ns > INT32_MAX)
  {
    out->reason = "record too slow for the envelope transform";
    return;
  }

  out->block            = block;
  out->env_bin_hz       = (float)(1e9 / ((double)env_period_ns * AM_ENV_POINTS));
  out->env_nyquist_hz   = out->env_bin_hz * FFT_BINS;
  out->cycles_per_block = (float)(carrier_hz * (double)env_period_ns * 1e-9);

  // Under this the block RMS is reading the carrier's phase, not the
  // envelope, and everything downstream would be the estimator talking to
  // itself
  if (out->cycles_per_block < AM_MIN_CYCLES)
  {
    out->reason = "carrier too slow for a block to hold";
    return;
  }

  emean = build_envelope(data, size, offset, block, g_env, &emin, &emax);
  span  = emax - emin;

  // An envelope of a few counts is the ADC's noise, and its ripple is the
  // noise's ripple rather than anybody's modulation
  if (emean < AM_MIN_CARRIER)
  {
    out->reason = "no carrier worth the name";
    return;
  }

  out->depth_pp = (emax + emin > 0.0f) ? (span / (emax + emin)) : 0.0f;

  if (span <= 0.0f)
  {
    out->verdict = AM_FLAT;
    out->reason  = "envelope is steady: CW, FM or PM";
    return;
  }

  // Requantise onto the envelope's OWN span rather than the ADC's: the 8
  // bits fft_spectrum() reads then cover the modulation instead of the
  // carrier's amplitude, so a 1% depth arrives with the same resolution a
  // 100% one does. What it costs is that a single outlying block sets the
  // span and crushes the rest - and a record like that has no single rate
  // to find anyway, so it lands in AM_ERRATIC where it belongs.
  for (int i = 0; i < AM_ENV_POINTS; i++)
  {
    float s = 255.0f * (g_env[i] - emin) / span;

    g_env8[i] = (uint8_t)(s < 0.0f ? 0 : s > 255.0f ? 255 : (int)(s + 0.5f));
  }

  fft_spectrum(g_env8, AM_ENV_POINTS, 0, g_mag);
  fft_analyze(g_mag, (int)env_period_ns, &an);

  for (int i = 0; i < AM_ENV_POINTS; i++)
  {
    if (g_env[i] < emin + LEVEL_BAND * span) lo_n++;
    if (g_env[i] > emax - LEVEL_BAND * span) hi_n++;
  }

  // The STRONGEST line, deliberately not fft_analyze()'s fundamental.
  //
  // That field is the root of a harmonic comb, which is the right answer for
  // a waveform - a mains record reads 50 Hz even when its 150 Hz harmonic is
  // the tallest line. An envelope is not a waveform: the lines in it are
  // independent, the modulation sitting beside the estimator's residual and
  // beside any other modulation present. Combing them knits unrelated lines
  // into one. Measured, a 3% modulation at 150 kHz next to a residual at
  // 180 kHz was explained as the 5th and 6th harmonics of a 30 kHz
  // fundamental that was not in the record, and the rate was then looked for
  // in an empty bin.
  //
  // A real modulation is always its envelope's strongest component: keying
  // and any other non-sinusoidal shape put harmonics ABOVE their own rate,
  // weaker than it, so the tallest line is the rate in every case.
  rate     = (an.count > 0) ? an.peak[0].freq : 0.0f;
  rate_bin = (an.count > 0) ? an.peak[0].bin : 0;

  floor_mag = line_floor(g_mag, &an);

  if (floor_mag <= 0.0f)
    floor_mag = -1.0f;

  peak_mag = (an.count > 0) ? slot_magnitude(g_mag, rate_bin) : 0.0f;

  if (peak_mag > 0.0f && floor_mag > 0.0f)
    out->snr_db = 20.0f * log10f(peak_mag / floor_mag);

  // A line at all: standing over the floor, and fast enough to be a rate
  // rather than the record tilting
  line = (an.count > 0 && rate > 0.0f && floor_mag > 0.0f &&
      peak_mag >= AM_MIN_SNR * floor_mag &&
      rate >= AM_MIN_RATE_BINS * an.bin_hz);

  if (line)
  {
    out->rate_hz = rate;
    out->depth   = line_amplitude(g_env, AM_ENV_POINTS, emean,
        rate / out->env_bin_hz) / emean /
        boxcar_gain(rate, (double)env_period_ns * 1e-9);

    // Shallower than the estimator's own ripple at this many carrier cycles
    // a block. The bound tightens as the carrier speeds up, so a fast
    // carrier is not held to a slow one's floor - but nothing is claimed
    // under AM_FLAT_DEPTH whatever the arithmetic says.
    min_depth = AM_RESIDUAL_K / out->cycles_per_block;

    if (min_depth < AM_FLAT_DEPTH)
      min_depth = AM_FLAT_DEPTH;

    if (out->depth < min_depth)
      line = false;
  }

  // Re-cut the record with a block one sample shorter and look for the same
  // line again. A modulation is indifferent to where the blocks fall; the
  // estimator's beat is MADE of where they fall, and moves by at least
  // 16/block of the envelope band. Nothing here needs the carrier's exact
  // frequency, which is what made predicting the beat's position unusable.
  if (line && block >= AM_CROSS_MIN_BLOCK)
  {
    float bin2 = (float)(1e9 / ((double)(block - 1) * period_ns *
        AM_ENV_POINTS));
    float mean2 = build_envelope(data, size, offset, block - 1, g_env,
        NULL, NULL);
    float depth2 = (mean2 > 0.0f) ?
        (line_amplitude(g_env, AM_ENV_POINTS, mean2, rate / bin2) / mean2 /
            boxcar_gain(rate, (double)(block - 1) * period_ns * 1e-9)) :
        0.0f;

    if (depth2 < AM_CROSS_KEEP * out->depth)
    {
      line = false;
      out->artifact = true;
    }
  }

  // Whatever was found did not survive. The extremes are still a reading,
  // and the only honest split left is between an envelope that stayed put
  // and one that moved without a rate to show for it.
  if (!line)
  {
    if (out->depth_pp <= AM_FLAT_DEPTH)
    {
      out->verdict = AM_FLAT;
      out->reason  = "envelope is steady: CW, FM or PM";
    }
    else
    {
      out->verdict = AM_ERRATIC;
      out->reason  = out->artifact ?
          "the only rate found is made of the blocks" :
          "envelope moves, but no one rate explains it";
    }

    return;
  }

  // Deep AND sitting at its two levels: sinusoidal AM reaches the same
  // depth but passes through its extremes instead of resting there
  if (out->depth_pp >= AM_BURST_DEPTH &&
      lo_n >= (int)(AM_BURST_LEVEL * AM_ENV_POINTS) &&
      hi_n >= (int)(AM_BURST_LEVEL * AM_ENV_POINTS))
  {
    out->verdict = AM_BURST;
    out->reason  = "envelope sits at two levels: keying or packets";
    return;
  }

  out->verdict = AM_MODULATED;
  out->reason  = "envelope carries one rate";
}
