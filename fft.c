/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Radix-2 FFT spectrum for the oscilloscope, sized for the Cortex-M4 FPU.
 *
 * Pure C (float + libm), no hardware dependencies: this module is also
 * compiled and tested on the host against synthetic waveforms.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "fft.h"

/*- Definitions -------------------------------------------------------------*/
#define PEAK_GUARD_BINS   3     // Hann main lobe: peaks closer than this are one
#define PEAK_CANDIDATES   24
#define PEAK_MIN_SNR      3.0f  // times the spectrum floor
#define COMB_MIN_BINS     3.0f  // a fundamental below this many bins cannot be
                                // told from its own neighbours — refuse it
#define COMB_MAX_DIVISOR  8     // subharmonics tried per peak

/*- Variables ---------------------------------------------------------------*/
static float g_re[FFT_SIZE];
static float g_im[FFT_SIZE];
static float g_window[FFT_SIZE / 2];   // Hann, symmetric half
static float g_tw_re[FFT_SIZE / 2];
static float g_tw_im[FFT_SIZE / 2];
static bool g_tables_ready = false;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void build_tables(void)
{
  for (int i = 0; i < FFT_SIZE / 2; i++)
  {
    g_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (FFT_SIZE - 1)));
    g_tw_re[i] = cosf(-2.0f * (float)M_PI * i / FFT_SIZE);
    g_tw_im[i] = sinf(-2.0f * (float)M_PI * i / FFT_SIZE);
  }

  g_tables_ready = true;
}

//-----------------------------------------------------------------------------
// Sum of `n` ring samples starting at *index, leaving *index past the last
static uint32_t box_sum(const uint8_t *data, int size, int *index, int n)
{
  uint32_t sum = 0;
  int i = *index;

  while (n--)
  {
    sum += data[i];

    if (++i == size)
      i = 0;
  }

  *index = i;

  return sum;
}

//-----------------------------------------------------------------------------
// Fill g_re with FFT_SIZE points taken `decim` samples apart. Each point is
// three consecutive boxes of `decim` samples weighted 1:2:1 — the triangular
// (3*decim wide) low-pass that keeps content above the decimated Nyquist
// from folding back down into the band we are about to display.
static void load_decimated(const uint8_t *data, int size, int offset, int decim)
{
  uint32_t prev, cur, next;
  float scale = 1.0f / (4.0f * (float)decim);
  int index = offset - decim;

  while (index < 0)
    index += size;

  prev = box_sum(data, size, &index, decim);
  cur  = box_sum(data, size, &index, decim);

  for (int i = 0; i < FFT_SIZE; i++)
  {
    next = box_sum(data, size, &index, decim);
    g_re[i] = (float)(prev + 2 * cur + next) * scale;
    prev = cur;
    cur = next;
  }
}

//-----------------------------------------------------------------------------
static void load_plain(const uint8_t *data, int size, int offset)
{
  int index = offset;

  for (int i = 0; i < FFT_SIZE; i++)
  {
    g_re[i] = (float)data[index];

    if (++index == size)
      index = 0;
  }
}

//-----------------------------------------------------------------------------
static void transform(float *mag)
{
  float sum = 0.0f;
  float dc;

  for (int i = 0; i < FFT_SIZE; i++)
    sum += g_re[i];

  dc = sum / FFT_SIZE;

  for (int i = 0; i < FFT_SIZE; i++)
  {
    int w = (i < FFT_SIZE / 2) ? i : (FFT_SIZE - 1 - i);
    g_re[i] = (g_re[i] - dc) * g_window[w];
    g_im[i] = 0.0f;
  }

  // Bit-reversal permutation
  for (int i = 1, j = 0; i < FFT_SIZE; i++)
  {
    int bit = FFT_SIZE >> 1;

    for (; j & bit; bit >>= 1)
      j ^= bit;

    j ^= bit;

    if (i < j)
    {
      float t = g_re[i]; g_re[i] = g_re[j]; g_re[j] = t;
      t = g_im[i]; g_im[i] = g_im[j]; g_im[j] = t;
    }
  }

  // Iterative radix-2 butterflies with the precomputed twiddle table
  for (int len = 2; len <= FFT_SIZE; len <<= 1)
  {
    int half = len >> 1;
    int step = FFT_SIZE / len;

    for (int i = 0; i < FFT_SIZE; i += len)
    {
      for (int k = 0; k < half; k++)
      {
        float wr = g_tw_re[k * step];
        float wi = g_tw_im[k * step];
        int a = i + k;
        int b = a + half;

        float tr = g_re[b] * wr - g_im[b] * wi;
        float ti = g_re[b] * wi + g_im[b] * wr;

        g_re[b] = g_re[a] - tr;
        g_im[b] = g_im[a] - ti;
        g_re[a] += tr;
        g_im[a] += ti;
      }
    }
  }

  for (int i = 0; i < FFT_BINS; i++)
    mag[i] = sqrtf(g_re[i] * g_re[i] + g_im[i] * g_im[i]);
}

//-----------------------------------------------------------------------------
void fft_spectrum(const uint8_t *data, int size, int offset, float *mag)
{
  if (!g_tables_ready)
    build_tables();

  load_plain(data, size, offset);
  transform(mag);
}

//-----------------------------------------------------------------------------
void fft_spectrum_decim(const uint8_t *data, int size, int offset, int decim,
    float *mag)
{
  if (!g_tables_ready)
    build_tables();

  if (decim <= 1)
    load_plain(data, size, offset);
  else
    load_decimated(data, size, offset, decim);

  transform(mag);
}

//-----------------------------------------------------------------------------
int fft_max_decimation(int size)
{
  int decim = size / FFT_SIZE;

  return (decim < 1) ? 1 : decim;
}

//-----------------------------------------------------------------------------
// Sub-bin position and height of the peak at `bin`, from the parabola
// through its two neighbours
static float interpolate(const float *mag, int bin, float *height)
{
  float y0 = mag[bin - 1], y1 = mag[bin], y2 = mag[bin + 1];
  float denom = y0 - 2.0f * y1 + y2;
  float delta = 0.0f;

  if (denom != 0.0f)
  {
    delta = 0.5f * (y0 - y2) / denom;

    if (delta > 0.5f) delta = 0.5f;
    else if (delta < -0.5f) delta = -0.5f;
  }

  if (height)
    *height = y1 - 0.25f * (y0 - y2) * delta;

  return delta;
}

//-----------------------------------------------------------------------------
int fft_peak_frequency(const float *mag, int sample_period_ns)
{
  // Skip the first bins: DC leakage from the Hann window spreads into them
  int peak = 2;

  for (int i = 3; i < FFT_BINS - 1; i++)
  {
    if (mag[i] > mag[peak])
      peak = i;
  }

  if (mag[peak] <= 0.0f)
    return 0;

  // Require the peak to stand out of the spectrum floor
  float sum = 0.0f;

  for (int i = 2; i < FFT_BINS; i++)
    sum += mag[i];

  if (mag[peak] < 4.0f * (sum / (FFT_BINS - 2)))
    return 0;

  double fs = 1e9 / (double)sample_period_ns;
  float delta = interpolate(mag, peak, NULL);

  return (int)(((double)peak + delta) * fs / FFT_SIZE);
}

//-----------------------------------------------------------------------------
// The strongest local maxima of the spectrum, sorted by height. Peaks within
// PEAK_GUARD_BINS of a stronger one are its own window leakage and dropped.
static int find_peaks(const float *mag, float floor_mag, float bin_hz,
    FftPeak *out, int max_peaks)
{
  struct { int bin; float mag; } cand[PEAK_CANDIDATES];
  int ncand = 0;
  int count = 0;

  for (int i = 2; i < FFT_BINS - 1; i++)
  {
    if (mag[i] <= mag[i - 1] || mag[i] < mag[i + 1])
      continue;

    if (mag[i] < floor_mag * PEAK_MIN_SNR)
      continue;

    // Insertion sort into a fixed-size "strongest so far" list
    int pos = ncand;

    while (pos > 0 && cand[pos - 1].mag < mag[i])
    {
      if (pos < PEAK_CANDIDATES)
        cand[pos] = cand[pos - 1];

      pos--;
    }

    if (pos < PEAK_CANDIDATES)
    {
      cand[pos].bin = i;
      cand[pos].mag = mag[i];

      if (ncand < PEAK_CANDIDATES)
        ncand++;
    }
  }

  for (int i = 0; i < ncand && count < max_peaks; i++)
  {
    bool shadowed = false;

    for (int j = 0; j < count; j++)
    {
      int d = cand[i].bin - out[j].bin;

      if (d < 0)
        d = -d;

      if (d <= PEAK_GUARD_BINS)
        shadowed = true;
    }

    if (shadowed)
      continue;

    float height;
    float delta = interpolate(mag, cand[i].bin, &height);

    out[count].bin      = cand[i].bin;
    out[count].freq     = ((float)cand[i].bin + delta) * bin_hz;
    out[count].mag      = height;
    out[count].rel_db   = 0.0f;
    out[count].harmonic = 0;
    count++;
  }

  return count;
}

//-----------------------------------------------------------------------------
// How far off an integer multiple of `f` a peak at `freq` may sit and still
// count as its harmonic. A bin and a half is the resolution limit, but the
// tolerance must also stay small compared to the comb spacing itself: at
// 10 Hz bins, allowing +-15 Hz around multiples of 50 Hz lets a bogus 83 Hz
// comb "explain" the 100 and 250 Hz peaks just as well as the real one.
// The term proportional to freq absorbs the error of a slightly-off
// estimate growing with the harmonic number.
static float comb_tolerance(float f, float freq, float bin_hz)
{
  float tol = 1.5f * bin_hz;

  if (tol > 0.15f * f)
    tol = 0.15f * f;

  tol += 0.01f * freq;

  return (tol > 0.30f * f) ? 0.30f * f : tol;
}

//-----------------------------------------------------------------------------
// The fundamental is the comb that explains the most spectral energy. Every
// peak and every subharmonic of it is a candidate; on a tie the higher
// frequency wins, or f0/2 (which explains the same peaks) would always be
// just as good an answer.
static float find_fundamental(const FftPeak *peak, int count, float bin_hz,
    int *matched_out)
{
  float best_f = 0.0f;
  float best_score = 0.0f;
  int best_matched = 0;

  for (int i = 0; i < count; i++)
  {
    for (int k = 1; k <= COMB_MAX_DIVISOR; k++)
    {
      float f = peak[i].freq / (float)k;
      float score = 0.0f;
      int matched = 0;

      if (f < COMB_MIN_BINS * bin_hz)
        break;

      for (int j = 0; j < count; j++)
      {
        int n = (int)(peak[j].freq / f + 0.5f);

        if (n < 1 || n > FFT_MAX_HARM)
          continue;

        if (fabsf(peak[j].freq - (float)n * f) <= comb_tolerance(f, peak[j].freq, bin_hz))
        {
          score += peak[j].mag;
          matched++;
        }
      }

      if (matched < 2)
        continue;

      if (score > best_score * 1.02f ||
          (score > best_score * 0.98f && f > best_f))
      {
        best_score = score;
        best_f = f;
        best_matched = matched;
      }
    }
  }

  *matched_out = best_matched;

  return best_f;
}

//-----------------------------------------------------------------------------
void fft_analyze(const float *mag, int sample_period_ns, FftAnalysis *out)
{
  float bin_hz = (float)(1e9 / ((double)sample_period_ns * FFT_SIZE));
  float sum = 0.0f;
  float floor_mag;
  float h1 = 0.0f, harm_sq = 0.0f;

  out->count       = 0;
  out->fundamental = 0.0f;
  out->harmonics   = 0;
  out->thd_x10     = -1;
  out->bin_hz      = bin_hz;
  out->nyquist_hz  = bin_hz * FFT_BINS;
  out->noise_db    = 0.0f;

  for (int i = 2; i < FFT_BINS; i++)
    sum += mag[i];

  floor_mag = sum / (FFT_BINS - 2);

  if (floor_mag <= 0.0f)
    return;

  out->count = find_peaks(mag, floor_mag, bin_hz, out->peak, FFT_MAX_PEAKS);

  if (out->count == 0)
    return;

  for (int i = 0; i < out->count; i++)
    out->peak[i].rel_db = 20.0f * log10f(out->peak[i].mag / out->peak[0].mag);

  out->noise_db = 20.0f * log10f(floor_mag / out->peak[0].mag);

  out->fundamental = find_fundamental(out->peak, out->count, bin_hz,
      &out->harmonics);

  if (out->fundamental <= 0.0f)
  {
    // No comb: a single tone, or peaks with nothing in common. The
    // strongest peak is then the best answer, and the distortion figure
    // below still says whether anything sits on its multiples.
    out->fundamental = out->peak[0].freq;
    out->harmonics = 1;
  }

  for (int i = 0; i < out->count; i++)
  {
    int n = (int)(out->peak[i].freq / out->fundamental + 0.5f);
    float tol = comb_tolerance(out->fundamental, out->peak[i].freq, bin_hz);

    if (n >= 1 && n <= FFT_MAX_HARM &&
        fabsf(out->peak[i].freq - (float)n * out->fundamental) <= tol)
      out->peak[i].harmonic = n;
  }

  // THD from the comb itself, not from the detected peaks: a harmonic that
  // sits just under the peak-detection floor still contributes distortion
  for (int n = 1; n <= FFT_MAX_HARM; n++)
  {
    int center = (int)((float)n * out->fundamental / bin_hz + 0.5f);
    float m = 0.0f;

    if (center < 1 || center >= FFT_BINS - 1)
      break;

    for (int b = center - 2; b <= center + 2; b++)
    {
      if (b > 0 && b < FFT_BINS && mag[b] > m)
        m = mag[b];
    }

    if (n == 1)
      h1 = m;
    else
      harm_sq += m * m;
  }

  if (h1 > 0.0f)
  {
    float thd = sqrtf(harm_sq) / h1 * 1000.0f;

    out->thd_x10 = (thd > 9999.0f) ? 9999 : (int)(thd + 0.5f);
  }
}
