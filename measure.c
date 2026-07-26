/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Waveform measurements over a ring buffer of 8-bit samples.
 *
 * Robustness against real-world probing artifacts is deliberate design:
 *  - levels (top/base) come from amplitude-histogram percentiles, so
 *    overshoot, ringing and glitch spikes do not inflate the amplitude;
 *  - the frequency is the MEDIAN of all measured periods, so a handful of
 *    glitch-induced crossings cannot poison it;
 *  - periodicity is judged by the fraction of periods near that median.
 *
 * Pure C, no hardware dependencies: this module is also compiled and tested
 * on the host against synthetic waveforms (see tests/host_test.c).
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "measure.h"

/*- Definitions -------------------------------------------------------------*/
#define MIN_AMPLITUDE     6   // top-base counts below which the record is DC
#define HYSTERESIS_PCT    20  // hysteresis as % of top-base amplitude
#define MIN_HYSTERESIS    2   // counts
#define PCTL_LOW          2   // percentile for the base level
#define PCTL_HIGH         98  // percentile for the top level
#define PEAK_MIN_SAMPLES  8   // levels held for fewer samples are glitches
#define LEVEL_BAND_DIV    8   // "at a level" = within span/8 of pk_lo or pk_hi
#define MAX_PERIODS       128 // periods kept for the median estimate

/*- Variables ---------------------------------------------------------------*/
static uint32_t g_hist[256];
static int g_periods[MAX_PERIODS];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint32_t isqrt64(uint64_t value)
{
  uint64_t result = 0;
  uint64_t bit = 1ull << 62;

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

  return (uint32_t)result;
}

//-----------------------------------------------------------------------------
static int percentile_level(int size, int pctl)
{
  uint32_t target = (uint32_t)((uint64_t)size * pctl / 100);
  uint32_t cum = 0;

  for (int v = 0; v < 256; v++)
  {
    cum += g_hist[v];

    if (cum > target)
      return v;
  }

  return 255;
}

//-----------------------------------------------------------------------------
static int median_period(int count)
{
  // Insertion sort: count <= MAX_PERIODS and this runs at most 10x a second
  for (int i = 1; i < count; i++)
  {
    int key = g_periods[i];
    int j = i - 1;

    while (j >= 0 && g_periods[j] > key)
    {
      g_periods[j + 1] = g_periods[j];
      j--;
    }

    g_periods[j + 1] = key;
  }

  return g_periods[count / 2];
}

//-----------------------------------------------------------------------------
void measure_run(const uint8_t *data, int size, int offset,
    int period_ns, int zero_point, Measure *m)
{
  int vmin = 255, vmax = 0;
  int64_t sum = 0;
  uint64_t sum_sq = 0;
  int index = offset;
  int slope_pos = 0, slope_neg = 0;
  int slope_prev = data[offset];

  m->frequency = 0;
  m->duty_x10  = -1;
  m->periods   = 0;
  m->period_good_pct = 0;
  m->level_pct = 0;

  for (int v = 0; v < 256; v++)
    g_hist[v] = 0;

  // Pass 1: histogram / extremes / mean / RMS / slope balance
  for (int i = 0; i < size; i++)
  {
    int v = data[index];
    int d = v - zero_point;
    int diff = v - slope_prev;

    g_hist[v]++;

    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;

    sum += d;
    sum_sq += (uint32_t)(d * d);

    if (diff > 1)
      slope_pos++;
    else if (diff < -1)
      slope_neg++;

    slope_prev = v;

    if (++index == size)
      index = 0;
  }

  m->vmin = vmin;
  m->vmax = vmax;
  m->base = percentile_level(size, PCTL_LOW);
  m->top  = percentile_level(size, PCTL_HIGH);

  // Spike-trimmed extremes by absolute sample count: a percentile erases
  // anything under 2% of the record, which blinds the auto-setup to short
  // bursts; a fixed 8-sample floor only erases true glitches
  {
    uint32_t cum = 0;
    int v;

    for (v = 255; v > 0; v--)
    {
      cum += g_hist[v];

      if (cum >= PEAK_MIN_SAMPLES)
        break;
    }

    m->pk_hi = v;
    cum = 0;

    for (v = 0; v < 255; v++)
    {
      cum += g_hist[v];

      if (cum >= PEAK_MIN_SAMPLES)
        break;
    }

    m->pk_lo = v;

    if (m->pk_hi < m->pk_lo)
    {
      m->pk_hi = vmax;
      m->pk_lo = vmin;
    }
  }

  // How digital the record looks: the share of samples resting AT the two
  // levels instead of travelling between them. A logic line spends nearly
  // all its time at a rail, noise spends nearly none — and that is the only
  // thing that separates real but irregular traffic (UART frames, bus
  // activity) from noise, since both have equally irregular "periods".
  {
    int span = m->pk_hi - m->pk_lo;
    int band = span / LEVEL_BAND_DIV;
    uint32_t at_level = 0;

    for (int v = m->pk_lo; v <= m->pk_lo + band; v++)
      at_level += g_hist[v];

    for (int v = m->pk_hi - band; v <= m->pk_hi; v++)
      at_level += g_hist[v];

    m->level_pct = (int)(at_level * 100 / (uint32_t)size);
  }
  m->mean_c100 = (int)(sum * 100 / size);
  m->rms_c100 = (int)isqrt64(sum_sq * 10000 / (uint32_t)size);
  m->slope_pos = slope_pos;
  m->slope_neg = slope_neg;

  if (m->top < m->base)
  {
    m->top = vmax;
    m->base = vmin;
  }

  // Amplitude gate and crossing thresholds use the spike-trimmed PEAKS, not
  // the percentiles: a short burst sits entirely inside the percentile trim
  // and would read as "no signal" with a mid level stuck at the baseline
  if ((m->pk_hi - m->pk_lo) < MIN_AMPLITUDE)
    return;

  // Pass 2: rising mid-level crossings with adaptive hysteresis; the
  // Schmitt arming rejects noise around the mid level
  int mid = (m->pk_lo + m->pk_hi) / 2;
  int hyst = (m->pk_hi - m->pk_lo) * HYSTERESIS_PCT / 100;

  if (hyst < MIN_HYSTERESIS)
    hyst = MIN_HYSTERESIS;

  int level_lo = mid - hyst;
  int level_hi = mid + hyst;

  int64_t first_x256 = 0, prev_cross_x256 = 0;
  int64_t last_x256 = 0;
  int nperiods = 0;
  int crossings = 0;
  int high_samples = 0;
  int span_start = -1, span_end = -1;
  bool armed = false;
  int prev = data[offset];

  index = offset;

  for (int i = 0; i < size; i++)
  {
    int v = data[index];

    if (v <= level_lo)
    {
      armed = true;
    }
    else if (armed && v > level_hi && prev <= level_hi)
    {
      // Sub-sample interpolation of where the signal crossed `mid`
      // between sample i-1 (prev) and sample i (v), scaled by 256 and
      // clamped to that interval
      int64_t cross = (int64_t)i * 256;

      if (v != prev)
      {
        int64_t frac = (int64_t)(v - mid) * 256 / (v - prev);

        if (frac < 0)
          frac = 0;
        else if (frac > 256)
          frac = 256;

        cross -= frac;
      }

      if (crossings == 0)
      {
        first_x256 = cross;
        span_start = i;
      }
      else
      {
        if (nperiods < MAX_PERIODS)
          g_periods[nperiods++] = (int)(cross - prev_cross_x256);

        last_x256 = cross;
        span_end = i;
      }

      prev_cross_x256 = cross;
      crossings++;
      armed = false;
    }

    if (crossings > 0 && v > mid)
      high_samples++;

    prev = v;

    if (++index == size)
      index = 0;
  }

  (void)first_x256;
  (void)last_x256;

  if (nperiods < 1)
    return;

  // Median period: immune to the few short/long intervals a glitch spike
  // or a burst of ringing injects
  int med = median_period(nperiods); // destroys g_periods ordering only
  int good = 0;

  if (med <= 0)
    return;

  for (int i = 0; i < nperiods; i++)
  {
    int p = g_periods[i];

    if (p >= med - med / 4 && p <= med + med / 4)
      good++;
  }

  m->periods = nperiods;
  m->period_good_pct = good * 100 / nperiods;
  m->frequency = (int)((1000000000ll * 256) / ((int64_t)med * period_ns));

  // Duty cycle over the whole-period span only. high_samples also counted
  // the tail after the last crossing; subtract it.
  int span = span_end - span_start;

  if (span > 0)
  {
    int tail_high = 0;

    index = offset + span_end;
    index %= size;

    for (int i = span_end; i < size; i++)
    {
      if (data[index] > mid)
        tail_high++;

      if (++index == size)
        index = 0;
    }

    int high_in_span = high_samples - tail_high;

    if (high_in_span < 0)
      high_in_span = 0;
    else if (high_in_span > span)
      high_in_span = span;

    m->duty_x10 = high_in_span * 1000 / span;
  }
}
