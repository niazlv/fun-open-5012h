/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Trend logger (see trend.h). The ring is append-only until it fills; then
 * it averages neighbouring pairs in place, halves its count and doubles the
 * time span of every point. 300 points at 1 s each start as a 5-minute
 * window and grow to hours without ever dropping the beginning of the run.
 */

/*- Includes ----------------------------------------------------------------*/
#include <string.h>
#include "trend.h"

/*- Variables ---------------------------------------------------------------*/
static TrendPoint g_points[TREND_POINTS];
static int g_count = 0;
static int g_span  = 1;  // pushes per stored point

// The bucket accumulating pushes toward the next point. Duty tracks its own
// valid-sample count: -1 samples (no duty measurable) must not drag the
// average of the valid ones.
static int     g_bucket_n = 0;
static int64_t g_sum_freq = 0;
static int64_t g_sum_vrms = 0;
static int64_t g_sum_duty = 0;
static int     g_duty_n   = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static int16_t clamp16(int64_t v)
{
  if (v > 32767)
    return 32767;

  if (v < -32768)
    return -32768;

  return (int16_t)v;
}

//-----------------------------------------------------------------------------
void trend_reset(void)
{
  g_count = 0;
  g_span = 1;
  g_bucket_n = 0;
  g_sum_freq = 0;
  g_sum_vrms = 0;
  g_sum_duty = 0;
  g_duty_n = 0;
}

//-----------------------------------------------------------------------------
void trend_push(int freq_hz, int vrms_mv, int duty_x10)
{
  TrendPoint *p;

  g_sum_freq += freq_hz;
  g_sum_vrms += vrms_mv;

  if (duty_x10 >= 0)
  {
    g_sum_duty += duty_x10;
    g_duty_n++;
  }

  if (++g_bucket_n < g_span)
    return;

  p = &g_points[g_count++];
  p->freq = (int32_t)(g_sum_freq / g_bucket_n);
  p->vrms = clamp16(g_sum_vrms / g_bucket_n);
  p->duty = g_duty_n ? (int16_t)(g_sum_duty / g_duty_n) : -1;

  g_bucket_n = 0;
  g_sum_freq = 0;
  g_sum_vrms = 0;
  g_sum_duty = 0;
  g_duty_n = 0;

  if (g_count == TREND_POINTS)
  {
    // 2:1 in place. A pair with one measurable duty keeps that one; only two
    // absent duties stay absent.
    for (int i = 0; i < TREND_POINTS / 2; i++)
    {
      const TrendPoint *a = &g_points[2 * i];
      const TrendPoint *b = &g_points[2 * i + 1];
      TrendPoint r;

      r.freq = (int32_t)(((int64_t)a->freq + b->freq) / 2);
      r.vrms = (int16_t)((a->vrms + b->vrms) / 2);

      if (a->duty >= 0 && b->duty >= 0)
        r.duty = (int16_t)((a->duty + b->duty) / 2);
      else if (a->duty >= 0)
        r.duty = a->duty;
      else
        r.duty = b->duty;

      g_points[i] = r;
    }

    g_count = TREND_POINTS / 2;
    g_span *= 2;
  }
}

//-----------------------------------------------------------------------------
int trend_count(void)
{
  return g_count;
}

//-----------------------------------------------------------------------------
int trend_seconds_per_point(void)
{
  return g_span;
}

//-----------------------------------------------------------------------------
const TrendPoint *trend_data(void)
{
  return g_points;
}
