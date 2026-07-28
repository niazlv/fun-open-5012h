/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Trend logger: a fixed-size ring of periodic measurement samples that
 * compresses itself 2:1 when full, so it never stops recording and its time
 * span only grows. Pure C, no hardware dependencies - compiled and tested on
 * the host (see tests/host_test.c).
 */

#ifndef _TREND_H_
#define _TREND_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>

/*- Definitions -------------------------------------------------------------*/
// One point per trace column: the plot never needs more
#define TREND_POINTS 300

/*- Types -------------------------------------------------------------------*/
// One logged point. After compressions a point is the average of its whole
// time span, not an instant.
typedef struct
{
  int32_t freq;   // Hz; 0 = no periodic signal during the span
  int16_t vrms;   // mV, clamped to int16
  int16_t duty;   // 0.1 % units; -1 = not measurable
} TrendPoint;

/*- Prototypes --------------------------------------------------------------*/
void trend_reset(void);

// Log one sample; call at a fixed cadence (the scope uses 1 s). Samples are
// bucketed to the current span, so after a 2:1 compression it takes span
// pushes to append one point - the ring's real-time rate never changes.
void trend_push(int freq_hz, int vrms_mv, int duty_x10);

int trend_count(void);

// Push cadences each stored point covers (1, 2, 4, ... after compressions)
int trend_seconds_per_point(void);

// Stored points, oldest first, trend_count() of them
const TrendPoint *trend_data(void);

#endif // _TREND_H_
