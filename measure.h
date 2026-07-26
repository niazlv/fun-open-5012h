/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Waveform measurements over a ring buffer of 8-bit samples.
 *
 * Pure C, no hardware dependencies: this module is also compiled and tested
 * on the host against synthetic waveforms (see tests/host_test.c).
 */

#ifndef _MEASURE_H_
#define _MEASURE_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int  vmin;       // raw counts, 0..255 (absolute extremes, spikes included)
  int  vmax;
  int  base;       // robust low/high levels: 2nd/98th percentile of the
  int  top;        // amplitude histogram, immune to spikes and overshoot
  int  pk_lo;      // spike-trimmed extremes: ignore levels held for < 8
  int  pk_hi;      // samples total — still sees short bursts a percentile
                   // would erase (a us-scale packet in a ms-scale record)
  int  mean_c100;  // mean of (sample - zero_point), in counts * 100
  int  rms_c100;   // RMS of (sample - zero_point), in counts * 100
  int  frequency;  // Hz; 0 when no periodic signal was found
  int  duty_x10;   // duty cycle in 0.1% units (0..1000); -1 when n/a
  int  periods;    // number of full periods seen
  int  slope_pos;  // samples where the signal was rising (beyond noise)
  int  slope_neg;  // samples where the signal was falling (beyond noise)
  int  period_good_pct; // % of periods within +-25% of the median period:
                        // ~100 for a periodic signal, low for noise; glitch
                        // spikes only dent it instead of poisoning min/max
  int  level_pct;       // % of samples sitting AT one of the two levels
                        // rather than travelling between them: square/logic
                        // ~90, sine ~45, triangle ~25, noise a few. Tells a
                        // real but IRREGULAR signal (bus traffic, UART
                        // frames) from noise, which period statistics alone
                        // cannot do — both look equally irregular.
} Measure;

/*- Prototypes --------------------------------------------------------------*/
// data is a ring buffer of `size` samples whose oldest sample sits at
// `offset`; period_ns is the sample period. Frequency detection uses
// mid-level crossings with hysteresis adaptive to the signal amplitude and
// sub-sample interpolation of the crossing instants, so it works regardless
// of where the trigger level sits and averages over every full period in
// the record.
void measure_run(const uint8_t *data, int size, int offset,
    int period_ns, int zero_point, Measure *m);

#endif // _MEASURE_H_
