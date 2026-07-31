/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
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
  int  mean3_c100; // mean of (sample - zero_point)^3, counts^3 * 100. Only the
                   // nonlinearity correction reads it (capture.c): shifting an
                   // RMS off a bent code axis needs the third moment, and this
                   // is the one place that has seen every sample.
  int  frequency;  // Hz; 0 when no periodic signal was found
  int  duty_x10;   // duty cycle in 0.1% units (0..1000); -1 when n/a
  int  width_pos_ns;    // mean time per period spent above the mid level and
  int  width_neg_ns;    // below it; -1 when n/a. Derived from the duty cycle
                        // and period_med_ns below rather than measured on
                        // their own, so the three can never disagree:
                        // width_pos + width_neg == period_med exactly, and
                        // width_pos / period_med IS the duty reading.
  int  periods;    // number of full periods seen
  int  slope_pos;  // samples where the signal was rising (beyond noise)
  int  slope_neg;  // samples where the signal was falling (beyond noise)
  int  period_med_ns;   // period in ns that `frequency` is 1/x of: the mean
                        // of the periods within +-1/2 of the median. The
                        // median alone reads high when the sample grid does
                        // not divide the period evenly (+9% at 2.31
                        // samples/period); its job here is to say which
                        // periods are real, not what they measure.
  int  period_min_ns;   // shortest and longest period seen - the one runt or
  int  period_max_ns;   // stretched cycle the median deliberately hides
  int  jitter_rms_ps;   // stddev of all periods, ps (sub-sample crossing
                        // interpolation: ~31 ps resolution at 8 ns/sample);
                        // -1 when no periods were found
  int  jitter_pp_ps;    // max-min period spread, ps; -1 when n/a
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
//
// It is a TIME-domain counter, and it stops being one below about 3 samples
// per period: at 2.5 exactly the Schmitt arming misses every second crossing
// and a 50.000 MHz input on a 125 MS/s record reads exactly 25.000 MHz, with
// nothing in the Measure to say so. Above that rate the spectrum is the only
// honest source - see fft_peak_frequency().
void measure_run(const uint8_t *data, int size, int offset,
    int period_ns, int zero_point, Measure *m);

// Integer square root, no libm and no float. Exported because an RMS whose
// reference moves has to be re-derived from the one measure_run() returned
// (see get_measurements_ex) rather than rescanned.
uint32_t isqrt64(uint64_t value);

// Time index (samples from the oldest) of the center of the narrowest
// complete pulse in the record, with its width in samples; -1 when the
// record has no two transitions. Thresholds are the caller's Schmitt pair
// (typically mid +- hysteresis from a Measure). count_out (optional) gets
// the number of pulses at that same width and polarity: 1 = a genuine
// outlier, thousands = the signal's own half-periods.
int measure_find_min_pulse(const uint8_t *data, int size, int offset,
    int level_lo, int level_hi, int *width_out, bool *high_out, int *count_out);

#endif // _MEASURE_H_
