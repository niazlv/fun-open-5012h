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

#ifndef _CAPTURE_H_
#define _CAPTURE_H_

/*- Includes ----------------------------------------------------------------*/
#include "measure.h"

/*- Definitions -------------------------------------------------------------*/
#define BASE_SAMPLE_RATE       125e6
#define BASE_SAMPLE_PERIOD     (1e9 / BASE_SAMPLE_RATE)
// Capture ring lives at the bottom of main SRAM (0x20000000), followed by the
// 24 KB decimated storage buffer (see capture.c). Together they must fit the
// 128 KB SRAM, and capture size must stay a multiple of the 16 KB DMA buffer
// and keep a 4:1 ratio to the storage buffer (buffer_decimate asm is 4:1).
#define CAPTURE_BUFFER_SIZE    (96 * 1024)

// The 8 KB of main SRAM left after capture + storage. Nothing is linked into
// this region, so the startup code neither loads nor zeroes it: the low 5 KB
// is app scratch (currently the logic decoder's run tables) and the top 3 KB
// holds the coredump ring, which has to survive a reset to be of any use.
#define CAPTURE_SPARE_RAM       0x2001E000u
#define CAPTURE_SPARE_RAM_SIZE  (5 * 1024)

#define COREDUMP_RAM_BASE       (CAPTURE_SPARE_RAM + CAPTURE_SPARE_RAM_SIZE)
#define COREDUMP_RAM_SIZE       (3 * 1024)
#define TRIGGER_MARGIN_SAMPLES 1024
#define DATA_BUFFER_SIZE       300

// Slowest the acquisition can be clocked. TIMER0/7 take a 16-bit prescaler
// and capture_set_horizontal_parameters loads it with (1 << sr_divider) - 1,
// so 16 is where the register runs out: 524 us per sample, a 51 s record.
// Ask for more and the write wraps - the hardware then samples four or eight
// times faster than the rate every readout, decoder and spectrum reports,
// with nothing on screen to say so. The screen is allowed to want more time
// than that (the roll view spans minutes); the RECORD is not.
#define MAX_SR_DIVIDER         16

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int      size;
  int      min_value;
  int      max_value;
  int      vertical_position;
  int      frequency;
  int      min[DATA_BUFFER_SIZE];
  int      max[DATA_BUFFER_SIZE];
  uint8_t  flags[DATA_BUFFER_SIZE];
} DataBuffer;

typedef struct
{
  int      vmin_raw;   // raw ADC counts, 0..255 (absolute extremes)
  int      vmax_raw;
  int      base_raw;   // robust levels (histogram percentiles): spikes and
  int      top_raw;    // noise peaks excluded — use these for auto-setup
  int      vpp_mv;
  int      vamp_mv;    // top_raw - base_raw in mV: the swing to fit on screen
  int      vrms_mv;    // from GROUND, like the peaks below: true RMS of the
  int      vavg_mv;    // input, DC included, and the DC level itself
  int      vmid_mv;    // (vmax + vmin) / 2 in mV, from the middle of the
                       // SCREEN: it is the 50% trigger level, and a trigger
                       // level is measured from there. The one field left on
                       // those terms, and the only one with no reading of its
                       // own on screen.
  // The absolute extremes and the larger of the two in magnitude - peak
  // voltage as an instrument means it, measured from GROUND rather than from
  // the middle of the screen. Everything else here is either a difference
  // (vpp, vamp - the reference cancels) or ground-referenced like these; the
  // rule is that any field a user reads as a voltage means volts on the probe
  // tip, whatever the vertical position was doing at the time.
  int      vmax_mv;
  int      vmin_mv;
  int      vp_mv;      // max(|vmax_mv|, |vmin_mv|), always >= 0
  // The two FLAT levels, from the percentiles of the amplitude histogram
  // (measure.h), so overshoot and ringing are outside them - their difference
  // is the amplitude of a square wave as a datasheet means it. Not the same
  // thing as vamp_mv above, which trims spikes rather than distribution tails
  // and is what the auto-setup steers by: a burst shorter than 2% of the
  // record has peaks but no percentiles.
  int      vtop_mv;
  int      vbase_mv;
  int      frequency;  // Hz, 0 when not measurable
  int      duty_x10;   // 0.1% units, -1 when n/a
  int      width_pos_ns; // time per period above / below the mid level, ns;
  int      width_neg_ns; // -1 when n/a. See measure.h - they are cut from the
                         // duty cycle, so they cannot disagree with it.
  // How much the frequency above can be trusted: noise crosses the mid
  // level constantly and would otherwise read as a signal
  int      period_med_ns;   // median period and its extremes over the record
  int      period_min_ns;
  int      period_max_ns;
  int      jitter_rms_ps;   // period stddev / spread, ps; -1 when n/a
  int      jitter_pp_ps;
  int      periods;         // whole periods the record held
  int      period_good_pct; // % of them within +-25% of the median
  int      level_pct;       // % of samples resting at one of the two levels:
                            // high = digital, even when the repetition is
                            // irregular (bus traffic); a few % = noise
} ScopeMeasure;

/*- Prototypes --------------------------------------------------------------*/
void capture_init(void);
void capture_disable_clock(void);
void capture_start(void);
// Machine stop (decode hunt, glitch finder): publishes the freshest, longest
// record - the raw ring - whenever the sweep has lapped it
void capture_stop(void);
// User stop (the RUN/STOP button): freezes what the display shows. In NORMAL
// and SINGLE that keeps the storage snapshot of the last triggered frame
// instead of letting the live ring replace it; AUTO still takes the ring.
void capture_stop_view(void);
// The frozen record is only the 24K storage snapshot (zoom/pan is limited to
// what it holds) - the trace view shows a tag while this is true
bool capture_stopped_on_snapshot(void);
void capture_set_vertical_parameters(void);
void capture_set_horizontal_parameters(int sr_divider, int trigger_offset);
void capture_set_trigger_level(int level);
void capture_set_trigger_edge(int edge);
void capture_set_trigger_mode(int mode);
int capture_get_state(void);
bool capture_buffer_updated(void);
void capture_consume_frame(void);
void capture_get_data(DataBuffer *db);
void capture_get_raw_data(int *raw, int size);
bool capture_get_measurements(ScopeMeasure *out);
// Same, but bypasses the 10 Hz recompute throttle: control loops
// (auto-setup, 50% trigger) must see the CURRENT acquisition, not a reading
// cached from before they changed the hardware settings
bool capture_get_measurements_fresh(ScopeMeasure *out);
bool capture_get_raw_measure(Measure *out);
// Same, unthrottled: control loops must see the CURRENT acquisition
bool capture_get_raw_measure_fresh(Measure *out);
// Mean level of each converter on its own, from the raw ring and without the
// channel delta applied. Dual-channel acquisitions only; false otherwise.
// Deliberately RAW in the other sense too: the nonlinearity sweep measures its
// own curve with these, so nothing that corrects for that curve may be in them.
bool capture_get_channel_means(int *a_x100, int *b_x100);
// The offset DAC directly, 0..4095, without going through a vertical position.
// For the linearity calibration only - anything else must use
// capture_set_vertical_parameters(), which keeps config and the hardware
// agreeing about where the trace is.
void capture_set_dac_raw(int code);
// The mean of a raw scan with the measured nonlinearity taken back out, in
// hundredths of an ADC code. What the gain calibration must aim at - see
// config.calib_nl2 and the note in capture.c.
int capture_measure_mean_x100(const Measure *m);
uint32_t capture_get_generation(void);
// Sample period (ns) of the record everything downstream reads. Equal to the
// ring's period while the full-rate window applies, four times it when the
// screen needs more time than that window spans.
int capture_get_record_period(void);
int capture_get_pending_record_period(void);
int capture_read_samples(uint8_t *dst, int max_count, int *period_ns, bool consume);
// The newest samples from the RAW full-rate capture ring (not the decimated
// storage buffer), usable while acquisition runs. May tear mid-write — meant
// for spectral estimation, where a seam only raises the noise floor.
int capture_read_fast_samples(uint8_t *dst, int max_count, int *period_ns);
// Peak pair over every ring sample written since the previous call, in raw
// ADC counts; nothing is copied and no record is waited for. Returns the
// number of samples folded, 0 when the ring has not moved (or is stopped),
// in which case vmin/vmax are untouched. This is how the roll view reads the
// input — see the comment on the definition for where samples are lost.
int capture_fold_samples(int *vmin, int *vmax);
// Progress toward the first record captured under the CURRENT acquisition
// timing, in ms of sweep; done == total once one has landed. A sweep is
// 98304 samples however long that takes - 51 s at the slow end - and until
// it completes every consumer is reading the record from before the timebase
// last moved.
void capture_record_fill(int *done_ms, int *total_ms);
bool capture_get_record(const uint8_t **data, int *size, int *offset,
    int *period_ns, int *trigger_timepos);

#endif // _CAPTURE_H_


