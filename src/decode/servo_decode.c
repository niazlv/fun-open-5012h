/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Servo / RC PWM decoder: a pulse train at a fixed frame rate whose WIDTH
 * carries the value. 1.0 to 2.0 ms inside a 20 ms frame is the classic hobby
 * servo; ESCs, flight controllers and modern receivers send the same shape
 * faster, up to a few hundred hertz.
 *
 * What tells it apart from everything else on this bus is not the pulse - a
 * 1.5 ms pulse is just a pulse, and the UART decoder was happily framing
 * bytes out of one - but the CADENCE. A servo line repeats at a fixed rate
 * whether the value changes or not, and nothing else here does: UART traffic
 * has data-dependent gaps, NEC alternates 1.12 and 2.25 ms according to the
 * bits it is sending, and a bus that is idle sends nothing at all. So the
 * period between pulses has to hold steady, and that is the whole test.
 *
 * Pure C, host-testable.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "logic_decode.h"

/*- Definitions -------------------------------------------------------------*/
// Standard travel is 1000-2000 us; 500-2500 is the extended range every
// hobby receiver and ESC understands, and the margin catches a pulse that
// has been trimmed past it rather than calling the line something else
#define SERVO_MIN_US        400
#define SERVO_MAX_US       2600

// 40 Hz is slower than any receiver runs, 500 Hz faster than any servo takes
#define SERVO_FRAME_MIN_US  2000
#define SERVO_FRAME_MAX_US 25000

// One pulse is a pulse. Two are a rate, and the rate is the evidence.
#define SERVO_MIN_PULSES      2
#define SERVO_FRAME_TOL(x)  ((x) / 4)   // +-25% between frames

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Collect the pulses at one polarity. Returns the number found, or -1 when a
// run at the pulse level is NOT a valid pulse: a servo line carries servo
// pulses and nothing else, so one oversized mark means this is some other
// signal that happens to contain pulses of about the right width.
//
// The first and last runs are skipped whatever they are - both are cut by the
// ends of the record, so their width is a fact about the buffer and not about
// the signal.
static int servo_pulses(const LogicScratch *scratch, int runs, int period_ns,
    int level, int *first_run, int *count_out)
{
  int n = 0;

  *first_run = -1;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (scratch->lvl[r] != level)
      continue;

    int64_t us = (int64_t)scratch->len[r] * period_ns / 1000;

    if (us < SERVO_MIN_US || us > SERVO_MAX_US)
      return -1;

    if (*first_run < 0)
      *first_run = r;

    n++;
  }

  *count_out = n;

  return n;
}

//-----------------------------------------------------------------------------
int servo_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  const int *run_len = scratch->len;
  const int *run_pos = scratch->pos;
  const uint8_t *run_lvl = scratch->lvl;
  int mid, runs;

  (void)data;
  (void)size;
  (void)offset;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_SERVO;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 4 || period_ns <= 0)
    return 0;

  // Which level is the pulse? Servo idles low and pulses high, but an
  // inverted receiver output exists and costs one more pass to allow for.
  //
  // Which one to try first is not a matter of taste. A digital servo runs at
  // 333 Hz, where a 1.5 ms pulse sits in a 3 ms frame - and then the gaps are
  // 1.5 ms too, so BOTH levels are a valid pulse train and whichever is tried
  // first wins. What breaks the tie is that a pulse is the shorter part of a
  // frame: the level the line spends less time at is the pulse. Where even
  // that is a draw, at exactly half, the convention decides.
  int64_t total[2] = { 0, 0 };
  int64_t runs_at[2] = { 0, 0 };

  for (int r = 1; r + 1 < runs; r++)
  {
    total[scratch->lvl[r] & 1] += scratch->len[r];
    runs_at[scratch->lvl[r] & 1]++;
  }

  int level = -1, first = -1, count = 0;
  // Per run, not in total: a record that ends after the last pulse has one
  // more run at that level than at the other, and totalled that alone would
  // decide it
  int prefer = (0 == runs_at[0] || total[1] * runs_at[0] <=
      total[0] * runs_at[1]) ? 1 : 0;

  for (int p = 0; p < 2; p++)
  {
    int try_level = p ? !prefer : prefer;
    int f, c;

    if (servo_pulses(scratch, runs, period_ns, try_level, &f, &c) >=
        SERVO_MIN_PULSES)
    {
      level = try_level;
      first = f;
      count = c;
      break;
    }
  }

  if (level < 0)
    return 0;

  out->idle_high = (level == 0); // the line rests at the other level

  // The frame rate, and the demand that it holds. Measured pulse start to
  // pulse start, which is how a receiver measures it too.
  int64_t frame_us = 0;
  int prev = -1;

  for (int r = first; r + 1 < runs && out->count < LOGIC_MAX_BYTES; r++)
  {
    if (run_lvl[r] != level)
      continue;

    if (prev >= 0)
    {
      int64_t us = (int64_t)(run_pos[r] - run_pos[prev]) * period_ns / 1000;

      if (us < SERVO_FRAME_MIN_US || us > SERVO_FRAME_MAX_US)
        return 0; // not a frame rate a servo line runs at

      if (frame_us == 0)
        frame_us = us;
      else if (us > frame_us + SERVO_FRAME_TOL(frame_us) ||
               us < frame_us - SERVO_FRAME_TOL(frame_us))
        return 0; // the cadence wandered: whatever this is, it is not clocked

      // The frame the PREVIOUS pulse started now has a known length, so it
      // can be marked out on the trace from one pulse to the next
      out->end[out->count - 1] = run_pos[r];
    }

    int64_t width_us = (int64_t)run_len[r] * period_ns / 1000;

    // Tens of microseconds: it holds the whole 400-2600 us range in the byte
    // the result is built out of, and 10 us is a hundredth of the travel a
    // servo has - finer than the mechanism resolves and finer than the record
    // measures at any timebase that shows a whole frame
    out->bytes[out->count] = (uint8_t)(width_us / 10);
    out->pos[out->count] = run_pos[r];
    out->end[out->count] = run_pos[r] + run_len[r];
    out->count++;

    prev = r;
  }

  if (out->count < SERVO_MIN_PULSES || frame_us <= 0)
    return 0;

  out->truncated = (count > out->count);

  // The last pulse has no successor to measure its frame against, so it gets
  // the rate the others agreed on
  out->end[out->count - 1] = out->pos[out->count - 1] +
      (int)(frame_us * 1000 / period_ns);

  // The line was still pulsing when the record ended - there is always more
  // of a servo signal than fits, it never stops - so the panel says so only
  // when the result itself was cut short
  out->overrun = out->truncated;

  out->rate = (int)(frame_us * 1000); // frame period in ns, as the others use

  int hz = (int)((1000000 + frame_us / 2) / frame_us);
  int us = (int)out->bytes[0] * 10;

  // The angle is what anyone is actually reading off a servo line, and it is
  // arithmetic on the width rather than a second measurement: 1000 us is
  // -90 degrees, 2000 is +90, and the middle is where the horn points
  // straight ahead. That mapping is a convention and not a law - the width is
  // what was measured and stays first - so a pulse outside the 1000..2000 it
  // is defined over is marked rather than quietly extrapolated. A receiver
  // may well send 2.2 ms and an ESC reads the same line as throttle; what the
  // '!' says is that the number beside it is off the end of the convention.
  int deg = (us - 1500) * 9 / 50;
  bool travel = (us >= 1000 && us <= 2000);

  snprintf(out->info, sizeof(out->info), "Servo %d.%02dms %+dd%s %dHz%s",
      us / 1000, (us % 1000) / 10, deg, travel ? "" : "!", hz,
      out->idle_high ? " inv" : "");

  return out->count;
}
