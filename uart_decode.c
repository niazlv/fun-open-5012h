/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * UART auto-decoder: detects polarity and baud rate from the captured
 * waveform, then decodes 8N1 frames. Pure C, host-testable.
 *
 * The bit time is not measured, it is *chosen*: every standard baud that could
 * physically be in the record is decoded end to end, and the rate whose frames
 * account for the most of the record wins. Measuring it directly - shortest
 * run = one bit - is what the first version did, and it fails exactly when it
 * matters. A message longer than the record leaves a partial run at each end
 * of the buffer; a partial run is shorter than a bit; the estimate collapses,
 * every stop bit lands in the wrong place, and the record falls through to the
 * raw decoder at a rate that changes with every acquisition.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "logic_decode.h"

/*- Definitions -------------------------------------------------------------*/
#define MIN_RUNS        6     // fewer transitions than this = not UART
#define MIN_BIT_SAMPLES 3     // undersampled below this, decode is unreliable
#define MAX_FRAME_BITS  10    // start + 8 data + stop
#define IDLE_GAP_BITS   (MAX_FRAME_BITS + 1)  // an idle this long = line at rest

// The widest bit, x256, that a record whose shortest run is `tmin` can hold:
// a run is a whole number of bit times, so a bit wider than the shortest run
// (plus a quarter for sampling slack) is not in this record at any rate
#define BIT_MAX_X256(tmin)  ((int64_t)(tmin) * 256 * 5 / 4)

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))

/*- Variables ---------------------------------------------------------------*/
// Ascending, and the search keeps the first of an equal-scoring pair, so a
// tie resolves to the slowest of them - the faster one of a tied pair reads
// the same edges twice
static const int g_standard_bauds[] =
{
  1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400,
  460800, 921600, 1000000, 1500000, 2000000, 3000000,
};

// Every standard rate that fits the record, plus one slot for a link that is
// not on a standard rate at all
#define MAX_CANDIDATES  (ARRAY_SIZE(g_standard_bauds) + 1)

// 0 = work the rate out from the record; otherwise decode at this rate and
// no other. A record holding two or three frames does not always pin a rate
// down on its own, and being told beats guessing.
static int g_forced_baud = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline int sample_at(const uint8_t *data, int size, int offset, int i)
{
  int index = offset + i;

  if (index >= size)
    index -= size;

  return data[index];
}

//-----------------------------------------------------------------------------
// One decode pass at a given bit time and polarity. Fills out->bytes and
// returns a score in SAMPLES: how much of the record the frames account for,
// a decoded frame worth its own length and a framing error costing half as
// much again. Counting frames instead is not enough - an exact harmonic of
// the real rate (921600 read as 8x115200) decodes eight times as many frames,
// each an eighth as long, and would win on frame count alone. It cannot win
// on record coverage, because those frames only explain an eighth of the
// traffic and the rest of the record stays unaccounted for.
static int64_t uart_scan(const uint8_t *data, int size, int offset, int mid,
    int idle, int bit_x256, const LogicScratch *scratch, int runs,
    LogicResult *out)
{
  const int *run_len = scratch->len;
  const int *run_pos = scratch->pos;
  const uint8_t *run_lvl = scratch->lvl;
  int64_t gap_x256 = (int64_t)bit_x256 * IDLE_GAP_BITS;

  out->count = 0;
  out->errors = 0;
  out->burst_start = false;
  out->truncated = false;

  // Decode 8N1: idle -> start bit (non-idle) -> 8 data bits LSB-first ->
  // stop bit (idle). The run list locates frame starts; the bits themselves
  // are sampled directly at their centers (x256 fixed-point positions).
  for (int r = 0; r < runs - 1; r++)
  {
    // A frame starts where a non-idle run follows an idle run
    if (run_lvl[r] != idle)
      continue;

    if (out->count >= LOGIC_MAX_BYTES)
    {
      out->truncated = true;
      break;
    }

    int64_t frame = (int64_t)run_pos[r + 1] * 256;
    int value = 0;
    bool ok = true;

    // Data bit n center = frame + (1.5 + n) * bit
    for (int b = 0; b < 8; b++)
    {
      int64_t center = frame + ((int64_t)bit_x256 * (3 + 2 * b)) / 2;
      int si = (int)(center / 256);

      if (si >= size)
      {
        ok = false;
        break;
      }

      int bitv = (sample_at(data, size, offset, si) > mid) ? 1 : 0;

      if (!idle)
        bitv = !bitv; // inverted logic: mark = low

      if (bitv)
        value |= (1 << b);
    }

    if (!ok)
      break; // the record ends inside this frame: not an error, just the end

    // Stop bit must be at idle level
    int64_t stop_center = frame + ((int64_t)bit_x256 * 19) / 2;
    int stop_i = (int)(stop_center / 256);

    if (stop_i >= size)
      break;

    if (((sample_at(data, size, offset, stop_i) > mid) ? 1 : 0) == idle)
    {
      // Did the record catch the line at rest before this byte? Only asked of
      // the first one: that is what makes this record the head of a message
      // rather than a slice out of its middle.
      if (out->count == 0 && (int64_t)run_len[r] * 256 >= gap_x256)
        out->burst_start = true;

      out->bytes[out->count] = (uint8_t)value;
      out->pos[out->count] = run_pos[r + 1];
      out->end[out->count] =
          (int)((frame + (int64_t)bit_x256 * MAX_FRAME_BITS) / 256);
      out->count++;
    }
    else
    {
      // Not a frame after all. Try the next idle-to-start edge rather than
      // skipping a frame's worth of record: a record that opens in the middle
      // of a message starts out of phase, and the only way back into phase is
      // to keep trying edges until one holds up. Skipping ahead from a wrong
      // guess just makes the next guess wrong too.
      out->errors++;
      continue;
    }

    // Resume the scan at the first run starting inside or after the stop
    // bit, so this frame's data runs are not mistaken for new frames
    int frame_end = (int)((frame + (int64_t)bit_x256 * (MAX_FRAME_BITS - 1)) / 256);

    while (r + 1 < runs && run_pos[r + 1] < frame_end)
      r++;

    r--; // the for-loop increment lands on the idle run before the next frame
  }

  return ((int64_t)out->count * 4 - (int64_t)out->errors * 6) *
      bit_x256 * MAX_FRAME_BITS / 256;
}

//-----------------------------------------------------------------------------
void uart_decode_set_baud(int baud)
{
  g_forced_baud = (baud > 0) ? baud : 0;
}

//-----------------------------------------------------------------------------
// Samples per bit, x256 fixed point, for one baud at one sample period
static int64_t bit_width_x256(int baud, int period_ns)
{
  return (256000000000ll + (int64_t)baud * period_ns / 2) /
      ((int64_t)baud * period_ns);
}

//-----------------------------------------------------------------------------
// The shortest run in the record that is not a record-boundary artefact: one
// bit time, or a small multiple of it on traffic that never sends two unlike
// bits in a row. Returns 0 when the record has nothing to measure.
//
// The first and last runs are excluded because both are cut off by the ends
// of the record: each is some fraction of the run it would have been, and
// says nothing about the bit time. Counting them is what made a message
// longer than the record decode at a different rate on every acquisition.
static int uart_min_run(const LogicScratch *scratch, int runs)
{
  const int *run_len = scratch->len;
  const int first = 1, last = runs - 2;
  int tmin;

  if (last < first)
    return 0; // nothing but boundary runs: no bit time to be had

  tmin = run_len[first];

  for (int r = first; r <= last; r++)
  {
    if (run_len[r] < tmin)
      tmin = run_len[r];
  }

  // Guarded against a lone noise spike by requiring a second run of similar
  // width; if the shortest is alone, step up to the next width and ask again
  for (int tries = 0; tries < 4; tries++)
  {
    int near = 0, next = 0;

    for (int r = first; r <= last; r++)
    {
      if (run_len[r] <= tmin + tmin / 4)
        near++;
      else if (next == 0 || run_len[r] < next)
        next = run_len[r];
    }

    if (near >= 2 || next == 0)
      break;

    tmin = next;
  }

  return tmin;
}

//-----------------------------------------------------------------------------
// The bit times worth trying, in x256 fixed-point samples: every standard
// rate that could physically be in this record, plus the rate the runs
// themselves imply, for a link that is not on a standard rate at all.
static int uart_candidates(int size, int period_ns, const LogicScratch *scratch,
    int runs, int *cand_bit, int *cand_rate)
{
  const int *run_len = scratch->len;
  const int first = 1, last = runs - 2;
  int tmin = uart_min_run(scratch, runs);
  int n_cand = 0;

  if (tmin < MIN_BIT_SAMPLES)
    return 0;

  // Candidates, in x256 fixed-point samples per bit. A run is a whole number
  // of bit times, so a bit cannot be meaningfully wider than the shortest
  // run - that upper bound is what keeps the list to a handful of entries and
  // keeps a half-rate harmonic out of it.
  int64_t bit_max = BIT_MAX_X256(tmin);
  int64_t bit_min = (int64_t)MIN_BIT_SAMPLES * 256;

  for (unsigned i = 0; i < ARRAY_SIZE(g_standard_bauds); i++)
  {
    int64_t bit = bit_width_x256(g_standard_bauds[i], period_ns);

    if (bit < bit_min || bit > bit_max)
      continue;

    if (bit * MAX_FRAME_BITS > (int64_t)size * 256)
      continue; // not even one frame fits in the record at this rate

    cand_bit[n_cand] = (int)bit;
    cand_rate[n_cand] = g_standard_bauds[i];
    n_cand++;
  }

  // Plus the rate the runs themselves imply, for a link that is not on a
  // standard baud at all: every run is an integer number of bits, so average
  // the implied bit width over the runs that fit inside a frame
  {
    int64_t width_sum = 0;
    int bit_count = 0;

    for (int r = first; r <= last; r++)
    {
      int bits = (run_len[r] + tmin / 2) / tmin;

      if (bits < 1 || bits > MAX_FRAME_BITS)
        continue; // idle gap or glitch: not part of the estimate

      width_sum += run_len[r];
      bit_count += bits;
    }

    if (bit_count >= MIN_RUNS)
    {
      int64_t bit = width_sum * 256 / bit_count;
      bool standard = false;

      // A width measured off the runs lands a percent or two from the truth;
      // when a standard rate is already that close, it IS that rate, and
      // letting the measurement compete would hand the win to whichever of
      // the two happens to be the wider by a hair
      for (int c = 0; c < n_cand; c++)
      {
        int64_t diff = bit - cand_bit[c];

        if (diff < 0)
          diff = -diff;

        if (diff * 100 <= (int64_t)cand_bit[c] * 4)
          standard = true;
      }

      if (!standard && bit >= bit_min && bit * MAX_FRAME_BITS <= (int64_t)size * 256)
      {
        cand_bit[n_cand] = (int)bit;
        cand_rate[n_cand] = (int)(256000000000ll / (bit * period_ns));
        n_cand++;
      }
    }
  }

  return n_cand;
}

//-----------------------------------------------------------------------------
int uart_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  const int *run_len = scratch->len;
  const uint8_t *run_lvl = scratch->lvl;
  int cand_bit[MAX_CANDIDATES];
  int cand_rate[MAX_CANDIDATES];
  int64_t best_score = 0;
  int best_bit = 0, best_rate = 0, best_idle = 1;
  int n_cand, mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_UART;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < MIN_RUNS || period_ns <= 0)
    return 0;

  if (g_forced_baud > 0)
  {
    // Told the rate: that one candidate and no search. The polarity is still
    // worked out and the framing still has to hold up, so a wrong answer
    // here reads as "no frames" rather than as plausible-looking rubbish.
    int64_t bit = bit_width_x256(g_forced_baud, period_ns);

    if (bit < (int64_t)MIN_BIT_SAMPLES * 256 ||
        bit * MAX_FRAME_BITS > (int64_t)size * 256)
      return 0; // undersampled, or not one frame fits: nothing to decode

    // Not second-guessing the rate, stating a fact about the record: at this
    // rate one bit is wider than the shortest run in it, so whatever the
    // record holds, it is not traffic at this rate
    if (bit > BIT_MAX_X256(uart_min_run(scratch, runs)))
      return 0;

    cand_bit[0] = (int)bit;
    cand_rate[0] = g_forced_baud;
    n_cand = 1;
  }
  else
  {
    n_cand = uart_candidates(size, period_ns, scratch, runs, cand_bit, cand_rate);
  }

  if (n_cand == 0)
    return 0;

  // Try every candidate at both polarities and keep the best fit. Both is
  // cheap - the expensive part, splitting the record into runs, is already
  // done - and it beats guessing the idle level from the longest run, which
  // is wrong whenever the record lands inside a message.

  for (int p = 0; p < 2; p++)
  {
    int idle = p ? 0 : 1;

    for (int c = 0; c < n_cand; c++)
    {
      int64_t score = uart_scan(data, size, offset, mid, idle, cand_bit[c],
          scratch, runs, out);

      if (score > best_score)
      {
        best_score = score;
        best_bit   = cand_bit[c];
        best_rate  = cand_rate[c];
        best_idle  = idle;
      }
    }
  }

  if (best_bit == 0)
    return 0;

  uart_scan(data, size, offset, mid, best_idle, best_bit, scratch, runs, out);

  if (out->count < 2 || out->errors > out->count)
    return 0;

  out->idle_high = (best_idle != 0);
  out->rate = best_rate;

  // A continuous 50% square wave decodes as a flawless stream of 0x55: start
  // bit, alternating data, stop bit, forever. Real traffic either lets the
  // line rest somewhere in the record, or carries more than one byte value.
  // With neither, this record is not distinguishable from a clock - which is
  // reported, not acted on: the frames are real if the caller is looking at a
  // UART, and only the dispatcher knows whether it was asked for one. A
  // 0x55 sync preamble caught at a fast timebase lands here every time.
  int longest_idle = 0;

  for (int r = 0; r < runs; r++)
  {
    if (run_lvl[r] == best_idle && run_len[r] > longest_idle)
      longest_idle = run_len[r];
  }

  bool rested = ((int64_t)longest_idle * 256 >= (int64_t)best_bit * IDLE_GAP_BITS);
  bool varied = false;

  for (int i = 1; i < out->count && !varied; i++)
    varied = (out->bytes[i] != out->bytes[0]);

  out->ambiguous = !rested &&
      !(varied && out->count >= 4 && out->errors * 4 <= out->count);

  // A mark longer than a frame is not 8N1, whatever else it is. The line goes
  // down for at most nine bit times - a start bit and eight zero data bits -
  // and then the stop bit MUST let it up again. A 9 ms pulse in front of a
  // 560 us bit stream is an infrared leader, and 1795 baud is arithmetic
  // performed on it rather than evidence of anything: the leader is what says
  // what the signal is, and it says NEC.
  //
  // Reported, not refused, like the clock case: asked for UART by name the
  // frames still stand, because a line held down by a break condition or a
  // transmitter coming out of reset is a UART line with a long mark in it.
  // In auto mode it means the record belongs to whatever owns that mark.
  for (int r = 1; r + 1 < runs; r++) // interior runs: the ends are cut short
  {
    if (run_lvl[r] != best_idle && (int64_t)run_len[r] * 256 >
        (int64_t)best_bit * (MAX_FRAME_BITS - 1) * 5 / 4)
    {
      out->ambiguous = true;
      break;
    }
  }

  // Was the line still sending when the record ran out? The last run is the
  // one the end of the buffer cut through: shorter than an idle gap means the
  // message went on past it, and what was decoded is the part that fit. The
  // panel says so, because otherwise four bytes out of forty look like all
  // there was - and the fix is a slower timebase, not a better decoder.
  out->overrun = ((int64_t)run_len[runs - 1] * 256 <
      (int64_t)best_bit * IDLE_GAP_BITS);

  // The panel appends the byte count, so the errors only get their words
  // when there are any - a clean line is the common case and reads better
  // without "err 0" taking up a third of the header
  if (out->errors > 0)
    snprintf(out->info, sizeof(out->info), "UART %d 8N1%s%s err %d",
        out->rate, out->idle_high ? "" : " inv", out->truncated ? "+" : "",
        out->errors);
  else
    snprintf(out->info, sizeof(out->info), "UART %d 8N1%s%s",
        out->rate, out->idle_high ? "" : " inv", out->truncated ? "+" : "");

  return out->count;
}
