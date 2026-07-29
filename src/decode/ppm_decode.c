/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CPPM - the sum signal on the one pin an RC receiver labels PPM, carrying
 * every channel of the link where a servo output carries one.
 *
 * A frame is a train of narrow separator pulses - 300 to 500 us, all the same
 * width - and the VALUE of a channel is the time from one separator to the
 * next: 1.0 ms is one end of the stick, 2.0 the other. After the last channel
 * comes a sync gap of several milliseconds, and the next frame starts. So a
 * receiver counts pulses between gaps, and so does this.
 *
 * That is worth stating plainly because it is upside down from the servo line
 * next to it: a servo pulse carries its value in its WIDTH and the gap is
 * only a gap, while here the pulses are all one width and the GAPS carry
 * everything. Reading a PPM stream with the servo decoder gives a column of
 * identical 0.4 ms numbers, all of them true and none of them the signal.
 *
 * What identifies it:
 *
 *   - every separator is the same narrow pulse, whatever the sticks are doing
 *   - every interval between two separators is either a channel (0.7..2.4 ms)
 *     or a sync gap (3..40 ms), and nothing lands between the two windows
 *   - between two sync gaps sit 3 to 16 channels, which is a receiver
 *
 * The third is the one that has to be met by a WHOLE frame - a sync, the
 * channels, and the closing sync - so a record that catches only part of one
 * is not enough to claim the protocol. The channels of the frame the record
 * ended inside are still reported, marked as cut: their numbering is known
 * because the sync in front of them was seen, and a missing tail is the
 * record running out rather than the receiver sending fewer.
 *
 * The frame period is NOT required to hold. Most transmitters pad the sync so
 * the frame is a fixed 20 or 22.5 ms, but flight controllers exist that keep
 * the SYNC fixed and let the frame breathe with the stick positions, and both
 * are ordinary PPM. What is required to hold is the channel COUNT, and where
 * it does not the header says so rather than averaging it away - a receiver
 * dropping a channel is the fault someone is looking at this signal to find.
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
// The separator itself. 300..500 us is what receivers send; the window is
// wide because the pulse is not the value and only has to be recognisable.
#define PPM_MARK_MIN_US       50
#define PPM_MARK_MAX_US      900

// A channel. 1000..2000 us is the travel, 500..2500 the extended range every
// receiver understands, and the margin catches one trimmed past it rather
// than calling the line something else.
#define PPM_CH_MIN_US        700
#define PPM_CH_MAX_US       2400

// ...and the gap that closes a frame. There is DELIBERATELY a hole between
// this and the channel window: an interval of 2.7 ms is neither a channel nor
// a sync, and a decoder that rounded it to the nearer one would turn a broken
// signal into a plausible reading of one.
#define PPM_SYNC_MIN_US     3000
#define PPM_SYNC_MAX_US    40000

// Four is the smallest link anyone sells and sixteen the largest anyone sends
#define PPM_MIN_CHANS          3
#define PPM_MAX_CHANS         16

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_ppm   (g_logic_analysis.ppm)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const PpmAnalysis *ppm_analysis(void)
{
  return &g_ppm;
}

//-----------------------------------------------------------------------------
// Which frame byte `idx` belongs to, or NULL
static const PpmFrame *ppm_frame_at(const PpmAnalysis *a, int idx)
{
  if (idx < 0)
    return NULL;

  for (int i = 0; i < a->frames && i < PPM_MAX_FRAMES; i++)
  {
    const PpmFrame *f = &a->frame[i];

    if (idx >= f->first && idx < f->first + f->count)
      return f;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
// The byte IS the interval, in tens of microseconds. Put back into the units
// the link is set up in, which is the only form anyone reads - and the same
// form the servo decoder uses, because it is the same number and a reader
// moving between a receiver's PPM pin and one of its servo outputs should not
// have to convert in their head.
void ppm_byte_label(const PpmAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)a;
  (void)idx;

  snprintf(buf, size, "%d.%02dms", v / 100, v % 100);
}

//-----------------------------------------------------------------------------
// ...and which channel it is. Numbered from one, and from the sync gap in
// front of it rather than from the start of the record: channel 3 is channel
// 3 of its own frame, whichever frame the record happened to open in.
void ppm_field_label(const PpmAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  const PpmFrame *f = ppm_frame_at(a, idx);

  if (!f)
    return;

  snprintf(buf, size, "CH%d", idx - f->first + 1);
}

//-----------------------------------------------------------------------------
// Are all the runs at `level` separator pulses? A PPM line carries separators
// and nothing else, so one oversized mark means this is some other signal
// that happens to contain narrow pulses.
//
// The first and last runs are skipped whatever they are - both are cut by the
// ends of the record, so their width is a fact about the buffer.
static bool ppm_marks_fit(const LogicScratch *s, int runs, int period_ns,
    int level)
{
  int n = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (s->lvl[r] != level)
      continue;

    int64_t us = (int64_t)s->len[r] * period_ns / 1000;

    if (us < PPM_MARK_MIN_US || us > PPM_MARK_MAX_US)
      return false;

    n++;
  }

  // Three channels between two syncs need four separators, and the sync that
  // closes the frame is a fifth
  return n >= PPM_MIN_CHANS + 2;
}

//-----------------------------------------------------------------------------
// One pass over the separators at `level`, from the first sync gap onwards.
// Returns the number of WHOLE frames - a sync, its channels and the closing
// sync - or -1 where an interval was neither a channel nor a sync, which is
// the record saying it is not this protocol at all.
static int ppm_walk(const LogicScratch *s, int runs, int period_ns, int level,
    LogicResult *out, PpmAnalysis *a)
{
  int whole = 0;
  int prev = -1;        // the separator before this one
  int frame_pos = -1;   // ...and where the frame being counted opened
  bool started = false; // a sync has gone past, so the channels are numbered
  PpmFrame *f = NULL;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (s->lvl[r] != level)
      continue;

    if (prev < 0)
    {
      prev = r;
      continue;
    }

    int p0 = s->pos[prev];
    int64_t us = (int64_t)(s->pos[r] - p0) * period_ns / 1000;

    prev = r;

    if (us >= PPM_CH_MIN_US && us <= PPM_CH_MAX_US)
    {
      if (!started || !f)
        continue;       // in front of the first sync, or of a frame there was
                        // no room to record: a channel belonging to nothing

      if (f->chans >= PPM_MAX_CHANS)
        return -1;      // more channels than any link sends: not a frame

      // Counted whatever happens to the result buffer: how many channels the
      // frame HELD is a fact about the signal, and how many of them fitted in
      // sixty-four bytes is a fact about the record
      f->chans++;

      if (!out)
        continue;

      if (out->count >= LOGIC_MAX_BYTES)
      {
        out->truncated = true;
        continue;
      }

      // Separator to separator, which is the interval that was measured -
      // so the byte lights the part of the trace its value came out of.
      //
      // Rounded to the nearest ten microseconds rather than truncated: at a
      // timebase that shows two whole frames a sample is several microseconds
      // wide, so the interval lands a sample either side of the round number
      // it really is, and always taking the lower one turns a 2.00 ms channel
      // into 1.99 about half the time.
      out->bytes[out->count] = (uint8_t)((us + 5) / 10);
      out->pos[out->count] = p0;
      out->end[out->count] = s->pos[r];
      out->count++;
      f->count++;

      continue;
    }

    if (us < PPM_SYNC_MIN_US || us > PPM_SYNC_MAX_US)
      return -1;        // neither a channel nor a sync: whatever it is, not PPM

    // A sync gap: `prev` was the last separator of the frame that was open,
    // and `r` is the first of the next one
    if (started && f)
    {
      if (f->chans < PPM_MIN_CHANS)
        return -1;      // too few between two syncs to be a receiver

      f->cut = false;
      whole++;

      // Measured first separator to first separator, which is how a receiver
      // measures it - and only off the first whole frame, because the frame
      // period is not required to hold. See the head of the file.
      if (a && 0 == a->frame_ns)
      {
        a->frame_ns = (int)((int64_t)(s->pos[r] - frame_pos) * period_ns);
        a->sync_ns = (int)(us * 1000);
      }
    }

    started = true;
    frame_pos = s->pos[r];
    f = NULL;

    if (a && a->frames < PPM_MAX_FRAMES)
    {
      f = &a->frame[a->frames];
      a->frames++;
      f->first = (uint8_t)(out ? out->count : 0);
      f->count = 0;
      f->chans = 0;
      f->cut = true;    // until a closing sync says otherwise
    }
    else if (out)
    {
      out->truncated = true;
    }
  }

  // The frame the record ended inside keeps its `cut`, and its channels with
  // it: they were numbered from a sync that WAS seen, so they are channel 1
  // and channel 2 of a frame whose tail the record did not reach - which is
  // a different statement from a receiver that sent two channels.
  return whole;
}

//-----------------------------------------------------------------------------
int ppm_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_PPM;

  runs = logic_runs(data, size, offset, scratch, &mid);

  // Two syncs and three channels between them is four separators and their
  // gaps, with the record opening somewhere in front
  if (runs < 2 * (PPM_MIN_CHANS + 2) || period_ns <= 0)
    return 0;

  // Which level is the separator? A receiver rests low and pulses high, and
  // the Futaba convention rests high and pulses low - both are ordinary PPM
  // and both are here. What decides is that the separator is the SHORTER part
  // of a channel by a wide margin: 400 us of pulse inside a 1500 us interval.
  // Mean per run and not the total, for the same reason the servo decoder
  // says: a record that ends after the last pulse has one more run at that
  // level, and totalled alone that would decide it.
  int64_t total[2] = { 0, 0 };
  int64_t runs_at[2] = { 0, 0 };

  for (int r = 1; r + 1 < runs; r++)
  {
    total[scratch->lvl[r] & 1] += scratch->len[r];
    runs_at[scratch->lvl[r] & 1]++;
  }

  int prefer = (0 == runs_at[0] || total[1] * runs_at[0] <=
      total[0] * runs_at[1]) ? 1 : 0;
  int level = -1;

  for (int p = 0; p < 2; p++)
  {
    int try_level = p ? !prefer : prefer;

    if (ppm_marks_fit(scratch, runs, period_ns, try_level))
    {
      level = try_level;
      break;
    }
  }

  if (level < 0)
    return 0;

  memset(&g_ppm, 0, sizeof(g_ppm));

  int whole = ppm_walk(scratch, runs, period_ns, level, out, &g_ppm);

  // One whole frame is the claim: a sync, the channels between it and the
  // next sync, and the next sync. Anything less is a train of pulses that
  // could be a great many things.
  if (whole < 1 || 0 == out->count)
    return 0;

  g_ppm.inv = (0 == level);
  out->idle_high = (0 == level);   // the pulses are the OTHER level
  out->burst_start = true;         // a frame is found from its own sync gap
  out->rate = g_ppm.frame_ns;

  // The channel count every whole frame agreed on. A frame the record cut
  // short is not counted against them: it has as many channels as fitted, and
  // that is a fact about the record.
  int chans = 0;

  for (int i = 0; i < g_ppm.frames; i++)
  {
    const PpmFrame *f = &g_ppm.frame[i];

    if (f->cut)
      continue;

    if (0 == chans)
      chans = f->chans;
    else if (chans != f->chans)
      chans = -1;
  }

  g_ppm.chans = (chans > 0) ? chans : 0;

  int hz = 0;

  if (g_ppm.frame_ns > 0)
    hz = (int)((1000000000LL + g_ppm.frame_ns / 2) / g_ppm.frame_ns);

  const char *inv = g_ppm.inv ? " inv" : "";

  if (0 == g_ppm.chans)
    snprintf(out->info, sizeof(out->info), "PPM %d frames %dHz%s",
        whole, hz, inv);
  else if (whole > 1)
    snprintf(out->info, sizeof(out->info), "PPM %dch %dHz x%d%s",
        g_ppm.chans, hz, whole, inv);
  else
    snprintf(out->info, sizeof(out->info), "PPM %dch %dHz%s",
        g_ppm.chans, hz, inv);

  return out->count;
}
