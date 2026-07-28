/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SENT - Single Edge Nibble Transmission, SAE J2716. The one-wire sensor
 * interface in a modern engine bay: pressure, position, mass air flow.
 *
 * There is no clock and no bit stream. Every pulse starts with a falling
 * edge, holds low for about five ticks and goes high for the rest, and what
 * carries the data is the TIME from one falling edge to the next: 12 ticks is
 * a nibble worth 0, 27 ticks a nibble worth 15.
 *
 * The tick itself is not agreed in advance. Every frame opens with a
 * synchronisation pulse of exactly 56 ticks, so the receiver divides by 56
 * and has the sender's own unit time - which is why a sensor whose clock has
 * drifted 20 % still reads correctly. This decoder does the same, and that is
 * also what makes SENT recognisable: a 56-tick pulse followed by a run of
 * pulses that all land between 12 and 27 of those same ticks is not a shape
 * other signals fall into by accident.
 *
 * Frame: sync | status | 1..6 data nibbles | CRC | optional pause.
 *
 * The CRC is four bits over the data nibbles, polynomial x^4+x^3+x^2+1 seeded
 * with 0101. J2716 blessed two of them - the 2008 one, and the 2010 version
 * that clocks an extra zero nibble through at the end - and parts in the
 * field use both, so both are accepted. A frame that matches neither is still
 * reported, but marked ambiguous: in auto mode the cascade moves on rather
 * than claim SENT on structure alone, and asked for SENT by name it decodes.
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
#define SENT_SYNC_TICKS      56
#define SENT_NIB_MIN_TICKS   12   // nibble value 0
#define SENT_NIB_MAX_TICKS   27   // nibble value 15

// The standard's range for the unit time is 0.5 to 90 us; 3 us is what
// nearly everything actually runs at
#define SENT_TICK_MIN_NS       400
#define SENT_TICK_MAX_NS    100000

// Below this the tick is not measured well enough for a nibble to be told
// from its neighbour: one sample of error is a quarter of a tick at four
#define SENT_TICK_MIN_SAMPLES  4

// A pulse period has to sit on a tick boundary, in tenths of a tick. The
// error is in the sampling and not in the sensor - each edge is placed within
// one sample, so a period measured between two of them is good to about two -
// which is a fifth of a tick where there are ten samples to one and half of a
// tick where there are four. A fixed tolerance in ticks is therefore either
// too loose at a slow timebase or too tight at a fast one, and too tight is
// what breaks a frame in the middle and hands a broken CRC to the panel.
#define SENT_QUANT_TOL_X10     3   // floor: 0.3 tick
#define SENT_QUANT_TOL_MAX    45   // ceiling: beyond 4.5 a nibble is not one
#define SENT_EDGE_SAMPLES      2   // ...and never tighter than two samples

// The sync pulse holds low for five ticks. That is not a tolerance in the
// standard, it is the definition of a pulse, and it is the one thing a signal
// cannot fake by accident: a UART record whose falling edges happen to land
// on a tick grid still has data-dependent low times, and they are whole bit
// periods rather than five of a tick that divides one. The sync is where this
// is checked strictly, because the sync is what anchors the whole frame.
#define SENT_SYNC_LOW_MIN_X10 35   // 3.5 ticks
#define SENT_SYNC_LOW_MAX_X10 75   // 7.5 ticks

// The nibbles that follow are checked loosely. Their VALUE is the period, not
// the low time, so a pulse whose low came out a little long is still a nibble
// - and cutting the frame there costs the CRC and every nibble after it.
#define SENT_NIB_LOW_MIN_X10  20
#define SENT_NIB_LOW_MAX_X10 105
#define SENT_LOW_MATCH_X10     5   // and within 50 % of the sync's own low

// The standard allows one data nibble; nothing in the field sends fewer than
// three, and a four-bit CRC over one nibble is a coin toss against the number
// of places a long record offers to start looking. So a shorter frame is
// decoded when SENT is asked for by name, and is not enough to claim SENT.
#define SENT_AUTO_MIN_DATA     3

#define SENT_MAX_NIBBLES      12   // status + up to 6 data + CRC, with slack

/*- Variables ---------------------------------------------------------------*/
static SentAnalysis g_sent;

// Shifting a nibble of zeros through the J2716 polynomial. The whole CRC is
// this table plus an exclusive-or per nibble.
static const uint8_t g_sent_crc_tab[16] =
{
  0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint8_t sent_crc(const uint8_t *nib, int n, bool recommended)
{
  uint8_t crc = 5;   // seed 0101

  for (int i = 0; i < n; i++)
    crc = (uint8_t)(g_sent_crc_tab[crc] ^ (nib[i] & 0x0F));

  if (recommended)
    crc = g_sent_crc_tab[crc];   // the 2010 version's extra zero nibble

  return crc;
}

//-----------------------------------------------------------------------------
// J2716 blessed two CRCs: the 2008 one, and the 2010 version that clocks an
// extra zero nibble through at the end. Both are tried and no more.
//
// The temptation is to try every convention anyone has ever shipped, but the
// CRC here is four bits: every extra hypothesis is another one-in-sixteen
// chance of agreeing with a frame that is actually broken, and a wrong frame
// wearing "CRC ok" hides the fault that a wrong frame saying "CRC!" reports.
// Two hypotheses is what the standard justifies. A part that turns out to
// disagree shows its nibbles on the panel and can be added on evidence.
static bool sent_crc_check(const uint8_t *nib, int take, bool *legacy)
{
  uint8_t rx = nib[take - 1];

  if (sent_crc(&nib[1], take - 2, true) == rx)
  {
    *legacy = false;

    return true;
  }

  if (sent_crc(&nib[1], take - 2, false) == rx)
  {
    *legacy = true;

    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
// How far a measured period may sit off a tick boundary: two samples of edge
// placement, and never less than a fifth of a tick
static int64_t sent_tol10(int64_t tick_x256)
{
  int64_t tol = (int64_t)SENT_EDGE_SAMPLES * 256 * 10 / tick_x256;

  if (tol < SENT_QUANT_TOL_X10)
    tol = SENT_QUANT_TOL_X10;

  if (tol > SENT_QUANT_TOL_MAX)
    tol = SENT_QUANT_TOL_MAX;

  return tol;
}

//-----------------------------------------------------------------------------
const SentAnalysis *sent_analysis(void)
{
  return &g_sent;
}

//-----------------------------------------------------------------------------
void sent_byte_label(const SentAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  const SentFrame *f = &a->frame[a->fidx[idx] % SENT_MAX_FRAMES];
  int rel = idx - f->first - 1;   // 0-based position among the data nibbles

  switch (a->role[idx])
  {
    case SENT_R_STATUS:
      snprintf(buf, size, "STAT");
      break;

    case SENT_R_DATA:
      // Three nibbles to a 12-bit signal is what nearly every sensor sends,
      // and the value goes on the nibble that finishes it. Where the count
      // does not say so plainly the nibbles stay nibbles.
      if (f->have_s1 && 2 == rel)
        snprintf(buf, size, "S1=%03X", (unsigned)f->s1);
      else if (f->have_s2 && 5 == rel)
        snprintf(buf, size, "S2=%03X", (unsigned)f->s2);
      else
        snprintf(buf, size, "D%d", rel + 1);
      break;

    case SENT_R_CRC:
      snprintf(buf, size, f->crc_ok ? "CRC" : "CRC!");
      break;

    default:
      break;
  }
}

//-----------------------------------------------------------------------------
// What this nibble is on its own, never the number its group adds up to.
// Grouping says three nibbles are one signal; this says which of the three
// you are looking at, and the two answers belong on different rows.
void sent_field_label(const SentAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (SENT_R_DATA == a->role[idx])
  {
    const SentFrame *f = &a->frame[a->fidx[idx] % SENT_MAX_FRAMES];

    snprintf(buf, size, "D%d", idx - f->first);
  }
  else
    sent_byte_label(a, idx, v, buf, size);
}

//-----------------------------------------------------------------------------
// Three nibbles are one number, so they are one thing on the screen: the
// signal is written once across all three rather than "D1 D2 S1=543", which
// leaves the reader to work out that the first two belong to the third.
void sent_group_at(const SentAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  const SentFrame *f = &a->frame[a->fidx[idx] % SENT_MAX_FRAMES];
  int rel = idx - f->first - 1;   // 0-based among the data nibbles

  if (rel < 0 || rel >= f->ndata)
    return;                       // the status nibble, or the CRC

  if (f->have_s1 && rel < 3)
  {
    *start = f->first + 1;
    *len = 3;
  }
  else if (f->have_s2 && rel >= 3 && rel < 6)
  {
    *start = f->first + 4;
    *len = 3;
  }
}

//-----------------------------------------------------------------------------
static void sent_tick_str(int tick_ns, char *buf, int size)
{
  if (tick_ns < 0)
    tick_ns = 0;

  if (tick_ns > 999999)
    tick_ns = 999999;

  snprintf(buf, size, "%d.%dus", tick_ns / 1000, (tick_ns % 1000) / 100);
}

//-----------------------------------------------------------------------------
// Read the pulses that follow the sync at run `sync_run`, at the tick time
// the sync itself gave. Returns how many nibbles came out, and sets *cut when
// the record ended before the frame did.
static int sent_nibbles(const LogicScratch *s, int runs, int64_t tick_x256,
    int first_run, int64_t sync_low10, uint8_t *nib, int *npos, int *nend,
    bool *cut)
{
  int n = 0;
  int64_t tol10 = sent_tol10(tick_x256);
  int64_t slack10 = (int64_t)SENT_EDGE_SAMPLES * 256 * 10 / tick_x256;

  *cut = false;

  for (int r = first_run; n < SENT_MAX_NIBBLES; r += 2)
  {
    if (r + 2 >= runs)
    {
      *cut = true;   // no closing edge, so no measurable pulse
      break;
    }

    int64_t per = s->pos[r + 2] - s->pos[r];
    int64_t t10 = per * 256 * 10 / tick_x256;      // ticks, tenths
    int q = (int)((t10 + 5) / 10);
    int64_t low10 = (int64_t)s->len[r] * 256 * 10 / tick_x256;

    if (q < SENT_NIB_MIN_TICKS || q > SENT_NIB_MAX_TICKS)
      break;                                       // a pause, or the next sync

    if (t10 - (int64_t)q * 10 > tol10 || (int64_t)q * 10 - t10 > tol10)
      break;                                       // not on a tick boundary

    if (low10 + slack10 < SENT_NIB_LOW_MIN_X10 ||
        low10 - slack10 > SENT_NIB_LOW_MAX_X10)
      break;

    // The same transmitter drove all of these, so the low time is the same
    // one every time - it is not a per-pulse parameter
    if ((low10 - slack10) * 10 > sync_low10 * (10 + SENT_LOW_MATCH_X10) ||
        (low10 + slack10) * 10 < sync_low10 * (10 - SENT_LOW_MATCH_X10))
      break;

    nib[n] = (uint8_t)(q - SENT_NIB_MIN_TICKS);
    npos[n] = s->pos[r];
    nend[n] = s->pos[r + 2];
    n++;
  }

  return n;
}

//-----------------------------------------------------------------------------
// One pass over the record with a given pulse polarity. Returns the number of
// frames found; out and a may be NULL for a counting-only pass.
static int sent_scan(const LogicScratch *s, int runs, int period_ns, int lvl,
    LogicResult *out, SentAnalysis *a)
{
  int found = 0;
  int expect = 0;   // nibbles per frame, once a checked frame has shown it

  // From run 1: the record's first run is cut by its own start, so the pulse
  // it belongs to began before the record and its length is a fact about the
  // buffer rather than about the signal
  for (int r = 1; r + 2 < runs; r++)
  {
    if (s->lvl[r] != lvl)
      continue;

    // Every frame opens with a pulse of exactly 56 ticks, which is how the
    // tick is learned in the first place
    int64_t sync = s->pos[r + 2] - s->pos[r];
    int64_t tick_x256 = sync * 256 / SENT_SYNC_TICKS;

    if (tick_x256 < SENT_TICK_MIN_SAMPLES * 256)
      continue;

    int tick_ns = (int)(tick_x256 * period_ns / 256);

    if (tick_ns < SENT_TICK_MIN_NS || tick_ns > SENT_TICK_MAX_NS)
      continue;

    int64_t low10 = (int64_t)s->len[r] * 256 * 10 / tick_x256;
    int64_t slack10 = (int64_t)SENT_EDGE_SAMPLES * 256 * 10 / tick_x256;

    // Strictly, here and only here: the sync is what calibrates the frame,
    // and its five-tick low is what a signal cannot fake by accident
    if (low10 + slack10 < SENT_SYNC_LOW_MIN_X10 ||
        low10 - slack10 > SENT_SYNC_LOW_MAX_X10)
      continue;

    uint8_t nib[SENT_MAX_NIBBLES];
    int npos[SENT_MAX_NIBBLES], nend[SENT_MAX_NIBBLES];
    bool cut;
    int n = sent_nibbles(s, runs, tick_x256, r + 2, low10, nib, npos, nend,
        &cut);

    // Status, at least one data nibble, CRC
    if (n < 3)
      continue;

    int take = 0;
    bool ok = false, legacy = false, no_crc = false;

    // A sensor sends frames of one length. Once one of them has checked out,
    // that length is what the next is tried as first - and a frame that came
    // out SHORTER than it, with the record ending inside it, is a frame this
    // record cut rather than a shorter format. Not testing its CRC at all is
    // the point: four bits agree with a broken frame once in sixteen tries,
    // and the frames around it have already said what the format is.
    bool short_and_cut = (expect >= 3 && n < expect && cut);

    if (expect >= 3 && expect <= n && sent_crc_check(nib, expect, &legacy))
    {
      take = expect;
      ok = true;
    }

    // A pause pulse is 12 to 768 ticks, so a short one is indistinguishable
    // from a nibble until the CRC is asked: try the frame as read, then one
    // nibble shorter. Not where the record cut it, though - there the shorter
    // reading is a second guess on top of a frame that is already incomplete.
    if (!take && !short_and_cut)
    {
      for (int t = n; t >= 3 && t >= n - (cut ? 0 : 1); t--)
      {
        if (sent_crc_check(nib, t, &legacy))
        {
          take = t;
          ok = true;
          break;
        }
      }
    }

    if (!take)
    {
      take = n;

      // A nibble is the distance between two falling edges, so the LAST pulse
      // of a frame needs a closing edge to have a value at all - which on a
      // real bus is the pause pulse or the next frame's sync. Where the record
      // ran out before that edge arrived, the last pulse has no value here,
      // and the last one that does is not the CRC: it is the data nibble
      // before it. Calling it the CRC would put a wrong number under the word
      // and shift every label in the frame.
      no_crc = cut;
    }

    if (ok && !expect)
      expect = take;

    found++;

    if (out && a)
    {
      if (out->count + take > LOGIC_MAX_BYTES || a->frames >= SENT_MAX_FRAMES)
      {
        out->truncated = true;
        break;
      }

      SentFrame *f = &a->frame[a->frames];

      memset(f, 0, sizeof(*f));
      f->status = nib[0];
      f->ndata = (uint8_t)(no_crc ? take - 1 : take - 2);
      f->crc = no_crc ? 0 : nib[take - 1];
      f->crc_ok = ok;
      f->legacy = legacy;
      f->no_crc = no_crc;
      f->first = out->count;
      f->count = take;

      // Three nibbles to a signal, MSB first. Six data nibbles are two
      // signals, three are one; anything else is left as nibbles rather
      // than assembled on a guess.
      if (6 == f->ndata || 3 == f->ndata)
      {
        f->s1 = (nib[1] << 8) | (nib[2] << 4) | nib[3];
        f->have_s1 = true;
      }

      if (6 == f->ndata)
      {
        f->s2 = (nib[4] << 8) | (nib[5] << 4) | nib[6];
        f->have_s2 = true;
      }

      for (int i = 0; i < take; i++)
      {
        int idx = out->count;

        out->bytes[idx] = nib[i];
        out->pos[idx] = npos[i];
        out->end[idx] = nend[i];
        a->fidx[idx] = (uint8_t)a->frames;
        a->role[idx] = (0 == i) ? SENT_R_STATUS :
            (!no_crc && i == take - 1) ? SENT_R_CRC : SENT_R_DATA;
        out->count++;
      }

      a->frames++;

      if (ok)
        a->crc_ok++;
      else if (!no_crc)
        out->errors++;   // a frame that was there and did not check out

      // The record ending right after a frame is not a cut frame: what ran
      // out was the pause that follows it. Only a frame that also failed its
      // CRC is one this record probably did not hold all of.
      if (cut && !ok)
        out->overrun = true;

      a->tick_ns = tick_ns;
    }

    // Carry on past the frame; the loop's own step takes it to the run after
    r += 2 * take;
  }

  return found;
}

//-----------------------------------------------------------------------------
int sent_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  (void)size;
  (void)offset;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_SENT;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 8 || period_ns <= 0)
    return 0;

  // A pulse begins with a falling edge on any sensor built to the standard,
  // but an inverting buffer between it and the probe costs one more pass
  int best = 0, best_lvl = 0;

  for (int p = 0; p < 2; p++)
  {
    int n = sent_scan(scratch, runs, period_ns, p, NULL, NULL);

    if (n > best)
    {
      best = n;
      best_lvl = p;
    }
  }

  if (0 == best)
    return 0;

  memset(&g_sent, 0, sizeof(g_sent));
  sent_scan(scratch, runs, period_ns, best_lvl, out, &g_sent);

  if (0 == out->count)
    return 0;

  out->rate = g_sent.tick_ns;
  out->idle_high = (0 == best_lvl);
  out->burst_start = true;   // a frame is found from its own sync pulse

  // The shape alone is a strong statement and not a proof. What settles it is
  // a frame long enough for its CRC to mean something and a CRC that agrees;
  // short of that the cascade should keep looking, and only a user who picked
  // SENT by name gets these frames.
  bool confident = false;

  for (int i = 0; i < g_sent.frames; i++)
  {
    if (g_sent.frame[i].crc_ok && g_sent.frame[i].ndata >= SENT_AUTO_MIN_DATA)
      confident = true;
  }

  out->ambiguous = !confident;

  char ts[12];

  sent_tick_str(g_sent.tick_ns, ts, sizeof(ts));

  // A frame whose last pulse never closed is not a frame that failed its CRC:
  // it is a frame this record did not hold the end of, and saying "CRC!" for
  // it points at the sensor for what is the timebase's doing
  bool any_cut = false, any_bad = false;

  for (int i = 0; i < g_sent.frames; i++)
  {
    if (g_sent.frame[i].no_crc)
      any_cut = true;
    else if (!g_sent.frame[i].crc_ok)
      any_bad = true;
  }

  const char *bad = any_bad ? " CRC!" : any_cut ? " cut" : "";

  if (1 == g_sent.frames)
  {
    const SentFrame *f = &g_sent.frame[0];

    if (f->have_s2)
      snprintf(out->info, sizeof(out->info), "SENT %s %03X %03X%s", ts,
          (unsigned)f->s1, (unsigned)f->s2, bad);
    else if (f->have_s1)
      snprintf(out->info, sizeof(out->info), "SENT %s %03X%s", ts,
          (unsigned)f->s1, bad);
    else
      snprintf(out->info, sizeof(out->info), "SENT %s %dnib%s", ts,
          f->ndata, bad);
  }
  else
  {
    snprintf(out->info, sizeof(out->info), "SENT %s %d frames%s", ts,
        g_sent.frames, bad);
  }

  return out->count;
}
