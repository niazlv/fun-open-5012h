/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Manchester / bi-phase - not a protocol but a line code, and the brick a
 * couple of dozen protocols are built out of: RC5 remotes, DALI lighting,
 * EM4100 tags, most of the 433 MHz weather sensors, 10BASE-T on the wire.
 *
 * Every bit carries a transition IN THE MIDDLE of its period, and that is the
 * whole of it. Whether there is also one on the BOUNDARY depends on the data:
 * two bits the same need one to get back to the right level, two different
 * bits do not. So the line only ever holds a level for half a bit or a whole
 * one, never longer - and that is a signature, not a heuristic. A run list
 * with exactly two lengths in it standing in a 1:2 ratio is not something
 * UART, NEC or a strip of LEDs falls into.
 *
 * It is also self-clocking, which is the point of the code: a receiver that
 * has lost the phase gets it back at the next mid-bit transition. This
 * decoder does the same - it steps from measured edge to measured edge rather
 * than from a grid, so a transmitter whose crystal is a percent off still
 * reads to the end of the frame.
 *
 * Two things it CANNOT work out on its own, and does not pretend to:
 *
 *   - WHICH WAY ROUND the bits are. G.E. Thomas says a rising mid-bit edge is
 *     a one, IEEE 802.3 says it is a zero, and the two conventions are exact
 *     inverses. RC5 uses one, DALI the other. Nothing in the waveform
 *     distinguishes them, so it is a setting and the panel says which was
 *     used;
 *   - the RATE, when there is nothing to measure it against. The shortest run
 *     is half a bit ONLY if the record contains two adjacent equal bits
 *     somewhere. A stream of alternating bits has no half-bit run in it at
 *     all, and then the estimate comes out at twice the bit period with
 *     nothing in the data to say so. Told the rate, that cannot happen -
 *     which is why the rate is a menu entry.
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
// RC5 is 1.778 ms a bit and DALI 833 us; the fast end is where a tag or a
// wired bus lives. Wider than anything named, because the job is to turn down
// signals that are not this shape rather than rates that are unusual.
#define MAN_RATE_MIN         100
#define MAN_RATE_MAX      500000

// Per HALF bit, which is the thing actually being measured
#define MAN_MIN_SAMPLES        3

// How far a run may sit from half a bit or a whole one. Wide enough for a
// resonator and a receiver's edge shaping, and nowhere near wide enough to
// let three halves pass for two - which is what keeps NEC's 560/1690 out.
#define MAN_TOL_PCT           30

// Under this a frame is not a frame. Eight bits is less than any of the
// protocols this code carries and is already enough shape to mean something.
#define MAN_MIN_BITS           8
#define MAN_MAX_BITS          96
#define MAN_MAX_FRAMES         6

// A frame sits in SILENCE, and the silence has to be silence. Three bit times
// is longer than anything inside a frame can be - the longest run there is one
// bit - so a run of it either side is a boundary.
//
// This is what keeps ordinary serial traffic out, and it is not a detail. A
// stretch of a UART's runs that happen to be one and two bit times long IS a
// valid Manchester waveform at half the UART's rate; there is no reading of
// those samples alone that says otherwise. What says otherwise is what BOUNDS
// the stretch: a UART's next run is three or four bit times, which is a
// perfectly ordinary length, while a real frame is followed by a line at rest.
#define MAN_IDLE_BITS          3

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_man   (g_logic_analysis.man)

// 0 = work the bit time out of the record. Otherwise the rate the user picked,
// which is the only thing that settles the factor-of-two question.
static int g_man_rate = 0;

// false = a rising mid-bit edge is a one (G.E. Thomas, RC5)
// true  = a rising mid-bit edge is a zero (IEEE 802.3, DALI)
static bool g_man_inverted = false;

// ...and which of the three the user asked for, because AUTO has to be
// remembered rather than collapsed into the flag it starts from
static int g_man_pol = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void manchester_decode_set_rate(int bps)
{
  g_man_rate = (bps > 0) ? bps : 0;
}

//-----------------------------------------------------------------------------
void manchester_decode_set_polarity(int inverted)
{
  g_man_pol = (inverted >= 0 && inverted <= MAN_POL_AUTO) ? inverted : 0;
  g_man_inverted = (1 == g_man_pol);
}

//-----------------------------------------------------------------------------
const ManAnalysis *manchester_analysis(void)
{
  return &g_man;
}

//-----------------------------------------------------------------------------
// How many halves this run is: 1, 2, or 0 for a run that is neither and so
// cannot be part of a Manchester frame at all
// 1 or 2 is an ordinary run. 3 or 4 is what a bit with NO transition in its
// middle leaves behind - the flat bit merges with the half before it, or with
// the halves on both sides - and that is an encoding violation rather than a
// different signal. It is allowed into a stretch, and counted.
static int man_halves(int len, int64_t half_x256)
{
  int64_t l = (int64_t)len * 256;

  for (int k = 1; k <= 4; k++)
  {
    int64_t want = half_x256 * k;
    int64_t d = l - want;

    if (d < 0)
      d = -d;

    if (d * 100 <= want * MAN_TOL_PCT)
      return k;
  }

  return 0;
}

//-----------------------------------------------------------------------------
// The half-bit time the record itself implies. Manchester runs come in
// exactly two lengths standing in a 1:2 ratio, so the shorter population IS
// the half bit - as long as the record contains one. Where every run is the
// same length the answer is ambiguous by exactly a factor of two, and the
// caller is told so rather than being handed one of the two readings.
static int64_t man_estimate_half(const LogicScratch *s, int runs, bool *both)
{
  int tmin = 0;

  *both = false;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (0 == tmin || s->len[r] < tmin)
      tmin = s->len[r];
  }

  if (tmin < MAN_MIN_SAMPLES)
    return 0;

  // Average the runs over the whole number of halves each of them holds, and
  // note whether both populations were actually there
  int64_t sum = 0;
  int halves = 0, longs = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    int k = ((int64_t)s->len[r] * 2 + tmin) / (2 * tmin);

    if (k < 1 || k > 2)
      continue;      // an idle gap between frames, or not this shape at all

    if (2 == k)
      longs++;

    sum += s->len[r];
    halves += k;
  }

  if (halves < MAN_MIN_BITS)
    return 0;

  *both = (longs > 0 && halves > 2 * longs);

  return sum * 256 / halves;
}

//-----------------------------------------------------------------------------
// Read a frame forward from the edge at run `e0`, taking that edge to be a
// MID-BIT one. Steps edge to edge rather than along a grid, which is what
// makes the code self-clocking in the first place.
//
// Returns the number of bits; fills the bit values MSB-first into `bytes` and
// the sample span of each byte.
static int man_read(const LogicScratch *s, int runs, int64_t bit_x256, int e0,
    int r1, uint8_t *bytes, int *bpos, int *bend, uint32_t *value, int *last,
    int *viol, int *viol_bit)
{
  int n = 0;
  int64_t half = bit_x256 / 2;
  int64_t tol = bit_x256 / 4;
  // Where the middle of the bit being read is, in x256 samples. Tracking the
  // POSITION rather than hopping edge to edge is what makes a missing
  // transition reportable: an edge-chaser has nowhere to be when the edge is
  // not there and simply stops, which turns a broken bit into a vanished
  // frame - and a vanished frame is the one answer that tells the reader
  // nothing at all.
  //
  // It still re-anchors on every edge it does find, which is the point of a
  // self-clocking code: a transmitter a percent off stays readable, and only
  // a bit with no middle costs anything.
  int64_t pos = (int64_t)s->pos[e0] * 256;
  // The last mid-bit edge a frame can have is the one it EXITS on. A frame
  // whose last bit leaves the line at its resting level has no run of its own
  // for that half - it merges into the idle - so the transition into the idle
  // IS that bit's middle, and stopping at the last in-frame run drops it.
  int64_t stop = ((int64_t)s->pos[r1] + s->len[r1]) * 256;
  int cur = e0;

  *value = 0;
  *viol = 0;
  *viol_bit = 0;
  *last = e0;

  while (n < MAN_MAX_BITS && pos <= stop + tol)
  {
    // The run this instant falls in, and whether an edge actually starts
    // there. An edge at the middle of a bit IS the bit; no edge there is an
    // encoding violation, and the level still gets read - it is what the
    // line was doing - but the frame is marked.
    while (cur + 1 < runs && (int64_t)s->pos[cur + 1] * 256 <= pos + tol)
      cur++;

    bool edge = ((int64_t)s->pos[cur] * 256 >= pos - tol) &&
        ((int64_t)s->pos[cur] * 256 <= pos + tol);

    if (!edge)
    {
      if (0 == *viol)
        *viol_bit = (uint8_t)n;

      (*viol)++;
    }

    int bit = (s->lvl[cur] != 0) ? 1 : 0;

    if (g_man_inverted)
      bit = !bit;

    if (0 == (n % 8))
      bpos[n / 8] = (int)((pos - half) / 256);

    bytes[n / 8] = (uint8_t)((n % 8) ? (bytes[n / 8] | (bit << (7 - n % 8)))
        : (uint8_t)(bit << 7));
    *value = (*value << 1) | (uint32_t)bit;
    bend[n / 8] = (int)((pos + half) / 256);
    *last = cur;
    n++;

    // One bit on - and snapped to the edge that is there, if one is
    int64_t want = pos + bit_x256;
    int j = cur;

    while (j + 1 < runs && (int64_t)s->pos[j + 1] * 256 <= want + tol)
      j++;

    if ((int64_t)s->pos[j] * 256 >= want - tol &&
        (int64_t)s->pos[j] * 256 <= want + tol)
      pos = (int64_t)s->pos[j] * 256;
    else
      pos = want;      // nothing there: keep the phase and carry on
  }

  return n;
}

//-----------------------------------------------------------------------------
int manchester_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;
  bool both = false;

  memset(out, 0, sizeof(*out));
  memset(&g_man, 0, sizeof(g_man));
  out->proto = PROTO_MANCH;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < MAN_MIN_BITS || period_ns <= 0)
    return 0;

  int64_t half_x256;

  if (g_man_rate > 0)
  {
    // Half a bit, from the rate the user picked
    half_x256 = (256000000000ll + (int64_t)g_man_rate * period_ns) /
        ((int64_t)g_man_rate * period_ns * 2);
    g_man.told = true;
    both = true;      // being told is what removes the factor-of-two doubt
  }
  else
  {
    half_x256 = man_estimate_half(scratch, runs, &both);
    g_man.told = false;
  }

  if (half_x256 < MAN_MIN_SAMPLES * 256)
    return 0;

  int64_t bit_x256 = half_x256 * 2;
  // Told the rate, that number is the bus's and is what gets reported: the
  // couple of hertz between it and 256e9/(bit x period) is this record's
  // sampling, not the transmitter's crystal, and printing it back would be
  // reporting the instrument to the user as if it were the signal.
  int rate = g_man.told ? g_man_rate :
      (int)(256000000000ll / (bit_x256 * period_ns));

  if (rate < MAN_RATE_MIN || rate > MAN_RATE_MAX)
    return 0;

  // Two passes at the most, and the second only when the convention was left
  // to be worked out. A preamble exists to be recognised: it is 0x55 in
  // nearly everything that has one, so a first byte of 0xAA means the frame
  // was read the wrong way round and the whole record flips.
  //
  // This is an assumption about the PROTOCOL and not a fact about the
  // waveform - 0x55 and 0xAA are equally good data, and no amount of looking
  // at the samples separates the two conventions. So it is done only when
  // asked for, and the panel says the reading was inferred.
  for (int pass = 0; pass < 2; pass++)
  {
    out->count = 0;
    out->truncated = false;
    g_man.frames = 0;
    g_man.bits = 0;
    g_man.viol = 0;
    g_man.sure = false;

    // Walk the record looking for stretches where EVERY run is half a bit or a
    // whole one. That is the structural claim, and it is checked before a
    // single bit is read out.
    for (int r = 1; r + 1 < runs && g_man.frames < MAN_MAX_FRAMES; r++)
    {
      if (0 == man_halves(scratch->len[r], half_x256))
        continue;

      int r1 = r;
      int shorts = 0, longs = 0, broken = 0;

      while (r1 + 1 < runs && man_halves(scratch->len[r1 + 1], half_x256))
        r1++;

      for (int k = r; k <= r1; k++)
      {
        int h = man_halves(scratch->len[k], half_x256);

        if (1 == h)
          shorts++;
        else if (2 == h)
          longs++;
        else
          broken++;
      }

      // A frame with a broken bit in it is still a frame; a stretch held
      // together BY its broken bits is a different signal read wrongly.
      //
      // The RATIO is what matters, not the count. Letting runs of three and
      // four halves into a stretch is exactly what lets an ordinary UART in:
      // at half its rate a three-bit run is three halves, so the runs that
      // used to END a stretch - and leave it bounded by something that is not
      // silence - now sit inside one and join two of them together. A real
      // frame has one broken bit in thirty runs, not one in three.
      if (broken > 2 || broken * 8 > shorts + longs)
      {
        r = r1;

        continue;
      }

      // Silence either side of it, or the end of the record - which is cut and
      // says nothing either way
      int64_t quiet = bit_x256 * MAN_IDLE_BITS;

      if (r > 1 && (int64_t)scratch->len[r - 1] * 256 < quiet)
      {
        r = r1;

        continue;
      }

      if (r1 + 1 < runs - 1 && (int64_t)scratch->len[r1 + 1] * 256 < quiet)
      {
        r = r1;

        continue;
      }

      // The edge that starts a frame may be a mid-bit one or a boundary one,
      // and nothing before it says which. The wrong choice runs out of
      // transitions almost at once - the mid-bit edges it is looking for are
      // only there when the data happens to put them there - so both are read
      // and the one that gets further is the right one. That is what a
      // self-clocking code lets you do.
      uint8_t b0[MAN_MAX_BITS / 8], b1[MAN_MAX_BITS / 8];
      int p0[MAN_MAX_BITS / 8], e0[MAN_MAX_BITS / 8];
      int p1[MAN_MAX_BITS / 8], e1[MAN_MAX_BITS / 8];
      uint32_t v0 = 0, v1 = 0;
      int last0 = r, last1 = r;
      int vi0 = 0, vi1 = 0, vb0 = 0, vb1 = 0;
      int lim = (r1 + 2 < runs) ? r1 + 2 : runs;
      int n0 = man_read(scratch, lim, bit_x256, r, r1, b0, p0, e0, &v0, &last0,
          &vi0, &vb0);
      int n1 = (r + 1 <= r1) ? man_read(scratch, lim, bit_x256, r + 1, r1, b1,
          p1, e1, &v1, &last1, &vi1, &vb1) : 0;

      // The reading with fewer violations wins, and the longer one only breaks
      // a tie: a phase that is half a bit out turns every single bit into a
      // violation, which is exactly how the wrong one now identifies itself.
      bool take0 = (vi0 < vi1) || (vi0 == vi1 && n0 >= n1);
      uint8_t *bytes = take0 ? b0 : b1;
      int *bpos = take0 ? p0 : p1;
      int *bend = take0 ? e0 : e1;
      uint32_t value = take0 ? v0 : v1;
      int n = take0 ? n0 : n1;
      int last = take0 ? last0 : last1;
      int viol = take0 ? vi0 : vi1;
      int viol_bit = take0 ? vb0 : vb1;

      r = r1;      // past this stretch either way

      if (n < MAN_MIN_BITS)
        continue;

      // One bit in twenty-four with no middle is a fault in an otherwise
      // sound frame. One in four is not a fault - it is a signal that does
      // not put a transition in the middle of its bits, which is every
      // signal that is not Manchester. The ratio is the line between "broken"
      // and "not this protocol", and it has to be drawn somewhere.
      if (viol * 6 > n)
        continue;

      // ...and it has to have eaten the WHOLE stretch. A Manchester frame's
      // runs are the frame: every one of them is half a bit or a whole one
      // because every bit put them there, so a reading that stops in the middle
      // of a stretch has not read a frame - it has found a few runs of a UART's
      // that happen to be one and two bit times long and given up where the
      // three-bit one starts. That check is what keeps ordinary serial traffic
      // out, and it costs nothing on a real frame.
      if (last < r1 && n < MAN_MAX_BITS)
        continue;

      if (n >= MAN_MAX_BITS)
        out->truncated = true;

      int nbytes = (n + 7) / 8;

      if (out->count + nbytes > LOGIC_MAX_BYTES)
      {
        out->truncated = true;
        break;
      }

      ManFrame *f = &g_man.frame[g_man.frames];

      f->bits = (uint8_t)n;
      f->value = (n <= 32) ? value : 0;
      f->first = (uint8_t)out->count;
      f->count = (uint8_t)nbytes;
      f->viol = (uint8_t)((viol > 255) ? 255 : viol);
      f->viol_bit = (uint8_t)viol_bit;

      for (int i = 0; i < nbytes; i++)
      {
        out->bytes[out->count] = bytes[i];
        out->pos[out->count] = bpos[i];
        out->end[out->count] = bend[i];
        out->count++;
      }

      // BOTH run lengths, and no broken bit - or this frame proves nothing.
      //
      // A stretch whose runs are all one length is a square wave, and a
      // square wave is Manchester of identical bits, and a stream of 0x55 out
      // of a UART, and a clock: the same samples, with nothing in them to
      // choose between the readings.
      //
      // And a frame with a bit that has no middle is still REPORTED - that is
      // the whole point of tracking the phase rather than chasing edges - but
      // it is not what a record gets claimed on. "Manchester with a fault in
      // it" and "something else that fits Manchester badly" look identical
      // from one frame; what tells them apart is a clean frame elsewhere in
      // the record, and this is what waits for one.
      if (shorts > 0 && longs > 0 && 0 == viol)
        g_man.sure = true;

      g_man.frames++;
      g_man.bits += n;
      g_man.viol += viol;
    }


    if (MAN_POL_AUTO != g_man_pol || 1 == pass)
      break;

    if (0 == g_man.frames || 0xAA != out->bytes[0])
      break;

    g_man_inverted = !g_man_inverted;
    g_man.auto_inv = true;
  }

  if (0 == out->count)
    return 0;

  g_man.rate = rate;
  g_man.inverted = g_man_inverted;

  out->rate = rate;
  out->idle_high = (0 != scratch->lvl[runs - 1]);
  out->burst_start = true;   // a frame is found from the idle around it

  // Nothing here is claimed unless some frame in the record held runs of
  // BOTH lengths. That is the actual signature - two lengths in a 1:2 ratio -
  // and without it the record is a square wave, which every one of half a
  // dozen readings fits equally well. Being told the rate closes the
  // factor-of-two question and does not close this one: a square wave is a
  // square wave whatever rate it is read at.
  (void)both;
  out->ambiguous = !g_man.sure;

  char body[16];

  if (1 == g_man.frames && g_man.frame[0].bits <= 32)
    snprintf(body, sizeof(body), "%db %lX", g_man.frame[0].bits,
        (unsigned long)g_man.frame[0].value);
  else if (1 == g_man.frames)
    snprintf(body, sizeof(body), "%db", g_man.frame[0].bits);
  else
    snprintf(body, sizeof(body), "%d frames", g_man.frames);

  // A bit with no transition in its middle is an error in the TRAFFIC, not a
  // reason to doubt the protocol - the same line already drawn for a MIDI
  // byte nobody should have sent. The frame is reported, the violation is
  // counted, and the bit is named: "somewhere in here" is not an answer.
  out->errors = g_man.viol;

  char bad[10] = "";

  if (g_man.viol > 0)
    snprintf(bad, sizeof(bad), " !b%d", g_man.frame[0].viol ?
        g_man.frame[0].viol_bit : 0);

  snprintf(out->info, sizeof(out->info), "MAN %s%d %s%s%s%s",
      g_man.told ? "" : "~", rate, body,
      g_man_inverted ? " inv" : "", g_man.auto_inv ? "?" : "", bad);

  return out->count;
}

//-----------------------------------------------------------------------------
// The frame's own reading, written once across the bytes it packed into. The
// bytes are a packing artefact - a 14-bit RC5 frame is not two bytes of
// anything - so the bit count travels with the value.
void manchester_byte_label(const ManAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  for (int i = 0; i < a->frames && i < MAN_MAX_FRAMES; i++)
  {
    const ManFrame *f = &a->frame[i];

    if (idx < f->first || idx >= f->first + f->count)
      continue;

    if (f->bits <= 32)
      snprintf(buf, size, "%db %lX%s", f->bits, (unsigned long)f->value,
          f->viol ? "!" : "");
    else
      snprintf(buf, size, "%db%s", f->bits, f->viol ? "!" : "");

    return;
  }
}

//-----------------------------------------------------------------------------
// Which bits of the frame this byte carries. The frame is a bit count and not
// a byte count, so the last byte usually holds fewer than eight of them - and
// saying so is the only way the hex above it reads.
void manchester_field_label(const ManAnalysis *a, int idx, uint8_t v,
    char *buf, int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  for (int i = 0; i < a->frames && i < MAN_MAX_FRAMES; i++)
  {
    const ManFrame *f = &a->frame[i];

    if (idx < f->first || idx >= f->first + f->count)
      continue;

    int k = idx - f->first;
    int hi = f->bits - 1 - k * 8;
    int lo = hi - 7;

    if (lo < 0)
      lo = 0;

    snprintf(buf, size, "b%d-%d", hi, lo);

    return;
  }
}

//-----------------------------------------------------------------------------
// One frame is one number and is shown as one
void manchester_group_at(const ManAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  for (int i = 0; i < a->frames && i < MAN_MAX_FRAMES; i++)
  {
    const ManFrame *f = &a->frame[i];

    if (idx >= f->first && idx < f->first + f->count)
    {
      *start = f->first;
      *len = f->count;

      return;
    }
  }
}
