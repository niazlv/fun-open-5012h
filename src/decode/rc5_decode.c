/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RC5, RC5X and RC6 - the Philips infrared remotes, and the ones in that
 * family that actually ride on Manchester. Their neighbours do not: Sony
 * SIRC, Samsung and RCA are pulse-coded like NEC and have nothing to do with
 * bi-phase at all.
 *
 * RC5 is the plain case and the reason the generic Manchester decoder exists:
 * fourteen bits, 1.778 ms each, a transition in the middle of every one.
 *
 *   S1  always 1 - and it is what the frame is found from
 *   S2  1 for RC5; 0 makes it RC5X and becomes the seventh command bit
 *   T   toggle, flipped on every fresh press and NOT on a repeat
 *   A   five bits of address
 *   C   six bits of command
 *
 * RC6 keeps the bi-phase and puts a leader in front of it - 2.666 ms of
 * carrier and 0.889 ms of silence - then twenty-one bits at 0.889 ms each:
 * a start bit, three mode bits, a toggle bit AT DOUBLE WIDTH, and eight bits
 * each of address and command. That double-width bit is why RC6 cannot be
 * read by a decoder that insists every run is half a bit or a whole one: on
 * the trailer they are two halves and four, and a generic reader stops there.
 *
 * The two also disagree about which way round a bit reads, which is the one
 * thing bi-phase never carries in the waveform:
 *
 *   RC5  a one is silence then carrier - the mid-bit edge goes AWAY from idle
 *   RC6  a one is carrier then silence - the mid-bit edge goes TOWARDS it
 *
 * Neither is guessed here. A demodulator rests with no carrier, so the record
 * says which level idle is, and the convention follows from that and from
 * which of the two protocols the frame's shape identified. Probe the LED
 * drive instead of the receiver and every level inverts - and so does the
 * idle, so the reading comes out the same.
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
// 1.778 ms by the standard. The window is for the clones, which are many.
#define RC5_BIT_MIN_NS   1400000
#define RC5_BIT_MAX_NS   2200000
#define RC5_BITS              14

// RC6 mode 0: a 2.666 ms mark, a 0.889 ms space, then 0.889 ms a bit
#define RC6_BIT_MIN_NS    700000
#define RC6_BIT_MAX_NS   1100000
#define RC6_LEAD_MARK_X10     30   // in bit times, x10: 2.666 / 0.889
#define RC6_BITS              21
#define RC6_TRAILER            4   // ...and this one is twice as wide

// How far a run may sit from half a bit or a whole one
#define RC5_TOL_PCT           32

// A frame stands in silence. RC5 repeats every 114 ms and RC6 every 107,
// so there is a great deal of it either side.
#define RC5_QUIET_BITS         3

#define RC5_MAX_BITS          24

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_rc5   (g_logic_analysis.rc5)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const Rc5Analysis *rc5_analysis(void)
{
  return &g_rc5;
}

//-----------------------------------------------------------------------------
static bool rc5_near(int64_t len_x256, int64_t want, int tol_pct)
{
  int64_t d = len_x256 - want;

  if (d < 0)
    d = -d;

  return (d * 100 <= want * tol_pct);
}

//-----------------------------------------------------------------------------
// The run index of the next edge `span_x256` further on, or -1 when there is
// no transition where one has to be. Stepping edge to edge rather than along
// a grid is the whole point of a self-clocking code, and it is what lets a
// double-width trailer bit be asked for by simply asking for a longer step.
//
// The window is a quarter of a BIT and not a quarter of the step. Those are
// the same thing while the step is one bit and are not the same thing at all
// for the four-and-a-half-bit jump over an RC6 leader: a window of a bit and
// a tenth there accepts the boundary edge half a bit early and reads the
// whole frame one edge out of place.
static int rc5_step(const LogicScratch *s, int runs, int e, int64_t span_x256,
    int64_t tol_x256)
{
  int64_t target = (int64_t)s->pos[e] * 256 + span_x256;
  int j = e + 1;

  while (j < runs && (int64_t)s->pos[j] * 256 < target - tol_x256)
    j++;

  if (j >= runs || (int64_t)s->pos[j] * 256 > target + tol_x256)
    return -1;

  return j;
}

//-----------------------------------------------------------------------------
// Read `nbits` bi-phase bits from the mid-bit edge at run `e0`. `wide` is the
// index of a bit that is twice as long as the others, or -1. `one_is_idle`
// says which way round a bit reads, which the caller knows and the waveform
// never will.
//
// Returns the number of bits read; the value is MSB first.
static int rc5_read(const LogicScratch *s, int runs, int64_t bit_x256, int e0,
    int nbits, int wide, int idle, bool one_is_idle, uint32_t *value,
    int *last)
{
  int e = e0;
  int n = 0;

  *value = 0;
  *last = e0;

  while (n < nbits && n < RC5_MAX_BITS)
  {
    int lvl = (0 != s->lvl[e]) ? 1 : 0;
    int bit = one_is_idle ? (lvl == idle) : (lvl != idle);

    *value = (*value << 1) | (uint32_t)bit;
    *last = e;
    n++;

    if (n >= nbits)
      break;

    // The trailer bit is twice as wide, so the step ONTO its middle and the
    // step OFF it are each one and a half bits - half of the bit before plus
    // half of the double one, and the other way round leaving it.
    int64_t step = bit_x256;

    if (n == wide || n - 1 == wide)
      step = bit_x256 * 3 / 2;

    int j = rc5_step(s, runs, e, step, bit_x256 / 4);

    if (j < 0)
      break;

    e = j;
  }

  return n;
}

//-----------------------------------------------------------------------------
// Is every run from r to r1 half a bit or a whole one? `wide` allows the two
// runs of a double-width bit to be twice that again.
// The run AT r1 is excluded: it is the second half of the last bit, and when
// the carrier stops there it merges into however much silence follows. That
// length is a fact about the gap before the next frame, not about this one -
// and a frame whose last bit happens to end at the idle level would otherwise
// be turned down for it.
static bool rc5_runs_fit(const LogicScratch *s, int r, int r1,
    int64_t bit_x256, bool allow_double)
{
  for (int k = r; k < r1; k++)
  {
    int64_t l = (int64_t)s->len[k] * 256;

    if (rc5_near(l, bit_x256 / 2, RC5_TOL_PCT) ||
        rc5_near(l, bit_x256, RC5_TOL_PCT))
      continue;

    if (allow_double && (rc5_near(l, bit_x256 * 2, RC5_TOL_PCT) ||
        rc5_near(l, bit_x256 * 3 / 2, RC5_TOL_PCT)))
      continue;

    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// Everything after the bits: what the frame says, and how it lands on screen
static int rc5_emit(LogicResult *out, uint32_t value, int nbits, int p0,
    int p1)
{
  int nbytes = (nbits + 7) / 8;

  if (nbytes > LOGIC_MAX_BYTES)
    return 0;

  for (int i = 0; i < nbytes; i++)
  {
    int shift = nbits - 8 * (i + 1);
    uint32_t v = (shift >= 0) ? (value >> shift) : (value << -shift);

    out->bytes[i] = (uint8_t)(v & 0xFF);
    out->pos[i] = p0 + (p1 - p0) * (8 * i) / nbits;
    out->end[i] = p0 + (p1 - p0) * (8 * (i + 1) > nbits ? nbits : 8 * (i + 1))
        / nbits;
  }

  out->count = nbytes;

  return nbytes;
}

//-----------------------------------------------------------------------------
int rc5_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  memset(&g_rc5, 0, sizeof(g_rc5));
  out->proto = PROTO_RC5;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 8 || period_ns <= 0)
    return 0;

  int idle = (0 != scratch->lvl[runs - 1]) ? 1 : 0;

  out->idle_high = (0 != idle);

  // ---- RC6 first: it has a leader, and a leader is the more specific claim
  for (int r = 1; r + 2 < runs; r++)
  {
    if (scratch->lvl[r] == idle)
      continue;                       // the leader is carrier, not silence

    // 2.666 ms of mark and 0.889 of space: three bit times and one
    int64_t mark = (int64_t)scratch->len[r] * 256;
    int64_t space = (int64_t)scratch->len[r + 1] * 256;
    int64_t bit_x256 = space;          // the space IS one bit time

    if (!rc5_near(mark * 10, bit_x256 * RC6_LEAD_MARK_X10, RC5_TOL_PCT))
      continue;

    int bit_ns = (int)(bit_x256 * period_ns / 256);

    if (bit_ns < RC6_BIT_MIN_NS || bit_ns > RC6_BIT_MAX_NS)
      continue;

    // From the start of the leader's mark: three bit times of it, one of
    // space, then the start bit - whose middle is half a bit into that. Four
    // and a half bit times in all, and getting this wrong is not subtle: the
    // frame simply does not read.
    int e0 = rc5_step(scratch, runs, r, bit_x256 * 9 / 2,
        bit_x256 / 4);

    if (e0 < 0)
      continue;

    uint32_t value = 0;
    int last = e0;
    int n = rc5_read(scratch, runs, bit_x256, e0, RC6_BITS, RC6_TRAILER, idle,
        true, &value, &last);

    if (n < RC6_BITS)
      continue;

    if (!rc5_runs_fit(scratch, r + 1, last, bit_x256, true))
      continue;

    if (0 == (value >> (RC6_BITS - 1)))
      continue;                       // the start bit is a one or this is not RC6

    g_rc5.kind = RC5_KIND_RC6;
    g_rc5.bit_ns = bit_ns;
    g_rc5.mode = (uint8_t)((value >> 17) & 7);
    g_rc5.toggle = (0 != ((value >> 16) & 1));
    g_rc5.addr = (uint8_t)((value >> 8) & 0xFF);
    g_rc5.cmd = (uint8_t)(value & 0xFF);
    g_rc5.bits = RC6_BITS;

    if (!rc5_emit(out, value, RC6_BITS, scratch->pos[r],
        scratch->pos[last] + (int)(bit_x256 / 512)))
      return 0;

    out->rate = bit_ns;
    out->burst_start = true;

    snprintf(out->info, sizeof(out->info), "RC6 m%d a%d c%d t%d",
        g_rc5.mode, g_rc5.addr, g_rc5.cmd, g_rc5.toggle ? 1 : 0);

    return out->count;
  }

  // ---- RC5: no leader at all. What starts a frame is the first carrier
  // after a long silence, and that edge is the middle of S1 - because S1 is
  // a one, a one is silence then carrier, and the silence half of it is
  // indistinguishable from the silence in front of the frame.
  // Not bounded by the bit count: how many RUNS fourteen bits take depends
  // on the data - a bit followed by an equal one splits into two runs and an
  // unequal one does not - so anything from fifteen to twenty-nine of them.
  // Demanding the worst case turned away every frame that was not all ones.
  for (int r = 1; r + 2 < runs; r++)
  {
    if (scratch->lvl[r] == idle)
      continue;

    // Half a bit, from this run or the next: a one followed by a one gives a
    // half-bit run here, a one followed by a zero gives a whole bit
    int64_t l = (int64_t)scratch->len[r] * 256;
    int64_t bit_x256 = 0;

    for (int k = 1; k <= 2 && !bit_x256; k++)
    {
      int64_t cand = (1 == k) ? l * 2 : l;
      int ns = (int)(cand * period_ns / 256);

      if (ns >= RC5_BIT_MIN_NS && ns <= RC5_BIT_MAX_NS)
        bit_x256 = cand;
    }

    if (!bit_x256)
      continue;

    // Silence in front of it, or the record's own start
    if (r > 1 && (int64_t)scratch->len[r - 1] * 256 <
        bit_x256 * RC5_QUIET_BITS)
      continue;

    uint32_t value = 0;
    int last = r;
    int n = rc5_read(scratch, runs, bit_x256, r, RC5_BITS, -1, idle, false,
        &value, &last);

    if (n < RC5_BITS)
      continue;

    if (!rc5_runs_fit(scratch, r, last, bit_x256, false))
      continue;

    // ...and silence behind it
    if (last + 1 < runs - 1 && (int64_t)scratch->len[last + 1] * 256 <
        bit_x256 * RC5_QUIET_BITS)
      continue;

    bool field = (0 != ((value >> 12) & 1));

    g_rc5.kind = field ? RC5_KIND_RC5 : RC5_KIND_RC5X;
    g_rc5.bit_ns = (int)(bit_x256 * period_ns / 256);
    g_rc5.field = field;
    g_rc5.toggle = (0 != ((value >> 11) & 1));
    g_rc5.addr = (uint8_t)((value >> 6) & 0x1F);
    // RC5X: the second start bit is the INVERTED seventh command bit, which
    // is how a six-bit field came to hold 128 commands
    g_rc5.cmd = (uint8_t)((value & 0x3F) | (field ? 0 : 0x40));
    g_rc5.bits = RC5_BITS;

    if (!rc5_emit(out, value, RC5_BITS, scratch->pos[r] -
        (int)(bit_x256 / 512), scratch->pos[last] + (int)(bit_x256 / 512)))
      return 0;

    out->rate = g_rc5.bit_ns;
    out->burst_start = true;

    snprintf(out->info, sizeof(out->info), "%s a%d c%d t%d",
        field ? "RC5" : "RC5X", g_rc5.addr, g_rc5.cmd,
        g_rc5.toggle ? 1 : 0);

    return out->count;
  }

  return 0;
}

//-----------------------------------------------------------------------------
// What the frame says, written once across the bytes its bits packed into.
// The bytes are a packing artefact - fourteen bits are not two bytes of
// anything - so the reading travels as a whole.
void rc5_byte_label(const Rc5Analysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)idx;
  (void)v;

  buf[0] = 0;

  snprintf(buf, size, "a%d c%d t%d", a->addr, a->cmd, a->toggle ? 1 : 0);
}

//-----------------------------------------------------------------------------
// Which field of the frame this byte straddles. They do not land on byte
// boundaries - five bits of address never will - so this says which bits are
// in it rather than pretending the byte is a field.
void rc5_field_label(const Rc5Analysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0)
    return;

  int hi = a->bits - 1 - idx * 8;
  int lo = hi - 7;

  if (hi < 0)
    return;

  if (lo < 0)
    lo = 0;

  snprintf(buf, size, "b%d-%d", hi, lo);
}

//-----------------------------------------------------------------------------
// One frame is one message and is shown as one
void rc5_group_at(const Rc5Analysis *a, int idx, int *start, int *len)
{
  (void)idx;

  *start = 0;
  *len = (a->bits + 7) / 8;

  if (*len < 1)
    *len = 1;
}
