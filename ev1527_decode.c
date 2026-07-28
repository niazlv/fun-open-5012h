/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * EV1527 - and the HS1527, RT1527, SC2262 family with it: the encoder inside
 * every 433 MHz gate remote, doorbell button, PIR sensor and cheap alarm
 * contact. One chip, one pin, no protocol negotiation of any kind.
 *
 * Frame: a sync pulse, then 24 bits. Twenty of them are the address burned
 * into the part at the factory, four are the buttons. The frame repeats for
 * as long as the button is held.
 *
 * Everything is measured in T, the unit the encoder's own resistor sets -
 * anywhere from about a tenth of a millisecond to a millisecond, and no two
 * remotes agree:
 *
 *   sync  1T high, 31T low
 *   '0'   1T high, 3T low
 *   '1'   3T high, 1T low
 *
 * The bus rests LOW: no carrier means no signal, so the gaps are the idle
 * level and the pulses are what is sent. That is upside down from every other
 * protocol here and is worth saying out loud.
 *
 * There is NO checksum. What identifies the protocol is its shape, and the
 * shape is strong enough to carry it: the sync gives T from one 1:31 ratio,
 * and then EVERY bit has to be exactly 4T long - only the split moves. That
 * is 24 independent checks of a constant no other protocol here holds.
 *
 * Which is also how it is told apart from NEC, the one signal it is likely to
 * be confused with, because both open with a pulse and a long gap:
 *
 *   - NEC's leader is a long MARK and a half-length gap, 2:1, with the long
 *     part on the pulse. EV1527's sync is a 1T pulse and a 31T gap, 1:31,
 *     with the long part on the gap. Opposite sides of the edge.
 *   - NEC holds its MARK constant at 560 us and puts the value in the gap, so
 *     its bit periods are 1120 or 2250 us. EV1527 holds its PERIOD constant
 *     at 4T and puts the value in the split. One of them has a constant mark
 *     and the other a constant period, and neither can fake the other's.
 *
 * Absolute durations cannot separate them - T runs from 100 to 400 us in the
 * wild, so 31T covers 3 to 12 ms and NEC's 9 ms leader sits inside that. Only
 * the shape separates them, and it separates them completely.
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
#define EV_SYNC_TICKS      32    // the sync pulse and its gap together
#define EV_BIT_TICKS        4    // every bit, whatever its value
#define EV_BITS            24    // 20 address, 4 buttons

// The unit time an EV1527's oscillator resistor sets. The window is wider
// than anything sold, because the point is to reject signals that are not
// this shape rather than remotes that are unusual.
#define EV_T_MIN_NS      50000
#define EV_T_MAX_NS    2000000

// Below this a tenth of T is not resolvable and the 1T/3T split stops being
// a decision
#define EV_T_MIN_SAMPLES    3

// In tenths of T. The period is the invariant and is held tightest; the two
// pulse widths are wide because a superregenerative receiver's AGC stretches
// and clips edges, and because the windows are far enough apart to afford it.
// What they must NOT do is meet: a pulse of 2T is not a bit of either kind,
// and a decoder that rounded it to the nearer one would read noise as data.
#define EV_PERIOD_TOL_X10  12    // 4T +- 1.2T
#define EV_ZERO_MIN_X10     4
#define EV_ZERO_MAX_X10    18
#define EV_ONE_MIN_X10     21
#define EV_ONE_MAX_X10     37

// The sync's own pulse is one T out of the thirty-two the ratio was measured
// over, so an error in it barely moves T at all. Loose on purpose: the 24
// bits behind it are what actually has to hold up.
#define EV_SYNC_HI_MIN_X10  3
#define EV_SYNC_HI_MAX_X10 25

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_ev    (g_logic_analysis.ev)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const Ev1527Analysis *ev1527_analysis(void)
{
  return &g_ev;
}

//-----------------------------------------------------------------------------
// The 24 bits go out most significant first and pack into three bytes, which
// is why the hex dump reads the address and the buttons straight off: an
// address of 0x5A3C7 with button 1 held is "5A 3C 71". The byte boundaries
// fall in odd places, and the bytes are still what the wire carried.
void ev1527_byte_label(const Ev1527Analysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  const Ev1527Frame *f = &a->frame[(idx / 3) % EV1527_MAX_FRAMES];

  snprintf(buf, size, "%05lX K%X", (unsigned long)f->addr, f->key);
}

//-----------------------------------------------------------------------------
// Which bits of the frame this byte holds. The address does not end on a byte
// boundary - it is twenty bits - so the last byte is four bits of address and
// the four buttons, and saying so is the only way the hex above it reads.
void ev1527_field_label(const Ev1527Analysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  static const char *const name[3] = { "A19-12", "A11-4", "A3-0+K" };

  (void)a;
  (void)v;

  buf[0] = 0;

  if (idx < 0)
    return;

  snprintf(buf, size, "%s", name[idx % 3]);
}

//-----------------------------------------------------------------------------
// One frame is one thing: an address and a button, written once across the
// three bytes it took to send them. Splitting a twenty-bit address over two
// and a half bytes and leaving the reader to reassemble it is exactly what
// the grouping exists to avoid.
void ev1527_group_at(const Ev1527Analysis *a, int idx, int *start, int *len)
{
  (void)a;

  *start = idx;
  *len = 1;

  if (idx < 0)
    return;

  *start = idx - idx % 3;
  *len = 3;
}

//-----------------------------------------------------------------------------
// One pass over the record at a given pulse level. Returns the number of
// whole frames found; out and a may be NULL for a counting-only pass.
static int ev1527_scan(const LogicScratch *s, int runs, int period_ns, int hi,
    LogicResult *out, Ev1527Analysis *a)
{
  int found = 0;

  // From run 1: the record's first run began before the record did, so its
  // length is a fact about the buffer rather than about the signal
  for (int r = 1; r + 2 * EV_BITS + 1 < runs; r++)
  {
    if (s->lvl[r] != hi)
      continue;

    // The sync: a pulse and a gap that together are 32 T, which is where T
    // comes from. Nothing else in the frame is 32 of anything.
    int64_t sync = (int64_t)s->len[r] + s->len[r + 1];
    int64_t t_x256 = sync * 256 / EV_SYNC_TICKS;

    if (t_x256 < EV_T_MIN_SAMPLES * 256)
      continue;

    int t_ns = (int)(t_x256 * period_ns / 256);

    if (t_ns < EV_T_MIN_NS || t_ns > EV_T_MAX_NS)
      continue;

    int64_t hi10 = (int64_t)s->len[r] * 10 * 256 / t_x256;

    if (hi10 < EV_SYNC_HI_MIN_X10 || hi10 > EV_SYNC_HI_MAX_X10)
      continue;

    // ...and then 24 bits, every one of them exactly four T long. That
    // constant is the whole identification: the value moves the split, it
    // never moves the period.
    uint32_t value = 0;
    bool ok = true;

    for (int k = 0; k < EV_BITS && ok; k++)
    {
      int rh = r + 2 + 2 * k;
      int64_t h = s->len[rh];
      int64_t l = s->len[rh + 1];
      int64_t h10 = h * 10 * 256 / t_x256;
      int64_t p10 = (h + l) * 10 * 256 / t_x256;
      // The last bit's gap is not bounded from above. Its VALUE is in the
      // pulse; the gap after it is only a gap, and if the remote stopped
      // transmitting there - the button let go, or the record simply ended -
      // that gap runs on into the idle line for as long as it likes. Holding
      // it to 4T would throw away the last frame of every burst, which on a
      // scope is a frame you were probably looking at.
      bool last = (EV_BITS - 1 == k);

      if (p10 < EV_BIT_TICKS * 10 - EV_PERIOD_TOL_X10 ||
          (!last && p10 > EV_BIT_TICKS * 10 + EV_PERIOD_TOL_X10))
      {
        ok = false;
        break;
      }

      if (h10 >= EV_ZERO_MIN_X10 && h10 <= EV_ZERO_MAX_X10)
        value <<= 1;                       // 1T of pulse: a zero
      else if (h10 >= EV_ONE_MIN_X10 && h10 <= EV_ONE_MAX_X10)
        value = (value << 1) | 1u;         // 3T of pulse: a one
      else
        ok = false;
    }

    if (!ok)
      continue;

    found++;

    if (out && a)
    {
      if (out->count + 3 > LOGIC_MAX_BYTES || a->frames >= EV1527_MAX_FRAMES)
      {
        out->truncated = true;
        break;
      }

      Ev1527Frame *f = &a->frame[a->frames];

      f->addr = (value >> 4) & 0xFFFFFu;
      f->key = (uint8_t)(value & 0x0F);

      for (int i = 0; i < 3; i++)
      {
        int rh = r + 2 + 2 * (8 * i);      // the byte's first bit

        out->bytes[out->count] = (uint8_t)(value >> (16 - 8 * i));
        out->pos[out->count] = s->pos[rh];
        out->end[out->count] = s->pos[rh + 15] + s->len[rh + 15];
        out->count++;
      }

      a->frames++;
      a->t_ns = t_ns;
    }

    // Past the frame; the loop's own step takes it to the run after
    r += 2 * EV_BITS + 1;
  }

  return found;
}

//-----------------------------------------------------------------------------
int ev1527_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_EV1527;

  runs = logic_runs(data, size, offset, scratch, &mid);

  // One frame is a sync pulse, its gap and two runs per bit, and the record
  // has to open somewhere before it: that is 3 + 2 x 24 runs at the very
  // least. Asking for more would throw away the single-frame record, which
  // is what a scope catches when the button is tapped rather than held.
  if (runs < 2 * EV_BITS + 3 || period_ns <= 0)
    return 0;

  // The encoder rests low and sends pulses high. A receiver module with an
  // inverting output is somebody's design, and costs one more pass.
  int best = 0, best_hi = 1;

  for (int p = 0; p < 2; p++)
  {
    int n = ev1527_scan(scratch, runs, period_ns, p, NULL, NULL);

    if (n > best)
    {
      best = n;
      best_hi = p;
    }
  }

  if (0 == best)
    return 0;

  memset(&g_ev, 0, sizeof(g_ev));
  ev1527_scan(scratch, runs, period_ns, best_hi, out, &g_ev);

  if (0 == out->count)
    return 0;

  out->rate = g_ev.t_ns;
  out->idle_high = (0 == best_hi);   // the pulses are the OTHER level
  out->burst_start = true;           // a frame is found from its own sync

  // A held button sends the same frame over and over, and there is no
  // checksum anywhere in the protocol - so repeats agreeing is the only
  // corroboration this signal has to offer. Disagreement is not an error,
  // though: a record long enough to hold several frames is long enough for a
  // finger to have moved to another button, and the panel shows every frame's
  // own address and key. The header just stops claiming one reading for all
  // of them.
  g_ev.agree = true;

  for (int i = 1; i < g_ev.frames; i++)
  {
    if (g_ev.frame[i].addr != g_ev.frame[0].addr ||
        g_ev.frame[i].key != g_ev.frame[0].key)
      g_ev.agree = false;
  }

  // Twenty-four bits of constant period is a statement no other protocol here
  // makes, so one whole frame settles it
  out->ambiguous = false;

  int us = g_ev.t_ns / 1000;

  if (g_ev.agree)
  {
    if (g_ev.frames > 1)
      snprintf(out->info, sizeof(out->info), "EV1527 %05lX K%X x%d %dus",
          (unsigned long)g_ev.frame[0].addr, g_ev.frame[0].key, g_ev.frames,
          us);
    else
      snprintf(out->info, sizeof(out->info), "EV1527 %05lX K%X %dus",
          (unsigned long)g_ev.frame[0].addr, g_ev.frame[0].key, us);
  }
  else
  {
    snprintf(out->info, sizeof(out->info), "EV1527 %d frames %dus",
        g_ev.frames, us);
  }

  return out->count;
}
