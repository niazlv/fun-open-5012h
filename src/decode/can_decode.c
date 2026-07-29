/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CAN decoder: classic CAN 2.0A/2.0B up to 1 Mbit/s, and CAN FD (ISO
 * 11898-1:2015) with its data phase read at its own bit rate.
 *
 * The bus is a differential pair, but a scope probe sits on one wire of it,
 * and one wire is all this needs. CAN_L and a transceiver's TX line rest high
 * and are pulled low to send; CAN_H does the opposite. So the polarity is not
 * assumed - the RECESSIVE level is whichever one the line rests at, and that
 * is read out of the record. Probe either wire.
 *
 * Bits are NRZ with stuffing: after five identical bits the transmitter
 * inserts one of the opposite value, which guarantees an edge at least every
 * five bit times and lets the decoder resynchronise on every edge instead of
 * free-running a sample clock across a 130-bit frame. So the bits are read
 * out of the RUNS - each run is as many bits as its length divides into -
 * and the phase can never drift further than one run.
 *
 * A classic frame:
 *   SOF | 11-bit id | RTR | IDE | r0 | DLC(4) | data | CRC(15) | del | ACK |
 *   del | EOF(7) | IFS(3)
 * and with IDE recessive, 18 more identifier bits and two more reserved ones
 * in the middle of it.
 *
 * An FD frame says so in the bit classic CAN keeps dominant (FDF, which is r0
 * on a base frame and r1 on an extended one), and then differs in four ways
 * that all have to be handled together:
 *
 *   - BRS. If it is recessive the rest of the frame up to the CRC delimiter
 *     runs at a SECOND bit rate, and the switch happens at the SAMPLE POINT
 *     of the BRS bit rather than at its edge - so that one bit is part
 *     arbitration-timed and part data-timed. Nothing on the wire says where
 *     the sample point is; see can_fd_body() for what is done about it.
 *   - The length code is a table above eight: 12, 16, 20, 24, 32, 48, 64.
 *   - The CRC is 17 bits up to 16 data bytes and 21 above, over a DIFFERENT
 *     stream: the bits AS TRANSMITTED, stuff bits and all, with the register
 *     starting at one rather than zero. Classic CAN CRCs the destuffed bits
 *     from a zero register. Getting this backwards decodes nothing.
 *   - The CRC field carries a stuff count first and is stuffed at FIXED
 *     positions - one bit before the count and one after every four - which
 *     the CRC does not see.
 *
 * Non-ISO CAN FD (the 2012 Bosch release, no stuff count, CRC register from
 * zero) is not read. It is a decade obsolete and telling it apart costs a
 * second parse of every frame.
 *
 * The whole thing is confirmed by its own CRC, which is why this decoder can
 * run ahead of the generic ones without stealing their records: a frame that
 * checks out is proof, and a record with no such frame in it is refused
 * outright. That is also what settles the two unknowns CAN FD adds - the data
 * rate and where the sample point sits - by trying the few candidates the
 * record's own runs allow and keeping the one the CRC agrees with.
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
#define CAN_MIN_SAMPLES_PER_BIT  3
#define CAN_MIN_RECORD_BITS     48   // shorter than the shortest possible frame

// Between two frames the line is recessive for the CRC delimiter, the ACK
// delimiter, seven bits of EOF and three of interframe space - eleven at the
// very least. Inside a frame stuffing caps a run at five. So a recessive run
// of six to ten bit times cannot happen, and one that does settles the
// question before a single frame is parsed. This holds for CAN FD too: its
// data phase runs FASTER, so its runs are shorter than an arbitration bit,
// never longer.
#define CAN_GAP_MIN_BITS        11
#define CAN_SOF_IDLE_BITS        7   // EOF alone; anchors the start of a frame

// A dominant run is capped at five by stuffing too, except for error flags -
// six, and up to twelve where they overlap
#define CAN_DOM_MAX_BITS        12

// The CRC-15 of classic CAN, and the two CAN FD added. All three are fed
// MSB-first; the FD pair start from a register with its top bit set.
#define CAN_CRC15_POLY          0x4599u
#define CAN_CRC17_POLY          0x1685Bu
#define CAN_CRC21_POLY          0x102899u

/*- Types -------------------------------------------------------------------*/
// Bit reader over the run-length split, with the destuffing state folded in.
// Reading bits out of runs rather than off a sample clock is what keeps a
// 130-bit frame in phase: every edge re-anchors it. `off` is where inside the
// current run the next bit starts, in samples x256, and it is an accumulator
// rather than a multiple of the bit time because CAN FD changes the bit time
// in the MIDDLE of a run.
typedef struct
{
  const LogicScratch *s;
  int   runs;
  int   bit_x256;      // the bit time in force, in samples, x256
  int   arb_x256;      // ...and the arbitration one, which the frame ends at
  int   rec_lvl;       // the SAMPLE level that means recessive
  int   r, k, n, off;  // current run, bits taken from it, bits in it, offset
  int   pos, end;      // sample span of the bit just returned
  bool  ended;
  int   last, same;    // destuffing: last bit value and how many in a row
  bool  stuff;         // stuffing is in force (SOF through the data field)
  bool  stuff_err;
  bool  fd;            // read CAN FD frames rather than refusing them
  // The FD CRCs run over the bits AS TRANSMITTED, so they are fed here, at
  // the raw read, and not by the parser. Which of the two it will be is not
  // known until the length code, so both run and the loser is dropped.
  bool     crc_on;
  uint32_t c17, c21;
  int      stuffs;     // dynamic stuff bits since the SOF: the stuff count
  int      last_raw;   // the last bit that went out on the wire
  int      fd_x256;    // the data phase's bit time, once a frame has fixed it
} CanRd;

// Where a frame's bytes go. Straight into the result, with the result's own
// count as the cursor and a rollback if the frame turns out not to be one -
// which is 650 bytes of staging buffer, and the copy that emptied it, not
// spent. A CAN FD frame is too big to hold twice on a part with 64K of
// tightly coupled memory and no room left in it.
//
// `out` is NULL through the whole rate search, where the bytes are not wanted
// and only the count is: `n` is what the frame WOULD have written, which is
// what the byte roles are laid out against afterwards.
typedef struct
{
  LogicResult *out;
  int          n;      // bytes this frame has put, whether they fit or not
} CanOut;

enum { CAN_NO = 0, CAN_OK, CAN_CUT };

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_can   (g_logic_analysis.can)

// Every rate anyone actually runs. 33.3k is the single-wire automotive one,
// 83.3k shows up on European vehicle buses, 800k and 1M on machinery.
static const int g_can_rates[] =
{
  10000, 20000, 33333, 50000, 62500, 83333, 100000, 125000,
  250000, 500000, 800000, 1000000,
};

// What CAN FD's data phase runs at, as a multiple of the arbitration rate.
// Expressed as a ratio and not as a rate because that is how a controller is
// configured - one prescaler, one divider - and because it needs no reference
// to the sample period to work out.
static const int g_can_fd_ratio[] = { 1, 2, 3, 4, 5, 8, 10, 16, 20 };

// The FD length codes above eight, which are a table and not a count
static const uint8_t g_can_fd_len[16] =
{
  0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// CRC-15/CAN, polynomial 0x4599, fed MSB-first over the DESTUFFED bits from
// the start of frame through the last data bit
static void can_crc_bit(uint16_t *crc, int bit)
{
  uint16_t inv = (uint16_t)(((*crc >> 14) & 1) ^ (bit & 1));

  *crc = (uint16_t)((*crc << 1) & 0x7FFF);

  if (inv)
    *crc ^= CAN_CRC15_POLY;
}

//-----------------------------------------------------------------------------
// CAN FD's pair, fed with every bit that went out on the wire - the stuff
// bits included, which is the change that closed classic CAN's one weakness:
// a pair of errors that turn an information bit into a stuff bit and back
// leaves the destuffed sequence the right LENGTH, and a CRC over destuffed
// bits cannot see it. Over stuffed ones it can.
static void can_fd_crc_bit(CanRd *rd, int bit)
{
  uint32_t inv;

  inv = ((rd->c17 >> 16) & 1u) ^ (uint32_t)(bit & 1);
  rd->c17 = ((rd->c17 << 1) & 0x1FFFFu) ^ (inv ? CAN_CRC17_POLY : 0u);

  inv = ((rd->c21 >> 20) & 1u) ^ (uint32_t)(bit & 1);
  rd->c21 = ((rd->c21 << 1) & 0x1FFFFFu) ^ (inv ? CAN_CRC21_POLY : 0u);
}

//-----------------------------------------------------------------------------
static int can_run_bits(const CanRd *rd, int r)
{
  return (int)(((int64_t)rd->s->len[r] * 256 + rd->bit_x256 / 2) / rd->bit_x256);
}

//-----------------------------------------------------------------------------
// One bit off the wire, stuffing not yet removed
static bool can_raw_bit(CanRd *rd, int *bit)
{
  while (rd->r < rd->runs && rd->k >= rd->n)
  {
    rd->r++;

    if (rd->r < rd->runs)
    {
      rd->k = 0;
      rd->off = 0;
      rd->n = can_run_bits(rd, rd->r);
    }
  }

  if (rd->r >= rd->runs)
  {
    rd->ended = true;
    return false;
  }

  *bit = (rd->s->lvl[rd->r] == rd->rec_lvl) ? 1 : 0;
  rd->pos = rd->s->pos[rd->r] + rd->off / 256;
  rd->off += rd->bit_x256;
  rd->end = rd->s->pos[rd->r] + rd->off / 256;
  rd->k++;
  rd->last_raw = *bit;

  if (rd->crc_on)
    can_fd_crc_bit(rd, *bit);

  return true;
}

//-----------------------------------------------------------------------------
// One bit of the frame: the stuff bit that follows five identical ones is
// consumed here and never reaches the parser. A stuff bit equal to the five
// before it is not a stuff bit at all - that is an error flag, or this is not
// a CAN frame.
static bool can_bit(CanRd *rd, int *bit)
{
  int b;

  if (!can_raw_bit(rd, &b))
    return false;

  if (rd->stuff && rd->same >= 5)
  {
    if (b == rd->last)
      rd->stuff_err = true;

    rd->last = b;
    rd->same = 1;
    rd->stuffs++;

    if (!can_raw_bit(rd, &b))
      return false;
  }

  if (b == rd->last)
  {
    rd->same++;
  }
  else
  {
    rd->last = b;
    rd->same = 1;
  }

  *bit = b;

  return true;
}

//-----------------------------------------------------------------------------
// A FIXED stuff bit: the inverse of the bit before it, in a place the
// standard puts one whether the stuff condition was met or not. It is not
// part of the CRC and it does not count towards the stuff count, which is why
// it is read here rather than through can_bit().
static int can_fixed_bit(CanRd *rd)
{
  bool on = rd->crc_on;
  int b, prev = rd->last_raw;

  rd->crc_on = false;

  if (!can_raw_bit(rd, &b))
  {
    rd->crc_on = on;
    return CAN_CUT;
  }

  rd->crc_on = on;

  return (b != prev) ? CAN_OK : CAN_NO;
}

//-----------------------------------------------------------------------------
static void can_put(CanOut *o, uint8_t v, int pos, int end)
{
  // Counted even when it does not fit: what the frame WOULD have written is
  // how the caller tells a record that is full from one that is not, and a
  // flag that outlives an attempt is a flag that poisons the next one
  o->n++;

  if (!o->out || o->out->count >= LOGIC_MAX_BYTES)
    return;

  int i = o->out->count++;

  o->out->bytes[i] = v;
  o->out->pos[i] = pos;
  o->out->end[i] = end;
}

//-----------------------------------------------------------------------------
// A field that is one number over several bytes - the identifier, and the FD
// CRC - divides the span it actually occupies on the trace between them
static void can_put_field(CanOut *o, uint32_t v, int bytes, int a, int b)
{
  for (int k = 0; k < bytes; k++)
    can_put(o, (uint8_t)(v >> (8 * (bytes - 1 - k))),
        a + (int)((int64_t)(b - a) * k / bytes),
        a + (int)((int64_t)(b - a) * (k + 1) / bytes));
}

//-----------------------------------------------------------------------------
// How long the BRS bit itself is. The bit rate switches at its SAMPLE POINT,
// so the bit is arbitration-timed up to there and data-timed after - and
// where "there" is depends on how the controllers on this bus are configured,
// which the wire does not say. 80% is the usual answer and this is what the
// nominal candidate assumes; can_fd_frame() covers the rest of the range by
// trying the neighbouring bit counts as well.
static int can_brs_x256(int arb_x256, int data_x256, int num, int den)
{
  return (int)(((int64_t)arb_x256 * num + (int64_t)data_x256 * (den - num)) /
      den);
}

//-----------------------------------------------------------------------------
// The data phase's bit time, measured off the record rather than searched
// for. Everything from the bit after BRS to the end of the frame is at the
// data rate, and the CRC field's fixed stuff bits - one every four bits, each
// the inverse of its neighbour - all but guarantee a run exactly one bit
// long in there. That shortest run is the unit; the rest of the runs then
// refine it, the same way the raw reader estimates a pulse width.
//
// Returns 0 when the window holds nothing usable.
static int can_fd_unit(const CanRd *rd, int from)
{
  const LogicScratch *s = rd->s;
  int m = 0, r, last;
  int64_t sum = 0;
  int bits = 0;

  for (r = from; r + 1 < rd->runs; r++)
  {
    // The frame ends at the first recessive stretch no run inside one can
    // reach: ACK delimiter, seven of EOF and three of interframe space
    if (s->lvl[r] == rd->rec_lvl &&
        (int64_t)s->len[r] * 256 >= (int64_t)rd->arb_x256 * 6)
      break;

    if (0 == m || s->len[r] < m)
      m = s->len[r];
  }

  last = r;

  if (m < 1)
    return 0;

  for (r = from; r < last; r++)
  {
    int q = (s->len[r] + m / 2) / m;

    if (q < 1 || q > 6)
      continue;   // stuffing caps a run at five; anything longer is not data

    sum += (int64_t)s->len[r] * 256;
    bits += q;
  }

  return (bits > 0) ? (int)(sum / bits) : m * 256;
}

//-----------------------------------------------------------------------------
// The end of an FD frame, read as RUNS rather than as bits. It has to be:
// the bit rate switches BACK at the sample point of the CRC delimiter, so
// that bit is neither one length nor the other, and counting bits across it
// would need the very number the wire does not carry. What follows is a
// shape, and a shape is enough - a recessive delimiter, one dominant bit if
// anybody acknowledged the frame, and then the eleven recessive bit times
// that no run inside a frame can reach.
static int can_fd_tail(CanRd *rd, CanFrame *f)
{
  const LogicScratch *s = rd->s;
  int arb = rd->arb_x256;
  int q, start;
  int64_t rec;

  if (s->lvl[rd->r] == rd->rec_lvl)
  {
    q = rd->r;                              // the delimiter shares the run
    start = s->pos[q] + rd->off / 256;      // the CRC sequence ended here
  }
  else
  {
    q = rd->r + 1;

    if (q >= rd->runs)
    {
      f->cut = true;
      return CAN_CUT;
    }

    if (s->lvl[q] != rd->rec_lvl)
      return CAN_NO;

    start = s->pos[q];
  }

  rec = (int64_t)(s->pos[q] + s->len[q] - start) * 256;

  if (rec <= 0)
    return CAN_NO;                          // no CRC delimiter at all

  if (rec >= (int64_t)arb * 9)
  {
    // Nothing pulled the acknowledge slot down, so the delimiter, the slot,
    // its delimiter, the end of frame and the interframe space are all one
    // recessive stretch. Which is the single most useful thing a scope can
    // say about a node that "does not work".
    f->ack = false;
    f->cut = (q + 1 >= rd->runs);
    rd->r = q;

    return CAN_OK;
  }

  if (q + 1 >= rd->runs)
  {
    f->cut = true;
    return CAN_CUT;   // the record ended before the acknowledge slot did
  }

  if ((int64_t)s->len[q + 1] * 256 > (int64_t)arb * 5 / 2)
    return CAN_NO;    // too long to be the one dominant bit of an ACK slot

  f->ack = true;

  if (q + 2 >= rd->runs)
  {
    f->cut = true;
    rd->r = q + 1;

    return CAN_OK;
  }

  if ((int64_t)s->len[q + 2] * 256 < (int64_t)arb * 8)
    return CAN_NO;    // the end of frame is not where it has to be

  rd->r = q + 2;

  return CAN_OK;
}

//-----------------------------------------------------------------------------
// An FD frame from the bit after BRS to the end, at one candidate data bit
// time and one candidate count of data-rate bits inside the BRS run. Called
// once per candidate off a saved reader, so it may fail freely.
static int can_fd_body(CanRd *rd, CanFrame *f, CanOut *o, int data_x256,
    int n_extra)
{
  int bit, i, k, st;
  int dlc = 0, dlc_start = 0, dlc_end = 0;
  int crc_start = 0, crc_end = 0;
  int nbits, sc = 0, gray, b2, b1, b0;
  uint32_t rx = 0, want;

  if (f->brs)
  {
    // The BRS bit has been read at the arbitration rate, which put the reader
    // one whole arbitration bit into the run BRS opened. Rewind that to where
    // the sample point actually is and re-cut the rest of the run at the data
    // rate: its end is a true bit boundary, whatever happened inside it.
    if (1 != rd->k)
      return CAN_NO;   // BRS did not open a run, so it was not recessive here

    rd->bit_x256 = data_x256;
    rd->off = can_brs_x256(rd->arb_x256, data_x256, 4, 5);
    rd->n = 1 + n_extra;
  }

  rd->fd_x256 = rd->bit_x256;
  f->cut = false;        // a candidate that failed may have set it

  if (!can_bit(rd, &bit))
    return CAN_CUT;

  f->esi = (bit != 0);

  for (i = 0; i < 4; i++)
  {
    if (!can_bit(rd, &bit))
      return CAN_CUT;

    if (0 == i)
      dlc_start = rd->pos;

    dlc_end = rd->end;
    dlc = (dlc << 1) | bit;
  }

  f->dlc = (uint8_t)dlc;
  f->len = g_can_fd_len[dlc];

  can_put(o, (uint8_t)dlc, dlc_start, dlc_end);

  for (k = 0; k < f->len; k++)
  {
    int v = 0, s0 = 0, e0 = 0;

    for (i = 0; i < 8; i++)
    {
      if (!can_bit(rd, &bit))
        return CAN_CUT;

      if (0 == i)
        s0 = rd->pos;

      e0 = rd->end;
      v = (v << 1) | bit;
    }

    can_put(o, (uint8_t)v, s0, e0);
  }

  // The CRC field: dynamic stuffing ends here and fixed stuffing takes over,
  // starting with one bit before the stuff count. If the data field happened
  // to end on five identical bits, THAT bit is this one - the standard does
  // not send two - so the dynamic counter is simply switched off.
  rd->stuff = false;

  st = can_fixed_bit(rd);

  if (CAN_OK != st)
    return st;

  // The stuff count: how many dynamic stuff bits the frame carried, modulo
  // eight, in gray code with a parity bit under it. It is inside the CRC.
  for (i = 0; i < 4; i++)
  {
    if (!can_raw_bit(rd, &bit))
      return CAN_CUT;

    sc = (sc << 1) | bit;
  }

  if (((sc >> 3) ^ (sc >> 2) ^ (sc >> 1) ^ sc) & 1)
    return CAN_NO;     // the parity bit is even over the three above it

  gray = sc >> 1;
  b2 = (gray >> 2) & 1;
  b1 = b2 ^ ((gray >> 1) & 1);
  b0 = b1 ^ (gray & 1);

  if (((b2 << 2) | (b1 << 1) | b0) != (rd->stuffs & 7))
    return CAN_NO;     // ...and it has to be the number actually counted

  // Everything from here is the CRC itself, which the CRC does not cover
  rd->crc_on = false;

  nbits = (f->len <= 16) ? 17 : 21;

  for (i = 0; i < nbits; i++)
  {
    if (0 == (i % 4))
    {
      st = can_fixed_bit(rd);

      if (CAN_OK != st)
        return st;
    }

    if (!can_raw_bit(rd, &bit))
      return CAN_CUT;

    if (0 == i)
      crc_start = rd->pos;

    crc_end = rd->end;
    rx = (rx << 1) | (uint32_t)bit;
  }

  want = (17 == nbits) ? rd->c17 : rd->c21;
  f->crc_ok = (want == rx);

  st = can_fd_tail(rd, f);

  if (CAN_OK != st)
    return st;

  can_put_field(o, rx, 3, crc_start, crc_end);

  return CAN_OK;
}

//-----------------------------------------------------------------------------
// An FD frame from the FDF bit on. Two things about the data phase are not on
// the wire - its bit rate, and where the sample point that starts it sits -
// so both are narrowed to a handful of candidates off the record's own runs
// and handed to the CRC to choose between. A 17- or 21-bit CRC agreeing by
// chance is a one-in-a-hundred-thousand event; the first candidate that
// checks out is the answer.
static int can_fd_frame(CanRd *rd, CanFrame *f, CanOut *o)
{
  CanRd save;
  int bit, i, st, bn, cn, best = CAN_NO;
  int unit, brs_run_x256;
  int cand[4], nc = 0;

  if (f->rtr)
    return CAN_NO;   // the bit is RRS on an FD frame, and always dominant

  f->fd = true;
  f->rtr = false;

  if (!can_bit(rd, &bit))
    return CAN_CUT;

  if (0 != bit)
    return CAN_NO;   // the reserved bit under FDF, dominant in every FD frame

  if (!can_bit(rd, &bit))
    return CAN_CUT;

  f->brs = (0 != bit);

  bn = o->n;
  cn = o->out ? o->out->count : 0;
  save = *rd;

  if (!f->brs)
  {
    // No switch: the data phase runs at the arbitration rate and there is
    // nothing to guess
    st = can_fd_body(rd, f, o, rd->arb_x256, 0);

    if (CAN_OK == st && f->crc_ok)
      return CAN_OK;

    f->fd = true;

    return (CAN_CUT == st) ? CAN_CUT : CAN_NO;
  }

  // BRS is recessive and the bit before it is the dominant reserved one, so
  // BRS always opens a run - and that run holds BRS plus however many of ESI
  // and the length code's bits stayed recessive. Four at the most: a fifth
  // would meet the stuff condition and put a dominant bit in.
  brs_run_x256 = (int)((int64_t)rd->s->len[rd->r] * 256);
  unit = can_fd_unit(rd, rd->r + 1);

  for (i = 0; i < (int)(sizeof(g_can_fd_ratio) / sizeof(g_can_fd_ratio[0])) &&
      nc < (int)(sizeof(cand) / sizeof(cand[0])); i++)
  {
    int t = rd->arb_x256 / g_can_fd_ratio[i];

    if (t < CAN_MIN_SAMPLES_PER_BIT * 256)
      break;          // too fast to sample: fewer than three points per bit

    // The measured unit is what picks the ratio out of the list. Without one
    // there is nothing to measure against and every ratio would be tried.
    if (unit <= 0 || t < unit - unit / 6 || t > unit + unit / 6)
      continue;

    cand[nc++] = t;
  }

  // ...and the measurement itself, for a bus whose data rate is not a whole
  // multiple of its arbitration rate
  if (unit > 0 && unit >= CAN_MIN_SAMPLES_PER_BIT * 256 &&
      nc < (int)(sizeof(cand) / sizeof(cand[0])))
    cand[nc++] = unit;

  for (i = 0; i < nc; i++)
  {
    int t = cand[i];
    int lo, hi, n;

    // How many data-rate bits sit in the BRS run after BRS itself. The answer
    // depends on the sample point, and the sample point is only known to lie
    // somewhere between 70% and 90% of a bit - so this is the range that
    // ignorance leaves, and it is usually one value wide.
    hi = (int)(((int64_t)brs_run_x256 -
        can_brs_x256(rd->arb_x256, t, 7, 10) + t / 2) / t);
    lo = (int)(((int64_t)brs_run_x256 -
        can_brs_x256(rd->arb_x256, t, 9, 10) + t / 2) / t);

    if (lo < 0)
      lo = 0;

    if (hi > 4)
      hi = 4;

    for (n = lo; n <= hi; n++)
    {
      *rd = save;
      o->n = bn;

      if (o->out)
        o->out->count = cn;

      st = can_fd_body(rd, f, o, t, n);

      if (CAN_OK == st && f->crc_ok)
        return CAN_OK;

      if (CAN_CUT == st)
        best = CAN_CUT;
    }
  }

  // Recognised, and not read. Which is worth saying out loud rather than
  // reporting the record as holding nothing at all.
  *rd = save;
  f->fd = true;
  f->brs = true;

  return best;
}

//-----------------------------------------------------------------------------
// Read one frame, starting at the bit the reader is positioned on. Returns
// CAN_NO when the bits are not a frame, CAN_CUT when the record ran out
// inside one, CAN_OK when a whole frame came out - f->crc_ok says whether it
// was also a correct one.
static int can_frame(CanRd *rd, CanFrame *f, CanOut *o)
{
  uint16_t crc = 0, rx_crc = 0;
  uint32_t id = 0;
  int bit, i, k;
  int id_start = 0, id_end = 0, dlc_start = 0, dlc_end = 0;
  int crc_start = 0, crc_end = 0;
  int dlc = 0, nd;
  int rtr_srr, ide;

  memset(f, 0, sizeof(*f));
  o->n = 0;

  rd->last = 1;       // the idle before the frame is recessive, and is not
  rd->same = 0;       // counted: the stuff counter starts at the SOF
  rd->stuff = true;
  rd->stuff_err = false;
  rd->stuffs = 0;
  rd->bit_x256 = rd->arb_x256;
  rd->fd_x256 = rd->arb_x256;

  // The FD CRCs cover the frame from its first bit, and whether this is an FD
  // frame is not known for another twenty. So they run from here whenever FD
  // frames are being read at all, and are dropped if the frame is a classic
  // one. Two shifts a bit is cheaper than buffering the header to replay it.
  rd->crc_on = rd->fd;
  rd->c17 = 1u << 16;
  rd->c21 = 1u << 20;

  if (!can_bit(rd, &bit))
    return CAN_CUT;

  if (bit != 0)
    return CAN_NO;    // start of frame is dominant

  can_crc_bit(&crc, 0);

  for (i = 0; i < 11; i++)
  {
    if (!can_bit(rd, &bit))
      return CAN_CUT;

    if (0 == i)
      id_start = rd->pos;

    id_end = rd->end;
    can_crc_bit(&crc, bit);
    id = (id << 1) | (uint32_t)bit;
  }

  if (!can_bit(rd, &rtr_srr))
    return CAN_CUT;

  can_crc_bit(&crc, rtr_srr);

  if (!can_bit(rd, &ide))
    return CAN_CUT;

  can_crc_bit(&crc, ide);

  if (ide)
  {
    // Extended: eighteen more identifier bits, then RTR and two reserved
    f->ext = true;

    for (i = 0; i < 18; i++)
    {
      if (!can_bit(rd, &bit))
        return CAN_CUT;

      id_end = rd->end;
      can_crc_bit(&crc, bit);
      id = (id << 1) | (uint32_t)bit;
    }

    if (!can_bit(rd, &bit))
      return CAN_CUT;

    f->rtr = (bit != 0);
    can_crc_bit(&crc, bit);

    if (!can_bit(rd, &bit))    // r1, and FDF in a CAN FD frame
      return CAN_CUT;

    can_crc_bit(&crc, bit);

    if (bit != 0)
    {
      f->id = id;
      can_put_field(o, id, 4, id_start, id_end);

      if (!rd->fd)
      {
        f->fd = true;

        return CAN_NO;
      }

      return can_fd_frame(rd, f, o);
    }
  }
  else
  {
    f->rtr = (rtr_srr != 0);
  }

  if (!can_bit(rd, &bit))      // r0, and FDF in a standard CAN FD frame
    return CAN_CUT;

  can_crc_bit(&crc, bit);

  if (bit != 0)
  {
    if (f->ext)
      return CAN_NO;           // r0 is reserved dominant even in an FD frame

    f->id = id;
    can_put_field(o, id, 2, id_start, id_end);

    if (!rd->fd)
    {
      // Recessive here is the flag that says the rest of this frame is
      // written in a format this decoder was not asked to read
      f->fd = true;

      return CAN_NO;
    }

    return can_fd_frame(rd, f, o);
  }

  f->id = id;
  can_put_field(o, id, f->ext ? 4 : 2, id_start, id_end);

  for (i = 0; i < 4; i++)
  {
    if (!can_bit(rd, &bit))
      return CAN_CUT;

    if (0 == i)
      dlc_start = rd->pos;

    dlc_end = rd->end;
    can_crc_bit(&crc, bit);
    dlc = (dlc << 1) | bit;
  }

  f->dlc = (uint8_t)dlc;
  nd = (dlc > 8) ? 8 : dlc;   // 9..15 are legal and all mean eight bytes

  if (f->rtr)
    nd = 0;                   // a remote frame asks for data, it carries none

  f->len = (uint8_t)nd;

  can_put(o, (uint8_t)dlc, dlc_start, dlc_end);

  for (k = 0; k < nd; k++)
  {
    int v = 0, s0 = 0, e0 = 0;

    for (i = 0; i < 8; i++)
    {
      if (!can_bit(rd, &bit))
        return CAN_CUT;

      if (0 == i)
        s0 = rd->pos;

      e0 = rd->end;
      can_crc_bit(&crc, bit);
      v = (v << 1) | bit;
    }

    can_put(o, (uint8_t)v, s0, e0);
  }

  for (i = 0; i < 15; i++)
  {
    if (!can_bit(rd, &bit))
      return CAN_CUT;

    if (0 == i)
      crc_start = rd->pos;

    crc_end = rd->end;
    rx_crc = (uint16_t)((rx_crc << 1) | bit);
  }

  // The delimiter is fixed-form, but a stuff bit can still stand in front of
  // it - stuffing covers the CRC field to its last bit - so it is read with
  // destuffing on and switched off immediately after
  if (!can_bit(rd, &bit))
    return CAN_CUT;

  rd->stuff = false;

  if (bit != 1)
    return CAN_NO;

  if (!can_bit(rd, &bit))
    return CAN_CUT;

  f->ack = (0 == bit);        // pulled down by any node that received it

  if (!can_bit(rd, &bit))
    return CAN_CUT;

  if (bit != 1)
    return CAN_NO;            // ACK delimiter

  for (i = 0; i < 7; i++)
  {
    if (!can_bit(rd, &bit))
    {
      f->cut = true;          // the record ended, the frame did not
      break;
    }

    if (bit != 1)
      return CAN_NO;          // end of frame is seven recessive bits
  }

  if (rd->stuff_err)
    return CAN_NO;

  f->crc_ok = (crc == rx_crc);

  // The CRC's two bytes are a field like the identifier and divide its span
  can_put_field(o, rx_crc, 2, crc_start, crc_end);

  return CAN_OK;
}

//-----------------------------------------------------------------------------
// Cheap refusal before any parsing: at this bit time, do the runs even look
// like CAN? Five bits is the most stuffing allows and eleven the least an
// interframe gap can be, so nothing lands in between. A UART record with six
// zero bits in a byte is thrown out here rather than after a thousand bit
// reads. Boundary runs are cut by the ends of the record and prove nothing.
//
// `fd` drops one rule and one only: a run SHORTER than a bit, which is what
// the whole data phase of a CAN FD frame is made of. Everything else holds
// there too - its runs are shorter than an arbitration bit, never longer.
static bool can_runs_plausible(const LogicScratch *s, int runs, int bit_x256,
    int rec_lvl, bool fd)
{
  bool gap = false;

  for (int r = 0; r < runs; r++)
  {
    int q = (int)(((int64_t)s->len[r] * 256 + bit_x256 / 2) / bit_x256);

    // The idle a frame starts behind is usually the first or the last run of
    // the record, and those are cut short by its ends - which makes their
    // length a lower bound, still good enough to count as a gap, and useless
    // for everything else
    if (s->lvl[r] == rec_lvl && q >= CAN_GAP_MIN_BITS)
      gap = true;

    if (0 == r || r + 1 >= runs)
      continue;

    if (q < 1 && !fd)
      return false;    // a run shorter than half a bit is not this bit time

    if (s->lvl[r] == rec_lvl)
    {
      if (q > 5 && q < CAN_GAP_MIN_BITS)
        return false;
    }
    else if (q > CAN_DOM_MAX_BITS)
      return false;
  }

  return gap;          // no interframe gap, no anchor for a start of frame
}

//-----------------------------------------------------------------------------
// Decode the whole record at one bit time and polarity. Returns the number of
// frames whose CRC checked out - the score, and the proof. out and a may be
// NULL, which is how the rate search runs before a winner is picked.
static int can_scan(const LogicScratch *s, int runs, int bit_x256, int rec_lvl,
    bool fd, LogicResult *out, CanAnalysis *a)
{
  CanRd rd;
  CanOut o;
  CanFrame f;
  int ok = 0;

  o.out = out;
  o.n = 0;

  memset(&rd, 0, sizeof(rd));
  rd.s = s;
  rd.runs = runs;
  rd.bit_x256 = bit_x256;
  rd.arb_x256 = bit_x256;
  rd.rec_lvl = rec_lvl;
  rd.fd = fd;

  for (int r = 1; r < runs; r++)
  {
    if (s->lvl[r] == rec_lvl)
      continue;   // a frame starts with a dominant bit

    // ...and only after the line has been recessive for longer than any run
    // inside a frame can be. That is what makes this a start of frame rather
    // than some dominant bit in the middle of one.
    if ((int64_t)s->len[r - 1] * 256 < (int64_t)bit_x256 * CAN_SOF_IDLE_BITS)
      continue;

    rd.bit_x256 = bit_x256;
    rd.r = r;
    rd.k = 0;
    rd.off = 0;
    rd.n = can_run_bits(&rd, r);
    rd.ended = false;

    // Where this frame's bytes start, and where the record goes back to if it
    // turns out not to be a frame at all
    int first = out ? out->count : 0;

    o.n = 0;

    int st = can_frame(&rd, &f, &o);

    if (CAN_OK != st && out)
      out->count = first;

    if (CAN_CUT == st)
    {
      if (out)
        out->overrun = true;

      break;
    }

    if (CAN_NO == st)
    {
      if (f.fd && a)
        a->fd_seen = true;

      continue;
    }

    if (f.crc_ok)
      ok++;

    if (out)
    {
      // More of this frame than the record can hold. Not a partial frame:
      // half a hex dump under a header that says 64 bytes is a lie about the
      // bus, and the flag says the record has more in it than fits.
      if (first + o.n > LOGIC_MAX_BYTES ||
          (a && a->frames >= CAN_MAX_FRAMES))
      {
        out->count = first;
        out->truncated = true;
        break;
      }

      if (a)
      {
        int idn = f.ext ? 4 : 2;
        int crcn = f.fd ? 3 : 2;   // 17 or 21 bits of CRC take three bytes

        f.first = first;
        f.count = o.n;
        a->frame[a->frames] = f;

        for (int i = 0; i < o.n; i++)
        {
          a->fidx[first + i] = (uint8_t)a->frames;
          a->role[first + i] = (i < idn) ? CAN_R_ID :
              (i == idn) ? CAN_R_DLC :
              (i < o.n - crcn) ? CAN_R_DATA :
              (i < o.n - 1) ? CAN_R_CRC : CAN_R_ACK;
        }

        a->frames++;

        if (f.crc_ok)
          a->crc_ok++;

        if (f.fd)
        {
          a->fd++;

          // Every frame on a bus switches to the same data rate, so the last
          // one to say what it is says it for the record
          if (rd.fd_x256 > 0)
            a->data_rate = rd.fd_x256;
        }
      }

      if (f.cut)
        out->overrun = true;

      out->errors += f.crc_ok ? 0 : 1;
    }

    // Carry on from wherever the frame ended
    if (rd.r > r)
      r = rd.r - 1;
  }

  return ok;
}

//-----------------------------------------------------------------------------
static void can_rate_str(int rate, char *buf, int size)
{
  if (rate < 0)
    rate = 0;

  if (rate > 99999999)
    rate = 99999999;

  if (rate >= 1000000)
  {
    if (0 == rate % 1000000)
      snprintf(buf, size, "%dM", rate / 1000000);
    else
      snprintf(buf, size, "%d.%dM", rate / 1000000, (rate % 1000000) / 100000);
  }
  else if (0 == rate % 1000)
  {
    snprintf(buf, size, "%dk", rate / 1000);
  }
  else
  {
    snprintf(buf, size, "%d.%dk", rate / 1000, (rate % 1000) / 100);
  }
}

//-----------------------------------------------------------------------------
const CanAnalysis *can_analysis(void)
{
  return &g_can;
}

//-----------------------------------------------------------------------------
void can_byte_label(const CanAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  const CanFrame *f = &a->frame[a->fidx[idx] % CAN_MAX_FRAMES];
  int rel = idx - f->first;

  switch (a->role[idx])
  {
    case CAN_R_ID:
      // The identifier is eleven or twenty-nine bits over two or four bytes,
      // so no single one of them is the identifier. The number goes on the
      // byte that finishes the field - by which point it is known - and the
      // ones before it are named for the field they are part of.
      if (rel == (f->ext ? 4 : 2) - 1)
        snprintf(buf, size, "ID=%X", (unsigned)f->id);
      else
        snprintf(buf, size, "ID");
      break;

    case CAN_R_DLC:
      // On an FD frame the code is not the count - 15 means 64 - so the count
      // is what the label says, and the code stays in the hex dump where it
      // belongs. The FD in front of it is the other half of the answer: this
      // frame is the reason the record needed two bit rates.
      if (f->fd)
        snprintf(buf, size, "FD%d", f->len);
      else
        snprintf(buf, size, f->rtr ? "RTR%d" : "DLC%d", f->dlc);
      break;

    case CAN_R_DATA:
      snprintf(buf, size, "D%d", rel - (f->ext ? 4 : 2) - 1);
      break;

    case CAN_R_CRC:
      snprintf(buf, size, f->crc_ok ? "CRC" : "CRC!");
      break;

    case CAN_R_ACK:
      // The acknowledge slot is one bit at the end of the frame and belongs
      // to nobody's byte, so it is reported on the last of them. Its absence
      // is the single most useful thing a scope can say about a CAN node: it
      // means nothing else on the bus heard the frame.
      snprintf(buf, size, f->ack ? "ACK" : "NAK");
      break;

    default:
      break;
  }
}

//-----------------------------------------------------------------------------
// What this byte is on its own. The identifier's bytes are part of one number
// and say so as a group; individually each is just a byte of the identifier.
void can_field_label(const CanAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (CAN_R_ID == a->role[idx])
    snprintf(buf, size, "ID");
  else
    can_byte_label(a, idx, v, buf, size);
}

//-----------------------------------------------------------------------------
// The identifier is one number over two bytes or four, so it is one thing on
// the screen. The CRC's bytes are NOT grouped: what is worth reading there is
// not the number but whether it checked out and whether anyone acknowledged
// the frame, and those are two different answers.
void can_group_at(const CanAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || CAN_R_ID != a->role[idx])
    return;

  const CanFrame *f = &a->frame[a->fidx[idx] % CAN_MAX_FRAMES];

  *start = f->first;
  *len = f->ext ? 4 : 2;
}

//-----------------------------------------------------------------------------
// A bit time in samples x256 back into bit/s, snapped to the standard rate it
// is within a tenth of. Reporting 1998400 for a 2 Mbit data phase would be
// arithmetic about the sample clock and not a fact about the bus.
static int can_rate_of(int bit_x256, int period_ns, const int *std, int n)
{
  int rate;

  if (bit_x256 <= 0 || period_ns <= 0)
    return 0;

  rate = (int)((int64_t)256 * 1000000000 / ((int64_t)bit_x256 * period_ns));

  for (int i = 0; i < n; i++)
  {
    if (rate > std[i] - std[i] / 10 && rate < std[i] + std[i] / 10)
      return std[i];
  }

  return rate;
}

//-----------------------------------------------------------------------------
int can_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  static const int fd_rates[] =
  {
    500000, 1000000, 2000000, 4000000, 5000000, 8000000,
  };

  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_CAN;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 8 || period_ns <= 0)
    return 0;

  // Recessive is where the line rests, which is the longest run in the record.
  // Whether that is a high level (CAN_L, or a transceiver's TX) or a low one
  // (CAN_H) is the probe's business, not the decoder's.
  int longest = 1;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (scratch->len[r] > scratch->len[longest])
      longest = r;
  }

  int best_ok = 0, best_bit = 0, best_lvl = 0;

  // Two passes, and the order is the point. Classic CAN first, with the
  // strict run test and no FD parsing at all: it is by far the commoner bus,
  // it costs exactly what it always cost, and a record it explains needs no
  // second search. Only when nothing classic checks out does the FD pass run,
  // which has to let sub-bit runs through and therefore looks at more
  // candidate rates than it would like.
  for (int pass = 0; pass < 2 && 0 == best_ok; pass++)
  {
    bool fd = (1 == pass);

    for (int p = 0; p < 2; p++)
    {
      int rec_lvl = (0 == p) ? scratch->lvl[longest] : !scratch->lvl[longest];

      for (unsigned i = 0; i < sizeof(g_can_rates) / sizeof(g_can_rates[0]);
          i++)
      {
        int64_t bit_x256 = (int64_t)256 * 1000000000 /
            ((int64_t)g_can_rates[i] * period_ns);

        if (bit_x256 < CAN_MIN_SAMPLES_PER_BIT * 256)
          continue;   // too fast to sample: fewer than three points per bit

        if (bit_x256 * CAN_MIN_RECORD_BITS > (int64_t)size * 256)
          continue;   // too slow: not even one frame fits in the record

        if (!can_runs_plausible(scratch, runs, (int)bit_x256, rec_lvl, fd))
          continue;

        int ok = can_scan(scratch, runs, (int)bit_x256, rec_lvl, fd, NULL,
            NULL);

        if (ok > best_ok)
        {
          best_ok = ok;
          best_bit = (int)bit_x256;
          best_lvl = rec_lvl;
        }
      }

      if (best_ok > 0)
        break;    // the line rests where it rests; no need to try it upside
                  // down
    }
  }

  // Nothing here passed a CRC. A frame that does is proof; a frame that does
  // not is a guess, and this decoder runs ahead of the generic ones precisely
  // because it never guesses.
  if (0 == best_ok)
    return 0;

  memset(&g_can, 0, sizeof(g_can));
  g_can.rate = 0;

  // The final read always takes FD frames, whichever pass found the rate: a
  // bus that carries both is a bus that carries both, and a classic frame is
  // what fixed the arbitration rate for the FD frames beside it.
  can_scan(scratch, runs, best_bit, best_lvl, true, out, &g_can);

  if (0 == out->count)
    return 0;

  // Report the standard rate that was tried, not the arithmetic back out of
  // the rounded bit time
  int rate = can_rate_of(best_bit, period_ns, g_can_rates,
      (int)(sizeof(g_can_rates) / sizeof(g_can_rates[0])));

  g_can.rate = rate;
  g_can.data_rate = (g_can.data_rate > 0) ?
      can_rate_of(g_can.data_rate, period_ns, fd_rates,
          (int)(sizeof(fd_rates) / sizeof(fd_rates[0]))) : rate;

  out->rate = rate;
  out->idle_high = (best_lvl != 0);
  out->burst_start = true;   // every frame here was found behind an idle line

  char rs[12], ds[12], rf[26], sfx[10];
  bool nak = false, esi = false;

  for (int i = 0; i < g_can.frames; i++)
  {
    if (!g_can.frame[i].ack)
      nak = true;

    if (g_can.frame[i].esi)
      esi = true;
  }

  can_rate_str(rate, rs, sizeof(rs));

  // Two rates when the bus switched between them, one when it did not. The
  // pair IS the headline of a CAN FD record.
  if (g_can.fd > 0 && g_can.data_rate != rate)
  {
    can_rate_str(g_can.data_rate, ds, sizeof(ds));
    snprintf(rf, sizeof(rf), "%s/%s", rs, ds);
  }
  else
  {
    snprintf(rf, sizeof(rf), "%s", rs);
  }

  snprintf(sfx, sizeof(sfx), "%s%s", nak ? " NAK" : "", esi ? " EP" : "");

  if (1 == g_can.frames)
  {
    const CanFrame *f = &g_can.frame[0];

    if (f->fd)
      // The one line that does NOT spell "ID" out: two rates, a 29-bit
      // identifier, 64 bytes and a NAK is already every character the panel
      // header has, and the byte under the identifier says ID= anyway
      snprintf(out->info, sizeof(out->info), "CAN FD %s %X %dB%s", rf,
          (unsigned)f->id, f->len, sfx);
    else if (f->rtr)
      snprintf(out->info, sizeof(out->info), "CAN %s ID %X RTR%s", rf,
          (unsigned)f->id, sfx);
    else
      snprintf(out->info, sizeof(out->info), "CAN %s ID %X %dB%s", rf,
          (unsigned)f->id, f->len, sfx);
  }
  else
  {
    snprintf(out->info, sizeof(out->info), "CAN%s %s %d frames%s",
        g_can.fd > 0 ? " FD" : "", rf, g_can.frames, sfx);
  }

  return out->count;
}
