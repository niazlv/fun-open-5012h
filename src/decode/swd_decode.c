/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SWD - Serial Wire Debug, read off SWDIO with the clock on the other wire.
 *
 * This is the same predicament as the SPI decoder: a clocked bus, one probe,
 * and no way to see the edges the bits are actually sampled on. What makes it
 * a different problem - and worth doing properly - is that SWD checks itself.
 * A packet carries a parity bit over its four address bits, a stop bit that
 * must be zero, a park bit that must be one, a three-bit acknowledgement with
 * only three legal values, and a parity bit over its thirty-two data bits.
 * Six independent checks, and a bit time that is even slightly wrong fails
 * them. So a transaction that comes out clean is not a plausible reading of
 * the record - it is a CONFIRMED one, and the clock is confirmed with it.
 *
 * That is why this decoder does not quantise the record into one long bit
 * stream the way the SPI one has to. It anchors on the RISING EDGE of every
 * start bit and reads forty-six bits from there:
 *
 *   start(1) APnDP RnW A[2] A[3] parity stop(0) park(1)
 *   turnaround
 *   ACK[3]              001 OK, 010 WAIT, 100 FAULT
 *   read:   data[32] parity turnaround
 *   write:  turnaround data[32] parity
 *
 * Re-anchoring at each packet is what makes a gated clock harmless. A debug
 * probe on USB clocks in bursts and stops in between, and between the bursts
 * the number of clock cycles is unknowable from SWDIO alone - which does not
 * matter in the slightest, because the next packet's own edge says where it
 * starts. A drifting global phase, which is the SPI decoder's real weakness,
 * never accumulates here beyond one packet.
 *
 * What is still assumed, and will produce NO output rather than wrong output
 * if it is untrue: one turnaround cycle (the reset default; DLCR.TURNROUND can
 * ask for up to four), and overrun detection off, which is the case unless
 * something deliberately turned it on. A multidrop TARGETSEL write is not
 * acknowledged at all by design, so it does not parse - and does not produce a
 * fabricated transaction either.
 *
 * The registers are named rather than numbered, and naming an AP register
 * takes context: which bank is selected lives in a DP SELECT write that went
 * past earlier in the same record. So SELECT is tracked, exactly the way the
 * 1-Wire decoder tracks a family code before it will name a function command.
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
#define SWD_MIN_BIT_SAMPLES   3
#define SWD_REQ_BITS          8
#define SWD_TX_BITS          46     // request, turnaround, ack, data, parity
#define SWD_SHORT_BITS       13     // ...when the ack was not OK
#define SWD_RESET_BITS       50     // a line reset is at least this many ones

// The bit time is searched around its estimate, one percent to a step. A
// measured frequency carries its own error and one worked out of the runs
// assumes the record holds a single-clock run somewhere; the reader itself
// re-anchors on edges, so this only has to get close enough for the first
// eight bits to land, not for all forty-six.
#define SWD_TRIM_STEPS        6

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_swd   (g_logic_analysis.swd)

// 0 = work the bit time out of the record. Otherwise the clock the user
// measured on SWCLK before moving the probe to SWDIO.
static int g_swd_clock_hz = 0;

static const char *const g_swd_dp_rd[4] =
    { "DPIDR", "CTRL/STAT", "RESEND", "RDBUFF" };
static const char *const g_swd_dp_wr[4] =
    { "ABORT", "CTRL/STAT", "SELECT", "TARGETSEL" };
static const char *const g_swd_ap_b0[4] = { "CSW", "TAR", "TAR-H", "DRW" };
static const char *const g_swd_ap_b1[4] = { "BD0", "BD1", "BD2", "BD3" };

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void swd_decode_set_clock(int hz)
{
  g_swd_clock_hz = (hz > 0) ? hz : 0;
}

//-----------------------------------------------------------------------------
const SwdAnalysis *swd_analysis(void)
{
  return &g_swd;
}

//-----------------------------------------------------------------------------
// The level at a sample position, with a cursor that only ever moves forward.
// Every reader here walks a packet from its start, so a monotone cursor turns
// what would be a search per bit into one step per run.
static inline int swd_level(const LogicScratch *s, int runs, int *cur,
    int64_t pos_x256)
{
  while (*cur + 1 < runs && (int64_t)s->pos[*cur + 1] * 256 <= pos_x256)
    (*cur)++;

  return s->lvl[*cur];
}

//-----------------------------------------------------------------------------
static inline bool swd_parity(uint32_t v, int bits)
{
  int ones = 0;

  for (int i = 0; i < bits; i++)
    ones += (int)((v >> i) & 1);

  return (0 != (ones & 1));
}

//-----------------------------------------------------------------------------
// Which register a request names. An AP register needs the bank out of the
// last SELECT written, which is why that is tracked at all.
static void swd_reg_name(bool ap, bool write, int a, uint32_t select,
    char *buf, int size)
{
  if (!ap)
  {
    snprintf(buf, size, "%s", write ? g_swd_dp_wr[a] : g_swd_dp_rd[a]);

    return;
  }

  int bank = (int)((select >> 4) & 0x0F);

  if (0 == bank)
    snprintf(buf, size, "%s", g_swd_ap_b0[a]);
  else if (1 == bank)
    snprintf(buf, size, "%s", g_swd_ap_b1[a]);
  else if (0x0F == bank && 3 == a)
    snprintf(buf, size, "IDR");
  else
    snprintf(buf, size, "AP%X%d", bank, a);
}

//-----------------------------------------------------------------------------
// A bit reader that keeps its PHASE rather than stepping a fixed grid.
//
// Forty-six bits is long enough for arithmetic to drift out of the packet: a
// bit time one percent off is half a bit out by the parity, and the parity is
// what the whole decoder rests on. So after every bit the reader looks for a
// real edge where the bit ought to end and, finding one within tolerance,
// takes it. That is sound here in a way it is not on most buses: on SWD every
// edge IS a clock boundary, because the line only changes when the clock says
// so. A measured edge beats a running total every time.
//
// Same technique the Manchester reader uses, and for the same reason.
typedef struct
{
  const LogicScratch *s;
  int      runs;
  int      size;
  int64_t  bit;
  int64_t  tol;
  int64_t  pos;     // leading edge of the next bit, x256
  int      cur;     // run cursor; only ever moves forward
} SwdRd;

//-----------------------------------------------------------------------------
static void swd_rd_init(SwdRd *rd, const LogicScratch *s, int runs, int size,
    int64_t bit_x256, int64_t start_x256)
{
  rd->s = s;
  rd->runs = runs;
  rd->size = size;
  rd->bit = bit_x256;
  rd->tol = bit_x256 * 3 / 8;
  rd->pos = start_x256;
  rd->cur = 0;
}

//-----------------------------------------------------------------------------
// The next bit, or -1 when the record runs out inside it
static int swd_rd_bit(SwdRd *rd)
{
  int64_t mid = rd->pos + rd->bit / 2;

  if (rd->pos < 0 || mid / 256 >= rd->size)
    return -1;

  while (rd->cur + 1 < rd->runs &&
      (int64_t)rd->s->pos[rd->cur + 1] * 256 <= mid)
    rd->cur++;

  int v = rd->s->lvl[rd->cur];
  int64_t want = rd->pos + rd->bit;
  int k = rd->cur;

  // The first run boundary that could be this bit's end
  while (k + 1 < rd->runs &&
      (int64_t)rd->s->pos[k + 1] * 256 < want - rd->tol)
    k++;

  if (k + 1 < rd->runs)
  {
    int64_t b = (int64_t)rd->s->pos[k + 1] * 256;

    if (b <= want + rd->tol)
      want = b;                   // an edge here: that IS the boundary
  }

  rd->pos = want;

  return v;
}

//-----------------------------------------------------------------------------
// Try to read one transaction starting at `p0` (x256 samples). Returns true
// on a packet, with `end` receiving where it finished - which is not p0 plus
// a bit count, because the phase has been re-anchored along the way.
static bool swd_try(const LogicScratch *s, int runs, int size, int64_t p0,
    int64_t bit_x256, SwdTx *tx, int64_t *data_pos, int64_t *end)
{
  SwdRd rd;
  int b[SWD_REQ_BITS];

  swd_rd_init(&rd, s, runs, size, bit_x256, p0);

  for (int k = 0; k < SWD_REQ_BITS; k++)
  {
    b[k] = swd_rd_bit(&rd);

    if (b[k] < 0)
      return false;
  }

  if (1 != b[0] || 0 != b[6] || 1 != b[7])
    return false;                 // start, stop and park are not negotiable

  if (b[5] != ((b[1] ^ b[2] ^ b[3] ^ b[4]) & 1))
    return false;                 // ...nor is the parity over the four in the
                                  // middle, which is what makes this a packet
                                  // rather than eight bits that look like one
  if (swd_rd_bit(&rd) < 0)
    return false;                 // turnaround: undriven, so never read

  int ack = 0;

  for (int k = 0; k < 3; k++)
  {
    int v = swd_rd_bit(&rd);

    if (v < 0)
      return false;

    ack |= v << k;
  }

  if (1 != ack && 2 != ack && 4 != ack)
    return false;                 // 000 and 111 mean nobody drove the line,
                                  // and the rest are not defined at all

  tx->req = (uint8_t)(b[0] | (b[1] << 1) | (b[2] << 2) | (b[3] << 3) |
      (b[4] << 4) | (b[5] << 5) | (b[6] << 6) | (b[7] << 7));
  tx->ack = (uint8_t)ack;
  tx->ap = (0 != b[1]);
  tx->write = (0 == b[2]);
  tx->reg = (uint8_t)((b[4] << 1) | b[3]);
  tx->data = 0;
  tx->par_ok = true;
  tx->has_data = false;

  if (1 != ack)
  {
    if (swd_rd_bit(&rd) < 0)      // the turnaround that ends it
      return false;

    *end = rd.pos;

    return true;                  // WAIT or FAULT: no data phase follows
  }

  // A read puts the data straight after the acknowledgement; a write has to
  // hand the line back first, so its data starts one cycle later
  if (tx->write && swd_rd_bit(&rd) < 0)
    return false;

  *data_pos = rd.pos;             // where the thirty-two bits actually begin

  uint32_t data = 0;

  for (int k = 0; k < 32; k++)
  {
    int v = swd_rd_bit(&rd);

    if (v < 0)
      return false;

    data |= (uint32_t)v << k;
  }

  int par = swd_rd_bit(&rd);

  if (par < 0)
    return false;

  tx->data = data;
  tx->has_data = true;
  tx->par_ok = (par == (swd_parity(data, 32) ? 1 : 0));

  if (!tx->write)
    swd_rd_bit(&rd);              // the trailing turnaround; may run off the
                                  // end of the record, which is not a fault
  *end = rd.pos;

  // The parity is REPORTED, not required. A packet that got this far has
  // already passed five checks, so a failed data parity is a fact about the
  // traffic - a marginal signal, a probe on a long lead - and hiding the
  // transaction would hide exactly the fault worth seeing.
  return true;
}

//-----------------------------------------------------------------------------
// The bit time the record implies: the shortest interior run is one clock,
// refined by averaging the runs over the whole number of bits each holds.
static int64_t swd_estimate_bit(const LogicScratch *s, int runs)
{
  int tmin = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (0 == tmin || s->len[r] < tmin)
      tmin = s->len[r];
  }

  if (tmin < SWD_MIN_BIT_SAMPLES)
    return 0;

  int64_t sum = 0;
  int count = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    int n = (s->len[r] + tmin / 2) / tmin;

    if (n < 1 || n > 64)
      continue;

    sum += s->len[r];
    count += n;
  }

  if (count < 8)
    return 0;

  return sum * 256 / count;
}

//-----------------------------------------------------------------------------
// One pass over the record at one bit time. Fills the analysis and the result
// when `out` is given; otherwise it only scores, which is what the search over
// bit times needs.
//
// The score weighs a transaction whose DATA PARITY agreed four times a bare
// one, and that ratio is the whole reason the search works. A bit time that
// is a few percent out still passes the request checks - eight bits is not
// long enough for the drift to matter - and then reads thirty-two bits of
// rubbish, which the parity catches and nothing else does. Scoring on
// transaction count alone let a four-percent error tie with the truth.
static int swd_pass(const LogicScratch *s, int runs, int size,
    int64_t bit_x256, SwdAnalysis *a, LogicResult *out)
{
  int64_t consumed = -1;          // sample position, x256, already read
  int txs = 0, oks = 0;

  if (a)
  {
    memset(a->tidx, 0xFF, sizeof(a->tidx));
    memset(a->role, 0, sizeof(a->role));
    a->txs = a->resets = a->switches = a->waits = a->faults = a->par_err = 0;
    a->ok_txs = a->gap_bad = 0;
    a->select = 0;
  }

  for (int r = 1; r + 1 < runs; r++)
  {
    // A line reset is at least fifty clocks of ones - far longer than the
    // eight a packet can hold - and it is how a debugger announces itself
    if (0 != s->lvl[r] &&
        (int64_t)s->len[r] * 256 >= bit_x256 * SWD_RESET_BITS)
    {
      if (a)
        a->resets++;

      // ...and what usually follows it is the sixteen bits that take a port
      // out of JTAG and into SWD. Its first bit is a zero, so the falling
      // edge that ends the reset IS that bit's leading edge - which is the
      // only place in this decoder where a sequence with no framing of its
      // own can be read with any confidence.
      int64_t p0 = (int64_t)s->pos[r + 1] * 256;
      int cur = r + 1;
      uint32_t seq = 0;
      bool whole = true;

      for (int k = 0; k < 16; k++)
      {
        int64_t p = p0 + bit_x256 * (2 * k + 1) / 2;

        if (p / 256 >= size)
        {
          whole = false;

          break;
        }

        seq |= (uint32_t)swd_level(s, runs, &cur, p) << k;
      }

      if (whole && (0xE79E == seq || 0xE73C == seq))
      {
        if (a)
          a->switches++;

        if (out && out->count + 2 <= LOGIC_MAX_BYTES)
        {
          for (int k = 0; k < 2; k++)
          {
            out->bytes[out->count] = (uint8_t)(seq >> (8 * k));
            out->pos[out->count] = (int)(p0 / 256) +
                (int)(bit_x256 * 8 * k / 256);
            out->end[out->count] = (int)(p0 / 256) +
                (int)(bit_x256 * 8 * (k + 1) / 256);

            if (a)
              a->role[out->count] = SWD_R_SWITCH;

            out->count++;
          }
        }

        consumed = p0 + bit_x256 * 16;
      }

      continue;
    }

    if (0 == s->lvl[r] || 0 != s->lvl[r - 1])
      continue;                   // every anchor here is a RISING edge

    SwdTx tx;
    int64_t data_pos = 0, end = 0;
    int64_t edge = (int64_t)s->pos[r] * 256;
    int64_t pstart = edge;

    // Two readings of the same edge, and the second is what makes back-to-back
    // packets readable at all.
    //
    // The obvious one is that this edge is the START bit. It usually is: a
    // host holds SWDIO low through its idle cycles, so a packet opens with a
    // rise. But it does not have to - a packet may follow the one before it
    // with no idle at all, and then the previous turnaround left the line
    // high and there is NO edge at the start bit to anchor on.
    //
    // What every packet does have is the stop bit and the park bit, which are
    // 0 and 1 by definition: a rising edge at bit 7, always, whatever the data
    // and whatever came before. So an edge that heads no packet is tried again
    // as that one, seven bit times further in.
    if (edge < consumed || !swd_try(s, runs, size, edge, bit_x256, &tx,
        &data_pos, &end))
    {
      pstart = edge - bit_x256 * 7;

      if (pstart < 0 || pstart < consumed ||
          !swd_try(s, runs, size, pstart, bit_x256, &tx, &data_pos, &end))
        continue;
    }

    txs++;

    if (1 == tx.ack && tx.has_data && tx.par_ok)
      oks++;

    // A DP SELECT write decides what an AP address means from here on, so it
    // is tracked as it goes past. Reading it out of the transaction after the
    // fact would be too late for the transactions in between.
    if (a && tx.write && !tx.ap && 2 == tx.reg && tx.has_data)
      a->select = tx.data;

    if (a)
    {
      if (2 == tx.ack)
        a->waits++;
      else if (4 == tx.ack)
        a->faults++;

      if (tx.has_data && !tx.par_ok)
        a->par_err++;

      if (1 == tx.ack && tx.has_data && tx.par_ok)
        a->ok_txs++;

      // What was on the wire between this packet and the last one. A host
      // clocking idle cycles holds SWDIO low, so anything else in there says
      // these two readings are not two packets on one bus - they are two
      // places where somebody else's signal happened to pass the checks.
      //
      // Only runs that START at or after the previous packet ended count: the
      // run straddling that point is the packet's own trailing turnaround,
      // left wherever the parity bit put it.
      if (consumed >= 0)
      {
        for (int k = 0; k < r; k++)
        {
          if ((int64_t)s->pos[k] * 256 >= consumed &&
              (int64_t)s->pos[k] * 256 < pstart && 0 != s->lvl[k])
            a->gap_bad++;
        }
      }
    }

    if (out)
    {
      int64_t p0 = pstart;
      int first = out->count;

      if (out->count >= LOGIC_MAX_BYTES)
      {
        out->truncated = true;

        break;
      }

      out->bytes[out->count] = tx.req;
      out->pos[out->count] = (int)(p0 / 256);
      out->end[out->count] = (int)((p0 + bit_x256 * 8) / 256);

      if (a)
      {
        a->role[out->count] = SWD_R_REQ;
        a->tidx[out->count] = (uint8_t)(a->txs);
      }

      out->count++;

      if (tx.has_data)
      {
        // Where the data phase really started, as the reader found it - not
        // p0 plus twelve or thirteen bit times. Those two numbers differ by
        // however much the phase was re-anchored on the way, and the marks
        // on the trace have to sit where the decoder actually looked.
        int64_t d0 = data_pos;

        for (int k = 0; k < 4 && out->count < LOGIC_MAX_BYTES; k++)
        {
          out->bytes[out->count] = (uint8_t)(tx.data >> (8 * k));
          out->pos[out->count] = (int)((d0 + bit_x256 * 8 * k) / 256);
          out->end[out->count] = (int)((d0 + bit_x256 * 8 * (k + 1)) / 256);

          if (a)
          {
            a->role[out->count] = SWD_R_DATA;
            a->tidx[out->count] = (uint8_t)(a->txs);
          }

          out->count++;
        }
      }

      if (a && a->txs < SWD_MAX_TX)
      {
        tx.first = (uint8_t)first;
        tx.count = (uint8_t)(out->count - first);
        a->tx[a->txs] = tx;
      }
    }

    if (a && a->txs < SWD_MAX_TX)
      a->txs++;

    consumed = end;
  }

  return oks * 4 + txs;
}

//-----------------------------------------------------------------------------
void swd_byte_label(const SwdAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  char reg[12];

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (SWD_R_SWITCH == a->role[idx])
  {
    snprintf(buf, size, "JTAG>SWD");

    return;
  }

  if (0xFF == a->tidx[idx] || a->tidx[idx] >= SWD_MAX_TX)
    return;

  const SwdTx *t = &a->tx[a->tidx[idx]];

  if (SWD_R_REQ == a->role[idx])
  {
    // The select in the analysis is whatever the LAST write left there, and
    // a name has to use what was current when the request went past. The
    // transaction's own bank is not recorded, so the common case - one
    // SELECT for the whole record - is what this names correctly, and a
    // record that switches banks halfway names the first half by the second
    // half's bank. Which is why the bank number is in the fallback text.
    swd_reg_name(t->ap, t->write, t->reg, a->select, reg, sizeof(reg));

    if (1 == t->ack)
      snprintf(buf, size, "%s %s", t->write ? "WR" : "RD", reg);
    else
      snprintf(buf, size, "%s %s %s", t->write ? "WR" : "RD", reg,
          (2 == t->ack) ? "WAIT" : "FAULT");

    return;
  }

  // The 32-bit value goes on the byte that completes it, written once across
  // the four - and the parity says whether to believe it
  if (SWD_R_DATA == a->role[idx] && idx == t->first + t->count - 1)
    snprintf(buf, size, "%08lX%s", (unsigned long)t->data,
        t->par_ok ? "" : " par!");

  (void)v;
}

//-----------------------------------------------------------------------------
void swd_field_label(const SwdAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (SWD_R_SWITCH == a->role[idx])
  {
    snprintf(buf, size, "SW%d", idx & 1);

    return;
  }

  if (SWD_R_REQ == a->role[idx])
  {
    snprintf(buf, size, "REQ");

    return;
  }

  if (SWD_R_DATA != a->role[idx] || 0xFF == a->tidx[idx] ||
      a->tidx[idx] >= SWD_MAX_TX)
    return;

  const SwdTx *t = &a->tx[a->tidx[idx]];

  snprintf(buf, size, "D%d", idx - t->first - 1);
}

//-----------------------------------------------------------------------------
// The four data bytes are one 32-bit register and are shown as one. A request
// stands alone: it is a byte and it means a byte.
void swd_group_at(const SwdAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || SWD_R_DATA != a->role[idx])
    return;

  int s = idx;

  while (s > 0 && SWD_R_DATA == a->role[s - 1])
    s--;

  *start = s;
  *len = 4;
}

//-----------------------------------------------------------------------------
int swd_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  (void)data;

  memset(out, 0, sizeof(*out));
  memset(&g_swd, 0, sizeof(g_swd));
  out->proto = PROTO_SWD;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 6 || period_ns <= 0)
    return 0;

  int64_t base;

  if (g_swd_clock_hz > 0)
  {
    base = (256000000000ll + (int64_t)g_swd_clock_hz * period_ns / 2) /
        ((int64_t)g_swd_clock_hz * period_ns);
    g_swd.told = true;
  }
  else
  {
    base = swd_estimate_bit(scratch, runs);
    g_swd.told = false;
  }

  if (base < SWD_MIN_BIT_SAMPLES * 256)
    return 0;

  // Search a narrow band around it. Both sources of the estimate carry error -
  // a measured frequency has its own, and one worked out of the runs assumes
  // the record holds a single-clock run somewhere - and forty-six bits turn a
  // one percent error into half a bit by the end of a packet.
  //
  // What makes the search safe rather than a fishing expedition is that a
  // transaction has to pass six checks to be counted at all. A wrong bit time
  // does not find fewer packets, it finds none.
  int64_t best_bit = 0;
  int best_score = 0;

  // Outward from the estimate rather than up from the bottom of the band, so
  // that a tie goes to the bit time nearest the one that was measured. Going
  // in order handed a four-percent error the win whenever it merely matched.
  for (int i = 0; i <= 2 * SWD_TRIM_STEPS; i++)
  {
    int k = (0 != (i & 1)) ? -((i + 1) / 2) : (i / 2);
    int64_t bit = base * (100 + k) / 100;

    if (bit < SWD_MIN_BIT_SAMPLES * 256)
      continue;

    int score = swd_pass(scratch, runs, size, bit, NULL, NULL);

    if (score > best_score)
    {
      best_score = score;
      best_bit = bit;
    }
  }

  if (0 == best_score || 0 == best_bit)
    return 0;

  swd_pass(scratch, runs, size, best_bit, &g_swd, out);

  if (0 == out->count)
    return 0;

  g_swd.clock_hz = (int)(256000000000ll / (best_bit * period_ns));

  out->rate = g_swd.clock_hz;
  out->idle_high = (0 != scratch->lvl[runs - 1]);
  out->burst_start = (g_swd.resets > 0 || g_swd.switches > 0);
  out->errors = g_swd.par_err + g_swd.faults;

  // What may be claimed automatically, and this is the part that took two
  // goes to get right.
  //
  // Counting transactions is not enough, and the arithmetic says why. A
  // candidate edge passes the request checks one time in eight and lands a
  // legal acknowledgement three times in eight - so about one edge in
  // twenty-one heads a WAIT or a FAULT "transaction" purely by chance, and a
  // console record has hundreds of edges. That is not a small effect: pointed
  // at 115200 baud of ASCII, the first version of this decoder found nine
  // transactions and announced a debug session.
  //
  // Two things fix it. The first is to count only the transactions that were
  // acknowledged AND whose thirty-two data bits carried the right parity -
  // all six checks, not four. The second is to ask what was BETWEEN them:
  // between two packets a host clocks idle cycles with SWDIO held low, so a
  // gap with traffic in it means the two readings came out of somebody else's
  // signal. A UART's gaps are full of its own message, and that is what
  // turns it down.
  //
  // A JTAG-to-SWD switch sequence needs neither: sixteen prescribed bits
  // behind fifty ones is a signature nothing else here produces.
  g_swd.sure = (g_swd.switches > 0 && g_swd.txs >= 1) ||
      (g_swd.ok_txs >= 2 && 0 == g_swd.gap_bad);

  out->ambiguous = !g_swd.sure;

  char rate[12];
  int hz = g_swd.clock_hz;

  if (hz >= 1000000)
    snprintf(rate, sizeof(rate), "%d.%02dM", hz / 1000000,
        (hz % 1000000) / 10000);
  else
    snprintf(rate, sizeof(rate), "%d.%dk", hz / 1000, (hz % 1000) / 100);

  // '~' where the rate was worked out of the data rather than read on SWCLK,
  // the same mark the SPI decoder uses for the same doubt. What follows it is
  // what actually happened on the bus, in the order anyone debugging asks:
  // did the debugger attach, how many transactions, and did any of them go
  // wrong.
  char tail[16];

  tail[0] = 0;

  if (g_swd.faults > 0 || g_swd.waits > 0 || g_swd.par_err > 0)
    snprintf(tail, sizeof(tail), " %s%s%s",
        g_swd.faults ? "F" : "", g_swd.waits ? "W" : "",
        g_swd.par_err ? "par!" : "");

  snprintf(out->info, sizeof(out->info), "SWD %s%s %s%dtx%s",
      g_swd.told ? "" : "~", rate, g_swd.switches ? "sw " :
      (g_swd.resets ? "rst " : ""), g_swd.txs, tail);

  return out->count;
}
