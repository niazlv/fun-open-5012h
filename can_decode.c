/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CAN 2.0A/2.0B decoder (classic CAN, up to 1 Mbit/s).
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
 * A frame:
 *   SOF | 11-bit id | RTR | IDE | r0 | DLC(4) | data | CRC(15) | del | ACK |
 *   del | EOF(7) | IFS(3)
 * and with IDE recessive, 18 more identifier bits and two more reserved ones
 * in the middle of it.
 *
 * The whole thing is confirmed by its own CRC-15, which is why this decoder
 * can run ahead of the generic ones without stealing their records: a frame
 * that checks out is proof, and a record with no such frame in it is refused
 * outright.
 *
 * CAN FD is recognised (its FDF bit is recessive where classic CAN has a
 * reserved dominant one) and NOT decoded: its data phase changes bit rate
 * mid-frame and its CRC is a different polynomial over a different span.
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
// of six to ten bit times cannot happen in classic CAN, and one that does
// settles the question before a single frame is parsed.
#define CAN_GAP_MIN_BITS        11
#define CAN_SOF_IDLE_BITS        7   // EOF alone; anchors the start of a frame

// A dominant run is capped at five by stuffing too, except for error flags -
// six, and up to twelve where they overlap
#define CAN_DOM_MAX_BITS        12

/*- Types -------------------------------------------------------------------*/
// Bit reader over the run-length split, with the destuffing state folded in.
// Reading bits out of runs rather than off a sample clock is what keeps a
// 130-bit frame in phase: every edge re-anchors it.
typedef struct
{
  const LogicScratch *s;
  int   runs;
  int   bit_x256;      // bit time in samples, x256
  int   rec_lvl;       // the SAMPLE level that means recessive
  int   r, k, n;       // current run, bits taken from it, bits in it
  int   pos, end;      // sample span of the bit just returned
  bool  ended;
  int   last, same;    // destuffing: last bit value and how many in a row
  bool  stuff;         // stuffing is in force (SOF through the CRC field)
  bool  stuff_err;
} CanRd;

// One frame's worth of bytes, held back until the frame is known to be one
typedef struct
{
  uint8_t by[16];
  int     pos[16];
  int     end[16];
  int     n;
} CanBuf;

enum { CAN_NO = 0, CAN_OK, CAN_CUT };

/*- Variables ---------------------------------------------------------------*/
static CanAnalysis g_can;

// Every rate anyone actually runs. 33.3k is the single-wire automotive one,
// 83.3k shows up on European vehicle buses, 800k and 1M on machinery.
static const int g_can_rates[] =
{
  10000, 20000, 33333, 50000, 62500, 83333, 100000, 125000,
  250000, 500000, 800000, 1000000,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// CRC-15/CAN, polynomial 0x4599, fed MSB-first over the destuffed bits from
// the start of frame through the last data bit
static void can_crc_bit(uint16_t *crc, int bit)
{
  uint16_t inv = (uint16_t)(((*crc >> 14) & 1) ^ (bit & 1));

  *crc = (uint16_t)((*crc << 1) & 0x7FFF);

  if (inv)
    *crc ^= 0x4599;
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
      rd->n = can_run_bits(rd, rd->r);
    }
  }

  if (rd->r >= rd->runs)
  {
    rd->ended = true;
    return false;
  }

  *bit = (rd->s->lvl[rd->r] == rd->rec_lvl) ? 1 : 0;
  rd->pos = rd->s->pos[rd->r] + (int)((int64_t)rd->k * rd->bit_x256 / 256);
  rd->end = rd->s->pos[rd->r] + (int)((int64_t)(rd->k + 1) * rd->bit_x256 / 256);
  rd->k++;

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
static void can_put(CanBuf *buf, uint8_t v, int pos, int end)
{
  if (buf->n >= (int)(sizeof(buf->by) / sizeof(buf->by[0])))
    return;

  buf->by[buf->n] = v;
  buf->pos[buf->n] = pos;
  buf->end[buf->n] = end;
  buf->n++;
}

//-----------------------------------------------------------------------------
// Read one frame, starting at the bit the reader is positioned on. Returns
// CAN_NO when the bits are not a frame, CAN_CUT when the record ran out inside
// one, CAN_OK when a whole frame came out - f->crc_ok says whether it was also
// a correct one.
static int can_frame(CanRd *rd, CanFrame *f, CanBuf *buf)
{
  uint16_t crc = 0, rx_crc = 0;
  uint32_t id = 0;
  int bit, i, k;
  int id_start = 0, id_end = 0, dlc_start = 0, dlc_end = 0;
  int crc_start = 0, crc_end = 0;
  int dlc = 0, nd;
  uint8_t d[8];
  int dpos[8], dend[8];

  memset(f, 0, sizeof(*f));
  buf->n = 0;

  rd->last = 1;       // the idle before the frame is recessive, and is not
  rd->same = 0;       // counted: the stuff counter starts at the SOF
  rd->stuff = true;
  rd->stuff_err = false;

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

  int rtr_srr, ide;

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
      f->fd = true;
      return CAN_NO;
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
    // Recessive here is the flag that says the rest of this frame is written
    // in a format this decoder does not read
    f->fd = !f->ext;
    return CAN_NO;
  }

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

  for (k = 0; k < nd; k++)
  {
    int v = 0;

    for (i = 0; i < 8; i++)
    {
      if (!can_bit(rd, &bit))
        return CAN_CUT;

      if (0 == i)
        dpos[k] = rd->pos;

      dend[k] = rd->end;
      can_crc_bit(&crc, bit);
      v = (v << 1) | bit;
    }

    d[k] = (uint8_t)v;
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

  f->id = id;
  f->crc_ok = (crc == rx_crc);

  // The identifier and the CRC are fields, not bytes, so their bytes divide
  // the span the field actually occupies on the trace. The data bytes are
  // eight bits each and get their own.
  int idn = f->ext ? 4 : 2;

  for (k = 0; k < idn; k++)
    can_put(buf, (uint8_t)(id >> (8 * (idn - 1 - k))),
        id_start + (int)((int64_t)(id_end - id_start) * k / idn),
        id_start + (int)((int64_t)(id_end - id_start) * (k + 1) / idn));

  can_put(buf, (uint8_t)dlc, dlc_start, dlc_end);

  for (k = 0; k < nd; k++)
    can_put(buf, d[k], dpos[k], dend[k]);

  can_put(buf, (uint8_t)(rx_crc >> 8), crc_start, (crc_start + crc_end) / 2);
  can_put(buf, (uint8_t)(rx_crc & 0xFF), (crc_start + crc_end) / 2, crc_end);

  return CAN_OK;
}

//-----------------------------------------------------------------------------
// Cheap refusal before any parsing: at this bit time, do the runs even look
// like CAN? Five bits is the most stuffing allows and eleven the least an
// interframe gap can be, so nothing lands in between. A UART record with six
// zero bits in a byte is thrown out here rather than after a thousand bit
// reads. Boundary runs are cut by the ends of the record and prove nothing.
static bool can_runs_plausible(const LogicScratch *s, int runs, int bit_x256,
    int rec_lvl)
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

    if (q < 1)
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
    LogicResult *out, CanAnalysis *a)
{
  CanRd rd;
  CanBuf buf;
  CanFrame f;
  int ok = 0;

  memset(&rd, 0, sizeof(rd));
  rd.s = s;
  rd.runs = runs;
  rd.bit_x256 = bit_x256;
  rd.rec_lvl = rec_lvl;

  for (int r = 1; r < runs; r++)
  {
    if (s->lvl[r] == rec_lvl)
      continue;   // a frame starts with a dominant bit

    // ...and only after the line has been recessive for longer than any run
    // inside a frame can be. That is what makes this a start of frame rather
    // than some dominant bit in the middle of one.
    if ((int64_t)s->len[r - 1] * 256 < (int64_t)bit_x256 * CAN_SOF_IDLE_BITS)
      continue;

    rd.r = r;
    rd.k = 0;
    rd.n = can_run_bits(&rd, r);
    rd.ended = false;

    int st = can_frame(&rd, &f, &buf);

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
      if (out->count + buf.n > LOGIC_MAX_BYTES ||
          (a && a->frames >= CAN_MAX_FRAMES))
      {
        out->truncated = true;
        break;
      }

      if (a)
      {
        f.first = out->count;
        f.count = buf.n;
        a->frame[a->frames] = f;
      }

      for (int i = 0; i < buf.n; i++)
      {
        int idx = out->count;
        int idn = f.ext ? 4 : 2;

        out->bytes[idx] = buf.by[i];
        out->pos[idx] = buf.pos[i];
        out->end[idx] = buf.end[i];

        if (a)
        {
          a->fidx[idx] = (uint8_t)a->frames;
          a->role[idx] = (i < idn) ? CAN_R_ID :
              (i == idn) ? CAN_R_DLC :
              (i < buf.n - 2) ? CAN_R_DATA :
              (i == buf.n - 2) ? CAN_R_CRC : CAN_R_ACK;
        }

        out->count++;
      }

      if (a)
      {
        a->frames++;

        if (f.crc_ok)
          a->crc_ok++;
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

  if (rate > 9999999)
    rate = 9999999;

  if (rate >= 1000000)
    snprintf(buf, size, "%dM", rate / 1000000);
  else if (0 == rate % 1000)
    snprintf(buf, size, "%dk", rate / 1000);
  else
    snprintf(buf, size, "%d.%dk", rate / 1000, (rate % 1000) / 100);
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
// the screen. The CRC's two bytes are NOT grouped: what is worth reading
// there is not the number but whether it checked out and whether anyone
// acknowledged the frame, and those are two different answers.
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
int can_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
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

  for (int p = 0; p < 2; p++)
  {
    int rec_lvl = (0 == p) ? scratch->lvl[longest] : !scratch->lvl[longest];

    for (unsigned i = 0; i < sizeof(g_can_rates) / sizeof(g_can_rates[0]); i++)
    {
      int64_t bit_x256 = (int64_t)256 * 1000000000 /
          ((int64_t)g_can_rates[i] * period_ns);

      if (bit_x256 < CAN_MIN_SAMPLES_PER_BIT * 256)
        continue;   // too fast to sample: fewer than three points per bit

      if (bit_x256 * CAN_MIN_RECORD_BITS > (int64_t)size * 256)
        continue;   // too slow: not even one frame fits in the record

      if (!can_runs_plausible(scratch, runs, (int)bit_x256, rec_lvl))
        continue;

      int ok = can_scan(scratch, runs, (int)bit_x256, rec_lvl, NULL, NULL);

      if (ok > best_ok)
      {
        best_ok = ok;
        best_bit = (int)bit_x256;
        best_lvl = rec_lvl;
      }
    }

    if (best_ok > 0)
      break;    // the line rests where it rests; no need to try it upside down
  }

  // Nothing here passed a CRC-15. A frame that does is proof; a frame that
  // does not is a guess, and this decoder runs ahead of the generic ones
  // precisely because it never guesses.
  if (0 == best_ok)
    return 0;

  memset(&g_can, 0, sizeof(g_can));
  g_can.rate = 0;

  can_scan(scratch, runs, best_bit, best_lvl, out, &g_can);

  if (0 == out->count)
    return 0;

  int rate = (int)((int64_t)256 * 1000000000 / ((int64_t)best_bit * period_ns));

  // Report the standard rate that was tried, not the arithmetic back out of
  // the rounded bit time
  for (unsigned i = 0; i < sizeof(g_can_rates) / sizeof(g_can_rates[0]); i++)
  {
    if (rate > g_can_rates[i] * 9 / 10 && rate < g_can_rates[i] * 11 / 10)
    {
      rate = g_can_rates[i];
      break;
    }
  }

  g_can.rate = rate;
  out->rate = rate;
  out->idle_high = (best_lvl != 0);
  out->burst_start = true;   // every frame here was found behind an idle line

  char rs[12];
  can_rate_str(rate, rs, sizeof(rs));

  bool nak = false;

  for (int i = 0; i < g_can.frames; i++)
  {
    if (!g_can.frame[i].ack)
      nak = true;
  }

  if (1 == g_can.frames)
  {
    const CanFrame *f = &g_can.frame[0];

    if (f->rtr)
      snprintf(out->info, sizeof(out->info), "CAN %s ID %X RTR%s", rs,
          (unsigned)f->id, nak ? " NAK" : "");
    else
      snprintf(out->info, sizeof(out->info), "CAN %s ID %X %dB%s", rs,
          (unsigned)f->id, f->dlc > 8 ? 8 : f->dlc, nak ? " NAK" : "");
  }
  else
  {
    snprintf(out->info, sizeof(out->info), "CAN %s %d frames%s", rs,
        g_can.frames, nak ? " NAK" : "");
  }

  return out->count;
}
