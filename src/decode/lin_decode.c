/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * LIN - Local Interconnect Network. The cheap single-wire bus behind every
 * door module, mirror motor, rain sensor and seat heater in a modern car: one
 * master, a handful of slaves, one wire and ground, 20 kbit/s at the most.
 *
 * A frame is a header from the master and a response from whichever slave
 * owns the identifier:
 *
 *   BREAK | delimiter | SYNC 0x55 | PID | data 1..8 | checksum
 *
 * Everything after the break is ordinary 8N1, so on the wire this is a UART.
 * What makes it LIN is the two fields in front of the traffic, and both of
 * them are gifts to a decoder:
 *
 *   - the BREAK is at least 13 dominant bits, and 8N1 CANNOT produce that.
 *     A start bit and eight zero data bits is nine, and then the stop bit is
 *     obliged to let the line up again. A break is therefore not something a
 *     serial link falls into by accident - it is the one field on this bus
 *     that identifies it outright.
 *
 *   - the SYNC byte is 0x55, whose every bit alternates, so on the wire it is
 *     nine runs of exactly one bit each. The distance from the first falling
 *     edge to the last is eight bit times, which is how a slave learns the
 *     master's rate - and how this decoder learns it, with no baud search and
 *     no standard-rate table involved.
 *
 * On top of that the PID carries two parity bits over its six identifier
 * bits, and the frame ends in a checksum. Neither is needed to FIND the
 * frame; both are needed to trust it, and both are reported.
 *
 * The checksum comes in two flavours and a bus uses one of them: the classic
 * one (LIN 1.3) sums the data bytes, the enhanced one (LIN 2.x) sums the PID
 * as well. Both are ones-complement sums - the carry is folded back in - and
 * inverted at the end. The diagnostic identifiers 0x3C and 0x3D are classic
 * even on a 2.x bus, which is a rule and not a guess.
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
// The standard says 13 dominant bits from the master and 11 for a receiver to
// call it a break. Ten here, because the master's 13 are counted at ITS clock
// and this decoder measures at the slave's; nothing legitimate lands between
// nine and ten, which is the only thing that matters.
#define LIN_BREAK_BITS        10

// The bus runs from 1 to 20 kbit/s. The window is a little wider at both ends
// so that a master running off a cheap resonator still reads.
#define LIN_RATE_MIN         500
#define LIN_RATE_MAX       25000

// Below this the bit is not sampled well enough to place a stop bit
#define LIN_MIN_BIT_SAMPLES    3

// How far the sync byte's nine runs may sit off one bit, in percent. They are
// measured against a bit time that came out of those same runs, so this is
// about the shape being even and not about the rate being right.
#define LIN_SYNC_TOL_PCT      35

// Recessive time between two bytes of one frame. The header is 34 bits and
// the response may lag it, so this is generous; what ends a frame properly is
// the next break, and this is for the record that does not hold one.
#define LIN_MAX_GAP_BITS      40

#define LIN_MAX_DATA           8

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_lin   (g_logic_analysis.lin)

// The transport-layer services that ride on identifiers 0x3C and 0x3D. A
// response uses the request's service id plus 0x40, so both directions are
// named from one table.
static const struct { uint8_t sid; const char *name; } g_lin_sid[] =
{
  { 0xB0, "AsgNAD" },   { 0xB1, "AsgFrmId" }, { 0xB2, "ReadById" },
  { 0xB3, "CondNAD" },  { 0xB4, "DataDump" }, { 0xB5, "SaveCfg" },
  { 0xB6, "AsgFrmRng" },
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The two parity bits a protected identifier carries over its six identifier
// bits. Not a checksum - four bits of the identifier go into each - but it is
// what catches a header read at the wrong rate before its data is believed.
static uint8_t lin_pid_of(uint8_t id)
{
  uint8_t b0 = (id >> 0) & 1, b1 = (id >> 1) & 1, b2 = (id >> 2) & 1;
  uint8_t b3 = (id >> 3) & 1, b4 = (id >> 4) & 1, b5 = (id >> 5) & 1;
  uint8_t p0 = (uint8_t)(b0 ^ b1 ^ b2 ^ b4);
  uint8_t p1 = (uint8_t)(!(b1 ^ b3 ^ b4 ^ b5));

  return (uint8_t)((id & 0x3F) | (p0 << 6) | (p1 << 7));
}

//-----------------------------------------------------------------------------
// Ones-complement sum of the bytes, inverted. `enhanced` folds the PID in as
// well, which is what LIN 2.x does everywhere except on the two diagnostic
// identifiers.
static uint8_t lin_checksum(const uint8_t *d, int n, uint8_t pid, bool enhanced)
{
  unsigned sum = enhanced ? pid : 0u;

  for (int i = 0; i < n; i++)
  {
    sum += d[i];

    if (sum > 0xFF)
      sum -= 0xFF;      // the carry goes back in at the bottom
  }

  return (uint8_t)(~sum);
}

//-----------------------------------------------------------------------------
// Identifiers 0x3C and 0x3D are the transport layer, and they are classic on
// every bus. Everything else follows whatever the bus itself uses.
static bool lin_is_diag(uint8_t id)
{
  return (0x3C == id || 0x3D == id);
}

//-----------------------------------------------------------------------------
static const char *lin_sid_name(uint8_t sid)
{
  // A positive response carries the request's service id plus 0x40
  uint8_t req = (sid >= 0xF0 && sid <= 0xF7) ? (uint8_t)(sid - 0x40) : sid;

  for (unsigned i = 0; i < sizeof(g_lin_sid) / sizeof(g_lin_sid[0]); i++)
  {
    if (g_lin_sid[i].sid == req)
      return g_lin_sid[i].name;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
const LinAnalysis *lin_analysis(void)
{
  return &g_lin;
}

//-----------------------------------------------------------------------------
void lin_byte_label(const LinAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  const LinFrame *f = &a->frame[a->fidx[idx] % LIN_MAX_FRAMES];

  switch (a->role[idx])
  {
    case LIN_R_SYNC:
      snprintf(buf, size, "SYNC");
      break;

    case LIN_R_PID:
      // The identifier is what a person reads off a LIN bus - the protected
      // one with its parity bits on top is the byte above. A '!' where the
      // parity does not check out, because then the six bits under it are not
      // an identifier, they are what this record made of one.
      snprintf(buf, size, "ID=%02X%s", f->id, f->parity_ok ? "" : "!");
      break;

    case LIN_R_DATA:
    {
      int di = idx - f->first - 2;   // 0-based among the data bytes

      // On the two diagnostic identifiers the data is not eight opaque bytes:
      // it is the transport layer, and its first three have names
      if (lin_is_diag(f->id))
      {
        if (0 == di)
          snprintf(buf, size, "NAD %02X", v);
        else if (1 == di)
          snprintf(buf, size, "PCI %02X", v);
        else if (2 == di)
        {
          const char *sid = lin_sid_name(v);

          if (0x7F == v)
            snprintf(buf, size, "NegResp");
          else if (sid)
            snprintf(buf, size, "%s", sid);
          else
            snprintf(buf, size, "SID %02X", v);
        }
        else
        {
          snprintf(buf, size, "D%d", di - 2);
        }

        break;
      }

      snprintf(buf, size, "D%d", di + 1);
      break;
    }

    case LIN_R_CSUM:
      snprintf(buf, size, f->csum_ok ? "CHK" : "CHK!");
      break;

    default:
      break;
  }
}

//-----------------------------------------------------------------------------
// One 8N1 byte whose start bit begins at sample position `start_x256`.
// Returns the value, or -1 when the stop bit is not where it has to be.
static int lin_byte(const uint8_t *data, int size, int offset, int mid,
    int dom, int64_t start_x256, int64_t bit_x256)
{
  int value = 0;

  for (int b = 0; b < 8; b++)   // LSB first
  {
    int64_t center = start_x256 + bit_x256 * (3 + 2 * b) / 2;
    int i = (int)(center / 256);

    if (i >= size)
      return -1;

    int lvl = (sample_at(data, size, offset, i) > mid) ? 1 : 0;

    if (lvl != dom)   // recessive is a one
      value |= (1 << b);
  }

  int64_t stop = start_x256 + bit_x256 * 19 / 2;
  int si = (int)(stop / 256);

  if (si >= size)
    return -1;

  if (((sample_at(data, size, offset, si) > mid) ? 1 : 0) == dom)
    return -1;        // the stop bit must let the line up

  return value;
}

//-----------------------------------------------------------------------------
// The run index of the next start bit at or after sample `from`, or -1 when
// the bus stays quiet to the end of the record
static int lin_next_start(const LogicScratch *s, int runs, int r, int dom,
    int from)
{
  for (; r < runs; r++)
  {
    if (s->lvl[r] == dom && s->pos[r] >= from)
      return r;
  }

  return -1;
}

//-----------------------------------------------------------------------------
// One pass over the record at a given dominant level. Returns the number of
// frames found; out and a may be NULL for a counting-only pass.
static int lin_scan(const uint8_t *data, int size, int offset, int mid,
    const LogicScratch *s, int runs, int period_ns, int dom, LogicResult *out,
    LinAnalysis *a)
{
  int found = 0;
  // Once a frame has settled which checksum a bus uses, the next frame is
  // tried that way first. A bus does not mix them, and an eight-bit checksum
  // agreeing with the wrong hypothesis once in 256 tries is worth designing
  // against rather than hoping about.
  int variant = -1;   // 0 = classic, 1 = enhanced

  for (int r = 1; r + 11 < runs; r++)
  {
    if (s->lvl[r] != dom)
      continue;

    // Runs alternate, so from a dominant run at r the sync byte's nine
    // one-bit runs are r+2 (its start bit) through r+10 (its bit 7), and the
    // distance between the first and the last of them is eight bit times.
    // That is the whole of the rate detection: the bus tells its own rate,
    // once per frame, and no candidate list is involved.
    int64_t span = s->pos[r + 10] - s->pos[r + 2];

    if (span <= 0)
      continue;

    int64_t bit_x256 = span * 256 / 8;

    if (bit_x256 < LIN_MIN_BIT_SAMPLES * 256)
      continue;

    int64_t rate = 256000000000ll / (bit_x256 * period_ns);

    if (rate < LIN_RATE_MIN || rate > LIN_RATE_MAX)
      continue;

    // Every one of those nine runs is one bit. This is what a run of ordinary
    // traffic cannot imitate for nine runs together, and it is checked before
    // anything is read.
    bool even = true;

    for (int k = 2; k <= 10 && even; k++)
    {
      int64_t len = (int64_t)s->len[r + k] * 256;

      even = (len * 100 >= bit_x256 * (100 - LIN_SYNC_TOL_PCT)) &&
          (len * 100 <= bit_x256 * (100 + LIN_SYNC_TOL_PCT));
    }

    if (!even)
      continue;

    // ...and in front of them, the field no 8N1 line can send
    if ((int64_t)s->len[r] * 256 < bit_x256 * LIN_BREAK_BITS)
      continue;

    if ((int64_t)s->len[r + 1] * 512 < bit_x256)
      continue;      // the delimiter is at least half a bit of recessive

    // Read the sync byte rather than assume it: the rate came out of its
    // edges, so its VALUE is not evidence, but its stop bit is - and a byte
    // that cannot be sampled here means the record ends inside the header.
    int sync = lin_byte(data, size, offset, mid, dom,
        (int64_t)s->pos[r + 2] * 256, bit_x256);

    if (0x55 != sync)
      continue;

    int64_t frame_bits = bit_x256 * 10;
    int cursor = (int)(((int64_t)s->pos[r + 2] * 256 + frame_bits) / 256);
    int rr = lin_next_start(s, runs, r + 10, dom,
        cursor - (int)(bit_x256 / 512));

    if (rr < 0)
      continue;      // the header ends the record: no identifier to be had

    int pid = lin_byte(data, size, offset, mid, dom,
        (int64_t)s->pos[rr] * 256, bit_x256);

    if (pid < 0)
      continue;

    // The response: everything up to the next break, and no more than eight
    // data bytes and a checksum. Where the frame ends is not a guess - the
    // bus goes quiet, or the master starts the next header with a break.
    uint8_t resp[LIN_MAX_DATA + 1];
    int rpos[LIN_MAX_DATA + 1], rend[LIN_MAX_DATA + 1];
    int nresp = 0;
    bool cut = false;
    int prev = rr;

    cursor = (int)(((int64_t)s->pos[rr] * 256 + frame_bits) / 256);

    while (nresp <= LIN_MAX_DATA)
    {
      int nr = lin_next_start(s, runs, prev + 1, dom,
          cursor - (int)(bit_x256 / 512));

      if (nr < 0)
      {
        // Nothing more on the wire to the end of the record. Whether that is
        // the frame ending or the record ending is the difference between "no
        // slave answered" and "the timebase was too fast", and the two must
        // not be confused: one is a fault on the bus and the other is a fault
        // in how it was looked at. The bus staying quiet for longer than a
        // frame's own gap is the frame ending.
        cut = ((int64_t)(size - cursor) * 256 <= bit_x256 * LIN_MAX_GAP_BITS);
        break;
      }

      if ((int64_t)(s->pos[nr] - cursor) * 256 > bit_x256 * LIN_MAX_GAP_BITS)
        break;               // the bus went quiet: this frame is over

      if ((int64_t)s->len[nr] * 256 >= bit_x256 * LIN_BREAK_BITS)
        break;               // that is the next frame's break, not a byte

      int v = lin_byte(data, size, offset, mid, dom,
          (int64_t)s->pos[nr] * 256, bit_x256);

      if (v < 0)
      {
        cut = true;          // a byte the record did not hold all of
        break;
      }

      resp[nresp] = (uint8_t)v;
      rpos[nresp] = s->pos[nr];
      rend[nresp] = (int)(((int64_t)s->pos[nr] * 256 + frame_bits) / 256);
      nresp++;

      prev = nr;
      cursor = rend[nresp - 1];
    }

    // The last byte of a response is its checksum and the rest is data. There
    // is no length search here and there does not need to be one: the bytes
    // between the identifier and the end of the frame ARE the response, and
    // guessing at shorter readings would only give an eight-bit checksum more
    // chances to agree with the wrong one.
    int ndata = (nresp > 0) ? nresp - 1 : 0;
    uint8_t rx = (nresp > 0) ? resp[nresp - 1] : 0;
    uint8_t id = (uint8_t)(pid & 0x3F);
    bool diag = lin_is_diag(id);
    bool csum_ok = false;
    bool enhanced = false;

    if (nresp > 0)
    {
      // Classic on the diagnostic identifiers, whatever the rest of the bus
      // does. Elsewhere the variant the previous frame settled is tried first.
      bool first_try = diag ? false : (variant != 0);

      if (rx == lin_checksum(resp, ndata, (uint8_t)pid, first_try))
      {
        csum_ok = true;
        enhanced = first_try;
      }
      else if (!diag &&
          rx == lin_checksum(resp, ndata, (uint8_t)pid, !first_try))
      {
        csum_ok = true;
        enhanced = !first_try;
      }
    }

    if (csum_ok && !diag)
      variant = enhanced ? 1 : 0;

    found++;

    if (!out || !a)
    {
      r = (nresp > 0) ? prev : rr;
      continue;
    }

    if (a->frames >= LIN_MAX_FRAMES ||
        out->count + 2 + nresp > LOGIC_MAX_BYTES)
    {
      out->truncated = true;
      break;
    }

    LinFrame *f = &a->frame[a->frames];

    memset(f, 0, sizeof(*f));
    f->pid = (uint8_t)pid;
    f->id = id;
    f->ndata = (uint8_t)ndata;
    f->csum = rx;
    f->parity_ok = (lin_pid_of(id) == (uint8_t)pid);
    f->csum_ok = csum_ok;
    f->enhanced = enhanced;
    f->no_resp = (0 == nresp) && !cut;
    f->cut = cut;
    f->first = (uint8_t)out->count;
    f->count = (uint8_t)(2 + nresp);

    // The sync byte and the identifier, then the response
    out->bytes[out->count] = 0x55;
    out->pos[out->count] = s->pos[r + 2];
    out->end[out->count] =
        (int)(((int64_t)s->pos[r + 2] * 256 + frame_bits) / 256);
    a->role[out->count] = LIN_R_SYNC;
    a->fidx[out->count] = (uint8_t)a->frames;
    out->count++;

    out->bytes[out->count] = (uint8_t)pid;
    out->pos[out->count] = s->pos[rr];
    out->end[out->count] =
        (int)(((int64_t)s->pos[rr] * 256 + frame_bits) / 256);
    a->role[out->count] = LIN_R_PID;
    a->fidx[out->count] = (uint8_t)a->frames;
    out->count++;

    for (int i = 0; i < nresp; i++)
    {
      out->bytes[out->count] = resp[i];
      out->pos[out->count] = rpos[i];
      out->end[out->count] = rend[i];
      a->role[out->count] = (i == nresp - 1) ? LIN_R_CSUM : LIN_R_DATA;
      a->fidx[out->count] = (uint8_t)a->frames;
      out->count++;
    }

    a->frames++;
    a->rate = (int)rate;

    if (csum_ok)
      a->csum_ok++;
    else if (nresp > 0)
      out->errors++;

    if (!f->parity_ok)
      out->errors++;

    if (cut)
      out->overrun = true;

    // Carry on after the frame; the loop's own step takes it past
    r = (nresp > 0) ? prev : rr;
  }

  return found;
}

//-----------------------------------------------------------------------------
// The nearest rate anybody actually configures. A rate measured off eight bit
// times of one sync byte lands a percent or two out, and "LIN 19200" is what
// the bus is; "LIN 19417" is what this record's sampling did to it.
static int lin_snap_rate(int rate)
{
  static const int common[] =
  {
    1200, 2400, 4800, 9600, 10417, 19200, 20000,
  };

  for (unsigned i = 0; i < sizeof(common) / sizeof(common[0]); i++)
  {
    int d = rate - common[i];

    if (d < 0)
      d = -d;

    if (d * 100 <= common[i] * 3)
      return common[i];
  }

  return rate;
}

//-----------------------------------------------------------------------------
int lin_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_LIN;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 12 || period_ns <= 0)
    return 0;

  // The bus rests recessive and a break pulls it dominant, which on a LIN
  // transceiver's bus pin means high and low respectively. An inverting
  // buffer, or a probe on the wrong side of one, costs a second pass.
  int best = 0, best_dom = 0;

  for (int p = 0; p < 2; p++)
  {
    int n = lin_scan(data, size, offset, mid, scratch, runs, period_ns, p,
        NULL, NULL);

    if (n > best)
    {
      best = n;
      best_dom = p;
    }
  }

  if (0 == best)
    return 0;

  memset(&g_lin, 0, sizeof(g_lin));
  lin_scan(data, size, offset, mid, scratch, runs, period_ns, best_dom, out,
      &g_lin);

  if (0 == out->count)
    return 0;

  g_lin.rate = lin_snap_rate(g_lin.rate);
  out->rate = g_lin.rate;
  out->idle_high = (0 == best_dom);
  out->burst_start = true;   // a frame is found from its own break

  // The break and the sync between them are the identification, and they are
  // already spent by the time the frame is read. What is left to check is the
  // parity over the identifier: a header read at a rate that is a little
  // wrong still produces a byte, and the parity is what says the six bits
  // under it are an identifier rather than the arithmetic's own doing.
  bool sure = false;

  for (int i = 0; i < g_lin.frames; i++)
  {
    if (g_lin.frame[i].parity_ok)
      sure = true;
  }

  out->ambiguous = !sure;

  bool any_bad = false, any_none = false, any_par = false;

  for (int i = 0; i < g_lin.frames; i++)
  {
    const LinFrame *f = &g_lin.frame[i];

    if (!f->parity_ok)
      any_par = true;
    else if (f->no_resp)
      any_none = true;
    else if (!f->csum_ok && !f->cut)
      any_bad = true;
  }

  // A header nobody answered is a fact about the BUS, not about the record:
  // the master asked for that identifier and no slave on the wire owns it.
  // It is the most useful thing this decoder can say when something has been
  // unplugged, so it goes in the header rather than being left to be inferred
  // from a frame that is two bytes long.
  const char *note = any_par ? " PID!" : any_bad ? " CHK!" :
      any_none ? " no resp" : "";

  // Which checksum the bus uses, said out loud in both directions. Leaving
  // the enhanced one silent and only naming the classic would make "nothing
  // here" mean two different things - a LIN 2.x bus, and a decoder that never
  // worked it out - and a reader cannot tell those apart from an absence.
  // Nothing is printed only when nothing was established: no checksum agreed,
  // or the only frames were the diagnostic identifiers, where classic is
  // mandatory and therefore says nothing about the bus at all.
  const char *cls = "";

  for (int i = 0; i < g_lin.frames; i++)
  {
    if (g_lin.frame[i].csum_ok && !lin_is_diag(g_lin.frame[i].id))
      cls = g_lin.frame[i].enhanced ? " enh" : " cls";
  }

  if (1 == g_lin.frames)
  {
    const LinFrame *f = &g_lin.frame[0];

    if (f->no_resp)
      snprintf(out->info, sizeof(out->info), "LIN %d ID=%02X%s",
          g_lin.rate, f->id, note);
    else
      snprintf(out->info, sizeof(out->info), "LIN %d ID=%02X %dB%s%s",
          g_lin.rate, f->id, f->ndata, note, cls);
  }
  else
  {
    snprintf(out->info, sizeof(out->info), "LIN %d %d frames%s%s",
        g_lin.rate, g_lin.frames, note, cls);
  }

  return out->count;
}
