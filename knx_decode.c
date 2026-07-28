/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * KNX TP1 - the twisted pair that runs the lights, blinds and heating in a
 * building. 9600 bit/s, and it is NOT Manchester, whatever it gets grouped
 * with. The line code is pulse PRESENCE:
 *
 *   0   an active pulse, about 35 us, at the START of the bit period
 *   1   nothing at all - the bus is left where it is
 *
 * So a bit time carries either a short pulse or silence, and the silence is
 * indistinguishable from the bus being idle except by counting bit periods
 * from the last pulse. That is the opposite of a self-clocking code, and it
 * is why this needs a decoder of its own rather than a setting on the
 * Manchester one: there is no transition in the middle of a bit to step to.
 *
 * A character is 13 bit times: a start bit (always 0, so always a pulse),
 * eight data bits least significant first, an EVEN parity bit, a stop bit,
 * and two bit times of gap before the next one may begin. The start bit is
 * what a character is found from, and the parity is what says it was read
 * correctly - one per character, which is eight checks in a short telegram.
 *
 * Above that, a telegram:
 *
 *   control | source (2) | destination (2) | DAF+length | TPCI/APCI... | FCS
 *
 * The source is always an individual address, area.line.device, packed 4/4/8.
 * The destination is either another of those or a GROUP address - which is
 * the whole point of KNX, since a group is what a switch and a lamp have in
 * common - and the top bit of the length octet says which. The check octet
 * is the complement of the exclusive-or of everything in front of it.
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
// 9600 bit/s: 104.17 us a bit. TP1 has exactly one rate.
#define KNX_BIT_NS        104167
#define KNX_BIT_TOL_PCT       12

// The active pulse is about 35 us - a third of a bit - and what matters is
// that it is well under one, so that "a pulse here" and "a bit time of
// silence" are never in doubt
#define KNX_PULSE_MIN_NS   12000
#define KNX_PULSE_MAX_NS   70000

// Bit slots in a character, and how far into one a pulse may sit
#define KNX_CHAR_BITS         11   // start, 8 data, parity, stop
#define KNX_SLOT_TOL_X100     40   // of a bit period

// Two bit times of gap end a character; a new one may start any time after
#define KNX_GAP_BITS           2

#define KNX_MIN_OCTETS         4

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_knx   (g_logic_analysis.knx)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const KnxAnalysis *knx_analysis(void)
{
  return &g_knx;
}

//-----------------------------------------------------------------------------
// KNX writes an individual address as area.line.device - 4, 4 and 8 bits -
// and a group address as main/middle/sub, 5, 3 and 8. Printing the sixteen
// bits as a number instead would be printing the wire rather than the bus.
void knx_addr_text(uint16_t a, bool group, char *buf, int size)
{
  if (group)
    snprintf(buf, size, "%d/%d/%d", (a >> 11) & 0x1F, (a >> 8) & 7, a & 0xFF);
  else
    snprintf(buf, size, "%d.%d.%d", (a >> 12) & 0x0F, (a >> 8) & 0x0F,
        a & 0xFF);
}

//-----------------------------------------------------------------------------
void knx_byte_label(const KnxAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  char addr[16];

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  switch (idx)
  {
    case 0:
      // Priority lives in bits 3..2 and the repeat flag is ACTIVE LOW: a
      // clear bit 5 means the sender has said this before
      snprintf(buf, size, "ctl p%d%s", (v >> 2) & 3,
          (0 == (v & 0x20)) ? " rpt" : "");
      break;

    case 1:
      break;                          // written on the byte that completes it

    case 2:
      knx_addr_text(a->src, false, addr, sizeof(addr));
      snprintf(buf, size, "%s", addr);
      break;

    case 3:
      break;

    case 4:
      knx_addr_text(a->dst, a->group, addr, sizeof(addr));
      snprintf(buf, size, "%s", addr);
      break;

    case 5:
      snprintf(buf, size, "%s len%d", a->group ? "grp" : "ind", a->len);
      break;

    default:
      if (idx == a->octets - 1)
        snprintf(buf, size, a->fcs_ok ? "FCS" : "FCS!");
      else
        snprintf(buf, size, "d%d", idx - 6);
      break;
  }
}

//-----------------------------------------------------------------------------
void knx_field_label(const KnxAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  static const char *const name[6] =
      { "CTRL", "SRC-H", "SRC-L", "DST-H", "DST-L", "NPCI" };

  if (idx < 6)
  {
    snprintf(buf, size, "%s", name[idx]);

    return;
  }

  if (idx == a->octets - 1)
    snprintf(buf, size, a->fcs_ok ? "FCS" : "FCS!");
  else
    snprintf(buf, size, "d%d", idx - 6);
}

//-----------------------------------------------------------------------------
// The two address octets are one address and are shown as one. Everything
// else stands alone: a control octet is a control octet.
void knx_group_at(const KnxAnalysis *a, int idx, int *start, int *len)
{
  (void)a;

  *start = idx;
  *len = 1;

  if (idx < 1 || idx > 4)
    return;

  *start = (idx <= 2) ? 1 : 3;
  *len = 2;
}

//-----------------------------------------------------------------------------
// Is there an active pulse whose leading edge sits within tolerance of
// `want`? Returns the run index, or -1. `from` is where to start looking.
static int knx_pulse_at(const LogicScratch *s, int runs, int idle, int *from,
    int64_t want_x256, int64_t tol_x256)
{
  for (int r = *from; r < runs; r++)
  {
    if (s->lvl[r] == idle)
      continue;

    int64_t p = (int64_t)s->pos[r] * 256;

    if (p < want_x256 - tol_x256)
    {
      *from = r + 1;

      continue;
    }

    if (p > want_x256 + tol_x256)
      return -1;                      // the next pulse is past this slot

    *from = r;

    return r;
  }

  return -1;
}

//-----------------------------------------------------------------------------
int knx_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  memset(&g_knx, 0, sizeof(g_knx));
  out->proto = PROTO_KNX;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 8 || period_ns <= 0)
    return 0;

  int64_t bit_x256 = (int64_t)KNX_BIT_NS * 256 / period_ns;

  if (bit_x256 < 3 * 256)
    return 0;                         // undersampled: a slot is not placeable

  int idle = (0 != scratch->lvl[runs - 1]) ? 1 : 0;
  int64_t tol = bit_x256 * KNX_SLOT_TOL_X100 / 100;

  out->idle_high = (0 != idle);

  // Walk the pulses. A character starts at one that is not inside the eleven
  // slots of the character before it.
  int next_char = 0;                  // sample position, x256, or 0 for none
  int cursor = 1;

  for (int r = 1; r + 1 < runs && out->count < LOGIC_MAX_BYTES; r++)
  {
    if (scratch->lvl[r] == idle)
      continue;

    int64_t plen = (int64_t)scratch->len[r] * period_ns;

    if (plen < KNX_PULSE_MIN_NS || plen > KNX_PULSE_MAX_NS)
    {
      out->count = 0;                 // not a pulse: whatever this is, stop

      break;
    }

    int64_t start = (int64_t)scratch->pos[r] * 256;

    if (next_char && start < next_char - tol)
      continue;                       // inside a character already read

    // Eleven slots from here. Slot 0 is this pulse - the start bit - and the
    // stop bit at slot 10 must be silent.
    int value = 0, ones = 0;
    bool ok = true;

    cursor = r;

    for (int k = 1; k <= 10 && ok; k++)
    {
      int from = cursor;
      int j = knx_pulse_at(scratch, runs, idle, &from,
          start + bit_x256 * k, tol);
      int bit = (j < 0) ? 1 : 0;      // no pulse is a one

      if (j >= 0)
      {
        int64_t l = (int64_t)scratch->len[j] * period_ns;

        if (l < KNX_PULSE_MIN_NS || l > KNX_PULSE_MAX_NS)
          ok = false;

        cursor = j;
      }

      if (k <= 8)
      {
        value |= bit << (k - 1);      // data, least significant first
        ones += bit;
      }
      else if (9 == k)
      {
        ones += bit;                  // the parity bit joins the count
      }
      else if (0 == bit)
      {
        ok = false;                   // the stop bit has to be silent
      }
    }

    if (!ok)
      continue;

    if (0 != (ones & 1))
    {
      g_knx.parity_err++;             // even parity over data and parity bit

      continue;
    }

    out->bytes[out->count] = (uint8_t)value;
    out->pos[out->count] = scratch->pos[r];
    out->end[out->count] = (int)((start + bit_x256 * KNX_CHAR_BITS) / 256);
    out->count++;

    next_char = start + bit_x256 * (KNX_CHAR_BITS + KNX_GAP_BITS);
    r = cursor;
  }

  if (out->count < KNX_MIN_OCTETS)
    return 0;

  g_knx.rate = 9600;
  g_knx.octets = out->count;

  // The check octet is the complement of the exclusive-or of everything in
  // front of it, and it is what says these octets are one telegram rather
  // than a run of characters that happened to have good parity
  uint8_t x = 0;

  for (int i = 0; i + 1 < out->count; i++)
    x ^= out->bytes[i];

  g_knx.fcs_ok = ((uint8_t)~x == out->bytes[out->count - 1]);

  g_knx.ctrl = out->bytes[0];
  g_knx.src = (uint16_t)((out->bytes[1] << 8) | out->bytes[2]);
  g_knx.dst = (uint16_t)((out->bytes[3] << 8) | out->bytes[4]);
  g_knx.group = (0 != (out->bytes[5] & 0x80));
  g_knx.len = (uint8_t)(out->bytes[5] & 0x0F);
  g_knx.repeat = (0 == (out->bytes[0] & 0x20));

  out->rate = 9600;
  out->burst_start = true;
  out->errors = g_knx.parity_err + (g_knx.fcs_ok ? 0 : 1);

  // Pulses a third of a bit wide on a 104 us grid, a parity bit per character
  // and a check octet over the lot. Nothing else here answers to that.
  out->ambiguous = !g_knx.fcs_ok;

  char s[16], d[16];

  knx_addr_text(g_knx.src, false, s, sizeof(s));
  knx_addr_text(g_knx.dst, g_knx.group, d, sizeof(d));

  snprintf(out->info, sizeof(out->info), "KNX %s>%s %dB%s", s, d, g_knx.len,
      g_knx.fcs_ok ? "" : " FCS!");

  return out->count;
}
