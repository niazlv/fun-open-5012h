/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * NEC infrared remote protocol decoder (as seen after a TSOP-style
 * demodulator: idle high, marks are low).
 *
 * Frame: 9 ms mark + 4.5 ms space, then 32 bits, each a 560 us mark
 * followed by a 560 us space ('0') or 1690 us space ('1'), LSB-first:
 * address, ~address, command, ~command. A 9 ms mark + 2.25 ms space is a
 * repeat code.
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
#define NEC_TOL(x)         ((x) / 4)  // +-25% timing tolerance
#define NEC_LEAD_MARK_NS   9000000
#define NEC_LEAD_SPACE_NS  4500000
#define NEC_RPT_SPACE_NS   2250000
#define NEC_MARK_NS         560000
#define NEC_SPACE0_NS       560000
#define NEC_SPACE1_NS      1690000

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static bool near_ns(int64_t value, int64_t target)
{
  int64_t d = value - target;

  if (d < 0)
    d = -d;

  return d <= NEC_TOL(target);
}

//-----------------------------------------------------------------------------
int nec_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;
  bool repeat_seen = false;

  (void)data;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_NEC;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 4)
    return 0;

  for (int r = 0; r + 1 < runs && out->count + 4 <= LOGIC_MAX_BYTES; r++)
  {
    // Lead: long mark (low) followed by the lead space (high)
    if (scratch->lvl[r] != 0 ||
        !near_ns((int64_t)scratch->len[r] * period_ns, NEC_LEAD_MARK_NS))
      continue;

    int64_t space_ns = (int64_t)scratch->len[r + 1] * period_ns;

    if (near_ns(space_ns, NEC_RPT_SPACE_NS))
    {
      repeat_seen = true;
      r++;
      continue;
    }

    if (!near_ns(space_ns, NEC_LEAD_SPACE_NS))
      continue;

    // 32 data bits = 64 runs after the lead pair
    uint32_t word = 0;
    int frame_start = scratch->pos[r];
    int bits = 0;
    int k = r + 2;

    while (bits < 32 && k + 1 < runs)
    {
      if (scratch->lvl[k] != 0 ||
          !near_ns((int64_t)scratch->len[k] * period_ns, NEC_MARK_NS))
        break;

      int64_t sp = (int64_t)scratch->len[k + 1] * period_ns;

      if (near_ns(sp, NEC_SPACE1_NS))
        word |= 1u << bits; // LSB first
      else if (!near_ns(sp, NEC_SPACE0_NS))
        break;

      bits++;
      k += 2;
    }

    // Why the bit loop stopped decides what this frame is. Out of record is
    // not out of protocol: a NEC frame is 67.5 ms end to end, which is longer
    // than the record holds at most timebases, and refusing everything that
    // did not fit whole means pointing the scope at a remote control and
    // being told there is no remote control. So a lead that was followed by
    // whole bytes reports those bytes and says the rest was cut off. A
    // TIMING failure mid-frame is different - that is a signal which started
    // like NEC and then was not NEC, and it stays an error.
    bool cut = (bits < 32 && k + 1 >= runs);
    int whole = bits / 8;

    if (bits == 32 || (cut && whole >= 1))
    {
      uint8_t byte[4] =
      {
        word & 0xff, (word >> 8) & 0xff, (word >> 16) & 0xff, (word >> 24) & 0xff
      };
      int n = (bits == 32) ? 4 : whole;

      // Address and command are each sent with their own complement, so a
      // whole frame carries its own integrity check
      if (bits == 32 && ((byte[2] ^ byte[3]) & 0xff) != 0xff)
        out->errors++;

      // k lands one past the last run the frame used, which on a frame the
      // record cut short is one past the LIST: the end of the record is then
      // the end of the frame, and reading scratch->pos[runs] for it was
      // reading whatever the spare SRAM happened to hold
      int frame_end = (k < runs) ? scratch->pos[k] + scratch->len[k] : size;
      int span = frame_end - frame_start;

      for (int i = 0; i < n && out->count < LOGIC_MAX_BYTES; i++)
      {
        out->bytes[out->count] = byte[i];
        out->pos[out->count] = frame_start + span * i / 4;
        out->end[out->count] = frame_start + span * (i + 1) / 4;
        out->count++;
      }

      if (bits == 32)
        snprintf(out->info, sizeof(out->info), "NEC A=%02X C=%02X%s",
            byte[0], byte[2], repeat_seen ? " rpt" : "");
      else
        snprintf(out->info, sizeof(out->info), "NEC A=%02X %d/32 bits",
            byte[0], bits);

      out->overrun = out->overrun || cut;

      r = k;
    }
    else
    {
      out->errors++;
    }
  }

  if (out->count == 0)
    return 0;

  out->rate = NEC_MARK_NS;

  if (!out->info[0])
    snprintf(out->info, sizeof(out->info), "NEC err %d", out->errors);

  return out->count;
}
