/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * 1-Wire (Dallas/Maxim) decoder, standard speed.
 *
 * Bus idles high. A reset is a >=400 us low pulse; the slave answers with a
 * presence pulse. Every bit slot starts with a falling edge: a short low
 * (1..15 us) is a '1', a long low (60..120 us) is a '0'; bits are LSB-first.
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
#define OW_RESET_MIN_NS     400000  // reset low pulse
#define OW_ONE_MAX_NS        20000  // low <= this = '1'
#define OW_ZERO_MIN_NS       45000  // low >= this = '0'
#define OW_ZERO_MAX_NS      150000  // longer lows (but < reset) are errors

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int onewire_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;
  bool seen_reset = false;
  bool presence = false;

  (void)data;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_ONEWIRE;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 6)
    return 0;

  // The bus must idle high: the longest run has to be a high level
  int longest = 0;

  for (int r = 1; r < runs; r++)
  {
    if (scratch->len[r] > scratch->len[longest])
      longest = r;
  }

  if (0 == scratch->lvl[longest])
    return 0;

  int value = 0, nbits = 0, byte_start = -1;
  int short_lows = 0;

  for (int r = 0; r < runs && out->count < LOGIC_MAX_BYTES; r++)
  {
    if (scratch->lvl[r] != 0)
      continue; // only low pulses carry information

    int64_t low_ns = (int64_t)scratch->len[r] * period_ns;

    if (low_ns >= OW_RESET_MIN_NS)
    {
      // Reset: restart byte assembly; the next low is the presence pulse
      seen_reset = true;
      value = 0;
      nbits = 0;
      byte_start = -1;

      if (r + 2 < runs && scratch->lvl[r + 2] == 0)
      {
        int64_t pres_ns = (int64_t)scratch->len[r + 2] * period_ns;

        if (pres_ns >= 40000 && pres_ns <= 300000)
        {
          presence = true;
          r += 2; // consume the presence pulse
        }
      }

      continue;
    }

    int bit;

    if (low_ns <= OW_ONE_MAX_NS)
    {
      bit = 1;
      short_lows++;
    }
    else if (low_ns >= OW_ZERO_MIN_NS && low_ns <= OW_ZERO_MAX_NS)
      bit = 0;
    else
    {
      out->errors++;
      continue;
    }

    if (nbits == 0)
      byte_start = scratch->pos[r];

    value |= bit << nbits; // LSB first

    if (++nbits == 8)
    {
      out->bytes[out->count] = (uint8_t)value;
      out->pos[out->count] = byte_start;
      out->end[out->count] = scratch->pos[r] + scratch->len[r];
      out->count++;

      value = 0;
      nbits = 0;
    }
  }

  // Bit slots without a reset are indistinguishable from random pulses, and
  // slow UART traffic can fake both the reset (a run of '0' bits) and the
  // zero slots. What UART can never produce is 1-Wire's other pulse
  // population: the 1-15 us '1' slots — require at least two of them.
  if (!seen_reset || short_lows < 2 || out->count == 0)
    return 0;

  out->rate = 60000; // nominal 60 us slot

  snprintf(out->info, sizeof(out->info), "1-Wire%s err %d",
      presence ? " +presence" : "", out->errors);

  return out->count;
}
