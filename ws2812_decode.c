/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * WS2812/WS2812B (NeoPixel) decoder.
 *
 * A bit is a high pulse followed by a low: ~0.4 us high = '0', ~0.8 us
 * high = '1', bit period ~1.25 us; a low longer than ~50 us latches the
 * frame. Bits are MSB-first, 24 per LED in G-R-B order; we emit them as
 * plain bytes (3 per LED).
 *
 * The classifier is adaptive: the threshold between short and long highs is
 * the midpoint of the observed extremes, so 400 kHz strips and clones with
 * off-spec timings decode too.
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
#define WS_PERIOD_MIN_NS      700   // sanity window for the bit period
#define WS_PERIOD_MAX_NS     4000
#define WS_RESET_MIN_NS     30000   // low gap that latches a frame

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int ws2812_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  (void)data;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_WS2812;
  out->idle_high = false;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 8)
    return 0;

  // Collect high-pulse widths; the stream is (high, low) pairs back to back
  int hmin = 0, hmax = 0, highs = 0;

  for (int r = 0; r < runs; r++)
  {
    if (scratch->lvl[r] != 1)
      continue;

    int w = scratch->len[r];

    if (0 == hmin || w < hmin)
      hmin = w;

    if (w > hmax)
      hmax = w;

    highs++;
  }

  if (highs < 8)
    return 0;

  // Two distinct pulse widths must exist (all-0x00 or all-0xFF frames are
  // ambiguous with a plain clock; reject rather than guess)
  if (hmax < hmin * 3 / 2)
    return 0;

  int64_t bitp = 0;
  int bitp_n = 0;

  // Bit period from rise-to-rise distance of adjacent high runs
  for (int r = 0; r + 2 < runs; r++)
  {
    if (scratch->lvl[r] != 1 || scratch->lvl[r + 2] != 1)
      continue;

    int64_t d = (int64_t)(scratch->pos[r + 2] - scratch->pos[r]) * period_ns;

    if (d >= WS_PERIOD_MIN_NS && d <= WS_PERIOD_MAX_NS)
    {
      bitp += d;
      bitp_n++;
    }
  }

  if (bitp_n < 8)
    return 0;

  bitp /= bitp_n;

  int thresh = (hmin + hmax) / 2; // samples

  int value = 0, nbits = 0, byte_start = -1;

  for (int r = 0; r < runs && out->count < LOGIC_MAX_BYTES; r++)
  {
    if (scratch->lvl[r] == 0)
    {
      // A long low latches the frame: drop a partial byte
      if ((int64_t)scratch->len[r] * period_ns >= WS_RESET_MIN_NS)
      {
        if (nbits != 0)
          out->errors++;

        value = 0;
        nbits = 0;
      }

      continue;
    }

    int bit = (scratch->len[r] > thresh) ? 1 : 0;

    if (nbits == 0)
      byte_start = scratch->pos[r];

    value = (value << 1) | bit; // MSB first

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

  if (out->count < 3) // less than one LED worth of data
    return 0;

  out->rate = (int)bitp;

  snprintf(out->info, sizeof(out->info), "WS2812 %dLED err %d",
      out->count / 3, out->errors);

  return out->count;
}
