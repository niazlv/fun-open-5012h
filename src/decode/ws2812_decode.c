/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
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

#define g_ws  (g_logic_analysis.ws)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const WsAnalysis *ws2812_analysis(void)
{
  return &g_ws;
}

//-----------------------------------------------------------------------------
// The three bytes of one pixel. A frame that ended mid-pixel gives up
// whatever it had, so the group is what arrived and never what a whole pixel
// would have been.
void ws2812_group_at(const WsAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= a->count)
    return;

  *start = idx - (idx - a->base[idx]) % 3;

  int n = 1;

  while (n < 3 && *start + n < a->count &&
      a->base[*start + n] == a->base[idx])
    n++;

  *len = n;
}

//-----------------------------------------------------------------------------
// Which channel of which pixel, numbered within its own update: a second
// frame in the record addresses pixel 0 again, and calling its first byte G7
// because seven pixels went past earlier names a pixel that is not lit.
void ws2812_field_label(const WsAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  static const char chan[3] = { 'G', 'R', 'B' }; // the wire order

  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= a->count)
    return;

  int off = idx - a->base[idx];

  snprintf(buf, size, "%c%d", chan[off % 3], off / 3);
}

//-----------------------------------------------------------------------------
// What the pixel above is, as a colour. This is the whole reason the bytes
// are grouped: three numbers in G-R-B order are not a colour anybody reads,
// and the one written across them is.
void ws2812_byte_label(const WsAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= a->count)
    return;

  uint8_t p = a->pix[idx];

  if (0xFF == p)
  {
    // The frame ended inside this pixel: the bytes are real, the colour they
    // would have made is not
    snprintf(buf, size, "cut");

    return;
  }

  snprintf(buf, size, "#%02X%02X%02X", a->rgb[p][0], a->rgb[p][1],
      a->rgb[p][2]);
}

//-----------------------------------------------------------------------------
int ws2812_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  (void)data;

  memset(out, 0, sizeof(*out));
  memset(&g_ws, 0, sizeof(g_ws));
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

    // The line rests LOW, and that is the cheapest thing that tells a strip
    // from the protocols it shares a pulse-width histogram with. Every high
    // here is one bit's mark and cannot outlast the cell that holds it, so a
    // high longer than the widest cell this decoder accepts is not a mark at
    // all - it is the line idle, which is a UART's rest level and never a
    // strip's. Before any histogram, before the period, and it costs one
    // comparison per run.
    if ((int64_t)w * period_ns >= WS_PERIOD_MAX_NS)
      return 0;

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

  // The same rule again, now that the cell is known and the bound can be the
  // real one rather than the widest this decoder allows. A mark is the FIRST
  // part of a cell and the low that carries the value is the rest of it, so a
  // high as long as a whole cell is a line at rest that happened to be short.
  // This is the form that reads a window which never caught the idle at all -
  // a slice out of the middle of a fast UART packet, where a run of set bits
  // is a mark several bit times long and still well under 4 us.
  if ((int64_t)hmax * period_ns >= bitp)
    return 0;

  int thresh = (hmin + hmax) / 2; // samples

  int value = 0, nbits = 0, byte_start = -1;
  // Where the update in progress began in out->bytes. The strip resets its
  // own pixel pointer on a latching gap, so this is what pixel numbering and
  // pixel grouping are both measured from.
  int fbase = 0;

  for (int r = 0; r < runs && out->count < LOGIC_MAX_BYTES; r++)
  {
    if (scratch->lvl[r] == 0)
    {
      // A long low latches the frame: drop a partial byte
      if ((int64_t)scratch->len[r] * period_ns >= WS_RESET_MIN_NS)
      {
        if (nbits != 0)
          out->errors++;

        if (out->count > fbase)
          fbase = out->count;

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
      g_ws.base[out->count] = (uint8_t)fbase;
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

  g_ws.bit_ns = (int)bitp;
  g_ws.count = out->count;

  // Assemble the pixels, one update at a time. Three bytes make one, and the
  // wire sends them G-R-B - so this is the only place the record is put back
  // into the order the colour is written in.
  for (int i = 0; i < out->count; )
  {
    bool whole = (0 == (i - g_ws.base[i]) % 3) && i + 2 < out->count &&
        g_ws.base[i + 1] == g_ws.base[i] && g_ws.base[i + 2] == g_ws.base[i];

    if (!whole)
    {
      g_ws.pix[i] = 0xFF;
      g_ws.partial = true;
      i++;

      continue;
    }

    int p = g_ws.pixels++;

    g_ws.rgb[p][0] = out->bytes[i + 1];
    g_ws.rgb[p][1] = out->bytes[i];
    g_ws.rgb[p][2] = out->bytes[i + 2];
    g_ws.pix[i] = g_ws.pix[i + 1] = g_ws.pix[i + 2] = (uint8_t)p;
    i += 3;
  }

  for (int i = 0; i < out->count; i++)
  {
    if (0 == i || g_ws.base[i] != g_ws.base[i - 1])
      g_ws.frames++;
  }

  out->rate = (int)bitp;

  if (g_ws.frames > 1)
    snprintf(out->info, sizeof(out->info), "WS2812 %dLED x%d err %d",
        g_ws.pixels, g_ws.frames, out->errors);
  else
    snprintf(out->info, sizeof(out->info), "WS2812 %dLED err %d",
        g_ws.pixels, out->errors);

  return out->count;
}
