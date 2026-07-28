/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DHT11 / DHT22 (AM2302) humidity and temperature sensors.
 *
 * One wire, idle high on a pull-up. The host pulls it down to ask - 18 ms for
 * a DHT11, about 1 ms for a DHT22 - and lets go; the sensor answers with an
 * 80 us low and an 80 us high, then sends forty bits. Every bit is a 50 us
 * low followed by a high whose LENGTH is the value: about 26 us is a zero,
 * about 70 us a one. Five bytes come out, and the fifth is the sum of the
 * other four.
 *
 * That sum is the gate. Forty slots of the right shape are already unusual,
 * and a checksum on top of them is what lets this decoder answer "yes" - a
 * record that has not got one is refused rather than reported with a caveat.
 *
 * Which of the two parts sent the bytes is a question about the numbers and
 * not about the wire, because the wire is identical. A DHT11 sends whole
 * percent and whole degrees with the fractional bytes left at zero; a DHT22
 * sends tenths across both bytes of each pair. Read one as the other and the
 * humidity comes out at 256 % - which is exactly what decides it.
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
// The datasheet says 50 us low, 26-28 us high for a zero and 70 for a one.
// The windows are wide because the parts - and the clones especially - are
// not, and because the two populations are far enough apart to afford it.
#define DHT_LOW_MIN_US     30
#define DHT_LOW_MAX_US     95
#define DHT_ZERO_MIN_US     8
#define DHT_ZERO_MAX_US    45
#define DHT_ONE_MIN_US     46
#define DHT_ONE_MAX_US    125

// The 80 us / 80 us the sensor answers the host with
#define DHT_RESP_MIN_US    50
#define DHT_RESP_MAX_US   130

// A frame is asked for at most once a second, so anything before one is a
// long silence - unless the record simply opened in the middle of it
#define DHT_IDLE_MIN_US   200

// The host's own start pulse, which is the one place the two parts differ on
// the wire rather than in the numbers
#define DHT11_START_MIN_US  15000

#define DHT_BITS  40

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_dht   (g_logic_analysis.dht)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline int64_t dht_us(int len, int period_ns)
{
  return (int64_t)len * period_ns / 1000;
}

//-----------------------------------------------------------------------------
// Is the low run at `r` the first bit of a frame, rather than the middle of
// something else that happens to have pulses of about the right length? What
// comes before a frame is the sensor's response pulse, or a silent line, or
// the start of the record.
static bool dht_anchored(const LogicScratch *s, int runs, int period_ns, int r)
{
  (void)runs;

  if (r <= 1)
    return true;         // the record opens here; nothing to ask

  int64_t high = dht_us(s->len[r - 1], period_ns);

  if (high >= DHT_IDLE_MIN_US)
    return true;         // a line that had been left alone

  if (high >= DHT_RESP_MIN_US && high <= DHT_RESP_MAX_US && r >= 2)
  {
    int64_t low = dht_us(s->len[r - 2], period_ns);

    // The 80/80 the sensor answers with. It is also a valid-looking bit slot
    // in its own right, which is why the frame search slides rather than
    // committing to the first slot it finds.
    if (low >= DHT_RESP_MIN_US && low <= DHT_RESP_MAX_US)
      return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
// Work out what the five bytes say, and which part said it
static bool dht_read(const uint8_t *b, bool dht11_start, DhtAnalysis *a)
{
  int rh22 = ((int)b[0] << 8) | b[1];
  int t22 = (int)((b[2] & 0x7F) << 8) | b[3];

  if (b[2] & 0x80)
    t22 = -t22;          // sign and magnitude, not two's complement

  // Newer DHT11 firmware does use the fractional bytes, and the sign bit
  // with them, so it is read the same way - just in whole units
  int rh11 = (int)b[0] * 10 + b[1];
  int t11 = (int)b[2] * 10 + (b[3] & 0x7F);

  if (b[3] & 0x80)
    t11 = -t11;

  bool ok22 = (rh22 <= 1000) && (t22 >= -400) && (t22 <= 800);
  bool ok11 = (b[1] <= 9) && ((b[3] & 0x7F) <= 9) &&
      (b[0] <= 100) && (b[2] <= 60);

  if (ok22 && !ok11)
  {
    a->dht22 = true;
    a->sure = true;
  }
  else if (ok11 && !ok22)
  {
    a->dht22 = false;
    a->sure = true;
  }
  else if (ok11 && ok22)
  {
    // Both readings are possible, which needs the fractional bytes to be
    // zero and the humidity byte to be tiny. The host's start pulse settles
    // it if the record caught one, and otherwise this is a guess and says so.
    a->dht22 = dht11_start ? false : (b[1] != 0 || b[3] != 0);
    a->sure = dht11_start;
  }
  else
  {
    return false;        // neither reading is a reading; not a DHT frame
  }

  a->rh_x10 = a->dht22 ? rh22 : rh11;
  a->t_x10 = a->dht22 ? t22 : t11;

  return true;
}

//-----------------------------------------------------------------------------
const DhtAnalysis *dht_analysis(void)
{
  return &g_dht;
}

//-----------------------------------------------------------------------------
void dht_byte_label(const DhtAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0)
    return;

  // The reading lands on the byte that COMPLETES it, not the one that opens
  // the pair. A DHT22's humidity is 0x022B - the 0x02 by itself is not 55.5%
  // and a label saying so under it claims more than that byte holds. The
  // first byte of each pair is named for the field, the second carries the
  // number, which is also the order they are read in.
  switch (idx % 5)
  {
    case 0:
      snprintf(buf, size, "RH");
      break;

    case 1:
      // The pair is one reading and is shown as one, so the label that closes
      // it carries the name as well: on its own "45.3%" under a byte holding
      // 0xC5 would be a number with nothing to attach it to
      if (a->dht22)
        snprintf(buf, size, "RH=%d.%d%%", a->rh_x10 / 10, a->rh_x10 % 10);
      else
        snprintf(buf, size, "RH=%d%%", a->rh_x10 / 10);
      break;

    case 2:
      snprintf(buf, size, "T");
      break;

    case 3:
    {
      int t = a->t_x10;
      const char *sign = (t < 0) ? "-" : "+";

      if (t < 0)
        t = -t;

      if (a->dht22)
        snprintf(buf, size, "T=%s%d.%dC", sign, t / 10, t % 10);
      else
        snprintf(buf, size, "T=%s%dC", sign, t / 10);
      break;
    }

    default:
      snprintf(buf, size, "SUM");
      break;
  }
}

//-----------------------------------------------------------------------------
// Which half of which reading this byte is. The pair says what the reading
// IS; this says which end of it you are looking at.
void dht_field_label(const DhtAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  static const char *const name[5] = { "RH-H", "RH-L", "T-H", "T-L", "SUM" };

  (void)a;
  (void)v;

  buf[0] = 0;

  if (idx < 0)
    return;

  snprintf(buf, size, "%s", name[idx % 5]);
}

//-----------------------------------------------------------------------------
// Humidity is two bytes and so is temperature: one reading each, and one
// thing each on the screen. The checksum stands alone.
void dht_group_at(const DhtAnalysis *a, int idx, int *start, int *len)
{
  (void)a;

  *start = idx;
  *len = 1;

  if (idx < 0)
    return;

  int rel = idx % 5;

  if (rel < 4)
  {
    *start = idx - (rel & 1);
    *len = 2;
  }
}

//-----------------------------------------------------------------------------
int dht_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;
  uint8_t bit[DHT_BITS];
  int bpos[DHT_BITS], bend[DHT_BITS], brun[DHT_BITS];
  int nb = 0;
  bool dht11_start = false;

  (void)size;
  (void)offset;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_DHT;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 8 || period_ns <= 0)
    return 0;

  // The bus rests high; a frame is a run of (low, high) pairs. The response
  // pulse passes for a bit slot too, so the window slides one slot at a time
  // and the checksum picks the alignment rather than the search doing it.
  for (int r = 0; r + 1 < runs; r++)
  {
    if (scratch->lvl[r] != 0)
      continue;

    int64_t low = dht_us(scratch->len[r], period_ns);

    if (low >= DHT11_START_MIN_US)
      dht11_start = true;   // 18 ms of it: nobody asks a DHT22 like that

    if (low < DHT_LOW_MIN_US || low > DHT_LOW_MAX_US)
    {
      nb = 0;               // whatever this is, the run of slots ends here
      continue;
    }

    int64_t high = dht_us(scratch->len[r + 1], period_ns);
    int v;

    if (high >= DHT_ZERO_MIN_US && high <= DHT_ZERO_MAX_US)
      v = 0;
    else if (high >= DHT_ONE_MIN_US && high <= DHT_ONE_MAX_US)
      v = 1;
    else
    {
      nb = 0;
      continue;
    }

    if (nb == DHT_BITS)
    {
      memmove(bit, bit + 1, DHT_BITS - 1);
      memmove(bpos, bpos + 1, (DHT_BITS - 1) * sizeof(bpos[0]));
      memmove(bend, bend + 1, (DHT_BITS - 1) * sizeof(bend[0]));
      memmove(brun, brun + 1, (DHT_BITS - 1) * sizeof(brun[0]));
      nb--;
    }

    bit[nb] = (uint8_t)v;
    bpos[nb] = scratch->pos[r];
    bend[nb] = scratch->pos[r + 1] + scratch->len[r + 1];
    brun[nb] = r;
    nb++;

    if (nb < DHT_BITS)
      continue;

    uint8_t by[5] = { 0, 0, 0, 0, 0 };

    for (int i = 0; i < DHT_BITS; i++)
      by[i / 8] = (uint8_t)((by[i / 8] << 1) | bit[i]);   // MSB first

    if ((uint8_t)(by[0] + by[1] + by[2] + by[3]) != by[4])
      continue;             // wrong alignment, or a frame that arrived broken

    if (!dht_anchored(scratch, runs, period_ns, brun[0]))
      continue;

    memset(&g_dht, 0, sizeof(g_dht));

    if (!dht_read(by, dht11_start, &g_dht))
      continue;             // the sum agreed on numbers that are not a reading

    g_dht.valid = true;

    for (int i = 0; i < 5; i++)
    {
      out->bytes[i] = by[i];
      out->pos[i] = bpos[i * 8];
      out->end[i] = bend[i * 8 + 7];
    }

    out->count = 5;
    out->rate = 50000;      // the 50 us low every bit slot opens with
    out->burst_start = true;

    int t = g_dht.t_x10;
    const char *sign = (t < 0) ? "-" : "+";

    if (t < 0)
      t = -t;

    if (g_dht.dht22)
      snprintf(out->info, sizeof(out->info), "DHT22%s %d.%d%% %s%d.%dC",
          g_dht.sure ? "" : "?", g_dht.rh_x10 / 10, g_dht.rh_x10 % 10,
          sign, t / 10, t % 10);
    else
      snprintf(out->info, sizeof(out->info), "DHT11%s %d%% %s%dC",
          g_dht.sure ? "" : "?", g_dht.rh_x10 / 10, sign, t / 10);

    return out->count;
  }

  return 0;
}
