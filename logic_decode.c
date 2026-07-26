/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Single-wire protocol decoding: shared run-length helper, the raw-bitstream
 * fallback and the auto-detecting dispatcher.
 *
 * Pure C, no hardware dependencies: compiled and tested on the host against
 * synthetic waveforms (see tests/host_test.c).
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "logic_decode.h"

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline int sample_at(const uint8_t *data, int size, int offset, int i)
{
  int index = offset + i;

  if (index >= size)
    index -= size;

  return data[index];
}

//-----------------------------------------------------------------------------
const char *logic_proto_name(proto_t proto)
{
  switch (proto)
  {
    case PROTO_UART:    return "UART";
    case PROTO_ONEWIRE: return "1-Wire";
    case PROTO_WS2812:  return "WS2812";
    case PROTO_NEC:     return "NEC IR";
    case PROTO_RAW:     return "RAW";
    default:            return "----";
  }
}

//-----------------------------------------------------------------------------
// Cheap content check for the run-split cache: 64 samples spread over the
// record. Two records that agree on all of them are either the same record
// or close enough that their run splits are interchangeable.
static uint32_t record_fingerprint(const uint8_t *data, int size, int offset)
{
  uint32_t h = (uint32_t)size * 2654435761u + (uint32_t)offset;
  int step = (size > 64) ? size / 64 : 1;

  for (int i = 0; i < size; i += step)
    h = h * 33u + data[i];

  return h;
}

//-----------------------------------------------------------------------------
// Threshold the record into level runs with a Schmitt comparator around the
// mid level. The unfinished tail run is recorded too (it carries the idle
// level of a record that ends between frames).
int logic_runs(const uint8_t *data, int size, int offset,
    LogicScratch *scratch, int *mid_out)
{
  int vmin = 255, vmax = 0;
  int runs = 0;
  uint32_t hash = record_fingerprint(data, size, offset);

  // Already split this exact record for an earlier decoder in the cascade
  if (scratch->cache_valid == LOGIC_CACHE_MAGIC &&
      scratch->cache_data == data && scratch->cache_size == size &&
      scratch->cache_offset == offset && scratch->cache_hash == hash)
  {
    *mid_out = scratch->cache_mid;
    return scratch->cache_runs;
  }

  scratch->cache_valid = 0;

  for (int i = 0; i < size; i++)
  {
    int v = sample_at(data, size, offset, i);

    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }

  int mid = (vmin + vmax) / 2;

  if ((vmax - vmin) < 20)
  {
    // No digital-looking swing. Cached as well, so an analog record costs
    // the cascade one min/max pass rather than five.
    scratch->cache_data   = data;
    scratch->cache_size   = size;
    scratch->cache_offset = offset;
    scratch->cache_runs   = 0;
    scratch->cache_mid    = mid;
    scratch->cache_hash   = hash;
    scratch->cache_valid  = LOGIC_CACHE_MAGIC;

    *mid_out = mid;

    return 0;
  }

  int hyst = (vmax - vmin) / 8;
  int level = (sample_at(data, size, offset, 0) > mid) ? 1 : 0;
  int run_start = 0;

  for (int i = 1; i < size && runs < LOGIC_MAX_RUNS; i++)
  {
    int v = sample_at(data, size, offset, i);
    int new_level = level;

    if (level == 0 && v > mid + hyst)
      new_level = 1;
    else if (level == 1 && v < mid - hyst)
      new_level = 0;

    if (new_level != level)
    {
      scratch->len[runs] = i - run_start;
      scratch->pos[runs] = run_start;
      scratch->lvl[runs] = (uint8_t)level;
      runs++;
      run_start = i;
      level = new_level;
    }
  }

  if (runs < LOGIC_MAX_RUNS && size - run_start > 0)
  {
    scratch->len[runs] = size - run_start;
    scratch->pos[runs] = run_start;
    scratch->lvl[runs] = (uint8_t)level;
    runs++;
  }

  *mid_out = mid;

  scratch->cache_data   = data;
  scratch->cache_size   = size;
  scratch->cache_offset = offset;
  scratch->cache_runs   = runs;
  scratch->cache_mid    = mid;
  scratch->cache_hash   = hash;
  scratch->cache_valid  = LOGIC_CACHE_MAGIC;

  return runs;
}

//-----------------------------------------------------------------------------
// Fallback for anything digital that no protocol matched (a lone SWD line,
// a clock, an unknown bus): estimate the unit pulse width, sample the level
// once per unit and pack the bits MSB-first into bytes
static int raw_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_RAW;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 4)
    return 0;

  // Unit width: shortest run, refined by averaging runs over their implied
  // integer bit counts (same idea as the UART auto-baud)
  int tmin = scratch->len[0];

  for (int r = 1; r < runs; r++)
  {
    if (scratch->len[r] < tmin)
      tmin = scratch->len[r];
  }

  if (tmin < 3)
    return 0;

  int64_t width_sum = 0;
  int bit_count = 0;

  for (int r = 0; r < runs; r++)
  {
    int bits = (scratch->len[r] + tmin / 2) / tmin;

    if (bits < 1 || bits > 64)
      continue;

    width_sum += scratch->len[r];
    bit_count += bits;
  }

  if (bit_count < 8)
    return 0;

  int bit_x256 = (int)(width_sum * 256 / bit_count);

  // Start at the first transition so the bit grid is phase-aligned
  int64_t start = (int64_t)scratch->pos[1] * 256;
  int total_bits = (int)(((int64_t)(size - scratch->pos[1]) * 256) / bit_x256);

  if (total_bits > LOGIC_MAX_BYTES * 8)
    total_bits = LOGIC_MAX_BYTES * 8;

  int value = 0, nbits = 0;
  int byte_start = scratch->pos[1];

  for (int b = 0; b < total_bits; b++)
  {
    int64_t center = start + (int64_t)bit_x256 * (2 * b + 1) / 2;
    int si = (int)(center / 256);

    if (si >= size)
      break;

    value = (value << 1) | ((sample_at(data, size, offset, si) > mid) ? 1 : 0);

    if (++nbits == 8)
    {
      int bend = (int)((start + (int64_t)bit_x256 * (b + 1)) / 256);

      out->bytes[out->count] = (uint8_t)value;
      out->pos[out->count] = byte_start;
      out->end[out->count] = bend;
      out->count++;

      value = 0;
      nbits = 0;
      byte_start = bend;
    }
  }

  if (out->count == 0)
    return 0;

  out->rate = (int)((int64_t)bit_x256 * period_ns / 256);

  int64_t bps = (out->rate > 0) ? 1000000000ll / out->rate : 0;

  if (bps >= 1000000)
    snprintf(out->info, sizeof(out->info), "RAW ~%d.%dMbit",
        (int)(bps / 1000000), (int)(bps % 1000000 / 100000));
  else
    snprintf(out->info, sizeof(out->info), "RAW ~%dkbit", (int)(bps / 1000));

  return out->count;
}

//-----------------------------------------------------------------------------
int logic_decode(const uint8_t *data, int size, int offset, int period_ns,
    proto_t forced, LogicScratch *scratch, LogicResult *out)
{
  typedef int (*decoder_fn)(const uint8_t *, int, int, int,
      LogicScratch *, LogicResult *);

  // Most specific first: a generic decoder (UART, and RAW even more so)
  // happily "decodes" garbage out of a more structured protocol, so in auto
  // mode the first decoder whose signature matches wins outright
  static const struct
  {
    proto_t proto;
    decoder_fn fn;
  } decoders[] =
  {
    { PROTO_NEC,     nec_decode },
    { PROTO_ONEWIRE, onewire_decode },
    { PROTO_WS2812,  ws2812_decode },
    { PROTO_UART,    uart_decode },
    { PROTO_RAW,     raw_decode },
  };

  LogicResult cur;

  memset(out, 0, sizeof(*out));

  // The record behind an unchanged (data, size, offset) is refilled by the
  // DMA between calls, so the split is only reusable within this one call
  scratch->cache_valid = 0;

  for (unsigned i = 0; i < sizeof(decoders) / sizeof(decoders[0]); i++)
  {
    if (forced != PROTO_AUTO && forced != decoders[i].proto)
      continue;

    if (decoders[i].fn(data, size, offset, period_ns, scratch, &cur) > 0)
    {
      *out = cur;
      return cur.count;
    }
  }

  return 0;
}
