/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
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

/*- Variables ---------------------------------------------------------------*/
LogicAnalysis g_logic_analysis;

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
    case PROTO_SERVO:   return "Servo";
    case PROTO_CAN:     return "CAN";
    case PROTO_DHT:     return "DHT";
    case PROTO_SENT:    return "SENT";
    case PROTO_MIDI:    return "MIDI";
    case PROTO_LIN:     return "LIN";
    case PROTO_EV1527:  return "EV1527";
    case PROTO_DSHOT:   return "DShot";
    // The mark is part of the name: this one is a reconstruction of a bus
    // whose clock the instrument never saw
    case PROTO_SPI:     return "SPI!";
    case PROTO_MANCH:   return "MAN";
    case PROTO_RC5:     return "RC5";
    case PROTO_DALI:    return "DALI";
    case PROTO_KNX:     return "KNX";
    case PROTO_RAW:     return "RAW";
    default:            return "----";
  }
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

  // Already split this exact record for an earlier decoder in the SAME
  // dispatcher call. Outside one, the record may have been refilled under
  // the same pointer and there is nothing cheap that can tell.
  if (scratch->cascade == LOGIC_CACHE_MAGIC &&
      scratch->cache_valid == LOGIC_CACHE_MAGIC &&
      scratch->cache_data == data && scratch->cache_size == size &&
      scratch->cache_offset == offset)
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
    // First, and safely so: a CAN frame is confirmed by its own CRC-15
    // before it is reported at all, so this one never takes a record that
    // belongs to anything else
    { PROTO_CAN,     can_decode },
    // Also CRC-confirmed before it is reported, and it has to run ahead of
    // WS2812: both hold a constant bit period and put the value in the duty,
    // at nearly the same rate. Nothing else on the list works in the range
    // of bit times DShot uses, so first costs nobody anything.
    { PROTO_DSHOT,   dshot_decode },
    { PROTO_NEC,     nec_decode },
    // The other infrared family, and it has to come before the generic
    // Manchester reader for the same reason NEC comes before everything:
    // read as a MESSAGE it says address, command and toggle, and read as a
    // line code it says fourteen bits. Both are true; only one is useful.
    { PROTO_RC5,     rc5_decode },
    // After NEC and before everything else: 24 bits of constant period is a
    // statement nothing else here makes, and its 31T gap is a long dominant
    // field that a break, a 1-Wire reset or a DHT start pulse would each
    // answer to. NEC keeps its place in front because a 9 ms leader is a
    // more specific claim than a ratio that holds at any T.
    { PROTO_EV1527,  ev1527_decode },
    // Its break is at least ten dominant bits and 8N1 cannot send more than
    // nine, so a LIN frame is as self-identifying as a CAN frame is. After
    // NEC only because a 9 ms infrared leader followed by four zero bits has
    // the same shape, and NEC's own leader is the more specific of the two.
    { PROTO_LIN,     lin_decode },
    { PROTO_ONEWIRE, onewire_decode },
    { PROTO_WS2812,  ws2812_decode },
    // After 1-Wire, whose zero slots are the same shape as a DHT bit and
    // whose reset pulse a DHT start pulse would answer to; before the two
    // generic ones, because forty slots and a checksum is a signature
    { PROTO_DHT,     dht_decode },
    // Its sync pulse calibrates the frame that follows it, so a SENT record
    // says what its own tick is - but the shape is only a strong hint, and
    // sent_decode marks a record with no CRC behind it ambiguous, which is
    // what keeps this from taking anyone else's
    { PROTO_SENT,    sent_decode },
    // MIDI is a UART link, so it has to come before the UART decoder or it
    // would never be reached; and before the servo, because a stream of
    // clock bytes at a steady tempo IS a fixed-rate pulse train. What earns
    // it the place is that it answers to one rate only and reads the bytes
    // as a grammar, so it takes nothing that is not MIDI.
    { PROTO_MIDI,    midi_decode },
    // Before UART, because a servo pulse train IS decodable as UART - a
    // 1.5 ms pulse frames bytes at 667 baud perfectly well - and after the
    // three above, whose leaders and reset pulses are more specific still
    // Confirmed by a parity bit per character and a check octet over the
    // telegram before it is reported, so it is safe this early - and it must
    // be, because a train of 35 us pulses is something the generic readers
    // will happily make bits out of.
    { PROTO_KNX,     knx_decode },
    // Before the generic bi-phase reader for the same reason RC5 is: read as
    // a line code a DALI frame is nineteen bits, and read as DALI it is
    // "ballast 5, off". Both are true and only one is worth showing.
    { PROTO_DALI,    dali_decode },
    // A line code rather than a protocol, and the check it makes is
    // structural: every run half a bit or a whole one, nothing else. That
    // rules out NEC's 1:3, a strip's four lengths and a UART's 1,2,3,4 - so
    // it can sit ahead of the generic decoders. Guessing its own rate leaves
    // a factor of two open, and it says so by marking the record ambiguous.
    { PROTO_MANCH,   manchester_decode },
    { PROTO_SERVO,   servo_decode },
    { PROTO_UART,    uart_decode },
    // Dead last but for the raw reader, and it never wins anyway: a data
    // line without its clock reports itself ambiguous every time, so auto
    // mode always passes over it. It is here so that asking for it by name
    // reaches it, and for no other reason.
    { PROTO_SPI,     spi_decode },
    { PROTO_RAW,     raw_decode },
  };

  // Static, not automatic: half a kilobyte of decoded record does not belong
  // on the stack of a function the main loop calls every frame
  static LogicResult cur;

  memset(out, 0, sizeof(*out));

  // The record behind an unchanged (data, size, offset) is refilled by the
  // DMA between calls, so the split is only reusable within this one call -
  // which is exactly what this marks, and what logic_runs asks for
  scratch->cache_valid = 0;
  scratch->cascade = LOGIC_CACHE_MAGIC;

  for (unsigned i = 0; i < sizeof(decoders) / sizeof(decoders[0]); i++)
  {
    if (forced != PROTO_AUTO && forced != decoders[i].proto)
      continue;

    if (decoders[i].fn(data, size, offset, period_ns, scratch, &cur) > 0)
    {
      // A decoder can report frames it cannot tell apart from something
      // simpler: a UART record that never catches the line at rest and holds
      // one repeated byte value IS a square wave, sample for sample. Guessing
      // the protocol from that is not on, so in auto mode it falls through to
      // the next decoder. Asked for UART by name - by the user, or by the
      // caller's own earlier match - the question is already answered and the
      // frames stand.
      if (cur.ambiguous && forced == PROTO_AUTO)
        continue;

      *out = cur;
      scratch->cascade = 0;

      return cur.count;
    }
  }

  scratch->cascade = 0;

  return 0;
}
