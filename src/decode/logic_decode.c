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
    case PROTO_SWO:     return "ITM";
    case PROTO_SWD:     return "SWD";
    case PROTO_USB:     return "USB";
    case PROTO_PD:      return "USB-PD";
    case PROTO_SIRC:    return "SIRC";
    case PROTO_PPM:     return "PPM";
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
    // Third, and it costs the ones behind it nothing: USB runs at 1.5 or 12
    // Mbit and at no other rate, so on any record too coarse for both this
    // decoder is two comparisons and gone. Where it does run, a packet is
    // reported only once a CRC5 or a CRC16 has agreed with it - the SYNC
    // shape and the PID's complement narrow the field, and the CRC is what
    // decides - so like CAN and DShot it cannot take a record that is not its
    // own. It has to come ahead of the shape-matchers for the reason SWO
    // does: 12 Mbit on the wire is single-digit samples per run, which is
    // territory the strip and infrared readers will happily make bits out of.
    { PROTO_USB,     usb_decode },
    // Beside USB and for the same reason: a message is reported only once its
    // CRC32 has agreed with it, so it can take nobody else's record. Cheaper
    // to try than any of them, too - Power Delivery runs at 300 kbit and at
    // no other rate, so a record whose unit time is outside that band leaves
    // after one comparison. It is the only decoder here that walks the
    // samples itself rather than the shared run split; see pd_decode.c for
    // why the split cannot hold the one message worth reading.
    { PROTO_PD,      pd_decode },
    { PROTO_NEC,     nec_decode },
    // Sony, and the Samsung and RCA remotes with it. It has to come ahead of
    // RC5 and not behind it, because the confusion between them runs one way
    // only: a SIRC frame whose unit has drifted to 700 us offers RC5 a bit
    // time of 1.4 ms and a record of runs that all measure half a bit or a
    // whole one, which is everything the bi-phase reader asks for. The other
    // direction cannot happen - RC6's leader is 3:1 where this one is 4:1, and
    // behind the leader every space in a SIRC frame is one unit where a
    // bi-phase code's vary with the data. So the specific claim goes first.
    { PROTO_SIRC,    sirc_decode },
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
    // Ahead of the shape-matchers, and it has to be: a trace pin at 1 Mbit
    // puts runs of three to nine samples on the wire at a 100 ns timebase,
    // which is a WS2812 strip's territory exactly, and the strip decoder gets
    // four bytes out of it. What settles the argument is that this one is
    // confirmed by a packet grammar - headers, payload sizes, reserved
    // discriminators, a synchronisation sequence - and the strip decoder is
    // confirmed by two pulse widths. The more specific claim goes first.
    { PROTO_SWO,     swo_decode },
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
    // Ahead of the generic readers, and it has earned the place the same way
    // CAN and DShot did: a transaction is reported only once its address
    // parity, its stop and park bits, its acknowledgement and its data parity
    // all agree, and it takes two of those before it will claim a record.
    { PROTO_SWD,     swd_decode },
    // Ahead of the servo decoder, which is the signal it shares a pin with -
    // and the two cannot take each other's records in either direction. A
    // servo line's pulses are 20 ms apart, so every interval on it is a sync
    // gap and no channels ever fall between two of them; a PPM stream's are
    // 1 to 2 ms apart, which is below the slowest frame rate a servo runs at.
    // The order is here because the QUESTION is worth asking in this order:
    // one pin of a receiver carries one channel and the other carries all of
    // them, and the more specific reading of a pulse train is the sum signal.
    { PROTO_PPM,     ppm_decode },
    { PROTO_SERVO,   servo_decode },
    { PROTO_UART,    uart_decode },
    // Dead last but for the raw reader, and it never wins anyway: a data
    // line without its clock reports itself ambiguous every time, so auto
    // mode always passes over it. It is here so that asking for it by name
    // reaches it, and for no other reason.
    { PROTO_SPI,     spi_decode },
    { PROTO_RAW,     raw_decode },
  };

  // Decoded straight into the caller's result, and there is no second copy
  // anywhere. There WAS one - a static LogicResult the cascade filled and
  // then assigned across on a match - and it cost 776 bytes of TCM to buy
  // nothing: every decoder opens by memsetting the result it was handed, so
  // a decoder that turns the record down leaves behind only what the NEXT
  // one is about to overwrite. The one case that copy really covered is the
  // last decoder failing, and one memset on the way out covers it instead.
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

    if (decoders[i].fn(data, size, offset, period_ns, scratch, out) > 0)
    {
      // A decoder can report frames it cannot tell apart from something
      // simpler: a UART record that never catches the line at rest and holds
      // one repeated byte value IS a square wave, sample for sample. Guessing
      // the protocol from that is not on, so in auto mode it falls through to
      // the next decoder. Asked for UART by name - by the user, or by the
      // caller's own earlier match - the question is already answered and the
      // frames stand.
      if (out->ambiguous && forced == PROTO_AUTO)
        continue;

      scratch->cascade = 0;

      return out->count;
    }
  }

  // Nothing matched, and the result still holds whatever the LAST decoder
  // made of the record before turning it down. Callers read `count` rather
  // than this return value - the scope does, on the very next line - so the
  // failure has to be written into the result and not only reported.
  memset(out, 0, sizeof(*out));

  scratch->cascade = 0;

  return 0;
}
