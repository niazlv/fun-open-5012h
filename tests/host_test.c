/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for measure.c and fft.c: generate synthetic waveforms with
 * known parameters and check the measured values against ground truth.
 *
 * Build & run (no hardware needed), from the repository root:
 *   make test
 *
 * The source list lives in tests/Makefile. It used to be spelled out here,
 * where nothing compiled it and a moved file left it quietly wrong.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "measure.h"
#include "fft.h"
#include "classify.h"
#include "logic_decode.h"
#include "record_window.h"
#include "trend.h"
#include "config.h"
#include "buttons.h"
#include "input.h"

#define ZERO_POINT 128

static int g_failures = 0;

// What input.c reaches for. The clock is driven by the test rather than read,
// so the double-click window can be crossed without sleeping; the flag it
// blits into the status corner has nowhere to go here.
Config config;
static uint32_t g_input_now;

uint32_t timer_ms(void) { return g_input_now; }
void timer_add(int *timer) { (void)timer; }
void timer_remove(int *timer) { (void)timer; }

void lcd_draw_buf(int x, int y, int w, int h, const uint16_t *buf)
{
  (void)x; (void)y; (void)w; (void)h; (void)buf;
}

static void check_near(const char *name, double got, double want, double tol_pct)
{
  double tol = fabs(want) * tol_pct / 100.0;

  if (tol < 1e-9)
    tol = 1e-9;

  if (fabs(got - want) <= tol)
  {
    printf("  PASS %-28s got %12.3f  want %12.3f\n", name, got, want);
  }
  else
  {
    printf("  FAIL %-28s got %12.3f  want %12.3f (tol %.1f%%)\n", name, got, want, tol_pct);
    g_failures++;
  }
}

// Synthesize into a ring buffer with an arbitrary start offset to exercise
// the wrap handling
static void synth(uint8_t *buf, int size, int offset,
    double (*fn)(double phase, void *arg), void *arg,
    double freq_hz, double period_ns, double amp, double dc)
{
  for (int i = 0; i < size; i++)
  {
    double t = i * period_ns * 1e-9;
    double phase = fmod(freq_hz * t, 1.0);
    double v = dc + amp * fn(phase, arg);
    int c = (int)lround(v);

    if (c < 0) c = 0;
    if (c > 255) c = 255;

    buf[(offset + i) % size] = (uint8_t)c;
  }
}

// 8N1 into a record: idle high, start bit low, LSB first, frames back to back
// with no inter-frame gap - which is what a UART actually sends. start_ns may
// be negative, for a record that opens in the middle of a message.
static void synth_uart(uint8_t *buf, int size, double period_ns, double baud,
    const char *msg, double start_ns)
{
  double bit_ns = 1e9 / baud;
  int len = (int)strlen(msg);

  for (int i = 0; i < size; i++)
  {
    double x = i * period_ns - start_ns;
    int level = 1;

    if (x >= 0)
    {
      int b = (int)(x / bit_ns);
      int frame = b / 10, pos = b % 10;

      if (frame < len)
        level = (pos == 0) ? 0 : (pos == 9) ? 1 : ((msg[frame] >> (pos - 1)) & 1);
    }

    buf[i] = level ? 200 : 56;
  }
}

// 1-Wire at 1 us per sample: the bus a DS18B20 or an iButton key actually
// puts on the probe. Reset + presence, then bytes as bit slots - LSB first,
// a short low for '1' and a long low for '0'.
typedef struct { uint8_t *buf; int size; int pos; } OwGen;

static void ow_level(OwGen *g, int level, int us)
{
  int end = g->pos + us;

  for (; g->pos < end && g->pos < g->size; g->pos++)
    g->buf[g->pos] = level ? 200 : 56;
}

static void ow_reset(OwGen *g)
{
  ow_level(g, 1, 100);
  ow_level(g, 0, 480);  // reset
  ow_level(g, 1, 70);
  ow_level(g, 0, 100);  // presence
  ow_level(g, 1, 200);
}

static void ow_byte(OwGen *g, uint8_t v)
{
  for (int b = 0; b < 8; b++)
  {
    int bit = (v >> b) & 1;

    ow_level(g, 0, bit ? 6 : 60);
    ow_level(g, 1, bit ? 64 : 10);
  }
}

static void ow_bytes(OwGen *g, const uint8_t *d, int n)
{
  for (int i = 0; i < n; i++)
    ow_byte(g, d[i]);
}

// One bus slot, for the SEARCH ROM triplets, which are not byte-aligned work
static void ow_bit(OwGen *g, int bit)
{
  ow_level(g, 0, bit ? 6 : 60);
  ow_level(g, 1, bit ? 64 : 10);
}

// Dallas CRC8: the tests build valid ROMs and scratchpads rather than
// hard-coding check bytes, so a changed payload stays a valid one
static uint8_t ow_crc(const uint8_t *d, int n)
{
  uint8_t crc = 0;

  while (n--)
  {
    uint8_t b = *d++;

    for (int i = 0; i < 8; i++)
    {
      uint8_t mix = (uint8_t)((crc ^ b) & 1);

      crc >>= 1;
      if (mix) crc ^= 0x8C;
      b >>= 1;
    }
  }

  return crc;
}

// CAN 2.0A/2.0B onto a wire: build the bit sequence a controller would send,
// stuff it, and paint it into the record at a given bit time. `inv` is the
// CAN_H view, where dominant is the HIGH level.
typedef struct
{
  uint8_t *buf;
  int      size;
  int      pos;         // samples written
  double   bit_samples;
  int      inv;
  uint16_t crc;
  uint16_t crc_xor;     // corrupt the transmitted CRC, for the refusal tests
  int      last, same;  // stuffing state
} CanGen;

static void can_g_raw(CanGen *g, int bit)
{
  // recessive (1) is the resting level; on CAN_H that is the low one
  int level = g->inv ? !bit : bit;
  int end = (int)((g->pos / g->bit_samples + 1) * g->bit_samples + 0.5);

  for (; g->pos < end && g->pos < g->size; g->pos++)
    g->buf[g->pos] = level ? 200 : 56;
}

// One frame bit: stuffed, and folded into the CRC when asked
static void can_g_bit(CanGen *g, int bit, int crc_it)
{
  if (g->same >= 5)
  {
    can_g_raw(g, !g->last);          // the stuff bit the receiver must drop
    g->last = !g->last;
    g->same = 1;
  }

  if (crc_it)
  {
    uint16_t inv = (uint16_t)(((g->crc >> 14) & 1) ^ (bit & 1));

    g->crc = (uint16_t)((g->crc << 1) & 0x7FFF);
    if (inv) g->crc ^= 0x4599;
  }

  can_g_raw(g, bit);

  if (bit == g->last) g->same++;
  else { g->last = bit; g->same = 1; }
}

static void can_g_field(CanGen *g, uint32_t v, int bits, int crc_it)
{
  for (int i = bits - 1; i >= 0; i--)
    can_g_bit(g, (int)((v >> i) & 1), crc_it);
}

static void can_g_idle(CanGen *g, int bits)
{
  for (int i = 0; i < bits; i++)
    can_g_raw(g, 1);

  g->last = 1;
  g->same = 0;
}

// A whole frame, ACK slot included: ack=1 is a bus where somebody heard it
static void can_g_frame(CanGen *g, uint32_t id, int ext, int rtr,
    const uint8_t *data, int dlc, int ack)
{
  g->crc = 0;
  g->last = 1;
  g->same = 0;

  can_g_bit(g, 0, 1);                       // SOF

  if (ext)
  {
    can_g_field(g, id >> 18, 11, 1);        // base id
    can_g_bit(g, 1, 1);                     // SRR
    can_g_bit(g, 1, 1);                     // IDE
    can_g_field(g, id & 0x3FFFF, 18, 1);    // extended id
    can_g_bit(g, rtr, 1);                   // RTR
    can_g_bit(g, 0, 1);                     // r1
  }
  else
  {
    can_g_field(g, id, 11, 1);
    can_g_bit(g, rtr, 1);                   // RTR
    can_g_bit(g, 0, 1);                     // IDE
  }

  can_g_bit(g, 0, 1);                       // r0
  can_g_field(g, (uint32_t)dlc, 4, 1);

  if (!rtr)
  {
    for (int i = 0; i < dlc && i < 8; i++)
      can_g_field(g, data[i], 8, 1);
  }

  uint16_t crc = (uint16_t)(g->crc ^ g->crc_xor);

  can_g_field(g, crc, 15, 0);
  can_g_bit(g, 1, 0);                       // CRC delimiter
  g->same = 0;                              // fixed form: no stuffing past here
  can_g_raw(g, ack ? 0 : 1);                // ACK slot
  can_g_raw(g, 1);                          // ACK delimiter
  can_g_idle(g, 7 + 3);                     // EOF + interframe space
}

// CAN FD (ISO 11898-1:2015) onto a wire. Separate from the classic generator
// because almost nothing about the tail of the frame is shared: two bit
// times with the switch at the SAMPLE POINT of BRS rather than at its edge,
// a CRC over the bits as transmitted from a register that starts at one,
// fixed stuff bits in the CRC field, and a stuff count in front of it.
//
// `sp` is where the sample point sits. It is a knob because the wire does not
// carry it and the decoder has to work without knowing - so the tests run it
// at both ends of the range a real controller is configured over.
typedef struct
{
  uint8_t *buf;
  int      size;
  double   t;           // exact position of the next bit boundary, in samples
  double   arb;         // arbitration bit time, in samples
  double   data;        // data phase bit time, in samples
  double   sp;          // sample point, as a fraction of a bit
  int      inv;         // the CAN_H view, where dominant is the HIGH level
  uint32_t c17, c21;
  uint32_t crc_xor;     // corrupt the transmitted CRC, for the refusal tests
  int      last, same;  // dynamic stuffing state
  int      last_raw;    // the last bit that went out, for the fixed stuff bits
  int      stuffs;      // dynamic stuff bits since the SOF
  int      crc_on;
  int      fast;        // the data phase is running
  // The 2012 Bosch release: no stuff count, and a CRC register from zero.
  // A decade obsolete, and the decoder says so rather than reading it wrong.
  int      noniso;
} FdGen;

// The two CAN FD polynomials, written out of their exponents rather than
// copied as a constant: x17+x16+x14+x13+x11+x6+x4+x+1 and
// x21+x20+x13+x11+x7+x4+x3+1. See the check-value test, which is what says
// these are the right numbers and not merely the same numbers twice.
#define FD_CRC17_POLY  ((1u<<16)|(1u<<14)|(1u<<13)|(1u<<11)|(1u<<6)|(1u<<4)| \
                        (1u<<3)|(1u<<1)|1u)
#define FD_CRC21_POLY  ((1u<<20)|(1u<<13)|(1u<<11)|(1u<<7)|(1u<<4)|(1u<<3)|1u)

static void fd_g_crc(FdGen *g, int bit)
{
  uint32_t inv;

  inv = ((g->c17 >> 16) & 1u) ^ (uint32_t)(bit & 1);
  g->c17 = ((g->c17 << 1) & 0x1FFFFu) ^ (inv ? FD_CRC17_POLY : 0u);

  inv = ((g->c21 >> 20) & 1u) ^ (uint32_t)(bit & 1);
  g->c21 = ((g->c21 << 1) & 0x1FFFFFu) ^ (inv ? FD_CRC21_POLY : 0u);
}

// One bit of any width, painted where it actually falls rather than on a
// grid: the BRS bit and the CRC delimiter are neither one bit time nor the
// other, so there is no grid to be on
static void fd_g_paint(FdGen *g, int bit, double w)
{
  int level = g->inv ? !bit : bit;
  int a = (int)(g->t + 0.5);
  int b = (int)(g->t + w + 0.5);

  for (int i = a; i < b; i++)
  {
    if (i >= 0 && i < g->size)
      g->buf[i] = level ? 200 : 56;
  }

  g->t += w;
  g->last_raw = bit;

  if (g->crc_on)
    fd_g_crc(g, bit);
}

static void fd_g_raw(FdGen *g, int bit)
{
  fd_g_paint(g, bit, g->fast ? g->data : g->arb);
}

// One frame bit, dynamically stuffed - and the stuff bit counted, because the
// stuff count field at the end of the frame is that number
static void fd_g_bit(FdGen *g, int bit)
{
  if (g->same >= 5)
  {
    fd_g_raw(g, !g->last);
    g->last = !g->last;
    g->same = 1;
    g->stuffs++;
  }

  fd_g_raw(g, bit);

  if (bit == g->last) g->same++;
  else { g->last = bit; g->same = 1; }
}

static void fd_g_field(FdGen *g, uint32_t v, int bits)
{
  for (int i = bits - 1; i >= 0; i--)
    fd_g_bit(g, (int)((v >> i) & 1));
}

// A fixed stuff bit: the inverse of the bit before it, in the CRC but not of
// it, and not counted
static void fd_g_fixed(FdGen *g)
{
  int on = g->crc_on;

  g->crc_on = 0;
  fd_g_raw(g, !g->last_raw);
  g->crc_on = on;
}

static void fd_g_idle(FdGen *g, int bits)
{
  for (int i = 0; i < bits; i++)
    fd_g_raw(g, 1);

  g->last = 1;
  g->same = 0;
}

static const uint8_t g_fd_len[16] =
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

// The stuff count: three bits of gray code and a parity bit under them
static int fd_g_sc(int stuffs)
{
  int v = stuffs & 7;
  int gray = v ^ (v >> 1);

  return (gray << 1) | (((gray >> 2) ^ (gray >> 1) ^ gray) & 1);
}

static void fd_g_frame(FdGen *g, uint32_t id, int ext, int brs, int esi,
    const uint8_t *data, int dlc, int ack)
{
  int len = g_fd_len[dlc & 15];
  int nbits, i;
  uint32_t crc;

  g->c17 = g->noniso ? 0u : (1u << 16);
  g->c21 = g->noniso ? 0u : (1u << 20);
  g->last = 1;
  g->same = 0;
  g->stuffs = 0;
  g->crc_on = 1;
  g->fast = 0;

  fd_g_bit(g, 0);                        // SOF

  if (ext)
  {
    fd_g_field(g, id >> 18, 11);         // base id
    fd_g_bit(g, 1);                      // SRR
    fd_g_bit(g, 1);                      // IDE
    fd_g_field(g, id & 0x3FFFF, 18);     // extended id
    fd_g_bit(g, 0);                      // RRS - dominant, FD has no remotes
    fd_g_bit(g, 1);                      // FDF
  }
  else
  {
    fd_g_field(g, id, 11);
    fd_g_bit(g, 0);                      // RRS
    fd_g_bit(g, 0);                      // IDE
    fd_g_bit(g, 1);                      // FDF
  }

  fd_g_bit(g, 0);                        // res

  if (brs)
  {
    // The bit rate switches at the SAMPLE POINT of this bit, so it is
    // arbitration-timed up to there and data-timed after
    fd_g_paint(g, 1, g->sp * g->arb + (1.0 - g->sp) * g->data);
    g->last = 1;
    g->same = 1;
    g->fast = 1;
  }
  else
  {
    fd_g_bit(g, 0);
  }

  fd_g_bit(g, esi ? 1 : 0);
  fd_g_field(g, (uint32_t)dlc, 4);

  for (i = 0; i < len; i++)
    fd_g_field(g, data[i], 8);

  // The CRC field: one fixed stuff bit, the stuff count (which the CRC still
  // covers), then the CRC itself with a fixed stuff bit every four bits. A
  // non-ISO frame has no stuff count, so the one fixed bit in front of the
  // CRC sequence is the one the loop below puts there.
  if (!g->noniso)
  {
    int sc = fd_g_sc(g->stuffs);

    fd_g_fixed(g);

    for (i = 3; i >= 0; i--)
      fd_g_raw(g, (sc >> i) & 1);
  }

  nbits = (len <= 16) ? 17 : 21;
  crc = ((nbits == 17) ? g->c17 : g->c21) ^ g->crc_xor;
  crc &= (nbits == 17) ? 0x1FFFFu : 0x1FFFFFu;
  g->crc_on = 0;

  for (i = 0; i < nbits; i++)
  {
    if (0 == (i % 4))
      fd_g_fixed(g);

    fd_g_raw(g, (int)((crc >> (nbits - 1 - i)) & 1));
  }

  // ...and the rate switches back at the sample point of the delimiter
  fd_g_paint(g, 1, g->sp * (g->fast ? g->data : g->arb) +
      (1.0 - g->sp) * g->arb);
  g->fast = 0;

  fd_g_raw(g, ack ? 0 : 1);              // ACK slot
  fd_g_raw(g, 1);                        // ACK delimiter
  fd_g_idle(g, 7 + 3);                   // EOF + interframe space
}

// DHT11 / DHT22 onto a wire at 1 us per sample: the host's start pulse, the
// sensor's 80/80 answer, then forty bit slots whose HIGH carries the value
typedef struct { uint8_t *buf; int size; int pos; } DhtGen;

static void dht_g_level(DhtGen *g, int level, int us)
{
  int end = g->pos + us;

  for (; g->pos < end && g->pos < g->size; g->pos++)
    g->buf[g->pos] = level ? 200 : 56;
}

// start_us = 0 leaves the host's pulse and the response out, for a record
// that only caught the data
static void dht_g_frame(DhtGen *g, const uint8_t *by, int start_us)
{
  if (start_us > 0)
  {
    dht_g_level(g, 1, 100);
    dht_g_level(g, 0, start_us);   // 18 ms for a DHT11, about 1 for a DHT22
    dht_g_level(g, 1, 30);
    dht_g_level(g, 0, 80);         // the sensor answers
    dht_g_level(g, 1, 80);
  }
  else
  {
    dht_g_level(g, 1, 400);
  }

  for (int i = 0; i < 40; i++)
  {
    int bit = (by[i / 8] >> (7 - (i % 8))) & 1;

    dht_g_level(g, 0, 50);
    dht_g_level(g, 1, bit ? 70 : 26);
  }

  dht_g_level(g, 0, 50);           // the sensor lets go
}

// SENT (SAE J2716): every pulse is a falling edge, five ticks low and high
// for the rest, and the whole value is the time from one falling edge to the
// next. `inv` swaps the levels, for a probe behind an inverting buffer.
typedef struct
{
  uint8_t *buf;
  int      size;
  int      pos;
  double   tick;     // samples per tick
  int      inv;
} SentGen;

static void sent_g_level(SentGen *g, int level, double ticks)
{
  int end = (int)(g->pos + ticks * g->tick + 0.5);
  int v = g->inv ? !level : level;

  for (; g->pos < end && g->pos < g->size; g->pos++)
    g->buf[g->pos] = v ? 200 : 56;
}

// One pulse of `ticks` total: five low, the remainder high
static void sent_g_pulse(SentGen *g, int ticks)
{
  sent_g_level(g, 0, 5);
  sent_g_level(g, 1, ticks - 5);
}

static uint8_t sent_t_crc(const uint8_t *nib, int n, int recommended)
{
  static const uint8_t tab[16] =
      { 0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5 };
  uint8_t crc = 5;

  for (int i = 0; i < n; i++)
    crc = (uint8_t)(tab[crc] ^ (nib[i] & 0xF));

  return recommended ? tab[crc] : crc;
}

// sync, status, data nibbles, CRC, then a pause long enough to be one
static void sent_g_frame(SentGen *g, uint8_t status, const uint8_t *nib,
    int n, int crc)
{
  sent_g_pulse(g, 56);
  sent_g_pulse(g, 12 + status);

  for (int i = 0; i < n; i++)
    sent_g_pulse(g, 12 + (nib[i] & 0xF));

  sent_g_pulse(g, 12 + (crc & 0xF));
  sent_g_pulse(g, 90);          // pause: longer than any nibble
}

// A servo line: `n` pulses of `width_us` on a grid of `frame_us`, idle low,
// with a lead-in so the first pulse is not the run the record's start cut
static void servo_g(uint8_t *buf, int size, int period_ns, int width_us,
    int frame_us, int n)
{
  int lead = 500;
  int w = (int)((int64_t)width_us * 1000 / period_ns);
  int f = (int)((int64_t)frame_us * 1000 / period_ns);

  memset(buf, 56, size);

  for (int k = 0; k < n; k++)
  {
    int s = lead + k * f;

    for (int i = 0; i < w && s + i < size; i++)
      buf[s + i] = 200;
  }
}

// MIDI on the wire: 8N1 at 31250 baud, idle high, LSB first - a UART, and
// the position is kept in fractional samples so the bit grid does not drift
// over a record the way integer stepping would
typedef struct { uint8_t *buf; int size; double t; double sps_bit; int inv; } MidiGen;

static void midi_g_run(MidiGen *g, int level, double bits)
{
  int a = (int)(g->t + 0.5);

  g->t += bits * g->sps_bit;

  int b = (int)(g->t + 0.5);
  int v = ((level ^ g->inv) & 1) ? 200 : 56;

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = (uint8_t)v;
}

static void midi_g_byte(MidiGen *g, uint8_t v)
{
  midi_g_run(g, 0, 1);                    // start bit

  for (int b = 0; b < 8; b++)
    midi_g_run(g, (v >> b) & 1, 1);       // data, LSB first

  midi_g_run(g, 1, 1);                    // stop bit
}

static void midi_g(MidiGen *g, uint8_t *buf, int size, double period_ns, int inv)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_bit = 1e9 / 31250.0 / period_ns;
  g->inv = inv;

  memset(buf, inv ? 56 : 200, (size_t)size);

  midi_g_run(g, 1, 20);                   // the line at rest before the first
}

static void midi_g_bytes(MidiGen *g, const uint8_t *d, int n)
{
  for (int i = 0; i < n; i++)
    midi_g_byte(g, d[i]);
}

// LIN: a break, a 0x55 sync byte, a protected identifier and a response.
// 8N1 like the rest, but the rate comes off the sync byte rather than being
// agreed in advance, so the generator states it and the decoder must find it.
typedef struct { uint8_t *buf; int size; double t; double sps_bit; int inv; } LinGen;

static void lin_g_run(LinGen *g, int recessive, double bits)
{
  int a = (int)(g->t + 0.5);

  g->t += bits * g->sps_bit;

  int b = (int)(g->t + 0.5);
  int v = ((recessive ^ g->inv) & 1) ? 200 : 56;

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = (uint8_t)v;
}

static void lin_g_byte(LinGen *g, uint8_t v)
{
  lin_g_run(g, 0, 1);                     // start bit: dominant

  for (int b = 0; b < 8; b++)
    lin_g_run(g, (v >> b) & 1, 1);        // LSB first, recessive is a one

  lin_g_run(g, 1, 1);                     // stop bit
}

static void lin_g(LinGen *g, uint8_t *buf, int size, double period_ns,
    double baud, int inv)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_bit = 1e9 / baud / period_ns;
  g->inv = inv;

  memset(buf, inv ? 56 : 200, (size_t)size);

  lin_g_run(g, 1, 20);                    // the bus at rest
}

static uint8_t lin_g_pid(uint8_t id)
{
  int b0 = (id >> 0) & 1, b1 = (id >> 1) & 1, b2 = (id >> 2) & 1;
  int b3 = (id >> 3) & 1, b4 = (id >> 4) & 1, b5 = (id >> 5) & 1;
  int p0 = b0 ^ b1 ^ b2 ^ b4;
  int p1 = !(b1 ^ b3 ^ b4 ^ b5);

  return (uint8_t)((id & 0x3F) | (p0 << 6) | (p1 << 7));
}

static uint8_t lin_g_csum(const uint8_t *d, int n, uint8_t pid, int enhanced)
{
  unsigned sum = enhanced ? pid : 0u;

  for (int i = 0; i < n; i++)
  {
    sum += d[i];

    if (sum > 0xFF)
      sum -= 0xFF;
  }

  return (uint8_t)(~sum);
}

// The header: 13 dominant bits, a delimiter, the sync byte and the identifier
static void lin_g_header(LinGen *g, uint8_t id)
{
  lin_g_run(g, 0, 13);                    // break
  lin_g_run(g, 1, 1);                     // break delimiter
  lin_g_byte(g, 0x55);
  lin_g_byte(g, lin_g_pid(id));
}

static void lin_g_frame(LinGen *g, uint8_t id, const uint8_t *d, int n,
    int enhanced)
{
  lin_g_header(g, id);

  for (int i = 0; i < n; i++)
    lin_g_byte(g, d[i]);

  lin_g_byte(g, lin_g_csum(d, n, lin_g_pid(id), enhanced));
  lin_g_run(g, 1, 8);                     // inter-frame space
}

// EV1527: idle LOW, pulses high, everything measured in T. Sync is 1T high
// and 31T low; a '0' is 1T high and 3T low, a '1' the other way round. Twenty
// bits of address then four of buttons, most significant first.
typedef struct { uint8_t *buf; int size; double t; double sps_t; int inv; } EvGen;

static void ev_g_run(EvGen *g, int high, double ticks)
{
  int a = (int)(g->t + 0.5);

  g->t += ticks * g->sps_t;

  int b = (int)(g->t + 0.5);
  int v = ((high ^ g->inv) & 1) ? 200 : 56;

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = (uint8_t)v;
}

static void ev_g(EvGen *g, uint8_t *buf, int size, double period_ns,
    double t_us, int inv)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_t = t_us * 1000.0 / period_ns;
  g->inv = inv;

  memset(buf, inv ? 200 : 56, (size_t)size);

  ev_g_run(g, 0, 40);                     // the line at rest
}

static void ev_g_frame(EvGen *g, uint32_t addr, uint8_t key)
{
  uint32_t v = ((addr & 0xFFFFF) << 4) | (key & 0x0F);

  ev_g_run(g, 1, 1);                      // sync: 1T of pulse...
  ev_g_run(g, 0, 31);                     // ...and 31T of gap

  for (int i = 23; i >= 0; i--)           // MSB first
  {
    int bit = (v >> i) & 1;

    ev_g_run(g, 1, bit ? 3 : 1);
    ev_g_run(g, 0, bit ? 1 : 3);
  }
}

// DShot: sixteen bits, constant period, the duty carries the value. High for
// 3/8 of the bit is a zero and for 3/4 a one; the line rests low and the
// frame is followed by idle until the next control loop.
typedef struct { uint8_t *buf; int size; double t; double sps_bit; int inv; } DsGen;

static void ds_g_run(DsGen *g, int high, double bits)
{
  int a = (int)(g->t + 0.5);

  g->t += bits * g->sps_bit;

  int b = (int)(g->t + 0.5);
  int v = ((high ^ g->inv) & 1) ? 200 : 56;

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = (uint8_t)v;
}

static void ds_g(DsGen *g, uint8_t *buf, int size, double period_ns,
    double kbit, int inv)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_bit = 1e6 / kbit / period_ns;
  g->inv = inv;

  memset(buf, inv ? 200 : 56, (size_t)size);

  ds_g_run(g, 0, 20);                     // the line at rest
}

static uint16_t ds_g_crc(uint16_t d, int inverted)
{
  uint8_t c = (uint8_t)((d ^ (d >> 4) ^ (d >> 8)) & 0x0F);

  return inverted ? (uint8_t)(~c & 0x0F) : c;
}

// value: 11 bits of throttle or command; telem: the request flag
static void ds_g_frame(DsGen *g, uint16_t value, int telem, int inverted)
{
  uint16_t d = (uint16_t)(((value & 0x7FF) << 1) | (telem ? 1 : 0));
  uint16_t frame = (uint16_t)((d << 4) | ds_g_crc(d, inverted));

  for (int i = 15; i >= 0; i--)           // MSB first
  {
    int one = (frame >> i) & 1;

    ds_g_run(g, 1, one ? 0.75 : 0.375);
    ds_g_run(g, 0, one ? 0.25 : 0.625);
  }

  ds_g_run(g, 0, 30);                     // idle until the next control loop
}

// An SPI master's MOSI line and nothing else: bits at a constant rate, MSB
// first, with an optional pause between bytes the way a bit-banged master
// leaves one. The clock is NOT generated - that is the whole point.
typedef struct { uint8_t *buf; int size; double t; double sps_bit; } SpiGen;

static void spi_g_level(SpiGen *g, int high, double bits)
{
  int a = (int)(g->t + 0.5);

  g->t += bits * g->sps_bit;

  int b = (int)(g->t + 0.5);

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = high ? (uint8_t)200 : (uint8_t)56;
}

static void spi_g(SpiGen *g, uint8_t *buf, int size, double period_ns,
    double clock_hz)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_bit = 1e9 / clock_hz / period_ns;

  memset(buf, 200, (size_t)size);       // the line idles high

  spi_g_level(g, 1, 20);
}

// gap_bits of idle after the byte: 0 for a hardware master running bytes
// back to back, more for a bit-banged one
static void spi_g_byte(SpiGen *g, uint8_t v, double gap_bits)
{
  for (int b = 7; b >= 0; b--)          // MSB first
    spi_g_level(g, (v >> b) & 1, 1);

  if (gap_bits > 0)
    spi_g_level(g, 1, gap_bits);
}

// Manchester: a transition in the middle of every bit, and one on the
// boundary only when two adjacent bits are the same. Generated in the G.E.
// Thomas convention - a rising mid-bit edge is a one - which is RC5's.
typedef struct { uint8_t *buf; int size; double t; double sps_half; } ManGen;

static void man_g_level(ManGen *g, int high, double halves)
{
  int a = (int)(g->t + 0.5);

  g->t += halves * g->sps_half;

  int b = (int)(g->t + 0.5);

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = high ? (uint8_t)200 : (uint8_t)56;
}

static void man_g(ManGen *g, uint8_t *buf, int size, double period_ns,
    double bps)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_half = 1e9 / bps / 2.0 / period_ns;

  memset(buf, 200, (size_t)size);       // the line rests high

  man_g_level(g, 1, 20);
}

static void man_g_bit(ManGen *g, int bit)
{
  man_g_level(g, bit ? 0 : 1, 1);       // first half...
  man_g_level(g, bit ? 1 : 0, 1);       // ...and the transition that IS the bit
}

// `nbits` of `value`, most significant first. `flat` is the index of a bit
// sent with NO transition in its middle - an encoding violation - or -1.
static void man_g_bits(ManGen *g, uint32_t value, int nbits, int flat)
{
  for (int i = nbits - 1; i >= 0; i--)
  {
    int k = nbits - 1 - i;
    int bit = (value >> i) & 1;

    if (k == flat)
      man_g_level(g, bit ? 0 : 1, 2);   // the whole bit at its first level
    else
      man_g_bit(g, bit);
  }

  man_g_level(g, 1, 40);                // ...and the line goes back to rest
}

static void man_g_frame(ManGen *g, uint32_t value, int nbits)
{
  man_g_bits(g, value, nbits, -1);
}

// RC5 / RC6 off an infrared demodulator: the output rests HIGH and a mark -
// carrier present - pulls it low. RC5 sends a one as silence then carrier,
// RC6 the other way round, which is the whole of the difference between them
// once the leader is past.
typedef struct { uint8_t *buf; int size; double t; double sps_bit; } IrGen;

static void ir_g_level(IrGen *g, int mark, double bits)
{
  int a = (int)(g->t + 0.5);

  g->t += bits * g->sps_bit;

  int b = (int)(g->t + 0.5);

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = mark ? (uint8_t)56 : (uint8_t)200;
}

static void ir_g(IrGen *g, uint8_t *buf, int size, double period_ns,
    double bit_us)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_bit = bit_us * 1000.0 / period_ns;

  memset(buf, 200, (size_t)size);      // no carrier: the output rests high

  ir_g_level(g, 0, 8);
}

// RC5: a one is silence then carrier
static void rc5_g_frame(IrGen *g, uint32_t value, int nbits)
{
  for (int i = nbits - 1; i >= 0; i--)
  {
    int one = (value >> i) & 1;

    ir_g_level(g, one ? 0 : 1, 0.5);
    ir_g_level(g, one ? 1 : 0, 0.5);
  }

  ir_g_level(g, 0, 20);
}

// RC6 mode 0: leader, then 21 bits of which the trailer is twice as wide,
// and a one is carrier then silence
static void rc6_g_frame(IrGen *g, uint32_t value, int nbits, int wide)
{
  ir_g_level(g, 1, 3.0);               // 2.666 ms of carrier
  ir_g_level(g, 0, 1.0);               // ...and 0.889 of silence

  for (int i = nbits - 1; i >= 0; i--)
  {
    int one = (value >> i) & 1;
    double h = (nbits - 1 - i == wide) ? 1.0 : 0.5;

    ir_g_level(g, one ? 1 : 0, h);
    ir_g_level(g, one ? 0 : 1, h);
  }

  ir_g_level(g, 0, 20);
}

// Sony SIRC off the same kind of demodulator: the output rests HIGH and a
// mark pulls it low. Everything is a multiple of T - the leader is 4T of
// mark and 1T of space, a '0' is 1T of mark and a '1' 2T, and the space
// between two bits is always one T.
typedef struct { uint8_t *buf; int size; double t; double sps_t; } SircGen;

static void sirc_g_level(SircGen *g, int mark, double ticks)
{
  int a = (int)(g->t + 0.5);

  g->t += ticks * g->sps_t;

  int b = (int)(g->t + 0.5);

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = mark ? (uint8_t)56 : (uint8_t)200;
}

static void sirc_g(SircGen *g, uint8_t *buf, int size, double period_ns,
    double t_us)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_t = t_us * 1000.0 / period_ns;

  memset(buf, 200, (size_t)size);      // no carrier: the output rests high

  sirc_g_level(g, 0, 12);
}

// LSB first, and the command is the bottom seven bits
static void sirc_g_frame(SircGen *g, uint32_t value, int nbits)
{
  sirc_g_level(g, 1, 4);               // the leader: 4T of mark...
  sirc_g_level(g, 0, 1);               // ...and 1T of space

  for (int i = 0; i < nbits; i++)
  {
    sirc_g_level(g, 1, ((value >> i) & 1) ? 2 : 1);
    sirc_g_level(g, 0, 1);
  }

  sirc_g_level(g, 0, 20);              // the gap to the next frame
}

// CPPM: narrow separator pulses on a line that rests low, and the value of a
// channel is the time from one separator to the next. A frame is n channels,
// a closing separator and then the sync gap that pads it out to frame_us.
typedef struct { uint8_t *buf; int size; double t; double sps_us; int inv; } PpmGen;

static void ppm_g_run(PpmGen *g, int high, double us)
{
  int a = (int)(g->t + 0.5);

  g->t += us * g->sps_us;

  int b = (int)(g->t + 0.5);
  int v = ((high ^ g->inv) & 1) ? 200 : 56;

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = (uint8_t)v;
}

static void ppm_g(PpmGen *g, uint8_t *buf, int size, double period_ns, int inv)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_us = 1000.0 / period_ns;
  g->inv = inv;

  memset(buf, inv ? 200 : 56, (size_t)size);

  ppm_g_run(g, 0, 2000);               // the line at rest in front of it all
}

static void ppm_g_frame(PpmGen *g, const int *ch, int n, int frame_us)
{
  const int mark = 400;                // the separator: fixed, and not the value
  int used = 0;

  for (int i = 0; i < n; i++)
  {
    ppm_g_run(g, 1, mark);
    ppm_g_run(g, 0, ch[i] - mark);
    used += ch[i];
  }

  // n channels take n+1 separators: this is the one that closes the last of
  // them, and the sync gap behind it pads the frame to its period
  ppm_g_run(g, 1, mark);
  ppm_g_run(g, 0, frame_us - used - mark);
}

// DALI on the pair: the bus rests HIGH and a transmitter pulls it down. A
// one is a low-to-high transition in the middle of the bit, a zero the other
// way round, and the two stop bits are the bus simply left alone.
typedef struct { uint8_t *buf; int size; double t; double sps_bit; } DaliGen;

static void dali_g_level(DaliGen *g, int high, double bits)
{
  int a = (int)(g->t + 0.5);

  g->t += bits * g->sps_bit;

  int b = (int)(g->t + 0.5);

  for (int i = a; i < b && i < g->size; i++)
    g->buf[i] = high ? (uint8_t)200 : (uint8_t)56;
}

static void dali_g(DaliGen *g, uint8_t *buf, int size, double period_ns)
{
  g->buf = buf;
  g->size = size;
  g->t = 0;
  g->sps_bit = 1e9 / 1200.0 / period_ns;

  memset(buf, 200, (size_t)size);
  dali_g_level(g, 1, 10);
}

static void dali_g_frame(DaliGen *g, uint32_t value, int nbits)
{
  for (int i = nbits - 1; i >= 0; i--)
  {
    int one = (value >> i) & 1;

    dali_g_level(g, one ? 0 : 1, 0.5);
    dali_g_level(g, one ? 1 : 0, 0.5);
  }

  dali_g_level(g, 1, 12);              // the stop bits and then some
}

// A forward frame: start bit, address byte, data byte
static void dali_g_fwd(DaliGen *g, uint8_t addr, uint8_t d)
{
  dali_g_frame(g, (1u << 16) | ((uint32_t)addr << 8) | d, 17);
}

// KNX TP1: 9600, and the line code is pulse PRESENCE. A zero is a ~35 us
// active pulse at the start of its bit period; a one is nothing at all. The
// bus rests high and a pulse pulls it down.
typedef struct { uint8_t *buf; int size; double t; double sps_bit; } KnxGen;

static void knx_g(KnxGen *g, uint8_t *buf, int size, double period_ns)
{
  g->buf = buf;
  g->size = size;
  g->sps_bit = 104167.0 / period_ns;
  g->t = 20 * g->sps_bit;

  memset(buf, 200, (size_t)size);
}

// One character: start(0), 8 data least significant first, even parity, stop
static void knx_g_char(KnxGen *g, uint8_t v)
{
  int bits[11];
  int ones = 0;

  bits[0] = 0;

  for (int k = 0; k < 8; k++)
  {
    bits[1 + k] = (v >> k) & 1;
    ones += bits[1 + k];
  }

  bits[9] = (ones & 1);                // even parity over data + parity
  bits[10] = 1;

  for (int k = 0; k < 11; k++)
  {
    if (bits[k])
      continue;                        // a one is silence

    int a = (int)(g->t + k * g->sps_bit + 0.5);
    int e = a + (int)(35000.0 / 104167.0 * g->sps_bit + 0.5);

    for (int i = a; i < e && i < g->size; i++)
      g->buf[i] = 56;
  }

  g->t += 13 * g->sps_bit;             // 11 slots and two of gap
}

// A whole telegram, with the check octet added
static void knx_g_telegram(KnxGen *g, const uint8_t *b, int n)
{
  uint8_t x = 0;

  for (int i = 0; i < n; i++)
  {
    knx_g_char(g, b[i]);
    x ^= b[i];
  }

  knx_g_char(g, (uint8_t)~x);
}

// SWO: an ordinary 8N1 line idling high, at whatever rate the trace clock
// divider landed on. What makes it SWO is the bytes, so the generator only
// has to put bytes on a wire.
typedef struct { uint8_t *buf; int size; double t; double sps_bit; } SwoGen;

static void swo_g(SwoGen *g, uint8_t *buf, int size, double period_ns,
    double baud)
{
  g->buf = buf;
  g->size = size;
  g->sps_bit = 1e9 / baud / period_ns;
  g->t = 20 * g->sps_bit;

  memset(buf, 200, (size_t)size);
}

static void swo_g_byte(SwoGen *g, uint8_t v)
{
  int bits[10];

  bits[0] = 0;                            // start

  for (int k = 0; k < 8; k++)
    bits[1 + k] = (v >> k) & 1;           // least significant first

  bits[9] = 1;                            // stop

  for (int k = 0; k < 10; k++)
  {
    int a = (int)(g->t + k * g->sps_bit + 0.5);
    int e = (int)(g->t + (k + 1) * g->sps_bit + 0.5);

    for (int i = a; i < e && i < g->size; i++)
      g->buf[i] = bits[k] ? 200 : 56;
  }

  g->t += 10 * g->sps_bit;
}

static void swo_g_bytes(SwoGen *g, const uint8_t *b, int n)
{
  for (int i = 0; i < n; i++)
    swo_g_byte(g, b[i]);
}

// SWD: bits at a clock the decoder cannot see. The turnaround cycles HOLD the
// previous level, which is what an undriven line on a real bus does and what
// the decoder must therefore ignore rather than read.
typedef struct
{
  uint8_t *buf; int size; double t; double sps_bit; int last; double drift;
} SwdGen;

static void swd_g(SwdGen *g, uint8_t *buf, int size, double period_ns,
    double hz)
{
  g->buf = buf;
  g->size = size;
  g->sps_bit = 1e9 / hz / period_ns;
  g->t = 20 * g->sps_bit;
  g->last = 0;
  g->drift = 0.0;

  memset(buf, 56, (size_t)size);          // the host holds SWDIO low at rest
}

static void swd_g_bit(SwdGen *g, int b)
{
  int a = (int)(g->t + 0.5);
  int e = (int)(g->t + g->sps_bit + 0.5);

  for (int i = a; i < e && i < g->size; i++)
    g->buf[i] = b ? 200 : 56;

  g->last = b;
  g->t += g->sps_bit;
  g->sps_bit *= 1.0 + g->drift;           // a bit-banged master slowing down
}

static void swd_g_trn(SwdGen *g)          // undriven: the line stays put
{
  swd_g_bit(g, g->last);
}

static void swd_g_idle(SwdGen *g, int n)
{
  for (int i = 0; i < n; i++)
    swd_g_bit(g, 0);
}

// One transaction. `a` is the register index A[3:2], `ack` is 1 OK, 2 WAIT,
// 4 FAULT, `flip` breaks the data parity on purpose, and `idle` is how many
// idle cycles follow it - zero being legal and the case that leaves the next
// packet's start bit with no rising edge of its own.
static void swd_g_txi(SwdGen *g, int ap, int write, int a, uint32_t data,
    int ack, int flip, int idle)
{
  int rnw = write ? 0 : 1;
  int a2 = a & 1, a3 = (a >> 1) & 1;
  int par = (ap ^ rnw ^ a2 ^ a3) & 1;
  int ones = 0;

  swd_g_bit(g, 1);                        // start
  swd_g_bit(g, ap);
  swd_g_bit(g, rnw);
  swd_g_bit(g, a2);
  swd_g_bit(g, a3);
  swd_g_bit(g, par);
  swd_g_bit(g, 0);                        // stop
  swd_g_bit(g, 1);                        // park
  swd_g_trn(g);

  for (int k = 0; k < 3; k++)
    swd_g_bit(g, (ack >> k) & 1);         // least significant first

  if (1 != ack)
  {
    swd_g_trn(g);
    swd_g_idle(g, idle);

    return;                               // WAIT and FAULT have no data phase
  }

  if (write)
    swd_g_trn(g);

  for (int k = 0; k < 32; k++)
  {
    int b = (int)((data >> k) & 1);

    ones += b;
    swd_g_bit(g, b);
  }

  swd_g_bit(g, (ones & 1) ^ (flip ? 1 : 0));

  if (!write)
    swd_g_trn(g);

  swd_g_idle(g, idle);
}

static void swd_g_tx(SwdGen *g, int ap, int write, int a, uint32_t data,
    int ack, int flip)
{
  swd_g_txi(g, ap, write, a, data, ack, flip, 8);
}

static void swd_g_reset(SwdGen *g)        // fifty-two clocks of ones
{
  for (int i = 0; i < 52; i++)
    swd_g_bit(g, 1);
}

static void swd_g_switch(SwdGen *g)       // 0xE79E, least significant first
{
  for (int k = 0; k < 16; k++)
    swd_g_bit(g, (0xE79E >> k) & 1);
}

// USB 1.x on ONE wire of the pair. NRZI - a zero is a transition, a one is
// the absence of one - with a zero stuffed in after every six ones, and an
// EOP that is SE0 for two bit times. SE0 is both wires low, so on whichever
// wire this is, it is low: the generator draws the wire and not the pair,
// which is exactly what the instrument sees.
typedef struct
{
  uint8_t *buf; int size; double t; double sps_bit;
  int level;        // where the line is now
  int j;            // ...and which level is J on this wire
  int ones;         // consecutive ones, for the stuffing
  double jit;       // edge jitter, in samples, either way
  double pj;        // ...as applied to the previous edge, so the two agree
  int noise;        // amplitude noise, in ADC counts
} UsbGen;

// Its own transcription of the two polynomials, so that a decoder agreeing
// with itself is not what the tests check. What pins them down is external:
// a SETUP token to address 0 endpoint 0 is 2D 00 10 on the wire and a
// zero-length DATA0 is C3 00 00, and both are asserted below.
static uint8_t tst_usb_crc5(uint32_t v, int bits)
{
  uint8_t c = 0x1F;

  for (int i = 0; i < bits; i++)
  {
    uint8_t x = (uint8_t)(((v >> i) ^ c) & 1);

    c >>= 1;

    if (x)
      c ^= 0x14;
  }

  return (uint8_t)(~c & 0x1F);
}

static uint16_t tst_usb_crc16(const uint8_t *d, int n)
{
  uint16_t c = 0xFFFF;

  for (int i = 0; i < n; i++)
  {
    c ^= d[i];

    for (int k = 0; k < 8; k++)
      c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
  }

  return (uint16_t)~c;
}

static void usb_g(UsbGen *g, uint8_t *buf, int size, double period_ns,
    double rate, int j_high)
{
  g->buf = buf;
  g->size = size;
  g->sps_bit = 1e9 / rate / period_ns;
  g->t = 30 * g->sps_bit;
  g->j = j_high;
  g->level = j_high;
  g->ones = 0;
  g->jit = 0.0;
  g->pj = 0.0;
  g->noise = 0;

  memset(buf, j_high ? 200 : 56, (size_t)size);   // the bus at rest, at J
}

// One held level. The jitter moves the edge at the END of the segment and is
// carried into the next one's start, so the boundaries stay shared and the
// nominal bit time never drifts - which is the point: what is being tested is
// an edge measured a sample late, not a transmitter running at the wrong rate.
static void usb_g_hold(UsbGen *g, int level, double bits)
{
  double j = (g->jit > 0.0)
      ? ((rand() / (double)RAND_MAX) * 2.0 - 1.0) * g->jit : 0.0;
  int a = (int)(g->t + g->pj + 0.5);
  int e = (int)(g->t + g->sps_bit * bits + j + 0.5);

  for (int i = a; i < e && i < g->size; i++)
  {
    int v = (level ? 200 : 56) +
        (g->noise ? (rand() % (2 * g->noise + 1)) - g->noise : 0);

    g->buf[i] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
  }

  g->pj = j;
  g->level = level;
  g->t += g->sps_bit * bits;
}

static void usb_g_bit(UsbGen *g, int b)
{
  if (!b)
    g->level = !g->level;                 // a zero is a transition

  usb_g_hold(g, g->level, 1.0);

  g->ones = b ? (g->ones + 1) : 0;

  if (6 == g->ones)                       // ...and six ones force one
  {
    g->level = !g->level;
    usb_g_hold(g, g->level, 1.0);
    g->ones = 0;
  }
}

static void usb_g_byte(UsbGen *g, uint8_t v)
{
  for (int k = 0; k < 8; k++)
    usb_g_bit(g, (v >> k) & 1);           // least significant bit first
}

// SYNC: 00000001, which from an idle J draws K J K J K J K K
static void usb_g_sync(UsbGen *g)
{
  g->ones = 0;

  for (int k = 0; k < 7; k++)
    usb_g_bit(g, 0);

  usb_g_bit(g, 1);
}

// EOP: two bit times of SE0 - low on either wire - then J, then the gap
static void usb_g_eop(UsbGen *g, double gap_bits)
{
  usb_g_hold(g, 0, 2.0);
  usb_g_hold(g, g->j, 1.0 + gap_bits);
}

static void usb_g_token(UsbGen *g, uint8_t pid, int addr, int ep, double gap)
{
  uint32_t v = (uint32_t)((addr & 0x7F) | ((ep & 0x0F) << 7));
  uint32_t f = v | ((uint32_t)tst_usb_crc5(v, 11) << 11);

  usb_g_sync(g);
  usb_g_byte(g, pid);
  usb_g_byte(g, (uint8_t)(f & 0xFF));
  usb_g_byte(g, (uint8_t)((f >> 8) & 0xFF));
  usb_g_eop(g, gap);
}

static void usb_g_sof(UsbGen *g, int frame, double gap)
{
  uint32_t v = (uint32_t)(frame & 0x7FF);
  uint32_t f = v | ((uint32_t)tst_usb_crc5(v, 11) << 11);

  usb_g_sync(g);
  usb_g_byte(g, 0xA5);
  usb_g_byte(g, (uint8_t)(f & 0xFF));
  usb_g_byte(g, (uint8_t)((f >> 8) & 0xFF));
  usb_g_eop(g, gap);
}

// A data packet with a CRC16 chosen by the caller: usb_g_data passes the
// right one, and the tests that want a broken packet pass a wrong one
static void usb_g_data_crc(UsbGen *g, uint8_t pid, const uint8_t *d, int n,
    uint16_t crc, double gap)
{
  usb_g_sync(g);
  usb_g_byte(g, pid);

  for (int i = 0; i < n; i++)
    usb_g_byte(g, d[i]);

  usb_g_byte(g, (uint8_t)(crc & 0xFF));
  usb_g_byte(g, (uint8_t)(crc >> 8));
  usb_g_eop(g, gap);
}

static void usb_g_data(UsbGen *g, uint8_t pid, const uint8_t *d, int n,
    double gap)
{
  usb_g_data_crc(g, pid, d, n, tst_usb_crc16(d, n), gap);
}

static void usb_g_hs(UsbGen *g, uint8_t pid, double gap)
{
  usb_g_sync(g);
  usb_g_byte(g, pid);
  usb_g_eop(g, gap);
}

// USB Power Delivery on CC. BMC: a transition at every bit boundary, and a
// one carries a second one in the middle of the bit. So a zero draws one run
// of a unit interval and a one draws two of half - and the code says nothing
// about levels, which is why the generator's starting level is arbitrary and
// the decoder must not care.
typedef struct
{
  uint8_t *buf; int size; double t; double sps_ui;
  int level;
  double jit; double pj; int noise;
} PdGen;

// Its own transcription of the 4b5b table and the polynomial, so that a
// decoder agreeing with itself is not what the tests check. The external
// anchor is CRC32("123456789") = 0xCBF43926, asserted below.
static const uint8_t tst_pd_4b5b[16] =
{
  0x1E, 0x09, 0x14, 0x15, 0x0A, 0x0B, 0x0E, 0x0F,
  0x12, 0x13, 0x16, 0x17, 0x1A, 0x1B, 0x1C, 0x1D,
};

#define TST_K_SYNC1 0x18
#define TST_K_SYNC2 0x11
#define TST_K_SYNC3 0x06
#define TST_K_RST1  0x07
#define TST_K_RST2  0x19
#define TST_K_EOP   0x0D

static uint32_t tst_pd_crc32(const uint8_t *d, int n)
{
  uint32_t c = 0xFFFFFFFFu;

  for (int i = 0; i < n; i++)
  {
    c ^= d[i];

    for (int k = 0; k < 8; k++)
      c = (c & 1u) ? ((c >> 1) ^ 0xEDB88320u) : (c >> 1);
  }

  return ~c;
}

static void pd_g(PdGen *g, uint8_t *buf, int size, double period_ns,
    double rate, int level)
{
  g->buf = buf;
  g->size = size;
  g->sps_ui = 1e9 / rate / period_ns;
  g->t = 8 * g->sps_ui;
  g->level = level;
  g->jit = 0.0;
  g->pj = 0.0;
  g->noise = 0;

  // CC at rest sits at its Rp/Rd divider level and does not swing at all;
  // 128 is the middle of the range the thresholder will find
  memset(buf, 128, (size_t)size);
}

static void pd_g_hold(PdGen *g, int level, double uis)
{
  double j = (g->jit > 0.0)
      ? ((rand() / (double)RAND_MAX) * 2.0 - 1.0) * g->jit : 0.0;
  int a = (int)(g->t + g->pj + 0.5);
  int e = (int)(g->t + g->sps_ui * uis + j + 0.5);

  for (int i = a; i < e && i < g->size; i++)
  {
    int v = (level ? 190 : 66) +
        (g->noise ? (rand() % (2 * g->noise + 1)) - g->noise : 0);

    g->buf[i] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
  }

  g->pj = j;
  g->level = level;
  g->t += g->sps_ui * uis;
}

// One BMC bit: the level always flips at the boundary, and a one flips again
// halfway through
static void pd_g_bit(PdGen *g, int b)
{
  if (b)
  {
    pd_g_hold(g, g->level, 0.5);
    g->level = !g->level;
    pd_g_hold(g, g->level, 0.5);
  }
  else
  {
    pd_g_hold(g, g->level, 1.0);
  }

  g->level = !g->level;
}

// A five-bit symbol, most significant bit of the table entry first
static void pd_g_sym(PdGen *g, uint8_t code)
{
  for (int k = 4; k >= 0; k--)
    pd_g_bit(g, (code >> k) & 1);
}

static void pd_g_byte(PdGen *g, uint8_t v)
{
  pd_g_sym(g, tst_pd_4b5b[v & 0x0F]);        // the low nibble goes first
  pd_g_sym(g, tst_pd_4b5b[(v >> 4) & 0x0F]);
}

// 64 bits of alternating 0 and 1, which is what the unit interval is measured
// off and what carries nothing else
static void pd_g_preamble(PdGen *g)
{
  for (int k = 0; k < 32; k++)
  {
    pd_g_bit(g, 0);
    pd_g_bit(g, 1);
  }
}

static void pd_g_os(PdGen *g, const uint8_t *k)
{
  for (int i = 0; i < 4; i++)
    pd_g_sym(g, k[i]);
}

static void pd_g_idle(PdGen *g, double uis)
{
  // The transmitter drives the last bit's TRAILING edge before it lets go of
  // CC, and it has to: without that edge the final bit has no boundary and is
  // not a bit at all. On a hard reset - which is an ordered set and nothing
  // else - that edge is the difference between seeing the event and not.
  pd_g_hold(g, g->level, 0.5);

  int a = (int)(g->t + g->pj + 0.5);
  int e = (int)(g->t + g->sps_ui * uis + 0.5);

  for (int i = a; i < e && i < g->size; i++)
    g->buf[i] = 128;                         // ...and then it lets go

  g->pj = 0.0;
  g->t += g->sps_ui * uis;
}

// A whole message: preamble, ordered set, header, objects, CRC32, EOP.
// `crc_over` lets a test send a checksum that is not the right one.
static void pd_g_msg_crc(PdGen *g, const uint8_t *os, uint16_t hdr,
    const uint32_t *obj, int ndo, uint32_t crc, double gap)
{
  uint8_t body[2 + 7 * 4];
  int n = 0;

  body[n++] = (uint8_t)(hdr & 0xFF);
  body[n++] = (uint8_t)(hdr >> 8);

  for (int i = 0; i < ndo; i++)
  {
    body[n++] = (uint8_t)(obj[i] & 0xFF);
    body[n++] = (uint8_t)(obj[i] >> 8);
    body[n++] = (uint8_t)(obj[i] >> 16);
    body[n++] = (uint8_t)(obj[i] >> 24);
  }

  pd_g_preamble(g);
  pd_g_os(g, os);

  for (int i = 0; i < n; i++)
    pd_g_byte(g, body[i]);

  for (int i = 0; i < 4; i++)
    pd_g_byte(g, (uint8_t)(crc >> (8 * i)));

  pd_g_sym(g, TST_K_EOP);
  pd_g_idle(g, gap);
}

static void pd_g_msg(PdGen *g, const uint8_t *os, uint16_t hdr,
    const uint32_t *obj, int ndo, double gap)
{
  uint8_t body[2 + 7 * 4];
  int n = 0;

  body[n++] = (uint8_t)(hdr & 0xFF);
  body[n++] = (uint8_t)(hdr >> 8);

  for (int i = 0; i < ndo; i++)
  {
    body[n++] = (uint8_t)(obj[i] & 0xFF);
    body[n++] = (uint8_t)(obj[i] >> 8);
    body[n++] = (uint8_t)(obj[i] >> 16);
    body[n++] = (uint8_t)(obj[i] >> 24);
  }

  pd_g_msg_crc(g, os, hdr, obj, ndo, tst_pd_crc32(body, n), gap);
}

// The header as the specification lays it out
static uint16_t tst_pd_hdr(int type, int ndo, int id, int rev, int src, int dfp)
{
  return (uint16_t)((type & 0x1F) | ((dfp ? 1 : 0) << 5) | ((rev & 3) << 6) |
      ((src ? 1 : 0) << 8) | ((id & 7) << 9) | ((ndo & 7) << 12));
}

// A fixed-supply Power Data Object: volts in fifty-millivolt units, amps in
// ten-milliamp ones
static uint32_t tst_pd_fixed(int mv, int ma)
{
  return ((uint32_t)(mv / 50) << 10) | (uint32_t)(ma / 10);
}

static double fn_sine(double ph, void *arg)     { (void)arg; return sin(2 * M_PI * ph); }
static double fn_square(double ph, void *arg)   { double d = *(double *)arg; return ph < d ? 1.0 : -1.0; }
static double fn_triangle(double ph, void *arg) { (void)arg; return ph < 0.5 ? (4 * ph - 1) : (3 - 4 * ph); }

int main(void)
{
  enum { SIZE = 24576 };
  static uint8_t buf[SIZE];
  Measure m;

  // --- 1 kHz sine, 1 us sample period, amplitude 100 counts, offset +10 ---
  // 24000 samples = exactly 24 periods, so the mean has no partial-period bias
  printf("sine 1 kHz amp=100 dc=+10 (offset 12345):\n");
  synth(buf, 24000, 12345 % 24000, fn_sine, NULL, 1000.0, 1000.0, 100.0, ZERO_POINT + 10);
  measure_run(buf, 24000, 12345 % 24000, 1000, ZERO_POINT, &m);
  check_near("frequency", m.frequency, 1000, 0.5);
  check_near("vpp counts", m.vmax - m.vmin, 200, 2);
  check_near("mean c100", m.mean_c100, 1000, 10);
  // RMS of sine with DC: sqrt(dc^2 + a^2/2) = sqrt(100 + 5000) = 71.41
  check_near("rms c100", m.rms_c100, 7141, 2);
  check_near("duty x10", m.duty_x10, 500, 4);

  // --- 30% duty square, 12.34 kHz ---
  printf("square 12.34 kHz duty=30%% amp=80:\n");
  double duty = 0.30;
  synth(buf, SIZE, 777, fn_square, &duty, 12340.0, 1000.0, 80.0, ZERO_POINT);
  measure_run(buf, SIZE, 777, 1000, ZERO_POINT, &m);
  check_near("frequency", m.frequency, 12340, 0.5);
  check_near("duty x10", m.duty_x10, 300, 4);
  check_near("rms c100", m.rms_c100, 8000, 2);

  // --- triangle 250 Hz: mean 0, rms = a/sqrt(3) ---
  printf("triangle 250 Hz amp=90:\n");
  synth(buf, SIZE, 0, fn_triangle, NULL, 250.0, 1000.0, 90.0, ZERO_POINT);
  measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
  check_near("frequency", m.frequency, 250, 0.5);
  check_near("rms c100", m.rms_c100, 90.0 / sqrt(3.0) * 100, 2.5);

  // --- noise robustness: sine + noise, hysteresis must prevent double counts ---
  printf("sine 5 kHz amp=40 + uniform noise +-6:\n");
  srand(1);
  synth(buf, SIZE, 100, fn_sine, NULL, 5000.0, 1000.0, 40.0, ZERO_POINT);
  for (int i = 0; i < SIZE; i++)
  {
    int v = buf[i] + (rand() % 13) - 6;
    buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
  }
  measure_run(buf, SIZE, 100, 1000, ZERO_POINT, &m);
  check_near("frequency", m.frequency, 5000, 1.0);

  // --- flat line: no frequency ---
  printf("flat line:\n");
  memset(buf, ZERO_POINT + 3, SIZE);
  measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
  check_near("frequency (none)", m.frequency, 0, 0);
  check_near("rms c100", m.rms_c100, 300, 1);

  // --- FFT: 1.9 MHz sine at 16 ns period (62.5 MS/s) ---
  printf("fft sine 1.9 MHz @ 16ns:\n");
  static float mag[FFT_BINS];
  synth(buf, SIZE, 4242, fn_sine, NULL, 1.9e6, 16.0, 90.0, ZERO_POINT);
  fft_spectrum(buf, SIZE, 4242, mag);
  check_near("peak freq", fft_peak_frequency(mag, 16), 1.9e6, 1.0);

  // --- FFT: two tones, stronger one wins ---
  printf("fft 3 MHz strong + 800 kHz weak @ 16ns:\n");
  for (int i = 0; i < SIZE; i++)
  {
    double t = i * 16e-9;
    double v = ZERO_POINT + 80 * sin(2 * M_PI * 3e6 * t) + 25 * sin(2 * M_PI * 8e5 * t);
    buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : (int)lround(v));
  }
  fft_spectrum(buf, SIZE, 0, mag);
  check_near("peak freq", fft_peak_frequency(mag, 16), 3e6, 1.0);

  // --- FFT: flat input has no peak ---
  printf("fft flat:\n");
  memset(buf, ZERO_POINT, SIZE);
  fft_spectrum(buf, SIZE, 0, mag);
  check_near("peak freq (none)", fft_peak_frequency(mag, 16), 0, 0);

  // ====================== whole-record spectrum =========================
  // The scope's own case: a 24576-sample record at 4 us covers 98 ms, so a
  // whole-record transform (decimation 48) resolves 10 Hz and can tell
  // mains apart from its harmonics. Transforming 512 raw samples of the
  // same record covers 2 ms and resolves 500 Hz: everything below is one
  // smear and the reported frequency is whatever the smear peaks at.
  {
    enum { REC = 24576, PER = 4000 };
    FftAnalysis an;
    int decim = fft_max_decimation(REC);

    check_near("max decimation", decim, 48, 0);

    // --- mains composite: 50 + 100 + 150 + 250 Hz, third harmonic the
    // strongest, plus HF interference above the decimated Nyquist ---
    printf("spectrum: mains composite (150 Hz is the tallest peak):\n");
    srand(7);

    for (int i = 0; i < REC; i++)
    {
      double t = i * PER * 1e-9;
      double v = ZERO_POINT
          + 60 * sin(2 * M_PI * 50.0 * t)
          + 25 * sin(2 * M_PI * 100.0 * t + 0.7)
          + 70 * sin(2 * M_PI * 150.0 * t + 1.3)
          + 15 * sin(2 * M_PI * 250.0 * t)
          + 20 * sin(2 * M_PI * 20000.0 * t)   // interference, aliases to 832 Hz
          + (rand() % 9) - 4;                  // noise

      buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : (int)lround(v));
    }

    fft_spectrum_decim(buf, REC, 0, decim, mag);
    fft_analyze(mag, PER * decim, &an);

    check_near("bin resolution", an.bin_hz, 1e9 / ((double)REC * PER), 1);
    check_near("nyquist", an.nyquist_hz, 1e9 / (2.0 * PER * decim), 1);
    check_near("fundamental", an.fundamental, 50.0, 4);
    check_near("strongest peak is 150", an.peak[0].freq, 150.0, 4);
    check_near("... labelled h3", an.peak[0].harmonic, 3, 0);
    check_near("peaks found", an.count >= 4, 1, 0);

    // Every harmonic must be in the list, at its own level relative to 150
    for (int h = 1; h <= 5; h++)
    {
      static const double amp[6] = { 0, 60, 25, 70, 0, 15 };
      char name[32];
      int found = -1;

      if (amp[h] == 0)
        continue;

      for (int i = 0; i < an.count; i++)
      {
        if (an.peak[i].harmonic == h)
          found = i;
      }

      snprintf(name, sizeof(name), "h%d present", h);
      check_near(name, found >= 0, 1, 0);

      if (found >= 0)
      {
        // Compared as an amplitude ratio: a relative tolerance on a dB
        // value near zero is meaninglessly tight
        snprintf(name, sizeof(name), "h%d level", h);
        check_near(name, pow(10.0, an.peak[found].rel_db / 20.0), amp[h] / 70.0, 10);
      }
    }

    // The 20 kHz interference folds to 832 Hz; the decimation filter has to
    // push it far below the signal instead of letting it become a peak
    {
      double worst = -99.0;

      for (int i = 0; i < an.count; i++)
      {
        if (an.peak[i].harmonic == 0 && an.peak[i].rel_db > worst)
          worst = an.peak[i].rel_db;
      }

      printf("  (strongest non-harmonic peak: %.1f dB)\n", worst);
      check_near("alias suppressed", worst < -20.0, 1, 0);
    }

    // --- missing fundamental: only 300/450/600 Hz present -> 150 Hz ---
    printf("spectrum: missing fundamental (300+450+600):\n");

    for (int i = 0; i < REC; i++)
    {
      double t = i * PER * 1e-9;
      double v = ZERO_POINT
          + 50 * sin(2 * M_PI * 300.0 * t)
          + 40 * sin(2 * M_PI * 450.0 * t)
          + 30 * sin(2 * M_PI * 600.0 * t);

      buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : (int)lround(v));
    }

    fft_spectrum_decim(buf, REC, 0, decim, mag);
    fft_analyze(mag, PER * decim, &an);
    check_near("fundamental", an.fundamental, 150.0, 4);
    check_near("300 Hz is h2", an.peak[0].harmonic, 2, 0);

    // --- a clean sine has no distortion and one peak ---
    printf("spectrum: clean 400 Hz sine:\n");
    synth(buf, REC, 3000, fn_sine, NULL, 400.0, PER, 90.0, ZERO_POINT);
    fft_spectrum_decim(buf, REC, 3000, decim, mag);
    fft_analyze(mag, PER * decim, &an);
    check_near("fundamental", an.fundamental, 400.0, 2);
    check_near("thd < 5%", an.thd_x10 >= 0 && an.thd_x10 < 50, 1, 0);

    // --- decim 1 must still agree with the plain transform ---
    printf("spectrum: decim 1 == fft_spectrum:\n");
    {
      static float mag2[FFT_BINS];
      double diff = 0;

      synth(buf, REC, 111, fn_sine, NULL, 1e5, 16.0, 90.0, ZERO_POINT);
      fft_spectrum(buf, REC, 111, mag);
      fft_spectrum_decim(buf, REC, 111, 1, mag2);

      for (int i = 0; i < FFT_BINS; i++)
        diff += fabs(mag[i] - mag2[i]);

      check_near("identical", diff, 0, 0);
    }
  }

  // ============================= classifier =============================
  {
    SignalClass sc;
    double d;

    printf("classify:\n");

    synth(buf, SIZE, 55, fn_sine, NULL, 2000.0, 1000.0, 90.0, ZERO_POINT);
    measure_run(buf, SIZE, 55, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("sine -> SINE", sc.type, SIG_SINE, 0);

    d = 0.5;
    synth(buf, SIZE, 0, fn_square, &d, 3000.0, 1000.0, 80.0, ZERO_POINT);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("square -> SQUARE", sc.type, SIG_SQUARE, 0);

    d = 0.15;
    synth(buf, SIZE, 0, fn_square, &d, 3000.0, 1000.0, 80.0, ZERO_POINT);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("pulse 15%% -> PULSE", sc.type, SIG_PULSE, 0);

    synth(buf, SIZE, 0, fn_triangle, NULL, 500.0, 1000.0, 90.0, ZERO_POINT);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("triangle -> TRIANGLE", sc.type, SIG_TRIANGLE, 0);

    // Sawtooth: slow ramp up, fast drop
    for (int i = 0; i < SIZE; i++)
    {
      double ph = fmod(500.0 * i * 1e-6, 1.0);
      buf[i] = (uint8_t)(ZERO_POINT - 90 + (int)(180.0 * ph));
    }
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("sawtooth -> SAW", sc.type, SIG_SAW, 0);

    memset(buf, ZERO_POINT, SIZE);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("flat -> DC", sc.type, SIG_DC, 0);

    srand(7);
    for (int i = 0; i < SIZE; i++)
      buf[i] = (uint8_t)(ZERO_POINT - 40 + rand() % 80);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("noise -> NOISE", sc.type, SIG_NOISE, 0);

    // Square with glitch spikes (full-swing needles): must stay SQUARE with
    // the right frequency — a single needle used to trip the noise gate
    d = 0.5;
    synth(buf, SIZE, 0, fn_square, &d, 3000.0, 1000.0, 80.0, ZERO_POINT);
    srand(11);
    for (int g = 0; g < 6; g++)
    {
      int at = 500 + (rand() % (SIZE - 1000));
      for (int w = 0; w < 3; w++)
        buf[at + w] = (uint8_t)((buf[at + w] > ZERO_POINT) ? 5 : 250);
    }
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("square+spikes -> SQUARE", sc.type, SIG_SQUARE, 0);
    check_near("square+spikes freq", m.frequency, 3000, 1.0);

    // Square with noise: noise peaks inflate vmax/vmin but must not push
    // the RMS/amplitude ratio out of the square bucket
    d = 0.5;
    synth(buf, SIZE, 0, fn_square, &d, 3000.0, 1000.0, 80.0, ZERO_POINT);
    srand(12);
    for (int i = 0; i < SIZE; i++)
    {
      int v = buf[i] + (rand() % 25) - 12;
      buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
    }
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("square+noise -> SQUARE", sc.type, SIG_SQUARE, 0);
    check_near("square+noise freq", m.frequency, 3000, 1.0);
    check_near("square+noise duty", m.duty_x10, 500, 5);

    // Short 1 MHz burst in a long quiet record ("a spike on a slow
    // timebase"): percentiles erase it, the spike-trimmed peaks must not —
    // this is what the auto-setup steers by
    for (int i = 0; i < SIZE; i++)
    {
      if (i < 400) // 16 periods, 1.6% of the record: under the 2% percentile
        buf[i] = (uint8_t)(((i % 25) < 12) ? 208 : 48);
      else
        buf[i] = 48;
    }
    measure_run(buf, SIZE, 0, 40, ZERO_POINT, &m);
    check_near("burst pk_hi sees it", m.pk_hi >= 190, 1, 0);
    check_near("burst pk_lo", m.pk_lo <= 60, 1, 0);
    check_near("burst pctl top blind", m.top < 100, 1, 0); // documents why pk exists
    check_near("burst freq", m.frequency, 1000000, 2.0);

    // --- the counter where the sample grid is coarse -------------------
    // A 125 MS/s record (8 ns) of an FPGA square through a ~100 MHz analog
    // frontend, harmonics folding as they do on the bench. The crossing
    // intervals here split into two clusters one sample apart, and the
    // MEDIAN of them lands on the fuller one instead of between: 54 MHz used
    // to read 59.15, 40 MHz 41.08, 17 MHz 17.40. The mean of the intervals
    // that agree with the median has no such bias.
    {
      static const struct { double f; double tol; const char *name; } hf[] =
      {
        { 17e6, 0.5, "17 MHz square @ 8ns" },
        { 40e6, 0.5, "40 MHz square @ 8ns" },
        { 54e6, 1.0, "54 MHz square @ 8ns (2.31 s/p)" },
      };

      for (unsigned k = 0; k < sizeof(hf) / sizeof(hf[0]); k++)
      {
        for (int i = 0; i < SIZE; i++)
        {
          double t = i * 8e-9, s = 0.0;

          for (int h = 1; h <= 9; h += 2)
          {
            double fh = hf[k].f * h;
            double g = 1.0 / sqrt(1.0 + (fh / 100e6) * (fh / 100e6));

            s += g * sin(2 * M_PI * fh * t + 0.3) / h;
          }

          double v = ZERO_POINT + (4.0 / M_PI) * s * 100.0;

          buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : (int)lround(v));
        }

        measure_run(buf, SIZE, 0, 8, ZERO_POINT, &m);
        check_near(hf[k].name, m.frequency, hf[k].f, hf[k].tol);
      }
    }

    // --- a small signal wearing big needles -----------------------------
    // Eight samples out of the whole record set pk_lo/pk_hi, on purpose, and
    // a Schmitt band scaled to THOSE came out wider than the signal's own
    // swing: nothing crossed it and this read 0.15 MHz, with
    // period_good_pct still saying 100%.
    printf("17 MHz, 30 counts tall, under needles 8x its size:\n");
    for (int i = 0; i < SIZE; i++)
      buf[i] = (uint8_t)lround(ZERO_POINT + 15.0 * sin(2 * M_PI * 17e6 * i * 8e-9));

    for (int i = 400; i < SIZE; i += 400)
      buf[i] = ((i / 400) & 1) ? 228 : 28;

    measure_run(buf, SIZE, 0, 8, ZERO_POINT, &m);
    check_near("frequency survives the needles", m.frequency, 17e6, 1.0);

    // ...and the harder half of the same problem: needles on ONE side move a
    // peak without moving its partner, and the midpoint of that lopsided
    // pair can land clean outside the signal, which is a level nothing
    // crosses at all
    printf("...and with the needles on one side only:\n");
    for (int i = 0; i < SIZE; i++)
      buf[i] = (uint8_t)lround(ZERO_POINT + 20.0 * sin(2 * M_PI * 17e6 * i * 8e-9));

    for (int g = 0; g < 8; g++)
      buf[1000 + g * 2000] = 218;

    measure_run(buf, SIZE, 0, 8, ZERO_POINT, &m);
    check_near("mid level stays on the signal", m.frequency, 17e6, 1.0);

    // --- the half-rate trap ---------------------------------------------
    // The worst failure this counter had, because it was silent and the
    // frequency it lands on is the most common clock on earth. At exactly
    // 2.5 samples per period the Schmitt arming misses every SECOND
    // crossing, deterministically: 50.000 MHz read exactly 25.000 MHz with
    // periods=128, good=100% and a period of a round 5 samples. Nothing in
    // the record contradicted it. The second pass at a narrower band is what
    // finds the crossings that were there all along.
    printf("50.000 MHz at 125 MS/s - the half-rate trap:\n");
    for (int i = 0; i < SIZE; i++)
      buf[i] = (uint8_t)lround(ZERO_POINT + 100.0 * sin(2 * M_PI * 50e6 * i * 8e-9 + 0.3));

    measure_run(buf, SIZE, 0, 8, ZERO_POINT, &m);
    check_near("50.000 MHz, not half of it", m.frequency, 50e6, 1.0);

    // The bench signal that started this: a 54 MHz sine is what reaches the
    // ADC once the frontend has taken the harmonics off, and 2.31 samples
    // per period is under three. It read 58.5 MHz.
    printf("54.000 MHz sine - 2.31 samples per period:\n");
    for (int i = 0; i < SIZE; i++)
      buf[i] = (uint8_t)lround(ZERO_POINT + 100.0 * sin(2 * M_PI * 54e6 * i * 8e-9 + 0.3));

    measure_run(buf, SIZE, 0, 8, ZERO_POINT, &m);
    check_near("54 MHz sine, not 58.5", m.frequency, 54e6, 1.0);

    // Mains hum picked up by hand on a floating probe: a 50 Hz fundamental
    // buried in harmonics, HF junk and noise. This is the record the
    // auto-setup surveys (HS_5_ms => 24576 samples at 4.096 us = 100 ms),
    // and both the frequency AND its regularity flag have to survive.
    printf("dirty mains:\n");
    {
      unsigned seed = 12345;

      for (int i = 0; i < SIZE; i++)
      {
        double t = i * 4.096e-6;
        double v = ZERO_POINT
            + 60.0 * sin(2 * M_PI * 50.0 * t)          // the fundamental
            + 14.0 * sin(2 * M_PI * 150.0 * t + 1.1)   // 3rd harmonic
            +  7.0 * sin(2 * M_PI * 250.0 * t + 2.7)   // 5th harmonic
            +  5.0 * sin(2 * M_PI * 13000.0 * t);      // switching junk
        seed = seed * 1103515245u + 12345u;
        v += (double)((int)((seed >> 16) & 0xff) - 128) * 5.0 / 128.0;

        int c = (int)lround(v);
        buf[i] = (uint8_t)(c < 0 ? 0 : (c > 255 ? 255 : c));
      }

      measure_run(buf, SIZE, 0, 4096, ZERO_POINT, &m);
      check_near("mains freq", m.frequency, 50, 4.0);
      check_near("mains periods >= 3", m.periods >= 3, 1, 0);
      check_near("mains regular >= 60%", m.period_good_pct >= 60, 1, 0);
      check_near("mains swing", m.pk_hi - m.pk_lo, 160, 25.0);

      // The SAME signal seen on a fast timebase (HS_10_us => 32 ns per
      // sample = 786 us of record, 4% of one period). It must NOT look like
      // a valid fast signal, or the auto-setup would chase the junk instead
      // of the hum. This is the case that used to send it into millivolts.
      for (int i = 0; i < SIZE; i++)
      {
        double t = i * 32e-9;
        double v = ZERO_POINT
            + 60.0 * sin(2 * M_PI * 50.0 * t + 0.3)
            + 14.0 * sin(2 * M_PI * 150.0 * t + 1.1)
            +  7.0 * sin(2 * M_PI * 250.0 * t + 2.7)
            +  5.0 * sin(2 * M_PI * 13000.0 * t);
        seed = seed * 1103515245u + 12345u;
        v += (double)((int)((seed >> 16) & 0xff) - 128) * 5.0 / 128.0;

        int c = (int)lround(v);
        buf[i] = (uint8_t)(c < 0 ? 0 : (c > 255 ? 255 : c));
      }

      measure_run(buf, SIZE, 0, 32, ZERO_POINT, &m);
      printf("    (fragment reads f=%d Hz, %d periods, %d%% regular, swing %d)\n",
          m.frequency, m.periods, m.period_good_pct, m.pk_hi - m.pk_lo);
      check_near("fragment not a fast signal",
          !(m.frequency >= 50000 && m.periods >= 3 && m.period_good_pct >= 60), 1, 0);
    }

    // Real signals that do NOT repeat evenly: UART frames, bus traffic, an
    // SWD line. Their intervals are as irregular as noise, so the auto-setup
    // cannot use regularity to tell them apart — it uses level_pct, the
    // share of samples resting at a rail. This is the pair of records that
    // has to come out different, or auto-setup walks past logic signals.
    printf("digital vs noise (level_pct):\n");
    {
      unsigned seed = 999;
      int bit = 40;   // samples per bit
      int i = 0;

      // 8N1-ish traffic: idle high, frames of random data, random gaps
      memset(buf, 210, SIZE);

      while (i < SIZE - 12 * bit)
      {
        int byte = (int)((seed = seed * 1103515245u + 12345u) >> 16) & 0xff;
        int bits[10];

        bits[0] = 0;                      // start
        for (int b = 0; b < 8; b++)
          bits[1 + b] = (byte >> b) & 1;
        bits[9] = 1;                      // stop

        for (int b = 0; b < 10; b++)
        {
          for (int s = 0; s < bit; s++)
            buf[i++] = bits[b] ? 210 : 46;
        }

        // idle gap of 0..3 byte times
        int gap = (int)(((seed = seed * 1103515245u + 12345u) >> 16) % 4) * 10 * bit;

        for (int s = 0; s < gap && i < SIZE; s++)
          buf[i++] = 210;
      }

      measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
      printf("    (uart traffic: %d periods, %d%% regular, %d%% at level)\n",
          m.periods, m.period_good_pct, m.level_pct);
      check_near("traffic is digital", m.level_pct >= 65, 1, 0);
      check_near("traffic has a frequency", m.frequency > 0, 1, 0);

      // Same amplitude, no structure: this must stay rejected
      for (int k = 0; k < SIZE; k++)
      {
        int acc = 0;

        for (int j = 0; j < 4; j++) // sum of uniforms ~ gaussian
        {
          seed = seed * 1103515245u + 12345u;
          acc += (int)((seed >> 16) & 0xff);
        }

        int v = 128 + (acc / 4 - 128) * 82 / 100;
        buf[k] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
      }

      measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
      printf("    (noise: %d periods, %d%% regular, %d%% at level)\n",
          m.periods, m.period_good_pct, m.level_pct);
      check_near("noise is not digital", m.level_pct < 65, 1, 0);
      check_near("noise is not regular", m.period_good_pct < 60, 1, 0);
    }

    // THD: clean sine through the real FFT should be low, square high
    static float mag2[FFT_BINS];
    // 2 MHz sine at 16 ns -> bin = 2e6 * 512 * 16e-9 = 16.4
    synth(buf, SIZE, 0, fn_sine, NULL, 2.0e6, 16.0, 90.0, ZERO_POINT);
    fft_spectrum(buf, SIZE, 0, mag2);
    measure_run(buf, SIZE, 0, 16, ZERO_POINT, &m);
    classify_signal(&m, mag2, FFT_BINS, 16, &sc);
    check_near("sine type w/fft", sc.type, SIG_SINE, 0);
    if (sc.thd_x10 < 0 || sc.thd_x10 > 50)
    {
      printf("  FAIL sine THD low               got %d want <50 (=5%%)\n", sc.thd_x10);
      g_failures++;
    }
    else
      printf("  PASS sine THD low               got %.1f%%\n", sc.thd_x10 / 10.0);

    d = 0.5;
    synth(buf, SIZE, 0, fn_square, &d, 2.0e6, 16.0, 80.0, ZERO_POINT);
    fft_spectrum(buf, SIZE, 0, mag2);
    measure_run(buf, SIZE, 0, 16, ZERO_POINT, &m);
    classify_signal(&m, mag2, FFT_BINS, 16, &sc);
    // ideal square THD (h2..h5) = sqrt(1/9 + 1/25) ~= 38%
    if (sc.thd_x10 < 250)
    {
      printf("  FAIL square THD high            got %d want >250 (=25%%)\n", sc.thd_x10);
      g_failures++;
    }
    else
      printf("  PASS square THD high            got %.1f%%\n", sc.thd_x10 / 10.0);
  }

  // ============================ UART decoder ============================
  {
    static LogicScratch scratch;
    LogicResult lr;
    const char *msg = "Hello!";

    printf("uart:\n");

    // Build 8N1 stream: idle-high, start low, LSB first. 115200 baud
    // sampled at 1 MS/s (8.68 samples/bit)
    for (int pass = 0; pass < 2; pass++)
    {
      double baud = (pass == 0) ? 115200.0 : 9600.0;
      double sps_per_bit = 1e6 / baud;
      int pos = (int)(20 * sps_per_bit); // leading idle before the first frame

      memset(buf, 200, SIZE); // idle high at 200 counts

      for (const char *c = msg; *c; c++)
      {
        int frame_bits[10];

        frame_bits[0] = 0; // start
        for (int b = 0; b < 8; b++)
          frame_bits[1 + b] = (*c >> b) & 1;
        frame_bits[9] = 1; // stop

        for (int b = 0; b < 10; b++)
        {
          int end = (int)((pos / sps_per_bit + 1) * sps_per_bit + 0.5);
          int level = frame_bits[b] ? 200 : 56;

          for (; pos < end && pos < SIZE; pos++)
            buf[pos] = (uint8_t)level;
        }

        // inter-frame idle gap of 2 bit times
        int gap_end = pos + (int)(2 * sps_per_bit);
        for (; pos < gap_end && pos < SIZE; pos++)
          buf[pos] = 200;
      }

      // add noise +-4 counts
      srand(3);
      for (int i = 0; i < SIZE; i++)
      {
        int v = buf[i] + (rand() % 9) - 4;
        buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
      }

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);

      printf(" baud %d:\n", (int)baud);
      check_near("proto uart", lr.proto, PROTO_UART, 0);
      check_near("bytes decoded", n, (int)strlen(msg), 0);
      check_near("baud detected", lr.rate, baud, 0);
      check_near("positions ascend", lr.pos[n-1] > lr.pos[0], 1, 0);

      bool match = (n == (int)strlen(msg));
      for (int i = 0; i < n && match; i++)
        match = (lr.bytes[i] == (uint8_t)msg[i]);

      if (match)
        printf("  PASS payload                    \"%.*s\"\n", n, (char *)lr.bytes);
      else
      {
        printf("  FAIL payload mismatch: got \"%.*s\"\n", n, (char *)lr.bytes);
        g_failures++;
      }
    }

    // ----- a console line, back to back frames, at four sample rates -----
    //
    // The message a board under test actually sends. Two things about it
    // broke the first decoder: the frames follow each other with no gap, and
    // 45 bytes at 115200 is 3.9 ms, which does not fit a 24576-sample record
    // until 256 ns/sample. At 128 ns both ends of the record cut a run in
    // half, and a bit time taken from the shortest run - which is what the
    // first version did - reads one of those halves as a bit, gets the rate
    // wrong, fails every stop bit, and hands the record to the raw decoder
    // at a rate that changes with every acquisition.
    {
      const char *line = "UUUU MILKV-UART-TEST 115200-8N1 #0123456789\r\n";
      const int period[] = { 128, 256, 512, 2048 };
      int full = (int)strlen(line);

      printf("uart: console line, frames back to back:\n");

      for (unsigned p = 0; p < sizeof(period)/sizeof(period[0]); p++)
      {
        // How much of the message the record can hold at this sample rate
        double bit_ns = 1e9 / 115200.0;
        int fits = (int)(((double)SIZE * period[p] - 100000.0) / (bit_ns * 10));
        int want = (fits < full) ? fits : full;
        char name[40];

        synth_uart(buf, SIZE, period[p], 115200.0, line, 100000.0);

        int n = logic_decode(buf, SIZE, 0, period[p], PROTO_AUTO, &scratch, &lr);

        snprintf(name, sizeof(name), "%d ns: proto uart", period[p]);
        check_near(name, lr.proto, PROTO_UART, 0);
        snprintf(name, sizeof(name), "%d ns: baud", period[p]);
        check_near(name, lr.rate, 115200, 0);
        snprintf(name, sizeof(name), "%d ns: framing errors", period[p]);
        check_near(name, lr.errors, 0, 0);
        snprintf(name, sizeof(name), "%d ns: bytes (fits %d)", period[p], want);
        check_near(name, n, want, 0);
        snprintf(name, sizeof(name), "%d ns: at message start", period[p]);
        check_near(name, lr.burst_start, 1, 0);

        bool match = (n > 0);

        for (int i = 0; i < n && match; i++)
          match = (lr.bytes[i] == (uint8_t)line[i]);

        snprintf(name, sizeof(name), "%d ns: payload prefix", period[p]);
        check_near(name, match, 1, 0);
      }

      // A record that opens in the middle of a message decodes just the same,
      // but it is not the head of one - which is what "stop at message start"
      // waits for, and the only thing that tells the two apart
      printf("uart: record opening mid-message:\n");
      synth_uart(buf, SIZE, 512.0, 115200.0, line, -1000000.0);
      check_near("bytes decoded", logic_decode(buf, SIZE, 0, 512, PROTO_AUTO,
          &scratch, &lr) > 0, 1, 0);
      check_near("baud", lr.rate, 115200, 0);
      check_near("not at message start", lr.burst_start, 0, 0);

      // More traffic than a result can hold, flagged rather than silently cut
      printf("uart: more bytes than fit a result:\n");
      {
        char big[LOGIC_MAX_BYTES + 20];

        for (unsigned i = 0; i < sizeof(big) - 1; i++)
          big[i] = (char)('a' + i % 26);

        big[sizeof(big) - 1] = 0;

        synth_uart(buf, SIZE, 512.0, 115200.0, big, 100000.0);
        check_near("bytes", logic_decode(buf, SIZE, 0, 512, PROTO_AUTO,
            &scratch, &lr), LOGIC_MAX_BYTES, 0);
        check_near("truncated flag", lr.truncated, 1, 0);
      }

      // What a fast timebase gives: 32 ns/sample is a 0.79 ms record, so a
      // 3.9 ms message comes out as its first few bytes. That is not a decode
      // failure and must not read as one - the frames are right, the record
      // is short, and overrun is what says so.
      printf("uart: fast timebase, message cut off:\n");
      synth_uart(buf, SIZE, 32.0, 115200.0, line, 100000.0);
      check_near("decodes what fits", logic_decode(buf, SIZE, 0, 32, PROTO_AUTO,
          &scratch, &lr) >= 4, 1, 0);
      check_near("baud", lr.rate, 115200, 0);
      check_near("errors", lr.errors, 0, 0);
      check_near("at message start", lr.burst_start, 1, 0);
      check_near("overrun flagged", lr.overrun, 1, 0);
      check_near("prefix is UUUU", memcmp(lr.bytes, "UUUU", 4), 0, 0);

      // Deeper into the same preamble there is no idle left in the record,
      // and 0x55 back to back IS a square wave - nothing in those samples
      // says which. Reported, not decided: the caller settles it.
      printf("uart: inside the 0x55 preamble:\n");
      synth_uart(buf, SIZE, 32.0, 115200.0, "UUUUUUUUUUUU", -200000.0);
      uart_decode(buf, SIZE, 0, 32, &scratch, &lr);
      check_near("ambiguous", lr.ambiguous, 1, 0);
      check_near("auto keeps off it", logic_decode(buf, SIZE, 0, 32, PROTO_AUTO,
          &scratch, &lr) > 0 && lr.proto == PROTO_UART, 0, 0);
      check_near("named, it decodes", logic_decode(buf, SIZE, 0, 32, PROTO_UART,
          &scratch, &lr) > 0, 1, 0);

      // Being told the rate overrides the search; being told the wrong one
      // decodes nothing rather than something plausible
      printf("uart: forced baud:\n");
      synth_uart(buf, SIZE, 512.0, 115200.0, line, 100000.0);

      uart_decode_set_baud(115200);
      check_near("forced right: bytes", uart_decode(buf, SIZE, 0, 512,
          &scratch, &lr), full, 0);
      check_near("forced right: baud", lr.rate, 115200, 0);

      uart_decode_set_baud(9600);
      check_near("forced wrong: nothing", uart_decode(buf, SIZE, 0, 512,
          &scratch, &lr), 0, 0);

      uart_decode_set_baud(0);
    }

    // ----- 1-Wire: reset + presence + 0xCC 0x44, 1 us sample period -----
    printf("1-wire:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0x44);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      check_near("proto 1-wire", lr.proto, PROTO_ONEWIRE, 0);
      check_near("bytes", n, 2, 0);
      check_near("byte0 = CC", lr.bytes[0], 0xCC, 0);
      check_near("byte1 = 44", lr.bytes[1], 0x44, 0);

      // SKIP ROM + CONVERT T names no device - the family code was never
      // sent - but the commands are a thermometer's, and saying "DS18x20?"
      // is honest where saying nothing throws away what the bus did show
      const OwAnalysis *a = onewire_analysis();
      check_near("SKIP is a ROM cmd", a->role[0], OW_R_ROMCMD, 0);
      check_near("CONVERT is a fn cmd", a->role[1], OW_R_FNCMD, 0);
      check_near("guessed a thermometer", a->thermometer, 1, 0);
      check_near("but not certain", a->sure, 0, 0);
      check_near("no ROM seen", a->rom_seen, 0, 0);
    }

    // ----- 1-Wire: the DS18B20 read cycle, and telling it from a key -----
    // Same bus, same bit slots, same decoder. What separates a thermometer
    // from the iButton in a door intercom is the transaction: a family code
    // under a CRC, and a scratchpad with a temperature in it.
    printf("1-wire ds18b20:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      // +25.0625 C at 12 bits: 401 sixteenths
      uint8_t sp[9] = { 0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0 };

      sp[8] = ow_crc(sp, 8);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xCC);          // SKIP ROM
      ow_byte(&g, 0x44);          // CONVERT T
      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xBE);          // READ SCRATCHPAD
      ow_bytes(&g, sp, 9);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("bytes", n, 13, 0);
      check_near("scratchpad CRC ok", a->sp_valid, 1, 0);
      check_near("named without a ROM", a->device &&
          !strcmp(a->device, "DS18B20"), 1, 0);
      // ...but named from behaviour, not from a family code: DS1822 and
      // DS1825 answer byte for byte the same way, so the name wears a '?'
      check_near("not from a ROM", a->sure, 0, 0);
      check_near("temp 25.0625 C", a->temp_mc, 25062, 0);
      check_near("12-bit conversion", a->bits, 12, 0);
      // Bytes 0..1 are the convert cycle, 2..3 the SKIP+READ SP that opens
      // the second one, so the scratchpad starts at 4
      check_near("temp byte roles", a->role[4] == OW_R_TEMPL &&
          a->role[5] == OW_R_TEMPH, 1, 0);
      check_near("header", !strcmp(lr.info, "DS18B20? +25.06C 12b"), 1, 0);

      // The reading is written on the trace at the byte that completes it
      char lab[16];
      onewire_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("label +25.06C", !strcmp(lab, "+25.06C"), 1, 0);
      onewire_byte_label(a, 8, lr.bytes[8], lab, sizeof(lab));
      check_near("label 12bit", !strcmp(lab, "12bit"), 1, 0);

      // The three reserved bytes hold their place with a name rather than a
      // pair of dots: a byte with nothing written under it reads as a byte
      // the decoder could not account for
      onewire_byte_label(a, 9, lr.bytes[9], lab, sizeof(lab));
      check_near("label RSV (FF)", !strcmp(lab, "RSV"), 1, 0);
      onewire_byte_label(a, 10, lr.bytes[10], lab, sizeof(lab));
      check_near("label RSV (0C)", !strcmp(lab, "RSV"), 1, 0);
      onewire_byte_label(a, 11, lr.bytes[11], lab, sizeof(lab));
      check_near("label RSV (10)", !strcmp(lab, "RSV"), 1, 0);
      onewire_byte_label(a, 12, lr.bytes[12], lab, sizeof(lab));
      check_near("label CRC", !strcmp(lab, "CRC"), 1, 0);

      // The two temperature bytes are one reading and are shown as one, but
      // each is still a byte with a place in the scratchpad
      int gs, gl;

      onewire_group_at(a, 4, &gs, &gl);
      check_near("temperature groups", gs == 4 && gl == 2, 1, 0);
      onewire_group_at(a, 8, &gs, &gl);
      check_near("the config byte does not", gs == 8 && gl == 1, 1, 0);
      onewire_field_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("field T-LO", !strcmp(lab, "T-LO"), 1, 0);
      onewire_field_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("field T-HI, not the reading", !strcmp(lab, "T-HI"), 1, 0);
    }

    // A reserved byte is still a byte with a value a working part returns.
    // The CRC says the read was clean, so one that comes back different is
    // the part differing - which is what a counterfeit does.
    printf("1-wire scratchpad with odd reserved bytes:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t sp[9] = { 0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x00, 0 };

      sp[8] = ow_crc(sp, 8);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xBE);
      ow_bytes(&g, sp, 9);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();
      char lab[16];

      check_near("still decodes", a->sp_valid, 1, 0);
      check_near("still a DS18B20", a->device &&
          !strcmp(a->device, "DS18B20"), 1, 0);
      onewire_byte_label(a, 9, lr.bytes[9], lab, sizeof(lab));
      check_near("byte 7 flagged", !strcmp(lab, "RSV!"), 1, 0);
    }

    printf("1-wire ds18b20 below zero:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t sp[9] = { 0x5E, 0xFF, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0 };

      sp[8] = ow_crc(sp, 8);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xBE);
      ow_bytes(&g, sp, 9);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("temp -10.125 C", a->temp_mc, -10125, 0);
      check_near("header", !strcmp(lr.info, "DS18B20? -10.12C 12b"), 1, 0);
    }

    printf("1-wire ds18b20 at 9 bits:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      // Same 401 sixteenths, but at 9 bits the low three are undefined and
      // the sensor only ever meant the 400 above them
      uint8_t sp[9] = { 0x91, 0x01, 0x4B, 0x46, 0x1F, 0xFF, 0x0C, 0x10, 0 };

      sp[8] = ow_crc(sp, 8);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xBE);
      ow_bytes(&g, sp, 9);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("9-bit conversion", a->bits, 9, 0);
      check_near("undefined bits dropped", a->temp_mc, 25000, 0);
    }

    // The older part is told apart by byte 4: a DS18B20 keeps its
    // configuration register there and its undefined bits read as ones, so
    // it is 1F, 3F, 5F or 7F. A DS18S20 has nothing there and reads FF.
    printf("1-wire ds18s20:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      // 25.0 C: half-degree count of 50, and the count registers that turn
      // it into sixteenths - COUNT REMAIN 0x0C out of COUNT PER C 0x10
      uint8_t sp[9] = { 0x32, 0x00, 0x4B, 0x46, 0xFF, 0xFF, 0x0C, 0x10, 0 };

      sp[8] = ow_crc(sp, 8);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xBE);
      ow_bytes(&g, sp, 9);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();
      char lab[16];

      check_near("DS18S20 by scratchpad", a->device &&
          !strcmp(a->device, "DS18S20"), 1, 0);
      check_near("marked as the old part", a->s20, 1, 0);
      check_near("no config register", a->bits, 0, 0);
      check_near("extended resolution", a->ext_res, 1, 0);
      check_near("25.0 C", a->temp_mc, 25000, 0);
      check_near("header", !strcmp(lr.info, "DS18S20? +25.00C ext"), 1, 0);

      // Byte 4 is reserved here, and bytes 6 and 7 are the count registers
      // rather than reserved - the labels have to follow the part
      onewire_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
      check_near("byte 4 is RSV", !strcmp(lab, "RSV"), 1, 0);
      onewire_byte_label(a, 8, lr.bytes[8], lab, sizeof(lab));
      check_near("byte 6 is CNTR", !strcmp(lab, "CNTR"), 1, 0);
      onewire_byte_label(a, 9, lr.bytes[9], lab, sizeof(lab));
      check_near("byte 7 is CNT/C", !strcmp(lab, "CNT/C"), 1, 0);
    }

    // The same arithmetic below zero, where dropping the half-degree bit is
    // a floor and not a truncation: -0.5 C is count -1, and -1 >> 1 is -1
    printf("1-wire ds18s20 below zero:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t sp[9] = { 0xFF, 0xFF, 0x4B, 0x46, 0xFF, 0xFF, 0x04, 0x10, 0 };

      sp[8] = ow_crc(sp, 8);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xBE);
      ow_bytes(&g, sp, 9);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("-0.5 C", a->temp_mc, -500, 0);
      check_near("header", !strcmp(lr.info, "DS18S20? -0.50C ext"), 1, 0);
    }

    // A byte 4 that is neither a configuration register nor FF names no
    // part: "not a DS18B20" is not the same statement as "a DS18S20"
    printf("1-wire scratchpad from something else:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t sp[9] = { 0x91, 0x01, 0x4B, 0x46, 0x33, 0xFF, 0x0C, 0x10, 0 };

      sp[8] = ow_crc(sp, 8);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xBE);
      ow_bytes(&g, sp, 9);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("class only", a->device &&
          !strcmp(a->device, "DS18x20"), 1, 0);
      check_near("not called the old part", a->s20, 0, 0);
    }

    printf("1-wire ibutton key:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t rom[8] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0 };

      rom[7] = ow_crc(rom, 7);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0x33);          // READ ROM - all a DS1990A can do
      ow_bytes(&g, rom, 8);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("bytes", n, 9, 0);
      check_near("ROM CRC ok", a->rom_valid, 1, 0);
      check_near("family 01", a->family, 0x01, 0);
      check_near("named a key", a->device &&
          !strcmp(a->device, "DS1990 key"), 1, 0);
      check_near("not a thermometer", a->thermometer, 0, 0);
      check_near("no temperature", a->temp_valid, 0, 0);
      // The whole conversation a DS1990 can have is this one: reset, READ
      // ROM, eight bytes. There is nothing missing from the record, so what
      // it holds is an EVENT - a key touched to a reader - and the header
      // says that and prints the number written on the key, six serial bytes
      // most significant first, the way a reader prints it.
      check_near("read as a touch", a->ibutton, 1, 0);
      check_near("header",
          !strcmp(lr.info, "iButton CDAB89674523"), 1, 0);

      char lab[16];
      onewire_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("family byte labelled", !strcmp(lab, "DS1990 key"), 1, 0);
      onewire_byte_label(a, 8, lr.bytes[8], lab, sizeof(lab));
      check_near("CRC byte labelled", !strcmp(lab, "CRC"), 1, 0);
    }

    // 0x81 is a DS1420 - a ROM and nothing else, functionally a DS1990 sold
    // as an identity token. Caught on the bench as "fam 81", which is the
    // decoder reporting its own table rather than the bus.
    printf("1-wire a serial ID button:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t rom[8] = { 0x81, 0x0A, 0x1B, 0x2C, 0x3D, 0x4E, 0x5F, 0 };

      rom[7] = ow_crc(rom, 7);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0x33);
      ow_bytes(&g, rom, 8);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();
      char lab[16];

      check_near("named", a->device && !strcmp(a->device, "DS1420 ID"), 1, 0);
      check_near("read as a touch", a->ibutton, 1, 0);
      check_near("header", !strcmp(lr.info, "iButton 5F4E3D2C1B0A"), 1, 0);
      onewire_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("family byte", !strcmp(lab, "DS1420 ID"), 1, 0);
    }

    // ...and a family this decoder has no name for at all. The transaction is
    // still one complete thing - an identity was asked for and given - so the
    // identity is what to show, rather than how much the table knows.
    printf("1-wire an unknown family, ROM only:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t rom[8] = { 0x77, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0 };

      rom[7] = ow_crc(rom, 7);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0x33);
      ow_bytes(&g, rom, 8);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("ROM checks out", a->rom_valid, 1, 0);
      check_near("no name for it", a->device == NULL, 1, 0);
      check_near("...but the identity is the answer", a->id_only, 1, 0);
      check_near("header",
          !strcmp(lr.info, "1-Wire 77 665544332211"), 1, 0);
    }

    // Every family code in the table has to name something, and the table is
    // indexed linearly - so a duplicate or a stray zero would be a silent
    // shadow. Sweeping the whole byte range is cheap and catches both.
    printf("1-wire family table:\n");
    {
      int named = 0, thermometers = 0, keys = 0;

      for (int code = 0; code < 256; code++)
      {
        OwGen g = { buf, SIZE, 0 };
        uint8_t rom[8] = { (uint8_t)code, 0x11, 0x22, 0x33, 0x44, 0x55,
            0x66, 0 };

        rom[7] = ow_crc(rom, 7);
        memset(buf, 200, SIZE);

        ow_reset(&g);
        ow_byte(&g, 0x33);
        ow_bytes(&g, rom, 8);

        if (onewire_decode(buf, SIZE, 0, 1000, &scratch, &lr) < 9)
          continue;

        const OwAnalysis *a = onewire_analysis();

        if (!a->rom_valid || a->family != (uint8_t)code)
          continue;

        if (a->device)
          named++;

        if (a->thermometer)
          thermometers++;

        if (a->ibutton)
          keys++;

        // Named or not, an identity read always yields the identity
        if (!a->device && !a->id_only)
          named = -1000;              // an unnamed family that said nothing
      }

      check_near("families named", named, 55, 0);
      // 10 18S20, 21 1921, 22 1822, 28 18B20, 3B 1825, 41 1922, 42 28EA00
      check_near("...of which thermometers", thermometers, 7, 0);
      // 01 DS1990 and 81 DS1420: the two that are a ROM and nothing else
      check_near("...and read as a touch", keys, 2, 0);
    }

    // The MagSafe chip, and the reason it is worth telling the switches
    // apart: 0xF5 reads two pins on a DS2413, eight channels on a DS2408 and
    // memory on a DS2406. Naming it from the family is the only way that
    // byte gets the right word under it.
    printf("1-wire DS2413 switch:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t rom[8] = { 0x3A, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0 };
      // PIOA reading low and PIOB reading high, with the top nibble the
      // complement of the bottom - which is what makes the byte a PIO answer
      // rather than a bus nobody is driving. Bit 0 is A's pin and bit 2 is
      // B's; the odd bits are the output latches.
      uint8_t pio = 0x0E | (uint8_t)((~0x0E & 0x0F) << 4);

      rom[7] = ow_crc(rom, 7);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0x33);
      ow_bytes(&g, rom, 8);
      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xF5);          // PIO ACCESS READ
      ow_byte(&g, pio);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();
      char lab[16];

      check_near("family named", a->device &&
          !strcmp(a->device, "DS2413 sw"), 1, 0);
      check_near("not a touch: it was told to do something", a->ibutton, 0, 0);
      check_near("the pins were read", a->pio_valid, 1, 0);
      check_near("header", !strcmp(lr.info, "DS2413 sw A=0 B=1"), 1, 0);

      onewire_byte_label(a, 10, lr.bytes[10], lab, sizeof(lab));
      check_near("command named for THIS family",
          !strcmp(lab, "PIO READ"), 1, 0);
      onewire_byte_label(a, 11, lr.bytes[11], lab, sizeof(lab));
      check_near("and the pins read off", !strcmp(lab, "A=0 B=1"), 1, 0);
    }

    // ...and the same opcode on a part that is not a DS2413 gets that part's
    // name, not this one's
    printf("1-wire the same opcode on another family:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t rom[8] = { 0x29, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0 };

      rom[7] = ow_crc(rom, 7);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0x33);
      ow_bytes(&g, rom, 8);
      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xF5);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();
      char lab[16];

      check_near("a DS2408", a->device &&
          !strcmp(a->device, "DS2408 sw"), 1, 0);
      onewire_byte_label(a, 10, lr.bytes[10], lab, sizeof(lab));
      check_near("0xF5 is channel access here",
          !strcmp(lab, "CH READ"), 1, 0);
      check_near("and no pins were invented", a->pio_valid, 0, 0);
    }

    // With no ROM on the wire there is no family, so a function byte is named
    // from the only reading worth offering - and says it is a guess
    printf("1-wire a function byte with no ROM behind it:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      char lab[16];

      memset(buf, 200, SIZE);
      ow_reset(&g);
      ow_byte(&g, 0xCC);          // SKIP ROM: nobody said what this is
      ow_byte(&g, 0x44);          // CONVERT T - probably

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      onewire_byte_label(onewire_analysis(), 1, lr.bytes[1], lab, sizeof(lab));
      check_near("named, with a question mark",
          !strcmp(lab, "CONVERT?"), 1, 0);
    }

    printf("1-wire ds18b20 by ROM:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t rom[8] = { 0x28, 0xFF, 0x64, 0x1E, 0x0C, 0x27, 0x9A, 0 };

      rom[7] = ow_crc(rom, 7);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0x33);
      ow_bytes(&g, rom, 8);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("family 28", a->family, 0x28, 0);
      check_near("DS18B20", a->device && !strcmp(a->device, "DS18B20"), 1, 0);
      check_near("thermometer", a->thermometer, 1, 0);
      check_near("header", !strcmp(lr.info, "DS18B20 ROM ok"), 1, 0);
    }

    printf("1-wire bad ROM CRC:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t rom[8] = { 0x28, 0xFF, 0x64, 0x1E, 0x0C, 0x27, 0x9A, 0 };

      rom[7] = (uint8_t)(ow_crc(rom, 7) ^ 0x55);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0x33);
      ow_bytes(&g, rom, 8);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      // Eight bytes that do not check out are not a device to name: a wrong
      // family code is exactly what a misread bit looks like
      check_near("ROM seen", a->rom_seen, 1, 0);
      check_near("CRC refused", a->rom_valid, 0, 0);
      check_near("nothing named", a->device == NULL, 1, 0);
      check_near("header", !strcmp(lr.info, "1-Wire ROM CRC bad"), 1, 0);

      char lab[16];
      onewire_byte_label(a, 8, lr.bytes[8], lab, sizeof(lab));
      check_near("CRC byte flagged", !strcmp(lab, "CRC!"), 1, 0);
    }

    printf("1-wire search rom:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      uint8_t rom[8] = { 0x28, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0 };

      rom[7] = ow_crc(rom, 7);
      memset(buf, 200, SIZE);

      ow_reset(&g);
      ow_byte(&g, 0xF0);          // SEARCH ROM

      // Three slots per ROM bit: the device sends the bit, then its
      // complement, then the master writes the branch it takes
      for (int j = 0; j < 64; j++)
      {
        int bit = (rom[j / 8] >> (j % 8)) & 1;

        ow_bit(&g, bit);
        ow_bit(&g, !bit);
        ow_bit(&g, bit);
      }

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("ROM out of the triplets", a->rom_valid, 1, 0);
      check_near("family 28", a->family, 0x28, 0);
      check_near("serial", a->rom[3], 0x33, 0);
      check_near("search labelled once", a->role[1] == OW_R_SEARCH &&
          a->role[2] == OW_R_NONE, 1, 0);
    }

    printf("1-wire aborted scratchpad read:\n");
    {
      OwGen g = { buf, SIZE, 0 };
      memset(buf, 200, SIZE);

      // Plenty of libraries read the two temperature bytes and reset: there
      // is no CRC to check, so the reading is believed only because it lands
      // in the range the part can report, and the naming stays a guess
      ow_reset(&g);
      ow_byte(&g, 0xCC);
      ow_byte(&g, 0xBE);
      ow_byte(&g, 0x91);
      ow_byte(&g, 0x01);
      ow_reset(&g);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const OwAnalysis *a = onewire_analysis();

      check_near("temp 25.0625 C", a->temp_mc, 25062, 0);
      check_near("no scratchpad CRC", a->sp_valid, 0, 0);
      check_near("named as a guess", a->device &&
          !strcmp(a->device, "DS18x20") && !a->sure, 1, 0);
    }

    // ----- WS2812: 2 LEDs GRB, 40 ns sample period -----
    printf("ws2812:\n");
    {
      double t = 60000; /* reset gap first */
      memset(buf, 56, SIZE); /* idle low */

      #define EMITNS(level, ns) do { \
        double e = t + (ns); \
        for (; t < e; t += 40.0) { int p = (int)(t / 40.0); \
          if (p < SIZE) buf[p] = (level) ? 200 : 56; } \
      } while (0)

      uint8_t ws_bytes[6] = { 0x11, 0x22, 0x33, 0xAA, 0x55, 0xFF };
      for (int by = 0; by < 6; by++)
      {
        for (int b = 7; b >= 0; b--)
        {
          int bit = (ws_bytes[by] >> b) & 1;
          EMITNS(1, bit ? 800 : 400);
          EMITNS(0, bit ? 450 : 850);
        }
      }
      #undef EMITNS

      int n = logic_decode(buf, SIZE, 0, 40, PROTO_AUTO, &scratch, &lr);
      check_near("proto ws2812", lr.proto, PROTO_WS2812, 0);
      check_near("bytes", n, 6, 0);
      bool ok = (n == 6);
      for (int i = 0; i < n && ok; i++)
        ok = (lr.bytes[i] == ws_bytes[i]);
      check_near("payload match", ok, 1, 0);

      // Three bytes are one pixel, and the colour is the thing the hex dump
      // cannot show: the wire order is G-R-B, so 11 22 33 is #221133
      const WsAnalysis *wa = ws2812_analysis();
      char lab[16];
      int gs, gl;

      check_near("pixels", wa->pixels, 2, 0);
      check_near("frames", wa->frames, 1, 0);
      check_near("no partial pixel", wa->partial, 0, 0);

      ws2812_group_at(wa, 1, &gs, &gl);
      check_near("group start", gs, 0, 0);
      check_near("group len", gl, 3, 0);

      ws2812_group_at(wa, 5, &gs, &gl);
      check_near("second group start", gs, 3, 0);

      ws2812_byte_label(wa, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("pixel 0 colour", strcmp(lab, "#221133"), 0, 0);

      ws2812_byte_label(wa, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("pixel 1 colour", strcmp(lab, "#55AAFF"), 0, 0);

      ws2812_field_label(wa, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("field G1", strcmp(lab, "G1"), 0, 0);

      ws2812_field_label(wa, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("field B1", strcmp(lab, "B1"), 0, 0);
    }

    // ----- a 3 Mbaud console is not a strip, and the rest level says so -----
    // The block above and this one are the same pulse widths read two ways.
    // A strip's marks are 400 and 800 ns inside a 1250 ns cell; a 3 Mbaud
    // UART's runs are 333, 666 and 1000 - two or three clusters in the same
    // place, so the width histogram genuinely cannot tell them apart and no
    // amount of care with it ever will.
    //
    // What tells them apart is where the line waits. A UART rests HIGH
    // between messages and a strip rests LOW - its frame is latched by a long
    // low and every high it sends is one bit's mark, bounded by the cell. So
    // a mark that outlasts a cell is not a mark, and the strip decoder is
    // gone before it has counted a single pulse.
    printf("uart 3M is not ws2812:\n");
    {
      const char *msg = "FAST 3M BAUD\r\n";
      const double bit_ns = 1000.0 / 3.0;  // 333.33 ns, no divider error
      double t = 5000;                     // some idle in front of the message

      memset(buf, 200, SIZE);              // ...and the line rests high

      #define EMITBIT(level, ns) do { \
        double e = t + (ns); \
        for (; t < e; t += 20.0) { int p = (int)(t / 20.0); \
          if (p < SIZE) buf[p] = (level) ? 200 : 56; } \
      } while (0)

      for (const char *c = msg; *c; c++)
      {
        EMITBIT(0, bit_ns);                        // start
        for (int b = 0; b < 8; b++)
          EMITBIT((*c >> b) & 1, bit_ns);          // data, LSB first
        EMITBIT(1, bit_ns);                        // stop
      }
      #undef EMITBIT

      int n = logic_decode(buf, SIZE, 0, 20, PROTO_AUTO, &scratch, &lr);

      check_near("proto uart", lr.proto, PROTO_UART, 0);
      check_near("bytes", n, 14, 0);
      check_near("payload match",
          n == 14 && !memcmp(lr.bytes, msg, 14), 1, 0);
      check_near("3 Mbaud", lr.rate, 3000000, 1.0);
      check_near("idle is high", lr.idle_high, 1, 0);

      // Not "the strip decoder lost the argument" - it never entered one. A
      // 5 ms mark is not a bit at any rate it reads, and that is one
      // comparison per run, ahead of every histogram it would have built.
      memset(&scratch, 0, sizeof(scratch));
      check_near("uart is not ws2812",
          ws2812_decode(buf, SIZE, 0, 20, &scratch, &lr), 0, 0);
    }

    // ----- NEC: addr 0x04 cmd 0x08, 10 us sample period -----
    printf("nec:\n");
    {
      int pos = 0;
      memset(buf, 200, SIZE);

      #define EMITUS(level, us) do { \
        int e = pos + (us) / 10; \
        for (; pos < e && pos < SIZE; pos++) buf[pos] = (level) ? 200 : 56; \
      } while (0)

      EMITUS(1, 5000);
      EMITUS(0, 9000);   /* lead mark */
      EMITUS(1, 4500);   /* lead space */

      uint32_t word = 0x04 | (0xFBu << 8) | (0x08u << 16) | (0xF7u << 24);
      for (int b = 0; b < 32; b++)
      {
        EMITUS(0, 560);
        EMITUS(1, (word >> b) & 1 ? 1690 : 560);
      }
      EMITUS(0, 560);    /* stop mark */
      #undef EMITUS

      int n = logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);
      check_near("proto nec", lr.proto, PROTO_NEC, 0);
      check_near("bytes", n, 4, 0);
      check_near("addr", lr.bytes[0], 0x04, 0);
      check_near("cmd", lr.bytes[2], 0x08, 0);
      check_near("no errors", lr.errors, 0, 0);
      check_near("whole frame, not cut", lr.overrun, 0, 0);

      // A NEC frame is 67.5 ms end to end and a record usually is not, so
      // the frame that arrives cut off is the normal case, not the odd one.
      // The lead plus whole bytes is still a decode; the rest is reported
      // missing rather than thrown away.
      //
      // And whatever happens, this must not come out as UART. NEC's 560 us
      // element IS a valid bit time - 1795 baud - and the bytes it frames
      // decode into something. What says it is not UART is the 9 ms lead:
      // 8N1 holds the line down for nine bit times at the very most, so a
      // mark sixteen long is proof of another protocol, and the scope used
      // to report "UART 1795 8N1" for a remote control because of it.
      printf("nec: frame cut off by the end of the record:\n");

      // The record ending is the record being SHORT, not the line going
      // quiet: the decoders are handed the first `cut` samples and nothing
      // after them, which is what a timebase too fast for a 67.5 ms frame
      // actually produces.
      for (int keep_bits = 8; keep_bits <= 24; keep_bits += 8)
      {
        int cut = (5000 + 9000 + 4500) / 10;
        char name[44];

        for (int b = 0; b < keep_bits; b++)
          cut += (560 + (((word >> b) & 1) ? 1690 : 560)) / 10;

        int cn = logic_decode(buf, cut, 0, 10000, PROTO_AUTO, &scratch, &lr);

        snprintf(name, sizeof(name), "%d bits: proto nec", keep_bits);
        check_near(name, lr.proto, PROTO_NEC, 0);
        snprintf(name, sizeof(name), "%d bits: bytes", keep_bits);
        check_near(name, cn, keep_bits / 8, 0);
        snprintf(name, sizeof(name), "%d bits: addr", keep_bits);
        check_near(name, lr.bytes[0], 0x04, 0);
        snprintf(name, sizeof(name), "%d bits: flagged cut", keep_bits);
        check_near(name, lr.overrun, 1, 0);

        // The same record through the UART decoder: it finds frames, because
        // 560 us is a real bit time and the bits do frame - and marks them
        // ambiguous, because the 9 ms lead cannot happen in 8N1
        uart_decode(buf, cut, 0, 10000, &scratch, &lr);
        snprintf(name, sizeof(name), "%d bits: uart ambiguous", keep_bits);
        check_near(name, lr.ambiguous, 1, 0);
      }
    }

    // ----- RAW fallback: 100 kHz square looks like a clock -----
    printf("raw fallback:\n");
    {
      double dd = 0.5;
      synth(buf, SIZE, 0, fn_square, &dd, 100000.0, 1000.0, 80.0, ZERO_POINT);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      check_near("proto raw", lr.proto, PROTO_RAW, 0);
      check_near("bytes > 0", n > 0, 1, 0);
      check_near("pattern AA/55", lr.bytes[0] == 0xAA || lr.bytes[0] == 0x55, 1, 0);
    }

    // ----- Servo / RC PWM: 1.5 ms in a 20 ms frame -----
    //
    // The signal the UART decoder used to claim: a 1.5 ms pulse frames bytes
    // at 667 baud as well as any other pulse does. What says servo is the
    // CADENCE - a fixed frame rate whether the value moves or not - and that
    // is what this checks, including that a train whose rate wanders is
    // refused rather than averaged into something plausible.
    printf("servo pwm:\n");
    {
      // 10 us per sample: a 20 ms frame is 2000 samples, so a 24576 sample
      // record holds twelve of them
      const int width_us[] = { 1500, 1500, 1000, 2000, 1500 };
      const int lead = 500; // idle ahead of the first pulse, so that pulse is
                            // not the run the start of the record cut through
      int pos = 0;

      memset(buf, 56, SIZE); // idle low

      for (unsigned f = 0; f < sizeof(width_us)/sizeof(width_us[0]); f++)
      {
        int start = lead + (int)f * 2000; // frames on the 20 ms grid

        for (int i = 0; i < width_us[f] / 10 && start + i < SIZE; i++)
          buf[start + i] = 200;
      }

      int n = logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);

      check_near("proto servo", lr.proto, PROTO_SERVO, 0);
      check_near("pulses", n, 5, 0);
      check_near("first 1.50 ms", lr.bytes[0], 150, 0);
      check_near("third 1.00 ms", lr.bytes[2], 100, 0);
      check_near("fourth 2.00 ms", lr.bytes[3], 200, 0);
      check_near("frame 20 ms", lr.rate, 20000000, 0);
      check_near("idle low", lr.idle_high, 0, 0);
      check_near("pulse marked at its start", lr.pos[0], lead, 0);
      // The frame is what gets marked out on the trace, pulse to pulse, so
      // the label has room to sit in; the pulse itself is on the waveform
      check_near("frame marked pulse to pulse", lr.end[0] - lr.pos[0], 2000, 0);

      // A pulse the start of the record cut through is a fact about the
      // buffer, not about the signal: its width is whatever survived
      pos = 0;

      for (; pos < 80; pos++)
        buf[pos] = 200; // the tail of a pulse that began before the record

      check_near("cut pulse not counted",
          logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr), 5, 0);
      check_near("...and the rest still fine", lr.bytes[0], 150, 0);

      // The angle is the width in the units a servo is thought about in:
      // 1000 us is -90 degrees, 2000 is +90, 1500 is straight ahead
      servo_g(buf, SIZE, 10000, 1500, 20000, 8);
      logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);
      check_near("centred", !strcmp(lr.info, "Servo 1.50ms +0d 50Hz"), 1, 0);

      servo_g(buf, SIZE, 10000, 1000, 20000, 8);
      logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);
      check_near("hard over one way",
          !strcmp(lr.info, "Servo 1.00ms -90d 50Hz"), 1, 0);

      servo_g(buf, SIZE, 10000, 2000, 20000, 8);
      logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);
      check_near("and the other",
          !strcmp(lr.info, "Servo 2.00ms +90d 50Hz"), 1, 0);
    }

    // The swing a servo makes going from stop to stop, one width at a time.
    // Every one of them has to stay a servo: a single low pulse is a start
    // bit and eight zeros to a UART decoder, and the only thing that says
    // otherwise is the cadence around it - which does not change as the pulse
    // moves. This is the regression that matters, because the width sweeps
    // through everything on the way.
    printf("servo: the whole sweep, 1.00 to 2.00 ms:\n");
    {
      int bad_proto = 0, bad_width = 0, bad_rate = 0;

      for (int us = 1000; us <= 2000; us += 50)
      {
        servo_g(buf, SIZE, 10000, us, 20000, 8);
        logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);

        if (lr.proto != PROTO_SERVO)
          bad_proto++;

        if (lr.bytes[0] * 10 < us - 10 || lr.bytes[0] * 10 > us + 10)
          bad_width++;

        if (lr.rate != 20000000)
          bad_rate++;
      }

      check_near("never anything but servo", bad_proto, 0, 0);
      check_near("width follows", bad_width, 0, 0);
      check_near("rate stays put", bad_rate, 0, 0);
    }

    // A digital servo updates at 333 Hz, which is a 3 ms frame with the same
    // 1 to 2 ms pulse inside it. Nothing about the pulse changed; only the
    // cadence did, and the cadence is what is being measured.
    printf("servo at 333 Hz:\n");
    {
      servo_g(buf, SIZE, 10000, 1500, 3000, 20);
      logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);

      check_near("proto servo", lr.proto, PROTO_SERVO, 0);
      check_near("1.50 ms", lr.bytes[0], 150, 0);
      check_near("header", !strcmp(lr.info, "Servo 1.50ms +0d 333Hz"), 1, 0);
    }

    // 2.5 ms is past the end of the 1000..2000 the angle is defined over.
    // It is still a pulse train at a fixed rate and still gets reported as
    // one - receivers do send 2.2 ms, and an ESC reads the same line as
    // throttle - but the angle beside it is an extrapolation and says so.
    printf("servo past its travel:\n");
    {
      servo_g(buf, SIZE, 10000, 2500, 20000, 8);
      logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);

      check_near("still a pulse train", lr.proto, PROTO_SERVO, 0);
      check_near("width is what it is", lr.bytes[0], 250, 0);
      check_near("angle marked as off the end",
          !strcmp(lr.info, "Servo 2.50ms +180d! 50Hz"), 1, 0);

      // A pulse train whose rate wanders is not a servo line, however servo
      // its pulses look: the receiver's clock is the whole point
      int pos = 0;

      memset(buf, 56, SIZE);

      for (int f = 0; f < 5; f++)
      {
        int hi = pos + 150;

        for (; pos < hi && pos < SIZE; pos++)
          buf[pos] = 200;

        pos += 400 + f * 900; // 5.5, 14.5, 23.5... ms apart
      }

      check_near("wandering rate refused",
          servo_decode(buf, SIZE, 0, 10000, &scratch, &lr), 0, 0);

      // And a 115200 UART line is not one either - its pulses are 60 times
      // too narrow, which the width window catches before the rate is asked
      synth_uart(buf, SIZE, 512.0, 115200.0, "servo?", 100000.0);
      check_near("uart is not servo",
          servo_decode(buf, SIZE, 0, 512, &scratch, &lr), 0, 0);
    }

    // ----- CPPM: the same receiver's other pin, with every channel on it ----
    //
    // Upside down from the servo line beside it: there the pulse carries the
    // value in its width, here every separator is the same 400 us and the
    // GAPS carry everything. A whole frame - a sync, its channels and the
    // closing sync - is what the protocol is claimed on.
    printf("cppm sum signal:\n");
    {
      const int ch[8] = { 1500, 1000, 2000, 1100, 1900, 1500, 1500, 1500 };
      PpmGen g;
      char lab[16];

      ppm_g(&g, buf, SIZE, 3000.0, 0);

      for (int i = 0; i < 3; i++)
        ppm_g_frame(&g, ch, 8, 20000);

      int n = logic_decode(buf, SIZE, 0, 3000, PROTO_AUTO, &scratch, &lr);
      const PpmAnalysis *a = ppm_analysis();

      check_near("auto takes it", lr.proto, PROTO_PPM, 0);
      check_near("eight channels", a->chans, 8, 0);
      check_near("header", !strcmp(lr.info, "PPM 8ch 50Hz"), 1, 0);
      check_near("frame 20 ms", lr.rate, 20000000, 1);
      check_near("idle low", lr.idle_high, 0, 0);

      // The channels of the whole frame, and then the ones of the frame the
      // record ended inside - which are numbered because the sync in front of
      // them was seen, and marked cut because the one behind them was not
      check_near("bytes for both frames", n, 16, 0);
      check_near("ch1 1.50 ms", lr.bytes[0], 150, 0);
      check_near("ch2 1.00 ms", lr.bytes[1], 100, 0);
      check_near("ch3 2.00 ms", lr.bytes[2], 200, 0);
      check_near("ch4 1.10 ms", lr.bytes[3], 110, 0);
      check_near("ch5 1.90 ms", lr.bytes[4], 190, 0);

      check_near("two frames recorded", a->frames, 2, 0);
      check_near("the first one whole", a->frame[0].cut, 0, 0);
      check_near("the last one cut", a->frame[1].cut, 1, 0);

      // Separator to separator: the byte lights the interval its value was
      // measured over, not the pulse that opened it
      check_near("channel marked interval-wide",
          lr.end[0] - lr.pos[0], 500, 1);

      ppm_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("the width, in the units it is set in",
          !strcmp(lab, "1.50ms"), 1, 0);
      ppm_field_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("channel numbered in its frame",
          !strcmp(lab, "CH3"), 1, 0);
      // ...and the cut frame's channels start over at one rather than
      // carrying on from nine
      ppm_field_label(a, 8, lr.bytes[8], lab, sizeof(lab));
      check_near("the next frame starts at one",
          !strcmp(lab, "CH1"), 1, 0);
    }

    // Two whole frames, and the Futaba convention with them: the line rests
    // high and the separators pull it down, which is ordinary PPM
    printf("cppm, repeats and the inverted convention:\n");
    {
      const int ch[8] = { 1500, 1000, 2000, 1100, 1900, 1500, 1500, 1500 };
      PpmGen g;

      ppm_g(&g, buf, SIZE, 3500.0, 0);

      for (int i = 0; i < 4; i++)
        ppm_g_frame(&g, ch, 8, 20000);

      logic_decode(buf, SIZE, 0, 3500, PROTO_AUTO, &scratch, &lr);

      check_near("two whole frames", !strcmp(lr.info, "PPM 8ch 50Hz x2"), 1, 0);

      ppm_g(&g, buf, SIZE, 3500.0, 1);

      for (int i = 0; i < 4; i++)
        ppm_g_frame(&g, ch, 8, 20000);

      logic_decode(buf, SIZE, 0, 3500, PROTO_AUTO, &scratch, &lr);

      check_near("inverted still decodes", lr.proto, PROTO_PPM, 0);
      check_near("...and says so",
          !strcmp(lr.info, "PPM 8ch 50Hz x2 inv"), 1, 0);
      check_near("idle is high now", lr.idle_high, 1, 0);
      check_near("ch3 still 2.00 ms", lr.bytes[2], 200, 0);
    }

    // A receiver dropping a channel is the fault somebody is looking at this
    // signal to find, so the count is not averaged into one number
    printf("cppm, a frame that lost a channel:\n");
    {
      const int ch[8] = { 1500, 1000, 2000, 1100, 1900, 1500, 1500, 1500 };
      PpmGen g;

      ppm_g(&g, buf, SIZE, 3500.0, 0);
      ppm_g_frame(&g, ch, 8, 20000);
      ppm_g_frame(&g, ch, 8, 20000);
      ppm_g_frame(&g, ch, 7, 20000);
      ppm_g_frame(&g, ch, 8, 20000);

      logic_decode(buf, SIZE, 0, 3500, PROTO_AUTO, &scratch, &lr);
      const PpmAnalysis *a = ppm_analysis();

      check_near("the two whole frames disagreed", a->chans, 0, 0);
      check_near("...and the header says so, not an average",
          !strcmp(lr.info, "PPM 2 frames 50Hz"), 1, 0);
      check_near("eight in the first", a->frame[0].chans, 8, 0);
      check_near("seven in the second", a->frame[1].chans, 7, 0);
    }

    // The two signals on the two pins of the same receiver, and neither
    // decoder can take the other's record. A servo line's pulses are 20 ms
    // apart, so every interval on it is a sync gap and no channels ever fall
    // between two of them; a PPM stream's are 1 to 2 ms apart, which is below
    // the slowest frame rate a servo runs at.
    printf("cppm and servo are not each other:\n");
    {
      const int ch[8] = { 1500, 1000, 2000, 1100, 1900, 1500, 1500, 1500 };
      PpmGen g;

      servo_g(buf, SIZE, 10000, 1500, 20000, 8);
      check_near("servo is not cppm",
          ppm_decode(buf, SIZE, 0, 10000, &scratch, &lr), 0, 0);

      ppm_g(&g, buf, SIZE, 3000.0, 0);

      for (int i = 0; i < 3; i++)
        ppm_g_frame(&g, ch, 8, 20000);

      check_near("cppm is not servo",
          servo_decode(buf, SIZE, 0, 3000, &scratch, &lr), 0, 0);

      // A single frame with no closing sync is a train of pulses that could
      // be a great many things, and one is not claimed for it
      ppm_g(&g, buf, SIZE, 3000.0, 0);
      ppm_g_frame(&g, ch, 8, 20000);

      check_near("one frame is not enough",
          ppm_decode(buf, SIZE, 0, 3000, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST 9600", 20000.0);
      check_near("uart is not cppm",
          ppm_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not cppm",
          ppm_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    }

    // ----- CAN 2.0A/2.0B: 500 kbit/s, 500 ns sample period (4 samples/bit) --
    printf("can:\n");
    {
      uint8_t data[8] = { 0x01, 0x02, 0x03, 0x04, 0xDE, 0xAD, 0xBE, 0xEF };
      CanGen g = { .buf = buf, .size = SIZE, .bit_samples = 4.0 };

      memset(buf, 200, SIZE);   // the bus rests recessive
      can_g_idle(&g, 20);
      can_g_frame(&g, 0x123, 0, 0, data, 8, 1);

      int n = logic_decode(buf, SIZE, 0, 500, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("proto can", lr.proto, PROTO_CAN, 0);
      check_near("bit rate", lr.rate, 500000, 0);
      check_near("frames", a->frames, 1, 0);
      check_near("crc checked out", a->frame[0].crc_ok, 1, 0);
      check_near("id 123", a->frame[0].id, 0x123, 0);
      check_near("dlc 8", a->frame[0].dlc, 8, 0);
      check_near("acknowledged", a->frame[0].ack, 1, 0);
      // two identifier bytes, the length code, eight data, two of CRC
      check_near("bytes", n, 13, 0);
      check_near("id high", lr.bytes[0], 0x01, 0);
      check_near("id low", lr.bytes[1], 0x23, 0);
      check_near("data[4]", lr.bytes[7], 0xDE, 0);
      check_near("header", !strcmp(lr.info, "CAN 500k ID 123 8B"), 1, 0);

      // Two bytes carry eleven bits of identifier, so neither of them is the
      // identifier: the name opens the field and the number closes it
      char lab[16];
      can_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("label ID", !strcmp(lab, "ID"), 1, 0);
      can_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("label ID=123", !strcmp(lab, "ID=123"), 1, 0);

      // ...and both of them are one identifier, shown as one
      int gs, gl;

      can_group_at(a, 0, &gs, &gl);
      check_near("id groups", gs == 0 && gl == 2, 1, 0);
      can_group_at(a, 1, &gs, &gl);
      check_near("from either byte", gs == 0 && gl == 2, 1, 0);
      // The CRC's two bytes are not a group: one says whether it checked out
      // and the other whether anyone acknowledged the frame
      can_group_at(a, 11, &gs, &gl);
      check_near("crc stands alone", gs == 11 && gl == 1, 1, 0);

      can_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("field is ID, not ID=123", !strcmp(lab, "ID"), 1, 0);
      can_field_label(a, 7, lr.bytes[7], lab, sizeof(lab));
      check_near("field D4", !strcmp(lab, "D4"), 1, 0);
      can_byte_label(a, 7, lr.bytes[7], lab, sizeof(lab));
      check_near("label D4", !strcmp(lab, "D4"), 1, 0);
      can_byte_label(a, 12, lr.bytes[12], lab, sizeof(lab));
      check_near("label ACK", !strcmp(lab, "ACK"), 1, 0);
    }

    // The probe sits on one wire of a differential pair, and on CAN_H the
    // dominant level is the high one. Same frame, upside down.
    printf("can on CAN_H:\n");
    {
      uint8_t data[2] = { 0xAA, 0x55 };
      CanGen g = { .buf = buf, .size = SIZE, .bit_samples = 4.0, .inv = 1 };

      memset(buf, 56, SIZE);
      can_g_idle(&g, 20);
      can_g_frame(&g, 0x7FF, 0, 0, data, 2, 1);

      logic_decode(buf, SIZE, 0, 500, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("proto can", lr.proto, PROTO_CAN, 0);
      check_near("id 7FF", a->frame[0].id, 0x7FF, 0);
      check_near("crc checked out", a->frame[0].crc_ok, 1, 0);
      check_near("data", lr.bytes[3] == 0xAA && lr.bytes[4] == 0x55, 1, 0);
    }

    printf("can extended id:\n");
    {
      uint8_t data[8] = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17 };
      CanGen g = { .buf = buf, .size = SIZE, .bit_samples = 4.0 };

      memset(buf, 200, SIZE);
      can_g_idle(&g, 20);
      can_g_frame(&g, 0x18DAF110, 1, 0, data, 8, 1);

      int n = logic_decode(buf, SIZE, 0, 500, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("29-bit id", a->frame[0].id, 0x18DAF110, 0);
      check_near("marked extended", a->frame[0].ext, 1, 0);
      check_near("bytes", n, 15, 0);   // four identifier bytes now
      check_near("data[0]", lr.bytes[5], 0x10, 0);
      check_near("header", !strcmp(lr.info, "CAN 500k ID 18DAF110 8B"), 1, 0);
    }

    printf("can remote frame:\n");
    {
      CanGen g = { .buf = buf, .size = SIZE, .bit_samples = 4.0 };

      memset(buf, 200, SIZE);
      can_g_idle(&g, 20);
      can_g_frame(&g, 0x201, 0, 1, NULL, 8, 1);

      int n = logic_decode(buf, SIZE, 0, 500, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("marked remote", a->frame[0].rtr, 1, 0);
      check_near("no data bytes", n, 5, 0);
      check_near("header", !strcmp(lr.info, "CAN 500k ID 201 RTR"), 1, 0);
    }

    // Nobody else on the bus: the frame is complete and correct, and no node
    // pulled the acknowledge slot down. That is the single most useful thing
    // this decoder can tell someone whose node "does not work".
    printf("can with nobody listening:\n");
    {
      uint8_t data[1] = { 0x5A };
      CanGen g = { .buf = buf, .size = SIZE, .bit_samples = 4.0 };

      memset(buf, 200, SIZE);
      can_g_idle(&g, 20);
      can_g_frame(&g, 0x100, 0, 0, data, 1, 0);

      logic_decode(buf, SIZE, 0, 500, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("crc still checks out", a->frame[0].crc_ok, 1, 0);
      check_near("not acknowledged", a->frame[0].ack, 0, 0);
      check_near("header says so", !strcmp(lr.info, "CAN 500k ID 100 1B NAK"),
          1, 0);
    }

    printf("can several frames at 125k:\n");
    {
      uint8_t d1[3] = { 0x11, 0x22, 0x33 };
      uint8_t d2[2] = { 0x44, 0x55 };
      // 125 kbit/s at 2 us per sample is four samples a bit again
      CanGen g = { .buf = buf, .size = SIZE, .bit_samples = 4.0 };

      memset(buf, 200, SIZE);
      can_g_idle(&g, 20);
      can_g_frame(&g, 0x0A0, 0, 0, d1, 3, 1);
      can_g_frame(&g, 0x0B0, 0, 0, d2, 2, 1);

      logic_decode(buf, SIZE, 0, 2000, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("bit rate", lr.rate, 125000, 0);
      check_near("frames", a->frames, 2, 0);
      check_near("both check out", a->crc_ok, 2, 0);
      check_near("second id", a->frame[1].id, 0x0B0, 0);
      check_near("header", !strcmp(lr.info, "CAN 125k 2 frames"), 1, 0);
    }

    // A frame whose CRC does not agree is not proof of anything, and proof is
    // the whole reason this decoder is allowed to run ahead of the others
    printf("can with a corrupted CRC:\n");
    {
      uint8_t data[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
      CanGen g = { .buf = buf, .size = SIZE, .bit_samples = 4.0,
          .crc_xor = 0x0040 };

      memset(buf, 200, SIZE);
      can_g_idle(&g, 20);
      can_g_frame(&g, 0x123, 0, 0, data, 4, 1);

      check_near("refused", can_decode(buf, SIZE, 0, 500, &scratch, &lr), 0, 0);
    }

    // The two CAN FD polynomials against the check values in the catalogue of
    // parametrised CRC algorithms: the CRC of the nine ASCII digits, register
    // from zero. This is the one test here whose expected numbers come from
    // outside this repository, and it is the reason the rest of the CAN FD
    // tests mean anything - a generator and a decoder that share a typo in a
    // polynomial agree with each other perfectly.
    printf("can fd: the polynomials themselves:\n");
    {
      FdGen g = { .buf = buf, .size = SIZE, .arb = 4.0, .data = 4.0,
          .sp = 0.8, .crc_on = 1 };
      const char *s = "123456789";

      g.c17 = 0;
      g.c21 = 0;

      for (const char *p = s; *p; p++)
      {
        for (int i = 7; i >= 0; i--)
          fd_g_crc(&g, (*p >> i) & 1);
      }

      check_near("crc-17/can-fd check value", g.c17, 0x04F03, 0);
      check_near("crc-21/can-fd check value", g.c21, 0x0ED841, 0);
    }

    // ----- CAN FD: 500 kbit/s arbitration, 2 Mbit/s data -------------------
    // 100 ns a sample is 20 samples to an arbitration bit and 5 to a data
    // one. The record has to hold both, which is what makes CAN FD a harder
    // record than CAN: the fast phase is where the resolution goes.
    printf("can fd 500k/2M:\n");
    {
      uint8_t data[12] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
          0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC };
      FdGen g = { .buf = buf, .size = SIZE, .arb = 20.0, .data = 5.0,
          .sp = 0.8 };

      memset(buf, 200, SIZE);
      g.t = 0;
      fd_g_idle(&g, 20);
      fd_g_frame(&g, 0x123, 0, 1, 0, data, 9, 1);   // code 9 is twelve bytes
      fd_g_idle(&g, 20);

      int n = logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("proto can", lr.proto, PROTO_CAN, 0);
      check_near("arbitration rate", lr.rate, 500000, 0);
      check_near("data rate", a->data_rate, 2000000, 0);
      check_near("frames", a->frames, 1, 0);
      check_near("and it is an FD one", a->frame[0].fd, 1, 0);
      check_near("crc checked out", a->frame[0].crc_ok, 1, 0);
      check_near("id 123", a->frame[0].id, 0x123, 0);
      check_near("bit rate switched", a->frame[0].brs, 1, 0);
      check_near("error active", a->frame[0].esi, 0, 0);
      check_near("acknowledged", a->frame[0].ack, 1, 0);
      // The code is 9 and the length is twelve: on an FD frame the code is a
      // table index and not a count
      check_near("length code", a->frame[0].dlc, 9, 0);
      check_near("twelve bytes", a->frame[0].len, 12, 0);
      // two identifier bytes, the length code, twelve data, three of CRC
      check_near("bytes", n, 18, 0);
      check_near("id high", lr.bytes[0], 0x01, 0);
      check_near("id low", lr.bytes[1], 0x23, 0);
      check_near("dlc byte is the code", lr.bytes[2], 9, 0);
      check_near("data[0]", lr.bytes[3], 0x11, 0);
      check_near("data[11]", lr.bytes[14], 0xCC, 0);
      check_near("header", !strcmp(lr.info, "CAN FD 500k/2M 123 12B"), 1, 0);

      char lab[16];
      can_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("label FD12", !strcmp(lab, "FD12"), 1, 0);
      can_byte_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("label D0", !strcmp(lab, "D0"), 1, 0);
      can_byte_label(a, 14, lr.bytes[14], lab, sizeof(lab));
      check_near("label D11", !strcmp(lab, "D11"), 1, 0);
      can_byte_label(a, 15, lr.bytes[15], lab, sizeof(lab));
      check_near("label CRC", !strcmp(lab, "CRC"), 1, 0);
      can_byte_label(a, 17, lr.bytes[17], lab, sizeof(lab));
      check_near("label ACK", !strcmp(lab, "ACK"), 1, 0);
    }

    // Nothing on the wire says where the sample point is, and the bit rate
    // switches AT it - so the BRS bit is a length the decoder cannot compute.
    // Both ends of the range a controller is configured over have to decode,
    // and so does the ratio that makes the error worst: at 500k/8M one
    // arbitration bit is sixteen data bits, and 5% of sample point is most of
    // a data bit.
    printf("can fd: the sample point it cannot see:\n");
    {
      uint8_t data[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };
      double sp[3] = { 0.70, 0.80, 0.90 };
      int bad = 0, bad8 = 0;

      for (int i = 0; i < 3; i++)
      {
        FdGen g = { .buf = buf, .size = SIZE, .arb = 20.0, .data = 5.0,
            .sp = sp[i] };

        memset(buf, 200, SIZE);
        g.t = 0;
        fd_g_idle(&g, 20);
        fd_g_frame(&g, 0x0A5, 0, 1, 0, data, 8, 1);
        fd_g_idle(&g, 20);

        can_decode(buf, SIZE, 0, 100, &scratch, &lr);

        if (!can_analysis()->frames || !can_analysis()->frame[0].crc_ok)
          bad++;

        // ...and the same frame at a sixteen-to-one ratio, 8 samples to a
        // data bit at 40 ns
        FdGen h = { .buf = buf, .size = SIZE, .arb = 128.0, .data = 8.0,
            .sp = sp[i] };

        memset(buf, 200, SIZE);
        h.t = 0;
        fd_g_idle(&h, 20);
        fd_g_frame(&h, 0x0A5, 0, 1, 0, data, 8, 1);
        fd_g_idle(&h, 20);

        can_decode(buf, SIZE, 0, 40, &scratch, &lr);

        if (!can_analysis()->frames || !can_analysis()->frame[0].crc_ok)
          bad8++;
      }

      check_near("500k/2M decodes at every sample point", bad, 0, 0);
      check_near("500k/8M too", bad8, 0, 0);
    }

    // Sixty-four bytes is the headline of CAN FD and the one frame that does
    // not fit anywhere: 5.5 kbit on the wire, 21 bits of CRC rather than 17,
    // and seventy-two decoded bytes out the other end
    printf("can fd, 64 bytes:\n");
    {
      uint8_t data[64];
      FdGen g = { .buf = buf, .size = SIZE, .arb = 16.0, .data = 4.0,
          .sp = 0.8 };

      for (int i = 0; i < 64; i++)
        data[i] = (uint8_t)(i * 7 + 1);

      memset(buf, 200, SIZE);
      g.t = 0;
      fd_g_idle(&g, 20);
      fd_g_frame(&g, 0x18DAF110, 1, 1, 0, data, 15, 1);
      fd_g_idle(&g, 20);

      int n = logic_decode(buf, SIZE, 0, 125, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("proto can", lr.proto, PROTO_CAN, 0);
      check_near("29-bit id", a->frame[0].id, 0x18DAF110, 0);
      check_near("marked extended", a->frame[0].ext, 1, 0);
      check_near("sixty-four bytes", a->frame[0].len, 64, 0);
      check_near("crc-21 checked out", a->frame[0].crc_ok, 1, 0);
      // four identifier bytes, the length code, sixty-four data, three of CRC
      check_near("bytes", n, 72, 0);
      check_near("data[0]", lr.bytes[5], 1, 0);
      check_near("data[63]", lr.bytes[68], (uint8_t)(63 * 7 + 1), 0);
      check_near("header", !strcmp(lr.info, "CAN FD 500k/2M 18DAF110 64B"),
          1, 0);
    }

    // An FD frame is allowed not to switch: BRS dominant means the data phase
    // runs at the arbitration rate. Everything else about it is still FD -
    // the length table, the stuff count, the 17-bit CRC - so a decoder that
    // only knew the switch would miss it entirely.
    printf("can fd without the switch:\n");
    {
      uint8_t data[16];
      FdGen g = { .buf = buf, .size = SIZE, .arb = 4.0, .data = 4.0,
          .sp = 0.8 };

      for (int i = 0; i < 16; i++)
        data[i] = (uint8_t)(0xF0 + i);

      memset(buf, 200, SIZE);
      g.t = 0;
      fd_g_idle(&g, 20);
      fd_g_frame(&g, 0x201, 0, 0, 0, data, 10, 1);   // code 10 is sixteen
      fd_g_idle(&g, 20);

      logic_decode(buf, SIZE, 0, 500, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("still FD", a->frame[0].fd, 1, 0);
      check_near("no switch", a->frame[0].brs, 0, 0);
      check_near("sixteen bytes", a->frame[0].len, 16, 0);
      check_near("crc checked out", a->frame[0].crc_ok, 1, 0);
      // One rate, so one rate is what the header says
      check_near("header", !strcmp(lr.info, "CAN FD 500k 201 16B"), 1, 0);
    }

    // Error passive, and nobody listening. Both are things a scope is bought
    // to find, and neither shows up in the data.
    printf("can fd from a failing node:\n");
    {
      uint8_t data[4] = { 0x5A, 0xA5, 0x5A, 0xA5 };
      FdGen g = { .buf = buf, .size = SIZE, .arb = 20.0, .data = 5.0,
          .sp = 0.8 };

      memset(buf, 200, SIZE);
      g.t = 0;
      fd_g_idle(&g, 20);
      fd_g_frame(&g, 0x100, 0, 1, 1, data, 4, 0);
      fd_g_idle(&g, 20);

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("crc still checks out", a->frame[0].crc_ok, 1, 0);
      check_near("error passive", a->frame[0].esi, 1, 0);
      check_near("not acknowledged", a->frame[0].ack, 0, 0);
      check_near("header says both",
          !strcmp(lr.info, "CAN FD 500k/2M 100 4B NAK EP"), 1, 0);
    }

    // On CAN_H the dominant level is the high one. Same frame, upside down.
    printf("can fd on CAN_H:\n");
    {
      uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
      FdGen g = { .buf = buf, .size = SIZE, .arb = 20.0, .data = 5.0,
          .sp = 0.8, .inv = 1 };

      memset(buf, 56, SIZE);
      g.t = 0;
      fd_g_idle(&g, 20);
      fd_g_frame(&g, 0x7FF, 0, 1, 0, data, 8, 1);
      fd_g_idle(&g, 20);

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("id 7FF", a->frame[0].id, 0x7FF, 0);
      check_near("crc checked out", a->frame[0].crc_ok, 1, 0);
      check_near("data", lr.bytes[3] == 1 && lr.bytes[10] == 8, 1, 0);
    }

    // A bus that carries both, which is what a CAN FD bus actually looks
    // like: the classic frames fix the arbitration rate and the FD ones bring
    // the second. Getting this wrong means whichever kind came first wins the
    // record and the other kind is invisible.
    printf("can classic and fd on one bus:\n");
    {
      uint8_t d1[2] = { 0xC0, 0xDE };
      uint8_t d2[8] = { 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80 };
      CanGen c = { .buf = buf, .size = SIZE, .bit_samples = 20.0 };
      FdGen g = { .buf = buf, .size = SIZE, .arb = 20.0, .data = 5.0,
          .sp = 0.8 };

      memset(buf, 200, SIZE);
      can_g_idle(&c, 20);
      can_g_frame(&c, 0x0FF, 0, 0, d1, 2, 1);

      g.t = c.pos;
      fd_g_idle(&g, 5);
      fd_g_frame(&g, 0x1AB, 0, 1, 0, d2, 8, 1);
      fd_g_idle(&g, 20);

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("both frames", a->frames, 2, 0);
      check_near("both check out", a->crc_ok, 2, 0);
      check_near("one of them FD", a->fd, 1, 0);
      check_near("the classic one first", a->frame[0].fd, 0, 0);
      check_near("classic id", a->frame[0].id, 0x0FF, 0);
      check_near("fd id", a->frame[1].id, 0x1AB, 0);
      check_near("fd flag", a->frame[1].fd, 1, 0);
      check_near("header", !strcmp(lr.info, "CAN FD 500k/2M 2 frames"), 1, 0);
    }

    // Every length code, both frame formats, switched and not, over a payload
    // chosen to force stuff bits (0x00 and 0xFF runs) and one that forces
    // almost none. This is the sweep that covers what the single cases cannot:
    // the 17-to-21-bit CRC boundary at sixteen bytes, a stuff count that wraps
    // its three bits, and the retry over data-rate candidates - a candidate
    // that guesses wrong reads a bogus length code, and whatever it leaves
    // behind must not follow the candidate that guesses right.
    printf("can fd: every length code:\n");
    {
      uint8_t data[64];
      int bad = 0, cases = 0;

      for (int i = 0; i < 64; i++)
        data[i] = (uint8_t)((i / 4) % 3 == 0 ? 0x00 :
            (i / 4) % 3 == 1 ? 0xFF : (i * 37 + 5));

      for (int dlc = 0; dlc <= 15; dlc++)
      {
        for (int ext = 0; ext < 2; ext++)
        {
          for (int brs = 0; brs < 2; brs++)
          {
            FdGen g = { .buf = buf, .size = SIZE, .arb = 16.0,
                .data = brs ? 4.0 : 16.0, .sp = 0.8 };

            memset(buf, 200, SIZE);
            g.t = 0;
            fd_g_idle(&g, 20);
            fd_g_frame(&g, ext ? 0x1BCDEF01 : 0x2AB, ext, brs, 0, data, dlc,
                1);
            fd_g_idle(&g, 20);

            can_decode(buf, SIZE, 0, 125, &scratch, &lr);

            const CanAnalysis *a = can_analysis();

            cases++;

            if (1 != a->frames || !a->frame[0].crc_ok ||
                a->frame[0].len != g_fd_len[dlc] ||
                a->frame[0].ext != (ext != 0) ||
                a->frame[0].brs != (brs != 0) ||
                a->frame[0].id != (uint32_t)(ext ? 0x1BCDEF01 : 0x2AB))
              bad++;
          }
        }
      }

      check_near("all sixty-four cases", cases, 64, 0);
      check_near("none of them wrong", bad, 0, 0);
    }

    // And the refusals. A CRC that does not agree is not proof of anything,
    // and proof is the whole reason this decoder runs ahead of the others.
    printf("can fd refused:\n");
    {
      uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
      FdGen g = { .buf = buf, .size = SIZE, .arb = 20.0, .data = 5.0,
          .sp = 0.8, .crc_xor = 0x40 };

      memset(buf, 200, SIZE);
      g.t = 0;
      fd_g_idle(&g, 20);
      fd_g_frame(&g, 0x123, 0, 1, 0, data, 8, 1);
      fd_g_idle(&g, 20);

      check_near("bad crc refused",
          can_decode(buf, SIZE, 0, 100, &scratch, &lr), 0, 0);

      // Non-ISO CAN FD - the 2012 release, no stuff count and a CRC register
      // from zero. Every bit of it is a legal-looking FD frame right up to
      // the CRC field, and it is refused there rather than half-read.
      FdGen h = { .buf = buf, .size = SIZE, .arb = 20.0, .data = 5.0,
          .sp = 0.8, .noniso = 1 };

      memset(buf, 200, SIZE);
      h.t = 0;
      fd_g_idle(&h, 20);
      fd_g_frame(&h, 0x123, 0, 1, 0, data, 8, 1);
      fd_g_idle(&h, 20);

      check_near("non-iso refused",
          can_decode(buf, SIZE, 0, 100, &scratch, &lr), 0, 0);
    }

    // A record with nothing provable in it is refused whole, and there is no
    // channel for "I saw something" - proof is the only thing this decoder
    // says out loud. Where the flag DOES reach anybody is a record that holds
    // both: one FD frame read, and another the decoder had to give up on. The
    // rate is settled by the first, so the second is known to be a frame and
    // known not to be readable, which is worth saying.
    printf("can fd: one read, one given up on:\n");
    {
      uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
      FdGen g = { .buf = buf, .size = SIZE, .arb = 20.0, .data = 5.0,
          .sp = 0.8 };
      FdGen h;

      memset(buf, 200, SIZE);
      g.t = 0;
      fd_g_idle(&g, 20);
      fd_g_frame(&g, 0x111, 0, 1, 0, data, 8, 1);

      h = g;
      h.noniso = 1;
      fd_g_idle(&h, 10);
      fd_g_frame(&h, 0x222, 0, 1, 0, data, 8, 1);
      fd_g_idle(&h, 20);

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const CanAnalysis *a = can_analysis();

      check_near("the readable one is read", a->frames, 1, 0);
      check_near("and it checks out", a->crc_ok, 1, 0);
      check_near("id 111", a->frame[0].id, 0x111, 0);
      check_near("the other one is reported as unread", a->fd_seen, 1, 0);
    }

    // And the other direction: everything else on this bench must not come
    // out as CAN
    printf("can rejects the rest:\n");
    {
      synth_uart(buf, SIZE, 512.0, 115200.0, "CAN? no.", 100000.0);
      check_near("uart is not can",
          can_decode(buf, SIZE, 0, 512, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not can",
          can_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      double duty = 0.075;
      synth(buf, SIZE, 0, fn_square, &duty, 50.0, 10000.0, 90.0, ZERO_POINT);
      check_near("servo is not can",
          can_decode(buf, SIZE, 0, 10000, &scratch, &lr), 0, 0);
    }

    // ----- DHT22 / DHT11: forty bits, five bytes, the last one their sum ---
    printf("dht22:\n");
    {
      // 45.3 %, +23.4 C - tenths across both bytes of each pair
      uint8_t by[5] = { 0x01, 0xC5, 0x00, 0xEA, 0 };
      DhtGen g = { buf, SIZE, 0 };

      by[4] = (uint8_t)(by[0] + by[1] + by[2] + by[3]);
      memset(buf, 200, SIZE);
      dht_g_frame(&g, by, 1000);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const DhtAnalysis *a = dht_analysis();

      check_near("proto dht", lr.proto, PROTO_DHT, 0);
      check_near("bytes", n, 5, 0);
      // The sensor's 80/80 answer is a valid-looking bit slot itself, so a
      // decoder that commits to the first slot it finds is one bit out
      check_near("byte 0", lr.bytes[0], 0x01, 0);
      check_near("byte 1", lr.bytes[1], 0xC5, 0);
      check_near("read as a DHT22", a->dht22, 1, 0);
      check_near("humidity 45.3%", a->rh_x10, 453, 0);
      check_near("temperature 23.4C", a->t_x10, 234, 0);
      check_near("no doubt about which", a->sure, 1, 0);
      check_near("header", !strcmp(lr.info, "DHT22 45.3% +23.4C"), 1, 0);

      // 0x01 is not 45.3 % - the pair is. The field name goes on the byte
      // that opens it and the number on the byte that completes it.
      char lab[16];
      dht_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("label RH=45.3%", !strcmp(lab, "RH=45.3%"), 1, 0);
      dht_byte_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("label T=+23.4C", !strcmp(lab, "T=+23.4C"), 1, 0);
      dht_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("label SUM", !strcmp(lab, "SUM"), 1, 0);

      // Each reading is two bytes and one number, so it is one thing shown
      int gs, gl;

      dht_group_at(a, 0, &gs, &gl);
      check_near("humidity groups", gs == 0 && gl == 2, 1, 0);
      dht_group_at(a, 3, &gs, &gl);
      check_near("temperature groups", gs == 2 && gl == 2, 1, 0);
      dht_group_at(a, 4, &gs, &gl);
      check_near("checksum is alone", gs == 4 && gl == 1, 1, 0);

      dht_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("field RH-H", !strcmp(lab, "RH-H"), 1, 0);
      dht_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("field RH-L, not the reading", !strcmp(lab, "RH-L"), 1, 0);
      dht_field_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("field T-L", !strcmp(lab, "T-L"), 1, 0);
    }

    printf("dht22 below zero:\n");
    {
      // 65.4 %, -10.1 C: the sign is a bit, not a two's complement
      uint8_t by[5] = { 0x02, 0x8E, 0x80, 0x65, 0 };
      DhtGen g = { buf, SIZE, 0 };

      by[4] = (uint8_t)(by[0] + by[1] + by[2] + by[3]);
      memset(buf, 200, SIZE);
      dht_g_frame(&g, by, 1000);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const DhtAnalysis *a = dht_analysis();

      check_near("temperature -10.1C", a->t_x10, -101, 0);
      check_near("header", !strcmp(lr.info, "DHT22 65.4% -10.1C"), 1, 0);
    }

    // Same forty bits on the same wire; only the meaning of the bytes
    // differs. Read a DHT11's 45 % as a DHT22's would give 1152 % humidity,
    // and that is what settles which part sent them.
    printf("dht11:\n");
    {
      uint8_t by[5] = { 45, 0, 23, 0, 0 };
      DhtGen g = { buf, SIZE, 0 };

      by[4] = (uint8_t)(by[0] + by[1] + by[2] + by[3]);
      memset(buf, 200, SIZE);
      dht_g_frame(&g, by, 18000);   // 18 ms, which is how a DHT11 is asked

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const DhtAnalysis *a = dht_analysis();

      check_near("read as a DHT11", a->dht22, 0, 0);
      check_near("humidity 45%", a->rh_x10, 450, 0);
      check_near("temperature 23C", a->t_x10, 230, 0);
      check_near("header", !strcmp(lr.info, "DHT11 45% +23C"), 1, 0);
    }

    // A record that only caught the data - the start pulse is 18 ms wide and
    // a fast timebase does not reach back that far
    printf("dht22 without the start pulse:\n");
    {
      uint8_t by[5] = { 0x01, 0xC5, 0x00, 0xEA, 0 };
      DhtGen g = { buf, SIZE, 0 };

      by[4] = (uint8_t)(by[0] + by[1] + by[2] + by[3]);
      memset(buf, 200, SIZE);
      dht_g_frame(&g, by, 0);

      check_near("still decodes",
          dht_decode(buf, SIZE, 0, 1000, &scratch, &lr), 5, 0);
      check_near("humidity", dht_analysis()->rh_x10, 453, 0);
    }

    printf("dht with a broken checksum:\n");
    {
      uint8_t by[5] = { 0x01, 0xC5, 0x00, 0xEA, 0x42 };
      DhtGen g = { buf, SIZE, 0 };

      memset(buf, 200, SIZE);
      dht_g_frame(&g, by, 1000);

      check_near("refused", dht_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    }

    printf("dht rejects the rest:\n");
    {
      // A 1-Wire zero slot is a 60 us low and a short high, which is the
      // shape of a DHT bit - hence 1-Wire ahead of it in the cascade, and
      // hence this check
      OwGen og = { buf, SIZE, 0 };
      uint8_t sp[9] = { 0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0 };

      sp[8] = ow_crc(sp, 8);
      memset(buf, 200, SIZE);
      ow_reset(&og);
      ow_byte(&og, 0xCC);
      ow_byte(&og, 0xBE);
      ow_bytes(&og, sp, 9);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      check_near("1-wire stays 1-wire", lr.proto, PROTO_ONEWIRE, 0);

      synth_uart(buf, SIZE, 512.0, 115200.0, "humidity?", 100000.0);
      check_near("uart is not dht",
          dht_decode(buf, SIZE, 0, 512, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not dht",
          dht_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    }

    // ----- SENT / SAE J2716: 3 us tick, 100 ns sample period -------------
    printf("sent:\n");
    {
      // Two 12-bit signals, 0x7A3 and 0x85C, the layout nearly every sensor
      // sends: three nibbles each, most significant first
      uint8_t nib[6] = { 0x7, 0xA, 0x3, 0x8, 0x5, 0xC };
      SentGen g = { buf, SIZE, 0, 30.0, 0 };   // 3 us at 100 ns per sample

      memset(buf, 200, SIZE);
      sent_g_level(&g, 1, 20);                 // the tail of an earlier pause
      sent_g_frame(&g, 0x1, nib, 6, sent_t_crc(nib, 6, 1));

      int n = logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SentAnalysis *a = sent_analysis();

      check_near("proto sent", lr.proto, PROTO_SENT, 0);
      // status + six data + CRC
      check_near("nibbles", n, 8, 0);
      // The tick is not agreed in advance - it is read off the frame's own
      // 56-tick sync pulse, which is what lets a drifting sensor still decode
      check_near("tick 3 us", a->tick_ns, 3000, 3);
      check_near("frames", a->frames, 1, 0);
      check_near("crc checked out", a->frame[0].crc_ok, 1, 0);
      check_near("status", a->frame[0].status, 1, 0);
      check_near("data nibbles", a->frame[0].ndata, 6, 0);
      check_near("signal 1", a->frame[0].s1, 0x7A3, 0);
      check_near("signal 2", a->frame[0].s2, 0x85C, 0);
      check_near("nibble 3", lr.bytes[3], 0x3, 0);
      check_near("header", !strcmp(lr.info, "SENT 3.0us 7A3 85C"), 1, 0);

      char lab[16];
      sent_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("label STAT", !strcmp(lab, "STAT"), 1, 0);
      sent_byte_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("label S1=7A3", !strcmp(lab, "S1=7A3"), 1, 0);

      // Three nibbles are one number, so they are one thing on the screen:
      // the signal is written once across all three rather than "D1 D2
      // S1=7A3", which leaves the reader to work out that the first two
      // belong to the third
      int gs, gl;

      sent_group_at(a, 1, &gs, &gl);
      check_near("D1 is in signal 1", gs == 1 && gl == 3, 1, 0);
      sent_group_at(a, 3, &gs, &gl);
      check_near("so is the third nibble", gs == 1 && gl == 3, 1, 0);
      sent_group_at(a, 5, &gs, &gl);
      check_near("signal 2 groups too", gs == 4 && gl == 3, 1, 0);
      sent_group_at(a, 0, &gs, &gl);
      check_near("status is alone", gs == 0 && gl == 1, 1, 0);
      sent_group_at(a, 7, &gs, &gl);
      check_near("and so is the CRC", gs == 7 && gl == 1, 1, 0);

      // ...and the group answering "these three are 0x7A3" still leaves
      // "which of the three is this" to answer, which is a different row
      sent_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("field D1", !strcmp(lab, "D1"), 1, 0);
      sent_field_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("field D3, not the signal", !strcmp(lab, "D3"), 1, 0);
      sent_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("field STAT", !strcmp(lab, "STAT"), 1, 0);
      sent_field_label(a, 7, lr.bytes[7], lab, sizeof(lab));
      check_near("field CRC", !strcmp(lab, "CRC"), 1, 0);
      sent_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
      check_near("label S2=85C", !strcmp(lab, "S2=85C"), 1, 0);
      sent_byte_label(a, 7, lr.bytes[7], lab, sizeof(lab));
      check_near("label CRC", !strcmp(lab, "CRC"), 1, 0);
    }

    // A frame off the bench, kept as it came: status 0, channel 1 = 0x394,
    // channel 2 = 0xABC, CRC 0xF. The three things every SENT CRC gets wrong
    // are all in this one vector - the status nibble is NOT in the sum, the
    // seed is 5 and not 0, and the table is applied once more after the last
    // data nibble. Get any of them wrong and 0xF does not come out.
    printf("sent, a frame off the bench:\n");
    {
      uint8_t nib[6] = { 0x3, 0x9, 0x4, 0xA, 0xB, 0xC };
      SentGen g = { buf, SIZE, 0, 30.0, 0 };

      check_near("crc is F", sent_t_crc(nib, 6, 1), 0xF, 0);

      memset(buf, 200, SIZE);
      sent_g_level(&g, 1, 20);
      sent_g_frame(&g, 0x0, nib, 6, 0xF);

      int n = logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SentAnalysis *a = sent_analysis();

      check_near("proto sent", lr.proto, PROTO_SENT, 0);
      check_near("nibbles", n, 8, 0);
      check_near("status", a->frame[0].status, 0x0, 0);
      check_near("channel 1", a->frame[0].s1, 0x394, 0);
      check_near("channel 2", a->frame[0].s2, 0xABC, 0);
      check_near("crc as received", a->frame[0].crc, 0xF, 0);
      check_near("crc checks out", a->frame[0].crc_ok, 1, 0);
      check_near("the 2010 one", a->frame[0].legacy, 0, 0);
      check_near("not ambiguous", lr.ambiguous, 0, 0);
      check_near("nothing cut", lr.overrun, 0, 0);
      check_near("header", !strcmp(lr.info, "SENT 3.0us 394 ABC"), 1, 0);
    }

    // J2716 blessed two CRCs - the 2008 one and the 2010 version with an
    // extra zero nibble - and parts in the field use both
    printf("sent with the 2008 crc:\n");
    {
      uint8_t nib[6] = { 0x7, 0xA, 0x3, 0x8, 0x5, 0xC };
      SentGen g = { buf, SIZE, 0, 30.0, 0 };

      memset(buf, 200, SIZE);
      sent_g_level(&g, 1, 20);
      sent_g_frame(&g, 0x1, nib, 6, sent_t_crc(nib, 6, 0));

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SentAnalysis *a = sent_analysis();

      check_near("accepted", a->frame[0].crc_ok, 1, 0);
      check_near("noted as the older one", a->frame[0].legacy, 1, 0);
    }

    // The tick is whatever the sync says it is: same frame, sensor running
    // at 6 us, and nothing in the decoder is told about it
    printf("sent at another tick:\n");
    {
      uint8_t nib[3] = { 0x1, 0xF, 0x0 };
      SentGen g = { buf, SIZE, 0, 60.0, 0 };

      memset(buf, 200, SIZE);
      sent_g_level(&g, 1, 20);
      sent_g_frame(&g, 0x0, nib, 3, sent_t_crc(nib, 3, 1));

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SentAnalysis *a = sent_analysis();

      check_near("tick 6 us", a->tick_ns, 6000, 3);
      check_near("one signal", a->frame[0].s1, 0x1F0, 0);
      check_near("header", !strcmp(lr.info, "SENT 6.0us 1F0"), 1, 0);
    }

    printf("sent inverted:\n");
    {
      uint8_t nib[6] = { 0x0, 0x0, 0x1, 0xF, 0xF, 0xE };
      SentGen g = { buf, SIZE, 0, 30.0, 1 };

      memset(buf, 56, SIZE);
      sent_g_level(&g, 1, 20);
      sent_g_frame(&g, 0x3, nib, 6, sent_t_crc(nib, 6, 1));

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SentAnalysis *a = sent_analysis();

      check_near("still decodes", a->frames, 1, 0);
      check_near("signal 1", a->frame[0].s1, 0x001, 0);
      check_near("signal 2", a->frame[0].s2, 0xFFE, 0);
    }

    printf("sent, several frames:\n");
    {
      uint8_t n1[6] = { 0x7, 0xA, 0x3, 0x8, 0x5, 0xC };
      uint8_t n2[6] = { 0x1, 0x2, 0x3, 0x4, 0x5, 0x6 };
      SentGen g = { buf, SIZE, 0, 30.0, 0 };

      memset(buf, 200, SIZE);
      sent_g_level(&g, 1, 20);
      sent_g_frame(&g, 0x1, n1, 6, sent_t_crc(n1, 6, 1));
      sent_g_frame(&g, 0x2, n2, 6, sent_t_crc(n2, 6, 1));

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SentAnalysis *a = sent_analysis();

      check_near("frames", a->frames, 2, 0);
      check_near("both check out", a->crc_ok, 2, 0);
      check_near("second signal 1", a->frame[1].s1, 0x123, 0);
      check_near("header", !strcmp(lr.info, "SENT 3.0us 2 frames"), 1, 0);
    }

    // Shape without a CRC behind it is a strong hint and not a proof: the
    // cascade keeps looking, and a user who picked SENT by name still gets
    // the nibbles
    printf("sent with a broken crc:\n");
    {
      uint8_t nib[6] = { 0x7, 0xA, 0x3, 0x8, 0x5, 0xC };
      SentGen g = { buf, SIZE, 0, 30.0, 0 };

      memset(buf, 200, SIZE);
      sent_g_level(&g, 1, 20);
      sent_g_frame(&g, 0x1, nib, 6, (sent_t_crc(nib, 6, 1) + 1) & 0xF);

      check_near("decodes", sent_decode(buf, SIZE, 0, 100, &scratch, &lr), 8, 0);
      check_near("marked ambiguous", lr.ambiguous, 1, 0);
      check_near("auto keeps off it", logic_decode(buf, SIZE, 0, 100,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_SENT, 0, 0);
      check_near("named, it decodes", logic_decode(buf, SIZE, 0, 100,
          PROTO_SENT, &scratch, &lr), 8, 0);
      check_near("crc flagged", sent_analysis()->frame[0].crc_ok, 0, 0);
    }

    // A nibble is the distance between two falling edges, so the LAST pulse
    // of a frame needs a closing edge to have a value at all - on a real bus
    // that is the pause pulse or the next frame's sync. A record that ends
    // before it leaves that pulse unmeasurable, and the frame then has no CRC
    // in it at all. What must not happen is the last MEASURED nibble being
    // called the CRC: that puts a data nibble under the word and shifts every
    // label in the frame.
    //
    // Taken alone such a frame is genuinely undecidable - "status + 5 data +
    // CRC with the pause cut" and "status + 6 data with the CRC cut" are the
    // same waveform, and a four-bit CRC agrees with the wrong one of them
    // once in sixteen. What settles it is the frame BEFORE it: a sensor sends
    // one frame length, so a shorter one that runs into the end of the record
    // is a cut frame, and its CRC is not worth testing at all.
    printf("sent, whole frame then a cut one:\n");
    {
      uint8_t nib[6] = { 0x2, 0xC, 0xF, 0xA, 0xB, 0xC };
      SentGen g = { buf, SIZE, 0, 30.0, 0 };

      memset(buf, 200, SIZE);
      sent_g_level(&g, 1, 20);
      sent_g_frame(&g, 0x0, nib, 6, sent_t_crc(nib, 6, 1));

      // ...and a second one whose CRC pulse never gets its closing edge,
      // exactly as a capture that ends mid-frame leaves it
      sent_g_pulse(&g, 56);
      sent_g_pulse(&g, 12 + 0x0);

      for (int i = 0; i < 6; i++)
        sent_g_pulse(&g, 12 + nib[i]);

      sent_g_pulse(&g, 12 + sent_t_crc(nib, 6, 1));

      int n = sent_decode(buf, SIZE, 0, 100, &scratch, &lr);
      const SentAnalysis *a = sent_analysis();

      check_near("frames", a->frames, 2, 0);
      check_near("first checked out", a->frame[0].crc_ok, 1, 0);
      check_near("second has no CRC here", a->frame[1].no_crc, 1, 0);
      check_near("second not called broken", a->frame[1].crc_ok, 0, 0);
      // status + six data, and the CRC pulse that has no value here
      check_near("its data survives", a->frame[1].s1, 0x2CF, 0);
      check_near("and the rest of it", a->frame[1].s2, 0xABC, 0);
      check_near("nibbles", n, 8 + 7, 0);
      check_near("marked cut", lr.overrun, 1, 0);

      // and the last nibble of it is labelled as the data it is
      char lab[16];
      sent_byte_label(a, 14, lr.bytes[14], lab, sizeof(lab));
      check_near("last is S2, not CRC", !strcmp(lab, "S2=ABC"), 1, 0);

      // The first frame is whole and checked, so the record is SENT even
      // though the second one is a stub
      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      check_near("auto takes it", lr.proto, PROTO_SENT, 0);
    }

    printf("sent rejects the rest:\n");
    {
      // A console line at a fast timebase very nearly IS a SENT frame: pick
      // a seven-bit interval as the sync and the tick comes out at exactly
      // an eighth of a bit, so every falling edge afterwards lands on the
      // tick grid and intervals of two and three bits sit inside the nibble
      // window. What it cannot fake is the low time - a UART start bit holds
      // low for whole bit periods, which is eight ticks, and every SENT pulse
      // holds low for five.
      const char *line = "MILKV-UART-TEST 115200-8N1 #0123456789";

      synth_uart(buf, SIZE, 128.0, 115200.0, line, 100000.0);
      check_near("uart at 128 ns is not sent",
          sent_decode(buf, SIZE, 0, 128, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 256.0, 115200.0, line, 100000.0);
      check_near("uart at 256 ns is not sent",
          sent_decode(buf, SIZE, 0, 256, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 512.0, 115200.0, "not a sensor", 100000.0);
      check_near("uart is not sent",
          sent_decode(buf, SIZE, 0, 512, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not sent",
          sent_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      double duty = 0.075;
      synth(buf, SIZE, 0, fn_square, &duty, 50.0, 10000.0, 90.0, ZERO_POINT);
      check_near("servo is not sent",
          sent_decode(buf, SIZE, 0, 10000, &scratch, &lr), 0, 0);
    }

    // ============================ MIDI decoder ============================
    //
    // On the wire it is a UART, so the framing is not what identifies it: the
    // rate is 31250 and nothing else, and the bytes obey a grammar. Both are
    // asked for, because either alone is a coincidence.
    printf("midi, notes on and off:\n");
    {
      const uint8_t s[] = { 0x90, 0x3C, 0x64, 0x80, 0x3C, 0x40 };
      MidiGen g;
      char lab[16];

      midi_g(&g, buf, SIZE, 1000.0, 0);
      midi_g_bytes(&g, s, sizeof(s));

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      check_near("auto takes it", lr.proto, PROTO_MIDI, 0);
      check_near("bytes", n, 6, 0);
      check_near("messages", a->msgs, 2, 0);
      check_near("both whole", a->full, 2, 0);
      check_near("grammar clean", a->bad, 0, 0);
      check_near("rate", lr.rate, 31250, 0);
      check_near("header", !strcmp(lr.info, "MIDI 2 msgs ch1"), 1, 0);

      // Three bytes, one message: they light together and the reading is
      // written once across them
      int gs, gl;
      midi_group_at(a, 1, &gs, &gl);
      check_near("group start", gs, 0, 0);
      check_near("group length", gl, 3, 0);

      midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("note on reads", !strcmp(lab, "On C4 v100"), 1, 0);
      midi_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("note off reads", !strcmp(lab, "Off C4"), 1, 0);

      // ...and each byte still says which part of it it is
      midi_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("status names the channel", !strcmp(lab, "On ch1"), 1, 0);
      midi_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("first data is the note", !strcmp(lab, "NOTE"), 1, 0);
      midi_field_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("second is the velocity", !strcmp(lab, "VEL"), 1, 0);
    }

    // A note on at velocity zero IS a note off, and every sequencer sends it
    // that way so that running status can cover a whole phrase
    printf("midi, running status:\n");
    {
      const uint8_t s[] = { 0x93, 0x3C, 0x64, 0x3E, 0x50, 0x3C, 0x00 };
      MidiGen g;
      char lab[16];

      midi_g(&g, buf, SIZE, 1000.0, 0);
      midi_g_bytes(&g, s, sizeof(s));

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      check_near("three messages", a->msgs, 3, 0);
      check_near("all whole", a->full, 3, 0);
      check_near("grammar clean", a->bad, 0, 0);
      check_near("channel", a->chan, 4, 0);

      int gs, gl;
      midi_group_at(a, 4, &gs, &gl);
      check_near("second group start", gs, 3, 0);
      check_near("...is two bytes", gl, 2, 0);

      midi_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("second note reads", !strcmp(lab, "On D4 v80"), 1, 0);
      midi_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
      check_near("velocity 0 is note off", !strcmp(lab, "Off C4"), 1, 0);
      midi_field_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("running data is the note", !strcmp(lab, "NOTE"), 1, 0);
    }

    // A real-time byte may land between any two bytes of another message and
    // disturbs neither it nor running status. The interrupted message carries
    // on in a record of its own, because a group on the screen is a run of
    // ADJACENT bytes and a clock byte in the middle of a note is exactly the
    // case that would otherwise break that.
    printf("midi, a clock inside a note:\n");
    {
      const uint8_t s[] = { 0x90, 0x3C, 0xF8, 0x64 };
      MidiGen g;
      char lab[16];

      midi_g(&g, buf, SIZE, 1000.0, 0);
      midi_g_bytes(&g, s, sizeof(s));

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      check_near("nothing is broken", a->bad, 0, 0);
      check_near("the note is whole", a->full, 1, 0);
      check_near("one real-time byte", a->rt, 1, 0);

      // The groups tile: walking from 0 by the length returned always lands
      // on the next group's first byte
      int gs, gl, walk = 0, groups = 0;

      while (walk < lr.count)
      {
        midi_group_at(a, walk, &gs, &gl);
        check_near("group starts where we are", gs, walk, 0);
        walk += gl;
        groups++;
      }

      check_near("three groups", groups, 3, 0);

      midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("the clock is named", !strcmp(lab, "Clock"), 1, 0);
      midi_byte_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("the note finishes", !strcmp(lab, "On C4 v100"), 1, 0);
      midi_field_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("...on its velocity byte", !strcmp(lab, "VEL"), 1, 0);
    }

    printf("midi, the rest of the messages:\n");
    {
      // Control change, channel mode, pitch bend, program change - and a
      // system-exclusive with a manufacturer in it
      const uint8_t s[] =
      {
        0xB2, 0x07, 0x64,              // CC7 = 100 on channel 3
        0xB2, 0x7B, 0x00,              // controller 123: all notes off
        0xE2, 0x00, 0x40,              // pitch bend at centre
        0xC2, 0x05,                    // program change
        0xF0, 0x41, 0x10, 0x42, 0xF7,  // Roland system exclusive
      };
      MidiGen g;
      char lab[16];

      midi_g(&g, buf, SIZE, 1000.0, 0);
      midi_g_bytes(&g, s, sizeof(s));

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      check_near("bytes", lr.count, (int)sizeof(s), 0);
      check_near("messages", a->msgs, 5, 0);
      check_near("all whole", a->full, 5, 0);
      check_near("grammar clean", a->bad, 0, 0);

      midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("control change", !strcmp(lab, "CC7=100"), 1, 0);
      midi_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("channel mode is named", !strcmp(lab, "AllOff"), 1, 0);
      midi_byte_label(a, 8, lr.bytes[8], lab, sizeof(lab));
      check_near("bend is signed from centre", !strcmp(lab, "Bend +0"), 1, 0);
      midi_byte_label(a, 10, lr.bytes[10], lab, sizeof(lab));
      check_near("program change", !strcmp(lab, "Prog 5"), 1, 0);

      // The whole exclusive is one message, from 0xF0 to the 0xF7
      int gs, gl;
      midi_group_at(a, 13, &gs, &gl);
      check_near("sysex starts at F0", gs, 11, 0);
      check_near("...and runs to the EOX", gl, 5, 0);

      midi_byte_label(a, 15, lr.bytes[15], lab, sizeof(lab));
      check_near("sysex names the maker", !strcmp(lab, "SysEx Roland"), 1, 0);
      midi_field_label(a, 12, lr.bytes[12], lab, sizeof(lab));
      check_near("its first byte is the id", !strcmp(lab, "MFR"), 1, 0);
      midi_field_label(a, 15, lr.bytes[15], lab, sizeof(lab));
      check_near("its last is the EOX", !strcmp(lab, "EOX"), 1, 0);

      // The band walks the record a GROUP at a time, so the groups have to
      // tile it: stepping from zero by the length returned must land on the
      // next group's first byte every time, right to the end
      int walk = 0, groups = 0;

      while (walk < lr.count && groups <= lr.count)
      {
        midi_group_at(a, walk, &gs, &gl);
        check_near("group starts where we are", gs, walk, 0);
        walk += gl;
        groups++;
      }

      check_near("groups tile the record", groups, 5, 0);
    }

    printf("midi, notes across the keyboard:\n");
    {
      const uint8_t s[] = { 0x90, 0x00, 0x40, 0x3D, 0x40, 0x7F, 0x40 };
      MidiGen g;
      char lab[16];

      midi_g(&g, buf, SIZE, 1000.0, 0);
      midi_g_bytes(&g, s, sizeof(s));

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      // 60 is C4 because 69 is A440, which is A4
      midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("note 0", !strcmp(lab, "On C-1 v64"), 1, 0);
      midi_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("note 61", !strcmp(lab, "On C#4 v64"), 1, 0);
      midi_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
      check_near("note 127", !strcmp(lab, "On G9 v64"), 1, 0);
    }

    // Half a message is not a message. What must never happen is a value
    // written under a field that has not arrived - the byte after 0x3C could
    // be anything, and printing a velocity for it would invent one.
    printf("midi, a message the record cut:\n");
    {
      const uint8_t s[] = { 0x80, 0x3C, 0x40, 0x90, 0x3C };
      MidiGen g;
      char lab[16];

      midi_g(&g, buf, SIZE, 1000.0, 0);
      midi_g_bytes(&g, s, sizeof(s));

      int n = midi_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      check_near("bytes", n, 5, 0);
      check_near("not counted as broken", a->bad, 0, 0);
      check_near("one whole message", a->full, 1, 0);
      check_near("the other is marked cut", a->msg[1].partial, 1, 0);

      midi_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("named, not valued", !strcmp(lab, "NoteOn"), 1, 0);
    }

    // The optocoupler on the receiving end leaves the line idle high, but an
    // inverting buffer between it and the probe is somebody's design
    printf("midi, inverted:\n");
    {
      const uint8_t s[] = { 0x90, 0x40, 0x7F, 0x80, 0x40, 0x00 };
      MidiGen g;
      char lab[16];

      midi_g(&g, buf, SIZE, 1000.0, 1);
      midi_g_bytes(&g, s, sizeof(s));

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      check_near("still MIDI", lr.proto, PROTO_MIDI, 0);
      check_near("and says so", lr.idle_high, 0, 0);
      check_near("messages", a->msgs, 2, 0);
      midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("reads the same", !strcmp(lab, "On E4 v127"), 1, 0);
    }

    // A clock line carries nothing but real-time bytes, so no message with
    // data ever completes on it. What says it is MIDI and not a bit pattern
    // is the line RESTING between the bytes: a clock at 120 BPM is one byte
    // every 20 ms and the byte is a third of a millisecond long. A square
    // wave has no rest in it anywhere.
    printf("midi, a clock line:\n");
    {
      MidiGen g;
      char lab[16];

      midi_g(&g, buf, SIZE, 1000.0, 0);

      for (int i = 0; i < 4; i++)
      {
        midi_g_byte(&g, 0xF8);
        midi_g_run(&g, 1, 60);
      }

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      check_near("auto takes it", lr.proto, PROTO_MIDI, 0);
      check_near("bytes", lr.count, 4, 0);
      check_near("real-time bytes", a->rt, 4, 0);
      check_near("grammar clean", a->bad, 0, 0);
      check_near("not ambiguous", lr.ambiguous, 0, 0);
      check_near("header", !strcmp(lr.info, "MIDI 4 msgs"), 1, 0);

      midi_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("named", !strcmp(lab, "Clock"), 1, 0);
    }

    // 0xF4, 0xF5, 0xF9 and 0xFD are the codes the standard leaves undefined.
    // Nothing transmits them, so a record holding one is a fault in the
    // traffic - which is REPORTED. It is not a reason to hand the record to
    // the UART decoder, which would read the same byte as an ordinary one and
    // say nothing about it: the messages around it are unmistakably MIDI, and
    // the fault is the thing worth seeing.
    printf("midi, a byte that is not a message:\n");
    {
      const uint8_t s[] = { 0x90, 0x3C, 0x64, 0xF4, 0x90, 0x40, 0x64 };
      MidiGen g;

      midi_g(&g, buf, SIZE, 1000.0, 0);
      midi_g_bytes(&g, s, sizeof(s));

      int n = midi_decode(buf, SIZE, 0, 1000, &scratch, &lr);

      check_near("still decodes", n, 7, 0);
      check_near("and counts the fault", midi_analysis()->bad, 1, 0);
      check_near("the header says so", strstr(lr.info, "err") != NULL, 1, 0);
      check_near("identification stands", lr.ambiguous, 0, 0);
      check_near("auto still takes it", logic_decode(buf, SIZE, 0, 1000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_MIDI, 1, 0);
    }

    // More messages than there is room to describe. The byte list has to be
    // cut where the descriptions run out: a byte with no message behind it
    // would take the group and the label of some other message, and a record
    // that says plainly it held more beats one that quietly mislabels its end.
    printf("midi, more messages than fit:\n");
    {
      MidiGen g;
      int gs, gl;

      midi_g(&g, buf, SIZE, 1000.0, 0);
      midi_g_byte(&g, 0xC0);              // program change...

      for (int i = 0; i < 50; i++)
        midi_g_byte(&g, (uint8_t)(i & 0x7F));   // ...and 50 under running status

      int n = midi_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      const MidiAnalysis *a = midi_analysis();

      check_near("messages stop at the limit", a->msgs, MIDI_MAX_MSGS, 0);
      check_near("bytes stop with them", n, MIDI_MAX_MSGS + 1, 0);
      check_near("and it says so", lr.truncated, 1, 0);

      // Every byte that IS reported still belongs to a message of its own
      int walk = 0, groups = 0;

      while (walk < n && groups <= n)
      {
        midi_group_at(a, walk, &gs, &gl);
        check_near("group starts where we are", gs, walk, 0);
        walk += gl;
        groups++;
      }

      check_near("groups tile what was kept", groups, MIDI_MAX_MSGS, 0);
    }

    // ---- the bench generator's obstacle course, phase by phase ----
    //
    // Six phases, each aimed at a place where a first-version MIDI decoder
    // falls over. Run as separate records because that is what the scope
    // actually catches: the generator's cycle is seconds long and a record is
    // tens of milliseconds.
    {
      MidiGen g;
      char lab[16];
      int gs, gl;

      printf("obstacle 1, the warm-up:\n");
      {
        const uint8_t s[] = { 0x90, 0x3C, 0x64, 0x80, 0x3C, 0x40 };

        midi_g(&g, buf, SIZE, 1000.0, 0);
        midi_g_bytes(&g, s, sizeof(s));
        logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
        const MidiAnalysis *a = midi_analysis();

        check_near("is MIDI", lr.proto, PROTO_MIDI, 0);
        check_near("clean", a->bad, 0, 0);
        midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
        check_near("note on", !strcmp(lab, "On C4 v100"), 1, 0);
        midi_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
        check_near("note off", !strcmp(lab, "Off C4"), 1, 0);
      }

      // A C major chord under running status, then the same three notes
      // released with note-on at velocity zero - which is what keeps the
      // running status unbroken, and is exactly why it is sent that way
      printf("obstacle 2, running status:\n");
      {
        const uint8_t s[] =
        {
          0x90, 0x3C, 0x64, 0x40, 0x64, 0x43, 0x64,
          0x3C, 0x00, 0x40, 0x00, 0x43, 0x00,
        };

        midi_g(&g, buf, SIZE, 1000.0, 0);
        midi_g_bytes(&g, s, sizeof(s));
        logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
        const MidiAnalysis *a = midi_analysis();

        check_near("is MIDI", lr.proto, PROTO_MIDI, 0);
        check_near("six messages", a->msgs, 6, 0);
        check_near("all whole", a->full, 6, 0);
        check_near("clean", a->bad, 0, 0);

        midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
        check_near("root", !strcmp(lab, "On C4 v100"), 1, 0);
        midi_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
        check_near("third", !strcmp(lab, "On E4 v100"), 1, 0);
        midi_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
        check_near("fifth", !strcmp(lab, "On G4 v100"), 1, 0);
        midi_byte_label(a, 8, lr.bytes[8], lab, sizeof(lab));
        check_near("root released", !strcmp(lab, "Off C4"), 1, 0);
        midi_byte_label(a, 12, lr.bytes[12], lab, sizeof(lab));
        check_near("fifth released", !strcmp(lab, "Off G4"), 1, 0);

        // The chord's second and third notes have no status byte on the wire
        midi_group_at(a, 3, &gs, &gl);
        check_near("second note groups its pair", gs, 3, 0);
        check_near("...and only its pair", gl, 2, 0);
      }

      // Two data bytes, then one, then one, then two again. A decoder that
      // expects two of everything comes off the rails at the program change
      // and never gets back on.
      printf("obstacle 3, the zoo of lengths:\n");
      {
        const uint8_t s[] =
        {
          0xB0, 0x07, 0x64, 0xC0, 0x05, 0xD0, 0x30,
          0xE0, 0x00, 0x40, 0xE0, 0x7F, 0x7F,
        };

        midi_g(&g, buf, SIZE, 1000.0, 0);
        midi_g_bytes(&g, s, sizeof(s));
        logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
        const MidiAnalysis *a = midi_analysis();

        check_near("is MIDI", lr.proto, PROTO_MIDI, 0);
        check_near("five messages", a->msgs, 5, 0);
        check_near("all whole", a->full, 5, 0);
        check_near("clean", a->bad, 0, 0);

        midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
        check_near("control change", !strcmp(lab, "CC7=100"), 1, 0);
        midi_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
        check_near("program change", !strcmp(lab, "Prog 5"), 1, 0);
        midi_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
        check_near("channel pressure", !strcmp(lab, "Press 48"), 1, 0);
        // Fourteen bits, LSB first, centre 8192 - shown as the offset from it
        midi_byte_label(a, 9, lr.bytes[9], lab, sizeof(lab));
        check_near("bend at centre", !strcmp(lab, "Bend +0"), 1, 0);
        midi_byte_label(a, 12, lr.bytes[12], lab, sizeof(lab));
        check_near("bend at the top", !strcmp(lab, "Bend +8191"), 1, 0);
      }

      printf("obstacle 4, real-time barging in:\n");
      {
        const uint8_t s[] = { 0xF8, 0x90, 0x3C, 0xF8, 0x64, 0xFE };

        midi_g(&g, buf, SIZE, 1000.0, 0);
        midi_g_bytes(&g, s, sizeof(s));
        logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
        const MidiAnalysis *a = midi_analysis();

        check_near("is MIDI", lr.proto, PROTO_MIDI, 0);
        check_near("clean", a->bad, 0, 0);
        check_near("the note is whole", a->full, 1, 0);
        check_near("three real-time bytes", a->rt, 3, 0);

        midi_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
        check_near("clock on its own", !strcmp(lab, "Clock"), 1, 0);
        midi_byte_label(a, 3, lr.bytes[3], lab, sizeof(lab));
        check_near("clock inside the note", !strcmp(lab, "Clock"), 1, 0);
        midi_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
        check_near("the note finishes", !strcmp(lab, "On C4 v100"), 1, 0);
        midi_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
        check_near("active sensing", !strcmp(lab, "Sense"), 1, 0);

        // The interrupted note is two groups, not one broken one
        midi_group_at(a, 2, &gs, &gl);
        check_near("first half groups", gs, 1, 0);
        check_near("...up to the clock", gl, 2, 0);
        midi_group_at(a, 4, &gs, &gl);
        check_near("the rest is its own group", gs, 4, 0);
        check_near("...of one byte", gl, 1, 0);
      }

      printf("obstacle 5, system exclusive:\n");
      {
        const uint8_t s[] =
        {
          0xF0, 0x7D, 0x01, 0x02, 0x03, 0xF7,
          0xF0, 0x7D, 0x11, 0xF8, 0x22, 0xF7,
        };

        midi_g(&g, buf, SIZE, 1000.0, 0);
        midi_g_bytes(&g, s, sizeof(s));
        logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
        const MidiAnalysis *a = midi_analysis();

        check_near("is MIDI", lr.proto, PROTO_MIDI, 0);
        check_near("clean", a->bad, 0, 0);
        check_near("both closed", a->full, 2, 0);
        check_near("one clock", a->rt, 1, 0);

        // The first is one group from F0 to F7
        midi_group_at(a, 3, &gs, &gl);
        check_near("plain sysex groups whole", gs, 0, 0);
        check_near("...F0 to F7", gl, 6, 0);

        midi_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
        check_near("named by its maker id", !strcmp(lab, "SysEx Test"), 1, 0);

        // The second is split by the clock and carries on afterwards
        midi_group_at(a, 7, &gs, &gl);
        check_near("second sysex starts", gs, 6, 0);
        check_near("...and stops at the clock", gl, 3, 0);
        midi_byte_label(a, 9, lr.bytes[9], lab, sizeof(lab));
        check_near("clock shown", !strcmp(lab, "Clock"), 1, 0);
        midi_group_at(a, 10, &gs, &gl);
        check_near("and resumes after it", gs, 10, 0);
        check_near("...to its EOX", gl, 2, 0);
        midi_byte_label(a, 11, lr.bytes[11], lab, sizeof(lab));
        check_near("still the same sysex", !strcmp(lab, "SysEx Test"), 1, 0);
        midi_field_label(a, 11, lr.bytes[11], lab, sizeof(lab));
        check_near("closed by its EOX", !strcmp(lab, "EOX"), 1, 0);
      }

      // Faults, and the decoder has to survive them AND say so. Being turned
      // down over them is the wrong answer: the UART decoder would take the
      // record and read every one of these bytes as an ordinary byte.
      printf("obstacle 6a, a message cut off by the next status:\n");
      {
        const uint8_t s[] = { 0x90, 0x3C, 0xB0, 0x07, 0x64 };

        midi_g(&g, buf, SIZE, 1000.0, 0);
        midi_g_bytes(&g, s, sizeof(s));
        logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
        const MidiAnalysis *a = midi_analysis();

        check_near("still MIDI", lr.proto, PROTO_MIDI, 0);
        check_near("the fault is counted", a->bad, 1, 0);
        check_near("the header says so", strstr(lr.info, "err") != NULL, 1, 0);

        // The half-sent note is named and NOT valued: the byte after 0x3C
        // could have been anything, and a velocity here would be invented
        midi_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
        check_near("half a note is not a note", !strcmp(lab, "NoteOn"), 1, 0);
        midi_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
        check_near("the CC is taken", !strcmp(lab, "CC7=100"), 1, 0);
      }

      // System common cancels running status, so the two bytes after it have
      // no status in front of them at all. Applying the old 0x90 is the naive
      // answer and the wrong one - 0xF3 killed it.
      printf("obstacle 6b, orphans after a system common:\n");
      {
        const uint8_t s[] = { 0x90, 0x3C, 0x64, 0xF3, 0x01, 0x3C, 0x64 };

        midi_g(&g, buf, SIZE, 1000.0, 0);
        midi_g_bytes(&g, s, sizeof(s));
        logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
        const MidiAnalysis *a = midi_analysis();

        check_near("still MIDI", lr.proto, PROTO_MIDI, 0);
        check_near("both orphans counted", a->bad, 2, 0);
        check_near("the header says so", strstr(lr.info, "err") != NULL, 1, 0);

        midi_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
        check_near("song select read", !strcmp(lab, "Song 1"), 1, 0);
        // ...and NOT read as another note on channel 1
        midi_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
        check_near("orphan not made a note", !strcmp(lab, "data"), 1, 0);
        midi_group_at(a, 5, &gs, &gl);
        check_near("orphan stands alone", gl, 1, 0);
      }

      // No timeouts in MIDI: a data byte 50 ms behind its status byte is
      // legal, and the message must not be thrown away. 4 us a sample puts
      // the whole 98 ms of it in one record.
      printf("obstacle 6c, a data byte 50 ms late:\n");
      {
        midi_g(&g, buf, SIZE, 4000.0, 0);
        midi_g_byte(&g, 0x90);
        midi_g_byte(&g, 0x40);
        midi_g_run(&g, 1, 50000.0 / 32.0);   // 50 ms of idle, in bit times
        midi_g_byte(&g, 0x64);

        logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
        const MidiAnalysis *a = midi_analysis();

        check_near("still MIDI", lr.proto, PROTO_MIDI, 0);
        check_near("bytes", lr.count, 3, 0);
        check_near("no fault", a->bad, 0, 0);
        check_near("the message is whole", a->full, 1, 0);
        midi_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
        check_near("waited for it", !strcmp(lab, "On E4 v100"), 1, 0);
      }
    }

    printf("midi rejects the rest:\n");
    {
      // A serial link at 31250 baud carrying text: the framing is perfect and
      // there is no grammar in it at all, because every ASCII byte is a data
      // byte and not one of them is a status. That is the case the rate alone
      // would get wrong.
      synth_uart(buf, SIZE, 1000.0, 31250.0, "Hello, world!", 20000.0);
      check_near("text at 31250 is refused",
          midi_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
      check_near("...and auto reads it as UART", logic_decode(buf, SIZE, 0,
          1000, PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_UART, 1, 0);

      // ...and a link at any other rate is not MIDI at all: every run in the
      // record is a whole number of bit times, and at 31250 one bit is wider
      // than the shortest run here
      synth_uart(buf, SIZE, 1000.0, 115200.0, "MILKV-UART-TEST", 20000.0);
      check_near("115200 is not midi",
          midi_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST", 20000.0);
      check_near("9600 is not midi",
          midi_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not midi",
          midi_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      double duty = 0.075;
      synth(buf, SIZE, 0, fn_square, &duty, 50.0, 10000.0, 90.0, ZERO_POINT);
      check_near("servo is not midi",
          midi_decode(buf, SIZE, 0, 10000, &scratch, &lr), 0, 0);

      // Undersampled: at 100 us a bit is a third of a sample and the stop bit
      // cannot be placed at all
      check_near("undersampled is refused",
          midi_decode(buf, SIZE, 0, 100000, &scratch, &lr), 0, 0);
    }

    // ============================ LIN decoder =============================
    //
    // The break is what identifies it: at least ten dominant bits, and 8N1
    // tops out at nine. The sync byte is what clocks it: nine runs of exactly
    // one bit, and eight bit times between the first falling edge and the
    // last. Neither is a guess, so there is no rate search here at all.
    printf("lin, one frame:\n");
    {
      const uint8_t d[4] = { 0x11, 0x22, 0x33, 0x44 };
      LinGen g;
      char lab[16];

      lin_g(&g, buf, SIZE, 1000.0, 19200.0, 0);
      lin_g_frame(&g, 0x2A, d, 4, 1);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const LinAnalysis *a = lin_analysis();

      check_near("auto takes it", lr.proto, PROTO_LIN, 0);
      check_near("sync, pid, data, checksum", n, 2 + 5, 0);
      check_near("rate off the sync byte", lr.rate, 19200, 0);
      check_near("one frame", a->frames, 1, 0);
      check_near("identifier", a->frame[0].id, 0x2A, 0);
      check_near("parity checks", a->frame[0].parity_ok, 1, 0);
      check_near("checksum checks", a->frame[0].csum_ok, 1, 0);
      check_near("enhanced", a->frame[0].enhanced, 1, 0);
      check_near("four data bytes", a->frame[0].ndata, 4, 0);
      check_near("header", !strcmp(lr.info, "LIN 19200 ID=2A 4B enh"), 1, 0);

      lin_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("sync named", !strcmp(lab, "SYNC"), 1, 0);
      lin_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("identifier under its parity", !strcmp(lab, "ID=2A"), 1, 0);
      lin_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("first data byte", !strcmp(lab, "D1"), 1, 0);
      lin_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
      check_near("checksum", !strcmp(lab, "CHK"), 1, 0);
    }

    // The rate is not searched for and not assumed - it comes out of the sync
    // byte's own edges every frame, which is how a slave does it
    printf("lin at the rates a car uses:\n");
    {
      const double baud[] = { 2400.0, 9600.0, 19200.0 };
      const uint8_t d[2] = { 0xAB, 0xCD };

      for (int i = 0; i < 3; i++)
      {
        LinGen g;

        lin_g(&g, buf, SIZE, 4000.0, baud[i], 0);
        lin_g_frame(&g, 0x05, d, 2, 1);

        logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);

        check_near("found without being told", lr.rate, baud[i], 0);
        check_near("and it is LIN", lr.proto, PROTO_LIN, 0);
      }
    }

    printf("lin, several frames and the classic checksum:\n");
    {
      const uint8_t d1[2] = { 0x01, 0x02 };
      const uint8_t d2[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
      LinGen g;

      // LIN 1.3: the checksum covers the data and not the identifier
      lin_g(&g, buf, SIZE, 1000.0, 9600.0, 0);
      lin_g_frame(&g, 0x11, d1, 2, 0);
      lin_g_frame(&g, 0x32, d2, 8, 0);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const LinAnalysis *a = lin_analysis();

      check_near("two frames", a->frames, 2, 0);
      check_near("both check out", a->csum_ok, 2, 0);
      check_near("second is eight bytes", a->frame[1].ndata, 8, 0);
      check_near("classic, not enhanced", a->frame[1].enhanced, 0, 0);
      check_near("header says so",
          !strcmp(lr.info, "LIN 9600 2 frames cls"), 1, 0);
    }

    // A header nobody answered. The master asked for that identifier and no
    // slave on the bus owns it - which is what an unplugged module looks like
    // and is the most useful thing the decoder can say about it.
    printf("lin, a header with no response:\n");
    {
      LinGen g;

      lin_g(&g, buf, SIZE, 1000.0, 19200.0, 0);
      lin_g_header(&g, 0x14);
      lin_g_run(&g, 1, 200);              // ...and nothing comes back

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const LinAnalysis *a = lin_analysis();

      check_near("still LIN", lr.proto, PROTO_LIN, 0);
      check_near("the header is there", n, 2, 0);
      check_near("and it is empty", a->frame[0].no_resp, 1, 0);
      check_near("parity still checks", a->frame[0].parity_ok, 1, 0);
      check_near("header", !strcmp(lr.info, "LIN 19200 ID=14 no resp"), 1, 0);
    }

    // The two diagnostic identifiers are the transport layer, and their first
    // three bytes have names rather than numbers
    printf("lin, a diagnostic frame:\n");
    {
      // NAD 0x01, PCI 0x06, SID 0xB2 (read by identifier), then its arguments
      const uint8_t d[8] = { 0x01, 0x06, 0xB2, 0x00, 0x01, 0x02, 0x03, 0xFF };
      LinGen g;
      char lab[16];

      lin_g(&g, buf, SIZE, 1000.0, 19200.0, 0);
      lin_g_frame(&g, 0x3C, d, 8, 0);     // classic even on a 2.x bus

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const LinAnalysis *a = lin_analysis();

      check_near("checksum checks", a->frame[0].csum_ok, 1, 0);
      check_near("...the classic one", a->frame[0].enhanced, 0, 0);
      // ...and that is mandatory here, so it says nothing about the bus
      check_near("not called out as classic",
          strstr(lr.info, "cls") == NULL, 1, 0);

      lin_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("node address", !strcmp(lab, "NAD 01"), 1, 0);
      lin_byte_label(a, 3, lr.bytes[3], lab, sizeof(lab));
      check_near("frame control", !strcmp(lab, "PCI 06"), 1, 0);
      lin_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("service named", !strcmp(lab, "ReadById"), 1, 0);
      lin_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("...and its arguments numbered", !strcmp(lab, "D1"), 1, 0);
    }

    // Faults are reported. The break and the sync have already identified the
    // bus by the time either of these is checked.
    printf("lin, a broken checksum and a broken parity:\n");
    {
      const uint8_t d[2] = { 0x55, 0xAA };
      LinGen g;
      char lab[16];

      lin_g(&g, buf, SIZE, 1000.0, 19200.0, 0);
      lin_g_header(&g, 0x0B);
      lin_g_byte(&g, d[0]);
      lin_g_byte(&g, d[1]);
      lin_g_byte(&g, (uint8_t)(lin_g_csum(d, 2, lin_g_pid(0x0B), 1) ^ 0x01));
      lin_g_run(&g, 1, 20);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const LinAnalysis *a = lin_analysis();

      check_near("still LIN", lr.proto, PROTO_LIN, 0);
      check_near("checksum flagged", a->frame[0].csum_ok, 0, 0);
      check_near("parity still fine", a->frame[0].parity_ok, 1, 0);
      check_near("counted as an error", lr.errors, 1, 0);
      check_near("header says so", strstr(lr.info, "CHK!") != NULL, 1, 0);
      lin_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("and the byte says so", !strcmp(lab, "CHK!"), 1, 0);

      // A protected identifier whose parity does not hold is not an
      // identifier, so the six bits under it are shown with a '!'
      lin_g(&g, buf, SIZE, 1000.0, 19200.0, 0);
      lin_g_run(&g, 0, 13);
      lin_g_run(&g, 1, 1);
      lin_g_byte(&g, 0x55);
      lin_g_byte(&g, (uint8_t)(lin_g_pid(0x0B) ^ 0x40));   // one parity bit off
      lin_g_byte(&g, d[0]);
      lin_g_byte(&g, d[1]);
      lin_g_byte(&g, lin_g_csum(d, 2, (uint8_t)(lin_g_pid(0x0B) ^ 0x40), 1));
      lin_g_run(&g, 1, 20);

      int n = lin_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      const LinAnalysis *b = lin_analysis();

      check_near("still decodes", n, 5, 0);
      check_near("parity flagged", b->frame[0].parity_ok, 0, 0);
      check_near("marked ambiguous", lr.ambiguous, 1, 0);
      lin_byte_label(b, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("identifier marked", !strcmp(lab, "ID=0B!"), 1, 0);
    }

    printf("lin, inverted:\n");
    {
      const uint8_t d[2] = { 0x12, 0x34 };
      LinGen g;

      lin_g(&g, buf, SIZE, 1000.0, 19200.0, 1);
      lin_g_frame(&g, 0x2A, d, 2, 1);

      logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const LinAnalysis *a = lin_analysis();

      check_near("still LIN", lr.proto, PROTO_LIN, 0);
      check_near("and says so", lr.idle_high, 0, 0);
      check_near("identifier", a->frame[0].id, 0x2A, 0);
      check_near("checksum checks", a->frame[0].csum_ok, 1, 0);
    }

    // A frame that came off the bench generator and was read on the device,
    // reproduced byte for byte. Its break is not a real break at all - it is
    // a 0x00 byte sent at half the frame's rate, which is the usual way to
    // make one out of a plain UART, and it lands at 18 bit times.
    //
    //   55 | 50 | 70 A5 | E9
    //   PID 0x50 = id 0x10, P0 = 0^0^0^1 = 1, P1 = !(0^0^1^0) = 0
    //   CHK classic: 0x70 + 0xA5 = 0x115 -> carry back -> 0x16 -> ~ -> 0xE9
    printf("lin, the frame the bench generator sent:\n");
    {
      const uint8_t d[2] = { 0x70, 0xA5 };
      LinGen g;
      char lab[16];

      check_near("its PID is 0x50", lin_g_pid(0x10), 0x50, 0);
      check_near("its classic checksum is 0xE9",
          lin_g_csum(d, 2, 0x50, 0), 0xE9, 0);

      lin_g(&g, buf, SIZE, 1000.0, 9600.0, 0);
      lin_g_run(&g, 0, 18);               // 0x00 at half rate: 18 bit times
      lin_g_run(&g, 1, 2);
      lin_g_byte(&g, 0x55);
      lin_g_byte(&g, 0x50);
      lin_g_byte(&g, d[0]);
      lin_g_byte(&g, d[1]);
      lin_g_byte(&g, 0xE9);
      lin_g_run(&g, 1, 40);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const LinAnalysis *a = lin_analysis();

      check_near("auto takes it", lr.proto, PROTO_LIN, 0);
      check_near("five bytes", n, 5, 0);
      check_near("rate", lr.rate, 9600, 0);
      check_near("identifier", a->frame[0].id, 0x10, 0);
      check_near("parity checks", a->frame[0].parity_ok, 1, 0);
      check_near("checksum checks", a->frame[0].csum_ok, 1, 0);
      check_near("and it is the classic one", a->frame[0].enhanced, 0, 0);
      check_near("no errors", lr.errors, 0, 0);
      // Which checksum the bus uses is said out loud, not left to be inferred
      // from the absence of the other word
      check_near("header", !strcmp(lr.info, "LIN 9600 ID=10 2B cls"), 1, 0);

      lin_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("identifier read", !strcmp(lab, "ID=10"), 1, 0);
      lin_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("counter byte", !strcmp(lab, "D1"), 1, 0);
      check_near("...and its value", lr.bytes[2], 0x70, 0);
      lin_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("checksum", !strcmp(lab, "CHK"), 1, 0);

      // The same data under the enhanced checksum is a DIFFERENT byte, which
      // is what makes telling the two apart worth doing rather than guessing
      check_near("enhanced would be another byte",
          lin_g_csum(d, 2, 0x50, 1) != 0xE9, 1, 0);
    }

    printf("lin rejects the rest:\n");
    {
      // 8N1 cannot send a break: nine dominant bits is the most it has, and
      // the stop bit is obliged to let the line up after them. This is the
      // whole identification, so it is the one thing worth testing hardest.
      synth_uart(buf, SIZE, 1000.0, 19200.0, "\x00\x00\x00\x00 break?", 20000.0);
      check_near("a run of zero bytes is not a break",
          lin_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST 9600", 20000.0);
      check_near("uart is not lin",
          lin_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not lin",
          lin_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      double duty = 0.075;
      synth(buf, SIZE, 0, fn_square, &duty, 50.0, 10000.0, 90.0, ZERO_POINT);
      check_near("servo is not lin",
          lin_decode(buf, SIZE, 0, 10000, &scratch, &lr), 0, 0);

      // ...and LIN does not take anyone else's records either. It runs ahead
      // of 1-Wire, WS2812, DHT, SENT, MIDI, servo and UART in the cascade, so
      // every one of those has to bounce off it - and the two that start with
      // a long dominant pulse of their own are the ones that matter: a 1-Wire
      // reset is 480 us of it and a DHT start pulse is 18 ms, and both are
      // followed by something that has to pass for a sync byte and does not.
      {
        MidiGen mg;
        const uint8_t s[] = { 0x90, 0x3C, 0x64, 0x80, 0x3C, 0x40 };

        midi_g(&mg, buf, SIZE, 1000.0, 0);
        midi_g_bytes(&mg, s, sizeof(s));
        check_near("midi is not lin",
            lin_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
      }

      {
        // A DS18B20 conversation: reset, SKIP ROM, CONVERT T, reset, READ
        const uint8_t sp[9] =
            { 0x50, 0x05, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0x00 };
        uint8_t sp2[9];
        OwGen g = { buf, SIZE, 0 };

        memcpy(sp2, sp, 9);
        sp2[8] = ow_crc(sp2, 8);

        memset(buf, 200, SIZE);
        ow_reset(&g);
        ow_byte(&g, 0xCC);
        ow_byte(&g, 0x44);
        ow_reset(&g);
        ow_byte(&g, 0xCC);
        ow_byte(&g, 0xBE);
        ow_bytes(&g, sp2, 9);

        check_near("1-wire is not lin",
            lin_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
        check_near("...and still decodes as 1-wire", logic_decode(buf, SIZE, 0,
            1000, PROTO_AUTO, &scratch, &lr) > 0 &&
            lr.proto == PROTO_ONEWIRE, 1, 0);
      }

      {
        // The 18 ms start pulse of a DHT11 is a longer dominant field than
        // any LIN break, and what follows it is not nine equal runs
        const uint8_t by[5] = { 45, 0, 23, 0, 68 };
        DhtGen g = { buf, SIZE, 0 };

        memset(buf, 200, SIZE);
        dht_g_frame(&g, by, 18000);

        check_near("dht is not lin",
            lin_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
        check_near("...and still decodes as dht", logic_decode(buf, SIZE, 0,
            1000, PROTO_AUTO, &scratch, &lr) > 0 &&
            lr.proto == PROTO_DHT, 1, 0);
      }

      {
        uint8_t nib[6] = { 0x7, 0xA, 0x3, 0x8, 0x5, 0xC };
        SentGen g = { buf, SIZE, 0, 30.0, 0 };

        memset(buf, 200, SIZE);
        sent_g_level(&g, 1, 20);
        sent_g_frame(&g, 0x1, nib, 6, sent_t_crc(nib, 6, 1));
        sent_g_frame(&g, 0x1, nib, 6, sent_t_crc(nib, 6, 1));

        check_near("sent is not lin",
            lin_decode(buf, SIZE, 0, 100, &scratch, &lr), 0, 0);
        check_near("...and still decodes as sent", logic_decode(buf, SIZE, 0,
            100, PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_SENT,
            1, 0);
      }
    }

    // =========================== EV1527 decoder ===========================
    //
    // No checksum anywhere in the protocol, so the shape has to carry the
    // identification on its own - and it can: T comes out of one 1:31 sync
    // ratio, and then all 24 bits have to be exactly 4T long, which is 24
    // independent checks of a constant nothing else here holds.
    printf("ev1527, one frame:\n");
    {
      EvGen g;
      char lab[16];

      ev_g(&g, buf, SIZE, 4000.0, 320.0, 0);
      ev_g_frame(&g, 0x5A3C7, 1);

      int n = logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
      const Ev1527Analysis *a = ev1527_analysis();

      check_near("auto takes it", lr.proto, PROTO_EV1527, 0);
      check_near("three bytes for 24 bits", n, 3, 0);
      check_near("address", a->frame[0].addr, 0x5A3C7, 0);
      check_near("key", a->frame[0].key, 1, 0);
      check_near("T found from the sync", lr.rate, 320000, 2);
      // The bus rests low, which is upside down from everything else here
      check_near("idle is low", lr.idle_high, 0, 0);
      check_near("header", !strcmp(lr.info, "EV1527 5A3C7 K1 320us"), 1, 0);

      // MSB first into three bytes, so the hex dump reads the address and
      // the buttons straight off: 5A3C7 with key 1 is "5A 3C 71"
      check_near("hex reads the address", lr.bytes[0], 0x5A, 0);
      check_near("...", lr.bytes[1], 0x3C, 0);
      check_near("...and the key", lr.bytes[2], 0x71, 0);

      int gs, gl;

      ev1527_group_at(a, 1, &gs, &gl);
      check_near("one frame is one group", gs, 0, 0);
      check_near("...of three bytes", gl, 3, 0);

      ev1527_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("group reads both", !strcmp(lab, "5A3C7 K1"), 1, 0);
      ev1527_field_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("last byte is address and key",
          !strcmp(lab, "A3-0+K"), 1, 0);
      ev1527_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("first is the top of the address",
          !strcmp(lab, "A19-12"), 1, 0);
    }

    // A held button repeats the frame, and with no checksum in the protocol
    // that repetition is the only corroboration there is
    printf("ev1527, a button held down:\n");
    {
      EvGen g;

      // A frame is 128 T, so four of them plus the lead-in need a record of
      // some 550 T: 8 us a sample puts that inside the 24K buffer at 40
      // samples to the T, which is still far more than the split needs
      ev_g(&g, buf, SIZE, 8000.0, 320.0, 0);

      for (int i = 0; i < 4; i++)
        ev_g_frame(&g, 0x5A3C7, 8);

      logic_decode(buf, SIZE, 0, 8000, PROTO_AUTO, &scratch, &lr);
      const Ev1527Analysis *a = ev1527_analysis();

      check_near("four frames", a->frames, 4, 0);
      check_near("all agreeing", a->agree, 1, 0);
      check_near("bytes", lr.count, 12, 0);
      check_near("header", !strcmp(lr.info, "EV1527 5A3C7 K8 x4 320us"), 1, 0);
    }

    // The four buttons of a remote, one after another. They are one-hot, so
    // the key nibble walks 1, 2, 4, 8 - and a decoder that dropped or shifted
    // a bit anywhere in the 24 would show it here and nowhere else.
    printf("ev1527, the buttons in turn:\n");
    {
      const uint8_t key[4] = { 1, 2, 4, 8 };

      for (int i = 0; i < 4; i++)
      {
        EvGen g;

        ev_g(&g, buf, SIZE, 4000.0, 320.0, 0);
        ev_g_frame(&g, 0x5A3C7, key[i]);
        ev_g_frame(&g, 0x5A3C7, key[i]);

        logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
        const Ev1527Analysis *a = ev1527_analysis();

        check_near("address holds", a->frame[0].addr, 0x5A3C7, 0);
        check_near("button", a->frame[0].key, key[i], 0);
        check_near("both repeats agree", a->agree, 1, 0);
      }
    }

    // Two different buttons in one record is not a fault - a finger moved -
    // so the header stops claiming one reading and the frames keep their own
    printf("ev1527, the button changing mid-record:\n");
    {
      EvGen g;
      char lab[16];

      ev_g(&g, buf, SIZE, 4000.0, 320.0, 0);
      ev_g_frame(&g, 0x5A3C7, 1);
      ev_g_frame(&g, 0x5A3C7, 2);

      logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
      const Ev1527Analysis *a = ev1527_analysis();

      check_near("two frames", a->frames, 2, 0);
      check_near("...that do not agree", a->agree, 0, 0);
      check_near("no error is claimed", lr.errors, 0, 0);
      check_near("header", !strcmp(lr.info, "EV1527 2 frames 320us"), 1, 0);

      ev1527_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("first frame keeps its key", !strcmp(lab, "5A3C7 K1"), 1, 0);
      ev1527_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("second keeps its own", !strcmp(lab, "5A3C7 K2"), 1, 0);
    }

    // T is set by a resistor and no two remotes agree, so it is measured and
    // never assumed - and an inverting receiver module costs one more pass
    printf("ev1527 at several unit times, and inverted:\n");
    {
      const double t_us[3] = { 130.0, 320.0, 800.0 };

      for (int i = 0; i < 3; i++)
      {
        EvGen g;
        // Forty samples to the T whatever T is, so the record holds the same
        // two frames each time and only the unit time under test changes
        double per = t_us[i] * 1000.0 / 40.0;

        ev_g(&g, buf, SIZE, per, t_us[i], 0);
        ev_g_frame(&g, 0x12345, 4);
        ev_g_frame(&g, 0x12345, 4);

        logic_decode(buf, SIZE, 0, (int)per, PROTO_AUTO, &scratch, &lr);

        check_near("still EV1527", lr.proto, PROTO_EV1527, 0);
        check_near("T measured", lr.rate, t_us[i] * 1000.0, 4);
        check_near("address", ev1527_analysis()->frame[0].addr, 0x12345, 0);
      }

      EvGen g;

      ev_g(&g, buf, SIZE, 4000.0, 320.0, 1);
      ev_g_frame(&g, 0x12345, 4);

      logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
      check_near("inverted still decodes", lr.proto, PROTO_EV1527, 0);
      check_near("and says the pulses are low", lr.idle_high, 1, 0);
      check_near("address", ev1527_analysis()->frame[0].addr, 0x12345, 0);
    }

    // NEC is the one signal EV1527 could be confused with: both open with a
    // pulse and a long gap, and absolute durations cannot separate them at
    // all - T runs 100..400 us in the wild, so 31T covers 3..12 ms and NEC's
    // 9 ms leader sits inside that. The SHAPE separates them completely, in
    // two independent ways, and this is the test that says so.
    printf("ev1527 and NEC are not each other:\n");
    {
      // A NEC frame, built the way nec_decode's own test does: 9 ms leader,
      // 4.5 ms space, then 32 bits of constant 560 us mark
      const uint8_t nec[4] = { 0x00, 0xFF, 0x15, 0xEA };
      int pos = 0;

      memset(buf, 200, SIZE);

      for (int i = 0; i < 9000 && pos < SIZE; i++, pos++)
        buf[pos] = 56;                    // leader mark, active low

      for (int i = 0; i < 4500 && pos < SIZE; i++, pos++)
        buf[pos] = 200;

      for (int by = 0; by < 4; by++)
      {
        for (int b = 0; b < 8; b++)
        {
          int one = (nec[by] >> b) & 1;

          for (int i = 0; i < 560 && pos < SIZE; i++, pos++)
            buf[pos] = 56;

          for (int i = 0; i < (one ? 1690 : 560) && pos < SIZE; i++, pos++)
            buf[pos] = 200;
        }
      }

      for (int i = 0; i < 560 && pos < SIZE; i++, pos++)
        buf[pos] = 56;

      check_near("nec is not ev1527",
          ev1527_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
      check_near("...and is still nec", logic_decode(buf, SIZE, 0, 1000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_NEC, 1, 0);

      // ...and the other way: an EV1527 frame whose 31T gap is 9.9 ms, right
      // on top of a NEC leader, must not come out as NEC
      EvGen g;

      ev_g(&g, buf, SIZE, 4000.0, 320.0, 0);
      ev_g_frame(&g, 0x5A3C7, 2);
      ev_g_frame(&g, 0x5A3C7, 2);

      check_near("ev1527 is not nec",
          nec_decode(buf, SIZE, 0, 4000, &scratch, &lr), 0, 0);
    }

    printf("ev1527 rejects the rest:\n");
    {
      EvGen g;

      // It runs ahead of LIN, 1-Wire, WS2812, DHT, SENT, MIDI, servo and
      // UART, and each of those has a long dominant field its sync could be
      // mistaken for. None of them has 24 bits of constant period behind it.
      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST 9600", 20000.0);
      check_near("uart is not ev1527",
          ev1527_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not ev1527",
          ev1527_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      double duty = 0.075;
      synth(buf, SIZE, 0, fn_square, &duty, 50.0, 10000.0, 90.0, ZERO_POINT);
      check_near("servo is not ev1527",
          ev1527_decode(buf, SIZE, 0, 10000, &scratch, &lr), 0, 0);

      {
        const uint8_t d[2] = { 0x11, 0x22 };
        LinGen lg;

        lin_g(&lg, buf, SIZE, 1000.0, 19200.0, 0);
        lin_g_frame(&lg, 0x2A, d, 2, 1);
        lin_g_frame(&lg, 0x2A, d, 2, 1);
        check_near("lin is not ev1527",
            ev1527_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
      }

      {
        const uint8_t by[5] = { 45, 0, 23, 0, 68 };
        DhtGen dg = { buf, SIZE, 0 };

        memset(buf, 200, SIZE);
        dht_g_frame(&dg, by, 18000);
        check_near("dht is not ev1527",
            ev1527_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
      }

      // ...and nobody takes an EV1527 record either
      ev_g(&g, buf, SIZE, 4000.0, 320.0, 0);
      ev_g_frame(&g, 0x5A3C7, 4);
      ev_g_frame(&g, 0x5A3C7, 4);

      check_near("ev1527 is not lin",
          lin_decode(buf, SIZE, 0, 4000, &scratch, &lr), 0, 0);
      check_near("ev1527 is not 1-wire",
          onewire_decode(buf, SIZE, 0, 4000, &scratch, &lr), 0, 0);
      check_near("ev1527 is not dht",
          dht_decode(buf, SIZE, 0, 4000, &scratch, &lr), 0, 0);
      check_near("ev1527 is not sent",
          sent_decode(buf, SIZE, 0, 4000, &scratch, &lr), 0, 0);
      check_near("ev1527 is not ws2812",
          ws2812_decode(buf, SIZE, 0, 4000, &scratch, &lr), 0, 0);
    }

    // The SAME twenty-four bits, read as a PT2262. The two parts put
    // identical waveforms on the air: a PT2262 spends two pulses on each of
    // twelve tri-state symbols, which is 24 pulses of 4T, pulse for pulse
    // what a 1527 sends as 24 bits. Only the boundaries move.
    printf("ev1527 read as a pt2262:\n");
    {
      EvGen g;
      char lab[16], tri[16];

      // 0F1F01FF0011: every pair is 00, 01 or 11, so the frame is one a
      // PT2262 could have sent. Wide-then-narrow is the pair it never emits.
      ev_g(&g, buf, SIZE, 4000.0, 320.0, 0);
      ev_g_frame(&g, 0x1D350, 0xF);

      logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
      const Ev1527Analysis *a = ev1527_analysis();

      check_near("still an EV1527 record", lr.proto, PROTO_EV1527, 0);
      check_near("...and it reads as a PT2262", a->frame[0].tri_ok, 1, 0);
      check_near("every frame did", a->tri_all, 1, 0);

      ev1527_tri_text(&a->frame[0], tri, sizeof(tri));
      check_near("twelve tri-state symbols",
          !strcmp(tri, "0F1F01FF0011"), 1, 0);

      // The header keeps the reading that is certainly true and marks the
      // other; the symbols themselves go on the row under the bytes, which
      // is where somebody matching DIP switches is looking
      check_near("header", !strcmp(lr.info, "EV1527 1D350 KF 320us PT"), 1, 0);

      ev1527_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("group reads the symbols",
          !strcmp(lab, "PT 0F1F01FF0011"), 1, 0);

      // ...and a frame that could NOT have come from one says nothing about
      // it. 5A3C7 K1 holds a wide pulse followed by a narrow one, which is
      // the one pair the part cannot produce.
      ev_g(&g, buf, SIZE, 4000.0, 320.0, 0);
      ev_g_frame(&g, 0x5A3C7, 1);

      logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
      a = ev1527_analysis();

      check_near("this one is not a pt2262", a->frame[0].tri_ok, 0, 0);
      check_near("header says nothing of it",
          !strcmp(lr.info, "EV1527 5A3C7 K1 320us"), 1, 0);

      ev1527_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("the address and key stand",
          !strcmp(lab, "5A3C7 K1"), 1, 0);
    }

    // ============================ SIRC decoder ============================
    //
    // The other pulse-coded infrared family. Nothing in the frame announces
    // its length: every space inside one is 1T and the one after the last bit
    // is the rest of the 45 ms, so the decoder reads until a long gap and
    // then asks whether it stopped on 12, 15 or 20 bits.
    printf("sony sirc, a twelve-bit frame:\n");
    {
      SircGen g;
      char lab[16];

      // Command 21 to device 1: LSB first, so the command is the bottom
      // seven bits and comes off the wire before the address does
      sirc_g(&g, buf, SIZE, 2000.0, 600.0);
      sirc_g_frame(&g, 21u | (1u << 7), 12);

      int n = logic_decode(buf, SIZE, 0, 2000, PROTO_AUTO, &scratch, &lr);
      const SircAnalysis *a = sirc_analysis();

      check_near("auto takes it", lr.proto, PROTO_SIRC, 0);
      check_near("two bytes for the frame", n, 2, 0);
      check_near("command first off the wire", lr.bytes[0], 21, 0);
      check_near("...then the address", lr.bytes[1], 1, 0);
      check_near("twelve bits", a->frame[0].bits, 12, 0);
      check_near("T off the leader", lr.rate, 600000, 2);
      // A demodulator rests with no carrier, so the line rests high
      check_near("idle is high", lr.idle_high, 1, 0);
      check_near("header", !strcmp(lr.info, "SIRC12 A=1 C=21"), 1, 0);

      int gs, gl;

      sirc_group_at(a, 1, &gs, &gl);
      check_near("one frame is one group", gs, 0, 0);
      check_near("...of two bytes", gl, 2, 0);

      sirc_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("group reads both", !strcmp(lab, "A1 C21"), 1, 0);
      sirc_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("first byte is the command", !strcmp(lab, "CMD"), 1, 0);
      sirc_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("second is the address", !strcmp(lab, "ADDR"), 1, 0);
    }

    // The other two lengths. The address field widens on the fifteen-bit
    // frame and an extended byte appears on the twenty-bit one, and only the
    // bit count says which - so reading 15 bits as 12 puts the address out.
    printf("sirc at fifteen and twenty bits:\n");
    {
      SircGen g;
      char lab[16];

      sirc_g(&g, buf, SIZE, 2500.0, 600.0);
      sirc_g_frame(&g, 21u | (138u << 7), 15);

      logic_decode(buf, SIZE, 0, 2500, PROTO_AUTO, &scratch, &lr);
      const SircAnalysis *a = sirc_analysis();

      check_near("fifteen bits", a->frame[0].bits, 15, 0);
      check_near("command", a->frame[0].cmd, 21, 0);
      check_near("eight bits of address", a->frame[0].addr, 138, 0);
      check_near("header", !strcmp(lr.info, "SIRC15 A=138 C=21"), 1, 0);

      sirc_g(&g, buf, SIZE, 3000.0, 600.0);
      sirc_g_frame(&g, 21u | (26u << 7) | (144u << 12), 20);

      int n = logic_decode(buf, SIZE, 0, 3000, PROTO_AUTO, &scratch, &lr);

      a = sirc_analysis();

      check_near("twenty bits", a->frame[0].bits, 20, 0);
      check_near("three bytes for it", n, 3, 0);
      check_near("command", a->frame[0].cmd, 21, 0);
      check_near("five bits of address again", a->frame[0].addr, 26, 0);
      check_near("...and the extended byte", a->frame[0].ext, 144, 0);
      check_near("header", !strcmp(lr.info, "SIRC20 A=26 C=21 E=144"), 1, 0);

      sirc_field_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("third byte is the extension", !strcmp(lab, "EXT"), 1, 0);
      sirc_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("group reads all three",
          !strcmp(lab, "A26 C21 E144"), 1, 0);
    }

    // A key held down repeats the frame every 45 ms, and a remote sends it at
    // least three times whatever the key does - so repeats agreeing is the
    // only corroboration a protocol with no checksum has to offer
    printf("sirc, a key held down:\n");
    {
      SircGen g;

      sirc_g(&g, buf, SIZE, 5000.0, 600.0);

      for (int i = 0; i < 3; i++)
        sirc_g_frame(&g, 21u | (1u << 7), 12);

      logic_decode(buf, SIZE, 0, 5000, PROTO_AUTO, &scratch, &lr);
      const SircAnalysis *a = sirc_analysis();

      check_near("three frames", a->frames, 3, 0);
      check_near("all agreeing", a->agree, 1, 0);
      check_near("bytes", lr.count, 6, 0);
      check_near("header", !strcmp(lr.info, "SIRC12 A=1 C=21 x3"), 1, 0);
    }

    // The record is not long enough for the frame at every timebase - a
    // twenty-bit frame is 33 ms - and the command is the BOTTOM seven bits,
    // so a frame cut in half still says which key was pressed. That is worth
    // reporting rather than refusing, exactly as it is for NEC.
    printf("sirc, a frame the record cut short:\n");
    {
      SircGen g;

      sirc_g(&g, buf, SIZE, 1000.0, 600.0);
      sirc_g_frame(&g, 21u | (26u << 7) | (144u << 12), 20);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      const SircAnalysis *a = sirc_analysis();

      check_near("still SIRC", lr.proto, PROTO_SIRC, 0);
      check_near("the command, and only it", n, 1, 0);
      check_near("...and it is right", lr.bytes[0], 21, 0);
      check_near("marked cut", a->frame[0].cut, 1, 0);
      check_near("...on the record too", lr.overrun, 1, 0);
      check_near("header says so", !strncmp(lr.info, "SIRC C=21 ", 10), 1, 0);
    }

    // Probe the emitter's LED driver instead of the receiver and everything
    // inverts, purpose included. The decoder asks the record which level is
    // rest rather than assuming, so the reading comes out the same.
    printf("sirc off the emitter, inverted:\n");
    {
      SircGen g;

      sirc_g(&g, buf, SIZE, 2000.0, 600.0);
      sirc_g_frame(&g, 21u | (1u << 7), 12);

      for (int i = 0; i < SIZE; i++)
        buf[i] = (buf[i] > 128) ? 56 : 200;

      logic_decode(buf, SIZE, 0, 2000, PROTO_AUTO, &scratch, &lr);

      check_near("inverted still decodes", lr.proto, PROTO_SIRC, 0);
      check_near("command", lr.bytes[0], 21, 0);
      check_near("address", lr.bytes[1], 1, 0);
      check_near("idle is low now", lr.idle_high, 0, 0);
    }

    // The one that matters. RC6 is the other infrared leader in this range
    // and the two are close enough that "a long mark, then bits" reads one as
    // the other. What separates them is the leader's own ratio - 4:1 here
    // against RC6's 3:1 - and behind it, that every space in a SIRC frame is
    // one unit where a bi-phase code's vary with the data.
    printf("sirc and the philips remotes are not each other:\n");
    {
      SircGen sg;
      IrGen ig;

      ir_g(&ig, buf, SIZE, 2000.0, 889.0);
      rc6_g_frame(&ig, (1u << 20) | (0u << 17) | (1u << 16) | (0x12 << 8) | 0x34,
          21, 4);

      check_near("rc6 is not sirc",
          sirc_decode(buf, SIZE, 0, 2000, &scratch, &lr), 0, 0);
      check_near("...and auto still says RC5",
          logic_decode(buf, SIZE, 0, 2000, PROTO_AUTO, &scratch, &lr) > 0 &&
          lr.proto == PROTO_RC5, 1, 0);

      ir_g(&ig, buf, SIZE, 2000.0, 1778.0);
      rc5_g_frame(&ig, (3u << 12) | (1u << 11) | (5u << 6) | 35u, 14);

      check_near("rc5 is not sirc",
          sirc_decode(buf, SIZE, 0, 2000, &scratch, &lr), 0, 0);

      // ...and the other way. A unit that has drifted to 700 us is what
      // offers the bi-phase reader a 1.4 ms bit time and a record of runs
      // that all measure half a bit or a whole one, which is why this decoder
      // has to come first in the cascade rather than after.
      sirc_g(&sg, buf, SIZE, 2000.0, 700.0);
      sirc_g_frame(&sg, 21u | (1u << 7), 12);

      check_near("a drifted sirc is still taken by sirc",
          logic_decode(buf, SIZE, 0, 2000, PROTO_AUTO, &scratch, &lr) > 0 &&
          lr.proto == PROTO_SIRC, 1, 0);

      // NEC is not a risk in either direction: its leader is 9 ms, which
      // would put T at 2.25 ms - past the top of the window by a factor of
      // two and a half - and a SIRC leader read as NEC's is out by four
      sirc_g(&sg, buf, SIZE, 2000.0, 600.0);
      sirc_g_frame(&sg, 21u | (1u << 7), 12);

      check_near("sirc is not nec",
          nec_decode(buf, SIZE, 0, 2000, &scratch, &lr), 0, 0);
      check_near("sirc is not ev1527",
          ev1527_decode(buf, SIZE, 0, 2000, &scratch, &lr), 0, 0);
      check_near("sirc is not servo",
          servo_decode(buf, SIZE, 0, 2000, &scratch, &lr), 0, 0);
    }

    printf("sirc rejects the rest:\n");
    {
      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST 9600", 20000.0);
      check_near("uart is not sirc",
          sirc_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not sirc",
          sirc_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      // A frame that stops at thirteen bits is a signal that started like
      // SIRC and then was not one, and nothing is reported for it
      {
        SircGen g;

        sirc_g(&g, buf, SIZE, 2000.0, 600.0);
        sirc_g_frame(&g, 21u | (1u << 7), 13);

        check_near("thirteen bits is not a frame",
            sirc_decode(buf, SIZE, 0, 2000, &scratch, &lr), 0, 0);
      }
    }

    // ============================ DShot decoder ===========================
    //
    // Sixteen bits, one number, and a CRC-4 that has to agree before anything
    // is reported. No sync field: what delimits a frame is the idle on both
    // sides of it, and that turns out to be what tells it from WS2812.
    printf("dshot600, mid throttle:\n");
    {
      DsGen g;
      char lab[16];

      ds_g(&g, buf, SIZE, 20.0, 600.0, 0);
      ds_g_frame(&g, 1047, 0, 0);

      int n = logic_decode(buf, SIZE, 0, 20, PROTO_AUTO, &scratch, &lr);
      const DshotAnalysis *a = dshot_analysis();

      check_near("auto takes it", lr.proto, PROTO_DSHOT, 0);
      check_near("two bytes for 16 bits", n, 2, 0);
      check_near("one frame", a->frames, 1, 0);
      check_near("throttle", a->frame[0].value, 1047, 0);
      check_near("no telemetry asked", a->frame[0].telem, 0, 0);
      check_near("crc checks", a->frame[0].crc_ok, 1, 0);
      check_near("not the inverted flavour", a->bidir, 0, 0);
      check_near("rate", lr.rate, 600000, 3);
      check_near("idle is low", lr.idle_high, 0, 0);
      // 1047 is 999 above the bottom of the 48..2047 band, so just under half
      check_near("header", !strcmp(lr.info, "DShot600 1047 49%"), 1, 0);

      int gs, gl;

      dshot_group_at(a, 1, &gs, &gl);
      check_near("one frame is one group", gs, 0, 0);
      check_near("...of two bytes", gl, 2, 0);

      dshot_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("group reads the throttle", !strcmp(lab, "1047 49%"), 1, 0);
      dshot_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("first byte is throttle", !strcmp(lab, "THR10-3"), 1, 0);
      dshot_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("second carries the crc", !strcmp(lab, "CRC"), 1, 0);
    }

    // The name IS the rate, and the rate is measured off the frame rather
    // than assumed: the four that exist all have to come out right
    printf("dshot at all four rates:\n");
    {
      const double kbit[4] = { 150.0, 300.0, 600.0, 1200.0 };
      const char *name[4] =
          { "DShot150", "DShot300", "DShot600", "DShot1200" };

      for (int i = 0; i < 4; i++)
      {
        DsGen g;
        // Twenty samples to the bit whatever the rate, so only the rate under
        // test changes and the record holds the same three frames
        double per = 1e6 / kbit[i] / 20.0;

        ds_g(&g, buf, SIZE, per, kbit[i], 0);
        ds_g_frame(&g, 500, 0, 0);
        ds_g_frame(&g, 500, 0, 0);
        ds_g_frame(&g, 500, 0, 0);

        logic_decode(buf, SIZE, 0, (int)per, PROTO_AUTO, &scratch, &lr);

        check_near("still DShot", lr.proto, PROTO_DSHOT, 0);
        check_near("rate", lr.rate, kbit[i] * 1000.0, 4);
        check_near("three frames", dshot_analysis()->frames, 3, 0);
        check_near("named by its rate",
            NULL != strstr(lr.info, name[i]), 1, 0);
      }
    }

    printf("dshot, the ends of the range and the commands:\n");
    {
      struct { uint16_t v; const char *reads; } t[] =
      {
        {    0, "disarm" },     // nothing armed, and not 0 % throttle
        {    1, "BEEP1" },      // 1..47 are commands and not throttle at all
        {   12, "SAVE" },
        {   21, "DIR REV" },
        {   17, "CMD 17" },     // a command with no name is still a command
        {   48, "48 0%" },      // the bottom of the throttle band IS zero
        { 2047, "2047 100%" },
      };

      for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++)
      {
        DsGen g;
        char lab[16];

        ds_g(&g, buf, SIZE, 20.0, 600.0, 0);
        ds_g_frame(&g, t[i].v, 0, 0);

        logic_decode(buf, SIZE, 0, 20, PROTO_AUTO, &scratch, &lr);
        const DshotAnalysis *a = dshot_analysis();

        check_near("value", a->frame[0].value, t[i].v, 0);
        dshot_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
        check_near("reads", !strcmp(lab, t[i].reads), 1, 0);
      }
    }

    printf("dshot, telemetry requested:\n");
    {
      DsGen g;
      char lab[16];

      ds_g(&g, buf, SIZE, 20.0, 600.0, 0);
      ds_g_frame(&g, 1000, 1, 0);

      logic_decode(buf, SIZE, 0, 20, PROTO_AUTO, &scratch, &lr);
      const DshotAnalysis *a = dshot_analysis();

      check_near("flag read", a->frame[0].telem, 1, 0);
      check_near("crc still checks", a->frame[0].crc_ok, 1, 0);
      dshot_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("and shown", !strcmp(lab, "1000 47% tel"), 1, 0);
    }

    // Bidirectional DShot inverts the line AND the CRC. Both have to be
    // spotted, and the inverted CRC is what says which flavour it is - an
    // inverting buffer on a normal link would invert the line and nothing
    // else.
    printf("dshot bidirectional:\n");
    {
      DsGen g;

      ds_g(&g, buf, SIZE, 20.0, 600.0, 1);
      ds_g_frame(&g, 1500, 0, 1);
      ds_g_frame(&g, 1500, 0, 1);

      logic_decode(buf, SIZE, 0, 20, PROTO_AUTO, &scratch, &lr);
      const DshotAnalysis *a = dshot_analysis();

      check_near("still DShot", lr.proto, PROTO_DSHOT, 0);
      check_near("throttle", a->frame[0].value, 1500, 0);
      check_near("crc checks", a->frame[0].crc_ok, 1, 0);
      check_near("the inverted one", a->bidir, 1, 0);
      check_near("line is inverted too", lr.idle_high, 1, 0);
      check_near("header says so",
          NULL != strstr(lr.info, "DShot600bd"), 1, 0);
    }

    printf("dshot, a frame with a broken crc:\n");
    {
      DsGen g;
      uint16_t d = (uint16_t)(1000 << 1);
      uint16_t frame = (uint16_t)((d << 4) | ((ds_g_crc(d, 0) + 1) & 0x0F));

      ds_g(&g, buf, SIZE, 20.0, 600.0, 0);

      for (int i = 15; i >= 0; i--)
      {
        int one = (frame >> i) & 1;

        ds_g_run(&g, 1, one ? 0.75 : 0.375);
        ds_g_run(&g, 0, one ? 0.25 : 0.625);
      }

      ds_g_run(&g, 0, 30);

      // Nothing is reported without its CRC behind it: a throttle command
      // read wrongly is a motor told the wrong thing, and a caveat under it
      // is not good enough
      check_near("refused outright",
          dshot_decode(buf, SIZE, 0, 20, &scratch, &lr), 0, 0);
    }

    // The one that matters. WS2812 is constant-period and duty-encoded at
    // very nearly DShot600's rate, and a naive decoder reads one as the
    // other. Three things separate them and all three are tested.
    printf("dshot and WS2812 are not each other:\n");
    {
      // A strip: 1.25 us bit, 0.35 us high for a zero and 0.7 for a one -
      // 28 % and 56 %, where DShot uses 37.5 % and 75 %
      const uint8_t px[9] =
          { 0x11, 0x22, 0x33, 0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x00 };
      int pos = 0;
      double sps = 1250.0 / 20.0;   // 20 ns a sample: 62.5 samples per bit

      memset(buf, 56, SIZE);
      pos = (int)(50000.0 / 20.0);  // the reset gap in front of it

      for (int by = 0; by < 9; by++)
      {
        for (int b = 7; b >= 0; b--)
        {
          int one = (px[by] >> b) & 1;
          int hi = (int)(sps * (one ? 0.56 : 0.28) + 0.5);
          int lo = (int)(sps + 0.5) - hi;

          for (int i = 0; i < hi && pos < SIZE; i++, pos++)
            buf[pos] = 200;

          for (int i = 0; i < lo && pos < SIZE; i++, pos++)
            buf[pos] = 56;
        }
      }

      check_near("ws2812 is not dshot",
          dshot_decode(buf, SIZE, 0, 20, &scratch, &lr), 0, 0);
      check_near("...and is still ws2812", logic_decode(buf, SIZE, 0, 20,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_WS2812, 1, 0);

      // A black strip sends nothing but zeros, which is the same waveform a
      // disarmed ESC sends. Only the frame boundary separates those: after
      // sixteen WS2812 bits comes the seventeenth, not an idle line.
      memset(buf, 56, SIZE);
      pos = (int)(50000.0 / 20.0);

      for (int b = 0; b < 24 * 4; b++)
      {
        int hi = (int)(sps * 0.28 + 0.5);
        int lo = (int)(sps + 0.5) - hi;

        for (int i = 0; i < hi && pos < SIZE; i++, pos++)
          buf[pos] = 200;

        for (int i = 0; i < lo && pos < SIZE; i++, pos++)
          buf[pos] = 56;
      }

      check_near("a black strip is not a disarmed esc",
          dshot_decode(buf, SIZE, 0, 20, &scratch, &lr), 0, 0);

      // ...and the other way round: a DShot record must not be read as a
      // strip of LEDs, which is why DShot runs first
      DsGen g;

      ds_g(&g, buf, SIZE, 20.0, 600.0, 0);
      ds_g_frame(&g, 1047, 0, 0);
      ds_g_frame(&g, 1047, 0, 0);

      logic_decode(buf, SIZE, 0, 20, PROTO_AUTO, &scratch, &lr);
      check_near("dshot is not read as ws2812", lr.proto, PROTO_DSHOT, 0);
    }

    printf("dshot rejects the rest:\n");
    {
      synth_uart(buf, SIZE, 20.0, 1000000.0, "MILKV-UART", 4000.0);
      check_near("uart is not dshot",
          dshot_decode(buf, SIZE, 0, 20, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not dshot",
          dshot_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      double duty = 0.075;
      synth(buf, SIZE, 0, fn_square, &duty, 50.0, 10000.0, 90.0, ZERO_POINT);
      check_near("servo is not dshot",
          dshot_decode(buf, SIZE, 0, 10000, &scratch, &lr), 0, 0);

      // A 37.5 % square wave is every DShot zero bit in a row, forever, and
      // has no frame boundary anywhere in it
      duty = 0.375;
      synth(buf, SIZE, 0, fn_square, &duty, 600000.0, 20.0, 90.0, ZERO_POINT);
      check_near("a 37.5% square wave is not dshot",
          dshot_decode(buf, SIZE, 0, 20, &scratch, &lr), 0, 0);
    }

    // ====================== SPI on one probe ==============================
    //
    // Not a decode. SPI is a clocked protocol and this instrument has one
    // probe, so the timing is an ASSUMPTION and the byte boundary is one of
    // eight guesses. What is tested here is that the assumptions are stated,
    // that being TOLD the clock beats guessing it, and above all that this
    // decoder never takes a record in auto mode.
    printf("spi, told the clock:\n");
    {
      // The first byte's top bit is a zero on purpose. The line idles HIGH,
      // so a first bit of one would be inside the same run as the idle - and
      // that run is cut by the start of the record, which makes its length a
      // fact about the buffer rather than about the signal. Where the idle
      // and the first bit share a level, that bit is not recoverable at all,
      // and no decoder can pretend otherwise.
      const uint8_t msg[6] = { 0x03, 0x00, 0x10, 0x00, 0x5A, 0x7E };
      SpiGen g;

      spi_g(&g, buf, SIZE, 1000.0, 20000.0);   // 20 kHz, 50 samples a bit

      for (int i = 0; i < 6; i++)
        spi_g_byte(&g, msg[i], 0);             // back to back, no gaps

      spi_g_level(&g, 1, 40);

      spi_decode_set_clock(20000);
      spi_decode_set_order(0);

      int n = spi_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      const SpiAnalysis *a = spi_analysis();

      check_near("bytes come out", n >= 6, 1, 0);
      check_near("the rate is the one it was told", lr.rate, 20000, 2);
      check_near("...and it says it was told", a->told, 1, 0);
      check_near("msb first", a->msb_first, 1, 0);

      // The record opens on the idle line, so the first edge is the first
      // bit of the first byte and the reconstruction starts there
      check_near("first byte", lr.bytes[0], 0x03, 0);
      check_near("second", lr.bytes[1], 0x00, 0);
      check_near("fifth", lr.bytes[4], 0x5A, 0);
      check_near("sixth", lr.bytes[5], 0x7E, 0);
    }

    // The one thing that must never happen: a data line without its clock
    // quantises into bits just as happily as anything else does, so this
    // decoder is reported ambiguous EVERY time and auto mode always passes
    // over it. It exists to be asked for by name.
    printf("spi is never chosen by itself:\n");
    {
      const uint8_t msg[4] = { 0x03, 0x00, 0x10, 0x00 };
      SpiGen g;

      spi_g(&g, buf, SIZE, 1000.0, 20000.0);

      for (int i = 0; i < 4; i++)
        spi_g_byte(&g, msg[i], 4);

      spi_g_level(&g, 1, 40);

      spi_decode_set_clock(20000);

      check_near("it decodes when asked",
          spi_decode(buf, SIZE, 0, 1000, &scratch, &lr) > 0, 1, 0);
      check_near("and says it is ambiguous", lr.ambiguous, 1, 0);
      check_near("auto never takes it", logic_decode(buf, SIZE, 0, 1000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_SPI, 0, 0);
      check_near("named, it comes back", logic_decode(buf, SIZE, 0, 1000,
          PROTO_SPI, &scratch, &lr) > 0 && lr.proto == PROTO_SPI, 1, 0);
    }

    // A bit-banged master pauses between bytes, and a pause is a byte
    // boundary for free - the one piece of EVIDENCE in this whole decoder
    // rather than a heuristic. The header has to say which of the two it was.
    printf("spi, byte boundaries from the gaps:\n");
    {
      // Every one of these has its top bit clear on purpose. The pause is the
      // line at REST - high here - so a byte starting with a one merges into
      // the pause in front of it, and where the pause ends is then not
      // observable at all. No decoder recovers that byte, and this one does
      // not pretend to: it is the same limit as the very first bit of the
      // record, for the same reason.
      const uint8_t msg[5] = { 0x12, 0x34, 0x56, 0x78, 0x5A };
      SpiGen g;

      spi_g(&g, buf, SIZE, 1000.0, 20000.0);

      for (int i = 0; i < 5; i++)
        spi_g_byte(&g, msg[i], 16);            // a clear pause after each

      spi_g_level(&g, 1, 40);

      spi_decode_set_clock(20000);
      spi_decode_set_order(1);

      int n = spi_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      const SpiAnalysis *a = spi_analysis();

      check_near("gaps were found", a->anchors >= 4, 1, 0);
      check_near("...and pinned the phase", a->pinned, 1, 0);
      check_near("the header says so",
          NULL != strstr(lr.info, " g"), 1, 0);
      check_near("bytes", n > 0, 1, 0);

      // With the boundary pinned the bytes are the bytes, not a rotation of
      // them - the gap makes this the one case that is not a guess
      // The pause is not skipped, and must not be: with no chip select on
      // the probe there is nothing that says a run of ones is idle rather
      // than 0xFF on the wire. So the gaps come out as filler between the
      // message bytes - two bytes of 0xFF to each sixteen-bit pause - and
      // the message is every third byte.
      int hit = 0;

      for (int i = 0; i + 12 < lr.count; i++)
      {
        if (0x12 == lr.bytes[i] && 0x34 == lr.bytes[i + 3] &&
            0x56 == lr.bytes[i + 6] && 0x78 == lr.bytes[i + 9] &&
            0x5A == lr.bytes[i + 12])
          hit = 1;
      }

      check_near("the message is in there whole", hit, 1, 0);
      check_near("and the pause came out as filler",
          0xFF == lr.bytes[1] && 0xFF == lr.bytes[2], 1, 0);
    }

    // Without a gap anywhere, the phase is scored rather than known, and the
    // header must NOT claim otherwise
    // A flash transaction, the way one actually goes past: chip select drops,
    // an opcode, three bytes of address, then the data - and a pause before
    // the next one. The pauses are what make the opcode readable at all.
    printf("spi, a flash transaction:\n");
    {
      const uint8_t tx[8] = { 0x03, 0x00, 0x10, 0x00, 0xDE, 0xAD, 0xBE, 0xEF };
      SpiGen g;
      char lab[16];

      spi_g(&g, buf, SIZE, 1000.0, 20000.0);

      for (int rep = 0; rep < 2; rep++)
      {
        for (int i = 0; i < 8; i++)
          spi_g_byte(&g, tx[i], 0);       // the transaction runs back to back

        spi_g_level(&g, 1, 20);           // ...and the bus rests between them
      }

      spi_g_level(&g, 1, 40);

      spi_decode_set_clock(20000);
      spi_decode_set_order(0);
      spi_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      const SpiAnalysis *a = spi_analysis();

      check_near("the pauses pinned it", a->pinned, 1, 0);
      check_near("the opcode was recognised", a->cmds >= 1, 1, 0);
      check_near("...and the order settled with it", a->msb_first, 1, 0);

      // Find the byte the pauses called a command
      int c = -1;

      for (int i = 0; i < lr.count; i++)
      {
        if (SPI_R_CMD == a->role[i])
          c = i;
      }

      check_near("a command was marked", c >= 0, 1, 0);

      if (c >= 0)
      {
        check_near("it is the READ opcode", lr.bytes[c], 0x03, 0);
        spi_byte_label(a, c, lr.bytes[c], lab, sizeof(lab));
        check_near("named", !strcmp(lab, "READ"), 1, 0);
        spi_field_label(a, c, lr.bytes[c], lab, sizeof(lab));
        check_near("...as a command", !strcmp(lab, "CMD"), 1, 0);

        // The three bytes after it are one number and are shown as one
        int gs, gl;

        spi_group_at(a, c + 2, &gs, &gl);
        check_near("address groups", gs, c + 1, 0);
        check_near("...as three bytes", gl, 3, 0);

        spi_byte_label(a, c + 3, lr.bytes[c + 3], lab, sizeof(lab));
        check_near("address assembled", !strcmp(lab, "A=001000"), 1, 0);
        spi_field_label(a, c + 1, lr.bytes[c + 1], lab, sizeof(lab));
        check_near("its bytes numbered", !strcmp(lab, "A2"), 1, 0);
      }

      // The payload is not a command and is not pretended to be
      if (c >= 0 && c + 4 < lr.count)
        check_near("payload has no role", a->role[c + 4], SPI_R_NONE, 0);
    }

    // ...and where nothing pinned the byte boundary, NOTHING is named a
    // command. The phase is already a preference; hanging "READ" off it would
    // make a reading look confirmed by the very thing it was chosen for.
    printf("spi names no commands it cannot place:\n");
    {
      const uint8_t tx[8] = { 0x03, 0x00, 0x10, 0x00, 0xDE, 0xAD, 0xBE, 0xEF };
      SpiGen g;

      spi_g(&g, buf, SIZE, 1000.0, 20000.0);

      for (int rep = 0; rep < 4; rep++)
        for (int i = 0; i < 8; i++)
          spi_g_byte(&g, tx[i], 0);       // no pauses anywhere

      spi_g_level(&g, 1, 40);

      spi_decode_set_clock(20000);
      spi_decode_set_order(1);
      spi_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      const SpiAnalysis *a = spi_analysis();

      check_near("nothing pinned it", a->pinned, 0, 0);
      check_near("so nothing is called a command", a->cmds, 0, 0);

      int roles = 0;

      for (int i = 0; i < lr.count; i++)
      {
        if (SPI_R_NONE != a->role[i])
          roles++;
      }

      check_near("and no byte carries a role", roles, 0, 0);
      check_near("the header says it was scored",
          NULL != strstr(lr.info, " ?"), 1, 0);
    }

    printf("spi, no gaps to go on:\n");
    {
      const uint8_t msg[6] = { 'H', 'e', 'l', 'l', 'o', '!' };
      SpiGen g;

      spi_g(&g, buf, SIZE, 1000.0, 20000.0);

      for (int i = 0; i < 6; i++)
        spi_g_byte(&g, msg[i], 0);

      spi_g_level(&g, 1, 40);

      spi_decode_set_clock(20000);
      spi_decode_set_order(1);
      spi_decode(buf, SIZE, 0, 1000, &scratch, &lr);

      check_near("nothing pinned it", spi_analysis()->pinned, 0, 0);
      check_near("and the header admits it",
          NULL != strstr(lr.info, " ?"), 1, 0);
      // ...and the doubt is in the NAME, every time, whatever else the line
      // says: this is a reconstruction of a bus the instrument cannot see
      check_near("the name is marked",
          0 == strncmp(lr.info, "SPI!", 4), 1, 0);
      // ASCII is what the scoring has to work with, and here it is enough
      check_near("text found anyway",
          NULL != memchr(lr.bytes, 'H', (size_t)lr.count), 1, 0);
    }

    // Being told the clock is the point of the two-pass workflow. Guessing it
    // from the data works when the data has a single-bit run in it somewhere
    // and fails when it does not - a stream of 0x0F has runs of four bits and
    // nothing shorter, so the guess comes out four times too slow and every
    // byte with it.
    printf("spi, told beats guessed:\n");
    {
      SpiGen g;

      spi_g(&g, buf, SIZE, 1000.0, 20000.0);

      for (int i = 0; i < 12; i++)
        spi_g_byte(&g, 0x0F, 0);

      spi_g_level(&g, 1, 40);

      spi_decode_set_order(1);

      // Told: the bit time is a fact, and 96 bits come out of the record.
      //
      // The byte VALUE is deliberately not asserted, because it is genuinely
      // not determined: 0F 0F 0F... and 3C 3C 3C... and 78 78 78... are the
      // same bits cut in different places, and nothing in a data line
      // without its clock says which cut is the right one. The scoring
      // prefers the printable one, and that is a preference and not
      // knowledge - which is what "p3 ?" in the header is there to admit.
      spi_decode_set_clock(20000);
      spi_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      check_near("told: rate", lr.rate, 20000, 2);
      check_near("told: says so", spi_analysis()->told, 1, 0);
      check_near("told: bits", spi_analysis()->bits, 92, 3);
      check_near("told: nothing pinned the phase",
          spi_analysis()->pinned, 0, 0);

      // ...and whatever cut it chose, the stream is one value repeating,
      // which is what four-bit runs at the right rate have to come out as
      int same = 1;

      for (int i = 1; i < lr.count; i++)
      {
        if (lr.bytes[i] != lr.bytes[0])
          same = 0;
      }

      check_near("told: one value, repeating", same, 1, 0);
      check_near("told: eleven bytes of it", lr.count, 11, 10);

      // Guessed: every run is four bits long, so the shortest run IS four
      // bits and the estimate is a quarter of the truth. Nothing in the
      // record says otherwise, and the header's '~' is the only warning
      // there can be.
      spi_decode_set_clock(0);
      spi_decode(buf, SIZE, 0, 1000, &scratch, &lr);
      // Every run in this record is four bits long, so the shortest run IS
      // four bits and the estimate comes out a quarter of the truth. There
      // is nothing in the data that says otherwise, which is the whole
      // argument for measuring the clock on SCK first.
      check_near("guessed: comes out four times slow", lr.rate, 5000, 5);
      check_near("guessed: a quarter of the bits, too",
          spi_analysis()->bits, 23, 10);
      check_near("guessed: and marks the rate as its own",
          NULL != strstr(lr.info, "~"), 1, 0);
      check_near("guessed: says it was not told", spi_analysis()->told, 0, 0);

      spi_decode_set_clock(0);
      spi_decode_set_order(0);
    }

    printf("spi refuses what it cannot quantise:\n");
    {
      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      spi_decode_set_clock(0);
      // A thresholded sine IS a square wave and a square wave IS a bit
      // stream, so this one does decode - and is ambiguous, like everything
      // else this decoder returns
      check_near("a sine quantises like anything else",
          spi_decode(buf, SIZE, 0, 1000, &scratch, &lr) > 0 && lr.ambiguous,
          1, 0);

      // Flat line: no edges, nothing to quantise, nothing to say
      memset(buf, 128, SIZE);
      check_near("a flat line is refused",
          spi_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      // Told a clock so slow that the record holds under two bytes of it:
      // sixteen bits is the least that is a reading of anything
      SpiGen g;

      spi_g(&g, buf, SIZE, 1000.0, 20000.0);

      for (int i = 0; i < 6; i++)
        spi_g_byte(&g, 0xA5, 0);

      spi_decode_set_clock(200);
      check_near("a bit that wide is refused",
          spi_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
      spi_decode_set_clock(0);
    }

    // ========================= Manchester / bi-phase ======================
    //
    // A line code and not a protocol - the brick RC5, DALI, EM4100 tags and
    // most 433 MHz weather sensors are built out of. The structural claim is
    // that every run is half a bit or a whole one and nothing else, and that
    // is checked before a single bit is read.
    printf("manchester, an RC5-shaped frame:\n");
    {
      // 14 bits, 1.778 ms each: start, field, toggle, 5 of address, 6 of
      // command. The value has adjacent equal bits in it, so the record
      // contains half-bit runs and the rate can be found either way.
      ManGen g;
      char lab[16];

      man_g(&g, buf, SIZE, 20000.0, 562.0);
      man_g_frame(&g, 0x300C, 14);

      manchester_decode_set_rate(562);
      manchester_decode_set_polarity(0);

      int n = logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      const ManAnalysis *a = manchester_analysis();

      check_near("auto takes it", lr.proto, PROTO_MANCH, 0);
      check_near("one frame", a->frames, 1, 0);
      check_near("fourteen bits", a->frame[0].bits, 14, 0);
      check_near("...and their value", a->frame[0].value, 0x300C, 0);
      check_near("rate", lr.rate, 562, 3);
      check_near("told", a->told, 1, 0);
      check_near("two bytes for fourteen bits", n, 2, 0);
      check_near("header", !strcmp(lr.info, "MAN 562 14b 300C"), 1, 0);

      // The frame is one number, so it is one thing on the screen - the two
      // bytes it packed into are where eight bits happened to land
      int gs, gl;

      manchester_group_at(a, 1, &gs, &gl);
      check_near("group start", gs, 0, 0);
      check_near("...of two bytes", gl, 2, 0);

      manchester_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("reads bits and value", !strcmp(lab, "14b 300C"), 1, 0);
      manchester_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("first byte\'s bits", !strcmp(lab, "b13-6"), 1, 0);
      manchester_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("...and the last six", !strcmp(lab, "b5-0"), 1, 0);
    }

    // Which way round a bit reads is NOT in the waveform: G.E. Thomas and
    // IEEE 802.3 are exact inverses of each other, RC5 uses one and DALI the
    // other. It is a setting, and flipping it flips every bit.
    printf("manchester, the other convention:\n");
    {
      ManGen g;

      man_g(&g, buf, SIZE, 20000.0, 1200.0);
      man_g_frame(&g, 0x5A5A5, 19);      // a DALI-shaped forward frame

      manchester_decode_set_rate(1200);
      manchester_decode_set_polarity(0);
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      check_near("as generated", manchester_analysis()->frame[0].value,
          0x5A5A5, 0);
      check_near("nineteen bits", manchester_analysis()->frame[0].bits, 19, 0);

      manchester_decode_set_polarity(1);
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      check_near("inverted reads the complement",
          manchester_analysis()->frame[0].value, (~0x5A5A5u) & 0x7FFFFu, 0);
      check_near("and says which it used",
          NULL != strstr(lr.info, "inv"), 1, 0);

      manchester_decode_set_polarity(0);
    }

    // Auto: the shortest run is half a bit only if two adjacent bits are ever
    // equal. When they are, the two populations settle the rate outright.
    printf("manchester, rate off the record:\n");
    {
      ManGen g;

      man_g(&g, buf, SIZE, 20000.0, 1200.0);
      man_g_frame(&g, 0x3C0F3, 19);      // plenty of adjacent equal bits

      manchester_decode_set_rate(0);
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      const ManAnalysis *a = manchester_analysis();

      check_near("found it", lr.rate, 1200, 5);
      check_near("says it was not told", a->told, 0, 0);
      check_near("value", a->frame[0].value, 0x3C0F3, 0);
      check_near("both run lengths were there, so it is sure",
          lr.ambiguous, 0, 0);
      check_near("the rate is marked as its own",
          NULL != strstr(lr.info, "~"), 1, 0);
    }

    // ...and when they are never equal, there is no half-bit run anywhere and
    // the estimate lands at twice the bit period with nothing in the data to
    // contradict it. That is the whole argument for the rate being a setting.
    printf("manchester, the factor of two it cannot close:\n");
    {
      ManGen g;

      // Strictly alternating, AND opening with a bit whose first half is at
      // the resting level so it merges into the idle: then there is not one
      // half-bit run anywhere in the record, and the shortest run is a whole
      // bit. The estimate lands at twice the bit period and the data cannot
      // say otherwise.
      man_g(&g, buf, SIZE, 20000.0, 1200.0);
      man_g_frame(&g, 0x15555, 18);

      manchester_decode_set_rate(0);
      int n = logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);

      check_near("auto does not claim it", n > 0 && lr.proto == PROTO_MANCH,
          0, 0);

      manchester_decode(buf, SIZE, 0, 20000, &scratch, &lr);
      check_near("named, it decodes", lr.count > 0, 1, 0);
      check_near("...and says it is unsure", lr.ambiguous, 1, 0);

      // Told the rate, the bits come out right - which is what being told
      // buys. It does NOT buy certainty: a record of strictly alternating
      // bits is a square wave, and a square wave is also a stream of 0x55
      // out of a UART and a clock, with nothing in the samples to choose
      // between them. So it stays out of auto mode either way, and the
      // difference is that by name it now reads correctly.
      manchester_decode_set_rate(1200);
      manchester_decode(buf, SIZE, 0, 20000, &scratch, &lr);
      check_near("told, the value is right",
          manchester_analysis()->frame[0].value, 0x15555, 0);
      check_near("...and the rate is the one given", lr.rate, 1200, 0);
      check_near("still not claimed in auto", lr.ambiguous, 1, 0);
      manchester_decode_set_rate(0);
    }

    // The bench's examination, and the two halves of it that matter. The
    // same payload goes out in both conventions and then once more with an
    // encoding violation in it.
    printf("manchester, the other convention found from the preamble:\n");
    {
      // 0x55 preamble then 0xA5C3 - the shape nearly every Manchester
      // protocol opens with, and the reason a preamble is there at all
      ManGen g;

      man_g(&g, buf, SIZE, 20000.0, 1000.0);
      man_g_frame(&g, 0x55A5C3, 24);

      manchester_decode_set_rate(1000);

      // Told the wrong convention, it reads the complement of everything -
      // which is not a fault, it is what the other convention IS
      manchester_decode_set_polarity(1);
      manchester_decode(buf, SIZE, 0, 20000, &scratch, &lr);
      check_near("read the wrong way round", lr.bytes[0], 0xAA, 0);

      // A record already the right way up is left alone
      manchester_decode_set_polarity(MAN_POL_AUTO);
      manchester_decode(buf, SIZE, 0, 20000, &scratch, &lr);
      check_near("nothing to flip", manchester_analysis()->auto_inv, 0, 0);
      check_near("still 55", lr.bytes[0], 0x55, 0);

      // ...and now the SAME message sent in the other convention. Inverting
      // the levels is the same as inverting the data, so a Thomas generator
      // fed the complement puts exactly an IEEE frame on the wire.
      man_g(&g, buf, SIZE, 20000.0, 1000.0);
      man_g_frame(&g, (~0x55A5C3u) & 0xFFFFFFu, 24);

      manchester_decode_set_polarity(MAN_POL_AUTO);
      manchester_decode(buf, SIZE, 0, 20000, &scratch, &lr);
      const ManAnalysis *a = manchester_analysis();

      check_near("preamble back to 55", lr.bytes[0], 0x55, 0);
      check_near("...and the payload with it", lr.bytes[1], 0xA5, 0);
      check_near("...", lr.bytes[2], 0xC3, 0);
      check_near("it says it inferred that", a->auto_inv, 1, 0);
      check_near("header marks it", NULL != strstr(lr.info, "inv?"), 1, 0);

      // ...and the case the bench actually produces: BOTH conventions in one
      // record, one after the other, which is what a slow enough timebase
      // catches. A rule that looked at the first byte of the RECORD would
      // flip on the first frame's preamble and leave every other frame
      // exactly as wrong as it was - so the convention is decided per FRAME.
      man_g(&g, buf, SIZE, 20000.0, 1000.0);
      man_g_frame(&g, 0x55A5C3, 24);
      man_g_frame(&g, (~0x55A5C3u) & 0xFFFFFFu, 24);

      manchester_decode_set_polarity(MAN_POL_AUTO);
      manchester_decode(buf, SIZE, 0, 20000, &scratch, &lr);
      const ManAnalysis *b = manchester_analysis();

      check_near("two frames", b->frames, 2, 0);
      check_near("the first was already right", b->frame[0].inv, 0, 0);
      check_near("...and reads 55", lr.bytes[0], 0x55, 0);
      check_near("the second was flipped", b->frame[1].inv, 1, 0);
      check_near("...and reads 55 too",
          lr.bytes[b->frame[1].first], 0x55, 0);
      check_near("...with the payload", lr.bytes[b->frame[1].first + 1],
          0xA5, 0);

      // The frame that was flipped says so on the trace, because a header
      // naming one convention names the wrong one for half the record
      char lab2[16];

      manchester_byte_label(b, b->frame[1].first, 0, lab2, sizeof(lab2));
      check_near("and it says so", NULL != strchr(lab2, 'i'), 1, 0);
      manchester_byte_label(b, 0, 0, lab2, sizeof(lab2));
      check_near("the other does not", NULL == strchr(lab2, 'i'), 1, 0);

      manchester_decode_set_polarity(0);
    }

    // A bit with no transition in its middle. The frame is Manchester and one
    // of its bits is broken, and those are different statements - so the
    // record must come back WITH an error on it, not fail to come back. That
    // is the same line already drawn for a MIDI byte nobody should have sent.
    printf("manchester, a bit with no middle:\n");
    {
      ManGen g;
      char lab[16];

      man_g(&g, buf, SIZE, 20000.0, 1000.0);
      man_g_bits(&g, 0x55A5C3, 24, 15);   // the last bit of the 0xA5 byte

      manchester_decode_set_rate(1000);
      manchester_decode_set_polarity(0);

      int n = manchester_decode(buf, SIZE, 0, 20000, &scratch, &lr);
      const ManAnalysis *a = manchester_analysis();

      check_near("the frame is still there", n > 0, 1, 0);
      check_near("all 24 bits of it", a->frame[0].bits, 24, 0);
      check_near("the violation is counted", a->frame[0].viol, 1, 0);
      check_near("...and named", a->frame[0].viol_bit, 15, 0);
      check_near("counted as an error", lr.errors, 1, 0);
      check_near("header says which bit",
          NULL != strstr(lr.info, "!b15"), 1, 0);

      // The bits either side of it are unaffected: the reader keeps the
      // phase through the broken bit instead of slipping half a bit and
      // turning the rest of the frame into porridge
      check_near("preamble intact", lr.bytes[0], 0x55, 0);
      check_near("and the last byte too", lr.bytes[2], 0xC3, 0);

      manchester_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("the frame label carries it",
          NULL != strchr(lab, '!'), 1, 0);

      // A record whose ONLY frame is broken is reported and not CLAIMED:
      // "Manchester with a fault in it" and "something else that fits
      // Manchester badly" look identical from one frame, so the cascade
      // waits for a clean one and this is shown when picked by name.
      check_near("not claimed on a broken frame alone", lr.ambiguous, 1, 0);
      check_near("auto keeps off it", logic_decode(buf, SIZE, 0, 20000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_MANCH, 0, 0);

      // ...but a clean frame beside it settles the record, and the broken one
      // still comes back with its violation
      man_g(&g, buf, SIZE, 20000.0, 1000.0);
      man_g_frame(&g, 0x55A5C3, 24);
      man_g_bits(&g, 0x55A5C3, 24, 15);

      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      const ManAnalysis *b = manchester_analysis();

      check_near("auto takes it now", lr.proto, PROTO_MANCH, 0);
      check_near("two frames", b->frames, 2, 0);
      check_near("the first is clean", b->frame[0].viol, 0, 0);
      check_near("the second is not", b->frame[1].viol, 1, 0);
      check_near("...on the same bit", b->frame[1].viol_bit, 15, 0);
      check_near("and the error is counted", lr.errors, 1, 0);
      // ...and the header names the frame as well as the bit, because a bit
      // number on its own points at a perfectly good bit of frame zero
      check_near("header names frame and bit",
          NULL != strstr(lr.info, "!f1b15"), 1, 0);
    }

    printf("manchester rejects the rest:\n");
    {
      // NEC\'s runs stand in a 1:3 ratio, not 1:2 - three halves cannot pass
      // for two, and that is what the tolerance is set to guarantee
      const uint8_t nec[4] = { 0x00, 0xFF, 0x15, 0xEA };
      int pos = 0;

      memset(buf, 200, SIZE);

      for (int i = 0; i < 9000 && pos < SIZE; i++, pos++)
        buf[pos] = 56;

      for (int i = 0; i < 4500 && pos < SIZE; i++, pos++)
        buf[pos] = 200;

      for (int by = 0; by < 4; by++)
      {
        for (int b = 0; b < 8; b++)
        {
          int one = (nec[by] >> b) & 1;

          for (int i = 0; i < 560 && pos < SIZE; i++, pos++)
            buf[pos] = 56;

          for (int i = 0; i < (one ? 1690 : 560) && pos < SIZE; i++, pos++)
            buf[pos] = 200;
        }
      }

      manchester_decode_set_rate(0);
      check_near("nec is not manchester", logic_decode(buf, SIZE, 0, 1000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_MANCH, 0, 0);
      check_near("...and is still nec", lr.proto, PROTO_NEC, 0);

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST 9600", 20000.0);
      check_near("uart is not manchester", logic_decode(buf, SIZE, 0, 1000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_MANCH, 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("a sine is a square wave, so it is not sure",
          manchester_decode(buf, SIZE, 0, 1000, &scratch, &lr) > 0 &&
          !lr.ambiguous, 0, 0);
      check_near("and auto keeps off it", logic_decode(buf, SIZE, 0, 1000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_MANCH, 0, 0);
    }

    // ======================== RC5 / RC5X / RC6 ============================
    //
    // The bi-phase half of the infrared world. The generic Manchester decoder
    // already reads RC5's bits; what these do is read the MESSAGE - address,
    // command, and the toggle bit that tells a fresh press from a held key.
    printf("rc5:\n");
    {
      // S1=1 S2=1 T=1 A=00101 C=110101 -> address 5, command 53
      uint32_t frame = (1u << 13) | (1u << 12) | (1u << 11) |
          (5u << 6) | 53u;
      IrGen g;
      char lab[16];

      ir_g(&g, buf, SIZE, 20000.0, 1778.0);
      rc5_g_frame(&g, frame, 14);

      int n = logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      const Rc5Analysis *a = rc5_analysis();

      check_near("auto takes it", lr.proto, PROTO_RC5, 0);
      check_near("plain RC5", a->kind, RC5_KIND_RC5, 0);
      check_near("address", a->addr, 5, 0);
      check_near("command", a->cmd, 53, 0);
      check_near("toggle", a->toggle, 1, 0);
      check_near("fourteen bits", a->bits, 14, 0);
      check_near("bit time", a->bit_ns, 1778000, 5);
      check_near("two bytes", n, 2, 0);
      check_near("header", !strcmp(lr.info, "RC5 a5 c53 t1"), 1, 0);

      rc5_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("reads as a message", !strcmp(lab, "a5 c53 t1"), 1, 0);

      int gs, gl;

      rc5_group_at(a, 1, &gs, &gl);
      check_near("one frame, one group", gs, 0, 0);
      check_near("...of two bytes", gl, 2, 0);
    }

    // RC5X: the second start bit is the INVERTED seventh command bit, which
    // is how a six-bit field came to hold 128 commands. A decoder that just
    // read S2 as "a start bit" would report command 35 instead of 99.
    printf("rc5x, the command that does not fit six bits:\n");
    {
      uint32_t frame = (1u << 13) | (0u << 12) | (0u << 11) |
          (5u << 6) | 35u;
      IrGen g;

      ir_g(&g, buf, SIZE, 20000.0, 1778.0);
      rc5_g_frame(&g, frame, 14);

      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      const Rc5Analysis *a = rc5_analysis();

      check_near("recognised as RC5X", a->kind, RC5_KIND_RC5X, 0);
      check_near("command carries the seventh bit", a->cmd, 35 + 64, 0);
      check_near("address still", a->addr, 5, 0);
      check_near("header", !strcmp(lr.info, "RC5X a5 c99 t0"), 1, 0);
    }

    // The toggle bit is the whole point of RC5 on a bench: it says a key was
    // pressed AGAIN rather than held down, and nothing else in the frame does
    printf("rc5, the toggle bit:\n");
    {
      uint32_t base = (1u << 13) | (1u << 12) | (5u << 6) | 53u;
      IrGen g;

      ir_g(&g, buf, SIZE, 20000.0, 1778.0);
      rc5_g_frame(&g, base, 14);        // toggle clear
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      check_near("held: toggle clear", rc5_analysis()->toggle, 0, 0);
      check_near("...and the key is the same", rc5_analysis()->cmd, 53, 0);
    }

    // RC6 mode 0: a leader, and a trailer bit twice as wide as the rest. The
    // generic Manchester reader cannot follow that - on the trailer its runs
    // are two halves and four - which is exactly why this decoder exists.
    printf("rc6:\n");
    {
      // start=1 mode=000 trailer=1 addr=0x12 cmd=0x34
      uint32_t frame = (1u << 20) | (0u << 17) | (1u << 16) |
          (0x12u << 8) | 0x34u;
      IrGen g;

      ir_g(&g, buf, SIZE, 20000.0, 889.0);
      rc6_g_frame(&g, frame, 21, 4);

      int n = logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      const Rc5Analysis *a = rc5_analysis();

      check_near("auto takes it", lr.proto, PROTO_RC5, 0);
      check_near("recognised as RC6", a->kind, RC5_KIND_RC6, 0);
      check_near("mode 0", a->mode, 0, 0);
      check_near("address", a->addr, 0x12, 0);
      check_near("command", a->cmd, 0x34, 0);
      check_near("toggle", a->toggle, 1, 0);
      check_near("twenty-one bits", a->bits, 21, 0);
      check_near("three bytes", n, 3, 0);
      check_near("bit time", a->bit_ns, 889000, 6);
      check_near("header", !strcmp(lr.info, "RC6 m0 a18 c52 t1"), 1, 0);

      // ...and the double-width bit is why the generic reader is not enough
      manchester_decode_set_rate(0);
      check_near("the generic reader does not get all of it",
          manchester_decode(buf, SIZE, 0, 20000, &scratch, &lr) > 0 &&
          21 == manchester_analysis()->frame[0].bits, 0, 0);
    }

    printf("rc5 rejects the rest:\n");
    {
      // NEC is pulse-coded and not bi-phase at all - one of the family that
      // the bench list groups with RC5 and that has nothing to do with it
      const uint8_t nec[4] = { 0x00, 0xFF, 0x15, 0xEA };
      int pos = 0;

      memset(buf, 200, SIZE);

      for (int i = 0; i < 9000 && pos < SIZE; i++, pos++)
        buf[pos] = 56;

      for (int i = 0; i < 4500 && pos < SIZE; i++, pos++)
        buf[pos] = 200;

      for (int by = 0; by < 4; by++)
      {
        for (int b = 0; b < 8; b++)
        {
          int one = (nec[by] >> b) & 1;

          for (int i = 0; i < 560 && pos < SIZE; i++, pos++)
            buf[pos] = 56;

          for (int i = 0; i < (one ? 1690 : 560) && pos < SIZE; i++, pos++)
            buf[pos] = 200;
        }
      }

      check_near("nec is not rc5",
          rc5_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
      check_near("...and is still nec", logic_decode(buf, SIZE, 0, 1000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_NEC, 1, 0);

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST", 20000.0);
      check_near("uart is not rc5",
          rc5_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not rc5",
          rc5_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      // A DALI-shaped Manchester frame is bi-phase too, and is not RC5: the
      // bit time is nowhere near 1.778 ms and there are not fourteen of them
      ManGen mg;

      man_g(&mg, buf, SIZE, 20000.0, 1200.0);
      man_g_frame(&mg, 0x5A5A5, 19);
      check_near("dali-shaped manchester is not rc5",
          rc5_decode(buf, SIZE, 0, 20000, &scratch, &lr), 0, 0);
    }

    // ============================== DALI ==================================
    //
    // Manchester at 1200, which the generic decoder already reads as bits.
    // What is tested here is the reading a lighting bus is looked at for:
    // which ballast, and what it was told.
    printf("dali, a level to one ballast:\n");
    {
      DaliGen g;
      char lab[16];

      // 0x0A = 0000101 0: short address 5, selector clear -> the data byte
      // is an arc power level
      dali_g(&g, buf, SIZE, 20000.0);
      dali_g_fwd(&g, 0x0A, 128);

      int n = logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      const DaliAnalysis *a = dali_analysis();

      check_near("auto takes it", lr.proto, PROTO_DALI, 0);
      check_near("one frame", a->frames, 1, 0);
      check_near("seventeen bits", a->frame[0].bits, 17, 0);
      check_near("short address", a->frame[0].kind, DALI_SHORT, 0);
      check_near("ballast 5", a->frame[0].target, 5, 0);
      check_near("...and it is a level, not a command",
          a->frame[0].cmd, 0, 0);
      check_near("level", a->frame[0].data, 128, 0);
      check_near("two bytes", n, 2, 0);
      check_near("rate", lr.rate, 1200, 5);
      check_near("header", !strcmp(lr.info, "DALI a5 lvl 128"), 1, 0);

      dali_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("first byte is the address", !strcmp(lab, "ADDR"), 1, 0);
      dali_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("second is a level", !strcmp(lab, "LEVEL"), 1, 0);
    }

    // The selector bit is the whole trap: the SAME data byte is "off" one way
    // and "level zero" the other, and a decoder that shows the byte and stops
    // has left the reader to do the part that matters
    printf("dali, the selector bit:\n");
    {
      DaliGen g;

      dali_g(&g, buf, SIZE, 20000.0);
      dali_g_fwd(&g, 0x0B, 0);          // short address 5, selector SET
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      check_near("a command this time", dali_analysis()->frame[0].cmd, 1, 0);
      check_near("and it is OFF", !strcmp(lr.info, "DALI a5 OFF"), 1, 0);

      dali_g(&g, buf, SIZE, 20000.0);
      dali_g_fwd(&g, 0x0A, 0);          // ...the same byte, selector clear
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      check_near("same byte, different meaning",
          !strcmp(lr.info, "DALI a5 lvl 0"), 1, 0);
    }

    printf("dali, groups, broadcast and the commissioning set:\n");
    {
      DaliGen g;

      // 0x87 = 100 0011 1: group 3, selector set
      dali_g(&g, buf, SIZE, 20000.0);
      dali_g_fwd(&g, 0x87, 18);         // GO TO SCENE 2
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      check_near("group", dali_analysis()->frame[0].kind, DALI_GROUP, 0);
      check_near("number", dali_analysis()->frame[0].target, 3, 0);
      check_near("scene named", !strcmp(lr.info, "DALI g3 SCENE 2"), 1, 0);

      dali_g(&g, buf, SIZE, 20000.0);
      dali_g_fwd(&g, 0xFF, 32);         // broadcast RESET
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      check_near("broadcast", dali_analysis()->frame[0].kind,
          DALI_BROADCAST, 0);
      check_near("named", !strcmp(lr.info, "DALI all RESET"), 1, 0);

      // Special commands live in the ADDRESS byte, not the data byte
      dali_g(&g, buf, SIZE, 20000.0);
      dali_g_fwd(&g, 0xA5, 0);          // INITIALISE
      logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      check_near("special", dali_analysis()->frame[0].kind, DALI_SPECIAL, 0);
      check_near("named", !strcmp(lr.info, "DALI INITIALISE 0"), 1, 0);
    }

    // A query and the answer to it is the commonest pair on the bus, and the
    // answer is a SHORTER frame - nine bits against seventeen
    printf("dali, a query and its answer:\n");
    {
      DaliGen g;
      char lab[16];

      dali_g(&g, buf, SIZE, 20000.0);
      dali_g_fwd(&g, 0x0B, 144);        // QUERY STATUS of ballast 5
      dali_g_frame(&g, (1u << 8) | 0xFF, 9);   // ...and the backward frame

      int n = logic_decode(buf, SIZE, 0, 20000, PROTO_AUTO, &scratch, &lr);
      const DaliAnalysis *a = dali_analysis();

      check_near("two frames", a->frames, 2, 0);
      check_near("the query is seventeen bits", a->frame[0].bits, 17, 0);
      check_near("the answer is nine", a->frame[1].bits, 9, 0);
      check_near("...and is a backward frame", a->frame[1].kind,
          DALI_BACKWARD, 0);
      check_near("three bytes in all", n, 3, 0);
      check_near("header keeps the question",
          !strcmp(lr.info, "DALI a5 Q STATUS +1"), 1, 0);

      dali_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("the answer reads as one", !strcmp(lab, "ans YES"), 1, 0);
      dali_field_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("...and is named so", !strcmp(lab, "ANS"), 1, 0);

      int gs, gl;

      dali_group_at(a, 1, &gs, &gl);
      check_near("the query groups", gs, 0, 0);
      check_near("...as two bytes", gl, 2, 0);
      dali_group_at(a, 2, &gs, &gl);
      check_near("the answer stands alone", gs, 2, 0);
      check_near("...as one", gl, 1, 0);
    }

    printf("dali rejects the rest:\n");
    {
      // An RC5 frame is bi-phase too, at 1.778 ms - not 833 us, and fourteen
      // bits rather than seventeen or nine
      IrGen ig;

      ir_g(&ig, buf, SIZE, 20000.0, 1778.0);
      rc5_g_frame(&ig, (1u << 13) | (1u << 12) | (5u << 6) | 53u, 14);
      check_near("rc5 is not dali",
          dali_decode(buf, SIZE, 0, 20000, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST", 20000.0);
      check_near("uart is not dali",
          dali_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not dali",
          dali_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    }

    // ============================= KNX TP1 ================================
    //
    // NOT Manchester, whatever it gets grouped with: a zero is a short pulse
    // at the start of its bit and a one is nothing at all, so there is no
    // mid-bit transition to step to and the slots are counted from the start
    // bit. What confirms it is a parity bit per character and a check octet
    // over the telegram.
    printf("knx, a group write:\n");
    {
      // 1.1.5 tells group 1/2/3 to switch on
      const uint8_t tg[7] =
          { 0xBC, 0x11, 0x05, 0x0A, 0x03, 0xE1, 0x81 };
      KnxGen g;
      char lab[16];

      knx_g(&g, buf, SIZE, 4000.0);
      knx_g_telegram(&g, tg, 7);

      int n = logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
      const KnxAnalysis *a = knx_analysis();

      check_near("auto takes it", lr.proto, PROTO_KNX, 0);
      check_near("eight octets", n, 8, 0);
      check_near("no parity errors", a->parity_err, 0, 0);
      check_near("the check octet agrees", a->fcs_ok, 1, 0);
      check_near("rate", lr.rate, 9600, 0);

      // 0x1105 as an individual address is 1.1.5; 0x0A03 as a GROUP address
      // is 1/2/3 - the same sixteen bits split 5/3/8 instead of 4/4/8
      check_near("source", a->src, 0x1105, 0);
      check_near("destination", a->dst, 0x0A03, 0);
      check_near("...and it is a group", a->group, 1, 0);
      check_near("length", a->len, 1, 0);
      check_near("header", !strcmp(lr.info, "KNX 1.1.5>1/2/3 1B"), 1, 0);

      knx_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("source reads as an address", !strcmp(lab, "1.1.5"), 1, 0);
      knx_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("destination as a group", !strcmp(lab, "1/2/3"), 1, 0);
      knx_field_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("its octets named", !strcmp(lab, "SRC-H"), 1, 0);
      knx_byte_label(a, 7, lr.bytes[7], lab, sizeof(lab));
      check_near("check octet named", !strcmp(lab, "FCS"), 1, 0);

      // Two octets are one address and are shown as one
      int gs, gl;

      knx_group_at(a, 2, &gs, &gl);
      check_near("source groups", gs, 1, 0);
      check_near("...as two", gl, 2, 0);
    }

    // An individual destination is the SAME sixteen bits read 4/4/8 instead
    // of 5/3/8, and the flag that decides is in the length octet
    printf("knx, an individual destination:\n");
    {
      const uint8_t tg[7] =
          { 0xBC, 0x11, 0x05, 0x11, 0x0A, 0x60, 0x80 };
      KnxGen g;
      char lab[16];

      knx_g(&g, buf, SIZE, 4000.0);
      knx_g_telegram(&g, tg, 7);

      logic_decode(buf, SIZE, 0, 4000, PROTO_AUTO, &scratch, &lr);
      const KnxAnalysis *a = knx_analysis();

      check_near("not a group", a->group, 0, 0);
      knx_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("read as a device", !strcmp(lab, "1.1.10"), 1, 0);
      check_near("header", !strcmp(lr.info, "KNX 1.1.5>1.1.10 0B"), 1, 0);
    }

    // The check octet is what says these are one telegram and not a run of
    // characters that happened to have good parity
    printf("knx, a broken check octet:\n");
    {
      uint8_t tg[8] = { 0xBC, 0x11, 0x05, 0x0A, 0x03, 0xE1, 0x81, 0x00 };
      KnxGen g;
      uint8_t x = 0;

      for (int i = 0; i < 7; i++)
        x ^= tg[i];

      tg[7] = (uint8_t)(~x ^ 0x01);    // one bit out

      knx_g(&g, buf, SIZE, 4000.0);

      for (int i = 0; i < 8; i++)
        knx_g_char(&g, tg[i]);

      int n = knx_decode(buf, SIZE, 0, 4000, &scratch, &lr);

      check_near("still decodes", n, 8, 0);
      check_near("but the check octet is flagged",
          knx_analysis()->fcs_ok, 0, 0);
      check_near("counted as an error", lr.errors, 1, 0);
      check_near("header says so", NULL != strstr(lr.info, "FCS!"), 1, 0);
      check_near("and auto keeps off it", logic_decode(buf, SIZE, 0, 4000,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_KNX, 0, 0);
    }

    printf("knx rejects the rest:\n");
    {
      DaliGen dg;

      dali_g(&dg, buf, SIZE, 20000.0);
      dali_g_fwd(&dg, 0x0A, 128);
      check_near("dali is not knx",
          knx_decode(buf, SIZE, 0, 20000, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST", 20000.0);
      check_near("uart at the same rate is not knx",
          knx_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not knx",
          knx_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    }

    // ============================== SWO / ITM ==============================
    //
    // Electrically a UART and nothing else, so every bit of the identification
    // is in what the bytes SAY. The packet grammar is the whole decoder.
    printf("swo, a printf on stimulus port 0:\n");
    {
      // The synchronisation sequence a trace session opens with - 47 zero
      // bits and a one - and then ITM_SendChar five times over
      const uint8_t s[] =
      {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x01, 'H', 0x01, 'e', 0x01, 'l', 0x01, 'l', 0x01, 'o',
      };
      SwoGen g;
      char lab[16];

      swo_g(&g, buf, SIZE, 100.0, 1000000.0);
      swo_g_bytes(&g, s, sizeof(s));

      int n = logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SwoAnalysis *a = swo_analysis();

      check_near("auto takes it", lr.proto, PROTO_SWO, 0);
      check_near("every byte", n, (int)sizeof(s), 0);
      check_near("rate", lr.rate, 1000000, 2);
      check_near("one synchronisation", a->sync, 1, 0);
      check_near("five characters", a->sw, 5, 0);
      check_near("nothing left over", a->bad, 0, 0);
      check_near("...so it is not ambiguous", lr.ambiguous, 0, 0);

      // The point of the pin: the firmware's own printf, off one wire
      check_near("the text came back", !strcmp(a->text, "Hello"), 1, 0);
      check_near("on port 0", a->text_port, 0, 0);
      check_near("and the header shows it",
          NULL != strstr(lr.info, "\"Hello\""), 1, 0);

      swo_byte_label(a, 7, lr.bytes[7], lab, sizeof(lab));
      check_near("a character reads as one", !strcmp(lab, "'H'"), 1, 0);
      swo_field_label(a, 6, lr.bytes[6], lab, sizeof(lab));
      check_near("its header names the port", !strcmp(lab, "p0"), 1, 0);
      swo_field_label(a, 7, lr.bytes[7], lab, sizeof(lab));
      check_near("and the payload is numbered", !strcmp(lab, "D0"), 1, 0);
      swo_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("the sync sequence is named", !strcmp(lab, "SYNC"), 1, 0);

      // A packet is one thing, so its header and payload light together
      int gs, gl;

      swo_group_at(a, 7, &gs, &gl);
      check_near("header and payload are one", gs, 6, 0);
      check_near("...of two bytes", gl, 2, 0);
      swo_group_at(a, 2, &gs, &gl);
      check_near("and the sync is one of six", gl, 6, 0);
    }

    // The other half of what a trace pin carries, and the half a UART could
    // never be mistaken for: the DWT reporting on the core itself
    printf("swo, what the DWT says:\n");
    {
      const uint8_t s[] =
      {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x0E, 0x0F, 0x10,                    // exception 15 entered
        0x17, 0x00, 0x01, 0x00, 0x08,        // PC sample = 0x08000100
        0x0E, 0x0F, 0x20,                    // ...and exited again
      };
      SwoGen g;
      char lab[16];

      swo_g(&g, buf, SIZE, 100.0, 2000000.0);
      swo_g_bytes(&g, s, sizeof(s));

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SwoAnalysis *a = swo_analysis();

      check_near("auto takes it", lr.proto, PROTO_SWO, 0);
      check_near("three hardware packets", a->hw, 3, 0);
      check_near("no stimulus traffic", a->sw, 0, 0);
      check_near("nothing left over", a->bad, 0, 0);

      // An exception number is a name, not a number: 15 is the SysTick
      swo_byte_label(a, 8, lr.bytes[8], lab, sizeof(lab));
      check_near("the exception is named", !strcmp(lab, "SysTick in"), 1, 0);
      swo_byte_label(a, 16, lr.bytes[16], lab, sizeof(lab));
      check_near("...and so is the way out", !strcmp(lab, "SysTick out"), 1, 0);
      swo_byte_label(a, 13, lr.bytes[13], lab, sizeof(lab));
      check_near("the sampled PC", !strcmp(lab, "PC=08000100"), 1, 0);
      swo_field_label(a, 9, lr.bytes[9], lab, sizeof(lab));
      check_near("its header names the DWT unit", !strcmp(lab, "DWT2"), 1, 0);
    }

    // Timestamps interleave with the data, and an overflow says the macrocell
    // dropped trace - which is worth seeing, because what follows has holes
    printf("swo, timestamps and an overflow:\n");
    {
      const uint8_t s[] =
      {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x01, 'A', 0x30, 0x01, 'B', 0x70, 0x01, 'C',
      };
      SwoGen g;
      char lab[16];

      swo_g(&g, buf, SIZE, 100.0, 1000000.0);
      swo_g_bytes(&g, s, sizeof(s));

      swo_decode(buf, SIZE, 0, 100, &scratch, &lr);
      const SwoAnalysis *a = swo_analysis();

      check_near("one timestamp", a->ts, 1, 0);
      check_near("three characters", a->sw, 3, 0);
      check_near("nothing left over", a->bad, 0, 0);

      swo_byte_label(a, 8, lr.bytes[8], lab, sizeof(lab));
      check_near("the short timestamp", !strcmp(lab, "t+3"), 1, 0);
      swo_byte_label(a, 11, lr.bytes[11], lab, sizeof(lab));
      check_near("the overflow says so", !strcmp(lab, "OVERFLOW"), 1, 0);

      // The text skips over both of them, because a timestamp is not a
      // character and the stream is still one line
      check_near("the text is still a line", !strcmp(a->text, "ABC"), 1, 0);
    }

    // A record that begins in the middle of a packet: the bytes in front of
    // the first whole one are not a reading of anything, and saying so is the
    // difference between a decoder and a guess
    printf("swo, a record that opened mid-packet:\n");
    {
      const uint8_t s[] =
      {
        0x04,                                 // the tail of a packet gone by
        0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x01, 'o', 0x01, 'k',
      };
      SwoGen g;
      char lab[16];

      swo_g(&g, buf, SIZE, 100.0, 1000000.0);
      swo_g_bytes(&g, s, sizeof(s));

      swo_decode(buf, SIZE, 0, 100, &scratch, &lr);
      const SwoAnalysis *a = swo_analysis();

      check_near("it found the first whole packet", a->cut, 1, 0);
      check_near("and read the rest", a->sw, 2, 0);
      check_near("with nothing left over", a->bad, 0, 0);

      swo_field_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("the orphan byte says so", !strcmp(lab, "cut"), 1, 0);
    }

    printf("swo rejects the rest:\n");
    {
      // The case that matters, and the reason the grammar is checked at all:
      // a console is the same signal on the same kind of wire
      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST", 20000.0);
      check_near("a console is not a trace pin",
          swo_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 1000.0, 9600.0, "Hello, world! 1234", 20000.0);
      check_near("...nor is a longer line",
          swo_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth_uart(buf, SIZE, 100.0, 1000000.0, "boot ok, temp 24.5C", 2000.0);
      check_near("...nor one at an SWO rate",
          swo_decode(buf, SIZE, 0, 100, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not swo",
          swo_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    }

    // ================================= SWD =================================
    //
    // One probe on a two-wire bus, like the SPI decoder - and unlike it, the
    // protocol checks itself, so a transaction that reads clean confirms the
    // recovered clock along with itself.
    printf("swd, a debugger attaching:\n");
    {
      SwdGen g;
      char lab[16];

      swd_g(&g, buf, SIZE, 100.0, 1000000.0);
      swd_g_reset(&g);                          // fifty clocks of ones
      swd_g_switch(&g);                         // out of JTAG, into SWD
      swd_g_idle(&g, 8);
      swd_g_tx(&g, 0, 0, 0, 0x2BA01477, 1, 0);  // DP read DPIDR
      swd_g_tx(&g, 0, 1, 1, 0x50000000, 1, 0);  // DP write CTRL/STAT

      int n = logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SwdAnalysis *a = swd_analysis();

      check_near("auto takes it", lr.proto, PROTO_SWD, 0);
      check_near("two transactions", a->txs, 2, 0);
      check_near("a line reset", a->resets, 1, 0);
      check_near("and the switch sequence", a->switches, 1, 0);
      check_near("clock", a->clock_hz, 1000000, 4);
      check_near("bytes: two of switch and two transactions", n, 12, 0);
      check_near("it is sure", a->sure, 1, 0);
      check_near("...so not ambiguous", lr.ambiguous, 0, 0);

      // 0xA5 is the request every debug session opens with, and it comes out
      // of the wire bits in the order they were sent
      check_near("the DPIDR request", lr.bytes[2], 0xA5, 0);
      check_near("...and the identifier it answered with", lr.bytes[3], 0x77, 0);
      check_near("the CTRL/STAT write", lr.bytes[7], 0xA9, 0);

      swd_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("the register is named", !strcmp(lab, "RD DPIDR"), 1, 0);
      swd_byte_label(a, 6, lr.bytes[6], lab, sizeof(lab));
      check_near("the value written once", !strcmp(lab, "2BA01477"), 1, 0);
      swd_byte_label(a, 7, lr.bytes[7], lab, sizeof(lab));
      check_near("and the write named", !strcmp(lab, "WR CTRL/STAT"), 1, 0);
      swd_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("the switch sequence is named",
          !strcmp(lab, "JTAG>SWD"), 1, 0);

      // Four bytes are one register and are shown as one
      int gs, gl;

      swd_group_at(a, 4, &gs, &gl);
      check_near("the data groups", gs, 3, 0);
      check_near("...as four", gl, 4, 0);
      swd_group_at(a, 2, &gs, &gl);
      check_near("the request stands alone", gl, 1, 0);
    }

    // An AP register has no name of its own: which one A[3:2] selects depends
    // on a SELECT write that went past earlier, exactly the way a 1-Wire
    // function command depends on the family code before it
    printf("swd, an AP read needs the bank:\n");
    {
      SwdGen g;
      char lab[16];

      swd_g(&g, buf, SIZE, 100.0, 1000000.0);
      swd_g_tx(&g, 0, 1, 2, 0x00000000, 1, 0);  // DP write SELECT, bank 0
      swd_g_tx(&g, 1, 0, 0, 0x23000042, 1, 0);  // AP read CSW
      swd_g_tx(&g, 1, 1, 1, 0x20000000, 1, 0);  // AP write TAR

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SwdAnalysis *a = swd_analysis();

      check_near("three transactions", a->txs, 3, 0);
      check_near("the bank was remembered", a->select, 0, 0);

      swd_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("SELECT named", !strcmp(lab, "WR SELECT"), 1, 0);
      swd_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("the AP register named", !strcmp(lab, "RD CSW"), 1, 0);
      swd_byte_label(a, 10, lr.bytes[10], lab, sizeof(lab));
      check_near("...and the next one", !strcmp(lab, "WR TAR"), 1, 0);
    }

    // WAIT means "ask again": the target acknowledged and then said nothing,
    // so there is no data phase to read and inventing one would be a lie
    printf("swd, a target that said WAIT:\n");
    {
      SwdGen g;
      char lab[16];

      swd_g(&g, buf, SIZE, 100.0, 1000000.0);
      swd_g_tx(&g, 1, 0, 3, 0, 2, 0);           // AP read DRW -> WAIT
      swd_g_tx(&g, 1, 0, 3, 0x20000000, 1, 0);  // ...and again, this time OK
      swd_g_tx(&g, 1, 0, 3, 0x20000004, 1, 0);

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SwdAnalysis *a = swd_analysis();

      check_near("auto takes it", lr.proto, PROTO_SWD, 0);
      check_near("all three transactions", a->txs, 3, 0);
      check_near("one of them waited", a->waits, 1, 0);
      // A WAIT carries no data phase, so it is one byte and not five -
      // inventing four more would be inventing a register read that never
      // happened
      check_near("one byte for it, five for each of the rest",
          lr.count, 11, 0);
      check_near("the two that answered", a->ok_txs, 2, 0);

      swd_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("and it says so", NULL != strstr(lab, "WAIT"), 1, 0);
      check_near("the header too", NULL != strstr(lr.info, "W"), 1, 0);
    }

    // The data parity is REPORTED and not required: a packet that got that
    // far passed five other checks, so a bad parity is a fact about the
    // traffic and hiding the transaction would hide the fault
    printf("swd, a data parity that does not agree:\n");
    {
      SwdGen g;
      char lab[16];

      swd_g(&g, buf, SIZE, 100.0, 1000000.0);
      swd_g_tx(&g, 0, 0, 0, 0x2BA01477, 1, 1);  // parity flipped on purpose
      swd_g_tx(&g, 0, 0, 3, 0x00000001, 1, 0);

      swd_decode(buf, SIZE, 0, 100, &scratch, &lr);
      const SwdAnalysis *a = swd_analysis();

      check_near("still decoded", a->txs, 2, 0);
      check_near("the parity is counted", a->par_err, 1, 0);
      check_near("...as an error", lr.errors, 1, 0);

      swd_byte_label(a, 4, lr.bytes[4], lab, sizeof(lab));
      check_near("and marked on the value",
          NULL != strstr(lab, "par!"), 1, 0);
      check_near("the header says so",
          NULL != strstr(lr.info, "par!"), 1, 0);
    }

    // One transaction is six checks, which unrelated traffic passes about
    // once in a hundred candidate edges - and a record has hundreds of them.
    // So one on its own is reported and marked, and auto mode passes over it.
    printf("swd, one transaction is not enough:\n");
    {
      SwdGen g;

      swd_g(&g, buf, SIZE, 100.0, 1000000.0);
      swd_g_tx(&g, 0, 0, 0, 0x2BA01477, 1, 0);

      check_near("asked for by name it decodes",
          swd_decode(buf, SIZE, 0, 100, &scratch, &lr) > 0, 1, 0);
      check_near("but it is not sure", swd_analysis()->sure, 0, 0);
      check_near("...and says so", lr.ambiguous, 1, 0);
      check_near("so auto passes over it",
          logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr) > 0 &&
          lr.proto == PROTO_SWD, 0, 0);
    }

    // Told the clock, the rate stops being an estimate - and the header
    // stops apologising for it
    printf("swd, told the clock off SWCLK:\n");
    {
      SwdGen g;

      swd_g(&g, buf, SIZE, 100.0, 2000000.0);
      swd_g_tx(&g, 0, 0, 0, 0x2BA01477, 1, 0);
      swd_g_tx(&g, 0, 1, 1, 0x50000000, 1, 0);

      swd_decode_set_clock(2000000);
      swd_decode(buf, SIZE, 0, 100, &scratch, &lr);

      check_near("two transactions", swd_analysis()->txs, 2, 0);
      check_near("and it was told", swd_analysis()->told, 1, 0);
      check_near("so the header does not hedge",
          NULL == strstr(lr.info, "~"), 1, 0);

      // ...and a clock that is not this record's produces NOTHING, which is
      // the failure this decoder must have: no output beats wrong output
      swd_decode_set_clock(200000);
      check_near("a wrong clock decodes nothing",
          swd_decode(buf, SIZE, 0, 100, &scratch, &lr), 0, 0);

      swd_decode_set_clock(0);
    }

    // A packet may follow the one before it with NO idle cycles, and then
    // the previous turnaround left the line high and the start bit has no
    // rising edge to be found by. What every packet does have is its stop and
    // park bits - 0 then 1, a rising edge at bit 7 whatever the data says -
    // so an edge that heads no packet is tried again as that one.
    printf("swd, packets back to back with no idle:\n");
    {
      SwdGen g;

      swd_g(&g, buf, SIZE, 100.0, 1000000.0);
      // Data ending in a 1 bit and an odd parity: the trailing turnaround
      // holds the line high, so the next start bit is not an edge at all
      swd_g_txi(&g, 0, 0, 0, 0x8BA01477, 1, 0, 0);
      swd_g_txi(&g, 0, 0, 3, 0x80000001, 1, 0, 0);
      swd_g_txi(&g, 0, 1, 1, 0x50000000, 1, 0, 8);

      logic_decode(buf, SIZE, 0, 100, PROTO_AUTO, &scratch, &lr);
      const SwdAnalysis *a = swd_analysis();

      check_near("auto takes it", lr.proto, PROTO_SWD, 0);
      check_near("all three, none swallowed", a->txs, 3, 0);
      check_near("every one of them clean", a->ok_txs, 3, 0);
      check_near("nothing between them", a->gap_bad, 0, 0);
      check_near("the second one's data, least byte first",
          lr.bytes[6], 0x01, 0);
      check_near("...read right through to its top", lr.bytes[9], 0x80, 0);
    }

    // A bit-banged master does not hold one rate: its loop takes longer as
    // the packet goes on, and no single bit time fits the whole of it. The
    // reader keeps its PHASE and re-anchors on the edges it finds, which is
    // what lets a packet like this come back at all - stepping a fixed grid
    // walks half a bit out by the parity, and the parity is what the decoder
    // rests on.
    printf("swd, a master whose rate drifts:\n");
    {
      SwdGen g;

      swd_g(&g, buf, SIZE, 100.0, 1000000.0);
      g.drift = 0.002;                          // 0.2% longer every bit
      swd_g_tx(&g, 0, 0, 0, 0x2BA01477, 1, 0);
      swd_g_tx(&g, 0, 1, 1, 0x50000000, 1, 0);

      swd_decode(buf, SIZE, 0, 100, &scratch, &lr);
      const SwdAnalysis *a = swd_analysis();

      check_near("both transactions", a->txs, 2, 0);
      check_near("both with the parity agreeing", a->ok_txs, 2, 0);
      check_near("and the value is right", lr.bytes[1], 0x77, 0);
      check_near("...to the last byte", lr.bytes[4], 0x2B, 0);
    }

    printf("swd rejects the rest:\n");
    {
      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST", 20000.0);
      check_near("uart is not swd",
          swd_decode(buf, SIZE, 0, 1000, &scratch, &lr) > 0 &&
          !lr.ambiguous, 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not swd",
          swd_decode(buf, SIZE, 0, 1000, &scratch, &lr) > 0 &&
          !lr.ambiguous, 0, 0);

      DaliGen dg;

      dali_g(&dg, buf, SIZE, 20000.0);
      dali_g_fwd(&dg, 0x0A, 128);
      check_near("dali is not swd",
          swd_decode(buf, SIZE, 0, 20000, &scratch, &lr) > 0 &&
          !lr.ambiguous, 0, 0);
    }

    // ================================= USB =================================
    //
    // One probe on a differential pair, which is the SPI and SWD predicament
    // again - and USB answers it better than either does, because NRZI puts
    // the data in the TRANSITIONS and both wires therefore carry the same
    // bits. What the missing wire costs is SE0, so the EOP is invisible, so a
    // data packet's length is not knowable from the waveform at all. Finding
    // it is the CRC16's job here, and that is what these first check.
    printf("usb full speed at 125 MS/s, a control transfer:\n");
    {
      UsbGen g;
      char lab[16];
      int gs, gl;
      // The eight bytes every enumeration begins with: GET_DESCRIPTOR(Device)
      static const uint8_t setup[8] =
          { 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00 };

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);   // full speed, J high: this is D+
      usb_g_token(&g, 0x2D, 0, 0, 4.0);     // SETUP, address 0, endpoint 0
      usb_g_data(&g, 0xC3, setup, 8, 4.0);  // DATA0
      usb_g_hs(&g, 0xD2, 20.0);             // ACK

      int n = logic_decode(buf, SIZE, 0, 8, PROTO_AUTO, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("auto takes it", lr.proto, PROTO_USB, 0);
      check_near("full speed", lr.rate, 12000000, 0);
      check_near("three packets", a->packets, 3, 0);
      check_near("two of them CRC-confirmed", a->crc_ok, 2, 0);
      check_near("so it is sure", a->sure, 1, 0);
      check_near("...and not ambiguous", lr.ambiguous, 0, 0);
      check_near("no errors", lr.errors, 0, 0);
      check_near("the probe is on D+", a->dplus, 1, 0);
      check_near("bytes: 3 token, 11 data, 1 handshake", n, 15, 0);

      // These three are known OUTSIDE this file: the SETUP token that opens
      // every enumeration goes out as 2D 00 10. A wrong CRC5 shows up here
      // and nowhere else, the generator and the decoder agreeing being no
      // evidence about either.
      check_near("SETUP token PID", lr.bytes[0], 0x2D, 0);
      check_near("...address and endpoint both zero", lr.bytes[1], 0x00, 0);
      check_near("...and the CRC5 the wire really carries", lr.bytes[2], 0x10, 0);

      check_near("DATA0 PID", lr.bytes[3], 0xC3, 0);
      check_near("bmRequestType", lr.bytes[4], 0x80, 0);
      check_near("bRequest GET_DESCRIPTOR", lr.bytes[5], 0x06, 0);
      check_near("wValue: device descriptor", lr.bytes[7], 0x01, 0);
      check_near("wLength", lr.bytes[10], 0x40, 0);
      check_near("eight payload bytes, found by the CRC16",
          a->pkt[1].ndata, 8, 0);
      check_near("ACK", lr.bytes[14], 0xD2, 0);

      usb_byte_label(a, 0, lr.bytes[0], lab, sizeof(lab));
      check_near("the PID is named", !strcmp(lab, "SETUP"), 1, 0);
      usb_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("the token reads as one thing",
          !strcmp(lab, "adr0 ep0"), 1, 0);
      usb_byte_label(a, 14, lr.bytes[14], lab, sizeof(lab));
      check_near("and the handshake", !strcmp(lab, "ACK"), 1, 0);

      // Two bytes that are one eleven-bit field and a checksum are shown as
      // one, the way a CAN identifier and a 1-Wire temperature are
      usb_group_at(a, 1, &gs, &gl);
      check_near("the token groups", gs, 1, 0);
      check_near("...as two", gl, 2, 0);
      usb_group_at(a, 13, &gs, &gl);
      check_near("so does the CRC16", gs, 12, 0);
      check_near("...as two", gl, 2, 0);
      usb_group_at(a, 5, &gs, &gl);
      check_near("a payload byte stands alone", gl, 1, 0);
      usb_group_at(a, 0, &gs, &gl);
      check_near("...and so does a PID", gl, 1, 0);
    }

    // The other external anchor, and the one that pins the CRC16's seed and
    // its inversion: a data packet with no payload at all is C3 00 00.
    printf("usb: a zero-length DATA0 is C3 00 00:\n");
    {
      UsbGen g;
      char lab[16];

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);
      usb_g_token(&g, 0x69, 5, 1, 4.0);        // IN, address 5, endpoint 1
      usb_g_data(&g, 0xC3, NULL, 0, 20.0);

      usb_decode(buf, SIZE, 0, 8, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("IN token", lr.bytes[0], 0x69, 0);
      check_near("DATA0 PID", lr.bytes[3], 0xC3, 0);
      check_near("the CRC over nothing is zero", lr.bytes[4], 0x00, 0);
      check_near("...both bytes of it", lr.bytes[5], 0x00, 0);
      check_near("and no payload came with it", a->pkt[1].ndata, 0, 0);
      check_near("five bytes in all", lr.count, 6, 0);

      usb_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("the address and endpoint", !strcmp(lab, "adr5 ep1"), 1, 0);
    }

    // Low speed is 666 ns to a bit, so it decodes at any timebase the record
    // is long enough for - and the level that means J is the opposite one,
    // which is the only thing that tells D+ from D- on a single wire.
    printf("usb low speed at 31.25 MS/s:\n");
    {
      UsbGen g;
      char lab[16];

      usb_g(&g, buf, SIZE, 32.0, 1.5e6, 0); // low speed, J low: still D+
      usb_g_sof(&g, 1234, 6.0);
      usb_g_token(&g, 0x69, 3, 1, 5.0);     // IN, address 3, endpoint 1
      usb_g_hs(&g, 0x5A, 40.0);             // NAK: the device has nothing

      int n = logic_decode(buf, SIZE, 0, 32, PROTO_AUTO, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("auto takes it", lr.proto, PROTO_USB, 0);
      check_near("low speed", lr.rate, 1500000, 0);
      check_near("...and J low at low speed is still D+", a->dplus, 1, 0);
      check_near("three packets", a->packets, 3, 0);
      check_near("a start of frame among them", a->sof, 1, 0);
      check_near("seven bytes", n, 7, 0);
      check_near("SOF PID", lr.bytes[0], 0xA5, 0);
      check_near("IN token", lr.bytes[3], 0x69, 0);
      check_near("NAK", lr.bytes[6], 0x5A, 0);

      usb_byte_label(a, 2, lr.bytes[2], lab, sizeof(lab));
      check_near("the frame number reads out",
          !strcmp(lab, "frame 1234"), 1, 0);
    }

    printf("usb: the other wire of the pair carries the same bits:\n");
    {
      UsbGen g;
      static const uint8_t pay[4] = { 0xDE, 0xAD, 0xBE, 0xEF };

      usb_g(&g, buf, SIZE, 8.0, 12e6, 0);   // full speed with J LOW: that is D-
      usb_g_token(&g, 0xE1, 12, 2, 4.0);    // OUT
      usb_g_data(&g, 0x4B, pay, 4, 20.0);   // DATA1

      usb_decode(buf, SIZE, 0, 8, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("the probe is on D-", a->dplus, 0, 0);
      check_near("OUT token", lr.bytes[0], 0xE1, 0);
      check_near("address 12", a->pkt[0].addr, 12, 0);
      check_near("endpoint 2", a->pkt[0].ep, 2, 0);
      check_near("DATA1 PID", lr.bytes[3], 0x4B, 0);
      check_near("the payload is byte for byte the same",
          memcmp(lr.bytes + 4, pay, 4), 0, 0);
      check_near("both CRCs agree", a->crc_ok, 2, 0);
    }

    // On this wire the EOP has NO edge to it: SE0 is low and J is low, so the
    // line simply stays down from the last data bit into the next packet's
    // sync. Nothing marks the end of the packet at all - which is the case
    // the CRC16 search exists for, and the one a decoder written around an
    // EOP cannot read.

    printf("usb: a payload of ones, which is what stuffing is for:\n");
    {
      UsbGen g;
      // 0xFF 0xFF 0xFF is twenty-four ones in a row and costs four stuff
      // bits, so this packet's bytes are NOT eight bit times of wire each
      static const uint8_t ones[6] = { 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x81 };

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);
      usb_g_token(&g, 0x2D, 1, 0, 4.0);
      usb_g_data(&g, 0xC3, ones, 6, 20.0);

      usb_decode(buf, SIZE, 0, 8, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("six payload bytes", a->pkt[1].ndata, 6, 0);
      check_near("every one of them back",
          memcmp(lr.bytes + 4, ones, 6), 0, 0);
      check_near("the CRC agrees", a->crc_ok, 2, 0);
      // The stuff bits are wire and not data, so the packet's bytes do not
      // divide its span evenly. That is why there is no bit grid for USB.
      check_near("the packet is longer on the wire than 9 bytes",
          lr.end[11] - lr.pos[3] > 9 * 8 * 10, 1, 0);
    }

    printf("usb: six zero bits in a payload draw a sync and are not one:\n");
    {
      UsbGen g;
      // A zero is a transition, so a payload byte of 0x00 puts eight one-bit
      // runs on the wire - the exact shape a SYNC is found by, with a hold
      // either side of it. Only knowing where the packet ENDED tells them
      // apart, which is why the search for the next packet resumes behind
      // this one rather than scanning through it.
      static const uint8_t trap[6] = { 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF };

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);
      usb_g_token(&g, 0x2D, 2, 0, 4.0);
      usb_g_data(&g, 0xC3, trap, 6, 4.0);
      usb_g_hs(&g, 0xD2, 20.0);

      usb_decode(buf, SIZE, 0, 8, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("three packets, not five", a->packets, 3, 0);
      check_near("the payload is whole",
          memcmp(lr.bytes + 4, trap, 6), 0, 0);
      check_near("and the ACK behind it is still found", lr.bytes[12], 0xD2, 0);
    }

    printf("usb: a data packet whose CRC does not agree:\n");
    {
      UsbGen g;
      static const uint8_t pay[4] = { 1, 2, 3, 4 };

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);
      usb_g_token(&g, 0x2D, 7, 0, 4.0);
      usb_g_data_crc(&g, 0xC3, pay, 4, 0x1234, 20.0);

      usb_decode(buf, SIZE, 0, 8, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("the token still checks out", a->pkt[0].crc_ok, 1, 0);
      check_near("the data packet does not", a->pkt[1].crc_ok, 0, 0);
      // Reported anyway, the way SWD reports a transaction whose data parity
      // failed: hiding it would hide the one fault worth seeing
      check_near("...and is reported anyway", a->pkt[1].ndata, 4, 0);
      check_near("with the bytes that were on the wire", lr.bytes[4], 1, 0);
      check_near("...to the last of them", lr.bytes[7], 4, 0);
      check_near("one error counted", lr.errors, 1, 0);
      // One good CRC is still one good CRC, so the record is USB
      check_near("the record is still USB", a->sure, 1, 0);
    }

    // The claim the whole full-speed case rests on: at 10.4 samples to a bit
    // there is about half a bit of slack in every run length, so an edge
    // measured a sample or two out of place still rounds to the right number
    // of bit times. Worth a test rather than an argument.
    printf("usb full speed with jitter on every edge and noise on the line:\n");
    {
      UsbGen g;
      static const uint8_t pay[8] =
          { 0x12, 0xFF, 0x00, 0xA5, 0x5A, 0xFF, 0xFF, 0x7E };

      srand(20260729);

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);
      g.jit = 1.5;                          // +-1.5 samples on every edge...
      g.noise = 14;                         // ...and +-14 counts on the level
      usb_g_token(&g, 0x2D, 21, 3, 4.0);
      usb_g_data(&g, 0xC3, pay, 8, 4.0);
      usb_g_hs(&g, 0xD2, 20.0);

      usb_decode(buf, SIZE, 0, 8, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("three packets even so", a->packets, 3, 0);
      check_near("both CRCs still agree", a->crc_ok, 2, 0);
      check_near("address 21", a->pkt[0].addr, 21, 0);
      check_near("endpoint 3", a->pkt[0].ep, 3, 0);
      check_near("the payload is byte for byte the same",
          memcmp(lr.bytes + 4, pay, 8), 0, 0);
      check_near("and no errors were invented", lr.errors, 0, 0);
    }

    // The tightest legal spacing, on the wire where it hurts: at full speed
    // on D+ the idle is HIGH, so between two packets there is no long hold
    // anywhere - three bit times of J and the next sync begins. The reader
    // therefore runs straight out of one packet and into the next, and the
    // only thing that says where the first one stopped is its CRC16.
    printf("usb: back-to-back packets with the minimum gap:\n");
    {
      UsbGen g;
      static const uint8_t pay[3] = { 0x01, 0x02, 0x03 };

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);
      usb_g_token(&g, 0x2D, 4, 0, 2.0);     // two bit times: the floor
      usb_g_data(&g, 0xC3, pay, 3, 2.0);
      usb_g_hs(&g, 0xD2, 2.0);
      usb_g_token(&g, 0x69, 4, 0, 20.0);    // IN, and room to breathe after it

      usb_decode(buf, SIZE, 0, 8, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("all four packets", a->packets, 4, 0);
      check_near("three CRCs", a->crc_ok, 3, 0);
      check_near("SETUP", lr.bytes[0], 0x2D, 0);
      check_near("DATA0", lr.bytes[3], 0xC3, 0);
      check_near("...whose payload stopped where the CRC16 said",
          a->pkt[1].ndata, 3, 0);
      check_near("ACK", lr.bytes[9], 0xD2, 0);
      check_near("and the IN behind it", lr.bytes[10], 0x69, 0);
    }

    printf("usb: the record ends inside a packet:\n");
    {
      UsbGen g;
      static const uint8_t pay[16] =
          { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);
      usb_g_token(&g, 0x2D, 9, 0, 4.0);
      g.t = SIZE - 40 * g.sps_bit;          // ...and room for part of one more
      usb_g_data(&g, 0xC3, pay, 16, 4.0);

      usb_decode(buf, SIZE, 0, 8, &scratch, &lr);
      const UsbAnalysis *a = usb_analysis();

      check_near("two packets", a->packets, 2, 0);
      check_near("the token checks out", a->pkt[0].crc_ok, 1, 0);
      check_near("the data packet is cut", a->pkt[1].cut, 1, 0);
      // ...and carries no CRC, because the bytes that would be one never
      // arrived. Calling the last two of what DID arrive a checksum would be
      // inventing a field out of a window that closed early.
      check_near("...so no CRC is claimed for it", a->pkt[1].has_crc, 0, 0);
      check_near("what arrived is still reported", a->pkt[1].ndata > 0, 1, 0);
      check_near("starting at the first payload byte", lr.bytes[4], 1, 0);
      // One good CRC is one good CRC: the record is USB whatever the window did
      check_near("the record is still USB", a->sure, 1, 0);
    }

    printf("usb full speed needs the fastest timebase:\n");
    {
      UsbGen g;
      static const uint8_t setup[8] =
          { 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00 };

      usb_g(&g, buf, SIZE, 8.0, 12e6, 1);
      usb_g_token(&g, 0x2D, 0, 0, 4.0);
      usb_g_data(&g, 0xC3, setup, 8, 20.0);
      check_near("at 125 MS/s it reads: 10.4 samples to a bit",
          usb_decode(buf, SIZE, 0, 8, &scratch, &lr) > 0, 1, 0);

      // The same traffic sampled four times slower is 2.6 samples to a bit,
      // and there is no reading of it - so none is offered. A decoder that
      // answered here would be inventing the difference between one bit time
      // and two out of the edge quantisation.
      usb_g(&g, buf, SIZE, 32.0, 12e6, 1);
      usb_g_token(&g, 0x2D, 0, 0, 4.0);
      usb_g_data(&g, 0xC3, setup, 8, 20.0);
      check_near("at 31.25 MS/s it does not",
          usb_decode(buf, SIZE, 0, 32, &scratch, &lr), 0, 0);
    }

    printf("usb rejects the rest:\n");
    {
      double half = 0.5;

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST", 20000.0);
      check_near("uart is not usb",
          usb_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not usb",
          usb_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      // The adversarial one: a 6 MHz square at 8 ns is one full-speed bit
      // time to a run, every run, for the whole record. Every six of them
      // are a sync pattern's six - and none of them is a sync, because a
      // sync has the line HOLDING either side of it and this never holds.
      synth(buf, SIZE, 0, fn_square, &half, 6e6, 8.0, 90.0, ZERO_POINT);
      check_near("a 6 MHz square is not usb",
          usb_decode(buf, SIZE, 0, 8, &scratch, &lr), 0, 0);

      // ...and the same at low speed's bit time
      synth(buf, SIZE, 0, fn_square, &half, 750e3, 32.0, 90.0, ZERO_POINT);
      check_near("a 750 kHz square is not usb either",
          usb_decode(buf, SIZE, 0, 32, &scratch, &lr), 0, 0);
    }

    // ============================== USB-PD =================================
    //
    // After USB this one is a relief: CC is single-ended by design, so there
    // is no pair to half-see and the end of a message is an EOP symbol on the
    // same wire rather than a line state one probe cannot reach.
    static const uint8_t TST_SOP[4] =
        { TST_K_SYNC1, TST_K_SYNC1, TST_K_SYNC1, TST_K_SYNC2 };
    static const uint8_t TST_SOPP[4] =
        { TST_K_SYNC1, TST_K_SYNC1, TST_K_SYNC3, TST_K_SYNC3 };
    static const uint8_t TST_HRST[4] =
        { TST_K_RST1, TST_K_RST1, TST_K_RST1, TST_K_RST2 };

    printf("usb-pd: the polynomial, against its published check value:\n");
    {
      // Everything below leans on the generator's CRC32 being right, and the
      // generator and the decoder agreeing proves nothing about either. This
      // does: 0xCBF43926 over "123456789" is the value this polynomial is
      // defined by, and it is the same one the decoder computes.
      check_near("crc32(\"123456789\")",
          (double)tst_pd_crc32((const uint8_t *)"123456789", 9),
          (double)0xCBF43926u, 0);
    }

    // The message anyone points a probe at CC to read - and the one that
    // motivated this decoder walking the samples itself. Five supplies is
    // about 550 level runs, and the shared split stops at 512.
    printf("usb-pd: a charger advertising five supplies, at 64 ns:\n");
    {
      PdGen g;
      char lab[16];
      int gs, gl;
      uint32_t pdo[5] =
      {
        tst_pd_fixed(5000, 3000), tst_pd_fixed(9000, 3000),
        tst_pd_fixed(12000, 3000), tst_pd_fixed(15000, 3000),
        tst_pd_fixed(20000, 5000),
      };

      pd_g(&g, buf, SIZE, 64.0, 300e3, 1);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 5, 0, 2, 1, 1), pdo, 5, 40.0);

      int n = logic_decode(buf, SIZE, 0, 64, PROTO_AUTO, &scratch, &lr);
      const PdAnalysis *a = pd_analysis();

      check_near("auto takes it", lr.proto, PROTO_PD, 0);
      check_near("300 kbit", lr.rate, 300000, 2000);
      check_near("one message", a->msgs, 1, 0);
      check_near("its CRC32 agrees", a->crc_ok, 1, 0);
      check_near("so it is sure", a->sure, 1, 0);
      check_near("...and not ambiguous", lr.ambiguous, 0, 0);
      check_near("no errors", lr.errors, 0, 0);
      check_near("bytes: header, five objects, CRC32", n, 26, 0);
      check_near("five objects", a->obj_count, 5, 0);

      // THE point of the streaming walk: this message does not fit the run
      // split every other decoder here reads, and decodes anyway
      int mid = 0;
      check_near("the shared split ran out of runs on this record",
          logic_runs(buf, SIZE, 0, &scratch, &mid), LOGIC_MAX_RUNS, 0);

      pd_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      check_near("the message is named", !strcmp(lab, "Src_Cap"), 1, 0);
      pd_byte_label(a, 5, lr.bytes[5], lab, sizeof(lab));
      check_near("the first supply is the mandatory one",
          !strcmp(lab, "5.0V 3.0A"), 1, 0);
      pd_byte_label(a, 21, lr.bytes[21], lab, sizeof(lab));
      check_near("...and the last is what the charger is sold on",
          !strcmp(lab, "20.0V 5.0A"), 1, 0);

      check_near("the capabilities obey their own ordering rules",
          a->caps_ordered, 1, 0);
      check_near("no request in this record to check against them",
          a->req_check, -1, 0);

      // Four bytes are one object and are shown as one
      pd_group_at(a, 19, &gs, &gl);
      check_near("an object groups", gs, 18, 0);
      check_near("...as four", gl, 4, 0);
      pd_group_at(a, 0, &gs, &gl);
      check_near("the header groups as two", gl, 2, 0);
      pd_group_at(a, 24, &gs, &gl);
      check_near("and the CRC32 as four", gs, 22, 0);
    }

    // The whole question and the whole answer in one record, which is what
    // "did they agree" actually needs
    printf("usb-pd: a negotiation - caps, request, and both acknowledged:\n");
    {
      PdGen g;
      char lab[16];
      uint32_t pdo[4] =
      {
        tst_pd_fixed(5000, 3000), tst_pd_fixed(9000, 3000),
        tst_pd_fixed(15000, 3000), tst_pd_fixed(20000, 5000),
      };
      // Object 4 at 5.0 A: exactly what that supply offers
      uint32_t rdo[1] = { (4u << 28) | (500u << 10) | 500u };

      pd_g(&g, buf, SIZE, 128.0, 300e3, 1);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 4, 0, 2, 1, 1), pdo, 4, 10.0);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 0, 0, 2, 0, 0), NULL, 0, 10.0);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(2, 1, 1, 2, 0, 0), rdo, 1, 10.0);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 0, 1, 2, 1, 1), NULL, 0, 20.0);

      pd_decode(buf, SIZE, 0, 128, &scratch, &lr);
      const PdAnalysis *a = pd_analysis();

      check_near("four messages", a->msgs, 4, 0);
      check_near("every CRC32 agrees", a->crc_ok, 4, 0);
      check_near("two of them were acknowledged", a->acked, 2, 0);
      check_near("the capabilities were seen", a->caps_seen, 1, 0);
      check_near("...and are in order", a->caps_ordered, 1, 0);
      check_near("the request names object 4", a->req_pos, 4, 0);
      check_near("...and it agrees with what was offered", a->req_check, 1, 0);
      check_near("which is 20 V", a->req_mv, 20000, 0);
      check_near("at 5 A", a->req_ma, 5000, 0);
      check_near("so the header says so",
          NULL != strstr(lr.info, "20.0V5.0A"), 1, 0);

      pd_byte_label(a, a->msg[1].first + 1, 0, lab, sizeof(lab));
      check_near("the acknowledgement is named",
          !strcmp(lab, "GoodCRC"), 1, 0);
      pd_byte_label(a, a->msg[2].first + 1, 0, lab, sizeof(lab));
      check_near("and the request", !strcmp(lab, "Request"), 1, 0);
      pd_byte_label(a, a->msg[2].first + 5, 0, lab, sizeof(lab));
      check_near("...which asked for #4 at 5 A",
          !strcmp(lab, "#4 5.0A"), 1, 0);
    }

    printf("usb-pd: a request for more than the supply offers:\n");
    {
      PdGen g;
      uint32_t pdo[2] =
          { tst_pd_fixed(5000, 3000), tst_pd_fixed(20000, 3000) };
      uint32_t rdo[1] = { (2u << 28) | (500u << 10) | 500u };  // 5 A of a 3 A

      pd_g(&g, buf, SIZE, 128.0, 300e3, 1);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 2, 0, 2, 1, 1), pdo, 2, 10.0);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(2, 1, 1, 2, 0, 0), rdo, 1, 20.0);

      pd_decode(buf, SIZE, 0, 128, &scratch, &lr);
      const PdAnalysis *a = pd_analysis();

      check_near("both messages check out", a->crc_ok, 2, 0);
      // Every checksum agreed and the traffic still does not add up. That is
      // the distinction worth drawing: the CRC says the bits arrived, and
      // this says the two ends were asking for different things.
      check_near("...and they still disagree", a->req_check, 0, 0);
      check_near("the header says so", NULL != strstr(lr.info, "req!"), 1, 0);
    }

    printf("usb-pd: capabilities that break their own ordering rule:\n");
    {
      PdGen g;
      // Not starting at 5 V, which every sink is entitled to
      uint32_t pdo[2] =
          { tst_pd_fixed(9000, 3000), tst_pd_fixed(20000, 5000) };

      pd_g(&g, buf, SIZE, 128.0, 300e3, 1);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 2, 0, 2, 1, 1), pdo, 2, 20.0);

      pd_decode(buf, SIZE, 0, 128, &scratch, &lr);
      const PdAnalysis *a = pd_analysis();

      check_near("the CRC32 still agrees", a->crc_ok, 1, 0);
      check_near("...and the ordering rule does not", a->caps_ordered, 0, 0);
    }

    // BMC puts the data in the transitions, so which level a run sits at
    // means nothing at all - an inverting probe must change no byte
    printf("usb-pd: the line inverted reads the same bytes:\n");
    {
      PdGen g;
      uint8_t up[32];
      int n_up;
      uint32_t pdo[2] =
          { tst_pd_fixed(5000, 3000), tst_pd_fixed(20000, 5000) };

      pd_g(&g, buf, SIZE, 64.0, 300e3, 1);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 2, 0, 2, 1, 1), pdo, 2, 20.0);
      n_up = pd_decode(buf, SIZE, 0, 64, &scratch, &lr);
      memcpy(up, lr.bytes, (size_t)n_up);

      pd_g(&g, buf, SIZE, 64.0, 300e3, 0);          // started the other way up
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 2, 0, 2, 1, 1), pdo, 2, 20.0);

      check_near("the same byte count", pd_decode(buf, SIZE, 0, 64, &scratch,
          &lr), n_up, 0);
      check_near("and the same bytes", memcmp(lr.bytes, up, (size_t)n_up),
          0, 0);
      check_near("...still confirmed", pd_analysis()->crc_ok, 1, 0);
    }

    printf("usb-pd: a message addressed to the cable, not the partner:\n");
    {
      PdGen g;
      char lab[16];
      uint32_t vdo[1] = { 0xFF008001u };

      pd_g(&g, buf, SIZE, 128.0, 300e3, 1);
      pd_g_msg(&g, TST_SOPP, tst_pd_hdr(15, 1, 0, 2, 0, 1), vdo, 1, 20.0);

      pd_decode(buf, SIZE, 0, 128, &scratch, &lr);
      const PdAnalysis *a = pd_analysis();

      check_near("the ordered set says SOP'", a->msg[0].sop, PD_SOP_P, 0);
      pd_byte_label(a, 1, lr.bytes[1], lab, sizeof(lab));
      // The tick is not decoration: the same message type to the cable and to
      // the port partner are different conversations, and the header does not
      // say which one this is
      check_near("...and the name carries the tick",
          !strcmp(lab, "'Vendor_Def"), 1, 0);
    }

    printf("usb-pd: a hard reset, which has no message behind it:\n");
    {
      PdGen g;
      uint32_t pdo[2] =
          { tst_pd_fixed(5000, 3000), tst_pd_fixed(20000, 5000) };

      pd_g(&g, buf, SIZE, 128.0, 300e3, 1);
      pd_g_preamble(&g);
      pd_g_os(&g, TST_HRST);
      pd_g_idle(&g, 30.0);
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 2, 0, 2, 1, 1), pdo, 2, 20.0);

      pd_decode(buf, SIZE, 0, 128, &scratch, &lr);
      const PdAnalysis *a = pd_analysis();

      check_near("the reset is counted", a->resets, 1, 0);
      // ...and produces no bytes, because there are none: an ordered set and
      // then silence is the whole of what a hard reset is on the wire
      check_near("the capabilities behind it still decode", a->msgs, 1, 0);
      check_near("...and check out", a->crc_ok, 1, 0);
      check_near("the header says a reset went past",
          NULL != strstr(lr.info, "rst"), 1, 0);
    }

    printf("usb-pd: a message whose CRC32 does not agree:\n");
    {
      PdGen g;
      uint32_t pdo[2] =
          { tst_pd_fixed(5000, 3000), tst_pd_fixed(20000, 5000) };

      pd_g(&g, buf, SIZE, 128.0, 300e3, 1);
      pd_g_msg_crc(&g, TST_SOP, tst_pd_hdr(1, 2, 0, 2, 1, 1), pdo, 2,
          0xDEADBEEFu, 20.0);

      check_near("it is still reported",
          pd_decode(buf, SIZE, 0, 128, &scratch, &lr) > 0, 1, 0);

      const PdAnalysis *a = pd_analysis();

      check_near("one message", a->msgs, 1, 0);
      check_near("...whose CRC32 does not agree", a->crc_ok, 0, 0);
      check_near("counted as an error", lr.errors, 1, 0);
      // Nothing in this record is confirmed, so auto mode must not take it -
      // an ordered set is twenty prescribed bits and that is a filter, not a
      // proof
      check_near("and it is not sure", a->sure, 0, 0);
      check_near("...so auto passes over it", logic_decode(buf, SIZE, 0, 128,
          PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_PD, 0, 0);
    }

    printf("usb-pd: jitter on every edge and noise on the line:\n");
    {
      PdGen g;
      uint32_t pdo[3] =
      {
        tst_pd_fixed(5000, 3000), tst_pd_fixed(9000, 3000),
        tst_pd_fixed(20000, 5000),
      };

      srand(20260730);

      pd_g(&g, buf, SIZE, 128.0, 300e3, 1);
      g.jit = 2.0;                                  // +-2 samples of 26
      g.noise = 20;
      pd_g_msg(&g, TST_SOP, tst_pd_hdr(1, 3, 0, 2, 1, 1), pdo, 3, 20.0);

      pd_decode(buf, SIZE, 0, 128, &scratch, &lr);
      const PdAnalysis *a = pd_analysis();

      check_near("the message still comes back", a->msgs, 1, 0);
      check_near("...confirmed", a->crc_ok, 1, 0);
      check_near("three objects", a->obj_count, 3, 0);
      check_near("and the last one is right", (double)a->obj[2],
          (double)tst_pd_fixed(20000, 5000), 0);

      char lab[16];

      pd_byte_label(a, 13, lr.bytes[13], lab, sizeof(lab));
      check_near("...and reads out as what it is",
          !strcmp(lab, "20.0V 5.0A"), 1, 0);
    }

    printf("usb-pd rejects the rest:\n");
    {
      double half = 0.5;

      synth_uart(buf, SIZE, 1000.0, 9600.0, "MILKV-UART-TEST", 20000.0);
      check_near("uart is not pd",
          pd_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
      check_near("sine is not pd",
          pd_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);

      // The adversarial one: a 300 kHz square at 64 ns puts HALF a unit
      // interval in every run, for the whole record. The unit time is right,
      // the run lengths are right, and it is not BMC - because BMC alternates
      // half-runs with whole ones and this never holds one for a whole bit.
      synth(buf, SIZE, 0, fn_square, &half, 300e3, 64.0, 90.0, ZERO_POINT);
      check_near("a 300 kHz square is not pd",
          pd_decode(buf, SIZE, 0, 64, &scratch, &lr), 0, 0);

      // ...and at the other unit time, where the rate gate turns it away
      // before the record is walked at all
      synth(buf, SIZE, 0, fn_square, &half, 150e3, 64.0, 90.0, ZERO_POINT);
      check_near("a 150 kHz square is not pd either",
          pd_decode(buf, SIZE, 0, 64, &scratch, &lr), 0, 0);
    }

    // ----- structured decoders must all reject a sine -----
    //
    // UART rejects it one step later than the rest. Thresholded, a sine is a
    // square wave, and a square wave IS a stream of 0x55 frames - there is no
    // reading of the samples that separates the two. So uart_decode reports
    // the frames it found and marks them ambiguous, and the dispatcher, which
    // knows whether anyone asked for UART, is what turns them down.
    synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
    uart_decode(buf, SIZE, 0, 1000, &scratch, &lr);
    check_near("sine uart is ambiguous", lr.ambiguous, 1, 0);
    check_near("sine not uart in auto", logic_decode(buf, SIZE, 0, 1000,
        PROTO_AUTO, &scratch, &lr) > 0 && lr.proto == PROTO_UART, 0, 0);
    check_near("sine not 1-wire", onewire_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    check_near("sine not ws2812", ws2812_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    check_near("sine not nec", nec_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
  }

  // ===== full-rate record window (record_window.c) =====
  //
  // The window is what makes a live record run at the ring's sample rate
  // instead of a quarter of it. Its index arithmetic decides what every
  // consumer is allowed to read, so sweep it rather than spot-check it.
  {
    enum { RING = 96 * 1024, WIN = 24 * 1024 };
    int bad_invariant = 0, placed = 0, refused = 0;

    printf("record window placement sweep:\n");

    for (int trigger = 0; trigger < RING; trigger += 61)
    {
      // The trigger sits mid-record by construction (capture.c asks the DMA
      // for CAPTURE_BUFFER_SIZE/2 + pan), but sweep it right around the ring
      // anyway - a wrap must not be special
      for (int shift = -RING/2; shift <= RING/2; shift += RING/8)
      {
        int offset = ((trigger + shift) % RING + RING) % RING;

        for (int span = 1; span < 3 * WIN; span = span * 3 + 1)
        {
          RecordWindow w;
          int lo = -span, hi = span;
          int d, tpos;

          if (!record_window_place(RING, offset, trigger, WIN, lo, hi, &w))
          {
            refused++;
            continue;
          }

          placed++;
          tpos = -w.start;

          // 1. the trigger is strictly inside, so index 0 is addressable and
          //    trigger != offset (which the ring math reads as "one past the
          //    newest sample")
          if (tpos < 1 || tpos > WIN - 1)
            bad_invariant++;

          // 2. the window covers everything the screen asked for
          if (w.start > lo || w.start + WIN - 1 < hi)
            bad_invariant++;

          // 3. the source is word-aligned: the copy moves words, and a byte's
          //    parity in the ring is the ADC it came from
          if ((w.src & 3) || w.src < 0 || w.src >= RING)
            bad_invariant++;

          // 4. every sample of the window is inside the ring's live record -
          //    counting forward from the oldest sample, none may run past the
          //    newest one
          d = ((w.src - offset) % RING + RING) % RING;

          if (d + WIN > RING)
            bad_invariant++;

          // 5. ...and the trigger lands where the window says it does
          if ((d + tpos) % RING != ((trigger - offset) % RING + RING) % RING)
            bad_invariant++;
        }
      }
    }

    check_near("placements attempted", placed > 0, 1, 0);
    check_near("wider than window refused", refused > 0, 1, 0);
    check_near("invariant violations", bad_invariant, 0, 0);

    // A range that cannot fit has to be refused, not truncated: the caller
    // falls back to the decimated whole-ring record
    {
      RecordWindow w;
      check_near("span > window refused",
          record_window_place(RING, 0, RING/2, WIN, -WIN, WIN, &w), 0, 0);
      check_near("trigger outside range refused",
          record_window_place(RING, 0, RING/2, WIN, 10, 20, &w), 0, 0);
    }
  }

  // ===== 50 MHz: full-rate window vs the 4:1 decimated record =====
  //
  // Both paths see the same ring; only the record built from it differs.
  // This is the bug the window fixes: a 50 MHz input read through a 4:1
  // decimation folds to ~11 MHz and looks entirely convincing.
  {
    enum { RING = 96 * 1024, WIN = 24 * 1024 };
    static uint8_t ring[RING], rec[WIN];
    RecordWindow w;
    int trigger = RING / 2;
    int offset = 0; // a full ring: oldest sample at 0

    printf("50 MHz input, 8 ns ring:\n");

    for (int i = 0; i < RING; i++)
      ring[i] = (uint8_t)lround(ZERO_POINT + 100.0 * sin(2 * M_PI * 50e6 * i * 8e-9));

    // Old path: every 4th sample of the whole ring, 32 ns apart
    for (int i = 0; i < WIN; i++)
      rec[i] = ring[i * 4];

    measure_run(rec, WIN, 0, 32, ZERO_POINT, &m);
    check_near("decimated 4:1 aliases", m.frequency < 16000000, 1, 0);

    // New path: a contiguous window at the ring's own rate. lo/hi are the
    // 50 ns/div geometry: 2 ns per pixel over 300 pixels, no pan.
    check_near("window placed", record_window_place(RING, offset, trigger, WIN, -40, 40, &w), 1, 0);

    for (int i = 0; i < WIN; i++)
      rec[i] = ring[(w.src + i) % RING];

    measure_run(rec, WIN, 0, 8, ZERO_POINT, &m);
    check_near("full-rate reads 50 MHz", m.frequency, 50000000, 0.5);
    check_near("full-rate periods regular", m.period_good_pct, 100, 2);

    // And the trigger still maps to the sample the ring put there
    check_near("trigger sample preserved", rec[-w.start], ring[trigger], 0);
  }

  // ===== interleave-mirror veto in the comb (fft.c) =====
  //
  // Two time-interleaved converters with residual gain mismatch put a spur
  // at exactly nyquist - f. Reproduced from hardware: a 50 MHz input at
  // 125 MS/s read "F0 = 12.5 MHz, h4 = 50 MHz" because the 12.5 spur's comb
  // outscored the real line. The veto must hand the crown back - and must
  // NOT break the mains case the comb exists for (weak fundamental, strong
  // third harmonic, no mirror relationship).
  {
    static uint8_t r[24576];
    static float fmag[FFT_BINS];
    FftAnalysis an;

    printf("interleave mirror veto:\n");

    for (int i = 0; i < 24576; i++)
    {
      double v = 60.0 * sin(2 * M_PI * 50e6 * i * 8e-9);

      if (i & 1)
        v *= 0.94; // odd converter 6% low: the spur source

      r[i] = (uint8_t)lround(ZERO_POINT + v);
    }

    fft_spectrum_decim(r, 24576, 0, 1, fmag);
    fft_analyze(fmag, 8, &an);
    check_near("50 MHz not dethroned by spur", an.fundamental, 50000000, 1);

    // Mains-like: 50 Hz weak, 150 Hz dominant, 1 ms sampling
    for (int i = 0; i < 24576; i++)
      r[i] = (uint8_t)lround(ZERO_POINT + 20.0 * sin(2 * M_PI * 50 * i * 1e-3)
          + 60.0 * sin(2 * M_PI * 150 * i * 1e-3));

    fft_spectrum_decim(r, 24576, 0, 1, fmag);
    fft_analyze(fmag, 1000000, &an);
    check_near("mains comb still wins", an.fundamental, 50, 6);

    // ===== folded harmonics (the 33.33 MHz case from hardware) =====
    //
    // An ugly divider output at 33.33 MHz carries a 2nd harmonic STRONGER
    // than the fundamental. At 125 MS/s that harmonic (66.67) is observed
    // folded at 58.33, the 4th at 8.33 - and every alias lands on a lattice
    // of 8.33 MHz, so a fold-blind comb crowned "F0 = 8.33 MHz, THD 980%".
    // The alias-aware comb must hand it back to 33.33.
    printf("folded harmonics:\n");

    for (int i = 0; i < 24576; i++)
    {
      double t = i * 8e-9;
      double v = ZERO_POINT
          + 35.0 * sin(2 * M_PI * 33.333e6 * t)
          + 55.0 * sin(2 * M_PI * 66.667e6 * t + 0.9)   // h2, dominant
          + 8.0  * sin(2 * M_PI * 133.333e6 * t + 0.3); // h4

      r[i] = (uint8_t)lround(v < 0 ? 0 : v > 255 ? 255 : v);
    }

    fft_spectrum_decim(r, 24576, 0, 1, fmag);
    fft_analyze(fmag, 8, &an);
    check_near("F0 is the input, not the lattice", an.fundamental, 33333000, 1);

    // The dominant folded peak must be labelled h2, not h7
    {
      int h_of_top = -1;

      for (int i = 0; i < an.count; i++)
      {
        if (an.peak[i].rel_db == 0.0f)
          h_of_top = an.peak[i].harmonic;
      }

      check_near("58.3 MHz peak labelled h2", h_of_top, 2, 0);
    }

    // ===== the {f, 2f} fold ambiguity (25 MHz case from hardware) =====
    //
    // A clean 25 MHz square at 125 MS/s shows 25 (dominant) and 50 (weak:
    // h2 residue + the folded 3rd). "f0=25, in-band h2" and "f0=50, folded
    // h2" explain the same two peaks with the same energy - and the tie
    // went to the higher candidate: the instrument read "F0 = 50.01 MHz,
    // THD 999.9%". The folded-match discount must break exactly this tie.
    printf("fold ambiguity:\n");

    for (int i = 0; i < 24576; i++)
    {
      double t = i * 8e-9;
      double v = ZERO_POINT
          + 60.0 * sin(2 * M_PI * 25e6 * t)
          + 4.5  * sin(2 * M_PI * 50e6 * t + 0.4); // -22.5 dB, as measured

      r[i] = (uint8_t)lround(v < 0 ? 0 : v > 255 ? 255 : v);
    }

    fft_spectrum_decim(r, 24576, 0, 1, fmag);
    fft_analyze(fmag, 8, &an);
    check_near("F0 is 25, not 50", an.fundamental, 25000000, 1);
  }

  // ===== period jitter statistics =====
  //
  // FULL periods must alternate (195, 205, 195, ...) - alternating
  // half-periods collapses into constant full periods and zero jitter.
  {
    static uint8_t jb[24000];
    Measure m;
    int idx = 0;

    printf("jitter statistics:\n");

    for (int k = 0; k < 120; k++)
    {
      int period = (k & 1) ? 205 : 195;

      for (int i = 0; i < period; i++)
        jb[idx++] = (i < period / 2) ? 190 : 66;
    }

    measure_run(jb, idx, 0, 1000, ZERO_POINT, &m);

    check_near("period min", m.period_min_ns, 195000, 1);
    check_near("period max", m.period_max_ns, 205000, 1);
    check_near("jitter rms (sigma 5us)", m.jitter_rms_ps, 5000000, 3);
    check_near("jitter pp (10us)", m.jitter_pp_ps, 10000000, 2);
  }

  // ===== glitch finder: narrowest complete pulse =====
  {
    static uint8_t gb[8000];
    static uint8_t wb[1000];
    int w = 0, cnt = 0, pos;
    bool high = false;

    printf("min pulse finder:\n");

    // A clean 20/20 square: every complete run ties at 20 samples. The tie
    // count is what tells the caller this is the signal itself, not a
    // glitch. First complete run is the low at [20..39] (the leading high
    // is partial); low runs complete = 199 of the 400 total.
    for (int i = 0; i < 8000; i++)
      gb[i] = ((i / 20) & 1) ? 66 : 190;

    pos = measure_find_min_pulse(gb, 8000, 0, 100, 156, &w, &high, &cnt);
    check_near("square: pos", pos, 30, 0);
    check_near("square: width", w, 20, 0);
    check_near("square: polarity low", high ? 1 : 0, 0, 0);
    check_near("square: tie count", cnt, 199, 0);

    // A 3-sample runt buried inside one low half-period, and the record
    // read through a nonzero ring offset: array 2025 = time 3026's run
    gb[2025] = gb[2026] = gb[2027] = 190;

    pos = measure_find_min_pulse(gb, 8000, 7000, 100, 156, &w, &high, &cnt);
    check_near("runt: pos", pos, 3026, 0);
    check_near("runt: width", w, 3, 0);
    check_near("runt: polarity high", high ? 1 : 0, 1, 0);
    check_near("runt: count is 1", cnt, 1, 0);

    // A pulse that physically wraps the array boundary must read as one
    // contiguous 4-sample run through the ring seam
    memset(wb, 66, sizeof(wb));
    wb[998] = wb[999] = wb[0] = wb[1] = 190;

    pos = measure_find_min_pulse(wb, 1000, 500, 100, 156, &w, &high, &cnt);
    check_near("wrap: pos", pos, 500, 0);
    check_near("wrap: width", w, 4, 0);

    // A flat record never completes two transitions
    memset(wb, 128, sizeof(wb));
    check_near("flat: -1",
        measure_find_min_pulse(wb, 1000, 0, 100, 156, NULL, NULL, NULL), -1, 0);
  }

  // ===== trend logger ring =====
  {
    const TrendPoint *pts;

    printf("trend logger:\n");
    trend_reset();

    // Plain appends: one point per push while the span is 1
    for (int i = 0; i < 10; i++)
      trend_push(1000 + i, 100 + i, (i & 1) ? -1 : 500);

    pts = trend_data();
    check_near("append: count", trend_count(), 10, 0);
    check_near("append: span", trend_seconds_per_point(), 1, 0);
    check_near("append: freq[9]", pts[9].freq, 1009, 0);
    check_near("append: duty n/a kept", pts[1].duty, -1, 0);
    check_near("append: duty valid", pts[0].duty, 500, 0);

    // Fill to the brim: the 300th push compresses 2:1 in place
    for (int i = 10; i < 300; i++)
      trend_push(1000 + i, 100, 500);

    check_near("compress: count", trend_count(), 150, 0);
    check_near("compress: span", trend_seconds_per_point(), 2, 0);
    check_near("compress: pair mean", pts[5].freq, (1010 + 1011) / 2, 0);

    // After compression a point needs span pushes: one push adds nothing,
    // the second appends the bucket average
    trend_push(2000, 100, 500);
    check_near("bucket: not yet", trend_count(), 150, 0);
    trend_push(2010, 100, 500);
    check_near("bucket: appended", trend_count(), 151, 0);
    check_near("bucket: averaged", pts[150].freq, 2005, 0);

    // A -1 duty in a compressed pair must not poison the valid one
    trend_reset();

    for (int i = 0; i < 300; i++)
      trend_push(100, 100, (i == 0) ? 400 : -1);

    check_near("duty: survivor kept", trend_data()[0].duty, 400, 0);
    check_near("duty: absent stays -1", trend_data()[1].duty, -1, 0);

    trend_reset();
    check_near("reset: empty", trend_count(), 0, 0);
  }

  // ============================ sticky shift ============================
  //
  // buttons.c reports the whole debounced key state on every change, not one
  // key event, so SHIFT is set in the chord itself and again when the other
  // key is released. Counting those towards the double click armed sticky
  // shift off a single later tap, and the key after it went through shifted
  // with nothing to explain it - which is what these first cases pin down.
  printf("sticky shift:\n");
  {
    config.shift_mode_enabled = true;
    config.shift_hold_lock = false; // the double click on its own
    config.key_remapping_enabled = false;
    g_input_now = 0;
    input_init();

    // A plain SHIFT+LEFT: press SHIFT, press LEFT, release LEFT, release SHIFT
    g_input_now += 10;   input_translate(BTN_SHIFT);
    g_input_now += 100;  input_translate(BTN_SHIFT | BTN_LEFT);
    g_input_now += 100;  input_translate(BTN_SHIFT);
    g_input_now += 100;  input_translate(0);
    check_near("chord does not arm", shift_mode_is_active(), 0, 0);

    // ...and neither does one deliberate tap after it
    g_input_now += 100;  input_translate(BTN_SHIFT);
    g_input_now += 50;   input_translate(0);
    check_near("tap after a chord does not arm", shift_mode_is_active(), 0, 0);

    g_input_now += 100;
    check_near("so the next key is unshifted",
        !(input_translate(BTN_UP) & BTN_SHIFT), 1, 0);
    g_input_now += 50;   input_translate(0);

    // Two solo taps inside the window: the double click the feature is for
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    g_input_now += 60;   input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    check_near("two solo taps arm", shift_mode_is_active(), 1, 0);

    g_input_now += 100;
    check_near("the next key is shifted",
        (input_translate(BTN_LEFT) & BTN_SHIFT) != 0, 1, 0);
    check_near("and it disarms", shift_mode_is_active(), 0, 0);

    // Holding that key has to keep the shift: acting shifted once and then
    // unshifted means the arrows change the timebase and then start panning
    g_input_now += 100;
    check_near("auto-repeat stays shifted",
        (input_translate(BTN_LEFT | BTN_REPEAT) & BTN_SHIFT) != 0, 1, 0);
    g_input_now += 100;  input_translate(0);
    g_input_now += 100;
    check_near("a press after the release is not",
        !(input_translate(BTN_LEFT) & BTN_SHIFT), 1, 0);
    g_input_now += 50;   input_translate(0);

    // Too slow, and it is two separate taps
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    g_input_now += 900;  input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    check_near("taps outside the window do not arm",
        shift_mode_is_active(), 0, 0);

    // Holding SHIFT down is one press however many repeats it generates
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(BTN_SHIFT | BTN_REPEAT);
    g_input_now += 60;   input_translate(BTN_SHIFT | BTN_REPEAT);
    g_input_now += 60;   input_translate(0);
    check_near("held SHIFT does not arm itself",
        shift_mode_is_active(), 0, 0);

    // A second double click cancels
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    g_input_now += 60;   input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    check_near("armed again", shift_mode_is_active(), 1, 0);
    g_input_now += 60;   input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    g_input_now += 60;   input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    check_near("a second double click cancels",
        shift_mode_is_active(), 0, 0);

    config.shift_mode_enabled = false;
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    g_input_now += 60;   input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    check_near("nothing arms while the feature is off",
        shift_mode_is_active(), 0, 0);
  }

  // ========================== shift hold lock ===========================
  //
  // The other way in: hold SHIFT on its own and it latches, and then stays on
  // for every key until SHIFT is tapped again. The hold is timed off the
  // auto-repeat events, so the cases below feed them the way buttons.c does.
  printf("shift hold lock:\n");
  {
    config.shift_mode_enabled = true;
    config.shift_hold_lock = true;
    config.key_remapping_enabled = false;
    g_input_now = 0;
    input_init();

    // A short hold is just a tap
    g_input_now += 10;   input_translate(BTN_SHIFT);
    g_input_now += 250;  input_translate(BTN_SHIFT | BTN_REPEAT);
    g_input_now += 100;  input_translate(0);
    check_near("a short hold does not latch", shift_mode_is_locked(), 0, 0);

    // Past the threshold it does, while the key is still down
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 250;  input_translate(BTN_SHIFT | BTN_REPEAT);
    check_near("not yet at 250 ms", shift_mode_is_locked(), 0, 0);
    g_input_now += 500;  input_translate(BTN_SHIFT | BTN_REPEAT);
    check_near("latched at 750 ms", shift_mode_is_locked(), 1, 0);
    g_input_now += 100;  input_translate(0);
    check_near("and it survives the release", shift_mode_is_locked(), 1, 0);

    // Latched means every key, and every repeat of it
    g_input_now += 100;
    check_near("first key shifted",
        (input_translate(BTN_UP) & BTN_SHIFT) != 0, 1, 0);
    g_input_now += 100;
    check_near("its repeat too",
        (input_translate(BTN_UP | BTN_REPEAT) & BTN_SHIFT) != 0, 1, 0);
    g_input_now += 100;  input_translate(0);
    g_input_now += 100;
    check_near("and the key after it",
        (input_translate(BTN_LEFT) & BTN_SHIFT) != 0, 1, 0);
    g_input_now += 100;  input_translate(0);
    check_near("still latched", shift_mode_is_locked(), 1, 0);

    // A tap switches it off
    g_input_now += 100;  input_translate(BTN_SHIFT);
    check_near("a tap releases the latch", shift_mode_is_locked(), 0, 0);

    // ...and holding that same press on must not put it straight back
    g_input_now += 800;  input_translate(BTN_SHIFT | BTN_REPEAT);
    check_near("the releasing hold does not re-latch",
        shift_mode_is_locked(), 0, 0);
    g_input_now += 100;  input_translate(0);

    g_input_now += 1000;
    check_near("the next key is unshifted",
        !(input_translate(BTN_UP) & BTN_SHIFT), 1, 0);
    g_input_now += 50;   input_translate(0);

    // Dwelling on SHIFT before the other key of a chord is how a chord gets
    // typed; the latch it crossed on the way has to be taken back
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 800;  input_translate(BTN_SHIFT | BTN_REPEAT);
    check_near("the slow chord latched on the way",
        shift_mode_is_locked(), 1, 0);
    g_input_now += 100;
    check_near("the chord itself is shifted",
        (input_translate(BTN_SHIFT | BTN_LEFT) & BTN_SHIFT) != 0, 1, 0);
    check_near("and it takes the latch back", shift_mode_is_locked(), 0, 0);
    g_input_now += 100;  input_translate(BTN_SHIFT);
    g_input_now += 100;  input_translate(0);
    g_input_now += 1000;
    check_near("so nothing is left shifted",
        !(input_translate(BTN_UP) & BTN_SHIFT), 1, 0);
    g_input_now += 50;   input_translate(0);

    // The double click still means one key, not a latch
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    g_input_now += 60;   input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    check_near("double click arms", shift_mode_is_active(), 1, 0);
    check_near("but does not latch", shift_mode_is_locked(), 0, 0);
    g_input_now += 100;  input_translate(BTN_LEFT);
    g_input_now += 100;  input_translate(0);
    g_input_now += 100;
    check_near("and it is spent after that key",
        !(input_translate(BTN_UP) & BTN_SHIFT), 1, 0);
    g_input_now += 50;   input_translate(0);

    // Each half switches off on its own
    config.shift_hold_lock = false;
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 800;  input_translate(BTN_SHIFT | BTN_REPEAT);
    g_input_now += 100;  input_translate(0);
    check_near("no latch with the hold off", shift_mode_is_locked(), 0, 0);

    config.shift_hold_lock = true;
    config.shift_mode_enabled = false;
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    g_input_now += 60;   input_translate(BTN_SHIFT);
    g_input_now += 60;   input_translate(0);
    check_near("no double click with the sticky off",
        shift_mode_is_active(), 0, 0);
    g_input_now += 1000; input_translate(BTN_SHIFT);
    g_input_now += 800;  input_translate(BTN_SHIFT | BTN_REPEAT);
    check_near("the hold still latches", shift_mode_is_locked(), 1, 0);
    g_input_now += 100;  input_translate(0);

    // Both off: whatever was latched goes with them
    config.shift_hold_lock = false;
    g_input_now += 100;  input_translate(BTN_SHIFT);
    check_near("both off clears the latch", shift_mode_is_locked(), 0, 0);
    g_input_now += 100;  input_translate(0);
  }

  // ============================= key remap ==============================
  //
  // The translation is a table of "this key acts as that one", and the cases
  // that matter are the ones where the table points at itself: a swap, and a
  // chain. Rewriting the bits in place got both wrong, because a key whose
  // button another key had just been mapped onto had that bit cleared again
  // when its own turn in the table came round.
  printf("key remap:\n");
  {
    config.shift_mode_enabled = false;
    config.key_remapping_enabled = false;
    key_remap_reset();
    input_init();

    // Off: the table is not consulted at all
    key_remap_set(key_remap_index_of(BTN_F1), BTN_AUTO);
    check_near("disabled: F1 stays F1", input_translate(BTN_F1), BTN_F1, 0);

    config.key_remapping_enabled = true;
    check_near("enabled: F1 acts as AUTO", input_translate(BTN_F1), BTN_AUTO, 0);
    check_near("AUTO itself is untouched", input_translate(BTN_AUTO), BTN_AUTO, 0);

    // The identity costs nothing and is what an untouched config holds
    key_remap_reset();
    check_near("reset: default table", key_remap_is_default(), 1, 0);
    check_near("reset: F1 is F1 again", input_translate(BTN_F1), BTN_F1, 0);

    for (int i = 0; i < key_remap_count(); i++)
    {
      uint32_t key = key_remap_button(i);

      if (input_translate(key) != (int)key)
        check_near("default table is the identity", i, -1, 0);
    }

    // A swap, both keys held at once
    key_remap_set(key_remap_index_of(BTN_F1), BTN_SAVE);
    key_remap_set(key_remap_index_of(BTN_SAVE), BTN_F1);
    check_near("swap: F1 alone", input_translate(BTN_F1), BTN_SAVE, 0);
    check_near("swap: SAVE alone", input_translate(BTN_SAVE), BTN_F1, 0);
    check_near("swap: both held", input_translate(BTN_F1 | BTN_SAVE),
        BTN_F1 | BTN_SAVE, 0);

    // A chain: F1 lands on F2's button while F2 is pointed elsewhere. Order
    // of the table must not decide the outcome.
    key_remap_reset();
    key_remap_set(key_remap_index_of(BTN_F1), BTN_F2);
    key_remap_set(key_remap_index_of(BTN_F2), BTN_TRIG);
    check_near("chain: F1 alone", input_translate(BTN_F1), BTN_F2, 0);
    check_near("chain: both held", input_translate(BTN_F1 | BTN_F2),
        BTN_F2 | BTN_TRIG, 0);

    // A chord holding a system key remaps the rest of itself, and the system
    // key comes through as it was
    key_remap_reset();
    key_remap_set(key_remap_index_of(BTN_F1), BTN_AUTO);
    check_near("SHIFT+F1 keeps SHIFT", input_translate(BTN_SHIFT | BTN_F1),
        BTN_SHIFT | BTN_AUTO, 0);
    check_near("MENU is never rewritten", input_translate(BTN_MENU), BTN_MENU, 0);
    check_near("repeats survive translation",
        input_translate(BTN_F1 | BTN_REPEAT), BTN_AUTO | BTN_REPEAT, 0);

    // A key turned off drops out of the state and takes nothing with it
    key_remap_set(key_remap_index_of(BTN_F1), 0);
    check_near("off: F1 does nothing", input_translate(BTN_F1), 0, 0);
    check_near("off: the rest of the chord stands",
        input_translate(BTN_F1 | BTN_EDGE), BTN_EDGE, 0);
    check_near("off is not the default", key_remap_is_default(), 0, 0);

    // Only a remappable key can be a target: a system key is not one
    key_remap_reset();
    key_remap_set(key_remap_index_of(BTN_F1), BTN_MENU);
    check_near("MENU is refused as a target", input_translate(BTN_F1), BTN_F1, 0);

    // While a capture is open the buttons arrive as they were pressed, or
    // the editor would record what the key already acts as
    key_remap_set(key_remap_index_of(BTN_F1), BTN_AUTO);
    input_capture_set(true);
    check_near("capture: raw F1", input_translate(BTN_F1), BTN_F1, 0);
    input_capture_set(false);
    check_near("capture off: F1 acts as AUTO again",
        input_translate(BTN_F1), BTN_AUTO, 0);

    // The table the previous firmware wrote had a twelfth entry, so every
    // key from index five down reads one place out. It only ever held the
    // identity, so it is recognised and dropped rather than migrated.
    static const uint32_t old_table[12] =
    {
      BTN_F1, BTN_F2, BTN_SAVE, BTN_AUTO, BTN_AC_DC, BTN_1X_10X,
      BTN_STOP, BTN_EDGE, BTN_50P, BTN_TRIG_UP, BTN_TRIG_DOWN, BTN_TRIG,
    };

    key_remap_reset();
    memcpy(config.key_mapping, old_table, sizeof(old_table));
    input_init();
    check_near("old table is dropped", key_remap_is_default(), 1, 0);
    check_near("old table leaves no tail", config.key_mapping[11], 0, 0);
    check_near("EDGE does not come up as STOP",
        input_translate(BTN_EDGE), BTN_EDGE, 0);

    config.key_remapping_enabled = false;
    key_remap_reset();
  }

  printf("\n%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED", g_failures);
  return g_failures ? 1 : 0;
}
