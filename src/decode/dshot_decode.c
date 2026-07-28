/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DShot - the digital throttle link between a flight controller and an ESC.
 * It replaced the analogue servo pulse on multirotors for the obvious reason:
 * a pulse width is a voltage-shaped number and a bit stream is a number.
 *
 * A frame is sixteen bits, most significant first, and nothing else:
 *
 *   11 bits  throttle: 0 disarms, 1..47 are commands, 48..2047 is the range
 *    1 bit   telemetry request
 *    4 bits  CRC
 *
 * Every bit is the SAME length and the duty carries the value: high for 3/8
 * of the bit is a zero, high for 3/4 of it is a one. The name is the rate in
 * kbit/s, so DShot600 is a 1.67 us bit and DShot150 a 6.67 us one, and the
 * frame is followed by an idle line until the next control loop comes round.
 *
 * There is no sync field and no preamble. What delimits a frame is the idle
 * on both sides of it, and what confirms it is the CRC:
 *
 *   crc = (d ^ (d >> 4) ^ (d >> 8)) & 0x0F,  d = the 12 bits above it
 *
 * Bidirectional DShot - the variant where the ESC answers with its eRPM -
 * inverts the whole line AND the CRC. Both are recognised; the answer itself
 * is GCR-encoded on the same wire and is not decoded here.
 *
 * The signal this has to be told apart from is not the infrared or the radio
 * ones - it is WS2812, and the resemblance is uncomfortably close. Both hold
 * a constant bit period and put the value in the duty, and a WS2812 bit is
 * 1.25 us against DShot600's 1.67. Three things separate them, and the
 * decoder uses all three:
 *
 *   - the DUTIES differ. WS2812 sends 28 % for a zero and 56 % for a one,
 *     DShot 37.5 % and 75 %. The windows here exclude 56 %, so a WS2812 one
 *     is not a DShot bit of either kind;
 *   - a DShot frame is SIXTEEN bits with the line idle on both sides of it.
 *     A WS2812 byte is followed by the next byte, not by an idle line, so
 *     there is nowhere in a strip's frame that sixteen bits end;
 *   - and the CRC. A black strip sends nothing but zeros, which is also what
 *     a disarmed ESC sends, and the two are the same waveform - so the frame
 *     boundary is what settles that one, not the arithmetic.
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
#define DSHOT_BITS         16

// DShot150 through DShot1200 is a 6.67 us bit down to a 0.83 us one. The
// window is wider at both ends so an unusual rate still reads.
#define DSHOT_T_MIN_NS    500
#define DSHOT_T_MAX_NS  15000

// The two duties are 37.5 % and 75 %, so telling them apart needs the eighth
// of a bit between them to be more than a sample or two wide
#define DSHOT_MIN_SAMPLES   8

// Duty windows, in percent of the bit period. The gap between them is where
// WS2812's 56 % one-bit falls, and leaving that gap open is the point: a
// pulse of half a bit is not a DShot bit of either kind, and rounding it to
// the nearer one is how a strip of addressable LEDs turns into a throttle
// command.
#define DSHOT_ZERO_MIN_PCT 20
#define DSHOT_ZERO_MAX_PCT 52
#define DSHOT_ONE_MIN_PCT  62
#define DSHOT_ONE_MAX_PCT  92

// How far a bit's period may sit from the first one's
#define DSHOT_PERIOD_TOL_PCT  20

// The idle in front of the frame and behind it, in bit times. Inside a frame
// the longest low is five eighths of a bit, so anything past one and a half
// is a boundary; the trailing side is held to two because that is the check
// that a strip of LEDs cannot pass.
#define DSHOT_GAP_LEAD_X10   15
#define DSHOT_GAP_TAIL_X10   20

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_dshot (g_logic_analysis.dshot)

// The rates anybody configures, in kbit/s. A measured rate within a few
// percent of one of these IS that one: "DShot600" is what the bus is,
// "DShot591" is what this record's sampling did to it.
static const int g_dshot_rates[] = { 150, 300, 600, 1200 };

// Throttle values 1..47 are not throttle at all. These are the ones a person
// actually meets on a bench; anything else in the range shows its number.
static const struct { uint16_t v; const char *name; } g_dshot_cmd[] =
{
  {  1, "BEEP1" },    {  2, "BEEP2" },    {  3, "BEEP3" },
  {  4, "BEEP4" },    {  5, "BEEP5" },    {  6, "ESC INFO" },
  {  7, "DIR 1" },    {  8, "DIR 2" },    {  9, "3D OFF" },
  { 10, "3D ON" },    { 11, "SETTINGS" }, { 12, "SAVE" },
  { 13, "EXTTEL ON" },{ 14, "EXTTEL OFF" },
  { 20, "DIR NORM" }, { 21, "DIR REV" },
  { 22, "LED0 ON" },  { 23, "LED1 ON" },  { 24, "LED2 ON" },
  { 25, "LED3 ON" },  { 26, "LED0 OFF" }, { 27, "LED1 OFF" },
  { 28, "LED2 OFF" }, { 29, "LED3 OFF" }, { 30, "AUDIO" },
  { 31, "SILENT" },   { 32, "TEL OFF" },  { 33, "ERPM TEL" },
  { 34, "TEMP TEL" }, { 35, "VOLT TEL" }, { 36, "CURR TEL" },
  { 37, "CONS TEL" }, { 38, "ERPMP TEL" },
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const DshotAnalysis *dshot_analysis(void)
{
  return &g_dshot;
}

//-----------------------------------------------------------------------------
// The four bits at the bottom of the frame, over the twelve above them.
// Bidirectional DShot inverts them, which is the only difference between the
// two flavours as far as this decoder is concerned.
static uint8_t dshot_crc(uint16_t d, bool inverted)
{
  uint8_t c = (uint8_t)((d ^ (d >> 4) ^ (d >> 8)) & 0x0F);

  return inverted ? (uint8_t)(~c & 0x0F) : c;
}

//-----------------------------------------------------------------------------
static const char *dshot_cmd_name(uint16_t v)
{
  for (unsigned i = 0; i < sizeof(g_dshot_cmd) / sizeof(g_dshot_cmd[0]); i++)
  {
    if (g_dshot_cmd[i].v == v)
      return g_dshot_cmd[i].name;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
// What the frame is telling the motor to do. The throttle band is 48..2047,
// so the percentage is of THAT and not of 2047 - the bottom of the range is
// where the motor starts turning, and calling it 2 % would be wrong twice.
static void dshot_value_text(const DshotFrame *f, char *buf, int size)
{
  if (0 == f->value)
  {
    snprintf(buf, size, "disarm");

    return;
  }

  if (f->value < 48)
  {
    const char *name = dshot_cmd_name(f->value);

    if (name)
      snprintf(buf, size, "%s", name);
    else
      snprintf(buf, size, "CMD %d", f->value);

    return;
  }

  snprintf(buf, size, "%d %d%%%s", f->value,
      (f->value - 48) * 100 / (2047 - 48), f->telem ? " tel" : "");
}

//-----------------------------------------------------------------------------
void dshot_byte_label(const DshotAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  dshot_value_text(&a->frame[(idx / 2) % DSHOT_MAX_FRAMES], buf, size);
}

//-----------------------------------------------------------------------------
// What each of the two bytes holds. The fields do not land on byte
// boundaries - eleven bits of throttle do not - so the second byte is the
// tail of the throttle, the telemetry flag and the CRC together, and whether
// that CRC agreed is the thing worth putting under it.
void dshot_field_label(const DshotAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (0 == idx % 2)
  {
    snprintf(buf, size, "THR10-3");

    return;
  }

  const DshotFrame *f = &a->frame[(idx / 2) % DSHOT_MAX_FRAMES];

  snprintf(buf, size, f->crc_ok ? "CRC" : "CRC!");
}

//-----------------------------------------------------------------------------
// A frame is sixteen bits and one number; the two bytes it packs into are
// where the byte boundaries happen to fall, and nobody reads them separately
void dshot_group_at(const DshotAnalysis *a, int idx, int *start, int *len)
{
  (void)a;

  *start = idx;
  *len = 1;

  if (idx < 0)
    return;

  *start = idx - idx % 2;
  *len = 2;
}

//-----------------------------------------------------------------------------
// One pass over the record at a given pulse level. Returns the number of
// frames found; out and a may be NULL for a counting-only pass.
static int dshot_scan(const LogicScratch *s, int runs, int size, int period_ns,
    int hi, LogicResult *out, DshotAnalysis *a)
{
  int found = 0;
  // Once a frame has settled which CRC this link uses, the next is tried that
  // way first. A link does not mix them, and a four-bit CRC agreeing with the
  // wrong hypothesis one time in sixteen is worth designing against.
  int variant = -1;

  // From run 1: the record's first run was cut by its own start, so its
  // length is a fact about the buffer and not about the idle before a frame
  for (int r = 1; r + 2 * DSHOT_BITS - 1 < runs; r++)
  {
    if (s->lvl[r] != hi)
      continue;

    int64_t t_x256 = ((int64_t)s->len[r] + s->len[r + 1]) * 256;

    if (t_x256 < DSHOT_MIN_SAMPLES * 256)
      continue;

    int t_ns = (int)(t_x256 * period_ns / 256);

    if (t_ns < DSHOT_T_MIN_NS || t_ns > DSHOT_T_MAX_NS)
      continue;

    // Idle in front of it. Inside a frame the longest low is five eighths of
    // a bit, so this is a frame boundary and not a bit of one.
    if ((int64_t)s->len[r - 1] * 256 * 10 < t_x256 * DSHOT_GAP_LEAD_X10)
      continue;

    uint16_t frame = 0;
    bool ok = true;

    for (int k = 0; k < DSHOT_BITS && ok; k++)
    {
      int rh = r + 2 * k;
      int64_t h = (int64_t)s->len[rh] * 256;
      int64_t p = h + (int64_t)s->len[rh + 1] * 256;
      int duty = (int)(h * 100 / t_x256);

      // The last bit's low runs into the idle after the frame, so its period
      // is not measurable and is not asked for. Its VALUE is in the pulse.
      if (k < DSHOT_BITS - 1)
      {
        int64_t d = p - t_x256;

        if (d < 0)
          d = -d;

        if (d * 100 > t_x256 * DSHOT_PERIOD_TOL_PCT)
          ok = false;
      }

      if (duty >= DSHOT_ZERO_MIN_PCT && duty <= DSHOT_ZERO_MAX_PCT)
        frame = (uint16_t)(frame << 1);
      else if (duty >= DSHOT_ONE_MIN_PCT && duty <= DSHOT_ONE_MAX_PCT)
        frame = (uint16_t)((frame << 1) | 1u);
      else
        ok = false;
    }

    if (!ok)
      continue;

    // ...and idle behind it. This is the check a strip of addressable LEDs
    // cannot pass: a WS2812 byte is followed by the next byte, so nowhere in
    // its frame do sixteen bits END. Unless the record itself ends here, in
    // which case there is nothing after the frame to look at.
    int tail = r + 2 * DSHOT_BITS - 1;
    bool at_end = (tail >= runs - 1) &&
        (s->pos[tail] + s->len[tail] >= size - 1);

    if (!at_end &&
        (int64_t)s->len[tail] * 256 * 10 < t_x256 * DSHOT_GAP_TAIL_X10)
      continue;

    uint16_t d = (uint16_t)(frame >> 4);
    uint8_t rx = (uint8_t)(frame & 0x0F);
    bool first_try = (1 == variant);
    bool crc_ok = false, bidir = false;

    if (rx == dshot_crc(d, first_try))
    {
      crc_ok = true;
      bidir = first_try;
    }
    else if (rx == dshot_crc(d, !first_try))
    {
      crc_ok = true;
      bidir = !first_try;
    }

    if (!crc_ok)
      continue;      // no frame is reported without its CRC behind it

    variant = bidir ? 1 : 0;
    found++;

    if (out && a)
    {
      if (a->frames >= DSHOT_MAX_FRAMES || out->count + 2 > LOGIC_MAX_BYTES)
      {
        out->truncated = true;
        break;
      }

      DshotFrame *f = &a->frame[a->frames];

      f->value = (uint16_t)(frame >> 5);
      f->telem = (0 != (frame & 0x10));
      f->crc = rx;
      f->crc_ok = true;

      // The rate off the whole frame rather than off its first bit: fifteen
      // periods between the first rising edge and the last is fifteen times
      // less sensitive to where a single edge landed
      int64_t span = s->pos[r + 2 * (DSHOT_BITS - 1)] - s->pos[r];

      a->rate = (int)(1000000000ll * (DSHOT_BITS - 1) /
          (span * period_ns > 0 ? span * period_ns : 1));
      a->bidir = bidir;

      out->bytes[out->count] = (uint8_t)(frame >> 8);
      out->pos[out->count] = s->pos[r];
      out->end[out->count] = s->pos[r + 16];
      out->count++;

      out->bytes[out->count] = (uint8_t)(frame & 0xFF);
      out->pos[out->count] = s->pos[r + 16];
      out->end[out->count] =
          (int)(s->pos[r + 30] + t_x256 / 256);
      out->count++;

      a->frames++;
      a->crc_ok++;
    }

    // Past the frame's thirty-two runs; the loop's own step takes it to the
    // first run after them
    r += 2 * DSHOT_BITS - 1;
  }

  return found;
}

//-----------------------------------------------------------------------------
static int dshot_snap_rate(int kbit)
{
  for (unsigned i = 0; i < sizeof(g_dshot_rates) / sizeof(g_dshot_rates[0]); i++)
  {
    int d = kbit - g_dshot_rates[i];

    if (d < 0)
      d = -d;

    if (d * 100 <= g_dshot_rates[i] * 6)
      return g_dshot_rates[i];
  }

  return 0;      // not one of the four: the number is reported as measured
}

//-----------------------------------------------------------------------------
int dshot_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_DSHOT;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 2 * DSHOT_BITS + 1 || period_ns <= 0)
    return 0;

  // The line rests low and the frame is pulses on it. Bidirectional DShot
  // inverts the whole thing - and inverts its CRC with it - so both levels
  // are tried and the CRC says which flavour this is.
  int best = 0, best_hi = 1;

  for (int p = 0; p < 2; p++)
  {
    int n = dshot_scan(scratch, runs, size, period_ns, p, NULL, NULL);

    if (n > best)
    {
      best = n;
      best_hi = p;
    }
  }

  if (0 == best)
    return 0;

  memset(&g_dshot, 0, sizeof(g_dshot));
  dshot_scan(scratch, runs, size, period_ns, best_hi, out, &g_dshot);

  if (0 == out->count)
    return 0;

  out->rate = g_dshot.rate;
  out->idle_high = (0 == best_hi);
  out->burst_start = true;   // a frame is found from the idle around it

  // Every frame reported here has already been confirmed by its own CRC, so
  // there is nothing left to be unsure about
  out->ambiguous = false;

  bool agree = true;

  for (int i = 1; i < g_dshot.frames; i++)
  {
    if (g_dshot.frame[i].value != g_dshot.frame[0].value ||
        g_dshot.frame[i].telem != g_dshot.frame[0].telem)
      agree = false;
  }

  int kbit = (g_dshot.rate + 500) / 1000;
  int snap = dshot_snap_rate(kbit);
  char name[16];

  if (snap)
    snprintf(name, sizeof(name), "DShot%d%s", snap, g_dshot.bidir ? "bd" : "");
  else
    snprintf(name, sizeof(name), "DShot~%dk%s", kbit,
        g_dshot.bidir ? "bd" : "");

  char body[20];

  dshot_value_text(&g_dshot.frame[0], body, sizeof(body));

  if (!agree)
    snprintf(out->info, sizeof(out->info), "%s %d frames", name,
        g_dshot.frames);
  else if (g_dshot.frames > 1)
    snprintf(out->info, sizeof(out->info), "%s %s x%d", name, body,
        g_dshot.frames);
  else
    snprintf(out->info, sizeof(out->info), "%s %s", name, body);

  return out->count;
}
