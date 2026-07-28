/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DALI - Digital Addressable Lighting Interface, IEC 62386. Two wires, no
 * polarity to get wrong, 1200 bit/s, and every ballast in a commercial
 * ceiling on the same pair.
 *
 * It IS Manchester, which is why the generic decoder already reads its bits.
 * What this one adds is the reading a person wants off a lighting bus: which
 * ballast, and what it was told to do.
 *
 *   forward   start bit, 8 address bits, 8 data bits, 2 stop bits
 *   backward  start bit, 8 data bits, 2 stop bits - the answer to a query
 *
 * The bus rests HIGH, at about sixteen volts, and a transmitter pulls it
 * down. A one is a transition from low to high in the middle of the bit and a
 * zero the other way, so the start bit - always a one - opens with the bus
 * being pulled down and lets it back up half a bit later. That first falling
 * edge out of a long idle is what a frame is found from.
 *
 * The stop bits are not encoded at all: they are two bit times of the bus
 * left alone, which is exactly what "no transition where the next one should
 * be" looks like, so the frame ends where the reader runs out of edges.
 *
 * The address byte is where the interest is, and it is not a number:
 *
 *   0AAAAAAS  a short address, one ballast out of 64
 *   100AAAAS  a group, one of 16
 *   1111111S  everyone
 *   101CCCC1  a special command - INITIALISE, RANDOMISE, the commissioning set
 *
 * The bottom bit of it, the selector, decides what the OTHER byte means: with
 * it clear the data byte is an arc power level, with it set the data byte is
 * a command. The same 0x00 is "off" one way and "level zero" the other, and a
 * decoder that shows the byte and stops has left the reader to do the part
 * that matters.
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
// 1200 bit/s, so 833.3 us a bit. The window is for the cheap gear.
#define DALI_BIT_MIN_NS   700000
#define DALI_BIT_MAX_NS  1000000
#define DALI_TOL_PCT          30

#define DALI_FWD_BITS         17   // start + address + data
#define DALI_BWD_BITS          9   // start + data
#define DALI_MAX_BITS         20

// A frame stands in silence: the settling time before an answer is at least
// 2.9 ms and between telegrams at least 9.2, against a bit of 0.83
#define DALI_QUIET_BITS        3

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_dali  (g_logic_analysis.dali)

// The commands worth a name. Numbered ones - scenes, groups - are formatted
// rather than listed, which is what keeps this table to a screenful.
static const struct { uint8_t op; const char *name; } g_dali_cmd[] =
{
  {   0, "OFF" },         {   1, "UP" },          {   2, "DOWN" },
  {   3, "STEP UP" },     {   4, "STEP DN" },     {   5, "MAX" },
  {   6, "MIN" },         {   7, "STEP DN OFF" }, {   8, "ON STEP UP" },
  {   9, "ENA DT" },      {  32, "RESET" },       {  33, "LVL->DTR" },
  {  42, "DTR->MAX" },    {  43, "DTR->MIN" },    {  44, "DTR->FAIL" },
  {  45, "DTR->PWRON" },  {  46, "DTR->FADE T" }, {  47, "DTR->FADE R" },
  { 128, "DTR->ADDR" },   { 144, "Q STATUS" },    { 145, "Q BALLAST" },
  { 146, "Q LAMP FAIL" }, { 147, "Q LAMP ON" },   { 148, "Q LIMIT" },
  { 149, "Q RESET" },     { 150, "Q NO ADDR" },   { 151, "Q VERSION" },
  { 152, "Q DTR" },       { 153, "Q DEV TYPE" },  { 154, "Q PHYS MIN" },
  { 155, "Q PWR FAIL" },  { 160, "Q LEVEL" },     { 161, "Q MAX" },
  { 162, "Q MIN" },       { 163, "Q PWRON" },     { 164, "Q FAIL" },
  { 165, "Q FADE" },      { 192, "Q GROUPS L" },  { 193, "Q GROUPS H" },
  { 194, "Q ADDR H" },    { 195, "Q ADDR M" },    { 196, "Q ADDR L" },
};

// ...and the special commands, which live in the ADDRESS byte instead
static const struct { uint8_t op; const char *name; } g_dali_special[] =
{
  { 0xA1, "TERMINATE" },  { 0xA3, "DTR0" },       { 0xA5, "INITIALISE" },
  { 0xA7, "RANDOMISE" },  { 0xA9, "COMPARE" },    { 0xAB, "WITHDRAW" },
  { 0xB1, "SEARCH H" },   { 0xB3, "SEARCH M" },   { 0xB5, "SEARCH L" },
  { 0xB7, "PROG ADDR" },  { 0xB9, "VERIFY ADDR" },{ 0xBB, "Q SHORT" },
  { 0xBD, "PHYS SEL" },   { 0xC1, "ENABLE DT" },  { 0xC3, "DTR1" },
  { 0xC5, "DTR2" },       { 0xC7, "WRITE MEM" },
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const DaliAnalysis *dali_analysis(void)
{
  return &g_dali;
}

//-----------------------------------------------------------------------------
static const char *dali_cmd_name(uint8_t op)
{
  for (unsigned i = 0; i < sizeof(g_dali_cmd) / sizeof(g_dali_cmd[0]); i++)
  {
    if (g_dali_cmd[i].op == op)
      return g_dali_cmd[i].name;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
static const char *dali_special_name(uint8_t op)
{
  for (unsigned i = 0;
      i < sizeof(g_dali_special) / sizeof(g_dali_special[0]); i++)
  {
    if (g_dali_special[i].op == op)
      return g_dali_special[i].name;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
// What the frame says, in the words a lighting bus is read in
void dali_frame_text(const DaliFrame *f, char *buf, int size)
{
  char who[12];

  buf[0] = 0;

  if (DALI_BACKWARD == f->kind)
  {
    // The answer to a query. 0xFF is yes and 0x00 is no; anything else is
    // the value that was asked for.
    if (0xFF == f->data)
      snprintf(buf, size, "ans YES");
    else if (0x00 == f->data)
      snprintf(buf, size, "ans NO");
    else
      snprintf(buf, size, "ans %d", f->data);

    return;
  }

  if (DALI_SPECIAL == f->kind)
  {
    const char *name = dali_special_name(f->addr);

    if (name)
      snprintf(buf, size, "%s %d", name, f->data);
    else
      snprintf(buf, size, "spec %02X %d", f->addr, f->data);

    return;
  }

  switch (f->kind)
  {
    case DALI_SHORT: snprintf(who, sizeof(who), "a%d", f->target); break;
    case DALI_GROUP: snprintf(who, sizeof(who), "g%d", f->target); break;
    default:         snprintf(who, sizeof(who), "all"); break;
  }

  // The selector decides what the data byte IS, and getting that wrong turns
  // "switch off" into "set the level to zero" - which is the same thing to a
  // lamp and a different thing entirely to whoever is reading the bus
  if (!f->cmd)
  {
    if (0xFF == f->data)
      snprintf(buf, size, "%s lvl -", who);   // MASK: leave the level alone
    else
      snprintf(buf, size, "%s lvl %d", who, f->data);

    return;
  }

  const char *name = dali_cmd_name(f->data);

  if (name)
    snprintf(buf, size, "%s %s", who, name);
  else if (f->data >= 16 && f->data <= 31)
    snprintf(buf, size, "%s SCENE %d", who, f->data - 16);
  else if (f->data >= 64 && f->data <= 79)
    snprintf(buf, size, "%s ST SCN %d", who, f->data - 64);
  else if (f->data >= 80 && f->data <= 95)
    snprintf(buf, size, "%s DEL SCN %d", who, f->data - 80);
  else if (f->data >= 96 && f->data <= 111)
    snprintf(buf, size, "%s ADD GRP %d", who, f->data - 96);
  else if (f->data >= 112 && f->data <= 127)
    snprintf(buf, size, "%s DEL GRP %d", who, f->data - 112);
  else if (f->data >= 176 && f->data <= 191)
    snprintf(buf, size, "%s Q SCN %d", who, f->data - 176);
  else
    snprintf(buf, size, "%s cmd %d", who, f->data);
}

//-----------------------------------------------------------------------------
void dali_byte_label(const DaliAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  for (int i = 0; i < a->frames && i < DALI_MAX_FRAMES; i++)
  {
    if (idx >= a->frame[i].first &&
        idx < a->frame[i].first + a->frame[i].count)
    {
      dali_frame_text(&a->frame[i], buf, size);

      return;
    }
  }
}

//-----------------------------------------------------------------------------
// Which byte of the frame this is. A forward frame is an address and a data
// byte and they are not interchangeable; a backward frame is one answer.
void dali_field_label(const DaliAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  for (int i = 0; i < a->frames && i < DALI_MAX_FRAMES; i++)
  {
    const DaliFrame *f = &a->frame[i];

    if (idx < f->first || idx >= f->first + f->count)
      continue;

    if (DALI_BACKWARD == f->kind)
      snprintf(buf, size, "ANS");
    else if (idx == f->first)
      snprintf(buf, size, "ADDR");
    else
      snprintf(buf, size, f->cmd ? "CMD" : "LEVEL");

    return;
  }
}

//-----------------------------------------------------------------------------
// One frame is one instruction and is shown as one
void dali_group_at(const DaliAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  for (int i = 0; i < a->frames && i < DALI_MAX_FRAMES; i++)
  {
    if (idx >= a->frame[i].first &&
        idx < a->frame[i].first + a->frame[i].count)
    {
      *start = a->frame[i].first;
      *len = a->frame[i].count;

      return;
    }
  }
}

//-----------------------------------------------------------------------------
static bool dali_near(int64_t l, int64_t want)
{
  int64_t d = l - want;

  if (d < 0)
    d = -d;

  return (d * 100 <= want * DALI_TOL_PCT);
}

//-----------------------------------------------------------------------------
int dali_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  memset(&g_dali, 0, sizeof(g_dali));
  out->proto = PROTO_DALI;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 8 || period_ns <= 0)
    return 0;

  int idle = (0 != scratch->lvl[runs - 1]) ? 1 : 0;

  out->idle_high = (0 != idle);

  for (int r = 1; r + 2 < runs && g_dali.frames < DALI_MAX_FRAMES; r++)
  {
    if (scratch->lvl[r] == idle)
      continue;              // a frame opens by pulling the bus DOWN

    // The start bit is a one: half a bit low, then back up. So this run is
    // half a bit and the bit time is twice it.
    int64_t bit_x256 = (int64_t)scratch->len[r] * 2 * 256;
    int bit_ns = (int)(bit_x256 * period_ns / 256);

    if (bit_ns < DALI_BIT_MIN_NS || bit_ns > DALI_BIT_MAX_NS)
      continue;

    // Silence in front of it, or the record's own start
    if (r > 1 && (int64_t)scratch->len[r - 1] * 256 <
        bit_x256 * DALI_QUIET_BITS)
      continue;

    // Step edge to edge from the middle of the start bit
    uint32_t value = 0;
    int n = 0;
    int e = r + 1;           // the rising edge in the middle of the start bit

    while (n < DALI_MAX_BITS && e < runs)
    {
      value = (value << 1) |
          (uint32_t)((scratch->lvl[e] != 0 ? 1 : 0) == idle ? 1 : 0);
      n++;

      int64_t target = (int64_t)scratch->pos[e] * 256 + bit_x256;
      int j = e + 1;

      while (j < runs && (int64_t)scratch->pos[j] * 256 <
          target - bit_x256 / 4)
        j++;

      if (j >= runs || (int64_t)scratch->pos[j] * 256 > target + bit_x256 / 4)
        break;               // the stop bits: the bus is simply left alone

      e = j;
    }

    if (DALI_FWD_BITS != n && DALI_BWD_BITS != n)
      continue;

    // ...and every run in it was half a bit or a whole one. Not the run AT
    // the last edge, though: that is the second half of the last bit, and
    // when the bus is simply left there - which is what the stop bits are -
    // it runs on into the idle for as long as the gap lasts. Its length is a
    // fact about the silence after the frame and not about the frame.
    bool fit = true;

    for (int k = r; k < e && fit; k++)
    {
      int64_t l = (int64_t)scratch->len[k] * 256;

      fit = dali_near(l, bit_x256 / 2) || dali_near(l, bit_x256);
    }

    if (!fit)
      continue;

    DaliFrame *f = &g_dali.frame[g_dali.frames];
    int nbytes = (DALI_FWD_BITS == n) ? 2 : 1;

    if (out->count + nbytes > LOGIC_MAX_BYTES)
    {
      out->truncated = true;
      break;
    }

    memset(f, 0, sizeof(*f));
    f->bits = (uint8_t)n;
    f->first = (uint8_t)out->count;
    f->count = (uint8_t)nbytes;

    if (DALI_FWD_BITS == n)
    {
      f->addr = (uint8_t)((value >> 8) & 0xFF);
      f->data = (uint8_t)(value & 0xFF);
      f->cmd = (0 != (f->addr & 1));

      if (0 == (f->addr & 0x80))
      {
        f->kind = DALI_SHORT;
        f->target = (uint8_t)((f->addr >> 1) & 0x3F);
      }
      else if (0x80 == (f->addr & 0xE0))
      {
        f->kind = DALI_GROUP;
        f->target = (uint8_t)((f->addr >> 1) & 0x0F);
      }
      else if (0xFE == (f->addr & 0xFE))
      {
        f->kind = DALI_BROADCAST;
      }
      else
      {
        f->kind = DALI_SPECIAL;
      }
    }
    else
    {
      f->kind = DALI_BACKWARD;
      f->data = (uint8_t)(value & 0xFF);
    }

    // The address byte first, then the data - which is the order they went
    // past, and the order the labels above assume
    if (2 == nbytes)
    {
      out->bytes[out->count] = f->addr;
      out->pos[out->count] = scratch->pos[r];
      out->end[out->count] = scratch->pos[r] +
          (int)(bit_x256 * 9 / 256);
      out->count++;
    }

    out->bytes[out->count] = f->data;
    out->pos[out->count] = (2 == nbytes) ?
        scratch->pos[r] + (int)(bit_x256 * 9 / 256) : scratch->pos[r];
    out->end[out->count] = scratch->pos[e] + (int)(bit_x256 / 512);
    out->count++;

    g_dali.frames++;
    g_dali.bits += n;
    g_dali.rate = (int)(1000000000ll / bit_ns);

    r = e;                   // past the frame
  }

  if (0 == out->count)
    return 0;

  out->rate = g_dali.rate;
  out->burst_start = true;

  // Seventeen bits or nine, all of them bi-phase at 1200, with the bus at
  // rest either side. Nothing else here answers to that shape, so there is
  // nothing left to be unsure about.
  out->ambiguous = false;

  char body[24];

  if (1 == g_dali.frames)
  {
    dali_frame_text(&g_dali.frame[0], body, sizeof(body));
    snprintf(out->info, sizeof(out->info), "DALI %s", body);
  }
  else
  {
    // A command and the answer to it is the commonest pair on the bus, and
    // saying "2 frames" for it would be throwing away the interesting half
    dali_frame_text(&g_dali.frame[0], body, sizeof(body));
    snprintf(out->info, sizeof(out->info), "DALI %s +%d", body,
        g_dali.frames - 1);
  }

  return out->count;
}
