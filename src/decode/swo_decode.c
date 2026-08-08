/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SWO / ITM - the trace pin every Cortex-M has and hardly anyone looks at.
 *
 * A part with a CoreSight ITM can send trace out of ONE pin, and the commonest
 * thing to send is printf: the library writes a character to stimulus port 0
 * and the macrocell puts it on the wire. That is why this decoder exists on an
 * instrument with one probe - it is the whole debug output of a running MCU on
 * a single wire, and this scope has a single wire's worth of input.
 *
 * The wire itself is an ordinary 8N1 serial line, idle high, LSB first, at
 * traceclk / (SWOSCALER + 1) - which is any rate at all, not a standard baud.
 * (The TPIU can also encode SWO in Manchester; nothing in common use does, and
 * this decoder reads the NRZ mode only. A Manchester-coded pin still reads as
 * bits under the Manchester decoder.)
 *
 * What makes it SWO rather than a UART is what the bytes SAY. ITM traffic is a
 * stream of self-describing packets, and the header byte says which:
 *
 *   xxxxxxSS  SS != 00   a source packet: SS says 1, 2 or 4 payload bytes,
 *                        bit 2 says software (a stimulus port) or hardware
 *                        (the DWT), and the top five bits are the port or
 *                        the discriminator
 *   0b0TTT0000           a short local timestamp, TTT = 1..6
 *   0b11TT0000           a long one, payload seven bits to a byte
 *   0x94 / 0xB4          global timestamp, lower / upper
 *   0bxxxx1x00           an extension packet
 *   0x70                 overflow - trace was dropped, and that is worth
 *                        seeing, because it means the rest has gaps in it
 *   0x00 x5, 0x80        synchronisation: 47 zero bits and a one
 *
 * That grammar is the identification, and it is strict enough to matter: the
 * discriminator values 3..7 and 24..31 are reserved, and each of the defined
 * ones has a payload size it must have. A UART carrying English text runs into
 * one of those within a few bytes, which is exactly what has to happen - the
 * two signals are the same signal electrically, and only the bytes tell them
 * apart.
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
#define SWO_MIN_BIT_SAMPLES   3
#define SWO_FRAME_BITS       10     // start + 8 data + stop

// Below this a link is not a trace port. SWO is traceclk / (SWOSCALER + 1) -
// a divider on a clock in the tens of megahertz - and a macrocell throttled
// to a few tens of kilobits would spend its life dropping trace and emitting
// overflow packets. The floor is here because MIDI is 8N1 at 31250 and its
// bytes DO parse as ITM often enough to matter: a status byte 0x90 heads a
// perfectly legal timestamp packet. A rate that no trace port is ever set to
// is the honest way to tell the two apart, and it costs nothing real.
#define SWO_MIN_RATE     100000
#define SWO_IDLE_GAP_BITS    11
#define SWO_MIN_BYTES         4

// A record can begin in the middle of a packet, and a packet is at most five
// bytes long - so there are at most five places the first whole one can start
#define SWO_RESYNC_MAX        5

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_swo   (g_logic_analysis.swo)

// SWO is traceclk / (SWOSCALER + 1), so it is not on a baud table at all: a
// 168 MHz part asked for "2 MHz" gets 2.0 MHz only if the division comes out
// even, and gets 2.1 or 1.87 otherwise. The rate is therefore MEASURED off the
// record, and this list only seeds the search with the round numbers debuggers
// ask for - the measured estimate competes with them and usually wins.
static const int g_swo_rates[] =
{
  115200, 230400, 460800, 921600, 1000000, 2000000, 3000000, 4000000,
  6000000, 8000000, 12000000,
};

#define SWO_MAX_CAND  ((int)ARRAY_SIZE(g_swo_rates) + 2)

// The exceptions by number, which is what a DWT exception-trace packet
// carries. Everything from 16 up is an external interrupt and is named by its
// IRQ number, because only the part's own header knows what IRQ 37 is wired to.
static const char *const g_swo_exc[16] =
{
  "Thread", "Reset", "NMI", "HardFault", "MemManage", "BusFault",
  "UsageFault", "-", "-", "-", "-", "SVCall", "DebugMon", "-", "PendSV",
  "SysTick",
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const SwoAnalysis *swo_analysis(void)
{
  return &g_swo;
}

//-----------------------------------------------------------------------------
// One 8N1 pass at a given bit time. Idle is high and only high: an SWO pin is
// a push-pull output that rests at one, and a decoder that also tried the
// inverse would be offering a reading of a signal nothing produces.
static int swo_scan(const uint8_t *data, int size, int offset, int mid,
    int64_t bit_x256, const LogicScratch *scratch, int runs, LogicResult *out)
{
  const int *run_len = scratch->len;
  const int *run_pos = scratch->pos;
  const uint8_t *run_lvl = scratch->lvl;
  int64_t gap_x256 = bit_x256 * SWO_IDLE_GAP_BITS;

  out->count = 0;
  out->errors = 0;
  out->burst_start = false;
  out->truncated = false;

  for (int r = 0; r < runs - 1; r++)
  {
    if (0 == run_lvl[r])
      continue;                 // a frame starts where a start bit breaks idle

    if (out->count >= LOGIC_MAX_BYTES)
    {
      out->truncated = true;

      break;
    }

    int64_t frame = (int64_t)run_pos[r + 1] * 256;
    int value = 0;
    bool ok = true;

    for (int b = 0; b < 8; b++)         // LSB first, sampled at bit centre
    {
      int64_t center = frame + (bit_x256 * (3 + 2 * b)) / 2;
      int si = (int)(center / 256);

      if (si >= size)
      {
        ok = false;

        break;
      }

      if (sample_at(data, size, offset, si) > mid)
        value |= (1 << b);
    }

    if (!ok)
      break;                    // the record ran out inside this frame

    int stop_i = (int)((frame + (bit_x256 * 19) / 2) / 256);

    if (stop_i >= size)
      break;

    if (sample_at(data, size, offset, stop_i) <= mid)
    {
      out->errors++;

      continue;                 // try the next edge rather than skip a frame
    }

    if (0 == out->count && (int64_t)run_len[r] * 256 >= gap_x256)
      out->burst_start = true;

    out->bytes[out->count] = (uint8_t)value;
    out->pos[out->count] = run_pos[r + 1];
    out->end[out->count] = (int)((frame + bit_x256 * SWO_FRAME_BITS) / 256);
    out->count++;

    // Resume past the stop bit, so this frame's own data runs are not read as
    // the start of the next one
    int frame_end = (int)((frame + bit_x256 * (SWO_FRAME_BITS - 1)) / 256);

    while (r + 1 < runs && run_pos[r + 1] < frame_end)
      r++;

    r--;
  }

  return out->count;
}

//-----------------------------------------------------------------------------
// A continuation-coded payload: seven bits to a byte, least significant group
// first, and the top bit says another byte follows. Returns the byte count, or
// -1 when the record ends before the payload does.
static int swo_cont(const uint8_t *b, int n, int i, uint32_t *val)
{
  uint32_t v = 0;
  int k = 0;

  while (k < 4)
  {
    if (i + k >= n)
      return -1;                // truncated, which is the end of the record
                                // and not a fault in the traffic
    v |= (uint32_t)(b[i + k] & 0x7F) << (7 * k);
    k++;

    if (0 == (b[i + k - 1] & 0x80))
      break;
  }

  *val = v;

  return k;
}

//-----------------------------------------------------------------------------
// Is a hardware source packet of this discriminator allowed to be this long?
//
// This is the single most useful gate in the whole decoder, and it is not a
// heuristic: the DWT's packets have FIXED sizes, and 3..7 and 24..31 are not
// assigned at all. Text read as ITM walks into one of these within a few
// bytes, which is what keeps a serial console from being called a trace port.
static bool swo_hw_ok(int id, int size, uint32_t payload)
{
  switch (id)
  {
    case 0: return (1 == size);               // event counter wrap
    case 1: return (2 == size);               // exception trace
    // PC sample: four bytes of program counter, or a single zero byte, which
    // is how the DWT says the core was asleep at the sample point
    case 2: return (4 == size) || (1 == size && 0 == payload);
    default: break;
  }

  // 8..15 are the data-trace address group: an even id carries a PC value
  // (four bytes), an odd one a data address (two)
  if (id >= 8 && id <= 15)
    return (0 == (id & 1)) ? (4 == size) : (2 == size);

  // 16..23 carry the data itself, whatever width the access was
  if (id >= 16 && id <= 23)
    return true;

  return false;
}

//-----------------------------------------------------------------------------
// Read the byte stream as ITM packets, starting `from` bytes in. Fills the
// analysis and returns the number of packets; a->bad counts the bytes that
// could not be the head of any packet.
static int swo_itm_parse(const uint8_t *b, int n, int from, SwoAnalysis *a)
{
  memset(a->pidx, 0xFF, sizeof(a->pidx));
  a->packets = 0;
  a->sw = a->hw = a->sync = a->ts = a->bad = 0;
  a->portmask = 0;
  a->cut = (uint8_t)from;

  int i = from;

  while (i < n)
  {
    uint8_t h = b[i];
    int kind = -1, id = 0, len = 1;
    uint32_t value = 0;

    if (0 == h)
    {
      // Synchronisation: at least 47 zero bits and then a one, which on the
      // byte grid is five or more zero bytes followed by 0x80
      int z = 0;

      while (i + z < n && 0 == b[i + z])
        z++;

      if (z < 5)
      {
        a->bad++;               // a lone zero byte heads no packet at all
        i++;

        continue;
      }

      if (i + z < n && 0x80 == b[i + z])
        len = z + 1;
      else if (i + z < n)
      {
        a->bad++;
        i++;

        continue;
      }
      else
        len = z;                // the record ended inside the sync sequence

      kind = ITM_SYNC;
    }
    else if (0x70 == h)
    {
      kind = ITM_OVERFLOW;      // trace was dropped: the rest has holes in it
    }
    else if (0 == (h & 0x0F))
    {
      if (0 == (h & 0x80))
      {
        kind = ITM_LTS;         // short form: the whole value is in the header
        value = (h >> 4) & 7;
      }
      else
      {
        int k = swo_cont(b, n, i + 1, &value);

        if (k < 0)
          break;

        kind = ITM_LTS;
        id = (h >> 4) & 3;      // which of the two was delayed, if either
        len = 1 + k;
      }
    }
    else if (0x94 == h || 0xB4 == h)
    {
      int k = swo_cont(b, n, i + 1, &value);

      if (k < 0)
        break;

      kind = ITM_GTS;
      id = (0xB4 == h) ? 1 : 0;
      len = 1 + k;
    }
    else if (0x08 == (h & 0x0B))
    {
      kind = ITM_EXT;
      id = (h >> 4) & 7;

      if (0 != (h & 0x80))
      {
        int k = swo_cont(b, n, i + 1, &value);

        if (k < 0)
          break;

        len = 1 + k;
      }
    }
    else if (0 != (h & 0x03))
    {
      int size = 1 << ((h & 3) - 1);

      if (i + 1 + size > n)
        break;                  // the record ended inside the payload

      for (int k = 0; k < size; k++)
        value |= (uint32_t)b[i + 1 + k] << (8 * k);

      id = h >> 3;
      len = 1 + size;

      if (0 != (h & 0x04))
      {
        if (!swo_hw_ok(id, size, value))
        {
          a->bad++;
          i++;

          continue;
        }

        kind = ITM_HW;
      }
      else
      {
        kind = ITM_SW;
      }
    }
    else
    {
      a->bad++;                 // reserved header: nothing sends this
      i++;

      continue;
    }

    if (a->packets < SWO_MAX_PKTS)
    {
      ItmPacket *p = &a->pkt[a->packets];

      p->kind = (uint8_t)kind;
      p->id = (uint8_t)id;
      p->first = (uint8_t)i;
      p->count = (uint8_t)len;
      p->value = value;

      for (int k = 0; k < len && i + k < LOGIC_MAX_BYTES; k++)
        a->pidx[i + k] = (uint8_t)a->packets;

      a->packets++;
    }

    if (ITM_SW == kind)
    {
      a->sw++;

      if (id < 32)
        a->portmask |= 1u << id;
    }
    else if (ITM_HW == kind)
      a->hw++;
    else if (ITM_SYNC == kind)
      a->sync++;
    else if (ITM_LTS == kind || ITM_GTS == kind)
      a->ts++;

    i += len;
  }

  return a->packets;
}

//-----------------------------------------------------------------------------
// The printf, put back together. A stimulus port is a byte pipe and the
// library writes one character at a time, so the text is the payloads of the
// one-byte software packets on whichever port carries the most of them - and
// the port matters, because port 0 is the console and port 31 is somebody
// else's instrumentation.
static void swo_text(const uint8_t *bytes, SwoAnalysis *a)
{
  int per_port[32];
  int best = -1, best_n = 0;

  memset(per_port, 0, sizeof(per_port));

  for (int i = 0; i < a->packets; i++)
  {
    const ItmPacket *p = &a->pkt[i];
    uint8_t v = (uint8_t)p->value;

    if (ITM_SW != p->kind || 2 != p->count || p->id >= 32)
      continue;

    if ((v >= 0x20 && v < 0x7F) || '\n' == v || '\r' == v || '\t' == v)
      per_port[p->id]++;
  }

  for (int i = 0; i < 32; i++)
  {
    if (per_port[i] > best_n)
    {
      best_n = per_port[i];
      best = i;
    }
  }

  a->text_port = (int8_t)best;
  a->text[0] = 0;

  if (best < 0)
    return;

  int w = 0;

  for (int i = 0; i < a->packets && w + 1 < (int)sizeof(a->text); i++)
  {
    const ItmPacket *p = &a->pkt[i];
    uint8_t v = (uint8_t)p->value;

    if (ITM_SW != p->kind || 2 != p->count || p->id != best)
      continue;

    // The line breaks are what make it a log rather than a string, and they
    // have no shape on a one-line header. A space keeps the words apart,
    // which is all the header has room to say.
    if ('\n' == v || '\r' == v || '\t' == v)
      v = ' ';
    else if (v < 0x20 || v >= 0x7F)
      continue;

    a->text[w++] = (char)v;
  }

  a->text[w] = 0;

  (void)bytes;
}

//-----------------------------------------------------------------------------
// How well one reading of the record holds up as ITM. Packets are worth
// having and unaccountable bytes are worth more against, because a wrong bit
// time produces plenty of both and the right one produces only the first.
static int swo_score(const SwoAnalysis *a)
{
  return a->packets * 4 - a->bad * 10 + a->sync * 8;
}

//-----------------------------------------------------------------------------
void swo_byte_label(const SwoAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || 0xFF == a->pidx[idx])
    return;

  const ItmPacket *p = &a->pkt[a->pidx[idx]];

  // The value goes on the byte that COMPLETES the packet, written once across
  // the group - the rule every other decoder here follows
  if (idx != p->first + p->count - 1)
    return;

  switch (p->kind)
  {
    case ITM_SYNC:
      snprintf(buf, size, "SYNC");
      break;

    case ITM_OVERFLOW:
      // Not cosmetic: the macrocell dropped trace, so whatever is missing
      // from the stream is missing because of this and not because it was
      // never sent
      snprintf(buf, size, "OVERFLOW");
      break;

    case ITM_LTS:
      snprintf(buf, size, "t+%lu%s", (unsigned long)p->value,
          (0 == p->id) ? "" : "d");
      break;

    case ITM_GTS:
      snprintf(buf, size, "gts%d %lX", p->id, (unsigned long)p->value);
      break;

    case ITM_EXT:
      snprintf(buf, size, "ext %d", p->id);
      break;

    case ITM_SW:
      // One byte on a stimulus port is a character nine times out of ten,
      // which is the entire reason anyone wires this pin up
      if (2 == p->count && p->value >= 0x20 && p->value < 0x7F)
        snprintf(buf, size, "'%c'", (char)p->value);
      else if (2 == p->count)
        snprintf(buf, size, "%02lX", (unsigned long)p->value);
      else
        snprintf(buf, size, "%lX", (unsigned long)p->value);
      break;

    case ITM_HW:
      if (1 == p->id)
      {
        // Exception trace: which one, and whether the core went in, came out,
        // or returned to it
        int num = (int)(p->value & 0x1FF);
        int fn = (int)((p->value >> 12) & 3);
        const char *what = (1 == fn) ? "in" : (2 == fn) ? "out" :
            (3 == fn) ? "ret" : "?";

        if (num < 16)
          snprintf(buf, size, "%s %s", g_swo_exc[num], what);
        else
          snprintf(buf, size, "IRQ%d %s", num - 16, what);
      }
      else if (2 == p->id)
      {
        if (2 == p->count)
          snprintf(buf, size, "asleep");
        else
          snprintf(buf, size, "PC=%08lX", (unsigned long)p->value);
      }
      else if (0 == p->id)
      {
        snprintf(buf, size, "cnt %02lX", (unsigned long)p->value);
      }
      else if (p->id >= 8 && p->id <= 15)
      {
        snprintf(buf, size, "%s%d=%lX", (0 == (p->id & 1)) ? "PC" : "ad",
            (p->id >> 1) & 3, (unsigned long)p->value);
      }
      else
      {
        snprintf(buf, size, "%s%d=%lX", (0 == (p->id & 1)) ? "rd" : "wr",
            (p->id >> 1) & 3, (unsigned long)p->value);
      }
      break;

    default:
      break;
  }

  (void)v;
}

//-----------------------------------------------------------------------------
void swo_field_label(const SwoAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (0xFF == a->pidx[idx])
  {
    // The record began inside a packet: these bytes are the tail of one whose
    // header went past before the acquisition started, and there is no
    // reading of them at all
    snprintf(buf, size, "cut");

    return;
  }

  const ItmPacket *p = &a->pkt[a->pidx[idx]];

  if (idx != p->first)
  {
    snprintf(buf, size, "D%d", idx - p->first - 1);

    return;
  }

  switch (p->kind)
  {
    case ITM_SW:   snprintf(buf, size, "p%d", p->id); break;
    case ITM_HW:   snprintf(buf, size, "DWT%d", p->id); break;
    case ITM_LTS:  snprintf(buf, size, "LTS"); break;
    case ITM_GTS:  snprintf(buf, size, "GTS"); break;
    case ITM_EXT:  snprintf(buf, size, "EXT"); break;
    case ITM_SYNC: snprintf(buf, size, "SYNC"); break;
    default:       snprintf(buf, size, "OVF"); break;
  }
}

//-----------------------------------------------------------------------------
// A packet is one thing - its header and its payload together - so it lights
// as one and its value is written once across it.
void swo_group_at(const SwoAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || 0xFF == a->pidx[idx])
    return;

  const ItmPacket *p = &a->pkt[a->pidx[idx]];

  *start = p->first;
  *len = p->count;
}

//-----------------------------------------------------------------------------
static int64_t swo_bit_x256(int baud, int period_ns)
{
  return (256000000000ll + (int64_t)baud * period_ns / 2) /
      ((int64_t)baud * period_ns);
}

//-----------------------------------------------------------------------------
// The shortest interior run, guarded against a lone spike the same way the
// UART's auto-baud is: one bit time, or a small multiple of it on traffic that
// never sends two unlike bits in a row.
static int swo_min_run(const LogicScratch *s, int runs)
{
  int tmin = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (0 == tmin || s->len[r] < tmin)
      tmin = s->len[r];
  }

  return tmin;
}

//-----------------------------------------------------------------------------
int swo_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int64_t cand[SWO_MAX_CAND];
  int cand_rate[SWO_MAX_CAND];
  int n_cand = 0;
  int mid, runs;

  memset(out, 0, sizeof(*out));
  memset(&g_swo, 0, sizeof(g_swo));
  out->proto = PROTO_SWO;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 6 || period_ns <= 0)
    return 0;

  int tmin = swo_min_run(scratch, runs);

  if (tmin < SWO_MIN_BIT_SAMPLES)
    return 0;

  // A run is a whole number of bit times, so a bit wider than the shortest run
  // (plus sampling slack) is not in this record at any rate
  int64_t bit_max = (int64_t)tmin * 256 * 5 / 4;
  int64_t bit_min = (int64_t)SWO_MIN_BIT_SAMPLES * 256;

  for (unsigned i = 0; i < ARRAY_SIZE(g_swo_rates) && n_cand < SWO_MAX_CAND;
      i++)
  {
    int64_t bit = swo_bit_x256(g_swo_rates[i], period_ns);

    if (bit < bit_min || bit > bit_max)
      continue;

    if (bit * SWO_FRAME_BITS > (int64_t)size * 256)
      continue;

    cand[n_cand] = bit;
    cand_rate[n_cand] = g_swo_rates[i];
    n_cand++;
  }

  // ...and the rate the record itself implies, which is the one that usually
  // wins: SWO comes out of an integer divider off the trace clock and lands
  // wherever it lands
  {
    int64_t sum = 0;
    int count = 0;

    for (int r = 1; r + 1 < runs; r++)
    {
      int bits = (scratch->len[r] + tmin / 2) / tmin;

      if (bits < 1 || bits > SWO_FRAME_BITS)
        continue;

      sum += scratch->len[r];
      count += bits;
    }

    if (count >= 6 && n_cand < SWO_MAX_CAND)
    {
      int64_t bit = sum * 256 / count;

      if (bit >= bit_min && bit * SWO_FRAME_BITS <= (int64_t)size * 256 &&
          256000000000ll / (bit * period_ns) >= SWO_MIN_RATE)
      {
        cand[n_cand] = bit;
        cand_rate[n_cand] = (int)(256000000000ll / (bit * period_ns));
        n_cand++;
      }
    }
  }

  if (0 == n_cand)
    return 0;

  // The caller's own result is the scratch pad for the search, and the winner
  // is decoded again at the end. A LogicResult is over half a kilobyte and an
  // analysis another third of one; keeping a copy of each would cost a
  // kilobyte of tightly coupled memory to save one re-run of a scan that has
  // already been run a dozen times.
  int best = 0, best_rate = 0, best_f = 0;
  int64_t best_bit = 0;

  for (int c = 0; c < n_cand; c++)
  {
    if (swo_scan(data, size, offset, mid, cand[c], scratch, runs, out) <
        SWO_MIN_BYTES)
      continue;

    // The record can begin in the middle of a packet, so the first whole one
    // starts at one of five places. Which of them is a question the grammar
    // answers on its own: a wrong start leaves unaccountable bytes behind it.
    for (int f = 0; f < SWO_RESYNC_MAX && f < out->count; f++)
    {
      swo_itm_parse(out->bytes, out->count, f, &g_swo);

      int sc = swo_score(&g_swo);

      if (sc > best)
      {
        best = sc;
        best_bit = cand[c];
        best_rate = cand_rate[c];
        best_f = f;
      }

      if (0 == g_swo.bad)
        break;      // nothing left over: no later start can do better
    }
  }

  if (0 == best_bit)
    return 0;

  swo_scan(data, size, offset, mid, best_bit, scratch, runs, out);

  if (out->count < SWO_MIN_BYTES)
    return 0;

  swo_itm_parse(out->bytes, out->count, best_f, &g_swo);
  g_swo.rate = best_rate;

  // Three gates, and every one of them is about the same question: does this
  // record tell a trace pin from a serial console, which is the same signal
  // electrically and reaches this decoder for exactly that reason.
  //
  // The first is that the packets have to TILE the record. Real ITM traffic
  // is a packet stream with nothing between the packets, so a byte left over
  // is a byte the grammar could not account for - and English text leaves
  // them within a line. A leading partial packet is not one of these: the
  // resync above has already chosen where the first whole packet starts.
  //
  // Except behind a synchronisation sequence, which is 47 zero bits and a one
  // and is not something a console sends by accident. That much is proof on
  // its own, and a record holding it may have a damaged byte in it as well.
  if (g_swo.packets < 2 || (g_swo.bad > 0 && 0 == g_swo.sync))
    return 0;

  // The second: something must have SENT something. A run of timestamps says
  // when, and nothing about what.
  if (0 == g_swo.sync && g_swo.sw < 2 && g_swo.hw < 2)
    return 0;

  // And the third, for the case that gets through: a stimulus port is a
  // CHANNEL, and a channel gets written to more than once - a printf sends
  // every character of a line down port 0. Bytes that merely happen to parse
  // scatter across the thirty-two ports instead, one packet each, which is
  // what this counts.
  {
    int ports = 0;

    for (int i = 0; i < 32; i++)
      ports += (int)((g_swo.portmask >> i) & 1);

    if (0 == g_swo.sync && ports * 2 > g_swo.sw + 1)
      return 0;
  }

  swo_text(out->bytes, &g_swo);

  out->rate = best_rate;
  out->errors = g_swo.bad;

  // A handful of packets and no synchronisation sequence is a reading the
  // record does not settle: the gates above make it unlikely to be a console,
  // not impossible. The cascade is the only place that knows whether anyone
  // asked for SWO by name, so the doubt is reported rather than acted on.
  out->ambiguous = (0 == g_swo.sync) && (g_swo.packets < 4);

  char rate[12];

  if (best_rate >= 1000000)
    snprintf(rate, sizeof(rate), "%d.%02dM", best_rate / 1000000,
        (best_rate % 1000000) / 10000);
  else
    snprintf(rate, sizeof(rate), "%d.%dk", best_rate / 1000,
        (best_rate % 1000) / 100);

  // The text is the point of the pin, so it goes in the header when there is
  // any: a log line read off one wire is worth more than a packet census.
  if (g_swo.text[0] && g_swo.text_port >= 0)
    snprintf(out->info, sizeof(out->info), "ITM %s p%d \"%.12s\"", rate,
        g_swo.text_port, g_swo.text);
  else
    snprintf(out->info, sizeof(out->info), "ITM %s %dpk%s%s", rate,
        g_swo.packets, g_swo.sync ? " sync" : "",
        g_swo.bad ? " ?" : "");

  return out->count;
}
