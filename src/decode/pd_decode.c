/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * USB Power Delivery, read off the CC wire.
 *
 * After the USB decoder this one is a relief: CC is single-ended by design.
 * There is no differential pair to half-see, no invisible SE0, and the end of
 * a message is an EOP symbol on the same wire as everything else rather than
 * a line state one probe cannot reach. Everything USB 2.0 made hard, Power
 * Delivery simply hands over.
 *
 * The line code is BMC - biphase mark. There is a transition at EVERY bit
 * boundary, and a one carries a second transition in the middle of the bit.
 * So a zero is one run of a unit interval and a one is two runs of half of
 * one, the code is polarity-blind, and reading it needs no phase tracking at
 * all: the run lengths ARE the bits. 300 kbit nominal, so a unit interval is
 * 3.33 us and half of one is 1.67 us - which at the timebases this decodes on
 * is twenty-six to a hundred samples. Compared with full-speed USB's ten to a
 * bit there is nothing here to be careful about.
 *
 * What a message is made of:
 *
 *   preamble     64 bits of alternating 0 and 1. Carries nothing, and is what
 *                the unit interval is measured from.
 *   ordered set  four 5-bit K-codes. SOP addresses the port partner, SOP' and
 *                SOP'' the plugs in the cable, and Hard Reset and Cable Reset
 *                are events with no message behind them at all.
 *   header       16 bits: how many data objects, which message, whose turn.
 *   data         those objects, four bytes each, up to seven.
 *   CRC32        over the header and the data. The confirmation.
 *   EOP          one more K-code.
 *
 * Everything between the ordered set and the EOP is 4b5b: four bits of data
 * to five on the wire, out of a sixteen-entry table that shares its alphabet
 * with FDDI. The six codes that are NOT in it are the K-codes.
 *
 * THE ONE ASSUMPTION, AND IT IS NOT ASSUMED. Which end of a five-bit symbol
 * goes out first is the single thing here worth getting wrong, so it is not
 * decided in advance: both readings of the ordered set are looked for, the
 * one that matches picks the order, and the CRC32 then either agrees or does
 * not. A wrong guess cannot produce a wrong answer, only no answer - and the
 * order that won is reported, because on a real capture that is a fact worth
 * writing down.
 *
 * WHY THIS ONE WALKS THE SAMPLES ITSELF. Every other decoder here reads the
 * shared run split in LogicScratch, which stops at LOGIC_MAX_RUNS = 512. BMC
 * spends one run on a zero and two on a one, so a message costs about 1.6
 * runs a bit: a 64-bit preamble is 96 runs before anything is said, and a
 * Source_Capabilities advertising five supplies comes to roughly 550. That is
 * over the cap - and it is exactly the message anyone points a probe at CC to
 * read. Raising the cap is not available either: LogicScratch is already 4636
 * bytes of the 5 KB of spare SRAM there is. So this decoder thresholds and
 * walks the record in one forward pass of its own, holds one message at a
 * time, and has no length limit.
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
#define PD_RATE_NOMINAL      300000

// The specification allows +-10% on the bit rate. The gate is a shade wider,
// because what CONFIRMS a message is its CRC32 and this only exists to keep
// the cascade from walking the record for nothing.
#define PD_RATE_MIN          255000
#define PD_RATE_MAX          345000

// Four samples to a half unit interval. At 1.67 us that is a 417 ns sample
// period, so every timebase up to about 2 ms/div reads and the ones past it
// answer 0 rather than guess.
#define PD_MIN_HALF_SAMPLES  4

// Header, extended header, seven objects, CRC32
#define PD_MAX_MSG_BYTES     (2 + 2 + 7 * 4 + 4)

// A run of BMC in the shared split, long enough that the record is worth
// walking. The preamble alone is 96 of them.
#define PD_MIN_BMC_RUNS      40

// K-codes: the six five-bit symbols that are not in the data table
#define PD_K_SYNC1  0x18u    // 11000
#define PD_K_SYNC2  0x11u    // 10001
#define PD_K_SYNC3  0x06u    // 00110
#define PD_K_RST1   0x07u    // 00111
#define PD_K_RST2   0x19u    // 11001
#define PD_K_EOP    0x0Du    // 01101

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_pd   (g_logic_analysis.pd)

// In pd_sop_t order
static const uint8_t g_pd_os[PD_SOP_COUNT][4] =
{
  { PD_K_SYNC1, PD_K_SYNC1, PD_K_SYNC1, PD_K_SYNC2 },  // SOP
  { PD_K_SYNC1, PD_K_SYNC1, PD_K_SYNC3, PD_K_SYNC3 },  // SOP'
  { PD_K_SYNC1, PD_K_SYNC3, PD_K_SYNC1, PD_K_SYNC3 },  // SOP''
  { PD_K_RST1,  PD_K_RST1,  PD_K_RST1,  PD_K_RST2  },  // Hard Reset
  { PD_K_RST1,  PD_K_SYNC1, PD_K_RST1,  PD_K_SYNC3 },  // Cable Reset
  { PD_K_SYNC1, PD_K_RST2,  PD_K_RST2,  PD_K_SYNC3 },  // SOP'_Debug
  { PD_K_SYNC1, PD_K_RST2,  PD_K_SYNC3, PD_K_SYNC2 },  // SOP''_Debug
};

// Five bits on the wire back to the four they carry; 0xFF for the six K-codes
// and the ten groups that are in no table at all
static const uint8_t g_pd_5b4b[32] =
{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x01, 0x04, 0x05, 0xFF, 0xFF, 0x06, 0x07,
  0xFF, 0xFF, 0x08, 0x09, 0x02, 0x03, 0x0A, 0x0B,
  0xFF, 0xFF, 0x0C, 0x0D, 0x0E, 0x0F, 0x00, 0xFF,
};

// Control messages: the header says zero data objects, and the type is the
// whole of what was said. Names kept to twelve characters - the label buffer
// the panel hands out is sixteen and a SOP' prefix takes two of them.
static const char *const g_pd_ctrl[] =
{
  "Reserved",   "GoodCRC",    "GotoMin",    "Accept",
  "Reject",     "Ping",       "PS_RDY",     "Get_Src_Cap",
  "Get_Snk_Cap", "DR_Swap",   "PR_Swap",    "VCONN_Swap",
  "Wait",       "Soft_Reset", "Data_Reset", "Data_RstOK",
  "Not_Suppd",  "GetSrcCapX", "Get_Status", "FR_Swap",
  "GetPPSStat", "GetCountry", "GetSnkCapX", "GetSrcInfo",
  "Get_Rev",
};

// ...and data messages, where the objects behind the header are the point
static const char *const g_pd_data[] =
{
  "Reserved",   "Src_Cap",    "Request",    "BIST",
  "Snk_Cap",    "BattStatus", "Alert",      "GetCountry",
  "Enter_USB",  "EPR_Request", "EPR_Mode",  "Src_Info",
  "Revision",
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const PdAnalysis *pd_analysis(void)
{
  return &g_pd;
}

//-----------------------------------------------------------------------------
static inline int pd_sample(const uint8_t *data, int size, int offset, int i)
{
  int index = offset + i;

  if (index >= size)
    index -= size;

  return data[index];
}

//-----------------------------------------------------------------------------
static inline uint8_t pd_rev5(uint8_t v)
{
  uint8_t r = 0;

  for (int i = 0; i < 5; i++)
    r = (uint8_t)(r | (((v >> i) & 1u) << (4 - i)));

  return r;
}

//-----------------------------------------------------------------------------
// CRC32 as Ethernet and everything descended from it computes it: poly
// x^32+x^26+..., reflected into 0xEDB88320, seeded all ones, inverted at the
// end. Over the header and the data objects, and nothing else - the preamble,
// the ordered set and the EOP are outside it.
//
// Anchor: this returns 0xCBF43926 for the nine bytes "123456789", which is the
// check value every implementation of this polynomial is measured against.
static uint32_t pd_crc32(const uint8_t *d, int n)
{
  uint32_t crc = 0xFFFFFFFFu;

  for (int i = 0; i < n; i++)
  {
    crc ^= d[i];

    for (int k = 0; k < 8; k++)
      crc = (0 != (crc & 1u)) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
  }

  return ~crc;
}

//-----------------------------------------------------------------------------
// The record thresholded into level runs, one at a time and without storing
// any of them. This is logic_runs' Schmitt comparator walked forward instead
// of filled into an array - see the file header for why this decoder cannot
// use the array.
typedef struct
{
  const uint8_t *data;
  int   size;
  int   offset;
  int   mid;
  int   hyst;
  int   level;
  int   i;        // the next sample to look at
  int   start;    // where the run in progress began
} PdWalk;

//-----------------------------------------------------------------------------
static void pd_walk_init(PdWalk *w, const uint8_t *data, int size, int offset,
    int mid, int hyst)
{
  w->data = data;
  w->size = size;
  w->offset = offset;
  w->mid = mid;
  w->hyst = hyst;
  w->level = (pd_sample(data, size, offset, 0) > mid) ? 1 : 0;
  w->i = 1;
  w->start = 0;
}

//-----------------------------------------------------------------------------
// The length of the next COMPLETED run, or -1 when the record ends inside one.
// A run that never finished is not a bit, so it is not returned as one.
static int pd_walk_run(PdWalk *w, int *pos)
{
  while (w->i < w->size)
  {
    int v = pd_sample(w->data, w->size, w->offset, w->i);
    int nl = w->level;

    if (0 == w->level && v > w->mid + w->hyst)
      nl = 1;
    else if (1 == w->level && v < w->mid - w->hyst)
      nl = 0;

    if (nl != w->level)
    {
      int len = w->i - w->start;

      *pos = w->start;
      w->start = w->i;
      w->level = nl;
      w->i++;

      return len;
    }

    w->i++;
  }

  return -1;
}

//-----------------------------------------------------------------------------
// BMC on top of it: run lengths in, bits out. No phase to keep - a whole unit
// interval IS a zero and two halves ARE a one, and every bit re-anchors on its
// own edges. Which is why a bit rate a few percent off costs nothing here and
// costs the USB decoder a whole search.
typedef struct
{
  PdWalk   w;
  int64_t  half;     // half a unit interval, in samples, x256
  int      pend;     // a bit already read and not yet handed out; -1 for none
  int      pend_pos;
  int      pend_end;
  bool     eot;      // a run too long to be inside a transmission
  bool     err;      // half a bit with nothing to pair it with
} PdBmc;

//-----------------------------------------------------------------------------
static void pd_bmc_init(PdBmc *b, const uint8_t *data, int size, int offset,
    int mid, int hyst, int64_t half_x256)
{
  pd_walk_init(&b->w, data, size, offset, mid, hyst);

  b->half = half_x256;
  b->pend = -1;
  b->eot = false;
  b->err = false;
}

//-----------------------------------------------------------------------------
// Step forward to a run that is a WHOLE unit interval and hold it as the next
// bit. Only a whole one is unambiguously a bit boundary: half a unit interval
// could be either half of a one, so starting on it would frame the entire
// message a half-bit out. The preamble alternates, so a boundary is never
// more than three runs away.
static bool pd_bmc_align(PdBmc *b)
{
  for (;;)
  {
    int pos;
    int len = pd_walk_run(&b->w, &pos);

    if (len < 0)
      return false;

    int64_t l = (int64_t)len * 256;

    if (l >= b->half * 3 / 2 && l <= b->half * 3)
    {
      b->pend = 0;                    // a whole interval with no transition
      b->pend_pos = pos;
      b->pend_end = pos + len;
      b->eot = false;
      b->err = false;

      return true;
    }
  }
}

//-----------------------------------------------------------------------------
// The next BMC bit, or -1. `pos` and `end` receive the samples it spans.
static int pd_bmc_bit(PdBmc *b, int *pos, int *end)
{
  if (b->pend >= 0)
  {
    int v = b->pend;

    *pos = b->pend_pos;
    *end = b->pend_end;
    b->pend = -1;

    return v;
  }

  int p1, p2;
  int r1 = pd_walk_run(&b->w, &p1);

  if (r1 < 0)
    return -1;

  int64_t l1 = (int64_t)r1 * 256;

  if (l1 > b->half * 3)
  {
    b->eot = true;                    // longer than any bit: the line went idle

    return -1;
  }

  if (l1 >= b->half * 3 / 2)
  {
    *pos = p1;                        // a whole unit interval: a zero
    *end = p1 + r1;

    return 0;
  }

  // Half of one, so the transition in the middle of a one - and the other
  // half has to be right behind it, or this was never a bit boundary
  int r2 = pd_walk_run(&b->w, &p2);

  if (r2 < 0)
    return -1;

  if ((int64_t)r2 * 256 >= b->half * 3 / 2)
  {
    b->err = true;

    return -1;
  }

  *pos = p1;
  *end = p2 + r2;

  return 1;
}

//-----------------------------------------------------------------------------
// The twenty bits of an ordered set as they arrive, for one reading of the
// symbol order. Bits are accumulated most-significant-first, so a symbol sent
// leftmost-bit-first lands as its table value and one sent the other way round
// lands reversed.
static uint32_t pd_os_word(const uint8_t *k, bool lsb)
{
  uint32_t v = 0;

  for (int i = 0; i < 4; i++)
    v |= (uint32_t)(lsb ? pd_rev5(k[i]) : k[i]) << (15 - 5 * i);

  return v;
}

//-----------------------------------------------------------------------------
// Which ordered set the last twenty bits are, and which way round the symbols
// came. -1 for none.
//
// Both readings are tried because nothing in the waveform decides the symbol
// order and guessing it would be the one way this decoder could be quietly
// wrong. Nothing is at risk in trying: the reversal of a K-code is in neither
// table, so the two readings cannot both match.
static int pd_match_os(uint32_t sr, bool *lsb)
{
  for (int i = 0; i < PD_SOP_COUNT; i++)
  {
    if (sr == pd_os_word(g_pd_os[i], false))
    {
      *lsb = false;

      return i;
    }

    if (sr == pd_os_word(g_pd_os[i], true))
    {
      *lsb = true;

      return i;
    }
  }

  return -1;
}

//-----------------------------------------------------------------------------
// The next five-bit group as the four bits it stands for, or -1 when it is in
// no table (a K-code where data was due, or a group that codes nothing).
// `start` and `end` receive the samples the group spans.
static int pd_symbol(PdBmc *b, bool lsb, int *start, int *end, bool *ran_out)
{
  uint32_t sym = 0;

  *ran_out = false;

  for (int k = 0; k < 5; k++)
  {
    int p, e;
    int bit = pd_bmc_bit(b, &p, &e);

    if (bit < 0)
    {
      *ran_out = true;

      return -1;
    }

    if (0 == k)
      *start = p;

    sym = (sym << 1) | (uint32_t)bit;
    *end = e;
  }

  uint8_t code = lsb ? pd_rev5((uint8_t)sym) : (uint8_t)sym;

  return (0xFF == g_pd_5b4b[code]) ? -1 : (int)g_pd_5b4b[code];
}

//-----------------------------------------------------------------------------
// One byte: two symbols, the low nibble first. False when the message ended
// inside it - and `ran_out` says whether that was the record running out (a
// fact about the window) or a group in no table (a fact about the signal).
static bool pd_byte(PdBmc *b, bool lsb, uint8_t *out, int *start, int *end,
    bool *ran_out, int *sym_err)
{
  int s1 = 0, e1 = 0, s2 = 0, e2 = 0;
  int lo = pd_symbol(b, lsb, &s1, &e1, ran_out);

  if (lo < 0)
  {
    if (!*ran_out)
      (*sym_err)++;

    *ran_out = true;

    return false;
  }

  int hi = pd_symbol(b, lsb, &s2, &e2, ran_out);

  if (hi < 0)
  {
    if (!*ran_out)
      (*sym_err)++;

    *ran_out = true;

    return false;
  }

  *out = (uint8_t)(lo | (hi << 4));
  *start = s1;
  *end = e2;

  return true;
}

//-----------------------------------------------------------------------------
// A message's type as a name. The SOP marker rides in front of it, because a
// Vendor_Defined to the cable and a Vendor_Defined to the port partner are
// different conversations and the header does not say which.
static void pd_msg_name(const PdMessage *m, char *buf, int size)
{
  const char *tick = "";

  if (PD_SOP_P == m->sop || PD_SOP_P_DBG == m->sop)
    tick = "'";
  else if (PD_SOP_PP == m->sop || PD_SOP_PP_DBG == m->sop)
    tick = "''";

  if (0 == m->ndo)
  {
    const char *n = (m->type < (int)(sizeof(g_pd_ctrl) / sizeof(g_pd_ctrl[0])))
        ? g_pd_ctrl[m->type] : "Ctrl?";

    snprintf(buf, size, "%s%s", tick, n);

    return;
  }

  if (15 == m->type)
  {
    snprintf(buf, size, "%sVendor_Def", tick);

    return;
  }

  const char *n = (m->type < (int)(sizeof(g_pd_data) / sizeof(g_pd_data[0])))
      ? g_pd_data[m->type] : "Data?";

  snprintf(buf, size, "%s%s", tick, n);
}

//-----------------------------------------------------------------------------
// What a Power Data Object offers. Fixed supplies are what nearly every
// charger advertises and what anyone pointing a probe at CC came to read, so
// they get their volts and their amps in full; the rest are named and bounded.
static void pd_pdo_text(uint32_t o, char *buf, int size)
{
  unsigned kind = (o >> 30) & 3u;

  if (0 == kind)
  {
    int mv = (int)((o >> 10) & 0x3FFu) * 50;
    int ma = (int)(o & 0x3FFu) * 10;

    snprintf(buf, size, "%d.%dV %d.%dA", mv / 1000, (mv % 1000) / 100,
        ma / 1000, (ma % 1000) / 100);
  }
  else if (1 == kind)
  {
    int mv = (int)((o >> 20) & 0x3FFu) * 50;
    int mw = (int)(o & 0x3FFu) * 250;

    snprintf(buf, size, "BAT %d.%dV %dW", mv / 1000, (mv % 1000) / 100,
        mw / 1000);
  }
  else if (2 == kind)
  {
    int mv = (int)((o >> 20) & 0x3FFu) * 50;
    int ma = (int)(o & 0x3FFu) * 10;

    snprintf(buf, size, "VAR %dV %d.%dA", mv / 1000, ma / 1000,
        (ma % 1000) / 100);
  }
  else
  {
    // Augmented, and the only one anybody meets is programmable supply: a
    // range rather than a point, which is what makes it worth marking
    int mv = (int)((o >> 17) & 0xFFu) * 100;
    int ma = (int)(o & 0x7Fu) * 50;

    snprintf(buf, size, "PPS %dV %d.%dA", mv / 1000, ma / 1000,
        (ma % 1000) / 100);
  }
}

//-----------------------------------------------------------------------------
// ...and what a Request asked for. The object position is the whole point:
// it names one of the supplies the source advertised, and by itself means
// nothing at all - which is what pd_check_request exists to say.
static void pd_rdo_text(uint32_t o, char *buf, int size)
{
  int pos = (int)((o >> 28) & 0x0Fu);
  int ma = (int)((o >> 10) & 0x3FFu) * 10;

  snprintf(buf, size, "#%d %d.%dA", pos, ma / 1000, (ma % 1000) / 100);
}

//-----------------------------------------------------------------------------
// Does a Source_Capabilities obey the rules it has to obey on its own? The
// first object must be a 5 V fixed supply - every sink is entitled to that
// one - and the fixed objects after it must climb in voltage. This is the
// only agreement check that needs a single message and nothing else.
static bool pd_caps_ordered(const uint32_t *o, int n)
{
  if (n < 1)
    return false;

  if (0 != ((o[0] >> 30) & 3u) || 100 != (int)((o[0] >> 10) & 0x3FFu))
    return false;                     // 100 x 50 mV: the mandatory 5 V supply

  int prev = 100;

  for (int i = 1; i < n; i++)
  {
    if (0 != ((o[i] >> 30) & 3u))
      continue;                       // the fixed ones come first and the

    int mv = (int)((o[i] >> 10) & 0x3FFu);   // rest are not ordered by voltage

    if (mv <= prev)
      return false;

    prev = mv;
  }

  return true;
}

//-----------------------------------------------------------------------------
// Everything the decoder can say about whether the traffic AGREES with itself.
// Three separate claims, and they are worth keeping apart:
//
//   the CRC32     per message, and the strongest of the three. It says the
//                 bits are what the transmitter sent.
//   the GoodCRC   a message and the acknowledgement of it are tens of
//                 microseconds apart, so they are all but certain to be in the
//                 same record - and the acknowledgement carries the MessageID
//                 of what it acknowledges. That pairing is the only piece of
//                 protocol-level agreement a single record can establish.
//   the Request   against the capabilities it answers. This one is worth the
//                 most and is the least likely to fit: up to 30 ms may pass
//                 between them, and a 500 us/div record spans 6.3 ms. So it is
//                 checked when both are here and says so plainly when not.
static void pd_check(PdAnalysis *a)
{
  a->req_pos = -1;
  a->req_check = -1;

  int caps = -1, req = -1;

  for (int i = 0; i < a->msgs; i++)
  {
    PdMessage *m = &a->msg[i];

    if (!m->crc_ok)
      continue;

    // A GoodCRC is never itself acknowledged, so it cannot open a pair
    if (0 == m->ndo && 1 == m->type)
      continue;

    // ...and the one behind a message, carrying that message's MessageID, is
    // the acknowledgement of it. Both halves are on the same wire and tens of
    // microseconds apart, which is why this pairing nearly always fits in one
    // record where the Request/Capabilities one nearly always does not.
    if (i + 1 < a->msgs && 0 == a->msg[i + 1].ndo && 1 == a->msg[i + 1].type &&
        a->msg[i + 1].crc_ok && a->msg[i + 1].id == m->id)
    {
      m->acked = true;
      a->acked++;
    }

    if (0 != m->ndo && !m->ext && 1 == m->type)
    {
      caps = i;
      a->caps_seen = true;
      a->caps_ordered = pd_caps_ordered(&a->obj[m->obj0], m->ndo);
    }

    if (0 != m->ndo && !m->ext && 2 == m->type)
    {
      req = i;
      a->req_pos = (int8_t)((a->obj[m->obj0] >> 28) & 0x0Fu);
    }
  }

  if (req < 0 || caps < 0)
    return;                           // one half of the question is missing,
                                      // and req_check stays -1 to say so
  const PdMessage *c = &a->msg[caps];

  if (a->req_pos < 1 || a->req_pos > c->ndo)
  {
    a->req_check = 0;                 // asked for a supply that was not offered

    return;
  }

  uint32_t pdo = a->obj[c->obj0 + a->req_pos - 1];

  if (0 == ((pdo >> 30) & 3u))
  {
    a->req_mv = (int)((pdo >> 10) & 0x3FFu) * 50;
    a->req_ma = (int)(pdo & 0x3FFu) * 10;
  }
  else if (3 == ((pdo >> 30) & 3u))
  {
    a->req_mv = (int)((pdo >> 17) & 0xFFu) * 100;
    a->req_ma = (int)(pdo & 0x7Fu) * 50;
  }
  else
  {
    a->req_mv = (int)((pdo >> 20) & 0x3FFu) * 50;
    a->req_ma = (int)(pdo & 0x3FFu) * 10;
  }

  // ...and the current it asked to draw has to be inside what that supply
  // offers, or the two do not agree whatever the checksums said
  int want = (int)((a->obj[a->msg[req].obj0] >> 10) & 0x3FFu) * 10;

  a->req_check = (want <= a->req_ma) ? 1 : 0;
}

//-----------------------------------------------------------------------------
// One pass over the record. Fills the analysis and the result when `out` is
// given; otherwise it only counts, which is what the half-interval search
// needs. Returns a score: a CRC-confirmed message is worth four bare ones.
static int pd_pass(const uint8_t *data, int size, int offset, int mid,
    int hyst, int64_t half, PdAnalysis *a, LogicResult *out)
{
  PdBmc bmc;
  uint8_t mb[PD_MAX_MSG_BYTES];
  int mp[PD_MAX_MSG_BYTES + 1];
  int msgs = 0, crcs = 0, resets = 0, sym_err = 0;
  bool lsb = false, lsb_known = false;

  pd_bmc_init(&bmc, data, size, offset, mid, hyst, half);

  if (a)
  {
    memset(a->role, 0, sizeof(a->role));
    memset(a->midx, 0xFF, sizeof(a->midx));
    a->obj_count = 0;
  }

  while (msgs < PD_MAX_MSGS)
  {
    if (!pd_bmc_align(&bmc))
      break;

    // Hunt for an ordered set. The preamble in front of it is 64 bits that
    // say nothing, so there is no point reading it as anything - the shift
    // register simply runs until four K-codes line up in it.
    uint32_t sr = 0;
    int have = 0;
    int os = -1;
    bool os_lsb = false;

    for (;;)
    {
      int p, e;
      int bit = pd_bmc_bit(&bmc, &p, &e);

      if (bit < 0)
        break;

      sr = ((sr << 1) | (uint32_t)bit) & 0xFFFFFu;

      if (++have >= 20)
      {
        os = pd_match_os(sr, &os_lsb);

        if (os >= 0)
          break;
      }
    }

    if (os < 0)
    {
      if (bmc.eot || bmc.err)
      {
        bmc.eot = false;              // a gap, or a reading that came apart:
        bmc.err = false;              // re-align and carry on down the record

        continue;
      }

      break;                          // the record ran out
    }

    // Once one symbol order has matched, it is the record's. A second reading
    // turning up halfway down would mean the first was a coincidence, and a
    // coincidence is not something to switch conventions on.
    if (lsb_known && os_lsb != lsb)
      continue;

    lsb = os_lsb;
    lsb_known = true;

    if (PD_HARD_RESET == os || PD_CABLE_RESET == os)
    {
      // Nothing follows these. The ordered set IS the event - there is no
      // header, no payload and no CRC behind it, so there are no bytes to
      // report and reporting some would be inventing them.
      resets++;

      continue;
    }

    PdMessage m;
    int n = 0;
    bool ran_out = false;

    memset(&m, 0, sizeof(m));
    m.sop = (uint8_t)os;

    // The header first, because it is what says how much else there is
    while (n < 2 && !ran_out)
    {
      int s = 0, e = 0;

      if (!pd_byte(&bmc, lsb, &mb[n], &s, &e, &ran_out, &sym_err))
        break;

      if (0 == n)
        mp[0] = s;

      n++;
      mp[n] = e;
    }

    if (n < 2)
      break;                          // no header, so nothing to frame

    uint32_t hdr = (uint32_t)(mb[0] | (mb[1] << 8));

    m.ext = (0 != (hdr & 0x8000u));
    m.ndo = (uint8_t)((hdr >> 12) & 7u);
    m.id = (uint8_t)((hdr >> 9) & 7u);
    m.src = (0 != (hdr & 0x0100u));
    m.rev = (uint8_t)((hdr >> 6) & 3u);
    m.dfp = (0 != (hdr & 0x0020u));
    m.type = (uint8_t)(hdr & 0x1Fu);

    // ...and now the rest of it is a known length: the objects the header
    // counted, then four bytes of CRC32. An extended message spends the first
    // two of those bytes on its own header and the rest on a chunk of a byte
    // stream, but the framing is the same either way.
    int want = 2 + (int)m.ndo * 4 + 4;

    if (want > PD_MAX_MSG_BYTES)
      want = PD_MAX_MSG_BYTES;

    while (n < want && !ran_out)
    {
      int s = 0, e = 0;

      if (!pd_byte(&bmc, lsb, &mb[n], &s, &e, &ran_out, &sym_err))
        break;

      n++;
      mp[n] = e;
    }

    m.cut = (n < want);

    if (!m.cut)
    {
      uint32_t rx = (uint32_t)mb[n - 4] | ((uint32_t)mb[n - 3] << 8) |
          ((uint32_t)mb[n - 2] << 16) | ((uint32_t)mb[n - 1] << 24);

      m.crc = rx;
      m.crc_ok = (pd_crc32(mb, n - 4) == rx);
    }

    msgs++;

    if (m.crc_ok)
      crcs++;

    if (out)
    {
      int first = out->count;
      int dend = m.cut ? n : (n - 4);

      m.obj0 = (uint8_t)(a ? a->obj_count : 0);

      for (int k = 0; k < n; k++)
      {
        if (out->count >= LOGIC_MAX_BYTES)
        {
          out->truncated = true;

          break;
        }

        out->bytes[out->count] = mb[k];
        out->pos[out->count] = mp[k];
        out->end[out->count] = mp[k + 1];

        if (a)
        {
          uint8_t role = PD_R_DATA;

          if (k < 2)
            role = PD_R_HDR;
          else if (m.ext && k < 4)
            role = PD_R_EXT;
          else if (!m.cut && k >= n - 4)
            role = PD_R_CRC;

          a->role[out->count] = role;
          a->midx[out->count] = (uint8_t)(msgs - 1);
        }

        out->count++;
      }

      // The objects, assembled, so a label can write one value across the
      // four bytes that carry it the way the SWD decoder writes a register
      if (a && !m.ext)
      {
        for (int k = 0; k < m.ndo && a->obj_count < PD_MAX_OBJS; k++)
        {
          int o = 2 + k * 4;

          if (o + 4 > dend)
            break;

          a->obj[a->obj_count] = (uint32_t)mb[o] | ((uint32_t)mb[o + 1] << 8) |
              ((uint32_t)mb[o + 2] << 16) | ((uint32_t)mb[o + 3] << 24);
          a->obj_count++;
        }
      }

      m.first = (uint8_t)first;
      m.count = (uint8_t)(out->count - first);
    }

    if (a)
      a->msg[msgs - 1] = m;

    if (out && out->truncated)
      break;

    if (ran_out && !bmc.eot && !bmc.err)
      break;                          // the record ended inside the message

    bmc.eot = false;
    bmc.err = false;
  }

  if (a)
  {
    a->msgs = msgs;
    a->crc_ok = crcs;
    a->resets = resets;
    a->sym_err = sym_err;
    a->lsb_first = lsb;
  }

  return crcs * 4 + msgs + resets;
}

//-----------------------------------------------------------------------------
// Half a unit interval, taken off the shared run split. Only an ESTIMATE is
// wanted here and only to gate the walk: BMC re-anchors on every bit, so a
// couple of percent out costs nothing, and the CRC32 is what actually decides.
static int64_t pd_estimate_half(const LogicScratch *s, int runs)
{
  int tmin = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (0 == tmin || s->len[r] < tmin)
      tmin = s->len[r];
  }

  if (tmin < PD_MIN_HALF_SAMPLES)
    return 0;

  // The shortest run is the low END of the half-interval cluster and not its
  // middle: jitter spreads that cluster, so anchoring on the minimum reads
  // several percent short every time and a tight rate gate then turns away a
  // record that is perfectly good Power Delivery. Re-centring on the mean of
  // everything under one and a half times the estimate walks onto the cluster
  // in a few passes, and cannot run away upward - the whole intervals sit at
  // twice the half and stay outside the window wherever the estimate moves
  // inside it. Five passes is more than convergence has ever needed.
  int64_t h = (int64_t)tmin * 256;

  for (int it = 0; it < 5; it++)
  {
    int64_t sum = 0;
    int count = 0;

    for (int r = 1; r + 1 < runs; r++)
    {
      int64_t l = (int64_t)s->len[r] * 256;

      if (l < h * 3 / 2)
      {
        sum += l;
        count++;
      }
    }

    if (count < 8)
      return 0;

    h = sum / count;
  }

  return h;
}

//-----------------------------------------------------------------------------
// Is there a stretch of the record that is BMC at all? Every run either half
// a unit interval or a whole one, for long enough to be a preamble. Cheap,
// and it is what keeps this decoder from walking the samples of every record
// the cascade sees.
static bool pd_looks_like_bmc(const LogicScratch *s, int runs, int64_t half)
{
  int best = 0, cur = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    int64_t l = (int64_t)s->len[r] * 256;

    if (l >= half / 2 && l <= half * 3)
      cur++;
    else
      cur = 0;

    if (cur > best)
      best = cur;
  }

  return (best >= PD_MIN_BMC_RUNS);
}

//-----------------------------------------------------------------------------
void pd_byte_label(const PdAnalysis *a, int idx, uint8_t v, char *buf, int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || 0xFF == a->midx[idx] ||
      a->midx[idx] >= PD_MAX_MSGS)
    return;

  const PdMessage *m = &a->msg[a->midx[idx]];

  // Every field here is spread over two or four bytes and is written ONCE, on
  // the byte that completes it - the same way a CAN identifier and a 1-Wire
  // temperature are written across the bytes that make them
  if (PD_R_HDR == a->role[idx])
  {
    if (idx == m->first + 1)
      pd_msg_name(m, buf, size);

    return;
  }

  if (PD_R_CRC == a->role[idx])
  {
    if (idx == m->first + m->count - 1)
      snprintf(buf, size, "crc %08lX%s", (unsigned long)m->crc,
          m->crc_ok ? "" : "!");

    return;
  }

  if (PD_R_EXT == a->role[idx])
  {
    if (idx == m->first + 3)
      snprintf(buf, size, "ext chunk");

    return;
  }

  if (PD_R_DATA != a->role[idx] || m->ext)
    return;

  // A data object is four bytes and one number. Which number depends on the
  // message: the same thirty-two bits are a supply in a Source_Capabilities
  // and a demand in a Request.
  int d0 = m->first + 2;
  int k = (idx - d0) / 4;

  if (idx != d0 + k * 4 + 3 || k < 0 || k >= m->ndo)
    return;

  if (m->obj0 + k >= a->obj_count)
    return;

  uint32_t o = a->obj[m->obj0 + k];

  if (2 == m->type)
    pd_rdo_text(o, buf, size);
  else if (1 == m->type || 4 == m->type)
    pd_pdo_text(o, buf, size);
  else
    snprintf(buf, size, "%08lX", (unsigned long)o);
}

//-----------------------------------------------------------------------------
void pd_field_label(const PdAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || 0xFF == a->midx[idx] ||
      a->midx[idx] >= PD_MAX_MSGS)
    return;

  const PdMessage *m = &a->msg[a->midx[idx]];

  switch (a->role[idx])
  {
    case PD_R_HDR:
      snprintf(buf, size, "HDR%d", idx - m->first);
      break;

    case PD_R_EXT:
      snprintf(buf, size, "EXT%d", idx - m->first - 2);
      break;

    case PD_R_CRC:
      snprintf(buf, size, "C%d", idx - (m->first + m->count - 4));
      break;

    case PD_R_DATA:
      if (m->ext)
        snprintf(buf, size, "D%d", idx - m->first - 4);
      else
        snprintf(buf, size, "O%d", (idx - m->first - 2) / 4 + 1);
      break;

    default:
      break;
  }
}

//-----------------------------------------------------------------------------
// A header is one sixteen-bit field, a CRC32 is one number and a data object
// is one more - all of them shown as one. The payload of an EXTENDED message
// is not: it is a byte stream by definition, and grouping it in fours would
// be drawing structure that is not there.
void pd_group_at(const PdAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || 0xFF == a->midx[idx] ||
      a->midx[idx] >= PD_MAX_MSGS)
    return;

  const PdMessage *m = &a->msg[a->midx[idx]];

  switch (a->role[idx])
  {
    case PD_R_HDR:  *start = m->first;                 *len = 2; break;
    case PD_R_EXT:  *start = m->first + 2;             *len = 2; break;
    case PD_R_CRC:  *start = m->first + m->count - 4;  *len = 4; break;

    case PD_R_DATA:
      if (m->ext)
        return;
      {
        int d0 = m->first + 2;

        *start = d0 + ((idx - d0) / 4) * 4;
        *len = 4;
      }
      break;

    default:
      return;
  }

  // A message the record cut short holds fewer bytes than its shape says, and
  // a group must never name bytes that never arrived
  int limit = m->first + m->count;

  if (*start + *len > limit)
    *len = limit - *start;

  if (*len < 1)
  {
    *start = idx;
    *len = 1;
  }
}

//-----------------------------------------------------------------------------
int pd_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  memset(&g_pd, 0, sizeof(g_pd));
  out->proto = PROTO_PD;
  g_pd.req_pos = -1;
  g_pd.req_check = -1;

  // The shared split is used for the GATE and for nothing else: it is already
  // in hand from the decoders ahead of this one, and one look at it settles
  // whether the record is worth walking. What it cannot do is carry the
  // decode - see the file header.
  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < PD_MIN_BMC_RUNS || period_ns <= 0)
    return 0;

  int64_t half = pd_estimate_half(scratch, runs);

  if (half < PD_MIN_HALF_SAMPLES * 256)
    return 0;

  // A unit interval is two halves, and Power Delivery runs at 300 kbit and
  // nothing else. A record whose unit time is outside that band is not PD,
  // and this is where nearly every record leaves.
  int64_t ui_ns = half * 2 * period_ns / 256;

  if (ui_ns <= 0)
    return 0;

  int rate = (int)(1000000000ll / ui_ns);

  if (rate < PD_RATE_MIN || rate > PD_RATE_MAX)
    return 0;

  if (!pd_looks_like_bmc(scratch, runs, half))
    return 0;

  // The threshold the walk needs. logic_runs hands back the midpoint but not
  // the swing it took it from, and the Schmitt band has to match the one the
  // split used or the two would disagree about what a run is.
  int vmin = 255, vmax = 0;

  for (int i = 0; i < size; i++)
  {
    int v = pd_sample(data, size, offset, i);

    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }

  int hyst = (vmax - vmin) / 8;

  // Decode at the estimate, and only widen if it did not work. Every pass
  // here walks the whole record, so a five-step search the way the USB
  // decoder does it would be five walks of 98304 samples for a rate that is
  // very probably already right: BMC re-anchors on every single bit, the
  // half-and-whole decision has fifty percent of margin either side, and the
  // estimate comes off runs of the signal's own preamble. One pass answers
  // nearly always; the other two are there for a transmitter running wide.
  static const int trim[3] = { 0, 6, -6 };
  int64_t best_half = 0;

  for (int i = 0; i < 3; i++)
  {
    int64_t h = half * (100 + trim[i]) / 100;

    if (h < PD_MIN_HALF_SAMPLES * 256)
      continue;

    memset(out, 0, sizeof(*out));
    memset(&g_pd, 0, sizeof(g_pd));
    out->proto = PROTO_PD;

    pd_pass(data, size, offset, mid, hyst, h, &g_pd, out);

    best_half = h;

    if (g_pd.crc_ok > 0)
      break;                          // confirmed: nothing wider can beat it
  }

  if (0 == g_pd.msgs && 0 == g_pd.resets)
    return 0;

  pd_check(&g_pd);

  g_pd.rate = (int)(1000000000ll / (best_half * 2 * period_ns / 256));
  out->rate = g_pd.rate;
  out->idle_high = (0 != scratch->lvl[runs - 1]);
  out->errors = g_pd.sym_err;

  for (int i = 0; i < g_pd.msgs; i++)
  {
    if (!g_pd.msg[i].crc_ok)
      out->errors++;
  }

  // The record opens on the head of a transmission rather than in the middle
  // of one: the first message came back whole and its CRC agreed
  out->burst_start = (g_pd.msgs > 0 && g_pd.msg[0].crc_ok);

  // What may be claimed automatically. One message confirmed by its own CRC32
  // and nothing less: the ordered set is twenty prescribed bits, which is a
  // strong filter but still only a filter, and a Hard Reset carries no
  // checksum at all - so a record holding only resets is reported and marked
  // ambiguous rather than claimed.
  g_pd.sure = (g_pd.crc_ok >= 1);
  out->ambiguous = !g_pd.sure;

  if (0 == out->count)
    return 0;                         // resets only: nothing to put in a panel

  // The header. What a person pointing a probe at CC wants off it, in the
  // order they want it: the rate that proves it is PD, how much traffic, and
  // then the ANSWER - what was actually negotiated - whenever the record held
  // both halves of the question.
  char tail[16];

  tail[0] = 0;

  if (1 == g_pd.req_check)
    snprintf(tail, sizeof(tail), " %d.%dV%d.%dA", g_pd.req_mv / 1000,
        (g_pd.req_mv % 1000) / 100, g_pd.req_ma / 1000,
        (g_pd.req_ma % 1000) / 100);
  else if (0 == g_pd.req_check)
    snprintf(tail, sizeof(tail), " req!");
  else if (g_pd.acked > 0)
    snprintf(tail, sizeof(tail), " %dack", g_pd.acked);

  bool crc_bad = (g_pd.crc_ok < g_pd.msgs);

  snprintf(out->info, sizeof(out->info), "PD %dk %dmsg%s%s%s",
      (g_pd.rate + 500) / 1000, g_pd.msgs, g_pd.resets ? " rst" : "",
      tail, crc_bad ? " crc!" : "");

  return out->count;
}
