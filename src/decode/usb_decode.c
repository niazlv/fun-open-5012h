/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * USB 1.x - low speed (1.5 Mbit) and full speed (12 Mbit) - read off ONE wire
 * of the differential pair.
 *
 * The instrument has one analog input, and USB has two wires. That sounds
 * fatal and is not, because almost everything USB carries is in the
 * TRANSITIONS and not in the levels: the line code is NRZI, where a zero is a
 * transition and a one is the absence of one, so a probe on D+ and a probe on
 * D- read the same bits. What the missing wire costs is the single-ended
 * states - SE0 is both wires low, and on one wire that is just a level. So
 * this decoder cannot see an EOP, a bus reset, a disconnect, a low-speed
 * keep-alive or a high-speed chirp, and it does not pretend to.
 *
 * What it CAN see it sees exactly, because a USB packet is built out of three
 * checks stacked on each other:
 *
 *   SYNC     eight bits, 00000001, NRZI-encoded from idle: K J K J K J K K.
 *            On the wire that is six runs of exactly one bit time and then a
 *            hold, which is a shape data does not often produce and never
 *            produces twice in a row. It gives the packet's start AND pins the
 *            bit phase, so nothing has to be tracked between packets.
 *   PID      eight bits whose high nibble is the complement of the low. One
 *            candidate in sixteen survives that by chance.
 *   CRC      five bits over a token's eleven, sixteen over a data packet's
 *            payload. A packet that checks out is CONFIRMED, not plausible -
 *            which is what lets this decoder sit ahead of the generic ones.
 *
 * The CRC does more than confirm: on a data packet it is what FINDS THE END.
 * A DATA packet's length is not in the packet - the host knows it from the
 * EOP, which is the one thing one wire cannot see. So the reader takes the
 * bits generously, past where the packet must have ended, and asks the CRC16
 * where the payload stops. The first length whose CRC agrees is the answer,
 * and a wrong one agreeing costs a one-in-sixty-five-thousand coincidence.
 *
 * SAMPLE RATE. Low speed is 666 ns to a bit - 83 samples at the top rate, 21
 * at a quarter of it, comfortable anywhere. Full speed is 83.3 ns, which is
 * 10.4 samples at 125 MS/s and nothing at all below it: full speed decodes on
 * the fastest timebases and nowhere else. Ten samples to a bit is enough here
 * and would not be under a fixed bit grid - what makes it work is that the
 * reader re-anchors on each real edge, and bit stuffing guarantees an edge at
 * least once every seven bit times, so the phase can never drift further than
 * that before it is corrected.
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
#define USB_FS_RATE          12000000
#define USB_LS_RATE           1500000

// Four samples to a bit is the floor, and full speed at the instrument's top
// rate is 10.4 of them. Below four the difference between one bit time and two
// is inside the edge quantisation and a run length stops meaning a bit count.
#define USB_MIN_BIT_SAMPLES         4

// SYNC's six one-bit runs. The seventh run - the K it holds through its last
// two bits - is where the packet data starts, two bit times in.
#define USB_SYNC_RUNS               6

// Bit stuffing inserts a zero after six ones, so the encoded stream can hold a
// level for at most seven bit times: the transition that starts the run, then
// six ones. Seven ones in a row therefore cannot happen inside a packet, and
// when the reader sees them it has run off the end of one.
#define USB_MAX_ONES                6

// One more than the result can hold, so a packet that overflows the record is
// still read whole and reported truncated rather than mis-framed
#define USB_MAX_WIRE_BYTES         (LOGIC_MAX_BYTES + 4)

// The bit time is searched a little either side of nominal. USB's own clocks
// are crystal-accurate and need none of it - but a software USB device (V-USB
// and everything descended from it, always low speed) bit-bangs the line out
// of an instruction loop and misses by a percent or two.
#define USB_TRIM_STEPS              2
#define USB_TRIM_PERCENT            2

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_usb   (g_logic_analysis.usb)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const UsbAnalysis *usb_analysis(void)
{
  return &g_usb;
}

//-----------------------------------------------------------------------------
// CRC5, the one over a token's eleven bits (and a SPLIT's nineteen). Poly
// x^5+x^2+1, seeded all ones, computed least significant bit first and
// inverted at the end - the same orientation the bits go out in, so the
// result compares straight against the five that follow on the wire.
//
// Anchor: the SETUP token every enumeration opens with is 2D 00 10 on the
// wire, i.e. address 0, endpoint 0, CRC5 = 0x02. That is what this returns.
static uint8_t usb_crc5(uint32_t v, int bits)
{
  uint8_t crc = 0x1F;

  for (int i = 0; i < bits; i++)
  {
    uint8_t b = (uint8_t)(((v >> i) & 1) ^ (crc & 1));

    crc >>= 1;

    if (b)
      crc ^= 0x14;                  // 0x05 reversed into five bits
  }

  return (uint8_t)((crc ^ 0x1F) & 0x1F);
}

//-----------------------------------------------------------------------------
// The low two bits of a PID name its type. PING and SPLIT are filed under
// "special" by the specification and are shaped like tokens, so what this
// returns is the SHAPE - which is the only thing the reader needs.
static uint8_t usb_pid_kind(uint8_t pid4)
{
  switch (pid4)
  {
    case 0x1: case 0x9: case 0xD: case 0x4:  return USB_P_TOKEN;
    case 0x5:                                return USB_P_SOF;
    case 0x3: case 0x7: case 0xB: case 0xF:  return USB_P_DATA;
    case 0x2: case 0x6: case 0xA: case 0xE:  return USB_P_HANDSHAKE;
    case 0x8:                                return USB_P_SPLIT;
    case 0xC:                                return USB_P_SPECIAL;
    default:                                 return USB_P_NONE;  // 0x0: reserved
  }
}

//-----------------------------------------------------------------------------
static const char *usb_pid_name(uint8_t pid4)
{
  switch (pid4)
  {
    case 0x1: return "OUT";
    case 0x9: return "IN";
    case 0x5: return "SOF";
    case 0xD: return "SETUP";
    case 0x3: return "DATA0";
    case 0xB: return "DATA1";
    case 0x7: return "DATA2";
    case 0xF: return "MDATA";
    case 0x2: return "ACK";
    case 0xA: return "NAK";
    case 0xE: return "STALL";
    case 0x6: return "NYET";
    case 0x4: return "PING";
    case 0x8: return "SPLIT";
    // The same code both ways round: PRE on a full-speed bus in front of a
    // low-speed packet, ERR on a high-speed one. This decoder only ever sees
    // the first of those.
    case 0xC: return "PRE";
    default:  return "RSVD";
  }
}

//-----------------------------------------------------------------------------
// A bit reader that keeps its PHASE rather than stepping a fixed grid: after
// every bit it looks for a real edge where the bit ought to end and, finding
// one within tolerance, takes it.
//
// That is not a refinement here, it is the whole reason full speed decodes at
// all. Ten and a bit samples to a bit means the nominal bit time is never a
// whole number of them, so a running total walks out of the packet within a
// few bytes. On a self-clocking code every edge IS a bit boundary, and bit
// stuffing puts one at least every seven bits - so the phase is corrected
// before it can drift half a bit, whatever the packet is carrying.
//
// The same technique the SWD and Manchester readers use, for the same reason.
typedef struct
{
  const LogicScratch *s;
  int      runs;
  int      size;
  int64_t  bit;
  int64_t  tol;
  int64_t  pos;     // leading edge of the next bit, x256
  int      cur;     // run cursor; only ever moves forward
} UsbRd;

//-----------------------------------------------------------------------------
static void usb_rd_init(UsbRd *rd, const LogicScratch *s, int runs, int size,
    int64_t bit_x256, int64_t start_x256)
{
  rd->s = s;
  rd->runs = runs;
  rd->size = size;
  rd->bit = bit_x256;
  rd->tol = bit_x256 * 3 / 8;
  rd->pos = start_x256;
  rd->cur = 0;
}

//-----------------------------------------------------------------------------
// The level of the next bit, or -1 when the record runs out inside it
static int usb_rd_bit(UsbRd *rd)
{
  int64_t mid = rd->pos + rd->bit / 2;

  if (rd->pos < 0 || mid / 256 >= rd->size)
    return -1;

  while (rd->cur + 1 < rd->runs &&
      (int64_t)rd->s->pos[rd->cur + 1] * 256 <= mid)
    rd->cur++;

  int v = rd->s->lvl[rd->cur];
  int64_t want = rd->pos + rd->bit;
  int k = rd->cur;

  // The first run boundary that could be this bit's end
  while (k + 1 < rd->runs &&
      (int64_t)rd->s->pos[k + 1] * 256 < want - rd->tol)
    k++;

  if (k + 1 < rd->runs)
  {
    int64_t b = (int64_t)rd->s->pos[k + 1] * 256;

    if (b <= want + rd->tol)
      want = b;                     // an edge here: that IS the boundary
  }

  rd->pos = want;

  return v;
}

//-----------------------------------------------------------------------------
// Is run `r` the first K of a SYNC pattern?
//
// SYNC is 00000001 sent least significant bit first and NRZI-encoded, so from
// an idle J the line goes K J K J K J K K: six runs of exactly one bit time,
// and then a K held through the last two. In front of it is the idle - after
// an EOP the line sits at J for its own bit time plus at least two of
// inter-packet delay, so three bit times is the floor and asking for one and a
// half leaves room for the measurement.
//
// The run in front matters for more than plausibility: without it, a SYNC's
// own second run would answer to this test as well as its first, and the
// packet would be read one bit time late.
static bool usb_is_sync(const LogicScratch *s, int runs, int r, int64_t bit)
{
  if (r < 1 || r + USB_SYNC_RUNS >= runs)
    return false;

  // The record's opening run is a fragment of whatever was already going on
  // and says nothing about its length, so it is exempt - and only it.
  if (r > 1 && (int64_t)s->len[r - 1] * 256 < bit * 3 / 2)
    return false;

  for (int k = 0; k < USB_SYNC_RUNS; k++)
  {
    int64_t l = (int64_t)s->len[r + k] * 256;

    if (l < bit / 2 || l > bit * 3 / 2)
      return false;                 // one bit time, and only one
  }

  // ...and then the hold. Two bit times of it belong to SYNC; more means the
  // packet opened with ones, which is a packet and not a longer preamble.
  return ((int64_t)s->len[r + USB_SYNC_RUNS] * 256 >= bit * 3 / 2);
}

//-----------------------------------------------------------------------------
// Where a packet's data starts: two bit times into SYNC's trailing K
static inline int64_t usb_data_start(const LogicScratch *s, int r, int64_t bit)
{
  return (int64_t)s->pos[r + USB_SYNC_RUNS] * 256 + bit * 2;
}

//-----------------------------------------------------------------------------
// The NRZI layer: raw levels in, de-stuffed data bits out.
//
// A one is HELD BACK until a transition proves it was data, and that is the
// whole trick that makes a packet's length come out right on one wire. The
// read has to run past the end of the packet - there is no EOP to see - and
// what it runs into is the idle line, which reads as ones. Six of them arrive
// before the seventh says "this cannot be a packet any more", and six ones
// emitted is most of a byte of pure nothing appended to the payload.
//
// Holding them costs nothing and settles it: the transmitter MUST put a stuff
// zero after six ones, so a run of ones that ends in a transition was data and
// a run that ends in a seventh one was the line at rest. What survives the
// end of a packet this way is at most three bits - the EOP's own edges - which
// can never complete a byte, so the whole-byte count IS the packet's.
//
// The stuff counter starts at ONE: bit stuffing is enabled from the sync
// pattern onward and SYNC's last bit is a one. (No valid PID can actually tell
// the difference - a PID is a nibble and its complement, so the longest run of
// ones it can open with is four, and the counter is reset long before six
// either way. It is written this way because it is what the transmitter does,
// not because a test could catch it.)
typedef struct
{
  UsbRd    rd;
  int      prev;                    // the level the last bit sat at
  int      ones;                    // the stuff counter
  int      held;                    // ones waiting for a transition
  int      qn;                      // bits ready to be handed out
  int      qi;
  uint8_t  qv[USB_MAX_ONES + 1];
  int64_t  qe[USB_MAX_ONES + 1];    // ...and where each of them ended
  bool     ended;                   // seven ones: the packet was over
  bool     cut;                     // the record ran out
} UsbNrz;

//-----------------------------------------------------------------------------
static void usb_nrz_init(UsbNrz *n, const LogicScratch *s, int runs, int size,
    int64_t bit, int r)
{
  memset(n, 0, sizeof(*n));

  usb_rd_init(&n->rd, s, runs, size, bit, usb_data_start(s, r, bit));

  n->prev = s->lvl[r + USB_SYNC_RUNS];
  n->ones = 1;
}

//-----------------------------------------------------------------------------
// The next data bit and the sample position it ended at, or -1
static int usb_nrz_next(UsbNrz *n, int64_t *end)
{
  while (n->qi >= n->qn)
  {
    if (n->ended || n->cut)
      return -1;

    int v = usb_rd_bit(&n->rd);

    if (v < 0)
    {
      n->cut = true;

      return -1;
    }

    int b = (v == n->prev) ? 1 : 0;

    n->prev = v;

    if (b)
    {
      if (n->ones >= USB_MAX_ONES)
      {
        n->ended = true;            // seven, which the encoding forbids

        return -1;
      }

      n->qe[n->held] = n->rd.pos;
      n->held++;
      n->ones++;

      continue;
    }

    // A transition, so everything held was data. And if six were held, this
    // zero is the stuff bit the transmitter had to insert: it is wire, not
    // data, and it goes no further than here.
    bool stuffed = (USB_MAX_ONES == n->ones);

    n->qn = n->held;

    for (int k = 0; k < n->qn; k++)
      n->qv[k] = 1;

    if (!stuffed)
    {
      n->qv[n->qn] = 0;
      n->qe[n->qn] = n->rd.pos;
      n->qn++;
    }

    n->held = 0;
    n->ones = 0;
    n->qi = 0;
  }

  *end = n->qe[n->qi];

  return n->qv[n->qi++];
}

//-----------------------------------------------------------------------------
// Read n bits of a packet's data, least significant bit first. Returns how
// many came back; short means the packet or the record ended inside them.
static int usb_read_bits(const LogicScratch *s, int runs, int size,
    int64_t bit, int r, int n, uint32_t *out)
{
  UsbNrz nrz;
  int got = 0;

  usb_nrz_init(&nrz, s, runs, size, bit, r);

  *out = 0;

  while (got < n)
  {
    int64_t end;
    int b = usb_nrz_next(&nrz, &end);

    if (b < 0)
      break;

    *out |= (uint32_t)b << got;
    got++;
  }

  return got;
}

//-----------------------------------------------------------------------------
// The run index of the next SYNC at or after `from` whose PID check nibble
// agrees, or -1 when there is none.
//
// The PID check is part of finding the packet and not a later verdict on it,
// because six zero bits in a payload draw the same six one-bit runs a SYNC
// does and there is nothing in the WAVEFORM to tell them apart. What tells
// them apart is what comes next: fifteen candidates in sixteen die on the
// complement, and the ones that survive are counted so a record made of
// nothing else can be recognised for what it is.
static int usb_next_sync(const LogicScratch *s, int runs, int size,
    int64_t bit, int from, int64_t min_x256, int *rejected)
{
  for (int r = (from < 1) ? 1 : from; r + USB_SYNC_RUNS < runs; r++)
  {
    if ((int64_t)s->pos[r] * 256 < min_x256)
      continue;

    if (!usb_is_sync(s, runs, r, bit))
      continue;

    uint32_t pid = 0;

    if (8 != usb_read_bits(s, runs, size, bit, r, 8, &pid))
      continue;                     // the record ran out inside the PID

    if (((pid >> 4) & 0x0F) != ((~pid) & 0x0F) || 0 == (pid & 0x0F))
    {
      if (rejected)
        (*rejected)++;

      continue;
    }

    return r;
  }

  return -1;
}

//-----------------------------------------------------------------------------
// A packet's bytes as they came off the wire, de-stuffed and packed least
// significant bit first, with the sample position of every byte boundary -
// which is not p0 plus eight bit times per byte, because a stuff bit takes a
// bit time of wire and carries nothing.
typedef struct
{
  uint8_t  b[USB_MAX_WIRE_BYTES];
  int      pos[USB_MAX_WIRE_BYTES + 1];
  int      n;      // whole bytes
  bool     cut;    // the record ran out inside the packet
  // ...as against ended: seven ones came past, which the encoding forbids
  // inside a packet, so the packet was over. That is the NORMAL stop - it is
  // the EOP and the idle behind it - and it only means something went wrong
  // when it lands before the packet's own shape says it should.
  bool     ended;
} UsbBits;

//-----------------------------------------------------------------------------
// Read a packet, past where it must have ended, and let the framing decide.
// Stops at seven ones - the encoding forbids them, so that is the EOP and the
// idle behind it - or at the end of the record, which is a different thing
// and has to stay distinguishable from it: a packet the WINDOW cut short
// carries no checksum, and calling its last two bytes one would be inventing
// a field out of a record that closed early.
static void usb_read(const LogicScratch *s, int runs, int size, int64_t bit,
    int r, UsbBits *u)
{
  UsbNrz nrz;
  int nb = 0;
  unsigned acc = 0;

  memset(u, 0, sizeof(*u));

  usb_nrz_init(&nrz, s, runs, size, bit, r);

  u->pos[0] = (int)(usb_data_start(s, r, bit) / 256);

  while (u->n < USB_MAX_WIRE_BYTES)
  {
    int64_t end;
    int b = usb_nrz_next(&nrz, &end);

    if (b < 0)
      break;

    acc |= (unsigned)b << nb;

    if (++nb == 8)
    {
      u->b[u->n] = (uint8_t)acc;
      u->n++;
      u->pos[u->n] = (int)(end / 256);
      acc = 0;
      nb = 0;
    }
  }

  u->cut = nrz.cut;
  u->ended = nrz.ended;

  // A packet ends on a byte boundary, and what the reader adds past that end
  // is the EOP's own edges: one bit where the idle level is J, three where it
  // is not, and never more, because the run of ones behind them is held back
  // rather than emitted. So the partial byte here is always partial, the
  // whole-byte count above is the packet's own, and neither depends on which
  // wire of the pair the probe happens to be on.
}

//-----------------------------------------------------------------------------
// How many payload bytes a data packet holds, found by asking the CRC16.
// Returns -1 when no length agrees.
//
// This is where the missing wire is paid for. A data packet's length is not
// in the packet - the receiver knows it from the EOP, which is a single-ended
// state and therefore invisible here - so the only thing that can say where
// the payload stops is the checksum over it. Every candidate length is tried,
// carrying the CRC forward one byte at a time so the scan costs one pass and
// not one per candidate.
//
// The SMALLEST length that agrees, and deliberately: the reader was told to
// overshoot, so on a bus whose inter-packet gap is short it may have run into
// the packet behind this one. The first agreement is this packet's own end;
// a shorter one agreeing by chance is one coincidence in sixty-five thousand.
//
// Poly x^16+x^15+x^2+1, seeded all ones, least significant bit first,
// inverted at the end. Anchor: a zero-length DATA0 is C3 00 00 on the wire,
// so the CRC over no payload at all has to come out 0x0000 - which is what
// the seed and the inversion here give before the loop runs once.
static int usb_data_len(const UsbBits *u, uint16_t *crc_out)
{
  uint16_t crc = 0xFFFF;

  for (int n = 0; 1 + n + 2 <= u->n; n++)
  {
    uint16_t want = (uint16_t)~crc;
    uint16_t rx = (uint16_t)(u->b[1 + n] | (u->b[2 + n] << 8));

    if (want == rx)
    {
      *crc_out = rx;

      return n;
    }

    // ...and fold this byte in for the next length along
    crc ^= u->b[1 + n];

    for (int k = 0; k < 8; k++)
      crc = (0 != (crc & 1)) ? (uint16_t)((crc >> 1) ^ 0xA001)
                             : (uint16_t)(crc >> 1);
  }

  return -1;
}

//-----------------------------------------------------------------------------
// One pass over the record at one bit time. Fills the analysis and the result
// when `out` is given; otherwise it only scores, which is what the search over
// bit times needs.
//
// A CRC-confirmed packet is worth four bare ones in the score, the same ratio
// the SWD decoder uses and for the same reason: a bit time a few percent out
// still finds SYNCs and still passes PID checks - eight bits is not long
// enough for the drift to show - and then reads a payload of rubbish, which
// only the CRC catches.
static int usb_pass(const LogicScratch *s, int runs, int size, int64_t bit,
    UsbAnalysis *a, LogicResult *out)
{
  UsbBits u;
  int packets = 0, crcs = 0, pid_err = 0, stuff_err = 0, sofs = 0;

  if (a)
  {
    memset(a->role, 0, sizeof(a->role));
    memset(a->pidx, 0xFF, sizeof(a->pidx));
  }

  int cur = usb_next_sync(s, runs, size, bit, 1, 0, &pid_err);

  while (cur >= 0 && packets < USB_MAX_PKTS)
  {
    usb_read(s, runs, size, bit, cur, &u);

    UsbPacket p;

    memset(&p, 0, sizeof(p));

    if (u.n < 1)
      break;                        // not even a PID: the record ended here

    p.pid = u.b[0];
    p.kind = usb_pid_kind((uint8_t)(p.pid & 0x0F));
    p.cut = u.cut;

    int nbytes = 1;                 // the PID, and whatever its shape adds

    switch (p.kind)
    {
      case USB_P_TOKEN:
      case USB_P_SOF:
      {
        // A token is three bytes and there is no such thing as a short one.
        // Fewer means either the record ran out - which is a fact about the
        // window and not about the bus - or the bits came apart, which is a
        // fault and is counted as one.
        if (u.n < 3)
        {
          if (!u.cut)
            stuff_err++;

          p.cut = true;

          nbytes = u.n;

          break;
        }

        uint32_t v = (uint32_t)(u.b[1] | (u.b[2] << 8));

        p.crc = (uint16_t)((v >> 11) & 0x1F);
        p.has_crc = true;
        p.crc_ok = (usb_crc5(v & 0x7FF, 11) == (uint8_t)p.crc);

        if (USB_P_SOF == p.kind)
          p.frame = (uint16_t)(v & 0x7FF);
        else
        {
          p.addr = (uint8_t)(v & 0x7F);
          p.ep = (uint8_t)((v >> 7) & 0x0F);
        }

        nbytes = 3;
      }
      break;

      case USB_P_SPLIT:
      {
        if (u.n < 4)
        {
          if (!u.cut)
            stuff_err++;

          p.cut = true;

          nbytes = u.n;

          break;
        }

        uint32_t v = (uint32_t)(u.b[1] | (u.b[2] << 8) | (u.b[3] << 16));

        p.crc = (uint16_t)((v >> 19) & 0x1F);
        p.has_crc = true;
        p.crc_ok = (usb_crc5(v & 0x7FFFF, 19) == (uint8_t)p.crc);
        p.addr = (uint8_t)(v & 0x7F);
        nbytes = 4;
      }
      break;

      case USB_P_DATA:
      {
        uint16_t rx = 0;
        int n = usb_data_len(&u, &rx);

        if (n >= 0)
        {
          p.ndata = (uint8_t)n;
          p.crc = rx;
          p.has_crc = true;
          p.crc_ok = true;
          p.cut = false;            // a CRC that agrees is a packet that ended
          nbytes = 1 + n + 2;
        }
        else if (u.cut)
        {
          // The record ran out inside it. What is here is payload and there is
          // no CRC behind it - reporting the last two bytes as one would be
          // inventing a field.
          p.ndata = (uint8_t)(u.n - 1);
          nbytes = u.n;
        }
        else
        {
          // No length agrees and the packet did end: the bits came apart. The
          // trailing two are where a CRC would be, so that is how they are
          // labelled - and labelled wrong, which is the point of saying so.
          // Bounded by the next packet's SYNC, since an unreadable packet is
          // exactly the case where the reader kept going into the one after.
          int bound = usb_next_sync(s, runs, size, bit, cur + USB_SYNC_RUNS + 1,
              0, NULL);
          int lim = u.n;

          if (bound >= 0)
          {
            while (lim > 1 && u.pos[lim] > s->pos[bound])
              lim--;
          }

          if (lim < 3)
          {
            p.ndata = 0;
            nbytes = 1;
          }
          else
          {
            p.ndata = (uint8_t)(lim - 3);
            p.crc = (uint16_t)(u.b[lim - 2] | (u.b[lim - 1] << 8));
            p.has_crc = true;
            nbytes = lim;
          }
        }
      }
      break;

      case USB_P_HANDSHAKE:
      case USB_P_SPECIAL:
      default:
        nbytes = 1;
        break;
    }

    if (nbytes > u.n)
      nbytes = u.n;

    packets++;

    if (p.crc_ok)
      crcs++;

    if (USB_P_SOF == p.kind)
      sofs++;

    if (out)
    {
      int first = out->count;

      // The idle level, taken off the run in front of the FIRST packet that
      // was actually accepted rather than the first SYNC-shaped thing in the
      // record. SYNC's own first run is a K by construction, so this run is a
      // J - which is what says which wire the probe is on.
      if (1 == packets)
        out->idle_high = (0 != s->lvl[cur - 1]);

      for (int k = 0; k < nbytes; k++)
      {
        if (out->count >= LOGIC_MAX_BYTES)
        {
          out->truncated = true;

          break;
        }

        out->bytes[out->count] = u.b[k];
        out->pos[out->count] = u.pos[k];
        out->end[out->count] = u.pos[k + 1];

        if (a)
        {
          uint8_t role = USB_R_DATA;

          if (0 == k)
            role = USB_R_PID;
          else if (USB_P_SOF == p.kind)
            role = USB_R_FRAME;
          else if (USB_P_TOKEN == p.kind)
            role = USB_R_TOKEN;
          else if (USB_P_SPLIT == p.kind)
            role = USB_R_SPLIT;
          else if (USB_P_DATA == p.kind && p.has_crc && k >= 1 + p.ndata)
            role = USB_R_CRC16;

          a->role[out->count] = role;
          a->pidx[out->count] = (uint8_t)(packets - 1);
        }

        out->count++;
      }

      p.first = (uint8_t)first;
      p.count = (uint8_t)(out->count - first);

      if (a)
        a->pkt[packets - 1] = p;

      if (out->truncated)
        break;
    }

    if (p.cut)
      break;

    // Resume behind this packet, so that six zero bits inside its payload -
    // which draw a SYNC's six one-bit runs exactly - cannot be mistaken for
    // the next one. Where the packet ENDED is known now and was not before.
    cur = usb_next_sync(s, runs, size, bit, cur + USB_SYNC_RUNS + 1,
        (int64_t)u.pos[nbytes] * 256, &pid_err);
  }

  if (a)
  {
    a->packets = packets;
    a->crc_ok = crcs;
    a->pid_err = pid_err;
    a->stuff_err = stuff_err;
    a->sof = sofs;
  }

  return crcs * 4 + packets;
}

//-----------------------------------------------------------------------------
void usb_byte_label(const UsbAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || 0xFF == a->pidx[idx] ||
      a->pidx[idx] >= USB_MAX_PKTS)
    return;

  const UsbPacket *p = &a->pkt[a->pidx[idx]];

  if (USB_R_PID == a->role[idx])
  {
    snprintf(buf, size, "%s%s", usb_pid_name((uint8_t)(p->pid & 0x0F)),
        p->cut ? " cut" : "");

    return;
  }

  // Everything else is a field spread over two or three bytes and is written
  // once, on the byte that completes it - the way the 1-Wire and CAN decoders
  // write a temperature or an identifier across the bytes that make it.
  if (idx != p->first + p->count - 1)
    return;

  if (USB_R_FRAME == a->role[idx])
    snprintf(buf, size, "frame %u%s", (unsigned)p->frame,
        p->crc_ok ? "" : " !");
  else if (USB_R_TOKEN == a->role[idx])
    snprintf(buf, size, "adr%u ep%u%s", (unsigned)p->addr, (unsigned)p->ep,
        p->crc_ok ? "" : " !");
  else if (USB_R_SPLIT == a->role[idx])
    snprintf(buf, size, "hub%u%s", (unsigned)p->addr, p->crc_ok ? "" : " !");
  else if (USB_R_CRC16 == a->role[idx])
    snprintf(buf, size, "crc %04X%s", (unsigned)p->crc, p->crc_ok ? "" : "!");
}

//-----------------------------------------------------------------------------
void usb_field_label(const UsbAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || 0xFF == a->pidx[idx] ||
      a->pidx[idx] >= USB_MAX_PKTS)
    return;

  const UsbPacket *p = &a->pkt[a->pidx[idx]];

  switch (a->role[idx])
  {
    case USB_R_PID:   snprintf(buf, size, "PID"); break;
    case USB_R_FRAME: snprintf(buf, size, "FR%d", idx - p->first - 1); break;
    case USB_R_TOKEN: snprintf(buf, size, "TK%d", idx - p->first - 1); break;
    case USB_R_SPLIT: snprintf(buf, size, "SP%d", idx - p->first - 1); break;
    case USB_R_CRC16: snprintf(buf, size, "C%d",
                          idx - p->first - 1 - p->ndata); break;
    case USB_R_DATA:  snprintf(buf, size, "D%d", idx - p->first - 1); break;
    default: break;
  }
}

//-----------------------------------------------------------------------------
// A token's two bytes are one eleven-bit field and a checksum, and a CRC16 is
// one sixteen-bit number: both are shown as one. A payload byte is a byte and
// means a byte, which is what the panel falls back to showing as a character.
void usb_group_at(const UsbAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  uint8_t role = a->role[idx];

  if (USB_R_TOKEN != role && USB_R_FRAME != role && USB_R_SPLIT != role &&
      USB_R_CRC16 != role)
    return;

  int s = idx;
  int e = idx;

  while (s > 0 && a->role[s - 1] == role && a->pidx[s - 1] == a->pidx[idx])
    s--;

  while (e + 1 < LOGIC_MAX_BYTES && a->role[e + 1] == role &&
      a->pidx[e + 1] == a->pidx[idx])
    e++;

  *start = s;
  *len = e - s + 1;
}

//-----------------------------------------------------------------------------
int usb_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  memset(&g_usb, 0, sizeof(g_usb));
  out->proto = PROTO_USB;

  runs = logic_runs(data, size, offset, scratch, &mid);

  // A SYNC alone is eight runs with the idle in front of it, and there is no
  // packet without one
  if (runs < 9 || period_ns <= 0)
    return 0;

  static const int rates[2] = { USB_FS_RATE, USB_LS_RATE };

  int64_t best_bit = 0;
  int best_score = 0;
  int best_rate = 0;

  for (int i = 0; i < 2; i++)
  {
    // Samples per bit at this speed, x256. Nothing is estimated off the
    // record: USB runs at these two rates and at no others, so a record that
    // fits neither is not USB and a search over rates would only find ways to
    // agree with something else.
    int64_t nominal = (256000000000ll + (int64_t)rates[i] * period_ns / 2) /
        ((int64_t)rates[i] * period_ns);

    if (nominal < USB_MIN_BIT_SAMPLES * 256)
      continue;                     // the record is too coarse for this speed

    for (int k = -USB_TRIM_STEPS; k <= USB_TRIM_STEPS; k++)
    {
      int64_t bit = nominal * (100 + k * USB_TRIM_PERCENT) / 100;

      if (bit < USB_MIN_BIT_SAMPLES * 256)
        continue;

      int score = usb_pass(scratch, runs, size, bit, NULL, NULL);

      if (score > best_score)
      {
        best_score = score;
        best_bit = bit;
        best_rate = rates[i];
      }
    }
  }

  if (0 == best_score || 0 == best_bit)
    return 0;

  usb_pass(scratch, runs, size, best_bit, &g_usb, out);

  if (0 == out->count)
    return 0;

  g_usb.rate = best_rate;
  g_usb.low_speed = (USB_LS_RATE == best_rate);

  // Which wire the probe is on. usb_pass left the idle level in the result:
  // SYNC's first run is a K by construction, so the run in front of it is a J,
  // and J is D+ high at full speed and D+ low at low speed. That is the whole
  // of the difference between the two, and the only thing one wire can say
  // about which wire it is.
  g_usb.dplus = (out->idle_high != g_usb.low_speed);

  out->rate = best_rate;
  out->errors = g_usb.stuff_err;

  for (int i = 0; i < g_usb.packets && i < USB_MAX_PKTS; i++)
  {
    if (g_usb.pkt[i].has_crc && !g_usb.pkt[i].crc_ok)
      out->errors++;
  }

  // The record opens on the head of a transaction: a token or a start of
  // frame, with the line at rest long enough in front of it to have been
  // between transactions rather than inside one. Eight bit times is the
  // dividing line - the gap WITHIN a transaction is a handful of them.
  int first = -1;

  for (int r = 1; r + USB_SYNC_RUNS < runs && scratch->pos[r] < out->pos[0];
      r++)
  {
    if (usb_is_sync(scratch, runs, r, best_bit))
      first = r;
  }

  out->burst_start = (first > 1 &&
      (USB_P_TOKEN == g_usb.pkt[0].kind || USB_P_SOF == g_usb.pkt[0].kind) &&
      (int64_t)scratch->len[first - 1] * 256 >= best_bit * 8);

  // What may be claimed automatically. One packet confirmed by its own CRC is
  // enough and nothing less will do: a SYNC is a shape six zero bits in a
  // payload also draw, and a PID check turns down only fifteen candidates in
  // sixteen. Together they are a filter and not a proof. The CRC is the proof,
  // and every packet that carries a length also carries one - the only shapes
  // without are the handshakes, and a bus does not send those on their own.
  g_usb.sure = (g_usb.crc_ok >= 1);
  out->ambiguous = !g_usb.sure;

  bool crc_bad = (out->errors > g_usb.stuff_err);

  snprintf(out->info, sizeof(out->info), "USB %s %s %dpkt%s%s",
      g_usb.low_speed ? "LS" : "FS", g_usb.dplus ? "D+" : "D-",
      g_usb.packets, g_usb.sof ? " sof" : "",
      (crc_bad || g_usb.stuff_err) ? " crc!" : "");

  return out->count;
}
