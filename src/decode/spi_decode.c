/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SPI on one probe - which is to say, NOT SPI.
 *
 * SPI is a clocked protocol. The data line carries no timing of its own: a
 * bit is whatever MOSI holds at the clock edge, and with the clock on another
 * wire this instrument cannot see any of that. There is no reading of these
 * samples that recovers it, and any decoder claiming otherwise is guessing
 * and not saying so.
 *
 * What CAN be done is reconstruct the bit stream, on assumptions that have to
 * be stated because they are what the answer rests on:
 *
 *   1. the master clocks at a CONSTANT rate. A hardware SPI peripheral does;
 *      a bit-banged master between bytes often does not;
 *   2. the data line only changes on bit boundaries, so every run of one
 *      level is a whole number of bit times;
 *   3. the byte boundary is somewhere in eight, and the bit order is one of
 *      two - sixteen readings, of which fifteen are wrong.
 *
 * With those, a run of length d is round(d / T) bits and the record becomes a
 * bit stream. What it does NOT become is unambiguous: forty microseconds of a
 * high line is eight bits of 0xFF at one rate and four at half of it, and
 * nothing in the record says which. That is why this decoder is never chosen
 * automatically - it reports itself ambiguous always, so the cascade passes
 * over it and only a user who picked it by name ever sees it.
 *
 * The way to use it is two passes with one probe, which is what makes the
 * whole thing worth having:
 *
 *   - put the probe on SCK. The scope measures its frequency, and Decoder >
 *     "SPI clock = measured" writes that number down;
 *   - move the probe to MOSI. T is now known to the accuracy of a frequency
 *     measurement rather than guessed from the data, and the quantisation
 *     stops drifting over long runs of identical bits.
 *
 * The byte boundary is found rather than assumed where the record allows it.
 * A master that pauses between bytes - and a bit-banged one nearly always
 * does - leaves a gap, and a gap is a byte boundary for free. Where there are
 * none, all sixteen readings are scored on what SPI traffic actually looks
 * like: 0x00 and 0xFF filler, printable text, values that repeat. The header
 * says which of the two happened, because "the gaps pinned it" and "this
 * scored best" are not the same claim.
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
#define SPI_MAX_BITS      (LOGIC_MAX_BYTES * 8)
#define SPI_MIN_SAMPLES    3    // below this a bit is not resolvable at all
#define SPI_MIN_BITS      16    // two bytes: less is not a reading of anything

// How long a run has to be before it is considered a pause rather than data.
// Twelve bits is longer than one byte can hold at one level - but not longer
// than TWO can, and 0x00 0x00 is twelve zeros in the middle of an address.
// So the length is only the first of three conditions; see below.
#define SPI_GAP_BITS      12
#define SPI_MAX_ANCHORS   16

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_spi   (g_logic_analysis.spi)

// The 25-series NOR flash command set: what is on the other end of a MOSI
// line more often than everything else put together, and the only family
// whose opcodes are worth matching against.
//
// SD cards in SPI mode are deliberately NOT here. Their commands are 0x40|n,
// which is 0x40..0x7F - the printable ASCII range. A table covering those
// would name half of any text payload a command, and the scoring would then
// prefer whichever cut produced the most of them. A recogniser that finds
// something in everything has told you nothing.
static const struct { uint8_t op; const char *name; bool addr; } g_spi_cmd[] =
{
  { 0x01, "WRSR",       false },
  { 0x02, "PAGE PROG",  true  },
  { 0x03, "READ",       true  },
  { 0x04, "WRDI",       false },
  { 0x05, "RDSR",       false },
  { 0x06, "WREN",       false },
  { 0x0B, "FAST READ",  true  },
  { 0x20, "ERASE 4K",   true  },
  { 0x35, "RDSR2",      false },
  { 0x4B, "UNIQ ID",    false },
  { 0x52, "ERASE 32K",  true  },
  { 0x5A, "SFDP",       true  },
  { 0x60, "CHIP ERASE", false },
  { 0x66, "RST EN",     false },
  { 0x90, "MFR/DEV",    true  },
  { 0x99, "RESET",      false },
  { 0x9F, "JEDEC ID",   false },
  { 0xAB, "REL PWRDN",  false },
  { 0xB9, "PWRDN",      false },
  { 0xC7, "CHIP ERASE", false },
  { 0xD8, "ERASE 64K",  true  },
};

// 0 = work the rate out from the record. Otherwise the clock the user
// measured on SCK before moving the probe, which is the whole point.
static int g_spi_clock_hz = 0;

// 0 = score both orders, 1 = MSB first, 2 = LSB first
static int g_spi_order = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void spi_decode_set_clock(int hz)
{
  g_spi_clock_hz = (hz > 0) ? hz : 0;
}

//-----------------------------------------------------------------------------
void spi_decode_set_order(int order)
{
  g_spi_order = (order >= 0 && order <= 2) ? order : 0;
}

//-----------------------------------------------------------------------------
const SpiAnalysis *spi_analysis(void)
{
  return &g_spi;
}

//-----------------------------------------------------------------------------
static const char *spi_cmd_name(uint8_t op, bool *takes_addr)
{
  for (unsigned i = 0; i < sizeof(g_spi_cmd) / sizeof(g_spi_cmd[0]); i++)
  {
    if (g_spi_cmd[i].op == op)
    {
      if (takes_addr)
        *takes_addr = g_spi_cmd[i].addr;

      return g_spi_cmd[i].name;
    }
  }

  return NULL;
}

//-----------------------------------------------------------------------------
// What this byte turned out to be. Empty for a byte that is just a byte -
// which is most of them, and the caller then shows it as a character the way
// it does for a serial line.
void spi_byte_label(const SpiAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (SPI_R_CMD == a->role[idx])
  {
    snprintf(buf, size, "%s", spi_cmd_name(v, NULL));

    return;
  }

  // The address goes on the byte that COMPLETES it, written once across all
  // three - the rule every other decoder here follows, and the reason the
  // three of them are one group
  if (SPI_R_ADDR != a->role[idx])
    return;

  for (int i = 0; i < a->txs && i < SPI_MAX_TX; i++)
  {
    if (a->tx_last[i] == idx)
      snprintf(buf, size, "A=%06lX", (unsigned long)a->tx_addr[i]);
  }
}

//-----------------------------------------------------------------------------
void spi_field_label(const SpiAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (SPI_R_CMD == a->role[idx])
  {
    snprintf(buf, size, "CMD");

    return;
  }

  if (SPI_R_ADDR == a->role[idx])
  {
    // Which of the three, counted from the command in front of them
    int k = 0;

    while (k < 3 && idx - k - 1 >= 0 && SPI_R_ADDR == a->role[idx - k - 1])
      k++;

    snprintf(buf, size, "A%d", 2 - k);
  }
}

//-----------------------------------------------------------------------------
// Three address bytes are one number and are shown as one. Everything else
// stands alone: a byte of payload is a byte of payload.
void spi_group_at(const SpiAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES || SPI_R_ADDR != a->role[idx])
    return;

  int s = idx;

  while (s > 0 && SPI_R_ADDR == a->role[s - 1])
    s--;

  *start = s;
  *len = 3;
}

//-----------------------------------------------------------------------------
static inline int spi_bit(const uint8_t *bits, int i)
{
  return (bits[i / 8] >> (i % 8)) & 1;
}

//-----------------------------------------------------------------------------
// Byte number `j` under one reading of the stream: the byte boundary sits at
// `phase` bits in, and the bits inside it run one way or the other.
static uint8_t spi_byte(const uint8_t *bits, int nbits, int phase, bool msb,
    int j)
{
  int base = phase + j * 8;
  uint8_t v = 0;

  for (int k = 0; k < 8; k++)
  {
    int b = (base + k < nbits) ? spi_bit(bits, base + k) : 0;

    if (msb)
      v = (uint8_t)((v << 1) | b);        // first bit off the wire is bit 7
    else
      v = (uint8_t)(v | (b << k));
  }

  return v;
}

//-----------------------------------------------------------------------------
// Pack the stream into bytes, with the byte grid RESTARTING at every pause.
//
// One phase for the whole record is wrong, and wrong on the commonest case
// there is: two transactions with a pause between them, and a pause is not a
// whole number of byte times. The second transaction then comes out shifted
// by however much of a byte the pause happened to be.
//
// Restarting at each pause costs nothing when the pause was not one. A run
// that is really data sits ON the byte grid already - it is data, it was
// clocked - so restarting there changes nothing at all; and where the
// quantisation has drifted a bit over a long record, restarting puts it back.
// It only ever moves the grid where the grid was going to be wrong.
//
// bit_of[] receives each byte's first bit index, which is what turns a byte
// back into a place on the screen.
static int spi_pack(const uint8_t *bits, int nbits, int phase, bool msb,
    const int *anchor, int anchors, uint8_t *bytes, int *bit_of, int max)
{
  int origin = phase;
  int n = 0, ai = 0;

  while (origin + 8 <= nbits && n < max)
  {
    while (ai < anchors && anchor[ai] <= origin)
      ai++;

    if (ai < anchors && anchor[ai] < origin + 8)
    {
      origin = anchor[ai];   // a pause cut this byte short: start afresh
      continue;
    }

    if (bit_of)
      bit_of[n] = origin;

    bytes[n++] = spi_byte(bits, nbits, origin, msb, 0);
    origin += 8;
  }

  return n;
}

//-----------------------------------------------------------------------------
// How much one reading of the stream looks like SPI traffic rather than like
// the same bits cut in the wrong place.
//
// The heuristics are what the bus actually carries. Filler is the strongest:
// 0x00 and 0xFF are most of what goes down a flash or a display link, and -
// this is the point - a byte boundary in the wrong place turns the EDGE of a
// filler run into neither, so the count of them peaks at the right phase.
// Text scores next, and a value repeating its neighbour last.
//
// A run of nothing but 0xFF scores the same at every phase, because it is the
// same bits whichever way you cut it. That is not a flaw in the scoring; it
// is the signal genuinely not saying, and it is why the gaps are worth more
// than all of this put together.
static int spi_score(const uint8_t *bits, int nbits, int phase, bool msb,
    const int *anchor, int anchors)
{
  uint8_t bytes[LOGIC_MAX_BYTES];
  int bit_of[LOGIC_MAX_BYTES];
  int n = spi_pack(bits, nbits, phase, msb, anchor, anchors, bytes, bit_of,
      LOGIC_MAX_BYTES);
  int score = 0;

  for (int j = 0; j < n; j++)
  {
    uint8_t v = bytes[j];

    if (0x00 == v || 0xFF == v)
      score += 4;
    else if (v >= 0x20 && v < 0x7F)
      score += 2;

    if (j > 0 && v == bytes[j - 1])
      score += 1;
  }

  // ...and the one thing here that is not a heuristic. Where a pause said a
  // transaction began, the byte sitting there can be checked against the
  // opcodes a flash actually answers to. This is the ONLY place a command
  // table is allowed to influence anything: there are two or three such
  // bytes in a record, so matching one is evidence. Let it score every byte
  // position instead and one of twenty opcodes turns up somewhere by chance
  // in every one of the sixteen readings - a recogniser that finds something
  // in everything has told you nothing.
  //
  // What it settles in practice is the BIT ORDER. The pauses fix where bytes
  // begin and say nothing whatever about which way round the bits inside
  // them go, and that is exactly the half an opcode answers.
  for (int j = 0; j < n; j++)
  {
    for (int i = 0; i < anchors; i++)
    {
      if (bit_of[j] == anchor[i] && spi_cmd_name(bytes[j], NULL))
        score += 12;
    }
  }

  return score;
}

//-----------------------------------------------------------------------------
// The bit time the record itself implies, in x256 samples: the shortest run
// is one bit or a small multiple, refined by averaging the runs over the whole
// number of bits each of them holds. Same idea as the UART's auto-baud, and
// the same weakness - a stream with no single-bit run in it anywhere reports
// a multiple of the truth, and there is nothing in the data that says so.
static int64_t spi_estimate_bit(const LogicScratch *s, int runs)
{
  int tmin = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (0 == tmin || s->len[r] < tmin)
      tmin = s->len[r];
  }

  if (tmin < SPI_MIN_SAMPLES)
    return 0;

  int64_t sum = 0;
  int count = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    int n = (s->len[r] + tmin / 2) / tmin;

    if (n < 1 || n > 32)
      continue;

    sum += s->len[r];
    count += n;
  }

  if (count < 4)
    return 0;

  return sum * 256 / count;
}

//-----------------------------------------------------------------------------
int spi_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  uint8_t bits[SPI_MAX_BITS / 8];
  int anchor[SPI_MAX_ANCHORS];
  int anchors = 0;
  int nbits = 0;
  int mid, runs;

  (void)data;
  (void)mid;

  memset(out, 0, sizeof(*out));
  memset(bits, 0, sizeof(bits));
  memset(&g_spi, 0, sizeof(g_spi));
  out->proto = PROTO_SPI;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 3 || period_ns <= 0)
    return 0;

  int64_t bit_x256;

  if (g_spi_clock_hz > 0)
  {
    bit_x256 = (256000000000ll + (int64_t)g_spi_clock_hz * period_ns / 2) /
        ((int64_t)g_spi_clock_hz * period_ns);
    g_spi.told = true;
  }
  else
  {
    bit_x256 = spi_estimate_bit(scratch, runs);
    g_spi.told = false;
  }

  if (bit_x256 < SPI_MIN_SAMPLES * 256)
    return 0;

  // A told clock has to be a clock this record could have been sent at. If
  // most of the runs in it are shorter than half a bit, then whatever is on
  // the probe is not running at that rate - the number was written down off
  // a different signal, or the probe moved to a different one. Quantising
  // every one of those runs up to a bit would produce a bit stream out of
  // nothing at all, which is exactly the failure this decoder must not have.
  {
    int short_runs = 0, total = 0;

    for (int r = 1; r + 1 < runs; r++)
    {
      total++;

      if ((int64_t)scratch->len[r] * 512 < bit_x256)
        short_runs++;
    }

    if (total > 0 && short_runs * 2 > total)
      return 0;
  }

  // Whatever the line was left at when the record ran out is its resting
  // level: traffic stops, the master lets go, and the tail run is what is
  // left. That is the only thing in a recording of a data line that says
  // which level means "nothing is being sent".
  int idle = scratch->lvl[runs - 1];

  out->idle_high = (0 != idle);

  // Quantise the interior runs. The first and the last are cut by the ends of
  // the record - each is some fraction of the run it would have been - so the
  // reconstruction covers the first edge to the last and nothing outside it.
  for (int r = 1; r + 1 < runs && nbits < SPI_MAX_BITS; r++)
  {
    int n = (int)(((int64_t)scratch->len[r] * 256 + bit_x256 / 2) / bit_x256);
    int level = scratch->lvl[r];

    if (n < 1)
      n = 1;      // a run shorter than a bit still happened; it is one bit

    // A pause is the line AT REST, and the record's own tail says which level
    // that is: whatever it was left at when the traffic stopped. A long run
    // at the other level is data - twelve zeros is 0x00 0x00 going past.
    //
    // Being wrong about one of these is cheap, which is why the test can
    // afford to be simple: restarting the byte grid at a run that was really
    // data changes nothing, because data sits on the grid already.
    if (level == idle && n >= SPI_GAP_BITS && anchors < SPI_MAX_ANCHORS &&
        nbits > 0)
      anchor[anchors++] = nbits + n;

    for (int k = 0; k < n && nbits < SPI_MAX_BITS; k++, nbits++)
    {
      if (level)
        bits[nbits / 8] |= (uint8_t)(1 << (nbits % 8));
    }
  }

  if (nbits < SPI_MIN_BITS)
    return 0;

  // Sixteen readings, of which fifteen are wrong. With pauses in the record
  // only the HEAD of it is really in question - everything after the first
  // pause has its byte boundary from the pause itself - but the bit order is
  // open either way, and that is half of what is being searched here.
  int best = -1, best_phase = 0;
  bool best_msb = true;

  for (int o = 0; o < 2; o++)
  {
    bool msb = (0 == o);

    if ((1 == g_spi_order && !msb) || (2 == g_spi_order && msb))
      continue;   // the user said which way round it goes

    for (int p = 0; p < 8; p++)
    {
      int sc = spi_score(bits, nbits, p, msb, anchor, anchors);

      if (sc > best)
      {
        best = sc;
        best_phase = p;
        best_msb = msb;
      }
    }
  }

  if (best < 0)
    return 0;

  // Pack it for real, and remember where every byte began
  int bit_of[LOGIC_MAX_BYTES];
  int nbytes = spi_pack(bits, nbits, best_phase, best_msb, anchor, anchors,
      out->bytes, bit_of, LOGIC_MAX_BYTES);

  if (0 == nbytes)
    return 0;

  out->count = nbytes;
  out->truncated = (LOGIC_MAX_BYTES == nbytes);

  // ...and turn those bit indices back into places on the trace, by walking
  // the runs a second time. A bit's place on the screen is a fact about the
  // record and is not going to be inferred from the bit array.
  {
    int base = 0;   // bit index at the start of the run being looked at
    int j = 0;

    for (int r = 1; r + 1 < runs && j < nbytes; r++)
    {
      int n = (int)(((int64_t)scratch->len[r] * 256 + bit_x256 / 2) / bit_x256);

      if (n < 1)
        n = 1;

      while (j < nbytes && bit_of[j] >= base && bit_of[j] < base + n)
      {
        out->pos[j] = scratch->pos[r] +
            (int)((int64_t)(bit_of[j] - base) * bit_x256 / 256);
        out->end[j] = out->pos[j] + (int)(bit_x256 * 8 / 256);
        j++;
      }

      base += n;
    }

    // Anything the walk did not reach is off the end of the reconstruction
    out->count = j;
  }

  if (0 == out->count)
    return 0;

  // Where a pause was found, every byte after it begins where the pause said
  // it did - that is not a guess. The head of the record still is one, and
  // the header keeps the two apart.
  bool pinned = (anchors > 0);

  // What the bytes turned out to be - and ONLY where a pause said a
  // transaction started there. Naming the first byte of a scored phase a
  // command would be a guess wearing a fact's clothes: the phase itself is
  // already a preference, and hanging "READ" off it would make a reading
  // look confirmed by the very thing it was chosen for.
  if (pinned)
  {
    for (int i = 0; i < anchors && g_spi.txs < SPI_MAX_TX; i++)
    {
      int j = -1;
      bool takes_addr = false;

      for (int k = 0; k < out->count; k++)
      {
        if (bit_of[k] == anchor[i])
          j = k;
      }

      if (j < 0 || !spi_cmd_name(out->bytes[j], &takes_addr))
        continue;

      g_spi.role[j] = SPI_R_CMD;
      g_spi.cmds++;

      if (!takes_addr || j + 3 >= out->count)
        continue;

      // Three bytes of address, most significant first, and they are one
      // number - so they are one thing on the screen
      g_spi.role[j + 1] = SPI_R_ADDR;
      g_spi.role[j + 2] = SPI_R_ADDR;
      g_spi.role[j + 3] = SPI_R_ADDR;

      g_spi.tx_last[g_spi.txs] = (uint8_t)(j + 3);
      g_spi.tx_addr[g_spi.txs] = ((uint32_t)out->bytes[j + 1] << 16) |
          ((uint32_t)out->bytes[j + 2] << 8) | out->bytes[j + 3];
      g_spi.txs++;
    }
  }

  g_spi.clock_hz = (int)(256000000000ll / (bit_x256 * period_ns));
  g_spi.msb_first = best_msb;
  g_spi.phase = best_phase;
  g_spi.bits = nbits;
  g_spi.anchors = anchors;
  g_spi.pinned = pinned;

  out->rate = g_spi.clock_hz;
  out->burst_start = pinned;

  // ALWAYS. There is no reading of a data line without its clock that tells
  // this protocol from any other digital signal - every one of them quantises
  // into bits just as happily - so the cascade must pass over this decoder
  // and only a user who asked for it by name ever gets it.
  out->ambiguous = true;

  char rate[12];
  int hz = g_spi.clock_hz;

  if (hz >= 1000000)
    snprintf(rate, sizeof(rate), "%d.%dM", hz / 1000000, (hz % 1000000) / 100000);
  else if (hz >= 1000)
    snprintf(rate, sizeof(rate), "%d.%dk", hz / 1000, (hz % 1000) / 100);
  else
    snprintf(rate, sizeof(rate), "%d", hz);

  // The doubt goes in the NAME, and it is there every single time. Without
  // it "SPI 20.0k MSB p2" reads exactly like the CAN and LIN headers beside
  // it - and those are decodes of buses this instrument can actually see,
  // while this is a reconstruction of one it cannot. The panel has to say so
  // where the eye lands first, not in a manual.
  //
  // A '!' rather than a word, because the panel appends the byte range to
  // this line and there is no room for a word - and because '!' already
  // means "read this with suspicion" everywhere else here: CRC!, PID!,
  // ID=0B!. One character, and it is the vocabulary the rest of the decoder
  // already speaks.
  //
  // The rest is how much of it was measured rather than assumed: '~' marks a
  // rate guessed off the data instead of read on SCK, 'g' says the pauses
  // fixed where bytes begin, '?' says there were none and the reading was
  // scored.
  snprintf(out->info, sizeof(out->info), "SPI! %s%s %s p%d %s",
      g_spi.told ? "" : "~", rate, best_msb ? "MSB" : "LSB", best_phase,
      pinned ? "g" : "?");

  return out->count;
}
