/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * MIDI 1.0 - the five-pin current loop that synthesisers, keyboards, drum
 * machines and every DAW interface have spoken since 1983.
 *
 * On the wire it is an ordinary 8N1 serial line, so the framing is the UART's.
 * What makes it MIDI is everything the UART cannot see:
 *
 *   - the rate is 31250 baud and nothing else. It is not a divisor anyone
 *     picks for a console link, and it is not in the standard baud table, so
 *     a record that frames cleanly at 31250 is already unusual;
 *
 *   - the bytes obey a grammar. Bit 7 says status or data: a status byte
 *     names a message and how many data bytes come after it, and a data byte
 *     never sets it. A stream where the two agree over several messages is
 *     not something a link carrying text or binary falls into by accident.
 *
 * Those two together are the identification. Neither alone is enough - ASCII
 * at 31250 baud is all data bytes and no grammar, and the grammar with no
 * rate behind it is a coincidence looking for a decoder - so this one asks
 * for both and marks the record ambiguous when the evidence is thin, which
 * keeps the cascade off records that only look the part.
 *
 * Three things about the byte stream are easy to get wrong and are what the
 * parser is mostly about:
 *
 *   - RUNNING STATUS. A status byte stays in force, so a run of notes goes
 *     out as one 0x90 and pairs of data bytes after it. The data bytes are
 *     the whole message and there is no status byte on the wire to point at.
 *
 *   - REAL-TIME BYTES. 0xF8..0xFF are one byte each and may arrive ANYWHERE,
 *     including between the data bytes of another message, without cancelling
 *     it or disturbing running status. A clock does not wait for a note.
 *
 *   - SYSTEM EXCLUSIVE. 0xF0 opens a message of any length that runs until
 *     0xF7 closes it, and it DOES cancel running status. System common
 *     cancels it too; real-time does not.
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
#define MIDI_BAUD            31250
#define MIDI_FRAME_BITS      10      // start + 8 data + stop
#define MIDI_IDLE_GAP_BITS   (MIDI_FRAME_BITS + 1)

// Below this the bit is not sampled well enough for the stop bit to be where
// the arithmetic says it is
#define MIDI_MIN_BIT_SAMPLES  3

// A run is a whole number of bit times, so the shortest one in the record is
// one bit or more. Two samples of slack: at the fastest timebase that still
// resolves 31250 baud a bit is only a few samples wide, and the edges are
// placed to within one of them at each end.
#define MIDI_MIN_RUN_SLACK    2

// A record this short says nothing on its own. Four bytes of MIDI clock is a
// clock; two bytes is two bytes.
#define MIDI_AUTO_MIN_BYTES   4

#define MIDI_VAR  0xFF   // MidiMsg.want for a system-exclusive message

/*- Variables ---------------------------------------------------------------*/
static MidiAnalysis g_midi;

// Sharps, because that is how a note number is written everywhere a note
// number is written. Note 60 is C4: 69 is A440 and 69 is A4.
static const char *const g_midi_note[12] =
{
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B",
};

// Controllers 120..127 are not controllers at all - they are the channel
// mode messages, and "CC123=0" is a poor way of saying "all notes off"
static const char *const g_midi_mode[8] =
{
  "SndOff", "RstCtl", "Local", "AllOff", "OmniOff", "OmniOn", "Mono", "Poly",
};

// The manufacturer IDs anyone is likely to have on the bench. A SysEx from
// one that is not here shows its ID in hex, which is what the byte says.
static const struct { uint8_t id; const char *name; } g_midi_mfr[] =
{
  { 0x01, "Seq" },    { 0x04, "Moog" },   { 0x07, "Kurzw" },
  { 0x18, "Emu" },    { 0x3E, "Waldrf" }, { 0x40, "Kawai" },
  { 0x41, "Roland" }, { 0x42, "Korg" },   { 0x43, "Yamaha" },
  { 0x44, "Casio" },  { 0x47, "Akai" },   { 0x52, "Zoom" },
  { 0x7D, "Test" },   { 0x7E, "Univ" },   { 0x7F, "UnivRT" },
};

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
// How many data bytes follow this status byte. -1 for a byte that is not a
// status byte at all, and for the four codes the standard leaves undefined
// (0xF4, 0xF5, 0xF9, 0xFD) - a transmitter does not send those, so a record
// holding one is being read wrongly. -2 asks the reader to keep going until
// 0xF7 arrives.
static int midi_data_len(uint8_t st)
{
  if (st < 0x80)
    return -1;

  if (st < 0xF0)
  {
    switch (st & 0xF0)
    {
      case 0xC0: return 1;   // program change
      case 0xD0: return 1;   // channel pressure
      default:   return 2;   // note off/on, poly pressure, control, bend
    }
  }

  switch (st)
  {
    case 0xF0: return -2;    // system exclusive: until 0xF7
    case 0xF1: return 1;     // MTC quarter frame
    case 0xF2: return 2;     // song position pointer
    case 0xF3: return 1;     // song select
    case 0xF6: return 0;     // tune request
    case 0xF7: return 0;     // end of exclusive
    case 0xF8: return 0;     // timing clock
    case 0xFA: return 0;     // start
    case 0xFB: return 0;     // continue
    case 0xFC: return 0;     // stop
    case 0xFE: return 0;     // active sensing
    case 0xFF: return 0;     // system reset
    default:   return -1;    // F4, F5, F9, FD
  }
}

//-----------------------------------------------------------------------------
// What the message is called, in full. Used where there is room for it.
static const char *midi_type_name(uint8_t st)
{
  if (st < 0x80)
    return "data";

  if (st < 0xF0)
  {
    switch (st & 0xF0)
    {
      case 0x80: return "NoteOff";
      case 0x90: return "NoteOn";
      case 0xA0: return "PolyAT";
      case 0xB0: return "CC";
      case 0xC0: return "Prog";
      case 0xD0: return "ChanAT";
      default:   return "Bend";
    }
  }

  switch (st)
  {
    case 0xF0: return "SysEx";
    case 0xF1: return "QFrame";
    case 0xF2: return "SPP";
    case 0xF3: return "Song";
    case 0xF6: return "Tune";
    case 0xF7: return "EOX";
    case 0xF8: return "Clock";
    case 0xFA: return "Start";
    case 0xFB: return "Cont";
    case 0xFC: return "Stop";
    case 0xFE: return "Sense";
    case 0xFF: return "Reset";
    default:   return "undef";
  }
}

//-----------------------------------------------------------------------------
// ...and what it is called when the channel number has to fit beside it, on
// the row that names one byte at a time
static const char *midi_short_name(uint8_t st)
{
  switch (st & 0xF0)
  {
    case 0x80: return "Off";
    case 0x90: return "On";
    case 0xA0: return "PAT";
    case 0xB0: return "CC";
    case 0xC0: return "Prog";
    case 0xD0: return "CAT";
    default:   return "Bend";
  }
}

//-----------------------------------------------------------------------------
// Which field of the message data byte number `di` is
static const char *midi_data_name(uint8_t st, int di)
{
  if (st < 0x80)
    return "DATA";

  if (st < 0xF0)
  {
    switch (st & 0xF0)
    {
      case 0x80:
      case 0x90: return di ? "VEL" : "NOTE";
      case 0xA0: return di ? "PRES" : "NOTE";
      case 0xB0: return di ? "VAL" : "CTRL";
      case 0xC0: return "PGM";
      case 0xD0: return "PRES";
      default:   return di ? "MSB" : "LSB";   // pitch bend, 7 bits each
    }
  }

  switch (st)
  {
    case 0xF1: return "QF";
    case 0xF2: return di ? "POS-H" : "POS-L";
    case 0xF3: return "SONG";
    default:   return "DATA";
  }
}

//-----------------------------------------------------------------------------
// How many data bytes the message takes; MIDI_VAR for a system-exclusive,
// which runs until 0xF7. Derived rather than stored: it is a property of the
// status byte and there is one MidiMsg per message in every record.
static uint8_t midi_want(uint8_t st)
{
  int len = midi_data_len(st);

  return (uint8_t)((0xF0 == st) ? MIDI_VAR : (len > 0 ? len : 0));
}

//-----------------------------------------------------------------------------
// Was this record's status byte on the wire? Under running status it was not,
// and neither was it for the part of a message that resumes after a real-time
// byte interrupted it. The role of the record's first byte already says so.
static bool midi_running(const MidiAnalysis *a, const MidiMsg *m)
{
  return (MIDI_R_STATUS != a->role[m->first % LOGIC_MAX_BYTES]);
}

//-----------------------------------------------------------------------------
static void midi_note_name(uint8_t n, char *buf, int size)
{
  n &= 0x7F;

  snprintf(buf, size, "%s%d", g_midi_note[n % 12], n / 12 - 1);
}

//-----------------------------------------------------------------------------
static const char *midi_mfr_name(uint8_t id)
{
  for (unsigned i = 0; i < sizeof(g_midi_mfr) / sizeof(g_midi_mfr[0]); i++)
  {
    if (g_midi_mfr[i].id == id)
      return g_midi_mfr[i].name;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
// The whole message in one string: what a musician would read off it. This is
// the label the group carries, written once across however many bytes it took
// to say it, so "90 3C 64" reads "On C4 v100" rather than "NoteOn NOTE VEL".
static void midi_msg_text(const MidiMsg *m, char *buf, int size)
{
  char note[8];

  buf[0] = 0;

  if (0 == m->status)
  {
    // A data byte the record opened on: its status went past before the
    // trigger did, and nothing can be read out of the byte on its own
    snprintf(buf, size, "data");

    return;
  }

  if (0xF0 == m->status)
  {
    const char *mfr = (m->ndata > 0) ? midi_mfr_name(m->d[0]) : NULL;

    if (mfr)
      snprintf(buf, size, "SysEx %s", mfr);
    else if (m->ndata > 0)
      snprintf(buf, size, "SysEx %02X", m->d[0]);
    else
      snprintf(buf, size, "SysEx");

    return;
  }

  // Named, and no more: a message whose data has not all arrived must not
  // have a value written under it. Half of a pitch bend is not a pitch bend.
  if (m->ndata < midi_want(m->status))
  {
    snprintf(buf, size, "%s", midi_type_name(m->status));

    return;
  }

  switch (m->status & 0xF0)
  {
    case 0x80:
      midi_note_name(m->d[0], note, sizeof(note));
      snprintf(buf, size, "Off %s", note);
      break;

    case 0x90:
      midi_note_name(m->d[0], note, sizeof(note));

      // Note on at velocity zero IS a note off, and every sequencer sends it
      // that way because running status then covers a whole phrase. Printing
      // "On C4 v0" would be the one reading a player never means.
      if (m->d[1])
        snprintf(buf, size, "On %s v%d", note, m->d[1]);
      else
        snprintf(buf, size, "Off %s", note);
      break;

    case 0xA0:
      midi_note_name(m->d[0], note, sizeof(note));
      snprintf(buf, size, "AT %s=%d", note, m->d[1]);
      break;

    case 0xB0:
      if (m->d[0] >= 120)
        snprintf(buf, size, "%s", g_midi_mode[m->d[0] - 120]);
      else
        snprintf(buf, size, "CC%d=%d", m->d[0], m->d[1]);
      break;

    case 0xC0:
      snprintf(buf, size, "Prog %d", m->d[0]);
      break;

    case 0xD0:
      snprintf(buf, size, "Press %d", m->d[0]);
      break;

    case 0xE0:
      // Fourteen bits across two seven-bit halves, centre 8192. What the
      // player did is the offset from centre, so that is what is shown.
      snprintf(buf, size, "Bend %+d",
          (((int)m->d[1] << 7) | m->d[0]) - 8192);
      break;

    default:
      switch (m->status)
      {
        case 0xF1:
          snprintf(buf, size, "QF %d:%d", m->d[0] >> 4, m->d[0] & 0x0F);
          break;

        case 0xF2:
          snprintf(buf, size, "SPP %d", ((int)m->d[1] << 7) | m->d[0]);
          break;

        case 0xF3:
          snprintf(buf, size, "Song %d", m->d[0]);
          break;

        default:
          snprintf(buf, size, "%s", midi_type_name(m->status));
          break;
      }
      break;
  }
}

//-----------------------------------------------------------------------------
const MidiAnalysis *midi_analysis(void)
{
  return &g_midi;
}

//-----------------------------------------------------------------------------
void midi_byte_label(const MidiAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (MIDI_R_RT == a->role[idx])
  {
    snprintf(buf, size, "%s", midi_type_name(v));

    return;
  }

  midi_msg_text(&a->msg[a->midx[idx] % MIDI_MAX_MSGS], buf, size);
}

//-----------------------------------------------------------------------------
// What THIS byte is, as opposed to what the message adds up to. The status
// byte carries the message type and the channel, because that is literally
// what its two nibbles are; the data bytes carry their field names.
void midi_field_label(const MidiAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  const MidiMsg *m = &a->msg[a->midx[idx] % MIDI_MAX_MSGS];

  switch (a->role[idx])
  {
    case MIDI_R_RT:
      snprintf(buf, size, "%s", midi_type_name(v));
      break;

    case MIDI_R_ORPHAN:
      snprintf(buf, size, "data");
      break;

    case MIDI_R_STATUS:
      if (m->status < 0xF0)
        snprintf(buf, size, "%s ch%d", midi_short_name(m->status),
            (m->status & 0x0F) + 1);
      else
        snprintf(buf, size, "%s", midi_type_name(m->status));
      break;

    case MIDI_R_SYSEX:
      // The first data byte of a SysEx says whose it is; the last one is the
      // 0xF7 that closes it and is not data at all
      if (0xF7 == v)
        snprintf(buf, size, "EOX");
      else if (idx == m->first + 1 && !midi_running(a, m))
        snprintf(buf, size, "MFR");
      else
        snprintf(buf, size, "DATA");
      break;

    case MIDI_R_DATA:
    {
      // Where the status byte is on the wire it takes the message's first
      // slot; under running status the data starts at the first byte. A
      // message that a real-time byte interrupted resumes in a record of its
      // own, and dfirst is how many of its data bytes went past before it.
      int di = m->dfirst + idx - m->first - (midi_running(a, m) ? 0 : 1);

      snprintf(buf, size, "%s", midi_data_name(m->status, di));
      break;
    }

    default:
      break;
  }
}

//-----------------------------------------------------------------------------
// One message is one thing on the screen. Its data bytes belong to it whether
// or not a status byte was sent for them - under running status the data IS
// the message - so the group is what the parser put together, and the reading
// is written once across all of it.
void midi_group_at(const MidiAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  const MidiMsg *m = &a->msg[a->midx[idx] % MIDI_MAX_MSGS];

  if (m->count < 1)
    return;

  *start = m->first;
  *len = m->count;
}

//-----------------------------------------------------------------------------
// Open a record for a new message. A message a real-time byte interrupted
// gets a second record rather than a discontiguous one: the band draws a
// group as a run of adjacent bytes, and a clock byte in the middle of a note
// is exactly the case that would break that.
static int midi_new(LogicResult *r, MidiAnalysis *a, int i, uint8_t status)
{
  // More messages than there is room to describe. The byte list is cut here
  // rather than carried on with: a byte whose message was never recorded
  // would group and label itself as some other message's, and a record that
  // says plainly it held more is worth more than one that quietly lies about
  // what the last of it was.
  if (a->msgs >= MIDI_MAX_MSGS)
  {
    r->count = i;
    r->truncated = true;

    return -1;
  }

  MidiMsg *m = &a->msg[a->msgs];

  memset(m, 0, sizeof(*m));
  m->status = status;
  m->first = (uint8_t)i;

  return a->msgs++;
}

//-----------------------------------------------------------------------------
// Read the byte stream as MIDI. Fills the analysis and counts what does not
// fit the grammar; the caller decides what to do about that.
static void midi_parse(LogicResult *r, MidiAnalysis *a)
{
  uint8_t running = 0;     // the channel status still in force
  uint8_t status = 0;      // the message currently taking data bytes
  uint8_t d[2] = { 0, 0 };
  int nd = 0;              // how many it has
  int need = 0;            // how many more it wants; -1 = until 0xF7
  int msg = -1;            // the record they are going into
  int chan = -1;           // the one channel in the record, if there is one
  bool seen_status = false;   // ...not counting real-time, which owns nothing

  memset(a, 0, sizeof(*a));

  for (int i = 0; i < r->count; i++)
  {
    uint8_t v = r->bytes[i];

    // System real-time. One byte, no data, and allowed to land anywhere -
    // including between two data bytes of a message it does not disturb.
    if (v >= 0xF8)
    {
      int m = midi_new(r, a, i, v);

      if (m < 0)
        break;

      a->msg[m].count = 1;
      a->midx[i] = (uint8_t)m;
      a->role[i] = MIDI_R_RT;

      if (-1 == midi_data_len(v))
        a->bad++;          // 0xF9 and 0xFD are not messages
      else
        a->rt++;

      msg = -1;            // what it interrupted resumes in its own record
      continue;
    }

    if (v & 0x80)
    {
      // A status byte while the last message was still owed data: that
      // message was cut short, which a transmitter does not do. The 0xF7
      // that closes a system-exclusive is the one status byte that is
      // SUPPOSED to arrive while a message is open - it is how it ends.
      if (0 != need && !(0xF7 == v && need < 0))
        a->bad++;

      if (0xF7 == v && need < 0)
      {
        // The end of a system-exclusive belongs to the system-exclusive
        if (msg < 0)
        {
          msg = midi_new(r, a, i, 0xF0);

          if (msg < 0)
            break;

          a->msg[msg].dfirst = (uint8_t)nd;
          a->msg[msg].d[0] = d[0];
          a->msg[msg].d[1] = d[1];
        }

        a->msg[msg].count++;
        a->msg[msg].ndata = (uint8_t)nd;
        a->midx[i] = (uint8_t)msg;
        a->role[i] = MIDI_R_SYSEX;
        a->full++;

        need = 0;
        status = 0;
        msg = -1;
        continue;
      }

      if (0xF7 == v)
        a->bad++;          // an EOX with no system-exclusive in front of it

      msg = midi_new(r, a, i, v);

      if (msg < 0)
        break;

      a->msg[msg].count = 1;
      a->midx[i] = (uint8_t)msg;
      a->role[i] = MIDI_R_STATUS;

      // -2 is the system-exclusive's "until 0xF7" and is not a fault; -1 is
      // 0xF4 and 0xF5, which the standard leaves undefined and nothing sends
      if (-1 == midi_data_len(v))
        a->bad++;

      status = v;
      seen_status = true;
      nd = 0;
      d[0] = 0;
      d[1] = 0;
      need = (0xF0 == v) ? -1 : (midi_data_len(v) > 0 ? midi_data_len(v) : 0);

      if (0xF0 == v)
        a->sysex = true;

      if (v < 0xF0)
      {
        int ch = v & 0x0F;

        chan = (chan < 0 || chan == ch) ? ch : 16;   // 16 = more than one
        running = v;
      }
      else
      {
        running = 0;       // system common cancels running status
      }

      if (0 == need)
      {
        a->full++;
        msg = -1;
        status = 0;
      }

      continue;
    }

    // A data byte. Which message it belongs to is the whole question.
    if (0 == need)
    {
      if (running)
      {
        // Running status: the last channel status is still in force and a
        // fresh message of that type starts right here, with no status byte
        // on the wire to point at
        status = running;
        nd = 0;
        d[0] = 0;
        d[1] = 0;
        need = midi_data_len(status);

        msg = midi_new(r, a, i, status);

        if (msg < 0)
          break;
      }
      else
      {
        // Nothing is expecting this byte. At the head of the record that is
        // no fault at all - the status went past before the trigger did, and
        // the byte simply cannot be read on its own. After a status byte has
        // gone by in this very record it is a fault: something either sent a
        // data byte too many or was decoded wrongly.
        int m = midi_new(r, a, i, 0);

        if (m < 0)
          break;

        a->msg[m].count = 1;
        a->midx[i] = (uint8_t)m;
        a->role[i] = MIDI_R_ORPHAN;

        if (seen_status)
          a->bad++;

        continue;
      }
    }
    else if (msg < 0)
    {
      // A real-time byte interrupted this message; it carries on here
      msg = midi_new(r, a, i, status);

      if (msg < 0)
        break;

      a->msg[msg].dfirst = (uint8_t)nd;
      a->msg[msg].d[0] = d[0];
      a->msg[msg].d[1] = d[1];
    }

    if (nd < 2)
      d[nd] = v;

    nd++;

    a->msg[msg].count++;
    a->msg[msg].ndata = (uint8_t)nd;
    a->msg[msg].d[0] = d[0];
    a->msg[msg].d[1] = d[1];
    a->midx[i] = (uint8_t)msg;
    a->role[i] = (need < 0) ? MIDI_R_SYSEX : MIDI_R_DATA;

    if (need > 0 && 0 == --need)
    {
      a->full++;
      msg = -1;            // complete; `status` stays in force for the next
    }
  }

  // Whatever was still open when the record ran out is not a broken message,
  // it is a message this record does not hold the end of
  if (0 != need && msg >= 0)
    a->msg[msg].partial = 1;

  a->chan = (chan < 0 || chan >= 16) ? 0 : chan + 1;
}

//-----------------------------------------------------------------------------
// One 8N1 decode pass at 31250 baud and a given polarity. The framing is the
// UART's, because on the wire MIDI is a UART; only the rate is not a choice.
static int midi_scan(const uint8_t *data, int size, int offset, int mid,
    int idle, int bit_x256, const LogicScratch *scratch, int runs,
    LogicResult *out)
{
  const int *run_len = scratch->len;
  const int *run_pos = scratch->pos;
  const uint8_t *run_lvl = scratch->lvl;
  int64_t gap_x256 = (int64_t)bit_x256 * MIDI_IDLE_GAP_BITS;

  out->count = 0;
  out->errors = 0;
  out->burst_start = false;
  out->truncated = false;

  for (int r = 0; r < runs - 1; r++)
  {
    if (run_lvl[r] != idle)
      continue;            // a frame starts where a start bit follows the idle

    if (out->count >= LOGIC_MAX_BYTES)
    {
      out->truncated = true;
      break;
    }

    int64_t frame = (int64_t)run_pos[r + 1] * 256;
    int value = 0;
    bool ok = true;

    for (int b = 0; b < 8; b++)   // LSB first, centre of each bit
    {
      int64_t center = frame + ((int64_t)bit_x256 * (3 + 2 * b)) / 2;
      int si = (int)(center / 256);

      if (si >= size)
      {
        ok = false;
        break;
      }

      int bitv = (sample_at(data, size, offset, si) > mid) ? 1 : 0;

      if (!idle)
        bitv = !bitv;

      if (bitv)
        value |= (1 << b);
    }

    if (!ok)
      break;               // the record ends inside this frame

    int64_t stop_center = frame + ((int64_t)bit_x256 * 19) / 2;
    int stop_i = (int)(stop_center / 256);

    if (stop_i >= size)
      break;

    if (((sample_at(data, size, offset, stop_i) > mid) ? 1 : 0) != idle)
    {
      out->errors++;
      continue;            // try the next edge rather than skipping a frame
    }

    if (0 == out->count && (int64_t)run_len[r] * 256 >= gap_x256)
      out->burst_start = true;

    out->bytes[out->count] = (uint8_t)value;
    out->pos[out->count] = run_pos[r + 1];
    out->end[out->count] =
        (int)((frame + (int64_t)bit_x256 * MIDI_FRAME_BITS) / 256);
    out->count++;

    // Resume at the first run inside or after the stop bit, so this frame's
    // own data runs are not read as new frames
    int frame_end =
        (int)((frame + (int64_t)bit_x256 * (MIDI_FRAME_BITS - 1)) / 256);

    while (r + 1 < runs && run_pos[r + 1] < frame_end)
      r++;

    r--;
  }

  return out->count;
}

//-----------------------------------------------------------------------------
// The shortest interior run: one bit time or a multiple of it. The first and
// last runs are cut by the ends of the record and say nothing.
static int midi_min_run(const LogicScratch *scratch, int runs)
{
  int tmin = 0;

  for (int r = 1; r + 1 < runs; r++)
  {
    if (0 == tmin || scratch->len[r] < tmin)
      tmin = scratch->len[r];
  }

  return tmin;
}

//-----------------------------------------------------------------------------
int midi_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_MIDI;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 4 || period_ns <= 0)
    return 0;

  int64_t bit = (256000000000ll + (int64_t)MIDI_BAUD * period_ns / 2) /
      ((int64_t)MIDI_BAUD * period_ns);

  if (bit < MIDI_MIN_BIT_SAMPLES * 256)
    return 0;              // undersampled: the stop bit cannot be placed

  if (bit * MIDI_FRAME_BITS > (int64_t)size * 256)
    return 0;              // not one frame fits in the record at this rate

  // A statement about the record rather than a guess at its rate: every run
  // is a whole number of bit times, so a record whose shortest run is well
  // under one bit at 31250 baud is not carrying 31250 baud.
  int tmin = midi_min_run(scratch, runs);

  if (0 == tmin || (int64_t)(tmin + MIDI_MIN_RUN_SLACK) * 256 < bit)
    return 0;

  // MIDI leaves the receiver's side of the optocoupler idle high, but an
  // inverting buffer between it and the probe costs one more pass
  int64_t best = 0;
  int best_idle = 1;

  for (int p = 0; p < 2; p++)
  {
    int idle = p ? 0 : 1;
    int n = midi_scan(data, size, offset, mid, idle, (int)bit, scratch, runs,
        out);

    if (n < 2)
      continue;

    midi_parse(out, &g_midi);

    // Bytes are what the record holds; whole messages are what says it was
    // read the right way up. Framing errors and grammar faults cost more than
    // a byte is worth, because the wrong polarity produces plenty of both.
    int64_t score = (int64_t)n * 2 + (int64_t)g_midi.full * 4 -
        (int64_t)out->errors * 6 - (int64_t)g_midi.bad * 8;

    if (score > best)
    {
      best = score;
      best_idle = idle;
    }
  }

  if (best <= 0)
    return 0;

  midi_scan(data, size, offset, mid, best_idle, (int)bit, scratch, runs, out);
  midi_parse(out, &g_midi);

  if (out->count < 2)
    return 0;

  // Grammar faults are REPORTED, not hidden and not refused. A dropped byte,
  // a message some transmitter cut short, a data byte with no status left in
  // front of it - those are faults in the traffic, and a decoder that answers
  // them by falling back to "UART 31250 8N1" has hidden the one thing worth
  // seeing. What is refused is a record where the faults are the majority,
  // because that is not traffic with faults in it, that is this decoder
  // reading something else wrongly.
  if (g_midi.bad * 2 > out->count)
    return 0;

  // No status byte anywhere means no grammar in the record at all: every byte
  // has bit 7 clear, which is what a line carrying text looks like and what a
  // thresholded sine looks like. There is nothing here to read AS MIDI - not
  // even when MIDI is picked by name, because the answer would be a screenful
  // of bytes labelled "data" - so this one is refused rather than qualified.
  bool has_status = false;

  for (int i = 0; i < out->count && !has_status; i++)
    has_status = (0 != (out->bytes[i] & 0x80));

  if (!has_status)
    return 0;

  out->idle_high = (0 != best_idle);
  out->rate = MIDI_BAUD;
  out->errors += g_midi.bad;

  // Was the line still sending when the record ran out? Same question the
  // UART asks, and the same answer: a tail run shorter than an idle gap means
  // the message went on past the end of the buffer.
  out->overrun = ((int64_t)scratch->len[runs - 1] * 256 <
      bit * MIDI_IDLE_GAP_BITS);

  // What settles it is a whole message that CARRIED something: a status byte
  // and the data bytes it asked for, arriving in that order. A status byte on
  // its own does not settle it, because the one-byte messages are the easiest
  // to fake - 0xF8 is a bit pattern, and a square wave sampled at the wrong
  // rate produces bit patterns all day.
  bool rich = false;

  for (int i = 0; i < g_midi.msgs && !rich; i++)
  {
    const MidiMsg *m = &g_midi.msg[i];

    rich = (m->ndata > 0 && !m->partial &&
        m->ndata >= midi_want(m->status));
  }

  // The other thing a square wave cannot do is STOP. Real-time bytes come one
  // at a time with the line resting between them - a clock at 120 BPM is one
  // byte every 20 ms and the byte itself is a third of a millisecond - so a
  // record of nothing but clock bytes is still MIDI if it has that rest in
  // it, and is still a square wave if it has not.
  int longest_idle = 0;

  for (int r = 0; r < runs; r++)
  {
    if (scratch->lvl[r] == best_idle && scratch->len[r] > longest_idle)
      longest_idle = scratch->len[r];
  }

  bool rested = ((int64_t)longest_idle * 256 >= bit * MIDI_IDLE_GAP_BITS);

  // Ambiguity is about IDENTIFICATION and nothing else: does this record tell
  // MIDI apart from something simpler. A grammar fault does not make it
  // ambiguous - a record holding twenty good MIDI bytes and one byte nobody
  // should have sent has answered the identification question, and turning it
  // down over the fault would hand the panel to the UART decoder, which reads
  // the same fault as an ordinary byte and says nothing about it.
  out->ambiguous = (!rich && !rested) ||
      (out->count < MIDI_AUTO_MIN_BYTES && !rich);

  // The header: one message is worth spelling out, and a record full of them
  // is worth counting. The channel goes in when the whole record is on one,
  // which is the common case and the one worth knowing at a glance.
  char body[24];
  const char *err = (out->errors > 0) ? " err" : "";

  if (1 == g_midi.msgs)
  {
    midi_msg_text(&g_midi.msg[0], body, sizeof(body));

    if (g_midi.chan > 0)
      snprintf(out->info, sizeof(out->info), "MIDI ch%d %s%s", g_midi.chan,
          body, err);
    else
      snprintf(out->info, sizeof(out->info), "MIDI %s%s", body, err);
  }
  else if (g_midi.chan > 0)
  {
    snprintf(out->info, sizeof(out->info), "MIDI %d msgs ch%d%s%s",
        g_midi.msgs, g_midi.chan, out->truncated ? "+" : "", err);
  }
  else
  {
    snprintf(out->info, sizeof(out->info), "MIDI %d msgs%s%s", g_midi.msgs,
        out->truncated ? "+" : "", err);
  }

  return out->count;
}
