/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Single-wire protocol decoders: shared result/scratch types and the
 * auto-detecting dispatcher. Pure C, host-testable.
 *
 * Every decoder consumes the same ring buffer of 8-bit samples and fills a
 * LogicResult, including the sample position of every decoded byte so the
 * UI can highlight bytes on the waveform and jump between them.
 */

#ifndef _LOGIC_DECODE_H_
#define _LOGIC_DECODE_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
// A console line is longer than a hex dump: 32 bytes cut "MILKV-UART-TEST
// 115200-8N1 #0123456789" in half, and the panel scrolls anyway. 64 costs
// 576 bytes of result, which is why both LogicResults in the firmware are
// static rather than automatic.
#define LOGIC_MAX_BYTES  64
#define LOGIC_MAX_RUNS   512

/*- Types -------------------------------------------------------------------*/
typedef enum
{
  PROTO_NONE = 0,
  PROTO_AUTO = 0, // as a config value: try everything
  PROTO_UART = 1,
  PROTO_ONEWIRE,
  PROTO_WS2812,
  PROTO_NEC,
  PROTO_RAW,      // fallback: any digital signal, raw bits at the detected rate
  // Appended, not inserted: config.decoder_proto stores one of these numbers
  // and a saved config would silently start meaning a different protocol.
  // The cascade's order is its own table and owes nothing to this one.
  PROTO_SERVO,    // RC/servo PWM: a fixed-rate pulse train, width is the value
  PROTO_CAN,      // CAN 2.0A/2.0B, one wire of the pair or the transceiver's TX
  PROTO_DHT,      // DHT11/DHT22 humidity and temperature, 40 bits and a sum
  PROTO_SENT,     // SAE J2716: nibbles in the time between falling edges
  PROTO_MIDI,     // MIDI 1.0: 8N1 at 31250 baud, and what the bytes say
  PROTO_LIN,      // LIN: a break, a sync byte that gives the rate, then 8N1
  PROTO_COUNT,
} proto_t;

typedef struct
{
  proto_t  proto;
  int      rate;       // UART: baud; others: bit/slot time in ns
  bool     idle_high;
  // The record caught the line at rest before the first byte, i.e. this is
  // the head of a message and not a slice out of the middle of one. What
  // "stop at message start" waits for.
  bool     burst_start;
  // More bytes were on the wire than LOGIC_MAX_BYTES could hold
  bool     truncated;
  // The record ran out while the line was still sending: what was decoded is
  // the part of a message that fit, not the message
  bool     overrun;
  // The decoder matched, but this record does not actually tell its protocol
  // apart from something simpler - see the dispatcher, which is the only
  // place that knows whether the caller asked for this protocol by name
  bool     ambiguous;
  int      count;      // decoded bytes
  int      errors;     // framing/timing errors
  char     info[32];   // one-line summary for the panel header
  uint8_t  bytes[LOGIC_MAX_BYTES];
  int      pos[LOGIC_MAX_BYTES];  // byte start, time-order sample index
  int      end[LOGIC_MAX_BYTES];  // byte end (exclusive), same units
} LogicResult;

// What a decoded 1-Wire byte is in its transaction. The bus carries one
// protocol at the bit level and a dozen different devices above it, so the
// numbers only mean something once the ROM/function command that framed them
// is known.
typedef enum
{
  OW_R_NONE = 0,
  OW_R_ROMCMD,   // READ ROM / MATCH / SKIP / SEARCH
  OW_R_FNCMD,    // CONVERT T / READ SP / ...
  OW_R_FAMILY,   // ROM byte 0: which device this is
  OW_R_SERIAL,   // ROM bytes 1..6
  OW_R_ROMCRC,   // ROM byte 7
  OW_R_SEARCH,   // the search triplets, three bus slots per ROM bit
  OW_R_TEMPL,    // scratchpad 0..8
  OW_R_TEMPH,
  OW_R_TH,
  OW_R_TL,
  OW_R_CFG,
  OW_R_RSVD,     // scratchpad byte 6: reserved, no value to expect
  OW_R_SPCRC,
  // The two reserved bytes a working part DOES have a value for. On a
  // DS18S20 they are not reserved at all - byte 6 counts the remainder of
  // the conversion and byte 7 the counts per degree - which is where the
  // 0x10 the DS18B20 still returns came from.
  OW_R_RSV_FF,   // byte 5, reads 0xFF
  OW_R_RSV_10,   // byte 7, reads 0x10
} ow_role_t;

// The transaction read of a 1-Wire record: which device answered and what it
// said. Filled by onewire_decode(), fetched with onewire_analysis().
typedef struct
{
  uint8_t  family;      // ROM family code; 0 when no ROM was on the wire
  uint8_t  rom[8];      // family, 6 serial bytes, CRC8
  bool     rom_seen;    // eight ROM bytes came past
  bool     rom_valid;   // ...and their CRC8 checks out
  bool     sp_valid;    // a nine-byte scratchpad with a good CRC8
  bool     temp_valid;  // a temperature was read out of it
  bool     thermometer; // the traffic is a DS18x20 conversation
  bool     s20;         // the older DS18S20: half-degree steps, no config byte
  bool     ext_res;     // ...read to 1/16 C anyway, out of its count registers
  int      temp_mc;     // milli-degrees C
  int      bits;        // conversion resolution 9..12; 0 = not applicable
  // Short device name ("DS18B20", "DS1990 key"), NULL when unidentified.
  // `sure` is what the ROM said, and only that: a family code under a good
  // CRC names the part outright. A scratchpad proves the CLASS - a config
  // register in byte 4 is a DS18B20's, not a DS18S20's - but DS1822 and
  // DS1825 keep the same one, so a part named without a ROM gets a '?'.
  const char *device;
  bool     sure;
  uint8_t  role[LOGIC_MAX_BYTES];
} OwAnalysis;

// What a decoded CAN byte is. A frame goes out as its identifier, its length
// code, its data and its CRC, so the hex dump is what was on the wire and the
// labels say which part is which.
typedef enum
{
  CAN_R_NONE = 0,
  CAN_R_ID,     // 2 bytes (11-bit id) or 4 (29-bit), right-aligned
  CAN_R_DLC,
  CAN_R_DATA,
  CAN_R_CRC,    // the 15-bit CRC, high byte
  CAN_R_ACK,    // its low byte, which is also where the ACK slot is reported
} can_role_t;

#define CAN_MAX_FRAMES  8

typedef struct
{
  uint32_t id;
  uint8_t  dlc;
  bool     ext;      // 29-bit identifier
  bool     rtr;      // remote request: a frame that asks rather than tells
  bool     fd;       // CAN FD: recognised by its FDF bit, not decoded
  bool     crc_ok;
  bool     ack;      // somebody on the bus acknowledged it
  bool     cut;      // the record ended inside the frame's end-of-frame field
  int      first;    // index of the frame's first byte in LogicResult.bytes
  int      count;
} CanFrame;

typedef struct
{
  int      rate;     // bit/s
  int      frames;
  int      crc_ok;   // how many of them checked out
  bool     fd_seen;  // a CAN FD frame went past; this decoder does not read it
  CanFrame frame[CAN_MAX_FRAMES];
  uint8_t  role[LOGIC_MAX_BYTES];
  uint8_t  fidx[LOGIC_MAX_BYTES];  // which frame each byte belongs to
} CanAnalysis;

// The reading behind a DHT frame's five bytes. Which of the two parts sent
// them is a question about the numbers, not about the wire: both send forty
// bits the same way, and only the meaning of the bytes differs.
typedef struct
{
  bool dht22;     // 16-bit tenths, rather than the DHT11's whole units
  bool sure;      // one reading was plausible and the other was not
  bool valid;     // the checksum agreed
  int  rh_x10;    // relative humidity, tenths of a percent
  int  t_x10;     // temperature, tenths of a degree, signed
} DhtAnalysis;

// SENT carries nibbles, not bytes: one result byte per nibble, holding 0..15.
typedef enum
{
  SENT_R_NONE = 0,
  SENT_R_STATUS,   // status and serial communication nibble
  SENT_R_DATA,
  SENT_R_CRC,
} sent_role_t;

#define SENT_MAX_FRAMES  8

typedef struct
{
  uint8_t status;
  uint8_t ndata;     // data nibbles between the status and the CRC
  uint8_t crc;       // as received
  bool    crc_ok;
  bool    legacy;    // matched the 2008 CRC rather than the 2010 one
  // The record ran out before the closing edge of the frame's last pulse, so
  // that pulse has no measurable value and the frame carries no CRC here.
  // Everything after the status nibble is data; nothing is being called a CRC.
  bool    no_crc;
  // The common layout: three nibbles to a 12-bit signal. Only assembled
  // where the nibble count says it plainly - six nibbles or three.
  bool    have_s1, have_s2;
  int     s1, s2;
  int     first;     // index of the frame's first nibble in LogicResult.bytes
  int     count;
} SentFrame;

typedef struct
{
  int       tick_ns;   // the unit time, read off the frame's own sync pulse
  int       frames;
  int       crc_ok;
  SentFrame frame[SENT_MAX_FRAMES];
  uint8_t   role[LOGIC_MAX_BYTES];
  uint8_t   fidx[LOGIC_MAX_BYTES];
} SentAnalysis;

// What a decoded MIDI byte is. The wire carries a stream and not frames: a
// status byte names a message and says how many data bytes follow, running
// status lets the next message leave its status byte out entirely, and a
// real-time byte may land between any two bytes without disturbing either.
typedef enum
{
  MIDI_R_NONE = 0,
  MIDI_R_STATUS,   // the status byte of a channel-voice or system-common message
  MIDI_R_DATA,     // one of its data bytes
  MIDI_R_RT,       // system real-time: clock, start, stop - a message on its own
  MIDI_R_SYSEX,    // inside a system-exclusive, including the 0xF7 that ends it
  MIDI_R_ORPHAN,   // a data byte whose status went past before the record began
} midi_role_t;

// The most messages one record is read as. A three-byte note message needs
// 21 of these to fill a record and a two-byte one under running status needs
// 32, so this covers any record of actual traffic; what it does not cover is
// a screenful of one-byte messages - MIDI clock, active sensing - and there
// the record is cut short and says so, which beats holding half a kilobyte
// of TCM against a case nobody reads to the end anyway.
#define MIDI_MAX_MSGS  40

// Eight bytes, and kept that way on purpose: there is one of these per message
// and a record can hold sixty-four of them, so what looks like a field worth
// caching is half a kilobyte of a part that has none to spare. Everything
// derivable is derived - how many data bytes the message takes comes out of
// the status byte, and whether its status byte was on the wire comes out of
// the role of its first byte.
typedef struct
{
  uint8_t status;   // the status in force; 0 = none, for an orphan data byte
  uint8_t d[2];     // its first two data bytes, as far as they arrived
  uint8_t ndata;    // how many did
  uint8_t dfirst;   // index of the first data byte THIS record carries
  uint8_t first;    // where the record starts in LogicResult.bytes
  uint8_t count;
  uint8_t partial;  // the capture ran out before the message did
} MidiMsg;

typedef struct
{
  int16_t msgs;
  int16_t full;     // messages that arrived whole
  int16_t rt;       // system real-time bytes
  int16_t bad;      // bytes the grammar cannot account for
  int16_t chan;     // 1..16 when the whole record is one channel, else 0
  bool    sysex;
  MidiMsg msg[MIDI_MAX_MSGS];
  uint8_t midx[LOGIC_MAX_BYTES];   // which message each byte belongs to
  uint8_t role[LOGIC_MAX_BYTES];
} MidiAnalysis;

// What a decoded LIN byte is. A frame is a header the master sends and a
// response whichever slave owns the identifier sends back, and the two halves
// are told apart by nothing on the wire - only by which field they are.
typedef enum
{
  LIN_R_NONE = 0,
  LIN_R_SYNC,    // 0x55, and the rate came out of its edges
  LIN_R_PID,     // six identifier bits and two of parity over them
  LIN_R_DATA,
  LIN_R_CSUM,
} lin_role_t;

#define LIN_MAX_FRAMES  6

typedef struct
{
  uint8_t pid;        // the protected identifier, as received
  uint8_t id;         // its low six bits, which is what anyone calls the frame
  uint8_t ndata;
  uint8_t csum;       // as received
  uint8_t first;      // index of the frame's sync byte in LogicResult.bytes
  uint8_t count;
  bool    parity_ok;
  bool    csum_ok;
  bool    enhanced;   // the checksum that agreed had the PID in it (LIN 2.x)
  // The master asked and nobody answered. Not an error in the record - a fact
  // about the bus, and usually the one worth knowing.
  bool    no_resp;
  bool    cut;        // the record ended inside the frame
} LinFrame;

typedef struct
{
  int      rate;      // bit/s, read off the sync byte
  int      frames;
  int      csum_ok;
  LinFrame frame[LIN_MAX_FRAMES];
  uint8_t  role[LOGIC_MAX_BYTES];
  uint8_t  fidx[LOGIC_MAX_BYTES];
} LinAnalysis;

// Working memory (~4.5 KB) provided by the caller: the firmware puts it in
// spare main SRAM, host tests use a static instance
typedef struct
{
  int      len[LOGIC_MAX_RUNS];
  int      pos[LOGIC_MAX_RUNS];
  uint8_t  lvl[LOGIC_MAX_RUNS];

  // Run-split cache. Every decoder in the auto cascade thresholds the SAME
  // record, and that is two full passes over 24K samples each time; the
  // dispatcher invalidates this once per call and the cascade then pays for
  // the split once instead of five times.
  const uint8_t *cache_data;
  int      cache_size;
  int      cache_offset;
  int      cache_runs;
  int      cache_mid;
  uint32_t cache_hash;
  uint32_t cache_valid;
} LogicScratch;

// Written into cache_valid by logic_runs(); the scratch lives in otherwise
// uninitialised SRAM, so the marker has to be something garbage will not be
#define LOGIC_CACHE_MAGIC  0x5ab1e10cu

/*- Prototypes --------------------------------------------------------------*/
// Individual decoders; each returns the number of bytes decoded (0 = the
// record does not look like this protocol)
int uart_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);

// Decode at this baud and no other; 0 (the default) works it out from the
// record. Sticks until changed.
void uart_decode_set_baud(int baud);

int onewire_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);

// The transaction read of the record onewire_decode() last looked at, and the
// per-byte label that comes out of it ("DS18B20", "SN", "+25.06C", "12bit").
// Valid until the next onewire_decode() call - the caller keeps a copy if it
// keeps the result.
const OwAnalysis *onewire_analysis(void);
void onewire_byte_label(const OwAnalysis *a, int idx, uint8_t v,
    char *buf, int size);

// Which bytes make up ONE value with byte `idx`, as a half-open span. A
// temperature is two bytes, a CAN identifier is four, a SENT signal is three
// nibbles - and a reader shown "D1 D2 S1=543" has to work out for themselves
// that the three of them are one number. UTF-8 has been saying it properly
// since it was added: the bytes light together and the value is written once
// across them. These say the same thing for the protocols that assemble one.
//
// Every group function tiles its result: walking from 0 and stepping by the
// length returned always lands on the next group's first byte.
void onewire_group_at(const OwAnalysis *a, int idx, int *start, int *len);

// ...and what a byte is ON ITS OWN, which is a different question and belongs
// on a different row: the group says "these three nibbles are 0x394", the
// field says "this one is D2 of them". A reader needs both, the way the row
// of bit numbers and the bracket labelled "command" under it are both needed.
void onewire_field_label(const OwAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
int ws2812_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
int nec_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
// Servo bytes are the pulse width in tens of microseconds, so 150 is 1.50 ms
int servo_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);

// CAN 2.0A/2.0B. Reports only what its CRC-15 confirms, so it is safe to run
// ahead of the generic decoders. Same accessors as 1-Wire: the frame-level
// read of the last record, and the per-byte label that comes out of it.
int can_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const CanAnalysis *can_analysis(void);
void can_byte_label(const CanAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void can_group_at(const CanAnalysis *a, int idx, int *start, int *len);
void can_field_label(const CanAnalysis *a, int idx, uint8_t v,
    char *buf, int size);

// DHT11 / DHT22 (AM2302): forty bits, five bytes, the last one their sum
int dht_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const DhtAnalysis *dht_analysis(void);
void dht_byte_label(const DhtAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void dht_group_at(const DhtAnalysis *a, int idx, int *start, int *len);
void dht_field_label(const DhtAnalysis *a, int idx, uint8_t v,
    char *buf, int size);

// SENT (SAE J2716). One result byte per nibble; rate is the tick time in ns
int sent_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const SentAnalysis *sent_analysis(void);
void sent_byte_label(const SentAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void sent_group_at(const SentAnalysis *a, int idx, int *start, int *len);
void sent_field_label(const SentAnalysis *a, int idx, uint8_t v,
    char *buf, int size);

// MIDI 1.0: 8N1 at 31250 baud and nothing else, read as messages rather than
// as bytes. The rate alone is not proof and the grammar alone is not proof;
// this one asks for both before it answers.
int midi_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const MidiAnalysis *midi_analysis(void);
void midi_byte_label(const MidiAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void midi_group_at(const MidiAnalysis *a, int idx, int *start, int *len);
void midi_field_label(const MidiAnalysis *a, int idx, uint8_t v,
    char *buf, int size);

// LIN. Found from its break - at least ten dominant bits, which 8N1 cannot
// send - and clocked from its own sync byte, so there is no rate to guess at.
// Its bytes each stand alone, so there is no group function: what a byte is
// worth saying about it fits on the one row.
int lin_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const LinAnalysis *lin_analysis(void);
void lin_byte_label(const LinAnalysis *a, int idx, uint8_t v,
    char *buf, int size);

// Dispatcher: forced = PROTO_AUTO tries every decoder and keeps the best
// fit (most bytes, then fewest errors); a specific proto runs only that one
int logic_decode(const uint8_t *data, int size, int offset, int period_ns,
    proto_t forced, LogicScratch *scratch, LogicResult *out);

// Shared helper: threshold the record into level runs with hysteresis.
// Returns the run count, 0 when the signal has no digital-looking swing.
// mid_out receives the threshold used.
int logic_runs(const uint8_t *data, int size, int offset,
    LogicScratch *scratch, int *mid_out);

const char *logic_proto_name(proto_t proto);

#endif // _LOGIC_DECODE_H_
