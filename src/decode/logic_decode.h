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
// 115200-8N1 #0123456789" in half, and the panel scrolls anyway. That is what
// 64 was for, and both LogicResults in the firmware are static rather than
// automatic because of what it costs.
//
// 80 is CAN FD's doing. One FD frame is four bytes of identifier, a length
// code, sixty-four of payload and three of CRC - seventy-two - and a record
// that cannot hold the biggest frame a protocol has shows it short, which for
// a hex dump is the one thing worse than showing it long. 144 bytes more of
// result, twice over.
#define LOGIC_MAX_BYTES  80
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
  PROTO_EV1527,   // 433 MHz remotes: 24 bits, every one of them 4T long
  PROTO_DSHOT,    // ESC throttle: 16 bits, duty carries the value, CRC-4
  // The data line of an SPI bus WITHOUT its clock. Not a decode: a
  // reconstruction, on assumptions it states. Never chosen automatically.
  PROTO_SPI,
  PROTO_MANCH,    // Manchester / bi-phase: the line code under RC5, DALI, tags
  PROTO_RC5,      // RC5, RC5X and RC6: the Philips remotes, read as messages
  PROTO_DALI,     // IEC 62386 lighting: Manchester at 1200, read as commands
  PROTO_KNX,      // KNX TP1: 9600, pulse-presence coded - NOT Manchester
  PROTO_SWO,      // SWO/ITM: a Cortex-M's trace pin, 8N1 carrying ITM packets
  // SWD on SWDIO alone. Like SPI it is one probe on a clocked bus - unlike
  // SPI, the protocol checks itself, so a transaction that reads clean is
  // confirmed rather than merely plausible.
  PROTO_SWD,
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
  OW_R_PIO,      // a DS2413's pin state, answering PIO ACCESS READ
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
  // A key was TOUCHED to a reader: a reset, READ ROM, eight bytes that check
  // out, and no function command anywhere. That is the whole conversation an
  // iButton has, and it is worth naming as an event rather than as a device.
  bool     ibutton;
  // ...and the same transaction from a family with no name here. The record
  // is not incomplete and the decoder is not confused: an identity was asked
  // for and given, and the identity is the answer.
  bool     id_only;
  bool     pio_valid;   // a DS2413 answered PIO ACCESS READ
  uint8_t  pio;
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
  // The CRC, high byte first: two bytes of a classic frame's 15-bit one,
  // three of a CAN FD frame's 17- or 21-bit one
  CAN_R_CRC,
  CAN_R_ACK,    // its low byte, which is also where the ACK slot is reported
} can_role_t;

#define CAN_MAX_FRAMES  8
#define CAN_MAX_DATA    64   // what a CAN FD frame can carry

typedef struct
{
  uint32_t id;
  uint8_t  dlc;
  // What the length code stands for. Classic CAN counts 0..8 and the codes
  // above eight all mean eight; CAN FD reads 9..15 as a table - 12, 16, 20,
  // 24, 32, 48, 64 - so on an FD frame the code is NOT the byte count.
  uint8_t  len;
  bool     ext;      // 29-bit identifier
  bool     rtr;      // remote request: a frame that asks rather than tells
  bool     fd;       // CAN FD, and decoded as such
  bool     brs;      // ...whose data phase ran at the faster of the two rates
  // The transmitter is error passive: it is still on the bus and its frames
  // still arrive, but it has been failing long enough to have lost the right
  // to flag anyone else's errors. Worth seeing before it goes bus-off.
  bool     esi;
  bool     crc_ok;
  bool     ack;      // somebody on the bus acknowledged it
  bool     cut;      // the record ended inside the frame's end-of-frame field
  int      first;    // index of the frame's first byte in LogicResult.bytes
  int      count;
} CanFrame;

typedef struct
{
  int      rate;      // arbitration bit/s
  // The data phase's bit rate, which is the whole point of CAN FD. Equal to
  // `rate` when no frame in the record switched.
  int      data_rate;
  int      frames;
  int      fd;        // how many of them were CAN FD
  int      crc_ok;    // how many of them checked out
  // An FD frame went past that this decoder could NOT read: no data rate it
  // tried made the CRC agree, or the frame is a non-ISO one (no stuff count,
  // and a CRC register that starts at zero rather than at one).
  bool     fd_seen;
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

// EV1527 and the family with it. Twenty-four bits: twenty of address burned
// in at the factory, four of buttons. No checksum anywhere in the protocol -
// a held button repeating the frame is all the corroboration there is.
#define EV1527_MAX_FRAMES  8

typedef struct
{
  uint32_t addr;    // 20 bits
  uint8_t  key;     // 4 bits: which buttons are down
} Ev1527Frame;

typedef struct
{
  int      t_ns;    // the unit time, read off the frame's own sync ratio
  int      frames;
  bool     agree;   // every frame in the record said the same thing
  Ev1527Frame frame[EV1527_MAX_FRAMES];
} Ev1527Analysis;

// DShot. Sixteen bits and one number: eleven of throttle (or a command), one
// asking for telemetry, and four of CRC over the twelve.
#define DSHOT_MAX_FRAMES  8

typedef struct
{
  uint16_t value;   // 0 disarms, 1..47 are commands, 48..2047 is the throttle
  uint8_t  crc;     // as received
  bool     telem;   // the frame asked the ESC for a telemetry packet
  bool     crc_ok;
} DshotFrame;

typedef struct
{
  int   rate;       // bit/s, measured over the whole frame
  int   frames;
  int   crc_ok;
  // Bidirectional DShot: the line is inverted and so is the CRC. Recognised
  // by which CRC agreed; the ESC's eRPM answer is GCR-encoded on the same
  // wire and is not decoded here.
  bool  bidir;
  DshotFrame frame[DSHOT_MAX_FRAMES];
} DshotAnalysis;

// What one probe on a data line was able to work out about a clocked bus it
// cannot see the clock of. Every field here is an assumption or the evidence
// for one, which is the only honest way to present the result.
// What a reconstructed byte turned out to be. Only ever filled where a PAUSE
// said a transaction started here - see spi_decode.c for why a command name
// anywhere else would be a guess wearing a fact's clothes.
typedef enum
{
  SPI_R_NONE = 0,
  SPI_R_CMD,      // the first byte of a transaction, and a known one
  SPI_R_ADDR,     // one of the three address bytes that command takes
} spi_role_t;

// The addresses that were assembled, one per transaction that carried one.
// Kept beside the roles rather than per byte: three bytes make one number and
// there are only ever a handful of transactions in a record.
#define SPI_MAX_TX  8

typedef struct
{
  int  clock_hz;    // the bit rate the reconstruction used
  int  phase;       // 0..7: where the byte boundary was put
  int  bits;        // how many bits came out of the record
  int  anchors;     // gaps long enough to be byte boundaries
  int  cmds;        // known commands found at transaction starts
  bool told;        // the rate was measured on SCK, not guessed from the data
  bool msb_first;
  bool pinned;      // ...and a gap agreed with the phase, so it is not a guess
  int  txs;
  uint8_t  tx_last[SPI_MAX_TX];   // byte index the address finishes on
  uint32_t tx_addr[SPI_MAX_TX];
  uint8_t role[LOGIC_MAX_BYTES];
} SpiAnalysis;

// Manchester is a line CODE and not a protocol, so a frame is a run of bits
// and not a run of bytes: RC5 sends fourteen of them, DALI nineteen. The bit
// count travels with the value, because two bytes of a fourteen-bit frame are
// a packing artefact and nothing else.
#define MAN_MAX_FRAMES  6

typedef struct
{
  uint8_t  bits;    // how many the frame held
  uint8_t  first;   // its first byte in LogicResult.bytes
  uint8_t  count;   // ...and how many they packed into
  // Bits with no transition in their middle. That is an ENCODING violation
  // and not a reason to stop: the frame is Manchester, one of its bits is
  // broken, and those are different statements. The bit index of the first
  // one is kept because "somewhere in here" is not an answer.
  uint8_t  viol;
  uint8_t  viol_bit;
  bool     inv;     // which convention THIS frame was read in
  uint32_t value;   // the frame right-aligned, for the frames that fit
} ManFrame;

typedef struct
{
  int      rate;    // bit/s
  int      frames;
  int      bits;
  bool     told;    // the rate was picked, not guessed off the record
  bool     inverted;// a rising mid-bit edge was read as a zero
  // Some frame held runs of BOTH lengths, which is the signature. Without it
  // the record is a square wave and every reading of one fits it.
  bool     sure;
  // The convention was inferred from the preamble rather than told: a frame
  // whose first byte reads 0xAA where a protocol's preamble is 0x55 was read
  // the wrong way round. That is an assumption about the PROTOCOL and not a
  // fact about the waveform, so it is reported as one.
  bool     auto_inv;
  int      viol;    // encoding violations over the whole record
  ManFrame frame[MAN_MAX_FRAMES];
} ManAnalysis;

// One union for every decoder's analysis, because only one of them is ever
// live. The cascade stops at the first decoder that answers, so that decoder
// is the LAST one to have run and its analysis is fetched immediately
// afterwards; everything the ones before it wrote belongs to a reading nobody
// is going to ask about. Ten separate copies held a kilobyte and a half of a
// part that has sixty-four of them, for nine analyses of records that were
// turned down.
// The Philips bi-phase remotes. RC5 is fourteen bits at 1.778 ms; RC6 puts a
// leader in front of twenty-one at 0.889 and makes its toggle bit twice as
// wide as the rest, which is exactly what a generic Manchester reader cannot
// follow.
typedef enum
{
  RC5_KIND_RC5 = 0,
  RC5_KIND_RC5X,   // the second start bit carries the seventh command bit
  RC5_KIND_RC6,
} rc5_kind_t;

typedef struct
{
  uint8_t kind;
  uint8_t addr;
  uint8_t cmd;
  uint8_t mode;    // RC6 only
  uint8_t bits;
  bool    toggle;  // flipped on a fresh press, held through a repeat
  bool    field;   // RC5's second start bit; 0 makes it RC5X
  int     bit_ns;
} Rc5Analysis;

// DALI. Which ballast, and what it was told - the address byte is not a
// number but a shape, and its bottom bit decides what the OTHER byte means.
typedef enum
{
  DALI_SHORT = 0,   // one ballast out of 64
  DALI_GROUP,       // one group out of 16
  DALI_BROADCAST,
  DALI_SPECIAL,     // the commissioning set, which lives in the address byte
  DALI_BACKWARD,    // a slave answering a query
} dali_kind_t;

#define DALI_MAX_FRAMES  4

typedef struct
{
  uint8_t bits;     // 17 forward, 9 backward
  uint8_t kind;
  uint8_t addr;     // the address byte as it went past
  uint8_t data;
  uint8_t target;   // the short address or the group number
  uint8_t first;
  uint8_t count;
  bool    cmd;      // the selector: the data byte is a command, not a level
} DaliFrame;

typedef struct
{
  int       rate;
  int       frames;
  int       bits;
  DaliFrame frame[DALI_MAX_FRAMES];
} DaliAnalysis;

// WS2812 and the strips that copy it. Three bytes to a pixel, and the wire
// sends them G-R-B: a reader looking at "11 22 33" is looking at green 0x11,
// red 0x22 and blue 0x33, which is the one order nobody writes a colour in.
//
// A gap long enough to latch the strip ends an update and the next one
// addresses pixel 0 again, so which pixel a byte belongs to is a question
// about its FRAME and not about its place in the record.
#define WS_MAX_PIXELS  (LOGIC_MAX_BYTES / 3)

typedef struct
{
  int      bit_ns;
  int      count;                  // bytes behind this reading
  int      pixels;                 // whole ones over the record
  int      frames;                 // updates: one per latching gap, plus the last
  // A frame whose byte count was not a multiple of three. The strip ignores
  // the remainder; here it means the record began or ended inside a pixel,
  // which is worth saying rather than padding over.
  bool     partial;
  uint8_t  rgb[WS_MAX_PIXELS][3];  // put back into R-G-B order
  uint8_t  pix[LOGIC_MAX_BYTES];   // which pixel; 0xFF for one cut short
  uint8_t  base[LOGIC_MAX_BYTES];  // first byte of this byte's frame
} WsAnalysis;

// KNX TP1. Not a line code question but an addressing one: what a person
// wants off this bus is who told whom, and both addresses are packed fields
// rather than numbers.
typedef struct
{
  int      rate;
  int      octets;
  int      parity_err;   // characters turned down for their parity bit
  uint16_t src;
  uint16_t dst;
  uint8_t  ctrl;
  uint8_t  len;
  bool     group;        // the destination is a group, not a device
  bool     repeat;       // the sender has said this before
  bool     fcs_ok;
} KnxAnalysis;

// SWO / ITM. What comes off a Cortex-M trace pin is a stream of packets, and
// which packet a byte heads decides everything about it - a stimulus port
// carries a character, a DWT discriminator carries a program counter or an
// exception number, and both are the same eight bits on the wire.
typedef enum
{
  ITM_NONE = 0,
  ITM_SYNC,       // 47 zero bits and a one: a trace session opening
  ITM_OVERFLOW,   // the macrocell dropped trace; what follows has holes in it
  ITM_LTS,        // local timestamp, short or continuation-coded
  ITM_GTS,        // global timestamp, lower or upper half
  ITM_EXT,        // extension: a stimulus port page, usually
  ITM_SW,         // instrumentation: a write to stimulus port N. printf.
  ITM_HW,         // the DWT: event counters, exceptions, PC samples, watches
} itm_kind_t;

// Sixty-four bytes of record hold at most this many packets in practice: a
// printf stream is two bytes a packet, so thirty-two covers a full record of
// the densest traffic there is.
#define SWO_MAX_PKTS  32

typedef struct
{
  uint8_t  kind;    // itm_kind_t
  uint8_t  id;      // stimulus port, DWT discriminator, or a format selector
  uint8_t  first;   // its header byte in LogicResult.bytes
  uint8_t  count;   // header and payload together
  uint32_t value;   // the payload, assembled
} ItmPacket;

typedef struct
{
  int      rate;    // bit/s on the wire
  int      packets;
  int      sw;      // instrumentation packets: the printf
  int      hw;      // DWT packets: exceptions, PC samples, watchpoints
  int      sync;
  int      ts;      // timestamps, local and global
  // Bytes that could head no packet at all. The single most useful number
  // here: the grammar is what tells SWO from a serial console, and this
  // counts how often the record failed it.
  int      bad;
  uint32_t portmask;    // which stimulus ports were written to
  // Where the first WHOLE packet starts. A record can open in the middle of
  // one, and the bytes in front of it are not a reading of anything.
  uint8_t  cut;
  int8_t   text_port;   // the port carrying the text, or -1
  uint8_t  pidx[LOGIC_MAX_BYTES];   // which packet each byte belongs to
  ItmPacket pkt[SWO_MAX_PKTS];
  char     text[20];    // the printf, reassembled, for the panel header
} SwoAnalysis;

// SWD. A transaction is a request byte and, when the target acknowledged it,
// four bytes of register - and what the register IS depends on a SELECT write
// that may have gone past several transactions earlier.
typedef enum
{
  SWD_R_NONE = 0,
  SWD_R_REQ,
  SWD_R_DATA,
  SWD_R_SWITCH,   // the sixteen bits that take a port out of JTAG into SWD
} swd_role_t;

#define SWD_MAX_TX  12

typedef struct
{
  uint8_t  req;       // the eight wire bits, start in bit 0: 0xA5 is DPIDR
  uint8_t  ack;       // 1 OK, 2 WAIT, 4 FAULT
  uint8_t  reg;       // A[3:2]
  uint8_t  first;     // its first byte in LogicResult.bytes
  uint8_t  count;
  bool     ap;        // AP rather than DP
  bool     write;
  bool     has_data;  // a data phase followed; WAIT and FAULT have none
  bool     par_ok;    // the parity over the thirty-two data bits
  uint32_t data;
} SwdTx;

typedef struct
{
  int      clock_hz;
  int      txs;
  int      resets;    // line resets: fifty clocks of ones, a debugger arriving
  int      switches;  // JTAG-to-SWD sequences
  int      waits;
  int      faults;
  int      par_err;
  // Transactions the target acknowledged AND whose data parity agreed. The
  // number that matters: a WAIT costs a would-be impostor only four of the
  // six checks, and four is not enough to be rare.
  int      ok_txs;
  // Transactions with something OTHER than an idle line in front of them.
  // Between two packets a host clocks idle cycles with SWDIO held low, so a
  // gap carrying traffic means these two "transactions" are slices out of
  // somebody else's signal.
  int      gap_bad;
  bool     told;      // the clock was measured on SWCLK, not guessed
  // What lets auto mode take the record. See swd_decode() - the short version
  // is that a debugger attaching is unmistakable and two clean transactions
  // back to back on an otherwise idle line is nearly so.
  bool     sure;
  uint32_t select;    // the last DP SELECT written: which AP bank is current
  uint8_t  role[LOGIC_MAX_BYTES];
  uint8_t  tidx[LOGIC_MAX_BYTES];
  SwdTx    tx[SWD_MAX_TX];
} SwdAnalysis;

typedef union
{
  OwAnalysis     ow;
  CanAnalysis    can;
  DhtAnalysis    dht;
  SentAnalysis   sent;
  MidiAnalysis   midi;
  LinAnalysis    lin;
  Ev1527Analysis ev;
  DshotAnalysis  dshot;
  SpiAnalysis    spi;
  ManAnalysis    man;
  Rc5Analysis    rc5;
  DaliAnalysis   dali;
  KnxAnalysis    knx;
  WsAnalysis     ws;
  SwoAnalysis    swo;
  SwdAnalysis    swd;
} LogicAnalysis;

extern LogicAnalysis g_logic_analysis;

// Working memory (~4.5 KB) provided by the caller: the firmware puts it in
// spare main SRAM, host tests use a static instance
typedef struct
{
  int      len[LOGIC_MAX_RUNS];
  int      pos[LOGIC_MAX_RUNS];
  uint8_t  lvl[LOGIC_MAX_RUNS];

  // Run-split cache. Every decoder in the auto cascade thresholds the SAME
  // record, and that is two full passes over 24K samples each time; the
  // cascade pays for the split once instead of fifteen times.
  //
  // It is live ONLY while the dispatcher is inside one call, which `cascade`
  // marks. That is not a shortcut, it is the only window in which reuse is
  // sound: the record behind an unchanged (data, size, offset) is refilled by
  // the DMA between calls. A decoder called on its own - which every one of
  // them supports, and which is how they are all tested - therefore always
  // splits afresh.
  //
  // There WAS a content fingerprint here instead, sampling the record at 64
  // points and later 256. It could not work, and not by a narrow margin: the
  // records this instrument takes are sparse - a KNX telegram is a handful of
  // 35 us pulses on a flat line - so a stride that lands between the pulses
  // reads two entirely unrelated captures as the same. That is not a
  // theoretical collision; an infrared frame came back as three bytes of a
  // lighting bus, and a KNX telegram with a deliberately broken check octet
  // came back with the previous telegram's good one.
  const uint8_t *cache_data;
  int      cache_size;
  int      cache_offset;
  int      cache_runs;
  int      cache_mid;
  uint32_t cascade;      // set while the dispatcher is inside one call
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
// WS2812/WS2812B. Three bytes to a pixel, so the group is the pixel and the
// value written across it is its colour - which the hex dump cannot show,
// the wire order being G-R-B.
int ws2812_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const WsAnalysis *ws2812_analysis(void);
void ws2812_byte_label(const WsAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void ws2812_field_label(const WsAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void ws2812_group_at(const WsAnalysis *a, int idx, int *start, int *len);

int nec_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
// Servo bytes are the pulse width in tens of microseconds, so 150 is 1.50 ms
int servo_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);

// CAN 2.0A/2.0B and CAN FD. Reports only what a frame's own CRC confirms, so
// it is safe to run ahead of the generic decoders. Same accessors as 1-Wire:
// the frame-level read of the last record, and the per-byte label that comes
// out of it.
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

// EV1527. Found from its 1:31 sync ratio and confirmed by 24 bits of
// constant period; three result bytes per frame, MSB first, so the hex dump
// reads the address and the buttons straight off.
int ev1527_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const Ev1527Analysis *ev1527_analysis(void);
void ev1527_byte_label(const Ev1527Analysis *a, int idx, uint8_t v,
    char *buf, int size);
void ev1527_group_at(const Ev1527Analysis *a, int idx, int *start, int *len);
void ev1527_field_label(const Ev1527Analysis *a, int idx, uint8_t v,
    char *buf, int size);

// DShot. Every frame is confirmed by its own CRC-4 before it is reported, so
// this one is safe ahead of WS2812 - which is the signal it has to be told
// apart from, both being constant-period and duty-encoded at about the same
// rate. Two result bytes per frame.
int dshot_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const DshotAnalysis *dshot_analysis(void);
void dshot_byte_label(const DshotAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void dshot_group_at(const DshotAnalysis *a, int idx, int *start, int *len);
void dshot_field_label(const DshotAnalysis *a, int idx, uint8_t v,
    char *buf, int size);

// SPI with one probe on the data line. Reports itself ambiguous every time,
// so the dispatcher never picks it in auto mode - a data line without its
// clock is not distinguishable from any other digital signal, and this
// decoder exists to be asked for by name.
int spi_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const SpiAnalysis *spi_analysis(void);

// The clock measured on SCK before the probe moved to the data line, in Hz.
// 0 (the default) works the bit time out of the data itself, which drifts on
// long runs of identical bits and cannot tell a rate from half of it.
void spi_decode_set_clock(int hz);

// 0 = try both bit orders and score them, 1 = MSB first, 2 = LSB first
void spi_decode_set_order(int order);

// What a byte turned out to be, where anything did. Empty for a byte that is
// just a byte, which is most of them - the caller falls back to showing it as
// a character, the way it does for a serial line.
void spi_byte_label(const SpiAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void spi_field_label(const SpiAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void spi_group_at(const SpiAnalysis *a, int idx, int *start, int *len);

// Manchester / bi-phase, the line code under RC5, DALI, EM4100 tags and most
// of the 433 MHz weather sensors. Every bit has a transition in its middle,
// so the line holds a level for half a bit or a whole one and never longer -
// which is the structural check this decoder makes before reading anything.
int manchester_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const ManAnalysis *manchester_analysis(void);
void manchester_byte_label(const ManAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void manchester_field_label(const ManAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void manchester_group_at(const ManAnalysis *a, int idx, int *start, int *len);

// The bit rate, in bit/s; 0 works it out of the record - which cannot tell a
// rate from half of it when the data never repeats a bit
void manchester_decode_set_rate(int bps);

// Which convention: 0 = a rising mid-bit edge is a one (G.E. Thomas, RC5),
// 1 = it is a zero (IEEE 802.3, DALI), 2 = work it out from the preamble.
//
// Nothing in the WAVEFORM decides this - the two conventions are exact
// inverses and 0x55 and 0xAA are equally good data. What the third option
// leans on is the protocol: a preamble is there to be recognised, it is
// 0x55 in nearly everything that has one, and a frame that starts 0xAA was
// therefore read the wrong way round. An assumption, and labelled as one.
#define MAN_POL_AUTO  2

void manchester_decode_set_polarity(int inverted);

// RC5, RC5X and RC6. Bi-phase like the generic decoder, but read as MESSAGES:
// address, command and the toggle bit that says a key was pressed again
// rather than held.
int rc5_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const Rc5Analysis *rc5_analysis(void);
void rc5_byte_label(const Rc5Analysis *a, int idx, uint8_t v,
    char *buf, int size);
void rc5_field_label(const Rc5Analysis *a, int idx, uint8_t v,
    char *buf, int size);
void rc5_group_at(const Rc5Analysis *a, int idx, int *start, int *len);

// DALI. Manchester at 1200, read as lighting commands rather than as bits:
// which ballast, and whether the data byte is an arc power level or an
// instruction - which its address byte's bottom bit decides.
int dali_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const DaliAnalysis *dali_analysis(void);
void dali_frame_text(const DaliFrame *f, char *buf, int size);
void dali_byte_label(const DaliAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void dali_field_label(const DaliAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void dali_group_at(const DaliAnalysis *a, int idx, int *start, int *len);

// KNX TP1 at 9600. Pulse-presence coded and not bi-phase: a zero is a 35 us
// pulse at the start of its bit and a one is nothing at all, so there is no
// mid-bit transition to step to and the slots are counted from the start bit.
int knx_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const KnxAnalysis *knx_analysis(void);
void knx_addr_text(uint16_t a, bool group, char *buf, int size);
void knx_byte_label(const KnxAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void knx_field_label(const KnxAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void knx_group_at(const KnxAnalysis *a, int idx, int *start, int *len);

// SWO / ITM: a Cortex-M's trace pin. An ordinary 8N1 line electrically, so
// what identifies it is the ITM packet grammar the bytes have to satisfy -
// which is strict enough that a serial console fails it within a few bytes.
// The rate is measured off the record: SWO comes out of an integer divider
// on the trace clock and is not on a baud table at all.
int swo_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const SwoAnalysis *swo_analysis(void);
void swo_byte_label(const SwoAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void swo_field_label(const SwoAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void swo_group_at(const SwoAnalysis *a, int idx, int *start, int *len);

// SWD with one probe on SWDIO. The clock is on the other wire and cannot be
// seen - but a transaction carries six checks of its own, so one that reads
// clean confirms the bit time along with itself. Anchored on each packet's
// own start bit, which is what makes a gated probe clock harmless.
int swd_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
const SwdAnalysis *swd_analysis(void);

// The clock measured on SWCLK before the probe moved to SWDIO, in Hz. 0 (the
// default) works the bit time out of the record, which needs a single-clock
// run somewhere in it to have anything to measure.
void swd_decode_set_clock(int hz);

void swd_byte_label(const SwdAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void swd_field_label(const SwdAnalysis *a, int idx, uint8_t v,
    char *buf, int size);
void swd_group_at(const SwdAnalysis *a, int idx, int *start, int *len);

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
