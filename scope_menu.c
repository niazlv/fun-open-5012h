/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Oscilloscope application menu
 *
 * The scope's section of the system menu plus its help pages. Everything in
 * here is scope specific, which is exactly why it does not live in
 * system_menu.c: the system menu shows this table only while the scope is the
 * running application.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "config.h"
#include "common.h"
#include "utils.h"
#include "capture.h"
#include "ui.h"
#include "menu_widget.h"
#include "logic_decode.h" // the protocol labels are indexed by proto_t
#include "scope.h"

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Trigger settings: values live in config, hardware follows immediately.
// The scope screen underneath repaints in full when the menu closes.
//-----------------------------------------------------------------------------
static void trigger_mode_changed(void)
{
  capture_set_trigger_mode(config.trigger_mode);
  capture_start(); // arms SINGLE, resumes after a stopped capture
}

//-----------------------------------------------------------------------------
static void trigger_edge_changed(void)
{
  capture_set_trigger_edge(config.trigger_edge);
}

//-----------------------------------------------------------------------------
static void trigger_level_apply(int value)
{
  (void)value; // the widget already stored it in config.trigger_level
  scope_apply_trigger_level();
}

//-----------------------------------------------------------------------------
static void action_trigger_50p(const void *arg)
{
  (void)arg;
  scope_trigger_50_percent();
  ui_request_redraw(); // the Level row shows the new value
}

/*- Variables ---------------------------------------------------------------*/
static const char *const g_trigger_mode_labels[] = { "Auto", "Normal", "Single" };
static const char *const g_trigger_edge_labels[] = { "Rise", "Fall", "Both" };

static const char *const g_measure_panel_labels[] = { "On", "Off" };

// What a status-line slot shows. Indexed by the MEASURE_* metric numbers, so
// the order here is the order in config.h. "Off" hands the space back to the
// trigger readouts, which is what an empty slot means.
static const char *const g_measure_slot_labels[] =
{
  "Off", "Vpp", "Frequency", "Duty", "Vrms", "Vavg", "Type", "THD", "Jitter",
};

// The slot choice cycles through MEASURE_COUNT entries, so this array MUST
// have one label per enum value - a missing one is an out-of-bounds read,
// not a cosmetic gap
_Static_assert(ARRAY_SIZE(g_measure_slot_labels) == MEASURE_COUNT,
    "one label per MEASURE_* value");

// In proto_t order, so the index the config stores IS the protocol number
static const char *const g_decoder_proto_labels[] =
{
  "Auto", "UART", "1-Wire", "WS2812", "NEC IR", "Raw", "Servo PWM", "CAN",
  "DHT11/22", "SENT", "MIDI", "LIN", "EV1527", "DShot", "SPI! no clk", "Manchester", "RC5/RC6 IR", "DALI", "KNX TP1",
  "SWO / ITM", "SWD (SWDIO)",
};

_Static_assert(ARRAY_SIZE(g_decoder_proto_labels) == PROTO_COUNT,
    "one label per proto_t value, in its order");

// On is index 0 so a config saved before the setting existed reads as on
static const char *const g_decoder_fit_labels[] = { "On", "Off" };
static const char *const g_decoder_bits_labels[] = { "On", "Off" };

// Auto is index 0, so a config saved before the setting existed scores both
static const char *const g_spi_order_labels[] = { "Auto", "MSB", "LSB" };

// Which way round a Manchester bit reads. Nothing in the waveform says, so
// this is the protocol riding on the code and has to be told.
static const char *const g_man_pol_labels[] = { "Rise=1", "Rise=0", "Auto 55" };

//-----------------------------------------------------------------------------
// Half of reading an SPI bus with one probe: write down the clock while the
// probe is still on SCK. Done by the scope on its next tick, because this
// menu is a screen over it.
static void action_spi_clock(const void *arg)
{
  (void)arg;
  scope_spi_clock_capture();
  menu_close_popups();
}

//-----------------------------------------------------------------------------
// ...and the same first half for SWD: the probe is on SWCLK now, and SWDIO is
// where it goes next
static void action_swd_clock(const void *arg)
{
  (void)arg;
  scope_swd_clock_capture();
  menu_close_popups();
}

//-----------------------------------------------------------------------------
// The scope does the work on its next tick, once this menu is off the screen
static void action_decode_catch(const void *arg)
{
  (void)arg;
  scope_decode_catch_start();
  menu_close_popups();
}

//-----------------------------------------------------------------------------
// Help pages
//-----------------------------------------------------------------------------
static const char *const g_scope_help_lines[] =
{
  "MODE        - Measurements on/off; which",
  "  ones and where: Menu > Measurements",
  "SHIFT+MODE  - FFT spectrum view",
  "  see the Spectrum (FFT) page for details",
  "SHIFT+EDGE  - Decoder: UART/1-Wire/",
  "  WS2812/NEC/raw; TRIG_UP/DN jump bytes",
  "AUTO        - Auto-setup (scale/trigger)",
  "50%         - Trigger level to mid-signal",
  "SHIFT+50%   - Find the narrowest pulse in",
  "  the record (stops) and pan to it",
  "SAVE        - Cursors T1>T2>V1>V2>off;",
  "  arrows drag (SHIFT x10), line shows",
  "  dT, 1/dT, dV",
  "SHIFT+SAVE  - Trend view: 1 Hz log of",
  "  f/Vrms/duty; EDGE metric, MODE clears",
  "STOP        - Freeze: pan/zoom/measure/",
  "  decode/spectrum on the frozen record",
  "TRIG / EDGE - Trigger mode / edge",
  "TRIG_UP/DN  - Trigger level",
  "MENU        - System menu, SHIFT+MENU exits",
};

static const info_page_t g_page_scope_help =
{
  .title = "Scope Functions",
  .lines = g_scope_help_lines,
  .count = ARRAY_SIZE(g_scope_help_lines),
};

// The spectrum view has its own key map and its own trap (resolution comes
// from the timebase, not from the sample rate), so it gets its own page
static const char *const g_spectrum_help_lines[] =
{
  "SHIFT+MODE - enter/leave   MODE - panel",
  "LEFT/RIGHT - cursor   UP/DOWN - peaks",
  "TRIG_UP/DN - wider span / finer df",
  "EDGE - hold: off / Max (M) / Avg (A);",
  "  max pins whatever ever appeared, avg",
  "  sinks the noise floor ~9 dB",
  "SHIFT+L/R - timebase  SHIFT+U/D - volts",
  "",
  "The whole record is transformed, so the",
  "resolution df = 1/record time and the",
  "TIMEBASE sets it, not the sample rate.",
  "df is on the panel: mains 50/100 Hz",
  "needs 5 ms/div or slower to separate.",
  "",
  "F0 comes from the harmonic comb, not",
  "from the tallest peak. h1..hN mark its",
  "harmonics, '-' marks peaks that belong",
  "to something else: interference, noise.",
};

static const info_page_t g_page_spectrum_help =
{
  .title = "Spectrum (FFT)",
  .lines = g_spectrum_help_lines,
  .count = ARRAY_SIZE(g_spectrum_help_lines),
};

// The decoder reads the whole record, and the record is what the TIMEBASE
// makes it - which is the one thing that decides whether a message decodes
// at all, and the one thing that is not obvious from the decoder panel
static const char *const g_decoder_help_lines[] =
{
  INFO_HEAD "Keys",
  "SHIFT+EDGE - enter/leave",
  "TRIG_UP/DN - step through the bytes,",
  "  the view follows the selected one",
  "",
  INFO_HEAD "On the trace",
  "Decoded bytes are marked on the",
  "trace itself: a tick where each",
  "frame starts, and the value written",
  "in where the byte is wide enough.",
  "The panel shows 16 of them at a",
  "time and the header the total.",
  "",
  INFO_HEAD "Bit grid",
  "Zoom in far enough and a grey grid",
  "appears over the trace: a hairline",
  "at every bit boundary and, in each",
  "cell, the NUMBER of the bit that",
  "landed there - S for the start bit,",
  "0..7 for the data, P for the stop.",
  "The lines are exactly as tall as the",
  "waveform is - a two-volt square wave",
  "gets two volts of line - and they",
  "are blended, not painted, so nothing",
  "they cross is hidden.",
  "The waveform already shows the level;",
  "what it cannot show is which bit that",
  "level became. 0 marks the LSB, so",
  "the bit ORDER is on the screen too.",
  "Decoder > Bit grid turns it off.",
  "Only UART, MIDI, LIN and raw get it:",
  "those four sample a byte on an even",
  "grid, so dividing it evenly puts the",
  "lines exactly where the decoder",
  "looked. NEC, DHT, 1-Wire and SENT",
  "time every bit on its own and CAN",
  "stuffs extra ones in, so an even",
  "grid would be lines in places",
  "nothing was ever read - and a grid",
  "you cannot trust is worse than none.",
  "",
  INFO_HEAD "Set-up from the rate",
  "Given the protocol AND the baud,",
  "entering the view sets the timebase",
  "and the trigger and then hunts: the",
  "trigger goes to Normal on the edge a",
  "start bit is, one division from the",
  "left, and every record is decoded",
  "until one is the head of a message.",
  "It freezes there. Decoder > Catch a",
  "message start does it on demand;",
  "Set up from the rate turns it off.",
  "",
  "No edge means 'message start' - each",
  "bit is an edge too. What marks one",
  "is the idle BEFORE it, and that can",
  "only be checked after the record is",
  "taken. Hence hunting rather than",
  "triggering.",
  "",
  INFO_HEAD "Record and timebase",
  "The whole record is decoded, so the",
  "TIMEBASE decides what fits: a 24K",
  "record needs >=3 samples per bit at",
  "one end and the whole message inside",
  "it at the other. 45 bytes at 115200",
  "is 3.9 ms and wants 500 us/div.",
  "",
  "A message longer than the record is",
  "decoded as far as it reached: the",
  "panel says 'cut' and how long the",
  "record was. '+' means more bytes",
  "than the 64 a result holds.",
  "",
  "Most records hold nothing - a burst",
  "is a few ms out of every 200. The",
  "last frames stay up, marked 'hold',",
  "so they do not flash past.",
  "",
  INFO_HEAD "Baud, hold and stopping",
  "Baud is found by trying every rate",
  "that fits and keeping the one whose",
  "frames explain most of the record.",
  "Fix it in UART baud if the record",
  "holds too few frames to be sure.",
  "",
  "Stop on frames freezes on a match.",
  "...at message start waits for a",
  "record that caught the line idle",
  "first, so the freeze lands on the",
  "head of a message, not its middle.",
  "",
  INFO_HEAD "1-Wire: DS18x20",
  "On 1-Wire the transaction is read,",
  "not just the bytes: the family code",
  "under its CRC and the scratchpad",
  "behind READ SP name the part.",
  "DS18B20 comes out as the reading",
  "itself - 'DS18B20 +25.06C 12b' - and",
  "an intercom key as 'DS1990 key",
  "ROM ok'. Same slots, same decoder;",
  "what tells them apart is which",
  "command framed the bytes.",
  "A CRC that does not check out names",
  "nothing and says 'CRC!' instead.",
  "Every scratchpad byte is labelled,",
  "reserved ones included - a byte with",
  "nothing under it reads as one the",
  "decoder could not account for. Bytes",
  "5 and 7 hold FF and 10 on a working",
  "part, so 'RSV!' behind a good CRC is",
  "the part itself differing.",
  "Byte 4 tells the DS18S20 from the",
  "DS18B20: a config register reads",
  "1F/3F/5F/7F, the older part has none",
  "and reads FF. On it bytes 6 and 7 are",
  "COUNT REMAIN and COUNT PER C, and the",
  "reading comes out in sixteenths from",
  "them - '+25.00C ext'.",
  "",
  INFO_HEAD "CAN",
  "CAN reads either wire of the pair:",
  "recessive is wherever the line rests,",
  "so CAN_H, CAN_L and a transceiver's",
  "TX all decode. The rate is found from",
  "the standard list, 10k to 1M.",
  "Every frame is confirmed by its own",
  "CRC-15 before it is shown, and a",
  "record with none is refused - which",
  "is why CAN can run first without",
  "taking anyone else's records.",
  "'NAK' on the last byte means the",
  "frame was correct and nobody on the",
  "bus acknowledged it. CAN FD is",
  "recognised and declined: its data",
  "phase changes rate mid-frame.",
  "",
  INFO_HEAD "DHT11 / DHT22",
  "DHT11/22 send forty bits whose HIGH",
  "carries the value: 26 us is a zero,",
  "70 a one, over a 50 us low. Five",
  "bytes, the fifth their sum, and no",
  "sum means no frame.",
  "The two parts are identical on the",
  "wire and differ in the numbers: a",
  "DHT11 sends whole units and leaves",
  "the fractional bytes at zero, a DHT22",
  "sends tenths across both. Reading one",
  "as the other gives 1152 % humidity,",
  "which is what decides it. If both fit",
  "the 18 ms start pulse settles it, and",
  "if it was off-screen the header says",
  "so with a '?'.",
  "",
  INFO_HEAD "Grouped values",
  "What makes up ONE value is shown as",
  "one: three SENT nibbles, four CAN id",
  "bytes, two DHT bytes. They light",
  "together and the value is written",
  "once across them, the way UTF-8 does",
  "with a letter. The numbers above stay",
  "one per byte - a byte is what was on",
  "the wire.",
  "Where there are groups the band grows",
  "a third row, and the three go from",
  "the specific to the general as they",
  "leave the trace: the number, then",
  "what that byte is (D1 D2 D3), then",
  "the value they make (S1=394).",
  "Labels alternate in brightness - a",
  "space between two of them looks just",
  "like the space inside 'READ ROM'.",
  "The label row is narrower than the",
  "hex rows, so it scrolls on its own",
  "and marks its edges with < >: a row",
  "that just stops reads as a decoder",
  "out of names, not as a narrow panel.",
  "",
  INFO_HEAD "SENT",
  "SENT has no clock and no bits: the",
  "time from one falling edge to the",
  "next is the nibble, 12 ticks for 0",
  "and 27 for 15. The tick is not agreed",
  "in advance - each frame opens with a",
  "56-tick sync pulse, and dividing by",
  "56 gives the sensor's own unit time.",
  "Six data nibbles read as two 12-bit",
  "signals, three as one. Both of the",
  "CRCs J2716 blessed are accepted; a",
  "frame matching neither is shown only",
  "when SENT is picked by name.",
  "A nibble needs a closing edge, so a",
  "record that ends before it holds a",
  "frame with no CRC in it - the last",
  "measured nibble is data and is not",
  "labelled CRC. It reads 'cut'.",
  "SENT needs >=4 samples per tick: at",
  "3 us that is 750 ns a sample, so",
  "0.2 to 1 ms/div.",
  "",
  INFO_HEAD "MIDI",
  "MIDI is 8N1 at 31250 baud and no",
  "other rate, so the framing is a",
  "UART's and the rate is half the",
  "evidence. The other half is the",
  "grammar: bit 7 says status or data,",
  "and a status byte says how many data",
  "bytes follow it. A record with no",
  "status byte in it is not read as",
  "MIDI at all - that is what ASCII at",
  "31250 looks like.",
  "Running status leaves the status byte",
  "out and sends the data alone; those",
  "bytes are still shown as the message",
  "they make. Clock, Start and Stop are",
  "one byte and may land in the MIDDLE",
  "of a note - the note carries on in a",
  "group of its own afterwards.",
  "Notes read as names, 60 = C4, and a",
  "note on at velocity 0 reads 'Off'",
  "because that is what it is.",
  "Pitch bend is signed from centre.",
  "A whole message settles it; a record",
  "of nothing but clock bytes needs the",
  "line RESTING between them, which a",
  "square wave never does.",
  "A fault in the traffic - a message",
  "the next status cut short, a data",
  "byte with no status left in front of",
  "it - is counted and shown as 'err'.",
  "It does not hand the record to the",
  "UART decoder, which would read the",
  "same byte as an ordinary one and say",
  "nothing about it.",
  "31250 baud is 32 us a bit: 0.2 to",
  "2 ms/div, and the whole record holds",
  "about 40 messages.",
  "",
  INFO_HEAD "LIN",
  "LIN is the cheap one-wire bus in a",
  "car door. A frame is BREAK, sync,",
  "PID, 1-8 data, checksum - and after",
  "the break it is plain 8N1.",
  "The break is >=13 dominant bits and",
  "8N1 cannot send more than nine, so",
  "it identifies the bus outright.",
  "The sync byte is 0x55: nine runs of",
  "exactly one bit, eight bit times",
  "between the first falling edge and",
  "the last. That IS the rate - nothing",
  "is guessed and no baud list is used,",
  "which is why 1k to 20k all decode.",
  "The PID carries two parity bits over",
  "six identifier bits; the panel shows",
  "the identifier, with a '!' when the",
  "parity does not hold.",
  "Both checksums are tried - classic",
  "(LIN 1.3, data only) and enhanced",
  "(2.x, PID included) - and a bus that",
  "has shown one is tried that way",
  "first. Whichever agreed is named in",
  "the header, 'cls' or 'enh'.",
  "IDs 3C and 3D are diagnostic and",
  "always classic, so nothing is said",
  "there; their first three bytes read",
  "NAD, PCI and the service name.",
  "A header with nothing after it says",
  "'no resp': the master asked and no",
  "slave on the wire owns that ID,",
  "which is what unplugged looks like.",
  "19200 is 52 us a bit, 9600 is 104:",
  "0.5 to 5 ms/div.",
  "",
  INFO_HEAD "EV1527",
  "EV1527 is the 433 MHz remote: gate",
  "fobs, doorbells, PIR sensors. Sync",
  "is 1T high and 31T low; a '0' is 1T",
  "high and 3T low, a '1' the other way",
  "round. 24 bits: 20 of address burned",
  "in at the factory, 4 of buttons.",
  "The line rests LOW here - the gaps",
  "are idle and the pulses are what is",
  "sent - which is upside down from",
  "everything else on this list.",
  "T is set by a resistor and no two",
  "remotes agree, so it is measured off",
  "the 1:31 sync every frame.",
  "There is NO checksum anywhere in the",
  "protocol. What identifies it is that",
  "every bit is exactly 4T long - only",
  "the split moves - which is 24 checks",
  "of a constant nothing else holds.",
  "That is also what tells it from NEC:",
  "NEC keeps its MARK constant and puts",
  "the value in the gap, EV1527 keeps",
  "its PERIOD constant and puts it in",
  "the split. Durations cannot separate",
  "them - 31T covers 3 to 12 ms and a",
  "NEC leader is 9 - only shape can.",
  "A held button repeats the frame, and",
  "with no checksum that agreement is",
  "the only corroboration there is; the",
  "header shows it as x4. Frames that",
  "disagree are not an error, a finger",
  "moved, and each keeps its own key.",
  "T of 100..800 us: 0.5 to 5 ms/div.",
  "",
  INFO_HEAD "DShot",
  "DShot is the digital throttle from a",
  "flight controller to an ESC. Sixteen",
  "bits: 11 of throttle, 1 asking for",
  "telemetry, 4 of CRC. 0 disarms,",
  "1..47 are commands (beeps, spin",
  "direction, save), 48..2047 is the",
  "range - so 48 is 0 % and not 2 %.",
  "Every bit is the same length and the",
  "DUTY is the value: 37.5 % is a zero,",
  "75 % a one. The name is the rate, so",
  "DShot600 is a 1.67 us bit.",
  "There is no sync field: what marks a",
  "frame is the idle on both sides of",
  "it, and what confirms it is the CRC.",
  "Nothing is shown without one - a",
  "throttle read wrongly is a motor",
  "told the wrong thing.",
  "Bidirectional DShot inverts the line",
  "AND the CRC; both are spotted and",
  "the header says 'bd'. The ESC's eRPM",
  "answer is GCR-coded and not read.",
  "The signal it must not be confused",
  "with is WS2812, not an IR remote:",
  "both hold a constant period and put",
  "the value in the duty, at nearly the",
  "same rate. The duties differ (28/56",
  "against 37.5/75), a DShot frame is",
  "16 bits with idle at both ends where",
  "a strip just carries on, and a black",
  "strip sending all zeros is settled",
  "by that boundary and nothing else.",
  "DShot600 needs >=8 samples a bit:",
  "1 to 20 us/div.",
  "",
  INFO_HEAD "SPI! (no clock)",
  "SPI! is NOT an SPI decode and does",
  "not pretend to be - the '!' is part",
  "of its name on the panel, every",
  "time. SPI is a",
  "clocked bus; with one probe on the",
  "data line the timing is gone, and no",
  "reading of these samples brings it",
  "back. What this does is rebuild the",
  "bit stream on stated assumptions:",
  "the master clocks at a fixed rate,",
  "the line only changes on bit",
  "boundaries, and the byte boundary is",
  "one of eight guesses times two bit",
  "orders. It is reported ambiguous",
  "every single time, so auto mode",
  "never picks it - ask for it by name.",
  "",
  "Use it in two passes with one probe:",
  "  1. probe on SCK, Decoder > 'SPI",
  "     clock = measured' writes the",
  "     frequency down;",
  "  2. probe on MOSI and decode.",
  "Then the bit time is a measurement",
  "and not a guess. Guessed, it can",
  "only ever be the shortest run in the",
  "record - a stream with no single-bit",
  "run in it reports a multiple of the",
  "truth and cannot know. The header",
  "marks a guessed rate with '~'.",
  "",
  "The byte boundary: a master that",
  "pauses leaves gaps at the RESTING",
  "level, and the byte grid RESTARTS at",
  "each one - a pause is not a whole",
  "number of bytes, so one phase for",
  "the whole record would shift every",
  "transaction after the first.",
  "Restarting costs nothing when the",
  "run was really data: data sits on",
  "the grid already. The header says",
  "'g' when pauses were found, '?' when",
  "there were none and all 16 readings",
  "were scored on filler, text and",
  "repeats - a score is not a",
  "measurement.",
  "",
  "Commands: at a byte a PAUSE says a",
  "transaction began, the opcode is",
  "checked against the 25-series flash",
  "set - READ, PAGE PROG, JEDEC ID,",
  "ERASE 4K and so on - and the three",
  "address bytes after it are grouped",
  "and shown as one number. Nothing is",
  "named anywhere else: with no pause",
  "the phase is already a preference,",
  "and hanging 'READ' off it would make",
  "a reading look confirmed by the very",
  "thing it was picked for.",
  "SD cards are deliberately not in the",
  "table: their commands are 0x40|n,",
  "which is printable ASCII, and that",
  "would name half of any text payload",
  "a command.",
  "",
  "A byte whose top bit matches the",
  "resting level merges into the pause",
  "in front of it, and where the pause",
  "ended is then not observable at all.",
  "Same for the very first bit of the",
  "record. Those bytes are lost, and no",
  "decoder recovers them.",
  "The bit grid is worth turning on",
  "here above all: the lines ARE the",
  "assumed clock edges. If they do not",
  "sit on the signal's own edges, the",
  "rate is wrong.",
  "",
  INFO_HEAD "Manchester",
  "Manchester is a line CODE, not a",
  "protocol - the brick under RC5,",
  "DALI, EM4100 tags and most 433 MHz",
  "weather sensors. Every bit has a",
  "transition in its MIDDLE; there is",
  "one on the boundary too only when",
  "two adjacent bits are the same. So",
  "the line holds a level for half a",
  "bit or a whole one and never longer,",
  "and that is the check made before a",
  "single bit is read - it is what",
  "keeps NEC's 1:3 out.",
  "It is self-clocking, and so is this:",
  "it steps edge to edge, not along a",
  "grid, so a drifting transmitter",
  "still reads to the end of a frame.",
  "",
  "Two things it cannot know, and does",
  "not guess:",
  "  the RATE, when the data never",
  "  repeats a bit - then there is no",
  "  half-bit run anywhere and the",
  "  estimate lands at twice the truth.",
  "  Decoder > Manchester rate closes",
  "  that. A guessed rate shows '~'.",
  "  WHICH WAY ROUND a bit reads.",
  "  Thomas says a rising mid-bit edge",
  "  is a one, IEEE 802.3 says zero -",
  "  exact inverses, RC5 uses one and",
  "  DALI the other. Decoder >",
  "  Manchester bit picks it; 'inv' on",
  "  the header says which was used.",
  "  'Auto 55' leans on the PROTOCOL",
  "  instead: a preamble is there to be",
  "  recognised and is 0x55 in nearly",
  "  everything, so a frame reading",
  "  0xAA was read backwards and gets",
  "  flipped. Per FRAME, not per",
  "  record - a capture can hold both",
  "  conventions one after the other,",
  "  and a flipped frame is marked 'i'",
  "  on the trace. An assumption about",
  "  the protocol, and 'inv?' says so.",
  "",
  "A bit with NO transition in its",
  "middle is an encoding violation. The",
  "frame still comes back - the reader",
  "keeps the bit PHASE instead of",
  "chasing edges, so it does not slip",
  "half a bit and turn the rest into",
  "porridge - and the header names the",
  "bit: '!b15'. Errors count it.",
  "A record whose only frame is broken",
  "is shown but not claimed: one broken",
  "frame looks the same as something",
  "else fitting Manchester badly, so",
  "the cascade waits for a clean one.",
  "",
  "A frame is a BIT count, not a byte",
  "count - RC5 sends 14, DALI 19 - so",
  "the panel writes '14b 300C' across",
  "the bytes it packed into and the",
  "field row gives each byte's bit",
  "range.",
  "Nothing is claimed unless a frame",
  "held runs of BOTH lengths. All one",
  "length is a square wave, which is",
  "also 0x55 out of a UART and a clock:",
  "same samples, no way to choose. It",
  "still decodes when picked by name.",
  "",
  INFO_HEAD "RC5 / RC6",
  "RC5/RC6 IR reads the Philips remotes",
  "as MESSAGES, not as bits: address,",
  "command, and the TOGGLE bit, which",
  "is the one thing that tells a fresh",
  "press from a key held down.",
  "RC5 is 14 bits at 1.778 ms with no",
  "leader at all - the frame is found",
  "from the first carrier after a long",
  "silence, which is the middle of its",
  "start bit. RC5X is the same frame",
  "with the second start bit inverted:",
  "it is the SEVENTH command bit, so a",
  "six-bit field carries 128 commands.",
  "Reading it as a start bit reports",
  "the wrong key.",
  "RC6 adds a 2.666 ms leader and makes",
  "its toggle bit TWICE as wide as the",
  "rest - which the generic Manchester",
  "reader cannot follow, because there",
  "its runs are two halves and four.",
  "",
  "Their neighbours are NOT bi-phase:",
  "Sony SIRC, Samsung and RCA are pulse",
  "coded like NEC, and the 433 MHz gate",
  "remotes (Nice, Came, FAAC) are pulse",
  "coded like EV1527. Different family,",
  "different decoder.",
  "1.778 ms a bit: 5 to 20 ms/div.",
  "",
  INFO_HEAD "DALI",
  "DALI is Manchester at 1200 read as",
  "LIGHTING: which ballast, and what it",
  "was told. Forward frames are 17 bits",
  "(start, address, data), the backward",
  "answer 9. The stop bits are just the",
  "bus left alone, so a frame ends",
  "where the transitions do.",
  "The address byte is a shape, not a",
  "number: 0AAAAAAS is one ballast,",
  "100AAAAS a group, 1111111S everyone,",
  "101CCCC1 the commissioning set.",
  "Its bottom bit decides what the",
  "OTHER byte means - clear and it is a",
  "level, set and it is a command. The",
  "same 0x00 is 'off' one way and",
  "'level zero' the other.",
  "833 us a bit: 2 to 10 ms/div.",
  "",
  INFO_HEAD "KNX TP1",
  "KNX TP1 is NOT Manchester, whatever",
  "it gets listed with. 9600, and the",
  "code is pulse PRESENCE: a zero is a",
  "35 us pulse at the START of its bit,",
  "a one is nothing at all. There is no",
  "mid-bit transition to step to, so",
  "the slots are counted from the start",
  "bit - the opposite of self-clocking.",
  "A character is 13 bit times: start,",
  "8 data LSB first, EVEN parity, stop,",
  "and 2 of gap. The parity is checked",
  "per character and the telegram ends",
  "in a check octet - the complement of",
  "the XOR of everything before it.",
  "Addresses are packed fields, not",
  "numbers: a source is 4/4/8 and reads",
  "1.1.5, a group destination is 5/3/8",
  "and reads 1/2/3. The top bit of the",
  "length octet says which.",
  "104 us a bit: 0.5 to 5 ms/div.",
  "",
  INFO_HEAD "1-Wire: families",
  "1-Wire: above the ROM every family",
  "invents its OWN opcodes and they",
  "collide. 0xF5 reads two pins on a",
  "DS2413, eight channels on a DS2408",
  "and memory on a DS2406; 0xF0 is READ",
  "MEMORY on an EEPROM and READ PIO on",
  "a switch. So the family code decides",
  "the name - it came past in the ROM",
  "under a CRC, and it is the one thing",
  "on this bus that says WHAT answered.",
  "With no ROM the DS18x20 set is still",
  "offered, with a '?' on it, because",
  "that is what the bus is mostly for.",
  "A part the table CAN name but whose",
  "opcodes it does not have gets a",
  "blank, not a guess: the family code",
  "has already ruled the DS18x20 set",
  "out. 55 families are listed.",
  "An unknown family still gives up its",
  "ID - the transaction is complete and",
  "readable whatever the table knows.",
  "",
  "A key TOUCHED to a reader reads as",
  "an event, not a part: reset, READ",
  "ROM, eight good bytes and no",
  "function command is the WHOLE",
  "conversation a DS1990 can have, so",
  "nothing is missing. The header then",
  "prints the number written on the key",
  "- six serial bytes, high first.",
  "",
  "A DS2413's PIO answer is checked",
  "before it is read: its top nibble is",
  "the complement of its bottom, so a",
  "bus nobody is driving (all ones)",
  "fails it instead of reading as two",
  "pins that are high.",
  "",
  INFO_HEAD "SWO / ITM",
  "SWO / ITM is the trace pin every",
  "Cortex-M has and hardly anyone",
  "looks at: the firmware's own printf,",
  "off ONE wire. The library writes a",
  "character to stimulus port 0 and the",
  "macrocell puts it on the pin.",
  "Electrically it is 8N1 idling high,",
  "at traceclk/(SWOSCALER+1) - any rate",
  "at all, not a standard baud - so the",
  "rate is measured off the record.",
  "What makes it SWO and not a console",
  "is the PACKET GRAMMAR. Every byte is",
  "a header or somebody's payload:",
  "  xxxxxxSS  a source packet, SS says",
  "  1/2/4 payload bytes, bit 2 says",
  "  stimulus port or DWT",
  "  0TTT0000  short timestamp",
  "  11TT0000  long one, 7 bits a byte",
  "  0x70      OVERFLOW - trace was",
  "  dropped, so what follows has holes",
  "  5x 00, 80 the sync sequence",
  "Reserved DWT numbers and wrong",
  "payload sizes are what turn a serial",
  "console down, and it walks into one",
  "within a few bytes. The packets must",
  "TILE the record with nothing left",
  "over, unless a sync sequence proves",
  "it outright. Under 100 kbit is not a",
  "trace port and is not offered - MIDI",
  "at 31250 would otherwise fit.",
  "The DWT half is the interesting one:",
  "exceptions by NAME ('SysTick in'),",
  "sampled program counters, and the",
  "watchpoint reads and writes.",
  "The header shows the text itself.",
  "2 MHz: 20 to 100 us/div.",
  "",
  INFO_HEAD "SWD",
  "SWD reads a debugger at work off",
  "SWDIO alone. Same predicament as",
  "SPI - the clock is on the other wire",
  "- and a different outcome, because",
  "SWD CHECKS ITSELF: parity over the",
  "four address bits, a stop bit that",
  "must be 0, a park bit that must be",
  "1, three acknowledgement bits with",
  "three legal values, and parity over",
  "the 32 data bits. Six checks. A bit",
  "time even slightly wrong fails them,",
  "so a transaction that reads clean",
  "confirms the recovered clock too.",
  "Each packet is anchored on its OWN",
  "start bit, so a probe that clocks in",
  "bursts and stops between them costs",
  "nothing - unlike SPI, the phase",
  "never drifts past one packet.",
  "Registers are NAMED, and an AP",
  "register needs the bank out of a DP",
  "SELECT write that went past earlier,",
  "so SELECT is tracked as it goes.",
  "Auto mode takes it on a JTAG-to-SWD",
  "switch sequence, or on two clean",
  "transactions with nothing but idle",
  "between them. Otherwise pick it by",
  "name: a WAIT costs an impostor only",
  "four of the six checks, and four is",
  "not rare enough.",
  "Assumed, and producing NO output",
  "rather than wrong output if untrue:",
  "one turnaround cycle and overrun",
  "detection off. Both are the reset",
  "defaults. Decoder > 'SWD clock =",
  "measured' with the probe on SWCLK",
  "first; '~' marks a guessed rate.",
};

static const info_page_t g_page_decoder_help =
{
  .title = "Decoder",
  .lines = g_decoder_help_lines,
  .count = ARRAY_SIZE(g_decoder_help_lines),
};

//-----------------------------------------------------------------------------
// Menu tables (const, in flash). config changes are persisted automatically
// by config_task once the struct CRC goes stale.
//-----------------------------------------------------------------------------
static const menu_item_t g_trigger_items[] =
{
  { .kind = MI_CHOICE, .label = "Mode",
    .u.choice = { &config.trigger_mode, g_trigger_mode_labels, 3, trigger_mode_changed } },
  { .kind = MI_CHOICE, .label = "Edge",
    .u.choice = { &config.trigger_edge, g_trigger_edge_labels, 3, trigger_edge_changed } },
  { .kind = MI_NUMBER, .label = "Level",
    .u.number = { &config.trigger_level, -100, 100, 1, 5, "px", trigger_level_apply } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "Level to 50%",
    .u.action = { action_trigger_50p, NULL } },
};

// The panel: a set of metrics in a small font, composited over the trace
static const menu_item_t g_panel_items[] =
{
  { .kind = MI_CHOICE, .label = "Panel",
    .u.choice = { &config.measure_panel_mode, g_measure_panel_labels, 2, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_TOGGLE, .label = "Vpp",
    .u.toggle = { &config.show_vpp, NULL } },
  { .kind = MI_TOGGLE, .label = "Frequency",
    .u.toggle = { &config.show_freq, NULL } },
  { .kind = MI_TOGGLE, .label = "Duty cycle",
    .u.toggle = { &config.show_duty, NULL } },
  { .kind = MI_TOGGLE, .label = "Vrms",
    .u.toggle = { &config.show_vrms, NULL } },
  { .kind = MI_TOGGLE, .label = "Vavg",
    .u.toggle = { &config.show_vavg, NULL } },
  { .kind = MI_TOGGLE, .label = "Signal type",
    .u.toggle = { &config.show_type, NULL } },
  { .kind = MI_TOGGLE, .label = "THD",
    .u.toggle = { &config.show_thd, NULL } },
  { .kind = MI_TOGGLE, .label = "Jitter",
    .desc = "Period sigma ~ peak-to-peak",
    .u.toggle = { &config.show_jitter, NULL } },
};

// The status line: exactly two values in the large font, and the user says
// which. Set both to Off and the line goes back to the trigger edge, the
// trigger level and the horizontal position.
static const menu_item_t g_line_items[] =
{
  { .kind = MI_CHOICE, .label = "Left",
    .desc = "Large readout at x=140",
    .u.choice = { &config.measure_line[0], g_measure_slot_labels,
        MEASURE_COUNT, NULL } },
  { .kind = MI_CHOICE, .label = "Right",
    .desc = "Large readout at x=228",
    .u.choice = { &config.measure_line[1], g_measure_slot_labels,
        MEASURE_COUNT, NULL } },
};

// Measurements display: values live in config; the scope picks them up on
// its next tick, no callbacks needed
static const menu_item_t g_measure_items[] =
{
  { .kind = MI_TOGGLE, .label = "Show (MODE)",
    .u.toggle = { &config.measure_display, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_SUBMENU, .label = "Panel over the trace",
    .u.submenu = { g_panel_items, ARRAY_SIZE(g_panel_items) } },
  { .kind = MI_SUBMENU, .label = "Status line",
    .u.submenu = { g_line_items, ARRAY_SIZE(g_line_items) } },
};

static const menu_item_t g_decoder_items[] =
{
  { .kind = MI_CHOICE, .label = "Protocol",
    .u.choice = { &config.decoder_proto, g_decoder_proto_labels,
        ARRAY_SIZE(g_decoder_proto_labels), NULL } },
  { .kind = MI_CHOICE, .label = "UART baud",
    .desc = "Auto reads it off the record",
    .u.choice = { &config.decoder_baud, decoder_baud_labels,
        DECODER_BAUD_COUNT, NULL } },
  { .kind = MI_TOGGLE, .label = "Stop on frames",
    .desc = "Freeze the record on a match",
    .u.toggle = { &config.decoder_stop, NULL } },
  { .kind = MI_TOGGLE, .label = "  ...at message start",
    .desc = "Only after the line was idle",
    .u.toggle = { &config.decoder_stop_start, NULL } },
  { .kind = MI_CHOICE, .label = "Set up from the rate",
    .desc = "Timebase and trigger, once",
    .u.choice = { &config.decoder_fit_mode, g_decoder_fit_labels, 2, NULL } },
  { .kind = MI_CHOICE, .label = "Bit grid",
    .desc = "Bit lines and numbers on the trace",
    .u.choice = { &config.decoder_bits_mode, g_decoder_bits_labels, 2,
        scope_decode_redraw } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "Catch a message start",
    .desc = "Arm and freeze on the first one",
    .u.action = { action_decode_catch, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "SPI clock = measured",
    .desc = "Probe on SCK, then move it to MOSI",
    .u.action = { action_spi_clock, NULL } },
  { .kind = MI_CHOICE, .label = "SPI bit order",
    .desc = "Auto scores both readings",
    .u.choice = { &config.spi_order, g_spi_order_labels, 3, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_CHOICE, .label = "Manchester rate",
    .desc = "Auto cannot tell a rate from half of it",
    .u.choice = { &config.man_rate, decoder_man_rate_labels,
        DECODER_MAN_RATE_COUNT, NULL } },
  { .kind = MI_CHOICE, .label = "Manchester bit",
    .desc = "RC5 is Rise=1, DALI is Rise=0",
    .u.choice = { &config.man_polarity, g_man_pol_labels, 3, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "SWD clock = measured",
    .desc = "Probe on SWCLK, then move it to SWDIO",
    .u.action = { action_swd_clock, NULL } },
};

//-----------------------------------------------------------------------------
// Display processing: persistence and averaging. Both accumulate per-column
// state inside the scope; a toggle mid-run must drop it.
static void display_processing_changed(void)
{
  scope_display_settings_changed();
}

static const char *const g_average_labels[] =
{
  "Off", "4 frames", "8 frames", "16 frames", "32 frames", "64 frames",
};

static const menu_item_t g_display_items[] =
{
  { .kind = MI_TOGGLE, .label = "Persistence",
    .desc = "Envelope accumulates over frames",
    .u.toggle = { &config.display_persist, display_processing_changed } },
  { .kind = MI_CHOICE, .label = "Averaging",
    .desc = "Mean of N triggered frames",
    .u.choice = { &config.average_mode, g_average_labels,
        ARRAY_SIZE(g_average_labels), display_processing_changed } },
};

// The four parameters the calibration screen edits, and the order the
// procedure wants them in. Z and D are the two that decide how quiet an
// unterminated input looks: Z centres the single ADC, D matches the second
// one to it. S and O are per vertical range and have to be redone on each.
static const char *const g_calib_help_lines[] =
{
  INFO_HEAD "Keys",
  "TRIG_UP/DN   - change the value",
  "SHIFT+TRIG_x - next parameter",
  "SHIFT+L/R    - timebase, SHIFT+U/D - volts",
  "AUTO         - auto-calibrate Z, D and O",
  "MODE         - hide the hint (back next step)",
  "",
  INFO_HEAD "Parameters",
  "Z zero    - offset DAC centre",
  "D delta   - ADC B against ADC A",
  "S scale   - gain, PER volts/div range",
  "O offset  - DAC step, PER volts/div range",
  "",
  INFO_HEAD "Procedure",
  "SHORT the input for Z and D - an open",
  "1 MOhm input is an antenna, and the noise",
  "it picks up is what makes the numbers",
  "impossible to aim.",
  "",
  "Z: single channel. Mid screen is 0 V, which",
  "is a raw ADC reading of 0x80 = 128. Aim the",
  "avg there. D: dual channel, make A and B",
  "read the same - their mismatch IS the",
  "sample-to-sample noise on the trace.",
  "The readout turns RED when the parameter",
  "does not belong to the current mode.",
  "",
  "S: apply a known level, ideally with the",
  "raw data in 0xd0-0xf0, and adjust until",
  "min and max match it. S and O are per",
  "vertical range; Z and D are shared.",
};

static const info_page_t g_page_calib_help =
{
  .title = "Calibration",
  .lines = g_calib_help_lines,
  .count = ARRAY_SIZE(g_calib_help_lines),
};

//-----------------------------------------------------------------------------
// Numeric calibration entry.
//
// The scope screen edits these with the trigger keys, blind and one step at a
// time; a gain correction of a few percent is hundreds of presses there. Here
// the value is a number you can see and hold a key on. Scale and DAC step are
// per vertical range, and a const menu table cannot point into a live index,
// so they go through proxies that the opening action loads and the apply
// callbacks write back.
//-----------------------------------------------------------------------------
static int g_calib_scale_proxy;
static int g_calib_dac_proxy;

static const char *const g_calib_range_labels[] =
{
  "50 mV", "100 mV", "200 mV", "500 mV", "1 V", "2 V", "5 V", "10 V",
};

//-----------------------------------------------------------------------------
static void calib_load_proxies(void)
{
  g_calib_scale_proxy = config.calib_vs_mult[config.vertical_scale];
  g_calib_dac_proxy   = config.calib_dac_mult[config.vertical_scale];
}

//-----------------------------------------------------------------------------
static void calib_range_changed(void)
{
  scope_set_vertical_scale(config.vertical_scale);
  calib_load_proxies();
  ui_request_redraw(); // the two per-range rows now show different numbers
}

//-----------------------------------------------------------------------------
static void calib_zero_apply(int value)
{
  (void)value;
  scope_calib_apply(true); // the offset DAC moved
}

//-----------------------------------------------------------------------------
static void calib_delta_apply(int value)
{
  (void)value;
  scope_calib_apply(false); // software only, no acquisition restart
}

//-----------------------------------------------------------------------------
static void calib_scale_apply(int value)
{
  config.calib_vs_mult[config.vertical_scale] = value;
  scope_calib_apply(false);
}

//-----------------------------------------------------------------------------
static void calib_dac_apply(int value)
{
  config.calib_dac_mult[config.vertical_scale] = value;
  scope_calib_apply(true);
}

//-----------------------------------------------------------------------------
static void action_autocal(const void *arg)
{
  (void)arg;
  // It runs on the scope screen and paints its progress there, so the whole
  // popup chain has to go - not ui_pop_to_root(), which would leave the app
  menu_close_popups();
  scope_autocal_start();
}

static const menu_item_t g_calib_value_items[] =
{
  { .kind = MI_CHOICE, .label = "Range",
    .desc = "Scale and DAC step below are for THIS range",
    .u.choice = { &config.vertical_scale, g_calib_range_labels,
        ARRAY_SIZE(g_calib_range_labels), calib_range_changed } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_NUMBER, .label = "Z  zero",
    .desc = "Offset DAC centre, input open -> 0 V",
    .u.number = { &config.calib_dac_zero, 1900, 2200, 1, 10, NULL,
        calib_zero_apply } },
  { .kind = MI_NUMBER, .label = "D  channel delta",
    .desc = "ADC A against ADC B, input open",
    .u.number = { &config.calib_channel_delta, -64, 64, 1, 4, NULL,
        calib_delta_apply } },
  { .kind = MI_NUMBER, .label = "S  scale",
    .desc = "Gain: raise to read higher",
    .u.number = { &g_calib_scale_proxy, 1, 4000000, 1, 200, NULL,
        calib_scale_apply } },
  { .kind = MI_NUMBER, .label = "O  DAC step",
    .desc = "1 position pixel = 1 screen pixel",
    .u.number = { &g_calib_dac_proxy, 0, 100000, 1, 100, NULL,
        calib_dac_apply } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_NUMBER, .label = "Reference, mV",
    .desc = "Level the gain step asks you to apply",
    .u.number = { &config.calib_ref_mv, 50, 40000, 10, 250, "mV", NULL } },
};

static const menu_def_t g_calib_values =
{
  .title = "Calibration Values",
  .items = g_calib_value_items,
  .count = ARRAY_SIZE(g_calib_value_items),
};

//-----------------------------------------------------------------------------
static void action_calib_values(const void *arg)
{
  (void)arg;
  calib_load_proxies(); // a const table cannot follow the range on its own
  menu_open_dialog(&g_calib_values); // MENU or LEFT backs out
}

static const menu_item_t g_calib_items[] =
{
  { .kind = MI_ACTION, .label = "Auto-calibrate",
    .desc = "Disconnect the probe first",
    .u.action = { action_autocal, NULL } },
  { .kind = MI_ACTION, .label = "Enter values",
    .desc = "Type Z, D, S and O per range",
    .u.action = { action_calib_values, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_TOGGLE, .label = "Calibration mode",
    .u.toggle = { &scope_calibration_mode, scope_calibration_changed } },
};

static const menu_item_t g_scope_items[] =
{
  { .kind = MI_SUBMENU, .label = "Trigger Settings",
    .u.submenu = { g_trigger_items, ARRAY_SIZE(g_trigger_items) } },
  { .kind = MI_SUBMENU, .label = "Measurements",
    .u.submenu = { g_measure_items, ARRAY_SIZE(g_measure_items) } },
  { .kind = MI_SUBMENU, .label = "Display",
    .u.submenu = { g_display_items, ARRAY_SIZE(g_display_items) } },
  { .kind = MI_SUBMENU, .label = "Decoder",
    .u.submenu = { g_decoder_items, ARRAY_SIZE(g_decoder_items) } },
  { .kind = MI_SUBMENU, .label = "Calibration",
    .u.submenu = { g_calib_items, ARRAY_SIZE(g_calib_items) } },
};

const menu_def_t scope_menu =
{
  .title = "Oscilloscope",
  .items = g_scope_items,
  .count = ARRAY_SIZE(g_scope_items),
};

// The read-only pages. They are not settings, so they do not sit among the
// settings: the system menu folds them into its Help section, next to the
// pages every other application contributes.
static const menu_item_t g_scope_help_items[] =
{
  { .kind = MI_ACTION, .label = "Scope Functions",
    .u.action = { menu_action_info, &g_page_scope_help } },
  { .kind = MI_ACTION, .label = "Spectrum (FFT)",
    .u.action = { menu_action_info, &g_page_spectrum_help } },
  { .kind = MI_ACTION, .label = "Decoder",
    .u.action = { menu_action_info, &g_page_decoder_help } },
  { .kind = MI_ACTION, .label = "Calibration",
    .u.action = { menu_action_info, &g_page_calib_help } },
};

const menu_def_t scope_help_menu =
{
  .title = "Oscilloscope",
  .items = g_scope_help_items,
  .count = ARRAY_SIZE(g_scope_help_items),
};
