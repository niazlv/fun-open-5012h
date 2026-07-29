/*
 * Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
 * Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "common.h"

/*- Definitions -------------------------------------------------------------*/
#define CALIB_MULTIPLIER   1024

/*- Types -------------------------------------------------------------------*/
// The measurable quantities, by name. The panel takes a set of them (the
// show_* flags), the status line takes one per slot by number - the line has
// room for exactly two values in the large font, so which two is a choice the
// user makes, not something to rotate through.
enum
{
  MEASURE_NONE = 0,   // status-line slot only: leave it to the trigger readout
  MEASURE_VPP,
  MEASURE_FREQ,
  MEASURE_DUTY,
  MEASURE_VRMS,
  MEASURE_VAVG,
  MEASURE_TYPE,
  MEASURE_THD,
  MEASURE_JITTER,     // period sigma / p-p, from the per-period statistics
  MEASURE_COUNT,
};

#define MEASURE_LINE_SLOTS 2

// Health of the settings store, reported by config_get_state(). FRESH on any
// boot other than the very first one means the previous session's settings
// did not survive - the symptom this used to produce with no way to see it.
typedef enum
{
  CONFIG_STORE_FRESH,        // nothing usable in flash, defaults in use
  CONFIG_STORE_LOADED,       // restored at boot, nothing saved since
  CONFIG_STORE_OK,           // last save was written and read back
  CONFIG_STORE_ERASE_ERROR,
  CONFIG_STORE_WRITE_ERROR,
  CONFIG_STORE_VERIFY_ERROR, // written, but it did not read back
} ConfigStoreStatus;

typedef struct
{
  uint32_t magic;
  int      size;
  int      version;
  int      count;

  int      power_cycles;
  int      charge_cycles;

  int      lcd_bl_level;

  bool     ac_coupling;
  bool     x10;

  int      trigger_mode;
  int      trigger_edge;
  int      trigger_level;
  int      trigger_level_mv;

  int      horizontal_scale;
  int64_t  horizontal_position;
  int      horizontal_position_px;
  int      horizontal_period;

  int      vertical_scale;
  int      vertical_position;
  int      vertical_position_mv;
  int      vertical_mult;

  int      sample_rate_limit;

  bool     measure_display;

  // General settings
  int      screen_brightness;
  bool     shift_mode_enabled;  // double click on SHIFT: sticky for one key
  bool     shift_mode_active;
  // Carved out of the alignment padding that already sat between the bools
  // above and key_mapping, so sizeof(Config) does not move and a config saved
  // by the firmware before it stays readable. It reads false there, which is
  // this switch off - one toggle in the menu, not a lost settings store.
  bool     shift_hold_lock;     // hold SHIFT: latched until it is tapped off
  
  // Key remapping (store original button codes)
  uint32_t key_mapping[32];  // Map for button remapping
  bool     key_remapping_enabled;

  // Measurements. Two independent places, because they are good at different
  // things: the translucent panel over the trace lists a whole set of metrics
  // in a small font, the status line shows two of them in the large one and
  // is readable from across a bench.
  //
  // measure_panel_mode: 0 = panel on (default), 1 = off. Zero-is-default is
  // deliberate - configs saved before this field existed read as 0 and get the
  // panel. The show_* flags pick which metrics the panel lists; all-false
  // means the default set. Carved out of padding so sizeof(Config) and stored
  // configs (including calibration) stay valid.
  int      measure_panel_mode;
  bool     show_vpp;
  bool     show_freq;
  bool     show_duty;
  bool     show_vrms;
  bool     show_vavg;
  bool     show_type;
  bool     show_thd;

  // False in every config saved before the slots existed, which is how the
  // one-time migration in config_init tells "the user asked for an empty
  // status line" from "this config predates the question". Bumping VERSION
  // instead would have invalidated every stored entry, calibration included.
  bool     measure_line_set;

  // What the status line's two slots show, by metric number. MEASURE_NONE
  // leaves a slot to the trigger edge / level / position readouts, and two
  // of them give the stock status line back.
  int      measure_line[MEASURE_LINE_SLOTS];

  // Logic decoder: forced protocol (proto_t; 0 = auto) and stop-on-decode
  int      decoder_proto;
  bool     decoder_stop;
  // show_jitter lives in what used to be decoder_reserved[0] - NOT appended
  // to the show_* run above: a ninth bool there pushes 3 alignment bytes in
  // front of measure_line, shifts every later field (crc included) by 4 while
  // sizeof stays 416, and the store then rejects and ERASES every saved
  // entry, calibration included. That is not a theory; it happened.
  bool     show_jitter;
  // Narrow stop-on-decode to records that caught the line at rest first, so
  // the freeze lands on the head of a message instead of a slice out of its
  // middle. Carved out of decoder_reserved; false in every config saved
  // before it existed, which is the old behaviour.
  bool     decoder_stop_start;
  bool     decoder_reserved[1];

  // Reference level, in mV, that the gain (S) calibration aims at: whatever
  // you can measure accurately and apply to the input. Carved out of padding
  // so sizeof(Config) and every stored calibration stay valid; a config saved
  // before this existed reads 0, which the gain step treats as "not set yet".
  int      calib_ref_mv;

  // Display processing, carved out of padding (configs saved before these
  // existed read 0 = both off). average_mode is the acquisition averaging
  // depth, 0 = off, otherwise N = 2 << mode (1..5 -> 4, 8, 16, 32, 64).
  //
  // persist_mode: 0 = off, 1 = infinite, 2 = decay. It occupies the four
  // bytes that were `bool display_persist; bool display_reserved[3];` - same
  // offset, same size, same alignment (calib_ref_mv above it is an int), so
  // sizeof(Config) and every field below are untouched. The numbering is not
  // arbitrary either: little-endian puts the old bool in the low byte, so a
  // config saved with persistence ON reads back as exactly 1, which is why 1
  // has to mean the behaviour that bool used to select. No migration code,
  // and nothing stored is invalidated.
  int      persist_mode;
  int      average_mode;

  // UART decoder baud: index into the menu's rate table, 0 = auto-detect.
  // Carved out of padding (configs saved before it existed read 0 = auto).
  int      decoder_baud;

  // Set the timebase from the decoded rate, so a whole message lands on the
  // screen without hunting for it: 0 = on, 1 = off. Zero-is-default, like
  // measure_panel_mode, so a config saved before this field existed gets it.
  int      decoder_fit_mode;

  // Draw the bit grid over the trace: a hairline at every bit boundary of a
  // decoded byte and the bit's number in the cell. 0 = on, 1 = off, so a
  // config saved before this field existed gets it - same zero-is-default as
  // decoder_fit_mode. Carved out of padding, which is why padding shrank by
  // one word rather than the field being appended.
  int      decoder_bits_mode;

  // The SPI clock, in Hz, measured on SCK before the probe moved to the data
  // line. 0 = work the bit time out of the data itself. Written by the
  // "SPI clock = measured" action rather than typed, because a frequency
  // measurement is exactly what the scope already has on screen while the
  // probe is still on the clock. Carved out of padding.
  int      spi_clock_hz;

  // 0 = score both bit orders, 1 = MSB first, 2 = LSB first. Zero-is-default
  // again: a config saved before this existed gets the scoring.
  int      spi_order;

  // Manchester: the bit rate as an index into the decoder menu's own table
  // (0 = work it out of the record) and which convention the bits follow
  // (0 = a rising mid-bit edge is a one, 1 = it is a zero). Nothing in a
  // Manchester waveform decides the second one - it is a property of the
  // protocol riding on the code, so it has to be told. Carved out of padding.
  int      man_rate;
  int      man_polarity;

  // The SWD clock, in Hz, measured on SWCLK before the probe moved to SWDIO.
  // Its own field rather than a share of spi_clock_hz: a bench that has both
  // buses on it would otherwise have to re-measure every time the probe moved
  // between them. 0 = work the bit time out of the record. Carved out of
  // padding, like everything above it.
  int      swd_clock_hz;

  // Where the roll view takes over from the swept trace, as an index into
  // scope.c's own table: 0 = 1 s/div, i.e. only where a sweep cannot work at
  // all. Zero-is-default again, so a config saved before this field existed
  // gets the stock behaviour. Carved out of padding.
  int      roll_from;

  // Snake's best score. A high score that dies with the power is not a high
  // score, and the game has nowhere else to keep it. Carved out of padding
  // like every field above it, so sizeof(Config) and every stored calibration
  // stay valid; a config saved before it existed reads 0, which is "no score
  // yet" - the same zero-is-default rule the rest of this struct follows.
  int      snake_high_score;

  // Flappy Bird's, on the same terms
  int      flappy_high_score;

  // 2048's, shared by every board size: one number, because the games are the
  // same game and the size is a setting rather than a mode
  int      g2048_high_score;

  // Tetris's, on the same terms
  int      tetris_high_score;

  // Seals the calibration block below on its own, so it can be believed when
  // the entry around it cannot. Everything else in here is a preference, and
  // losing one costs a minute of setting the scope back up. Calibration is not
  // a preference: it is a measurement of THIS unit's analog chain, it takes a
  // reference source and a calibration run to reproduce, and the numbers
  // config_reset_calibration() falls back on belong to a different unit. It
  // should not die of a checksum failure somewhere else in the struct.
  //
  // Carved out of padding like every field above it, so no offset moves. Reads
  // 0 in a config saved before this existed, which fails to validate and
  // leaves the recovery path with nothing to find - calibration in such an
  // entry still loads the ordinary way, vouched for by the entry's own crc.
  uint32_t calib_crc;

  uint32_t padding[1];  // Reduced padding to accommodate new fields

  // The calibration block. Contiguous, and last before crc, on purpose:
  // calib_crc covers exactly the bytes from calib_channel_delta up to crc. A
  // new field must NOT be appended here - put it in padding[] above, or it
  // lands inside the sealed range and silently changes what the seal means.
  int      calib_channel_delta;
  int      calib_dac_zero;
  int      calib_dac_mult[VS_COUNT];
  int      calib_vs_mult[VS_COUNT];

  uint32_t crc;
} Config;

/*- Variables ---------------------------------------------------------------*/
extern Config config;

/*- Prototypes --------------------------------------------------------------*/
void config_init(void);

// Preferences back to defaults. Deliberately does NOT touch the calibration
// block - the boot-time button combo in main.c means "I have made a mess of
// the settings", not "throw away this unit's analog calibration", and until
// the two were separated it meant both.
void config_reset(void);

// ...which is what this is for: the calibration menu's own way of saying it,
// and where config_init() lands when there is nothing to recover
void config_reset_calibration(void);

// Where the calibration in use came from, for the System Information line
typedef enum
{
  CONFIG_CALIB_DEFAULTS,   // another unit's numbers - measurements are a guess
  CONFIG_CALIB_LOADED,     // came in with a valid config entry
  CONFIG_CALIB_RECOVERED,  // salvaged from an entry whose config half was bad
} ConfigCalibSource;

ConfigCalibSource config_calib_source(void);

// One line of the above for the System Information page
void config_get_calib_state(char *buf, int size);

// The calibration numbers as text, for the page that exists so they can be
// copied onto paper. Line by line; returns false once past the last one.
bool config_get_calib_line(int index, char *buf, int size);

//-----------------------------------------------------------------------------
// Reading the store from outside, for the flash viewer.
//
// The point is being able to answer "what is actually on the flash" separately
// from "what is the scope using", because the two disagreeing is the whole
// class of bug this store keeps producing. Everything here is read-only.
//-----------------------------------------------------------------------------

// Why an entry is not being used, which is more useful than a yes/no: a slot
// that fails on magic has never been written, one that fails on crc was.
typedef enum
{
  CONFIG_ENTRY_BLANK,        // erased, 0xff throughout
  CONFIG_ENTRY_VALID,
  CONFIG_ENTRY_BAD_MAGIC,
  CONFIG_ENTRY_BAD_VERSION,  // written by another firmware
  CONFIG_ENTRY_BAD_SIZE,     // ...one whose Config was a different size
  CONFIG_ENTRY_BAD_CRC,      // written, and did not survive
} ConfigEntryState;

int config_store_entries(void);
const Config *config_store_entry(int index);
ConfigEntryState config_store_entry_state(int index);
const char *config_entry_state_name(ConfigEntryState state);

// Which slot config was loaded from / is being written to, -1 if neither
int config_store_live_index(void);

// Whether an entry's calibration block seals, independently of the entry
bool config_store_entry_calib_ok(int index);

// One line of a Config rendered as "name  value", for both the store viewer
// and the page that shows what is in RAM. cfg may be any entry or &config.
// Returns false once past the last line.
bool config_describe(const Config *cfg, int index, char *buf, int size);
void config_task(void);

// Saves now rather than at the next timer tick, and waits for the flash
void config_flush(void);

// One line of store health for the System Information page
void config_get_state(char *buf, int size);

#endif // _CONFIG_H_
