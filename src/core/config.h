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
  // The SPECTRUM's answer to the same question, next to the counter's. They
  // are different instruments and they fail differently: the counter resolves
  // parts per million at low frequencies and goes blind above about three
  // samples per period (50.000 MHz on a 125 MS/s record reads exactly
  // 25.000), while the peak is only as fine as a bin but cannot miss an edge
  // it never had to find. Where the two disagree, the faster signal is why.
  MEASURE_FFT_FREQ,
  // The lowest frequency above nyquist that would have folded to the reading
  // shown by the two above. It appears only while the record cannot rule that
  // reading out - see alias.h - so a blank here is an answer too: it means the
  // harmonics stand where only an in-band source can put them, or that the
  // question does not arise. Never a second opinion on a frequency; the only
  // claim it makes is "could equally be this".
  MEASURE_ALIAS,

  // New metrics MUST be appended at the end of this enum and never inserted:
  // config.measure_line[] stores these numbers on flash, so an inserted value
  // silently re-points a status line slot the user set up at whatever moved
  // into its number.
  MEASURE_VP,        // peak from 0 V: max(|Vmax|, |Vmin|)
  MEASURE_VMAX,      // the two extremes on their own, also from 0 V
  MEASURE_VMIN,
  MEASURE_VAMP,      // Vtop - Vbase: the swing between the flat levels, with
                     // overshoot and ringing left out (see measure.h)
  MEASURE_PERIOD,    // time of one whole cycle - what the frequency is 1/x of
  MEASURE_WIDTH_POS, // and how that cycle splits: time above the mid level
  MEASURE_WIDTH_NEG, // and time below it. T+ / T is the duty cycle.
  MEASURE_COUNT,
};

#define MEASURE_LINE_SLOTS 2

// The measurements panel's own look. Both are zero-is-default, so a config
// saved before they existed - or migrated from store version 1 - comes back
// with the panel it already had.
//
// The font decides how much fits, not just how big it reads: the band is two
// rows either way, three cells to a row in the 6x8 font and two in the 8x16.
// Enabling more metrics than that shows the first of them and says how many
// were left out; it does not shrink the text to fit.
enum
{
  PANEL_FONT_SMALL = 0,   // 6x8: six readings
  PANEL_FONT_LARGE,       // 8x16: four, readable across a bench
  PANEL_FONT_COUNT,
};

enum
{
  PANEL_BG_DIM = 0,       // the trace behind the text at half brightness
  PANEL_BG_SOLID,         // an opaque band with a hairline along its top
  PANEL_BG_OFF,           // nothing: the text sits straight over the trace
  PANEL_BG_COUNT,
};

// What to draw between samples once the screen has more pixels than the
// record has samples. Straight lines are the default because they invent
// nothing beyond the corner at each sample; the reconstruction is exact for
// a signal below nyquist and a convincing lie for one above it.
enum
{
  DRAW_LINEAR = 0,
  DRAW_SINC,
  DRAW_COUNT,
};

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

  // MEASURE_ALIAS as a panel/status metric. It sits HERE, in the three-byte
  // alignment hole that already followed measure_display, and not at the end
  // of the show_* run where it belongs by meaning: that run ends flush against
  // measure_line, so a bool appended there pushes every later field - crc and
  // the whole calibration block included - and the store drops an entry whose
  // sizeof does not match, erasing calibration with it. Same reasoning as
  // show_jitter and show_fft_freq, which is why neither of those is beside its
  // siblings either: no offset moves, and version 1's did not.
  //
  // Its later siblings (show_vp and the rest) DO sit together, further down -
  // that is what the version 2 reserve bought. This one stays where it is:
  // moving it now would break the prefix a version 1 store is migrated by.
  bool     show_alias;

  // General settings

  // How long without a key before the backlight dims: an index into
  // lcd_dim_labels, 0 = Off. See lcd.c for what it does and main.c for who
  // is allowed to be dimmed.
  //
  // This is the word that used to be `int screen_brightness`, which nothing
  // ever read - the backlight has always been lcd_bl_level, and 80 was
  // written into this every boot for nobody. Reusing it costs no offset and
  // leaves padding[] for a field that has to be new. A config saved before
  // this holds that 80 here, which is out of range for the list and reads as
  // Off, so no unit starts dimming because its firmware was updated
  // (lcd_backlight_init writes the 0 back).
  int      lcd_dim_timeout;     // was screen_brightness

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
  // sizeof does not move, and the store then rejects and ERASES every saved
  // entry, calibration included. That is not a theory; it happened.
  bool     show_jitter;
  // Narrow stop-on-decode to records that caught the line at rest first, so
  // the freeze lands on the head of a message instead of a slice out of its
  // middle. Carved out of decoder_reserved; false in every config saved
  // before it existed, which is the old behaviour.
  bool     decoder_stop_start;
  // The spectrum's frequency as a panel/status metric. Carved out of the last
  // decoder_reserved byte for the same reason show_jitter was: appending it
  // to the show_* run above would move every field behind it and make the
  // store throw away every saved config, calibration included.
  bool     show_fft_freq;

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

  // How the trace is drawn between samples when the timebase is zoomed in
  // past one sample per pixel: DRAW_LINEAR joins them with a straight line,
  // DRAW_SINC reconstructs the band-limited curve that actually passes
  // through them. Zero-is-default, so a config saved before this existed
  // keeps the straight lines. This was the LAST word of version 1's padding[]
  // - which is why there is a version 2, and why the reserve below exists.
  int      draw_mode;

  /*
   * ---- Everything above this line is the VERSION 1 layout, byte for byte ----
   *
   * Version 2 grew the struct instead of hunting for another alignment hole to
   * hide a bool in, and it can carry a version 1 store forward because it kept
   * that layout as its own prefix: config.c migrates an old entry with two
   * memcpys (the prefix, then the calibration block to its new offset) rather
   * than a sixty-field copy list with one field quietly missing from it.
   *
   * So the rule for a field added from here on is the same rule as before, and
   * for the same reason - NEVER insert one above this line or between the ones
   * below; take the space out of padding[] - only now there is padding to take
   * it out of, and running out of it again means a version 3, not a hole hunt.
   */

  // The panel flags for the metrics MEASURE_VP..MEASURE_WIDTH_NEG. Plain bools
  // again, in the same order as the enum, because there is finally room for
  // them; false in a migrated config, which is the panel it already had.
  bool     show_vp;
  bool     show_vmax;
  bool     show_vmin;
  bool     show_vamp;
  bool     show_period;
  bool     show_width_pos;
  bool     show_width_neg;

  // The graticule behind the trace: 0 = on, 1 = off. Zero-is-default like
  // measure_panel_mode, so a config saved before this field existed - or
  // migrated from version 1 - comes back with the ruling it had rather than a
  // blank screen. "Off" empties the trace area only; the frame around it is
  // the edge of the window and not part of the ruling, so it stays.
  int      grid_mode;

  // How the measurements panel looks: PANEL_FONT_* and PANEL_BG_*. Two words
  // out of the reserve below, and worth it - the panel is on screen whenever
  // the instrument is, and until now its size and its background were
  // decisions the user had no say in.
  int      measure_panel_font;
  int      measure_panel_bg;

  /*
   * The reserve. A new field is carved out of HERE, shrinking the array by
   * exactly what it takes so that sizeof(Config) and every offset below stay
   * put - an entry whose size does not match is not read, and the padding is
   * what keeps that from being every entry on the flash after every change.
   *
   * One word. Version 2 started with four; grid_mode took one and the panel's
   * font and background two. The limit is not the flash: the 512-byte entry slot
   * would
   * take another 68 bytes happily. What cannot take them is the TCM, where this
   * struct is resident THREE times over - config itself, the snapshot config.c
   * compares against to decide whether a save is due, and its one-entry read
   * cache - against a linker assert that keeps 16 KB of the 64 clear for stack
   * and heap. That assert had about 100 bytes of slack left before version 2,
   * and 24 bytes of struct is 72 bytes of TCM.
   *
   * So this is what fits rather than what would be comfortable, and the way to
   * make it comfortable is to free TCM (the read cache is the obvious 500
   * bytes: nothing needs it to outlive the call that reads it), not to trim the
   * flash side. Running out is also no longer the dead end it was in version 1:
   * config.c can carry a layout forward now, so a version 3 costs a prefix
   * constant, a static assert and one more branch in adopt_v1_entry().
   */
  uint32_t padding[1];

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
  CONFIG_ENTRY_V1,           // the layout before this one, and intact. What
                             // the store looks like on the first boot after
                             // the update, and not a fault: config.c reads it
                             // (calibration included) and writes it back as
                             // one of ours.
} ConfigEntryState;

int config_store_entries(void);

// One entry into the caller's buffer - a `Config` on the stack is what this is
// for. There is no buffer inside config.c to hand a pointer to any more: it was
// a third resident copy of this struct in a TCM that had ~100 bytes of slack,
// and nothing ever needed an entry to outlive the call that read it.
bool config_store_read_entry(int index, Config *out);

ConfigEntryState config_store_entry_state(int index);
// ...and the same verdict on an entry already in hand, so a caller that has
// just read one does not pay for reading it again
ConfigEntryState config_entry_state_of(const Config *entry);
const char *config_entry_state_name(ConfigEntryState state);

// Which slot config was loaded from / is being written to, -1 if neither
int config_store_live_index(void);

// Where the store physically is, for anything that has to point at it rather
// than read it - the flash viewer's "jump to the store" actions, mainly.
// `external` distinguishes an address inside the SPI part (where a pointer is
// meaningless and only the viewer's SPI source can reach it) from one in the
// MCU's own flash. `base` is the address of entry 0 in whichever it is.
uint32_t config_store_base(void);
uint32_t config_store_entry_size(void);
bool config_store_is_external(void);

// Whether an entry's calibration block seals, independently of the entry
bool config_store_entry_calib_ok(int index);
// ...and on an entry already read, for the same reason as config_entry_state_of
bool config_entry_calib_ok(const Config *entry);

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
