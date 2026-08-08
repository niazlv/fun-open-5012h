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

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "gd32f4xx.h"
#include "hal_gpio.h"
#include "utils.h"
#include "lcd.h"
#include "timer.h"
#include "images.h"
#include "common.h"
#include "config.h"
#include "buttons.h"
#include "battery.h"
#include "capture.h"
#include "fft.h"
#include "alias.h"
#include "classify.h"
#include "logic_decode.h"
#include "trend.h"
#include "scope.h"
#include "scope_internal.h"

/*- Variables: the screen geometry ------------------------------------------*/
/*
 * Normal: what this instrument has always looked like. 8 divisions down; the
 * settings and the two readings share the bottom bar, the trigger mode and the
 * capture state sit in the top one beside the record map.
 *
 * Large: what only the READINGS being big looks like, which is the arrangement
 * the stock firmware uses and the reason it is legible at arm's length:
 *
 *   - every SETTING moves into the top bar as its own compact field: the probe
 *     ratio, volts per division, the capture state, time per division, the
 *     coupling, the trigger mode. Still 8x16. They are what you set, not what
 *     you read, and a word like AUTO does not become more informative at 32 px -
 *     it just eats the room the numbers need. (That was the first attempt: the
 *     state words were doubled and the settings left small. It read as a size
 *     applied to whatever happened to have room.)
 *   - the bottom bar holds nothing but the two readings, and inside a reading
 *     only the DIGITS are doubled - the name before them and the unit after
 *     stay 8x16, exactly as "UPP 7.84 U" does on the stock display. A padded
 *     unit at 16 px costs 24 px that the number wants.
 *   - which leaves the graticule 175 px, seven divisions instead of eight. ONE
 *     division, where doubling everything cost two.
 *
 * The record map is the one thing that does not fit: the settings row needs the
 * width it was using. mv_y = -1 turns it off, and draw_miniview returns.
 */
static const ScopeGeom g_geom_normal =
{
  .grid_top = 20, .grid_h = 200, .status_y = 223, .status_h = 16,
  .status_row2 = 0,
  .slot0_x = 140, .slot1_x = 228, .slot_w = 88,
  .vscale_x = 10, .vscale_y = 223,
  .acdc_x = 54, .acdc_y = 223,
  .hscale_x = 82, .hscale_y = 223,
  .mode_x = 10, .mode_y = 4,
  .state_x = 46, .state_y = 4,
  .mv_y = 1, .bar_scale = 1,
};

static const ScopeGeom g_geom_large =
{
  .grid_top = 20, .grid_h = 175, .status_y = 202, .status_h = 44,
  .status_row2 = 16,
  .slot0_x = 2, .slot1_x = 162, .slot_w = 158,
  .vscale_x = 30, .vscale_y = 2,
  .acdc_x = 176, .acdc_y = 2,
  .hscale_x = 126, .hscale_y = 2,
  .mode_x = 208, .mode_y = 2,
  .state_x = 88, .state_y = 2,
  .mv_y = -1, .bar_scale = 2,
};

ScopeGeom g_geom =
{
  .grid_top = 20, .grid_h = 200, .status_y = 223, .status_h = 16,
  .slot0_x = 140, .slot1_x = 228, .slot_w = 88,
  .vscale_x = 10, .vscale_y = 223,
  .acdc_x = 54, .acdc_y = 223,
  .hscale_x = 82, .hscale_y = 223,
  .mode_x = 10, .mode_y = 4,
  .state_x = 46, .state_y = 4,
  .mv_y = 1, .bar_scale = 1,
};


/*- Variables ---------------------------------------------------------------*/
// Five distinct grid column patterns; g_grid_index picks one per column
// (a byte index table instead of 300 pointers saves ~1.3 KB of scarce TCM)
static uint16_t g_grid_column_0[GRID_HEIGHT_MAX];
static uint16_t g_grid_column_1[GRID_HEIGHT_MAX];
static uint16_t g_grid_column_2[GRID_HEIGHT_MAX];
static uint16_t g_grid_column_3[GRID_HEIGHT_MAX];
static uint16_t g_grid_column_4[GRID_HEIGHT_MAX];

static uint16_t *const g_grid_columns[5] =
{
  g_grid_column_0, g_grid_column_1, g_grid_column_2,
  g_grid_column_3, g_grid_column_4,
};

static uint8_t g_grid_index[GRID_WIDTH];

DataBuffer g_data_buffer;
DisplayBuffer g_display_buffer;

int g_trace_column = (GRID_WIDTH-1);

// Shadow of what each trace column currently shows on the LCD. Columns whose
// data and overlay did not change since the last sweep are skipped entirely,
// so a static trace costs ~300 compares instead of ~11 ms of LCD writes.
static uint8_t g_shadow_min[GRID_WIDTH];
static uint8_t g_shadow_max[GRID_WIDTH];
static uint8_t g_shadow_flags[GRID_WIDTH];
bool g_shadow_valid = false;
static int g_shadow_marker_px = 0x7fffffff;
bool g_sweep_force = true;
// Set when the record has fewer than RECON_MIN_SAMPLES samples per period, so
// the reconstruction drawn between them is past what it can honestly do.
// Recomputed once per display rebuild, read by every column.
static bool g_recon_strained = false;
#define RECON_MIN_SAMPLES 3


// Persistence and averaging state, all in display (column) space: persist is
// the accumulated envelope, avg the per-column EMA of the trace midpoint in
// px*64 fixed point. Both forget everything on any pan/zoom/scale change -
// the column-to-time mapping they accumulated under is gone.
static uint8_t  g_persist_min[GRID_WIDTH];
static uint8_t  g_persist_max[GRID_WIDTH];
// How brightly this column's envelope still glows, 0 = nothing there. Was a
// plain have/have-not flag; a byte was already being spent on it, so the
// decay mode costs no RAM at all - infinite persistence simply pins it full.
static uint8_t  g_persist_lvl[GRID_WIDTH];
static uint16_t g_persist_ramp[PERSIST_RAMP_STEPS];
static uint32_t g_persist_stamp;
static uint16_t g_avg_acc[GRID_WIDTH];
static uint8_t  g_avg_have[GRID_WIDTH];
static uint32_t g_avg_gen;

// Measurement cursors: SAVE cycles Off -> T1 -> T2 -> V1 -> V2 -> Off. Time
// cursors live in trigger-relative ns and voltage cursors in absolute mV, so
// both stay glued to the signal through pan and zoom exactly like the
// trigger marker does.
int     g_cursor_sel = 0;
static int64_t g_cursor_t[2];
static int     g_cursor_v[2];
static bool    g_cursor_dirty = false;

// Trend view (SHIFT+SAVE): the 1 Hz logger runs regardless; the view plots
// one metric of its ring. EDGE cycles the metric, MODE clears the ring.
bool g_trend_mode = false;

// Roll view: no button turns it on, the timebase does. Past the point where a
// triggered sweep would leave the screen unchanged for seconds at a time, the
// trace becomes a strip chart - the input's peak envelope over one column's
// worth of time, appended at the right, everything else one column older.
//
// The history is kept here rather than in g_display_buffer, and the 900 bytes
// of TCM that costs buy exactly one thing: the strip survives a look at
// something else. The spectrum, the trend and the decoder all own the trace
// columns and write their own picture into them, so a plot that lived there
// was gone the moment any of them was opened - ten minutes of recording lost
// to a glance at the spectrum and back.
//
// Rows, not millivolts, because that is what a column of the display buffer
// is and this is copied straight into one. What that costs is spelled out in
// roll_rescale(), which is why the vertical keys remap the stored rows
// instead of dropping them.
uint8_t g_roll_row_min[GRID_WIDTH];
uint8_t g_roll_row_max[GRID_WIDTH];
uint8_t g_roll_row_flags[GRID_WIDTH];

bool g_roll_active = false;
static int  g_roll_col_us = 0;    // time one display column covers
static int  g_roll_acc_us = 0;    // ...and how much of it is in the open one
static uint32_t g_roll_stamp = 0; // timer_us() at the previous fold
static int  g_roll_min = 255;     // open column's envelope, raw ADC counts
static int  g_roll_max = 0;

static int cursor_t_col(int64_t t_ns);
static int cursor_v_row(int mv);
int roll_row(int raw);
int64_t roll_screen_ns(void);
void roll_sync(void);
static void roll_publish(void);

// One Vpp/DC toggle per hold of the AC/DC key, not one per auto-repeat tick
static bool g_acdc_held = false;


// Not static: the scope's menu toggles it directly (see scope_menu.c). The
// full calibration UI has been in here since the base project, but nothing
// could ever reach it — scope_init() is only ever called with false.
bool scope_calibration_mode = false;
bool g_calibration_dual_channel = false;

bool g_line_owner = false; // measurements hold the status line's end

bool g_fft_mode = false;


                                   // frequencies and the hold restarts

static bool g_autoset_active = false;
static int g_autoset_timer = TIMER_DISABLE;

// Auto-setup phases, in the order autoset_step() walks them
enum
{
  AS_SURVEY,  // park on the long record every slow signal is visible in
  AS_VERT,    // converge the vertical there (amplitude survives undersampling)
  AS_FAST,    // ask the maximum sample rate whether something faster exists
  AS_SLOW,    // one last look through an eight times longer record
  AS_REFINE,  // re-measure on the timebase the estimate asked for
};

// Auto-setup run state, reset in autoset_start()
static int g_autoset_phase = AS_SURVEY;
static int g_autoset_saved_hs = 0;      // settings to restore if nothing is found
static int g_autoset_saved_vs = 0;
static int g_autoset_saved_vpos = 0;
static int g_autoset_zoom_floor = 0;    // no zoom-in below this scale (anti ping-pong)
static int g_autoset_prev_scale = -1;   // scale the last zoom step left behind
static int g_autoset_cent_dir = 1;      // DC-centering direction, flips on divergence
static int g_autoset_cent_prev = AUTOSET_NO_ERR;
static int g_autoset_vert_tries = 0;
static int g_autoset_refine_tries = 0;
static int g_autoset_freq = 0;          // current estimate
static int g_autoset_vamp_mv = 0;       // last measured swing


// The linearity sweep's accumulators. Sums rather than a table of points: the
// fit is three coefficients, and three coefficients need six sums, not 33
// samples kept around in a TCM that has none to spare.

// Held-key coalescing: while a pan key auto-repeats, the DMA restart and
// miniview/sample-rate redraw run only every Nth tick; this timer fires
// once after the key is released to settle the final state
static int g_pan_settle_timer = TIMER_DISABLE;
static int g_pan_repeat_count = 0;

// Displayed-frames-per-second, surfaced in System Info for diagnosing
// "the scope feels slow" reports with a number
static int g_fps_counter = 0;
static int g_fps_value = 0;
static int g_fps_timer = TIMER_DISABLE;


/*
 * Measurements panel state: what the band SAYS, as characters.
 *
 * It used to be a 1bpp mask of the whole band - one bit per trace-area pixel,
 * ~1 KB of it - rebuilt whenever the text changed and read a bit at a time by
 * the column sweep. Storing the text instead and picking the glyph column out
 * of the font while compositing costs the same arithmetic per column, drops the
 * kilobyte, and takes the panel's height out of a struct size: a taller band
 * for the 8x16 font is free, where a taller mask was not (see the linker
 * script's TCM assert).
 */
typedef struct
{
  char text[MPANEL_CHARS_MAX + 1]; // "name value", nothing else
  uint8_t len;
  uint8_t label_len;               // leading characters drawn dimmer: the name
  uint8_t x;                       // first character column of the band's row
  uint8_t row;
  uint16_t color;
} MPanelCell;

bool g_mpanel_active = false;
static bool g_mpanel_force = false;   // rebuild the panel text on the next tick
static uint32_t g_mpanel_builds = 0;  // texts built / bands repainted, for
static uint32_t g_mpanel_paints = 0;  // the System Info diagnostic
static MPanelCell g_mpanel_cell[MPANEL_CELLS_MAX];
static int g_mpanel_cells = 0;
// ...or two lines of free text instead of the grid, for the things the band is
// borrowed to SAY rather than to measure (calibration hints, the
// auto-calibration's report). Sentences, so they keep the small font.
static char g_mpanel_text[2][MPANEL_TEXT_MAX + 2];
static bool g_mpanel_is_text = false;

/*
 * ...or readings placed where the user put them, which is the other layout.
 *
 * Same idea as a band cell and the same painter, with the position in pixels
 * instead of character columns and the font per widget rather than per panel.
 * Built from config.measure_widget[] at the same 2 Hz the band is.
 */
typedef struct
{
  char text[MPANEL_CHARS_MAX + 1];
  uint8_t len;
  uint8_t label_len;
  uint8_t size;                    // 0 = 6x8, 1 = 8x16, 2 = 8x16 doubled
  int16_t x, y;                    // top-left, in trace-area pixels
  uint16_t color;
} PanelPlaced;

static PanelPlaced g_placed[PANEL_WIDGETS_MAX];
static int g_placed_n = 0;

// The layout editor. It owns the screen and the keyboard while it is up: the
// trace is a mock waveform and the readings are mock values, so that arranging
// them is not a moving target and every one of them has something to show
// whatever is on the probe.
bool g_layout_edit = false;
static int g_layout_sel = 0;      // which slot the arrows act on
static bool g_layout_grab = false; // ...and whether they move it or the choice

// Logic decoder view; the run tables live in the spare main-SRAM block
bool g_decode_mode = false;
static bool g_decode_panel_pending = false;
static bool g_decode_force = false;   // run the next decode past the throttle
bool g_logic_have = false;
// The frames on the panel are older than the record on the screen: kept
// because the records since then had nothing in them (see decode_update)
bool g_decode_held = false;
// A record has proved this line really is a UART - it caught the line at
// rest between frames, which no clock and no sine ever does. Until then,
// records that decode as UART but are indistinguishable from a plain square
// wave are not taken, however the protocol was selected.
static bool g_uart_locked = false;
// The timebase has been set from the decoded rate once; it is the user's
// again after that, until the decoder view is re-entered
static bool g_decode_fitted = false;
// Hunting for the head of a message: every record is decoded, not one every
// 250 ms, and the acquisition freezes on the first one that caught the line
// resting before its first frame
static bool g_decode_hunt = false;
// Arm the hunt on the next scope tick. The menu is a screen over the scope
// and the scope is not drawing while it is open, so a menu action leaves the
// work here rather than doing it under the menu.
static bool g_decode_hunt_request = false;
static bool g_spi_clock_request = false;
static bool g_swd_clock_request = false;
LogicResult g_logic;
// The protocol-level read of the same record - which device answered on the
// 1-Wire bus, which CAN frames went past, what the MIDI link said. Kept next
// to the result and not fetched on demand: the decoder's own copy belongs to
// whatever record it was last handed, and the panel can be holding frames
// several records old.
//
// One union, because only one of them is ever live: the snapshot is taken for
// the protocol the record decoded AS, and every reader switches on that same
// protocol before it looks at anything. Five separate copies would hold a
// kilobyte and a half of TCM for four protocols that are not on the screen.
static union
{
  OwAnalysis   ow;
  CanAnalysis  can;    // the CAN frame read, likewise
  DhtAnalysis  dht;    // ...the humidity/temperature one
  SentAnalysis sent;   // ...the automotive sensor one
  MidiAnalysis midi;   // ...the message read of a MIDI link
  LinAnalysis  lin;    // ...the frame read of the car's cheap bus
  Ev1527Analysis ev;   // ...which remote pressed which button
  DshotAnalysis dshot; // ...what the flight controller told the motor
  SpiAnalysis  spi;    // ...what one probe made of a bus it half sees
  ManAnalysis  man;    // ...the bits under a self-clocking line code
  Rc5Analysis  rc5;    // ...which key on a Philips remote went down
  DaliAnalysis dali;   // ...which ballast was told what
  KnxAnalysis  knx;    // ...who told whom on a building's twisted pair
  WsAnalysis   ws;     // ...what colour each pixel of a strip was told to be
  SwoAnalysis  swo;    // ...what a running MCU printed out of its trace pin
  SwdAnalysis  swd;    // ...which registers a debugger was reading
  UsbAnalysis  usb;    // ...which packets went past on one wire of a pair
  PdAnalysis   pd;     // ...and what a charger and its load agreed on
  SircAnalysis sirc;   // ...which key on a Sony remote went down
  PpmAnalysis  ppm;    // ...and every channel of an RC link at once
} g_pana;
int g_decode_sel = 0;              // selected byte (jump target)
static int g_decode_period_ns = 0;        // record metrics the decode ran on
static int g_decode_trig_pos = 0;
int g_decode_size = 0;             // record length the decode ran on

// UART rate table shared with the decoder's menu entry. Index 0 is auto: the
// decoder works the rate out of the record, which is right nearly always and
// wrong exactly when the record holds too few frames to pin it down.
const char *const decoder_baud_labels[DECODER_BAUD_COUNT] =
{
  "Auto", "9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600",
};

const int decoder_baud_values[DECODER_BAUD_COUNT] =
{
  0, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
};

//-----------------------------------------------------------------------------
// Manchester bit rates worth having a name for: RC5 is 1.778 ms a bit, DALI
// 833 us, an EM4100 tag 512 us at the usual RF/64. Index 0 is auto.
const int decoder_man_rate_values[DECODER_MAN_RATE_COUNT] =
{
  0, 500, 562, 1000, 1200, 1953, 2400, 4800, 9600, 19200,
};

const char *const decoder_man_rate_labels[DECODER_MAN_RATE_COUNT] =
{
  "Auto", "500", "562 RC5", "1000", "1200 DALI", "1953 tag", "2400",
  "4800", "9600", "19200",
};

static int decoder_man_rate_value(void)
{
  int i = config.man_rate;

  if (i < 0 || i >= DECODER_MAN_RATE_COUNT)
    return 0;

  return decoder_man_rate_values[i];
}

//-----------------------------------------------------------------------------
static int decoder_baud_value(void)
{
  int i = config.decoder_baud;

  if (i < 0 || i >= DECODER_BAUD_COUNT)
    return 0;

  return decoder_baud_values[i];
}

//-----------------------------------------------------------------------------
// The panel's font and the geometry that follows from it. Everything that
// touches the band asks these rather than reading a constant: the band is
// taller in the 8x16 font, and what stacks on top of it (the decoder's byte
// strip) has to follow.
static const Font *mpanel_font(void)
{
  return (PANEL_FONT_SMALL == config.measure_panel_font) ?
      FONT_SMALL : FONT_LARGE;
}

// ...and how many times over each of its pixels is drawn. There is no 16x32 font
// in the image, so the biggest band is the 8x16 one doubled - see PANEL_FONT_HUGE.
static int mpanel_scale(void)
{
  return (PANEL_FONT_HUGE == config.measure_panel_font) ? 2 : 1;
}

// One glyph cell as drawn, which is what the band measures itself in
static int mpanel_cell_w(void)
{
  return mpanel_font()->width * mpanel_scale();
}

// How many characters of the chosen font fit across the band: 48 in the 6x8,
// 36 in the 8x16, 18 doubled
static int mpanel_chars(void)
{
  return (GRID_WIDTH - 2 * MPANEL_PAD_X) / mpanel_cell_w();
}

static int mpanel_row_h(void)
{
  return mpanel_font()->height * mpanel_scale() + MPANEL_GAP_Y;
}

// Two rows of readings, or one at the doubled size: two would be 72 px of the
// 200 the trace area has, and a band that owns a third of the screen is not a
// band any more. One row of 16x32 is 36 px - the same share the 8x16 band has.
static int mpanel_rows(void)
{
  return (PANEL_FONT_HUGE == config.measure_panel_font) ? 1 : MPANEL_ROWS;
}

// Whether the readings are in the band at the bottom or placed as widgets.
// Free text always takes the band: it is borrowed to say something, and a
// sentence has nowhere else to go.
static bool mpanel_band_mode(void)
{
  return g_mpanel_is_text ||
      PANEL_LAYOUT_WIDGETS != config.measure_layout_mode;
}

// 24 px in the small font, 40 in the large: two rows of glyphs, the gap between
// them, and a margin above and below. Zero in the widget layout, where there is
// no band at all - which is also what gives the decoder's byte strip the whole
// height back (DBAND_BOTTOM follows this).
static int mpanel_h(void)
{
  if (!mpanel_band_mode())
    return 0;

  return 2 * MPANEL_PAD_Y + mpanel_rows() * mpanel_row_h() - MPANEL_GAP_Y;
}

static int mpanel_row0(void)
{
  return GRID_HEIGHT - 1 - mpanel_h();
}


_Static_assert(sizeof(LogicScratch) <= CAPTURE_SPARE_RAM_SIZE,
    "decoder scratch must fit the spare SRAM block");


// ...and on the trace itself, in a band the sweep composites like the
// measurements panel: where each byte begins and ends, and its value written
// in where the byte is wide enough to hold the text. A byte list tells you
// what was said; this tells you where on the waveform it was said.
//
// Two rows where the zoom allows it - the number, and under it what the
// number means as text. Side by side in one row they run together into
// something that has to be parsed rather than read.
//
// The band is bottom-aligned and only as tall as it needs to be: one row
// where the bytes are too narrow to name, none at all where they are too
// narrow even for the number. A band of empty pixels is trace it has taken
// away for nothing.
#define DBAND_BOTTOM   mpanel_row0()
#define DBAND_H1       11   // tint, boundary ticks and one row of text
#define DBAND_H2       20   // ...and the character under it
// ...and a third for what the bytes add up to, where several of them do. The
// rows go from the specific to the general as they get further from the
// trace: the number that was on the wire, then what that byte is, then the
// value its group makes - the way a protocol analyser puts bit0 bit1 bit2
// over one bracket labelled "command".
#define DBAND_H3       29
#define DBAND_TEXT_Y0   1   // text rows inside the band
#define DBAND_TEXT_Y1  10
#define DBAND_TEXT_Y2  19

// The bit grid: a hairline straight across the trace at every bit boundary of
// a decoded byte, and in each cell the number of the bit that landed there.
// The waveform already shows the LEVEL; what it cannot show is which bit of
// which byte that level became, and that is the one thing needed to check a
// decode by eye - especially the bit ORDER, which is the single most
// confusing thing about reading a serial line off a screen.
//
// Drawn OVER everything, trace included: a boundary is a fact about that
// column, and a line that stops at the waveform is a line you cannot follow
// to the edge it is meant to line up with.
//
// But only as TALL as the signal. A hairline from the top of the grid to the
// bottom is a ruler for a screen; a hairline that spans exactly what the
// waveform spans is a ruler for the WAVEFORM, and that is the only thing it
// is ever used to measure against. A two-volt square wave gets two volts of
// line, and the rest of the grid is left alone.
//
// And blended rather than painted: half the column's own colour and half the
// grey. Over black it comes out a whisper, over the trace it tints rather
// than replaces, and nothing the line crosses is ever hidden by it.
#define DBIT_BLEND(px, col)    (uint16_t)(MPANEL_DIM(px) + MPANEL_DIM(col))
#define DBIT_TICK_COLOR        LCD_COLOR(120, 120, 140)
#define DBIT_TEXT_COLOR        LCD_COLOR(140, 140, 150)

// Under this the ticks are a grey wall rather than a grid, and the numbers
// have nowhere to go. Both are checked separately: the ticks alone are worth
// having at a zoom where a character is not.
#define DBIT_MIN_PX            4

static int g_dband_rows = 0;             // 0 = no band; bottom stays put
// Where the band starts, set with its height in decode_band_build(). Not
// DBAND_BOTTOM here: that follows the measurements panel, whose height is a
// setting now and not a constant an initializer could use.
static int g_dband_row0 = 0;
// The character the selection is inside, so its bytes light up together on
// the trace exactly as they do in the panel
static int g_dband_sel_start = 0;
static int g_dband_sel_len = 1;
static int8_t g_dband_byte[GRID_WIDTH];  // decoded byte under each column, -1 none
static uint8_t g_dband_edge[(GRID_WIDTH + 7) / 8];             // byte starts
// ...and where a GROUP starts, which is a subset of them. A byte boundary is
// a fact about the rows that show bytes, and only about those: the bottom row
// holds one value written across every byte of its group, so the boundaries
// INSIDE that group divide nothing there. Ticking them anyway ran a coloured
// hairline down the middle of "#221133" and "S1=543" - the row whose whole
// point is that those bytes are one thing was the row being cut into three.
static uint8_t g_dband_gedge[(GRID_WIDTH + 7) / 8];            // group starts
// Columns no byte covers that were filled in to make a group one strip. The
// fill is for the group's row and for nothing else: the rows that show BYTES
// keep their boundaries exactly as the record drew them, gaps included.
static uint8_t g_dband_gap[(GRID_WIDTH + 7) / 8];
// The first row of the group's value. Above it a tick marks a byte, at it and
// below a tick marks a group; DBAND_H1 has no such row and never reaches it.
static int g_dband_group_y = DBAND_H3;
static uint8_t g_dband_mask[DBAND_H3][(GRID_WIDTH + 7) / 8];   // the text

// The colour swatch beside a group's value, where the value NAMES a colour.
// "#221133" asks the reader to render a colour in their head, which is the
// one thing an instrument that has the colour should never make them do.
//
// A box five pixels wide and one of gutter - exactly one character cell - so
// it costs one character everywhere the fit arithmetic runs, on the trace and
// in the panel alike. Outlined rather than bare: an unlit pixel is #000000,
// and a black square on a dimmed band is indistinguishable from no square at
// all, which is the one reading it must not have.
#define DBAND_SW_W     5
#define DBAND_SW_H     7
#define DBAND_SW_EDGE  LCD_COLOR(160, 160, 170)

// Which columns a swatch covers, and nothing else: the box's own edges are
// its outermost columns, and a column knows it is one by its neighbour not
// being in the box. Boxes are always separated by the text they sit beside,
// so two of them can never touch and be read as one.
static uint8_t g_dband_sw[(GRID_WIDTH + 7) / 8];

static inline bool dband_sw_at(int c)
{
  return (c >= 0 && c < GRID_WIDTH) &&
      ((g_dband_sw[c / 8] >> (c % 8)) & 1);
}

// The colour a group's value IS, for the swatch. Answered by the one protocol
// whose values are colours; everything else has nothing to show.
static bool decode_group_color(int idx, uint16_t *col);

// Column-major, one byte per screen column holding the eight pixels of the
// bit-number row there: the sweep then reads ONE byte per column instead of
// masking eight rows, and it costs the same memory either way.
//
// ...and that memory is the spare SRAM rather than TCM, next to the run
// tables the decoder already keeps there. Both are the same KIND of thing -
// derived from the current record, rebuilt whenever it changes, worthless
// across a reset - and TCM is down to its last few hundred bytes with the
// stack still to come out of it. The region is not linked into, so it holds
// garbage at boot; nothing reads these until decode_band_build has cleared
// them, which is also the only thing that sets g_dbit_on.
typedef struct
{
  uint8_t glyph[GRID_WIDTH];
  uint8_t edge[(GRID_WIDTH + 7) / 8];
} DecodeBitGrid;

#define DBIT ((DecodeBitGrid *)(CAPTURE_SPARE_RAM + sizeof(LogicScratch)))

_Static_assert(sizeof(LogicScratch) + sizeof(DecodeBitGrid) <=
    CAPTURE_SPARE_RAM_SIZE, "decoder scratch overruns the spare SRAM");

static bool g_dbit_on = false;
// The rows the grid lives between, and where the numbers sit inside them.
// Recomputed every frame from the trace's own envelope, because that is what
// they follow - the decode changes rarely, the signal's height every frame.
static int g_dbit_top = 0;
static int g_dbit_bot = -1;
// Middle of the grid until a decode places it; not GRID_HEIGHT/2-4 written out
// here, because the grid's height is a runtime value now and this is a file
// scope initialiser. decode_bit_rows() sets it before anything reads it.
static int g_dbit_text_row = GRID_HEIGHT_MAX / 2 - 4;

_Static_assert(LOGIC_MAX_BYTES <= 127, "byte index must fit g_dband_byte");
_Static_assert(DBAND_TEXT_Y1 + 8 <= DBAND_H2, "second text row must fit the band");
_Static_assert(DBAND_TEXT_Y2 + 8 <= DBAND_H3, "third text row must fit the band");

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void grid_init(void)
{
  // Setup column data
  for (int y = 0; y < 200; y++)
    g_grid_column_0[y] = GRID_BG_COLOR;

  for (int y = 0; y < 200; y++)
    g_grid_column_1[y] = (4 == (y % 5)) ? GRID_FG_COLOR : GRID_BG_COLOR;

  for (int y = 0; y < 200; y++)
    g_grid_column_2[y] = (24 == (y % 25)) ? GRID_FG_COLOR : GRID_BG_COLOR;
  g_grid_column_2[0]   = GRID_FG_COLOR;
  g_grid_column_2[1]   = GRID_FG_COLOR;
  g_grid_column_2[98]  = GRID_FG_COLOR;
  g_grid_column_2[100] = GRID_FG_COLOR;
  g_grid_column_2[197] = GRID_FG_COLOR;
  g_grid_column_2[198] = GRID_FG_COLOR;

  for (int y = 0; y < 200; y++)
    g_grid_column_3[y] = (4 == (y % 5)) ? GRID_FG_COLOR : GRID_BG_COLOR;
  g_grid_column_3[0]   = GRID_FG_COLOR;
  g_grid_column_3[1]   = GRID_FG_COLOR;
  g_grid_column_3[2]   = GRID_FG_COLOR;
  g_grid_column_3[98]  = GRID_FG_COLOR;
  g_grid_column_3[100] = GRID_FG_COLOR;
  g_grid_column_3[196] = GRID_FG_COLOR;
  g_grid_column_3[197] = GRID_FG_COLOR;
  g_grid_column_3[198] = GRID_FG_COLOR;

  for (int y = 0; y < 200; y++)
    g_grid_column_4[y] = (24 == (y % 25)) ? GRID_FG_COLOR : GRID_BG_COLOR;

  // Assign columns
  for (int x = 0; x < 299; x++)
    g_grid_index[x] = 0;

  for (int x = 4; x < 299; x += 5)
    g_grid_index[x] = 2;

  for (int x = 24; x < 299; x += 25)
    g_grid_index[x] = 3;

  g_grid_index[0]   = 1;
  g_grid_index[1]   = 1;
  g_grid_index[2]   = 4;
  g_grid_index[148] = 1;
  g_grid_index[150] = 1;
  g_grid_index[296] = 4;
  g_grid_index[297] = 1;
  g_grid_index[298] = 1;
}

//-----------------------------------------------------------------------------
static bool trace_ready(void)
{
  return (g_trace_column == (GRID_WIDTH-1));
}

//-----------------------------------------------------------------------------
void redraw_trace(void)
{
  if (trace_ready())
  {
    g_trace_column = 0;
    // The trigger-position overlay is composited into the columns, so a
    // moved marker invalidates the whole sweep, not just changed data
    g_sweep_force = !g_shadow_valid || (config.horizontal_position_px != g_shadow_marker_px);
    g_shadow_marker_px = config.horizontal_position_px;
  }
}

//-----------------------------------------------------------------------------
static void update_column_from_image(int index, uint16_t *column, int x, int y, const Image *image)
{
  int image_col = index - x + image->ox + 1;

  if (image_col < 0 || image_col > (image->width-1))
    return;

  for (int i = 0; i < image->height; i++)
  {
    int color = image->data[i * image->width + image_col];

    if (color)
      column[y + i] = color;
  }
}

//-----------------------------------------------------------------------------
// The phosphor ramp: PERSIST_GLOW_COLOR down to black, in equal steps.
// Built rather than written out because it is indexed by a brightness that
// falls linearly with time, so the steps have to be linear too - a hand
// picked set of dim yellows would decay in visible jumps.
static void persist_build_ramp(void)
{
  for (int i = 0; i < PERSIST_RAMP_STEPS; i++)
  {
    // i = 0 is the last step before nothing at all, so it is not black: a
    // glow that reaches zero is dropped, not drawn
    int n = i + 1;

    g_persist_ramp[i] = LCD_COLOR(PERSIST_GLOW_R * n / PERSIST_RAMP_STEPS,
        PERSIST_GLOW_G * n / PERSIST_RAMP_STEPS,
        PERSIST_GLOW_B * n / PERSIST_RAMP_STEPS);
  }
}

//-----------------------------------------------------------------------------
// One edge of the decaying envelope, in this column, drawn as a piece of a
// CURVE rather than as a dot.
//
// The two edges are where the beam has been, so they are what a phosphor
// leaves behind - filling everything between them turns a trail into a solid
// olive polygon, which is what the first version did and what nobody wants to
// look at. But a single pixel per column is not a curve either: on a steep
// edge the envelope's top moves a long way from one column to the next, and
// the marks come out as a row of dots with gaps between them.
//
// So each column reaches halfway to whichever neighbours still glow - the
// same halves-meet-in-the-middle trick close_gaps() plays on the trace, and
// it makes the two ends of the join land on the same pixel.
static void persist_edge(uint16_t *column, const uint8_t *edge, int c,
    uint16_t glow)
{
  int top = edge[c], bot = edge[c];

  for (int n = -1; n <= 1; n += 2)
  {
    int i = c + n, mid;

    if (i < 0 || i >= GRID_WIDTH || !g_persist_lvl[i])
      continue; // nothing glowing there to join up with

    mid = (edge[c] + edge[i]) / 2;

    if (mid < top)
      top = mid;

    if (mid > bot)
      bot = mid;
  }

  for (int y = top; y <= bot; y++)
    column[y] = glow;
}

//-----------------------------------------------------------------------------
static void update_from_display_buffer(uint16_t *column, DisplayBuffer *db)
{
  bool clip_h = db->flags[g_trace_column] & SAMPLE_FLAG_CLIP_H;
  bool clip_l = db->flags[g_trace_column] & SAMPLE_FLAG_CLIP_L;
  int color;

  // The accumulated envelope first, so the live trace paints over it. This
  // is how a runt that fired once stays visible: the envelope holds every
  // level any frame ever reached in this column.
  if (config.persist_mode != PERSIST_OFF && g_persist_lvl[g_trace_column])
  {
    int c = g_trace_column;

    if (config.persist_mode == PERSIST_DECAY)
    {
      // The trail: two fading curves, the highest and the lowest the beam
      // has lately been. Not the area between them - see persist_edge.
      uint16_t glow = g_persist_ramp[g_persist_lvl[c] * PERSIST_RAMP_STEPS / 256];

      persist_edge(column, g_persist_min, c, glow);
      persist_edge(column, g_persist_max, c, glow);
    }
    else
    {
      // Infinite is a different question and keeps its answer: everywhere
      // the signal has EVER been, as an area, so a runt that fired once is
      // a mark you cannot miss rather than a hairline
      for (int y = g_persist_min[c]; y <= g_persist_max[c]; y++)
        column[y] = TRACE_PERSIST_COLOR;
    }
  }

  if (db->flags[g_trace_column] & SAMPLE_FLAG_VALID)
  {
    if (clip_h || clip_l)
      color = TRACE_CLIP_COLOR;
    else if (db->flags[g_trace_column] & SAMPLE_FLAG_FILLED)
      color = g_recon_strained ? TRACE_RECON_WARN_COLOR : TRACE_FILLED_COLOR;
    else
      color =  TRACE_COLOR;

    for (int y = db->min[g_trace_column]; y <= db->max[g_trace_column]; y++)
      column[y] = color;
  }
  else
  {
    column[GRID_HEIGHT/2-1] = TRACE_INVALID_COLOR;
  }

  if (clip_h)
  {
    for (int i = 0; i < 3; i++)
      column[i] = TRACE_CLIP_COLOR;
  }

  if (clip_l)
  {
    for (int i = 0; i < 3; i++)
      column[GRID_HEIGHT-2 - i] = TRACE_CLIP_COLOR;
  }
}

//-----------------------------------------------------------------------------
// One character column of one glyph into the column buffer: `bit` says which of
// the glyph's columns this is, and the caller has already worked out that this
// trace column falls inside the character. Anything outside the font - the
// half-space the value formatters use before a unit - paints nothing, which is
// what a blank is: the layout has already reserved its width.
/*
 * The three sizes a placed reading comes in, out of the two bits its flags keep
 * them in: the 6x8 font, the 8x16 font, and the 8x16 font doubled to 16x32.
 *
 * A 16x32 reading is four glyphs' worth of screen for one, and it exists for one
 * reason: it is bigger than anything the top and bottom bars can hold. Those are
 * 20 and 16 pixels tall and packed end to end, so 8x16 is their ceiling forever -
 * and the trace area is 300x200 with a compositor that can put text anywhere in
 * it. "The bar values are too small" is answered here, not in the bars.
 */
static int widget_size_of(uint8_t flags)
{
  if (flags & PW_HUGE)
    return 2;

  return (flags & PW_LARGE) ? 1 : 0;
}

static const Font *widget_font(int size)
{
  return size ? FONT_LARGE : FONT_SMALL;
}

static int widget_scale(int size)
{
  return (size > 1) ? 2 : 1;
}

/*
 * Vertical distances are stored against the BIGGEST trace area there is, not
 * against whatever one is on screen, and these two are the conversion.
 *
 * The area is 200 px at the normal text size and 175 at the large one, so a
 * distance kept in plain pixels means a different fraction of the screen in each
 * - and a layout arranged in one of them arrives in the other with its spacing
 * wrong. What that looks like is two readings, one measured from the top and one
 * from the bottom, closing 25 px on each other and ending up on top of one
 * another: the pair did not move WITH the area, so the area moved through them.
 *
 * Normalised, the pair keeps its share of the height and its shape. Layouts
 * saved before this need no migration: they were written on the 200 px area,
 * which is exactly what they are now read against.
 *
 * Horizontally there is nothing to do - the trace area is 300 px wide at every
 * size, and the bars stretch rather than the graticule.
 */
static int widget_y_px(int stored)
{
  return stored * PANEL_WIDGET_STEP * GRID_HEIGHT / GRID_HEIGHT_MAX;
}

static int widget_y_store(int px)
{
  return px * GRID_HEIGHT_MAX / GRID_HEIGHT / PANEL_WIDGET_STEP;
}

//-----------------------------------------------------------------------------
// Where a stored widget actually IS, given how big its text came out.
//
// The stored position is a distance from a CORNER (see PanelWidget), so this is
// where the two meet: the size is known here and nowhere else, and a right- or
// bottom-anchored reading is placed by its own right or bottom edge. Which is
// what keeps a reading that grew from moving, and a reading in the bottom right
// corner from growing off the screen. Clamped, because a layout arranged at one
// size and read back at another can name a position that no longer exists.
static void widget_pos(const PanelWidget *w, int wide, int gh, int *px, int *py)
{
  int max_x = GRID_WIDTH - 2 - wide;
  int max_y = GRID_HEIGHT - 2 - gh;
  int x = (w->flags & PW_ANCHOR_RIGHT) ?
      (GRID_WIDTH - 2 - wide - w->x * PANEL_WIDGET_STEP) :
      (w->x * PANEL_WIDGET_STEP);
  int y = (w->flags & PW_ANCHOR_BOTTOM) ?
      (GRID_HEIGHT - 2 - gh - widget_y_px(w->y)) :
      widget_y_px(w->y);

  *px = (x < 0) ? 0 : ((x > max_x) ? ((max_x < 0) ? 0 : max_x) : x);
  *py = (y < 0) ? 0 : ((y > max_y) ? ((max_y < 0) ? 0 : max_y) : y);
}

//-----------------------------------------------------------------------------
// ...and the way back: store an absolute top-left, measured from whichever
// corner the reading is nearest. Nothing asks the user which corner - the corner
// is where they put it, so it follows from the position on every move.
static void widget_set_pos(PanelWidget *w, int x, int y, int wide, int gh)
{
  bool right  = (x + wide / 2) > GRID_WIDTH / 2;
  bool bottom = (y + gh / 2) > GRID_HEIGHT / 2;
  int dx = right ? (GRID_WIDTH - 2 - wide - x) : x;
  int dy = bottom ? (GRID_HEIGHT - 2 - gh - y) : y;

  if (dx < 0)
    dx = 0;
  if (dy < 0)
    dy = 0;

  w->flags &= (uint8_t)~PW_ANCHOR_MASK;
  w->flags |= (uint8_t)((right ? PW_ANCHOR_RIGHT : 0) |
      (bottom ? PW_ANCHOR_BOTTOM : 0));
  w->x = (uint8_t)(dx / PANEL_WIDGET_STEP);
  w->y = (uint8_t)widget_y_store(dy);
}

//-----------------------------------------------------------------------------
// `scale` repeats each pixel row that many times, which is how a widget gets a
// 16x32 glyph out of an 8x16 font. Doubling and not a second font: there is no
// 16x32 font in the image and one would cost 3 KB of flash for the ninety-five
// characters, where this costs a multiply. It is a chunky glyph read from a
// bench, not typography - and it is the only size the two bars cannot hold, so
// it is the whole reason the readings can leave them.
__attribute__((noinline))
static void mpanel_glyph_column(uint16_t *column, int row0, const Font *font,
    char ch, int bit, uint16_t color, int scale)
{
  const uint8_t *bitmap;

  if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
    return;

  bitmap = font->data + (ch - FONT_FIRST_CHAR) * font->pitch;

  for (int y = 0; y < font->height; y++)
  {
    int i = y * font->width + bit;

    if (!((bitmap[i / 8] >> (i % 8)) & 1))
      continue;

    for (int s = 0; s < scale; s++)
      column[row0 + y * scale + s] = color;
  }
}

//-----------------------------------------------------------------------------
// Everything the band says, for ONE trace column. Two shapes: the grid of
// reading cells, or two lines of free text. Both are stored as characters and
// turned into pixels here, inside the column buffer, so the panel is composed
// by the same sweep that draws the trace and can never flicker against it.
//-----------------------------------------------------------------------------
// The placed readings, for one trace column: each widget's own background over
// its own box, then its text. Same painter as the band, and for the same
// reason - a reading composited by the sweep cannot flicker against the trace.
//
// The selection frame belongs to the editor and is drawn here too, because here
// is where this column's pixels are.
__attribute__((noinline))
static void widgets_paint_column(int c, uint16_t *column)
{
  for (int i = 0; i < g_placed_n; i++)
  {
    const PanelPlaced *w = &g_placed[i];
    const Font *font = widget_font(w->size);
    int sc = widget_scale(w->size);
    int gw = font->width * sc;      // one glyph cell, as drawn
    int gh = font->height * sc;
    int at = c - w->x;
    int wide = w->len * gw;
    bool sel = g_layout_edit && i == g_layout_sel;
    int ch_i, bit;
    uint16_t color;

    // One pixel of air around the text, which is also what the frame and the
    // solid background cover
    if (at < -1 || at > wide)
      continue;

    for (int y = w->y - 1; y <= w->y + gh; y++)
    {
      if (y < 0 || y >= GRID_HEIGHT - 1)
        continue;

      if (PANEL_BG_SOLID == config.measure_panel_bg || sel)
        column[y] = BG_COLOR;
      else if (PANEL_BG_OFF != config.measure_panel_bg)
        column[y] = MPANEL_DIM(column[y]);
    }

    // The frame: the box's edge, so a grabbed widget reads as held
    if (sel)
    {
      uint16_t edge = g_layout_grab ? MEASURE_VOLTAGE_COLOR : MPANEL_EDGE_COLOR;
      int top = w->y - 1;
      int bot = w->y + gh;

      if (-1 == at || at == wide)
      {
        for (int y = top; y <= bot; y++)
        {
          if (y >= 0 && y < GRID_HEIGHT - 1)
            column[y] = edge;
        }
      }
      else
      {
        if (top >= 0)
          column[top] = edge;

        if (bot < GRID_HEIGHT - 1)
          column[bot] = edge;
      }
    }

    if (at < 0 || at >= wide)
      continue;

    ch_i = at / gw;
    bit = (at % gw) / sc;
    color = (ch_i < w->label_len) ? MPANEL_DIM(w->color) : w->color;

    mpanel_glyph_column(column, w->y, font, w->text[ch_i], bit, color, sc);
  }
}

__attribute__((noinline))
static void mpanel_paint_column(int c, uint16_t *column)
{
  int row0 = mpanel_row0() + MPANEL_PAD_Y;
  int x = c - MPANEL_PAD_X;

  if (x < 0)
    return;

  if (g_mpanel_is_text)
  {
    const Font *font = FONT_SMALL;
    int ch_i = x / font->width;
    int bit = x % font->width;

    if (ch_i >= MPANEL_TEXT_MAX)
      return;

    for (int r = 0; r < 2; r++)
      mpanel_glyph_column(column, row0 + r * (font->height + MPANEL_GAP_Y),
          font, g_mpanel_text[r][ch_i], bit, LCD_WHITE_COLOR, 1);

    return;
  }

  {
    const Font *font = mpanel_font();
    int sc = mpanel_scale();
    int cw = font->width * sc;
    int ch_i = x / cw;
    int bit = (x % cw) / sc;

    // At most one pair per row covers this character, and there are never more
    // than a handful of them - a scan is cheaper than a map to look it up in
    for (int i = 0; i < g_mpanel_cells; i++)
    {
      const MPanelCell *cell = &g_mpanel_cell[i];
      int at = ch_i - cell->x;
      uint16_t color;

      if (at < 0 || at >= cell->len)
        continue;

      // The name dimmer than the number it names: the reading is what the eye
      // should land on, and a label at full brightness competes with it
      color = (at < cell->label_len) ? MPANEL_DIM(cell->color) : cell->color;

      mpanel_glyph_column(column, row0 + cell->row * mpanel_row_h(), font,
          cell->text[at], bit, color, sc);
    }
  }
}

//-----------------------------------------------------------------------------
// Spectrum column: connected curve (min..max span, like the waveform trace)
// over the gradient fill. Peaks and the cursor ride the flags bits (set in
// fft_update), so the dirty-column compare repaints exactly the columns
// whose marker state changed.
static void update_from_spectrum(uint16_t *column, DisplayBuffer *db)
{
  int c = g_trace_column;
  int flags = db->flags[c];
  int top = db->min[c];
  int bottom = db->max[c];

  // Cursor: a hairline dropping from the top of the grid onto the curve
  if (flags & SAMPLE_FLAG_FILLED)
  {
    for (int y = 0; y < top; y++)
      column[y] = FFT_CURSOR_COLOR;
  }

  // Heat-gradient fill under the curve
  for (int y = bottom + 1; y <= GRID_HEIGHT-2; y++)
    column[y] = g_fft_grad[GRID_HEIGHT-2 - y];

  // The curve itself, one extra pixel below the span for weight
  for (int y = top; y <= bottom + 1 && y <= GRID_HEIGHT-2; y++)
    column[y] = FFT_LINE_COLOR;

  // Peak marker: a tick floating above the curve
  if (flags & (SAMPLE_FLAG_CLIP_H | SAMPLE_FLAG_CLIP_L))
  {
    int color = (flags & SAMPLE_FLAG_CLIP_H) ? FFT_PEAK_H_COLOR : FFT_PEAK_X_COLOR;

    for (int y = top - 6; y <= top - 3; y++)
    {
      if (y >= 0)
        column[y] = color;
    }
  }
}

//-----------------------------------------------------------------------------
// Compose the full pixel column c: grid, trace, overlays, translucent
// measurements panel and the decoder byte strip
void build_trace_column(int c, uint16_t *column)
{
  // The background the rest of this composites over: this column's share of
  // the graticule, or pattern 0 - the one with no ruling in it, which every
  // column between the divisions already uses - when the grid is switched off.
  // Nothing below has to know which it got, and a blank screen costs exactly
  // what a ruled one does.
  const uint16_t *grid = g_grid_columns[config.grid_mode ? 0 : g_grid_index[c]];

  for (int i = 0; i < GRID_HEIGHT; i++)
    column[i] = grid[i];

  if (g_fft_mode)
    update_from_spectrum(column, &g_display_buffer);
  else
    update_from_display_buffer(column, &g_display_buffer);

  // The trigger-position overlay is meaningless on the spectrum, the trend
  // and the roll strip - none of them is drawn around a trigger
  if (!g_fft_mode && !g_trend_mode && !g_roll_active)
  {
    if (config.horizontal_position_px < -(GRID_WIDTH/2-1))
    {
      update_column_from_image(c, column, GRID_WIDTH-1, 1, &image_trigger_offset_right);
    }
    else if (config.horizontal_position_px > (GRID_WIDTH/2-1))
    {
      update_column_from_image(c, column, 1, 1, &image_trigger_offset_left);
    }
    else
    {
      int pos = GRID_WIDTH/2 - config.horizontal_position_px;
      update_column_from_image(c, column, pos, 0, &image_trigger_offset);
    }
  }

  // Measurement cursors ride the sweep like every other overlay: the pair of
  // time hairlines and the pair of voltage lines, the active one solid, its
  // partner dashed
  if (g_cursor_sel && !g_fft_mode && !g_trend_mode && !g_roll_active)
  {
    for (int k = 0; k < 2; k++)
    {
      int row = cursor_v_row(g_cursor_v[k]);

      if ((g_cursor_sel == 3 + k) || (c & 3) < 2)
        column[row] = CURSOR_V_COLOR;
    }

    for (int k = 0; k < 2; k++)
    {
      if (c == cursor_t_col(g_cursor_t[k]))
      {
        bool solid = (g_cursor_sel == 1 + k);

        for (int y = 0; y < GRID_HEIGHT-1; y++)
        {
          if (solid || (y & 3) < 2)
            column[y] = CURSOR_T_COLOR;
        }
      }
    }
  }

  // Bit grid: a hairline straight across at every bit boundary, and the bit's
  // number in the cell. Over everything, waveform included - a boundary is a
  // fact about that column, and a line broken wherever the trace crosses it
  // is a line the eye cannot follow to the edge it is meant to line up with.
  if (g_dbit_on && g_dbit_bot >= g_dbit_top)
  {
    if ((DBIT->edge[c / 8] >> (c % 8)) & 1)
    {
      for (int y = g_dbit_top; y <= g_dbit_bot; y++)
        column[y] = DBIT_BLEND(column[y], DBIT_TICK_COLOR);
    }

    for (int y = 0; y < 8; y++)
    {
      if ((DBIT->glyph[c] >> y) & 1)
        column[g_dbit_text_row + y] = DBIT_TEXT_COLOR;
    }
  }

  // Decoder band: the same trick, a couple of rows of it. Under every column
  // that a decoded byte covers the trace is dimmed and tinted in the byte's
  // colour, the column where a frame starts gets a full-height tick, and the
  // value is written in wherever it fits.
  //
  // In two steps, and the order is the whole point: everything that marks
  // where a byte IS goes down first, and the text goes over it. A tick and a
  // glyph want the same pixel often enough that whichever is written second
  // wins, and the one worth keeping is never the tick.
  if (g_dband_rows && g_dband_byte[c] >= 0)
  {
    int b = g_dband_byte[c];
    bool grouped = (g_dband_sel_len > 1 && b >= g_dband_sel_start &&
        b < g_dband_sel_start + g_dband_sel_len);
    uint16_t tint = (b == g_decode_sel) ? DSTRIP_SEL :
        grouped ? DSTRIP_GROUP :
        ((b & 1) ? DSTRIP_ODD : DSTRIP_EVEN);
    bool edge = (g_dband_edge[c / 8] >> (c % 8)) & 1;
    bool gedge = (g_dband_gedge[c / 8] >> (c % 8)) & 1;
    bool gap = (g_dband_gap[c / 8] >> (c % 8)) & 1;

    for (int y = 0; y < g_dband_rows; y++)
    {
      int row = g_dband_row0 + y;

      // A filled gap is the group's business alone. Above the group's row the
      // column is left exactly as it was before anything was filled - no byte
      // covers it, so no byte claims it.
      if (gap && y < g_dband_group_y)
        continue;
      // Byte boundaries divide the rows that show bytes; the row that shows
      // what a whole group came to is divided by the group's edges alone
      bool tick = (y < g_dband_group_y) ? edge : gedge;

      if (tick || y == g_dband_rows - 1)
        column[row] = tint;                    // boundary / baseline
      else
        column[row] = MPANEL_DIM(column[row]); // room for the text to read

      // The colour itself, beside the hex that names it
      if (dband_sw_at(c) && y >= g_dband_group_y &&
          y < g_dband_group_y + DBAND_SW_H)
      {
        bool frame = !dband_sw_at(c - 1) || !dband_sw_at(c + 1) ||
            y == g_dband_group_y || y == g_dband_group_y + DBAND_SW_H - 1;
        uint16_t swc;

        if (frame || !decode_group_color(b, &swc))
          swc = DBAND_SW_EDGE;

        column[row] = swc;
      }
    }
  }

  // ...and the text over all of it, whether or not a byte covers this column.
  // Bytes do not touch: a WS2812 byte ends on its last bit's high half and
  // the next begins after the low tail, so a group's value - written once
  // across every byte it took - runs over columns no byte covers. Gating the
  // whole band on coverage took the glyphs in those columns out with the
  // tint, and the value came out sliced at each byte boundary.
  if (g_dband_rows)
  {
    for (int y = 0; y < g_dband_rows; y++)
    {
      if (g_dband_mask[y][c / 8] & (1 << (c % 8)))
        column[g_dband_row0 + y] = LCD_WHITE_COLOR;
    }
  }

  // The measurements panel: its background, then its text, all inside the
  // column buffer, so the panel is part of the normal sweep and can never
  // flicker or be painted over.
  //
  // Which background is the user's call. Dimming the trace keeps the waveform
  // visible through the readings and is what this has always done; a solid band
  // is the one that stays legible over a bright, busy trace, which is exactly
  // when the numbers matter; and nothing at all is for reading the waveform
  // under them with the numbers still there.
  if (g_mpanel_active)
  {
    if (mpanel_band_mode())
    {
      int h = mpanel_h();
      int row0 = mpanel_row0();

      for (int y = 0; y < h; y++)
      {
        uint16_t *px = &column[row0 + y];

        if (PANEL_BG_SOLID == config.measure_panel_bg)
          *px = (0 == y) ? MPANEL_EDGE_COLOR : BG_COLOR;
        else if (PANEL_BG_OFF != config.measure_panel_bg)
          *px = MPANEL_DIM(*px);
      }

      mpanel_paint_column(c, column);
    }
    else
    {
      widgets_paint_column(c, column);
    }
  }
}

//-----------------------------------------------------------------------------
static void draw_trace(void)
{
  uint16_t column[GRID_HEIGHT_MAX];
  int c = g_trace_column;

  if (trace_ready())
    return;

  if (!g_sweep_force &&
      g_display_buffer.min[c] == g_shadow_min[c] &&
      g_display_buffer.max[c] == g_shadow_max[c] &&
      g_display_buffer.flags[c] == g_shadow_flags[c])
  {
    g_trace_column++;

    if (g_trace_column == (GRID_WIDTH-1))
      g_shadow_valid = true;

    return;
  }

  build_trace_column(c, column);

  // An opaque panel is a hole in the sweep: columns it covers paint only
  // the part below it, so the panel pixels are never touched
  int hole = 0;

  if (g_decode_mode && c >= DECODE_PANEL_COL0 && c <= DECODE_PANEL_COL1)
    hole = DECODE_PANEL_H;
  else if (g_fft_mode && g_fft_panel_on && c >= FFT_PANEL_COL0 && c <= FFT_PANEL_COL1)
    hole = FFT_PANEL_H;
  else if (g_snap_tag && c >= SNAP_TAG_COL0 && c <= SNAP_TAG_COL1)
    hole = SNAP_TAG_H;

  if (hole)
    lcd_draw_buf(GRID_LEFT+1 + c, GRID_TOP+1 + hole, 1,
        GRID_HEIGHT-1 - hole, &column[hole]);
  else
    lcd_draw_buf(GRID_LEFT+1 + c, GRID_TOP+1, 1, GRID_HEIGHT-1, column);

  g_shadow_min[c]   = g_display_buffer.min[c];
  g_shadow_max[c]   = g_display_buffer.max[c];
  g_shadow_flags[c] = g_display_buffer.flags[c];

  g_trace_column++;

  if (g_trace_column == (GRID_WIDTH-1))
    g_shadow_valid = true;
}

//-----------------------------------------------------------------------------
// Compose one pair: the name, one space, the value. The value keeps the leading
// spaces its formatter padded it with - that padding is what holds a metric's
// width constant, and a constant width is why nothing in the band moves when a
// reading changes.
static void mpanel_cell_set(MPanelCell *cell, const char *label,
    const char *value, uint16_t color)
{
  int n = snprintf(cell->text, sizeof(cell->text), "%s%s%s",
      label, label[0] ? " " : "", value);

  if (n < 0)
    n = 0;
  else if (n > (int)sizeof(cell->text) - 1)
    n = (int)sizeof(cell->text) - 1;

  cell->len = (uint8_t)n;
  cell->label_len = (uint8_t)strlen(label);
  cell->color = color;
  cell->x = 0;
  cell->row = 0;
}

//-----------------------------------------------------------------------------
// Is what was decoded meant to be read as text? Two of the protocols carry
// it and the rest do not, and UTF-8 grouping on the rest is not merely
// useless but wrong: two WS2812 colour bytes are not one Cyrillic letter
// because the first of them happens to begin with 110.
static bool decode_proto_is_text(void)
{
  return (g_logic.proto == PROTO_UART || g_logic.proto == PROTO_RAW);
}

//-----------------------------------------------------------------------------
// A serial line underneath: 8N1 frames with the line idle between them. MIDI
// is one - it is a UART at 31250 baud with a grammar on top - and everything
// that depends on there being an idle level applies to it unchanged: fitting
// the window to the rate, triggering on the falling edge a start bit is, and
// asking whether the record caught the line at REST before its first byte or
// landed in the middle of a message.
static bool decode_proto_is_serial(proto_t proto)
{
  // SWO joins them: whatever the packets mean, the wire is 8N1 idling high,
  // so fitting the window to the rate and waiting for a line at rest work on
  // it exactly as they do on a console
  return (proto == PROTO_UART || proto == PROTO_MIDI || proto == PROTO_SWO);
}

//-----------------------------------------------------------------------------
// Which bytes make up ONE value with byte `idx`. UTF-8 has been answering
// this since it was added - two bytes, one letter, lit together with the code
// point written once across them - and every protocol that assembles a number
// out of several bytes needs the same answer: three SENT nibbles are one
// signal, four CAN bytes are one identifier, two DHT bytes are one reading.
// Numbering them D1 D2 D3 and putting the value under the last leaves the
// reader to work out that the first two belong to it.
static void decode_group_at(int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= g_logic.count)
    return;

  if (decode_proto_is_text())
  {
    utf8_group_at(g_logic.bytes, g_logic.count, idx, start, len);
  }
  else
  {
    switch (g_logic.proto)
    {
      case PROTO_SENT:    sent_group_at(&g_pana.sent, idx, start, len); break;
      case PROTO_CAN:     can_group_at(&g_pana.can, idx, start, len); break;
      case PROTO_DHT:     dht_group_at(&g_pana.dht, idx, start, len); break;
      case PROTO_ONEWIRE: onewire_group_at(&g_pana.ow, idx, start, len); break;
      case PROTO_MIDI:    midi_group_at(&g_pana.midi, idx, start, len); break;
      case PROTO_EV1527:  ev1527_group_at(&g_pana.ev, idx, start, len); break;
      case PROTO_DSHOT:   dshot_group_at(&g_pana.dshot, idx, start, len); break;
      case PROTO_SPI:     spi_group_at(&g_pana.spi, idx, start, len); break;
      case PROTO_MANCH:   manchester_group_at(&g_pana.man, idx, start, len); break;
      case PROTO_RC5:     rc5_group_at(&g_pana.rc5, idx, start, len); break;
      case PROTO_DALI:    dali_group_at(&g_pana.dali, idx, start, len); break;
      case PROTO_KNX:     knx_group_at(&g_pana.knx, idx, start, len); break;
      case PROTO_WS2812:  ws2812_group_at(&g_pana.ws, idx, start, len); break;
      case PROTO_SWO:     swo_group_at(&g_pana.swo, idx, start, len); break;
      case PROTO_SWD:     swd_group_at(&g_pana.swd, idx, start, len); break;
      case PROTO_USB:     usb_group_at(&g_pana.usb, idx, start, len); break;
      case PROTO_PD:      pd_group_at(&g_pana.pd, idx, start, len); break;
      case PROTO_SIRC:    sirc_group_at(&g_pana.sirc, idx, start, len); break;
      // PPM is deliberately absent: its channels each stand alone, and a
      // frame of eight of them is not one value written across eight bytes
      default: break;
    }
  }

  // The group belongs to the result, not past the end of it
  if (*start < 0)
    *start = 0;

  if (*start > idx)
    *start = idx;

  if (*start + *len > g_logic.count)
    *len = g_logic.count - *start;

  if (*len < 1)
  {
    *start = idx;
    *len = 1;
  }
}

//-----------------------------------------------------------------------------
// The colour byte `idx`'s group came to, where its value is a colour at all.
// A WS2812 pixel is the only value any of these decoders reports that has
// one; a CAN identifier and a temperature do not, and inventing a colour for
// them would be decoration rather than information.
static bool decode_group_color(int idx, uint16_t *col)
{
  if (g_logic.proto != PROTO_WS2812 || idx < 0 || idx >= g_logic.count)
    return false;

  uint8_t p = g_pana.ws.pix[idx];

  // A pixel the frame cut short has three bytes and no colour
  if (p >= WS_MAX_PIXELS)
    return false;

  *col = LCD_COLOR(g_pana.ws.rgb[p][0], g_pana.ws.rgb[p][1],
      g_pana.ws.rgb[p][2]);

  return true;
}

//-----------------------------------------------------------------------------
// What a byte means as text, for the row under the number: the character
// itself, or a name where it has no printable shape. A serial line is read as
// text, and the bytes that decide how that text lays out - CR, LF, TAB - are
// precisely the ones with nothing to show for themselves. SP is named too: a
// blank under a number reads as a rendering fault, not as a space.
//
// An empty string for the rest. Inventing a look for byte 0x9B helps nobody,
// and the number above it has already said everything there is to say.
static void dband_ascii_text(char *buf, int size, uint8_t v)
{
  static const struct { uint8_t v; const char *name; } named[] =
  {
    { 0x00, "NUL" }, { 0x07, "BEL" }, { 0x08, "BS" },  { 0x09, "TAB" },
    { 0x0A, "LF" },  { 0x0D, "CR" },  { 0x1B, "ESC" }, { 0x20, "SP" },
    { 0x7F, "DEL" },
  };

  for (int i = 0; i < ARRAY_SIZE(named); i++)
  {
    if (named[i].v == v)
    {
      snprintf(buf, size, "%s", named[i].name);
      return;
    }
  }

  if (v > 32 && v < 127)
    snprintf(buf, size, "%c", v);
  else
    buf[0] = 0;
}

//-----------------------------------------------------------------------------
// What byte number `idx` MEANS in the protocol it came from, for the row
// under the number. Every decoder puts bytes on the trace; what a byte is
// worth saying about it depends entirely on which one.
//
// A character is the answer for the two protocols that carry text and for no
// others: on a WS2812 strip byte 1 is how green the first pixel is, and
// printing 'A' under it because 0x41 happens to be a letter is noise dressed
// as information.
static void dband_meaning_text(char *buf, int size, int idx, uint8_t v)
{
  static const char *const nec_field[4] = { "ADDR", "~ADDR", "CMD", "~CMD" };

  switch (g_logic.proto)
  {
    case PROTO_NEC:
      snprintf(buf, size, "%s", nec_field[idx & 3]);
      break;

    case PROTO_WS2812:
      // The colour of the pixel, written once across the three bytes it took.
      // The wire sends them G-R-B, so the hex dump is the one form of a
      // colour nobody can read at a glance
      ws2812_byte_label(&g_pana.ws, idx, v, buf, size);
      break;

    case PROTO_ONEWIRE:
      // The commands, and what the device answered with: which family code
      // came back, which byte of the ROM is its CRC, and the temperature
      // itself at the byte that completes it
      onewire_byte_label(&g_pana.ow, idx, v, buf, size);
      break;

    case PROTO_CAN:
      // Which field of which frame: the identifier at the byte it starts on,
      // the data bytes numbered, and at the end of the frame whether its CRC
      // checked out and whether anyone acknowledged it
      can_byte_label(&g_pana.can, idx, v, buf, size);
      break;

    case PROTO_DHT:
      // Five bytes, and the reading written at the byte that completes each
      // pair
      dht_byte_label(&g_pana.dht, idx, v, buf, size);
      break;

    case PROTO_SENT:
      // Nibbles: the status one, the data ones numbered, and the 12-bit
      // signals written where their third nibble finishes them
      sent_byte_label(&g_pana.sent, idx, v, buf, size);
      break;

    case PROTO_MIDI:
      // What the message says, written once across the bytes it took: a
      // note and its velocity, a controller and its value, a pitch bend
      midi_byte_label(&g_pana.midi, idx, v, buf, size);
      break;

    case PROTO_LIN:
      // Which field of the frame: the sync, the identifier under its parity
      // bits, the data numbered, and whether the checksum agreed
      lin_byte_label(&g_pana.lin, idx, v, buf, size);
      break;

    case PROTO_EV1527:
      // Which remote and which button, written once across the three bytes
      // its twenty-four bits packed into
      ev1527_byte_label(&g_pana.ev, idx, v, buf, size);
      break;

    case PROTO_DSHOT:
      // What the motor was told, written once across the two bytes the
      // sixteen bits packed into
      dshot_byte_label(&g_pana.dshot, idx, v, buf, size);
      break;

    case PROTO_SPI:
      // A command, or the address it took - and only where a pause said a
      // transaction began there. Everywhere else a byte is just a byte, and
      // the character it stands for is as much as can honestly be said about
      // it, exactly as on a serial line.
      spi_byte_label(&g_pana.spi, idx, v, buf, size);

      if (0 == buf[0])
        dband_ascii_text(buf, size, v);
      break;

    case PROTO_MANCH:
      // A frame is a bit count and a value; the bytes it packed into are
      // where eight bits happened to land and are not a reading of anything
      manchester_byte_label(&g_pana.man, idx, v, buf, size);
      break;

    case PROTO_RC5:
      // Which key, on which device, and whether it was pressed again
      rc5_byte_label(&g_pana.rc5, idx, v, buf, size);
      break;

    case PROTO_DALI:
      // Which ballast, and what it was told to do
      dali_byte_label(&g_pana.dali, idx, v, buf, size);
      break;

    case PROTO_KNX:
      // Who told whom: both addresses are packed fields and not numbers
      knx_byte_label(&g_pana.knx, idx, v, buf, size);
      break;

    case PROTO_SWO:
      // What the packet was: a character out of a stimulus port, an exception
      // the core took, a program counter the DWT sampled
      swo_byte_label(&g_pana.swo, idx, v, buf, size);
      break;

    case PROTO_SWD:
      // Which register the debugger touched, and what came back
      swd_byte_label(&g_pana.swd, idx, v, buf, size);
      break;

    case PROTO_USB:
      // Which packet this is - SETUP, DATA0, ACK - and, on the byte that
      // completes it, the address and endpoint it named or the CRC16 that
      // found its end. A payload byte gets nothing here and falls through to
      // being shown as a character, the way a serial line's does.
      usb_byte_label(&g_pana.usb, idx, v, buf, size);

      if (0 == buf[0])
        dband_ascii_text(buf, size, v);
      break;

    case PROTO_PD:
      // Which message, and then the answer: what the charger offered on this
      // object, what the sink asked for, and whether the CRC32 agreed. Held
      // back to the byte that COMPLETES each field - a data object is four
      // bytes and one number, and the number is written once across them.
      pd_byte_label(&g_pana.pd, idx, v, buf, size);
      break;

    case PROTO_SIRC:
      // Which key on which device, written once across the two bytes the
      // frame took - or the three, where an extended byte came with it
      sirc_byte_label(&g_pana.sirc, idx, v, buf, size);
      break;

    case PROTO_PPM:
      // The byte IS the interval, and the units the link is set up in are
      // the only ones anyone reads it in - the same form the servo decoder
      // puts the pin next door in
      ppm_byte_label(&g_pana.ppm, idx, v, buf, size);
      break;

    case PROTO_SERVO:
      // The byte IS the width, in tens of microseconds; put it back into the
      // units the servo is commanded in, which is the only form anyone reads
      snprintf(buf, size, "%d.%02dms", v / 100, v % 100);
      break;

    default: // UART and the raw bit stream: text is what they usually carry
      dband_ascii_text(buf, size, v);
      break;
  }
}

//-----------------------------------------------------------------------------
// What byte `idx` is ON ITS OWN - the field it belongs to, never the value
// its group adds up to. That value spans several bytes and is written once
// across them a row lower; this is the row that still says which byte of the
// three you are looking at, the way a protocol analyser numbers bit0 bit1
// bit2 above the bracket labelled "command".
static void dband_field_text(char *buf, int size, int idx, uint8_t v)
{
  switch (g_logic.proto)
  {
    case PROTO_SENT:    sent_field_label(&g_pana.sent, idx, v, buf, size); break;
    case PROTO_CAN:     can_field_label(&g_pana.can, idx, v, buf, size); break;
    case PROTO_DHT:     dht_field_label(&g_pana.dht, idx, v, buf, size); break;
    case PROTO_ONEWIRE: onewire_field_label(&g_pana.ow, idx, v, buf, size); break;
    case PROTO_MIDI:    midi_field_label(&g_pana.midi, idx, v, buf, size); break;
    case PROTO_EV1527:  ev1527_field_label(&g_pana.ev, idx, v, buf, size); break;
    case PROTO_DSHOT:   dshot_field_label(&g_pana.dshot, idx, v, buf, size); break;
    case PROTO_WS2812:  ws2812_field_label(&g_pana.ws, idx, v, buf, size); break;

    case PROTO_MANCH:
      manchester_field_label(&g_pana.man, idx, v, buf, size);
      break;

    case PROTO_RC5:
      rc5_field_label(&g_pana.rc5, idx, v, buf, size);
      break;

    case PROTO_DALI:
      dali_field_label(&g_pana.dali, idx, v, buf, size);
      break;

    case PROTO_KNX:
      knx_field_label(&g_pana.knx, idx, v, buf, size);
      break;

    case PROTO_SPI:
      spi_field_label(&g_pana.spi, idx, v, buf, size);

      if (0 == buf[0])
        dband_ascii_text(buf, size, v);
      break;

    case PROTO_SWO:
      swo_field_label(&g_pana.swo, idx, v, buf, size);
      break;

    case PROTO_SWD:
      swd_field_label(&g_pana.swd, idx, v, buf, size);
      break;

    case PROTO_USB:
      usb_field_label(&g_pana.usb, idx, v, buf, size);
      break;

    case PROTO_PD:
      pd_field_label(&g_pana.pd, idx, v, buf, size);
      break;

    case PROTO_SIRC:
      // The command comes FIRST off the wire, which is the opposite way round
      // from how anyone writes a remote code down, so the row says which is
      // which rather than leaving it to be inferred from the order
      sirc_field_label(&g_pana.sirc, idx, v, buf, size);
      break;

    case PROTO_PPM:
      // Which channel of its own frame - numbered from the sync in front of
      // it, so channel 3 is channel 3 whichever frame the record opened in
      ppm_field_label(&g_pana.ppm, idx, v, buf, size);
      break;

    // Everything else names its bytes one at a time already
    default: dband_meaning_text(buf, size, idx, v); break;
  }
}

//-----------------------------------------------------------------------------
// One string into the decoder band's text mask: left edge at x, top edge at y
static void dband_render_text(int x, int y, const char *str)
{
  const Font *font = FONT_SMALL;

  for (; *str; str++)
  {
    char ch = *str;

    if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
      ch = ' ';

    const uint8_t *bitmap = font->data + (ch - FONT_FIRST_CHAR) * font->pitch;
    int size = font->width * font->height;

    for (int i = 0; i < size; i++)
    {
      if ((bitmap[i / 8] >> (i % 8)) & 1)
      {
        int px = x + i % font->width;
        int py = y + i / font->width;

        if (px >= 0 && px < GRID_WIDTH && py >= 0 && py < DBAND_H3)
          g_dband_mask[py][px / 8] |= (uint8_t)(1 << (px % 8));
      }
    }

    x += font->width;
  }
}

//-----------------------------------------------------------------------------
// ...and the swatch box beside it. Only its columns are recorded: which rows
// it occupies is the same everywhere it appears, and the colour is looked up
// from the byte the column already knows about.
static void dband_render_swatch(int x)
{
  for (int i = 0; i < DBAND_SW_W; i++)
  {
    int px = x + i;

    if (px >= 0 && px < GRID_WIDTH)
      g_dband_sw[px / 8] |= (uint8_t)(1 << (px % 8));
  }
}

//-----------------------------------------------------------------------------
// Which screen column is this record sample under? Both are clocks on the
// same event - the sample `g_decode_period_ns` apart in the record, the
// column `horizontal_period` apart on the screen - pinned together at the
// trigger. Saturated well outside the grid rather than wrapped: a record can
// be half a second long and a column two nanoseconds wide, and that ratio
// overflows an int long before it stops being simply "off the screen".
static int band_column(int sample, int trig_col)
{
  int64_t col = trig_col + ((int64_t)(sample - g_decode_trig_pos) *
      g_decode_period_ns) / config.horizontal_period;

  if (col < -1)
    return -1;

  if (col > GRID_WIDTH)
    return GRID_WIDTH;

  return (int)col;
}

//-----------------------------------------------------------------------------
// How a byte of this protocol is laid out in TIME: how many equal bit cells
// it spans, which of them is the first data bit, and whether the data runs
// most significant first.
//
// Only the protocols whose decoder itself worked on a uniform bit grid are
// here, and that is the whole rule. UART, MIDI and LIN sample at start +
// (1.5 + n) bit times and WS2812 and the raw reader step a fixed cell, so
// dividing their bytes evenly reproduces exactly where they looked. NEC, DHT,
// 1-Wire and SENT measure every bit on its own - their cells are not equal in
// time at all - and CAN's are equal but its stuffing puts a varying number of
// them in a byte. Drawing an even grid over any of those would put the lines
// somewhere the decoder never looked, and a grid you cannot trust is worse
// than no grid: the entire point of it is checking the decode against the
// waveform.
//
// Returns 0 when this protocol has no such layout.
static int decode_bit_slots(int *data0, bool *msb_first)
{
  *data0 = 0;
  *msb_first = false;

  switch (g_logic.proto)
  {
    case PROTO_UART:
    case PROTO_MIDI:
    case PROTO_LIN:
    // An SWO pin is 8N1 on the wire, whatever the bytes turn out to mean
    case PROTO_SWO:
      *data0 = 1;          // the start bit takes the first cell
      return 10;           // ...and the stop bit the last

    // The raw reader steps a fixed cell and packs eight of them, so its
    // bytes divide exactly. WS2812 does NOT belong here even though its bit
    // period is constant: a byte's recorded span runs from the first bit's
    // rising edge to the LAST bit's falling one, which is seven periods and
    // a fraction, and dividing that by eight would walk the lines backwards
    // through the byte.
    case PROTO_RAW:
      *msb_first = true;
      return 8;

    // ...and SPI belongs here more than anything else does, because a
    // uniform bit grid is not an approximation of what it did - it IS what
    // it did. The lines are the assumed clock edges drawn on the waveform,
    // which is the one thing worth seeing on a reconstruction: if they do
    // not land on the signal's own edges, the assumed rate is wrong.
    case PROTO_SPI:
      *msb_first = g_pana.spi.msb_first;
      return 8;

    // SWD for the same reason, and with less doubt attached: its bytes are
    // eight clock cycles exactly, first bit off the wire in bit 0, and the
    // grid drawn over them is the recovered clock. Lines that miss the
    // signal's own edges mean the rate is wrong - except that here the
    // parity already said it was not.
    case PROTO_SWD:
      return 8;

    // USB deliberately has none, and it is the one protocol here where a bit
    // grid would be a lie rather than an approximation. Its bytes are eight
    // DATA bits, but a byte containing six consecutive ones also carries a
    // stuff bit - wire that holds no data - so that byte spans nine bit times
    // and its neighbour spans eight. Dividing each span into eight equal
    // cells would put the lines a whole bit out inside exactly the bytes
    // worth checking, and lines drawn for checking a decode must land on the
    // boundary they name or not be drawn.
    case PROTO_USB:
      return 0;

    // Power Delivery is turned down for a DIFFERENT reason, and the
    // difference is worth writing down in case anyone extends this. Its byte
    // really is ten unit intervals of wire, every one of them the same width,
    // so the LINES would land exactly on the signal's own bit boundaries -
    // they would be right. What would be wrong is the numbers in the cells:
    // those ten wire bits are two 4b5b symbols, so cell k is not data bit k
    // and never was, and the grid exists to write the numbers.
    case PROTO_PD:
      return 0;

    default:
      return 0;
  }
}

//-----------------------------------------------------------------------------
// The same map as band_column, without its saturation. The bit grid divides a
// byte's SPAN, so both ends of that span have to be where they really are -
// clamping the left edge to the screen first and dividing afterwards would
// squeeze a whole byte's worth of cells into the visible part of it, and
// lines drawn for checking a decode must never land anywhere but on the
// boundary they name. Clipping happens per pixel, at the point of drawing.
static int band_column_raw(int sample, int trig_col)
{
  int64_t col = trig_col + ((int64_t)(sample - g_decode_trig_pos) *
      g_decode_period_ns) / config.horizontal_period;

  if (col < -1000000)
    col = -1000000;

  if (col > 1000000)
    col = 1000000;

  return (int)col;
}

//-----------------------------------------------------------------------------
// One character into the bit-number row, left edge at x
static void dbit_render_char(int x, char ch)
{
  const Font *font = FONT_SMALL;

  if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR)
    return;

  const uint8_t *bitmap = font->data + (ch - FONT_FIRST_CHAR) * font->pitch;
  int size = font->width * font->height;

  for (int i = 0; i < size; i++)
  {
    if (!((bitmap[i / 8] >> (i % 8)) & 1))
      continue;

    int px = x + i % font->width;
    int py = i / font->width;

    if (px >= 0 && px < GRID_WIDTH && py < 8)
      DBIT->glyph[px] |= (uint8_t)(1 << py);
  }
}

//-----------------------------------------------------------------------------
// The bit grid for one byte: a boundary at every cell edge, and the bit's
// number in the middle of the cell where there is room for it.
static void dbit_build_byte(int b, int trig_col)
{
  const Font *font = FONT_SMALL;
  int data0;
  bool msb;
  int slots = decode_bit_slots(&data0, &msb);

  if (slots < 1)
    return;

  int base = g_logic.pos[b];
  int span = g_logic.end[b] - base;

  // How wide one cell is on the screen, from the unclipped arithmetic - the
  // byte may run off either edge and the answer must not change if it does
  int64_t cell = ((int64_t)span * g_decode_period_ns) /
      ((int64_t)config.horizontal_period * slots);

  if (span <= 0 || cell < DBIT_MIN_PX)
    return;

  for (int k = 0; k <= slots; k++)
  {
    int cx = band_column_raw(base + (int)((int64_t)span * k / slots), trig_col);

    if (cx >= 0 && cx < GRID_WIDTH)
      DBIT->edge[cx / 8] |= (uint8_t)(1 << (cx % 8));
  }

  if (cell < font->width + 2)
    return;                // the cells are there, the numbers do not fit them

  for (int k = 0; k < slots; k++)
  {
    int a = band_column_raw(base + (int)((int64_t)span * k / slots), trig_col);
    int e = band_column_raw(base + (int)((int64_t)span * (k + 1) / slots),
        trig_col);
    int d = k - data0;
    char ch;

    if (e <= 0 || a >= GRID_WIDTH)
      continue;            // this cell is off the screen entirely

    if (d < 0)
      ch = 'S';            // the start bit: not data, and the byte begins here
    else if (d >= 8)
      ch = 'P';            // ...and the stop bit, which is where it ends
    else
      ch = (char)('0' + (msb ? 7 - d : d));

    dbit_render_char(a + (e - a - font->width) / 2, ch);
  }
}

//-----------------------------------------------------------------------------
// Map the decoded bytes onto trace columns, once per change, so the sweep
// only has to look up an array. A record sample and a screen column are two
// clocks on the same event: the sample is `period_ns` apart in the record,
// the column is `horizontal_period` apart on the screen, and the trigger is
// where the two are pinned together.
//
// Nothing is drawn from a held result: those positions belong to a record
// that has since been overwritten, and drawing them over the current trace
// would be pointing at the wrong place on the waveform.
void decode_band_build(void)
{
  const Font *font = FONT_SMALL;
  int was_rows = g_dband_rows;
  int trig_col = GRID_WIDTH/2 - config.horizontal_position_px;
  int widest = 0;

  memset(g_dband_byte, -1, sizeof(g_dband_byte));
  memset(g_dband_edge, 0, sizeof(g_dband_edge));
  memset(g_dband_gedge, 0, sizeof(g_dband_gedge));
  memset(g_dband_gap, 0, sizeof(g_dband_gap));
  memset(g_dband_sw, 0, sizeof(g_dband_sw));
  memset(g_dband_mask, 0, sizeof(g_dband_mask));
  memset(DBIT, 0, sizeof(*DBIT));
  g_dband_rows = 0;
  g_dbit_on = false;

  g_dband_sel_start = g_decode_sel;
  g_dband_sel_len = 1;

  if (g_logic.count > 0)
    decode_group_at(g_decode_sel, &g_dband_sel_start, &g_dband_sel_len);

  if (g_decode_mode && !g_fft_mode && !g_trend_mode && g_logic_have &&
      !g_decode_held && g_logic.count > 0 && g_decode_period_ns > 0 &&
      config.horizontal_period > 0)
  {
    // First pass: where each byte sits, and how wide the widest of them is.
    // The width decides the shape of the whole band, so it has to be known
    // before any text is placed.
    for (int b = 0; b < g_logic.count; b++)
    {
      int c0 = band_column(g_logic.pos[b], trig_col);
      int c1 = band_column(g_logic.end[b], trig_col);
      int lo = (c0 < 0) ? 0 : c0;
      int hi = (c1 > GRID_WIDTH - 1) ? GRID_WIDTH - 1 : c1;

      if (hi <= 0 || lo >= GRID_WIDTH - 1)
        continue; // this byte is off the screen at this pan and timebase

      for (int c = lo; c < hi; c++)
        g_dband_byte[c] = (int8_t)b;

      if (c0 >= 0 && c0 < GRID_WIDTH - 1)
        g_dband_edge[c0 / 8] |= (uint8_t)(1 << (c0 % 8));

      if (c1 - c0 > widest)
        widest = c1 - c0;

      g_dband_rows = DBAND_H1;

      // The bit grid rides the same pass. It is per byte and not per record,
      // because a byte is what has a known number of cells in it.
      if (0 == config.decoder_bits_mode)
      {
        dbit_build_byte(b, trig_col);
        g_dbit_on = true;
      }
    }
  }

  // Room for the number AND something under it: two rows. The number needs
  // two characters plus a pixel of gutter at each end, and anything less than
  // that has no first row to put a second one under.
  int hex_px = 2 * font->width + 2;

  if (g_dband_rows && widest >= hex_px)
  {
    // A third row only where there is something that spans several bytes to
    // put in it. Every row is screen taken from the trace, and a protocol
    // whose bytes each stand alone has nothing to say on it.
    bool grouped = false;

    for (int b = 0; b < g_logic.count && !grouped; )
    {
      int gs, gl;

      decode_group_at(b, &gs, &gl);
      gl -= b - gs;

      if (gl < 1)
        gl = 1;

      grouped = (gl > 1);
      b += gl;
    }

    g_dband_rows = grouped ? DBAND_H3 : DBAND_H2;
  }

  // Which row stops being about bytes. DBAND_H1 has no such row, so nothing
  // ever reaches it and every tick stays a byte tick.
  g_dband_group_y = (g_dband_rows == DBAND_H3) ? DBAND_TEXT_Y2 :
      (g_dband_rows == DBAND_H2) ? DBAND_TEXT_Y1 : g_dband_rows;

  g_dband_row0 = DBAND_BOTTOM - g_dband_rows;

  // Second pass, walked a CHARACTER at a time rather than a byte at a time.
  // Numbers are per byte - a byte is what was on the wire - but what the
  // numbers mean can span several of them, and a code point written once
  // across the group says "these two bytes are one character" in a way that
  // the same label repeated under each of them never would.
  for (int b = 0; b < g_logic.count && g_dband_rows; )
  {
    int gs, glyph;

    decode_group_at(b, &gs, &glyph);

    // Walking forward from zero always lands on a group's first byte, but a
    // decoder that ever answered otherwise would run this loop off the end
    glyph -= b - gs;

    if (glyph < 1)
      glyph = 1;

    int c0 = band_column(g_logic.pos[b], trig_col);
    int gend = band_column(g_logic.end[b + glyph - 1], trig_col);
    // Wide enough for the longest a decoder writes: "DS1990 key", "RH=45.3%",
    // "SysEx Waldrf", "On C#-1 v127"
    char text[16];

    // Where the group starts is the only boundary its value's row has
    if (c0 >= 0 && c0 < GRID_WIDTH - 1)
      g_dband_gedge[c0 / 8] |= (uint8_t)(1 << (c0 % 8));

    // A group is one strip. Its bytes do not touch - the gap is the tail of
    // the last bit of the byte before it - and leaving those columns uncovered
    // left an undimmed slot of raw trace under the middle of the group's own
    // value. The tail belongs to the byte it is the tail OF, so that is the
    // byte whose tint it takes.
    if (glyph > 1)
    {
      int lo = (c0 < 0) ? 0 : c0;
      int hi = (gend > GRID_WIDTH - 1) ? GRID_WIDTH - 1 : gend;
      int last = -1;

      for (int c = lo; c < hi; c++)
      {
        if (g_dband_byte[c] >= 0)
        {
          last = g_dband_byte[c];
        }
        else if (last >= 0)
        {
          g_dband_byte[c] = (int8_t)last;
          g_dband_gap[c / 8] |= (uint8_t)(1 << (c % 8));
        }
      }
    }

    for (int i = 0; i < glyph; i++)
    {
      int bc0 = band_column(g_logic.pos[b + i], trig_col);
      int bc1 = band_column(g_logic.end[b + i], trig_col);
      int width = bc1 - bc0;
      int room = width - 2; // a pixel of gutter at each end
      uint8_t v = g_logic.bytes[b + i];

      if (bc1 <= 0 || bc0 >= GRID_WIDTH - 1)
        continue;

      // Top row: the number, wherever it fits at all
      if (2 * font->width <= room)
      {
        snprintf(text, sizeof(text), "%02X", v);
        dband_render_text(bc0 + (width - 2 * font->width) / 2,
            (g_dband_rows > DBAND_H1) ? DBAND_TEXT_Y0 :
            (DBAND_H1 - font->height) / 2, text);
      }
      else if (g_dband_rows == DBAND_H1 && glyph == 1 && v > 32 && v < 127 &&
          font->width <= room)
      {
        // One row only and the number did not fit: a bare character is still
        // worth more than an empty box
        snprintf(text, sizeof(text), "%c", v);
        dband_render_text(bc0 + (width - font->width) / 2,
            (DBAND_H1 - font->height) / 2, text);
      }

      // Middle row, where there is one: what THIS byte is, as opposed to what
      // its group adds up to. Grouping alone answers "these three nibbles are
      // 0x394" and leaves "which of the three am I looking at" unanswered.
      if (g_dband_rows == DBAND_H3)
      {
        int need;

        dband_field_text(text, sizeof(text), b + i, v);
        need = (int)strlen(text) * font->width;

        if (text[0] && need <= room)
          dband_render_text(bc0 + (width - need) / 2, DBAND_TEXT_Y1, text);
      }
    }

    b += glyph;

    if (g_dband_rows < DBAND_H2 || gend <= 0 || c0 >= GRID_WIDTH - 1)
      continue;

    // Bottom row: what the bytes above mean, written once across however many
    // of them it took to say it. For a protocol that is the label the group's
    // last byte carries - the one that completes the value - so a three
    // nibble SENT signal reads "S1=543" once across all three.
    //
    // With three rows a byte that is a group of its own has already had its
    // say on the row above, and repeating it here would only widen the band
    // to write the same word twice.
    if (g_dband_rows == DBAND_H3 && glyph == 1)
      continue;

    if (glyph > 1 && decode_proto_is_text())
      snprintf(text, sizeof(text), "U+%04X",
          (unsigned)utf8_code_point(&g_logic.bytes[b - glyph], glyph));
    else
      dband_meaning_text(text, sizeof(text), b - 1, g_logic.bytes[b - 1]);

    uint16_t swc;
    // One character cell for the box and its gutter, and only where the value
    // is a colour. The swatch goes with the text or not at all: half of a
    // "#221133 [ ]" that did not fit says less than the hex on its own.
    int swpx = decode_group_color(b - 1, &swc) ? font->width : 0;
    int span = gend - c0;
    int need = (int)strlen(text) * font->width;

    if (text[0] && need + swpx <= span - 2)
    {
      int x = c0 + (span - need - swpx) / 2;

      if (swpx)
        dband_render_swatch(x);

      dband_render_text(x + swpx,
          (g_dband_rows == DBAND_H3) ? DBAND_TEXT_Y2 : DBAND_TEXT_Y1, text);
    }
  }

  // Bottom-aligned, so a band that grew or shrank is repainted over the
  // taller of the two
  int rows = (g_dband_rows > was_rows) ? g_dband_rows : was_rows;

  if (rows)
    overlay_repaint_region(DBAND_BOTTOM - rows, rows);
}

//-----------------------------------------------------------------------------
// Forget the panel's rendered state so the next update rebuilds the text and
// repaints the band even if the numbers happen to come out identical. Needed
// after anything that leaves the bottom of the screen in an unknown state:
// the panel is only repainted when its TEXT changes, so a band that was
// cleared by something else would otherwise stay blank indefinitely.
void mpanel_invalidate(void)
{
  g_mpanel_cells = -1;          // matches neither a grid nor free text
  g_mpanel_text[0][0] = '\x01'; // ...and never a freshly built line either
  g_mpanel_text[1][0] = 0;
  g_mpanel_force = true;        // and do it on the next tick, not in 500 ms

  if (config.measure_display)
    g_measure_timer = 0;
}

//-----------------------------------------------------------------------------
// Whether the translucent panel is the right place for the measurements: in
// panel view, with the measurements on, and only where the trace area is ours
// to composite into (the spectrum and the calibration screen draw their own)
bool mpanel_wanted(void)
{
  // The editor IS the readings on screen; without them there would be nothing
  // to arrange
  if (g_layout_edit)
    return true;

  // The calibration screen draws its own trace but goes through the same
  // sweep, so the band is available there too - and that is where a hint is
  // worth more than a measurement
  if (g_autocal_active)
    return true; // it is talking to the user through the band

  if (scope_calibration_mode)
    return g_calib_hint;

  return config.measure_display && !g_fft_mode && !g_trend_mode &&
      config.measure_panel_mode == 0;
}

//-----------------------------------------------------------------------------
void mpanel_set_active(bool active)
{
  if (active == g_mpanel_active)
    return;

  g_mpanel_active = active;
  mpanel_invalidate();
  overlay_repaint_region(mpanel_row0(), mpanel_h());
}

//-----------------------------------------------------------------------------
// Put fixed text in the panel instead of the reading grid. mpanel_update()
// builds cells out of what was measured; this is for text that is chosen rather
// than measured, like the calibration hint, and it rebuilds only when the text
// actually changes.
void mpanel_set_lines(const char *l0, const char *l1)
{
  if (g_mpanel_is_text &&
      0 == strcmp(l0, g_mpanel_text[0]) && 0 == strcmp(l1, g_mpanel_text[1]))
    return;

  strncpy(g_mpanel_text[0], l0, MPANEL_TEXT_MAX);
  strncpy(g_mpanel_text[1], l1, MPANEL_TEXT_MAX);
  g_mpanel_text[0][MPANEL_TEXT_MAX] = 0;
  g_mpanel_text[1][MPANEL_TEXT_MAX] = 0;
  g_mpanel_is_text = true;

  g_mpanel_builds++;
  g_mpanel_paints++;
  overlay_repaint_region(mpanel_row0(), mpanel_h());
}

//-----------------------------------------------------------------------------
// Plausible readings for the editor. Arranging a layout against numbers that
// jump - or against a probe with nothing on it, where half the metrics read
// "--" and the grey makes them hard to see - is arranging blind. A 10 kHz,
// 2 Vpp, 50% square is what the mock trace behind them draws.
static void layout_mock_measure(ScopeMeasure *sm)
{
  memset(sm, 0, sizeof(*sm));

  sm->vpp_mv  = 2000;
  sm->vamp_mv = 1980;
  sm->vtop_mv = 1000;
  sm->vbase_mv = -1000;
  sm->vrms_mv = 1000;
  sm->vavg_mv = 0;
  sm->vmax_mv = 1000;
  sm->vmin_mv = -1000;
  sm->vp_mv   = 1000;
  sm->vmid_mv = 0;
  sm->frequency     = 10000;
  sm->duty_x10      = 500;
  sm->period_med_ns = 100000;
  sm->period_min_ns = 99800;
  sm->period_max_ns = 100200;
  sm->width_pos_ns  = 50000;
  sm->width_neg_ns  = 50000;
  sm->jitter_rms_ps = 1200;
  sm->jitter_pp_ps  = 8000;
  sm->periods       = 80;
  sm->period_good_pct = 100;
  sm->level_pct     = 90;
}

//-----------------------------------------------------------------------------
// Build the placed readings from config.measure_widget[]. Slot for slot, not
// compacted: the editor selects by slot, and a hole in the middle of the array
// is a slot the user emptied rather than a reason to renumber the rest.
static void widgets_update(const ScopeMeasure *sm)
{
  PanelPlaced next[PANEL_WIDGETS_MAX];
  static int heartbeat = 0;

  memset(next, 0, sizeof(next));

  for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
  {
    const PanelWidget *w = &config.measure_widget[i];
    PanelPlaced *p = &next[i];
    MeasureItem item;
    int len;

    if (MEASURE_NONE == w->metric || w->metric >= MEASURE_COUNT)
      continue;

    if (!measure_format(w->metric, sm, &item))
      continue;

    len = snprintf(p->text, sizeof(p->text), "%s %s", item.label, item.value);

    if (len < 0)
      len = 0;
    else if (len > (int)sizeof(p->text) - 1)
      len = (int)sizeof(p->text) - 1;

    p->len       = (uint8_t)len;
    p->label_len = (uint8_t)strlen(item.label);
    p->size      = (uint8_t)widget_size_of(w->flags);
    p->color     = measure_item_color(&item);

    // The text's own size decides where a corner-anchored reading starts, and
    // the text is only known now
    {
      const Font *font = widget_font(p->size);
      int sc = widget_scale(p->size);
      int x, y;

      widget_pos(w, len * font->width * sc, font->height * sc, &x, &y);
      p->x = (int16_t)x;
      p->y = (int16_t)y;
    }
  }

  if (g_placed_n == PANEL_WIDGETS_MAX &&
      0 == memcmp(next, g_placed, sizeof(next)))
  {
    // Same readings in the same places - but heal the area anyway every few
    // seconds, for the reason the band does (see mpanel_update)
    if (++heartbeat < MPANEL_HEARTBEAT)
      return;

    heartbeat = 0;
    g_mpanel_paints++;
    overlay_repaint_region(0, GRID_HEIGHT - 1);
    return;
  }

  heartbeat = 0;
  g_mpanel_builds++;
  memcpy(g_placed, next, sizeof(next));
  g_placed_n = PANEL_WIDGETS_MAX;

  // The whole trace area, because a widget can be anywhere in it. At 2 Hz that
  // is a fraction of what the sweep itself costs.
  g_mpanel_paints++;
  overlay_repaint_region(0, GRID_HEIGHT - 1);
}

//-----------------------------------------------------------------------------
// noinline deliberately: it is called once per scope tick from one place, and
// welded into scope_task() -O3 unrolls the metric scan and the cell building
// into three kilobytes of straight-line code. A call costs nothing at 2 Hz.
__attribute__((noinline))
static void mpanel_update(void)
{
  ScopeMeasure sm;
  MeasureItem items[MEASURE_ITEMS_MAX];
  MPanelCell next[MPANEL_CELLS_MAX];
  int n, cells = 0;
  static int throttle = 0;
  static int heartbeat = 0;

  // Live values jitter in their last digit; rebuilding the text (and
  // repainting the band) 10x a second serialized the main loop behind the
  // LCD bus and made buttons feel mushy. 2 Hz is plenty for reading.
  if (!g_mpanel_force && ++throttle < 5)
    return;

  throttle = 0;

  // Unthrottled: this runs at 2 Hz anyway, and the 10 Hz recompute throttle
  // behind capture_get_measurements() can only ever hand back an older frame.
  // The editor arranges against mock values instead - see layout_mock_measure.
  if (g_layout_edit)
    layout_mock_measure(&sm);
  else if (!capture_get_measurements_fresh(&sm))
    return;

  // Only now: nothing was rebuilt, so a request made before the first
  // acquisition landed - which is every entry into the scope - still stands.
  // Consuming it here used to cost the panel up to half a second of the
  // throttle before it first appeared.
  g_mpanel_force = false;

  // The other layout: readings where the user put them, not a band
  if (!mpanel_band_mode())
  {
    widgets_update(&sm);
    return;
  }

  n = measure_build_items(&sm, items);

  // memcmp decides below whether anything changed, so the holes the compiler
  // leaves between these fields have to be a known value
  memset(next, 0, sizeof(next));

  // Everything that fits - and if something does not, the last cell says how
  // many were left out instead of the band simply ending. Running out of room
  // silently reads exactly like "there is nothing more to show".
  {
    int chars = mpanel_chars();
    int x = 0, row = 0;

    for (int i = 0; i < n && cells < MPANEL_CELLS_MAX; i++)
    {
      MPanelCell *cell = &next[cells];

      mpanel_cell_set(cell, items[i].label, items[i].value,
          measure_item_color(&items[i]));

      // Wrap to the second row when this pair would run off the first, and
      // stop when there is no third row to wrap into
      if (x + cell->len > chars)
      {
        if (++row >= mpanel_rows())
          break;

        x = 0;
      }

      cell->x = (uint8_t)x;
      cell->row = (uint8_t)row;
      x += cell->len + MPANEL_GUTTER;
      cells++;
    }

    // What did not fit says so, rather than the band simply ending: running out
    // of room silently reads exactly like "there is nothing more to show". The
    // marker needs room of its own, so the last pair placed gives way to it.
    if (cells < n)
    {
      char more[12];
      int hidden = n - cells;
      int at = cells;

      snprintf(more, sizeof(more), "+%d more", hidden);

      // ...but it says it in two characters rather than eating the only reading
      // on the band. At 16x32 a row is eighteen characters and one reading fills
      // twelve of them, so "+4 more" does not fit and "+4" does - and a band
      // showing nothing but "+4 more" (which is what this did) has replaced the
      // measurement with a note about the measurements.
      if (x + (int)strlen(more) > chars)
        snprintf(more, sizeof(more), "+%d", hidden);

      if (x + (int)strlen(more) > chars && at > 0)
      {
        at--;                     // no room left: the marker takes its place
        x = next[at].x;
        row = next[at].row;
        hidden++;
        snprintf(more, sizeof(more), "+%d more", hidden);

        if (x + (int)strlen(more) > chars)
          snprintf(more, sizeof(more), "+%d", hidden);
      }
      else
      {
        row = (at > 0) ? next[at - 1].row : 0;
      }

      mpanel_cell_set(&next[at], "", more, MPANEL_NONE_COLOR);
      next[at].x = (uint8_t)x;
      next[at].row = (uint8_t)row;
      cells = at + 1;
    }
  }

  if (!g_mpanel_is_text && cells == g_mpanel_cells &&
      0 == memcmp(next, g_mpanel_cell, sizeof(next)))
  {
    // Same readings, so the band is still right — but repaint it every few
    // seconds anyway. The band lives inside the trace area, and anything
    // that paints there without going through build_trace_column() leaves a
    // hole the sweep will not heal (it skips columns whose trace data has
    // not changed). A steady signal is exactly when that hole would stay.
    if (++heartbeat < MPANEL_HEARTBEAT)
      return;

    heartbeat = 0;
    g_mpanel_paints++;
    overlay_repaint_region(mpanel_row0(), mpanel_h());
    return;
  }

  heartbeat = 0;
  g_mpanel_builds++;
  memcpy(g_mpanel_cell, next, sizeof(next));
  g_mpanel_cells = cells;
  g_mpanel_is_text = false;

  g_mpanel_paints++;
  overlay_repaint_region(mpanel_row0(), mpanel_h());
}

/*- The layout editor -------------------------------------------------------*/
// Both live further down, and the editor is here because it belongs beside the
// panel it edits
void refresh_view(void);

/*
 * A screen for arranging the readings, because a menu cannot do it: where a
 * number should sit is a question about the picture, and a list of coordinates
 * is not a picture. So the trace area stays the trace area, the readings stay
 * on it, and the arrows move them.
 *
 * Two things are mocked while it is up. The trace is a synthetic square wave,
 * and the readings are the fixed set in layout_mock_measure(): a layout arranged
 * against numbers that jump - or against a probe with nothing on it, where half
 * the readings are grey dashes - is a layout arranged blind. What is NOT mocked
 * is the drawing: the widgets go through the same column compositor they will be
 * drawn by afterwards, so the editor cannot flatter the result.
 */

//-----------------------------------------------------------------------------
// A 10 kHz-looking square at 50% into the display buffer, which is what the
// mock readings describe. Pixel space, so it needs no acquisition and no
// timebase: this is a backdrop to arrange against, not a measurement.
#define LAYOUT_MOCK_PERIOD   60
#define LAYOUT_MOCK_AMP      45

static void layout_mock_trace(void)
{
  for (int c = 0; c < GRID_WIDTH; c++)
  {
    int phase = c % LAYOUT_MOCK_PERIOD;
    bool high = phase < LAYOUT_MOCK_PERIOD / 2;
    bool edge = (0 == phase) || (LAYOUT_MOCK_PERIOD / 2 == phase);
    int top = GRID_CENTER_Y - LAYOUT_MOCK_AMP;
    int bottom = GRID_CENTER_Y + LAYOUT_MOCK_AMP;

    // An edge column spans both levels, which is how a real record draws a
    // transition too
    g_display_buffer.min[c] = (uint8_t)(edge ? top : (high ? top : bottom));
    g_display_buffer.max[c] = (uint8_t)(edge ? bottom : (high ? top : bottom));
    g_display_buffer.flags[c] = SAMPLE_FLAG_VALID;
  }
}

//-----------------------------------------------------------------------------
// Lay the band's own readings out as widgets, in the places the band had them.
// That is the layout the user is already looking at, so the editor opens on
// something familiar rather than on an empty screen - and AUTO brings it back
// when an arrangement has gone wrong.
static void layout_seed_from_band(void)
{
  ScopeMeasure sm;
  MeasureItem items[MEASURE_ITEMS_MAX];
  const Font *font = mpanel_font();
  int sc = mpanel_scale();
  int cell = font->width * sc;
  int gh = font->height * sc;
  int chars = mpanel_chars();
  int row_h = mpanel_row_h();
  int rows = mpanel_rows();
  int band_y = GRID_HEIGHT - 1 - (2 * MPANEL_PAD_Y + rows * row_h -
      MPANEL_GAP_Y) + MPANEL_PAD_Y;
  uint8_t size_flags = (PANEL_FONT_HUGE == config.measure_panel_font) ? PW_HUGE :
      ((PANEL_FONT_LARGE == config.measure_panel_font) ? PW_LARGE : 0);
  int n, placed = 0, x = 0, row = 0;

  layout_mock_measure(&sm);
  n = measure_build_items(&sm, items);

  memset(config.measure_widget, 0, sizeof(config.measure_widget));

  for (int i = 0; i < n && placed < PANEL_WIDGETS_MAX; i++)
  {
    PanelWidget *w = &config.measure_widget[placed];
    int len = (int)strlen(items[i].label) + 1 + (int)strlen(items[i].value);

    if (x + len > chars)
    {
      if (++row >= rows)
        break;

      x = 0;
    }

    w->metric = (uint8_t)items[i].metric;
    w->flags = size_flags;

    // Through the anchor, so that the row the band put along the bottom stays
    // along the bottom - and so that a reading in the right half is measured
    // from the right edge, which is what it will grow towards later
    widget_set_pos(w, MPANEL_PAD_X + x * cell, band_y + row * row_h,
        len * cell, gh);

    x += len + MPANEL_GUTTER;
    placed++;
  }
}

//-----------------------------------------------------------------------------
// The selected slot, and the moves the arrows make on it
static PanelWidget *layout_selected(void)
{
  if (g_layout_sel < 0 || g_layout_sel >= PANEL_WIDGETS_MAX)
    g_layout_sel = 0;

  return &config.measure_widget[g_layout_sel];
}

static int layout_used(void)
{
  int n = 0;

  for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
  {
    if (MEASURE_NONE != config.measure_widget[i].metric)
      n++;
  }

  return n;
}

// A reading the layout is not already showing, so that adding one twice takes
// two deliberate presses of the metric key rather than being the default
static int layout_unused_metric(void)
{
  for (int m = MEASURE_NONE + 1; m < MEASURE_COUNT; m++)
  {
    bool taken = false;

    for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
    {
      if (config.measure_widget[i].metric == m)
        taken = true;
    }

    if (!taken)
      return m;
  }

  return MEASURE_VPP;
}

//-----------------------------------------------------------------------------
// The next slot that holds something, wrapping, so the selection walks the
// readings rather than the array's holes
static void layout_select_step(int dir)
{
  for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
  {
    g_layout_sel = (g_layout_sel + dir + PANEL_WIDGETS_MAX) % PANEL_WIDGETS_MAX;

    if (MEASURE_NONE != config.measure_widget[g_layout_sel].metric)
      return;
  }
}

//-----------------------------------------------------------------------------
// Which reading a widget shows, stepped through the metrics. MEASURE_NONE is
// skipped: a widget showing nothing would be an invisible thing to be holding.
static void layout_metric_step(PanelWidget *w, int dir)
{
  int m = w->metric + dir;

  if (m <= MEASURE_NONE)
    m = MEASURE_COUNT - 1;
  else if (m >= MEASURE_COUNT)
    m = MEASURE_NONE + 1;

  w->metric = (uint8_t)m;
}

//-----------------------------------------------------------------------------
// Move the selected widget, clamped so it cannot be pushed off the screen and
// lost. The text's own width decides the right-hand limit, which is why this
// asks g_placed rather than the config: that is where the composed width is.
static void layout_move(int dx, int dy)
{
  PanelWidget *w = layout_selected();
  const PanelPlaced *p = &g_placed[g_layout_sel];
  int size = widget_size_of(w->flags);
  const Font *font = widget_font(size);
  int sc = widget_scale(size);
  int wide = (p->len ? p->len : 8) * font->width * sc;
  int gh = font->height * sc;
  int max_x = GRID_WIDTH - 2 - wide;
  int max_y = GRID_HEIGHT - 2 - gh;
  int x, y;

  // In screen pixels, not in the stored units: the arrows move a reading across
  // the screen, and which corner that ends up being measured from is decided
  // afterwards, by where it lands
  widget_pos(w, wide, gh, &x, &y);

  x += dx * PANEL_WIDGET_STEP;
  y += dy * PANEL_WIDGET_STEP;

  if (max_x < 0)
    max_x = 0;

  if (max_y < 0)
    max_y = 0;

  if (x < 0)
    x = 0;
  else if (x > max_x)
    x = max_x;

  if (y < 0)
    y = 0;
  else if (y > max_y)
    y = max_y;

  widget_set_pos(w, x, y, wide, gh);
}

//-----------------------------------------------------------------------------
// One line of key hints where the status line normally is. The editor owns that
// line: what the arrows do here changes with whether a widget is held, and a
// screen full of readings with no way to know that is a puzzle.
void layout_edit_footer(void)
{
  char buf[64];

  // Two rows of the 6x8 font in the sixteen pixels the status line has, which
  // is the only way all of the keys fit: 51 characters to a row here.
  //
  // Exactly the rectangle draw_status_line() clears, and no wider: a footer
  // that reached x=0 left its first letters standing in the margin the status
  // line does not own when the editor closed.
  lcd_fill_rect(0, GRID_BOTTOM + 1, LCD_WIDTH, STATUS_LINE_HEIGHT, BG_COLOR);
  lcd_set_font(FONT_SMALL);
  lcd_set_color(BG_COLOR, MEASURE_MODE_COLOR);

  if (g_layout_grab)
  {
    snprintf(buf, sizeof(buf), "HOLDING %s", g_placed[g_layout_sel].text);
    lcd_puts(GRID_LEFT + 2, STATUS_LINE_Y, buf);
    lcd_puts(GRID_LEFT + 2, STATUS_LINE_Y + 8,
        "arrows move it   MODE put it down");
  }
  else
  {
    lcd_puts(GRID_LEFT + 2, STATUS_LINE_Y,
        "L/R pick   U/D metric   MODE take   F1 size");
    lcd_puts(GRID_LEFT + 2, STATUS_LINE_Y + 8,
        "EDGE add   50% delete   AUTO tidy   SAVE done");
  }

  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
// Redraw everything the editor owns: the mock trace under the widgets, the
// widgets themselves (built from the mock readings), and the hint line.
static void layout_edit_refresh(void)
{
  ScopeMeasure sm;

  layout_mock_measure(&sm);
  layout_mock_trace();
  g_placed_n = 0;            // force the rebuild past its own compare
  widgets_update(&sm);
  layout_edit_footer();
}

//-----------------------------------------------------------------------------
void scope_layout_edit_start(void)
{
  // An empty layout would be an empty screen to arrange nothing on, so the
  // band's own arrangement is the starting point
  if (0 == layout_used())
    layout_seed_from_band();

  config.measure_layout_mode = PANEL_LAYOUT_WIDGETS;
  g_layout_edit = true;
  g_layout_grab = false;
  g_layout_sel = 0;

  if (MEASURE_NONE == config.measure_widget[0].metric)
    layout_select_step(1);

  g_mpanel_is_text = false;
  g_mpanel_active = true;
  mpanel_invalidate();
  layout_edit_refresh();
}

//-----------------------------------------------------------------------------
static void layout_edit_finish(void)
{
  g_layout_edit = false;
  g_layout_grab = false;

  // Back to the real record and the real readings. The layout itself stays as
  // it was arranged - it is in config, so the store writes it out on its own.
  mpanel_invalidate();
  refresh_view();
  draw_status_line();
}

//-----------------------------------------------------------------------------
// The editor owns the keyboard while it is up, the way auto-calibration does:
// every key here means something else than it does over a live trace.
static void layout_edit_keys(int buttons)
{
  PanelWidget *w = layout_selected();

  // SAVE and not MENU: the launcher takes MENU for the system menu before an
  // application sees it (a plain press opens the menu over whatever is on
  // screen), so the editor never gets to hear about it. Closing that menu
  // repaints straight back into the editor, which is at least consistent.
  if (buttons & BTN_SAVE)
  {
    layout_edit_finish();
    return;
  }

  if (buttons & BTN_MODE)
  {
    g_layout_grab = !g_layout_grab;
  }
  else if (buttons & BTN_F1)
  {
    // 6x8 -> 8x16 -> 16x32 -> 6x8. Three steps where there used to be a toggle,
    // because the third one is the point of the screen: it is the only size that
    // is bigger than anything the two bars can hold.
    int size = (widget_size_of(w->flags) + 1) % 3;

    w->flags &= (uint8_t)~PW_SIZE_MASK;
    w->flags |= (1 == size) ? PW_LARGE : ((2 == size) ? PW_HUGE : 0);

    // Growing one can push it off the right edge or the bottom; the clamp lives
    // in the move, so ask for a move of nothing
    layout_move(0, 0);
  }
  else if (buttons & BTN_EDGE)
  {
    // A new reading in the first free slot, at the top left corner - which is
    // empty in every layout that came from the band - and already held, so the
    // arrows carry it away from there without another keypress. Anywhere near
    // the widget it was added from would land on top of something.
    for (int i = 0; i < PANEL_WIDGETS_MAX; i++)
    {
      if (MEASURE_NONE != config.measure_widget[i].metric)
        continue;

      config.measure_widget[i].metric = (uint8_t)layout_unused_metric();
      config.measure_widget[i].x = 1;
      config.measure_widget[i].y = 1;
      // The size of the one it was added from, but never its corner: x and y
      // here mean the top left one, and inheriting a bottom-right anchor would
      // put the new reading in the opposite corner from where it says it is
      config.measure_widget[i].flags = w->flags & PW_SIZE_MASK;
      g_layout_sel = i;
      g_layout_grab = true;
      break;
    }
  }
  else if (buttons & BTN_50P)
  {
    // ...and never the last one: an empty layout is a screen with no readings
    // and no way back to them except the menu
    if (layout_used() > 1)
    {
      w->metric = MEASURE_NONE;
      layout_select_step(1);
    }
  }
  else if (buttons & BTN_AUTO)
  {
    layout_seed_from_band();
    g_layout_sel = 0;
    g_layout_grab = false;
  }
  else if (g_layout_grab)
  {
    if (buttons & BTN_LEFT)
      layout_move(-1, 0);
    else if (buttons & BTN_RIGHT)
      layout_move(1, 0);
    else if (buttons & BTN_UP)
      layout_move(0, -1);
    else if (buttons & BTN_DOWN)
      layout_move(0, 1);
  }
  else
  {
    if (buttons & BTN_LEFT)
      layout_select_step(-1);
    else if (buttons & BTN_RIGHT)
      layout_select_step(1);
    else if (buttons & BTN_UP)
      layout_metric_step(w, 1);
    else if (buttons & BTN_DOWN)
      layout_metric_step(w, -1);
  }

  layout_edit_refresh();
}

//-----------------------------------------------------------------------------
/*
 * The signal behind the strip: the record's own min/max envelope, one column at
 * a time, instead of the zigzag that used to stand in for it.
 *
 * One byte per column - the envelope's top row in the high nibble, its bottom
 * row in the low one, MV_ENV_NONE where no record reaches. Display ROWS and not
 * the raw counts they came from, which costs nothing: the mapping from counts to
 * rows depends on the vertical scale and position, and both of those already
 * invalidate this through scope_display_settings_changed(). It buys back 159
 * bytes of the borrowed TCM block, which is a region with a floor rather than
 * room to spare - see the linker script.
 *
 * WHY THE RECORD AND NOT THE RING, when the strip's axis is the ring's whole
 * 786 us. Because the ring cannot be read as a record once acquisition is
 * running. A sweep is one full lap of it (capture.c: the post-trigger count is
 * CAPTURE_BUFFER_SIZE - trigger_offset), and dma_finish() hands straight over
 * to dma_start(), which begins overwriting from ring index 0 again. The ring is
 * a coherent, trigger-anchored record for exactly as long as it takes the next
 * sweep to start, which at 125 MS/s is 786 us - tens of laps between two
 * repaints of this strip. Whatever the main loop finds there afterwards is a
 * different sweep, and the trigger position saved from the last dma_finish does
 * not describe it. The 24 KB storage record is the ONLY trigger-anchored
 * snapshot that survives, and it is what the trace is drawn from - so it is
 * also the only thing that can be drawn here without the frame below lying
 * about which signal it is framing.
 *
 * At slow timebases the record IS the whole ring, decimated 4:1, so the strip
 * fills end to end. Past ~16 us/div it becomes a 24 KB full-rate WINDOW of the
 * ring (update_storage_window) and covers about a quarter of the strip; the
 * rest stays empty, which is the true statement - there is no record there, and
 * panning into it moves the window rather than revealing anything.
 */


//-----------------------------------------------------------------------------
int clip_for_display(int value)
{
  value = GRID_HEIGHT/2-1 - value;

  if (value > (GRID_HEIGHT-2))
    value = (GRID_HEIGHT-2);
  else if (value < 0)
    value = 0;

  return value;
}

//---------------------------------------------------------------------
void close_gaps(DisplayBuffer *db)
{
  for (int i = 0; i < GRID_WIDTH-1; i++)
  {
    if ((db->flags[i] & SAMPLE_FLAG_VALID) && (db->flags[i+1] & SAMPLE_FLAG_VALID))
    {
      if (db->max[i] < (db->min[i+1]-1))
      {
        int avg = (db->max[i] + db->min[i+1]) / 2;
        db->max[i] = avg;
        db->min[i+1] = avg+1;
      }
      else if (db->min[i] > (db->max[i+1] + 1))
      {
        int avg = (db->min[i] + db->max[i+1]) / 2;
        db->min[i] = avg;
        db->max[i+1] = avg-1;
      }
    }
  }
}

/*- Display post-processing, cursors and the trend view ---------------------*/
void update_display(void);

//-----------------------------------------------------------------------------
// Forget the accumulated envelope and averages: called whenever the mapping
// from column to time or voltage changes (pan, zoom, scale, coupling) and
// when the menu toggles the features themselves
void scope_display_settings_changed(void)
{
  memset(g_persist_lvl, 0, sizeof(g_persist_lvl));
  g_persist_stamp = timer_ms();
  memset(g_avg_have, 0, sizeof(g_avg_have));
  g_shadow_valid = false;

  // The record map's envelope is stored as display rows, so it was built
  // through the vertical scale and position - which is exactly what just moved
  g_mv_env_valid = false;
}

//-----------------------------------------------------------------------------
// The probe attenuation changed - from the 1X/10X key or from the menu. Every
// millivolt on this screen is derived from one of the two multipliers below, so
// both have to be recomputed together; the trace itself does not move, because
// the mV-per-count and the mV-per-pixel moved by the same factor.
void scope_probe_changed(void)
{
  if (scope_calibration_mode)
    return;   // it is forced off there, see scope_init

  config.vertical_mult = config.calib_vs_mult[config.vertical_scale] *
      config_probe_mult();
  config.vertical_position_mv = config.vertical_position *
      vs_mv_px(config.vertical_scale);
  config.trigger_level_mv = config.trigger_level * vs_mv_px(config.vertical_scale);

  // The hardware trigger is a converter code, and the level it is derived from
  // has just changed units
  capture_set_trigger_level(config.trigger_level_mv);

  mpanel_invalidate();
  draw_status_line();
  refresh_view();
}

//-----------------------------------------------------------------------------
/*
 * The text size changed, so the screen is divided up differently: see ScopeGeom.
 *
 * Everything that is positioned against the grid asks g_geom, so setting it is
 * most of the work. What is left is the state that CACHES a position: the
 * per-column shadow of what is on the panel (rows moved), the grid patterns
 * (fewer rows to rule), the trace's own history (a column's pixel row means a
 * different voltage now), and the state slot's "nothing has changed" latch.
 *
 * The vertical position and the trigger level are in PIXELS from the middle of
 * the grid, and the middle stays the middle - but a smaller grid puts both
 * nearer its edge, so they are clamped back inside it here rather than being
 * quietly off-screen with a marker pinned to the frame.
 */
void scope_ui_scale_changed(void)
{
  int half;

  g_geom = (UI_SCALE_LARGE == config.ui_scale) ? g_geom_large : g_geom_normal;

  half = GRID_HEIGHT / 2 - 1;

  if (config.vertical_position > half)
    config.vertical_position = half;
  else if (config.vertical_position < -half)
    config.vertical_position = -half;

  if (config.trigger_level > half)
    config.trigger_level = half;
  else if (config.trigger_level < -half)
    config.trigger_level = -half;

  config.vertical_position_mv = config.vertical_position *
      vs_mv_px(config.vertical_scale);
  config.trigger_level_mv = config.trigger_level * vs_mv_px(config.vertical_scale);

  capture_set_vertical_parameters();
  capture_set_trigger_level(config.trigger_level_mv);

  g_state = -1;              // the capture-state slot repaints itself
  g_vpos_marker.valid = false;
  g_trig_marker.valid = false;
  g_shadow_valid = false;
  memset(g_persist_lvl, 0, sizeof(g_persist_lvl));
  memset(g_avg_have, 0, sizeof(g_avg_have));
  grid_init();
  memset(g_mpanel_cell, 0, sizeof(g_mpanel_cell));
  g_mpanel_is_text = false;
  mpanel_invalidate();
}

//-----------------------------------------------------------------------------
void scope_measure_panel_changed(void)
{
  // The cells were composed to the old font's cell width, so they go; the band
  // rebuilds at the new one on the scope's next tick. The trace area itself is
  // repainted in full when the menu closes over it, which is what covers the
  // rows the band gives back when it shrinks.
  memset(g_mpanel_cell, 0, sizeof(g_mpanel_cell));
  g_mpanel_is_text = false;
  mpanel_invalidate();
}

//-----------------------------------------------------------------------------
// Acquisition averaging, display space: an EMA of each column's midpoint
// across trigger-aligned frames, drawn as a thin trace instead of the
// min/max band. At fast timebases a column IS one sample, so this is
// textbook average mode there (~sqrt(N) noise reduction); at slow timebases
// it steadies the centre of the band. Only NEW frames advance the average -
// a pan redraw of the same record must not multiply-count it.
static void display_average(void)
{
  uint32_t gen = capture_get_generation();
  int shift = config.average_mode + 1; // N = 2 << mode
  bool fresh = (gen != g_avg_gen);

  g_avg_gen = gen;

  if (config.average_mode <= 0)
    return;

  for (int c = 0; c < GRID_WIDTH; c++)
  {
    int px;

    if (!(g_display_buffer.flags[c] & SAMPLE_FLAG_VALID))
    {
      g_avg_have[c] = 0;
      continue;
    }

    px = (g_display_buffer.min[c] + g_display_buffer.max[c]) * 32; // mid * 64

    if (!g_avg_have[c])
    {
      g_avg_acc[c] = px;
      g_avg_have[c] = 1;
    }
    else if (fresh)
    {
      int acc = g_avg_acc[c];

      acc += (px - acc) >> shift;
      g_avg_acc[c] = acc;
    }

    px = (g_avg_acc[c] + 32) / 64;

    g_display_buffer.min[c] = px;
    g_display_buffer.max[c] = px;
  }
}

//-----------------------------------------------------------------------------
// Persistence: fold what is about to be displayed into the envelope.
//
// Infinite mode is the plain union of every frame, pinned at full brightness
// - the runt that fired once an hour ago is still there, which is the whole
// point of it.
//
// Decay mode is the same envelope with a brightness that runs out, and one
// rule makes it behave like a phosphor: the glow is refreshed only where the
// envelope GROWS, not merely where the trace is. A steady trace therefore
// stops refreshing its own column immediately and its envelope fades to
// nothing underneath it, while the one excursion that reached further lights
// up and then dies away over PERSIST_DECAY_MS. Refreshing wherever the trace
// happened to be would pin every column at full brightness forever, which is
// infinite persistence again with extra steps.
//
// A column whose glow has run out drops back to the live band, and by then
// it is black, so nothing is seen to snap.
static void display_persist_accum(void)
{
  uint32_t now = timer_ms();
  int drop = 0;

  if (config.persist_mode == PERSIST_OFF)
    return;

  if (config.persist_mode == PERSIST_DECAY)
  {
    drop = (int)(((now - g_persist_stamp) * 255) / PERSIST_DECAY_MS);

    // Only when it came to something: at a few hundred frames a second the
    // elapsed time per frame rounds to no decay at all, and advancing the
    // stamp anyway would throw that time away and leave the glow immortal
    if (drop > 0)
      g_persist_stamp = now;
  }

  for (int c = 0; c < GRID_WIDTH; c++)
  {
    int lo = g_display_buffer.min[c], hi = g_display_buffer.max[c];
    bool grew_top = false, grew_bot = false;

    if (!(g_display_buffer.flags[c] & SAMPLE_FLAG_VALID))
      continue;

    if (!g_persist_lvl[c])
    {
      g_persist_min[c] = lo;
      g_persist_max[c] = hi;
      grew_top = grew_bot = true;
    }
    else
    {
      if (lo < g_persist_min[c])
      {
        g_persist_min[c] = lo;
        grew_top = true;
      }

      if (hi > g_persist_max[c])
      {
        g_persist_max[c] = hi;
        grew_bot = true;
      }
    }

    if (config.persist_mode == PERSIST_INFINITE)
    {
      g_persist_lvl[c] = 255;
      continue;
    }

    if (grew_top || grew_bot)
      g_persist_lvl[c] = 255;
    else
      g_persist_lvl[c] = (g_persist_lvl[c] > drop) ? (g_persist_lvl[c] - drop) : 0;

    // ...and the glow pulls back toward the live trace as it fades.
    //
    // Dimming alone is not enough, because it is per column and a column that
    // grows at all is pinned bright - which is most of them on any steep
    // edge, where a pixel of trigger jitter moves the band every frame. That
    // pinned the WHOLE accumulated band, so an excursion from a minute ago
    // stayed at full brightness underneath a live edge that had nothing to do
    // with it. Each end therefore retreats on its own, and only the end that
    // did not just grow: an edge column re-lights the side the beam is on and
    // sheds the side it left.
    //
    // Exponential, with a floor of one pixel so the last few always close.
    if (drop > 0 && !grew_top)
    {
      int step = ((lo - g_persist_min[c]) * drop) / 255;

      g_persist_min[c] += (step > 1) ? step : 1;

      if (g_persist_min[c] > lo)
        g_persist_min[c] = lo;
    }

    if (drop > 0 && !grew_bot)
    {
      int step = ((g_persist_max[c] - hi) * drop) / 255;

      g_persist_max[c] -= (step > 1) ? step : 1;

      if (g_persist_max[c] < hi)
        g_persist_max[c] = hi;
    }
  }
}

//-----------------------------------------------------------------------------
// Measurement cursors. Both mappings are exactly the arithmetic the trace
// itself is drawn with, so the cursors track pan and zoom.
static int cursor_t_col(int64_t t_ns)
{
  return GRID_WIDTH/2 + (int)((t_ns - config.horizontal_position) /
      hs_px_value[config.horizontal_scale]);
}

//-----------------------------------------------------------------------------
static int cursor_v_row(int mv)
{
  int px = (mv - config.vertical_position_mv) / vs_mv_px(config.vertical_scale) +
      config.vertical_position;

  return clip_for_display(px);
}

//-----------------------------------------------------------------------------
// The whole status line while cursors are up: both deltas and the implied
// frequency, with the active cursor named so the arrows have an obvious
// target
void cursor_readout(void)
{
  static const char *const names[] = { "", "T1", "T2", "V1", "V2" };
  char dt[14], f[14], dv[14], buf[64];
  int64_t d = g_cursor_t[1] - g_cursor_t[0];

  if (g_toast_active)
    return;

  format_ns(d, dt, sizeof(dt));
  format_mv(g_cursor_v[0] - g_cursor_v[1], dv, sizeof(dv));

  if (d != 0)
  {
    int64_t ad = (d < 0) ? -d : d;

    format_hz_mhz(1000000000000ll / ad, f, sizeof(f));
  }
  else
  {
    snprintf(f, sizeof(f), "-");
  }

  lcd_fill_rect(0, GRID_BOTTOM+1, LCD_WIDTH, STATUS_LINE_HEIGHT, BG_COLOR);
  lcd_set_color(BG_COLOR, CURSOR_T_COLOR);
  snprintf(buf, sizeof(buf), "%s dT %s 1/dT %s dV %s",
      names[g_cursor_sel], dt, f, dv);
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, buf);
}

//-----------------------------------------------------------------------------
// Arrow keys while a cursor is active: LEFT/RIGHT drag a time cursor,
// UP/DOWN drag a voltage cursor, one pixel per press, x10 with SHIFT.
// Everything else falls through to the normal handlers.
static bool cursor_buttons(int buttons, bool shift, bool repeat)
{
  int mul = shift ? 10 : 1;

  if (repeat)
    mul *= 4;

  if (g_cursor_sel == 1 || g_cursor_sel == 2)
  {
    int64_t step = (int64_t)hs_px_value[config.horizontal_scale] * mul;
    int64_t *t = &g_cursor_t[g_cursor_sel - 1];

    if (buttons & BTN_LEFT)
      *t -= step;
    else if (buttons & BTN_RIGHT)
      *t += step;
    else
      return false;
  }
  else
  {
    int step = vs_mv_px(config.vertical_scale) * mul;
    int *v = &g_cursor_v[g_cursor_sel - 3];

    if (buttons & BTN_UP)
      *v += step;
    else if (buttons & BTN_DOWN)
      *v -= step;
    else
      return false;
  }

  g_cursor_dirty = true;
  cursor_readout();

  return true;
}

//-----------------------------------------------------------------------------
// Roll view
//
// A triggered sweep shows a record, and a record only exists once it is
// whole: at 500 ms/div that is six seconds of a frozen screen before six
// seconds of new signal replace all of it at once. Past that the useful
// picture is not a waveform at all - it is the input plotted against wall
// clock, appended a column at a time, which is what the stock firmware turns
// into above 500 ms/div and what this does.
//
// Nothing here waits for the acquisition. The ring is folded into a peak pair
// every pass of the main loop (capture_fold_samples), the pair accumulates
// into the column being written, and when that column has had its share of
// time the whole screen shifts left by one. The trigger takes no part - the
// acquisition is held in AUTO for the duration so the ring keeps turning and
// the measurements keep landing, whatever the trigger is set to.
//-----------------------------------------------------------------------------

// The timebase the user's roll threshold starts at. Index 0 is the default
// and means "only where a sweep cannot work at all", which is what the stock
// scope does; the rest pull roll down into timebases a sweep can still show,
// where it trades the trigger for a screen that keeps moving.
static int roll_threshold_hs(void)
{
  static const uint8_t from[] =
      { HS_ROLL_FIRST, HS_500_ms, HS_200_ms, HS_100_ms };

  int sel = config.roll_from;

  if (sel < 0 || sel >= (int)ARRAY_SIZE(from))
    sel = 0;

  return from[sel];
}

//-----------------------------------------------------------------------------
// Roll is a property of the timebase, not a mode with a key. The other views
// own the whole trace area and say what it means themselves, so none of them
// rolls: the spectrum has no time axis, the trend plots its own log, and a
// decoder needs the record roll refuses to wait for.
static bool roll_wanted(void)
{
  if (g_fft_mode || g_trend_mode || g_decode_mode || scope_calibration_mode ||
      g_autocal_active || g_autoset_active)
    return false;

  return config.horizontal_scale >= roll_threshold_hs();
}

//-----------------------------------------------------------------------------
// One display column's worth of time. Column, not division: 25 px per
// division is the whole resolution of the plot, and a column is what the
// envelope is accumulated into.
static int roll_column_us(void)
{
  int64_t us = hs_div_value[config.horizontal_scale] / (GRID_DIV_PX * 1000);

  return (us < 1) ? 1 : (int)us;
}

//-----------------------------------------------------------------------------
// Raw ADC count to display row, by the same route the swept trace takes:
// counts to millivolts through the range's calibration, millivolts to pixels
// through the volts/div, then the user's vertical position.
int roll_row(int raw)
{
  int mv = ((raw - ZERO_POINT) * config.vertical_mult + config.vertical_mult/2) /
      CALIB_MULTIPLIER;

  return clip_for_display((mv - config.vertical_position_mv) /
      vs_mv_px(config.vertical_scale) + config.vertical_position);
}

//-----------------------------------------------------------------------------
// The strip onto the screen. The columns are already in display space, so
// this is a copy - and it is a copy rather than the plot living there in the
// first place because the display buffer belongs to whatever view is up.
static void roll_publish(void)
{
  if (!g_roll_active)
    return; // another view owns the columns; the strip waits its turn

  memcpy(g_display_buffer.min, g_roll_row_min, GRID_WIDTH);
  memcpy(g_display_buffer.max, g_roll_row_max, GRID_WIDTH);
  memcpy(g_display_buffer.flags, g_roll_row_flags, GRID_WIDTH);

  g_shadow_valid = false;
  redraw_trace();
}

//-----------------------------------------------------------------------------
// Start the open column over and put the whole strip back on screen. Every
// path that rewrites the stored columns ends here.
//
// Deliberately does NOT restamp the clock. The strip is anchored to wall
// clock, and time that passed while the view was elsewhere is time the input
// was not watched: roll_task shifts blank columns in for it, so a break in
// the recording is drawn as a break rather than closed up.
static void roll_reopen(void)
{
  g_roll_min   = 255;
  g_roll_max   = 0;
  g_roll_acc_us = 0;
  roll_publish();
}

//-----------------------------------------------------------------------------
// Forget the plotted history. Only where there is nothing left to carry over:
// the first entry since boot, and coming back after longer than the strip
// spans. Restamps, because a blank strip has no gap left to draw.
static void roll_clear(void)
{
  memset(g_roll_row_min, GRID_HEIGHT/2-1, GRID_WIDTH);
  memset(g_roll_row_max, GRID_HEIGHT/2-1, GRID_WIDTH);
  memset(g_roll_row_flags, SAMPLE_FLAG_NONE, GRID_WIDTH);

  g_roll_stamp = timer_us();
  roll_reopen();
}

//-----------------------------------------------------------------------------
// A timebase step INSIDE the view: rescale the strip along time instead of
// dropping it.
//
// The sample rate does not move with the roll timebase - it is pinned, see
// ROLL_SR_DIVIDER - so a column means the same thing at every roll timebase:
// the peak envelope of a stretch of wall clock. Only the length of the
// stretch changes. Slower, and a new column is the union of the old ones its
// span covers; faster, and one old column feeds the several new ones that
// fall inside it.
//
// That second direction cannot un-blur anything, and does not pretend to: a
// column recorded over 400 ms knows only its own two extremes, and the four
// 100 ms columns it becomes all carry them. It is the envelope at the
// resolution it was RECORDED at - which is a true statement about the signal,
// where throwing ten minutes of history away over one keypress is not.
//
// In place, because a second copy of the strip is another 900 bytes of TCM.
// The two directions need opposite orders and the arithmetic is what
// guarantees it: compressing, a new column never reads anything to the right
// of itself, so the sweep goes right to left; expanding, never anything to
// the left, so it goes the other way. Either way a column is written only
// after everything that reads it has been.
static void roll_retime(int old_us, int new_us)
{
  int step = (new_us > old_us) ? -1 : 1;
  int j = (step < 0) ? GRID_WIDTH-1 : 0;

  for (int n = 0; n < GRID_WIDTH; n++, j += step)
  {
    // Age of this column's edges, in us behind the newest edge of the strip.
    // Column c of either scale covers [(W-1-c)*us, (W-c)*us).
    int lo = (GRID_WIDTH-1 - j) * new_us;
    int hi = lo + new_us - 1;
    int first = GRID_WIDTH-1 - hi / old_us; // oldest old column it touches
    int last  = GRID_WIDTH-1 - lo / old_us; // ...and the newest
    int min = GRID_HEIGHT, max = -1, flags = SAMPLE_FLAG_NONE;

    if (first < 0)
      first = 0;

    if (last > GRID_WIDTH-1)
      last = GRID_WIDTH-1;

    for (int i = first; i <= last; i++)
    {
      if (!(g_roll_row_flags[i] & SAMPLE_FLAG_VALID))
        continue;

      if (g_roll_row_min[i] < min)
        min = g_roll_row_min[i];

      if (g_roll_row_max[i] > max)
        max = g_roll_row_max[i];

      flags |= g_roll_row_flags[i];
    }

    // Nothing was ever recorded this far back: the strip now spans more time
    // than it has been running for, and those columns are honestly empty
    if (max < 0)
    {
      g_roll_row_min[j]   = GRID_HEIGHT/2-1;
      g_roll_row_max[j]   = GRID_HEIGHT/2-1;
      g_roll_row_flags[j] = SAMPLE_FLAG_NONE;
    }
    else
    {
      g_roll_row_min[j]   = min;
      g_roll_row_max[j]   = max;
      g_roll_row_flags[j] = flags;
    }
  }

  roll_reopen();
}

//-----------------------------------------------------------------------------
// A vertical change moves every stored column, and clearing an hour of
// history because the range was nudged is not an answer. The rows carry
// enough to move them: a row is the input's millivolts divided by the
// volts/div and offset by the vertical position, so the old settings give the
// millivolts back and the new ones put them somewhere else.
//
// What that cannot recover is a column that was already against the top or
// the bottom of the grid - clip_for_display() threw the rest of it away when
// the column was written, exactly as it does for the live trace. Zooming out
// therefore un-clips nothing; it only stops NEW columns from clipping.
//
// Runs whether or not the view is up. The strip outlives a look at the
// spectrum now, and a volts/div change made while looking at it would
// otherwise leave every stored row meaning something the mapping no longer
// says.
static void roll_rescale(int old_scale, int old_vpos)
{
  int scale = vs_mv_px(config.vertical_scale);

  for (int c = 0; c < GRID_WIDTH; c++)
  {
    if (!(g_roll_row_flags[c] & SAMPLE_FLAG_VALID))
      continue;

    // clip_for_display() is row = GRID_HEIGHT/2-1 - value, so this runs it
    // backwards into a value, into millivolts, and forwards again
    int lo = (GRID_HEIGHT/2-1 - g_roll_row_min[c] - old_vpos) * old_scale;
    int hi = (GRID_HEIGHT/2-1 - g_roll_row_max[c] - old_vpos) * old_scale;

    g_roll_row_min[c] = clip_for_display(lo / scale + config.vertical_position);
    g_roll_row_max[c] = clip_for_display(hi / scale + config.vertical_position);
  }

  roll_publish();
}

//-----------------------------------------------------------------------------
// Close the open column: everything one place to the left, the accumulated
// envelope into the column that just came free at the right.
static void roll_commit(void)
{
  int flags = SAMPLE_FLAG_NONE;

  memmove(&g_roll_row_min[0], &g_roll_row_min[1], GRID_WIDTH-1);
  memmove(&g_roll_row_max[0], &g_roll_row_max[1], GRID_WIDTH-1);
  memmove(&g_roll_row_flags[0], &g_roll_row_flags[1], GRID_WIDTH-1);

  if (g_roll_min <= g_roll_max)
  {
    // min/max are counts here and rows on the way out, and the two run
    // opposite ways: a bigger count is a HIGHER pixel, i.e. a smaller row
    g_roll_row_min[GRID_WIDTH-1] = roll_row(g_roll_max);
    g_roll_row_max[GRID_WIDTH-1] = roll_row(g_roll_min);

    flags = SAMPLE_FLAG_VALID;

    if (g_roll_min == 0)
      flags |= SAMPLE_FLAG_CLIP_L;

    if (g_roll_max == 255)
      flags |= SAMPLE_FLAG_CLIP_H;
  }
  else
  {
    g_roll_row_min[GRID_WIDTH-1] = GRID_HEIGHT/2-1;
    g_roll_row_max[GRID_WIDTH-1] = GRID_HEIGHT/2-1;
  }

  g_roll_row_flags[GRID_WIDTH-1] = flags;

  g_roll_min = 255;
  g_roll_max = 0;
}

//-----------------------------------------------------------------------------
// Called every pass of the main loop while the view is up
static void roll_task(void)
{
  uint32_t now = timer_us();
  int vmin, vmax, elapsed, columns = 0;

  if (g_roll_col_us <= 0)
    return; // never entered the view properly; nothing to divide time into

  if (capture_get_state() == CAPTURE_STATE_STOP)
  {
    // Frozen: the plot is a picture of the last minutes and the time spent
    // stopped is not part of it. Restart the clock so resuming appends the
    // next column rather than a screenful of blank ones.
    g_roll_stamp = now;
    return;
  }

  if (capture_fold_samples(&vmin, &vmax) > 0)
  {
    if (vmin < g_roll_min)
      g_roll_min = vmin;

    if (vmax > g_roll_max)
      g_roll_max = vmax;
  }

  elapsed = (int)(now - g_roll_stamp); // wrap-safe: 1 MHz into 32 bits
  g_roll_stamp = now;

  // More than a screen's worth of time since the last look. The scope does
  // not run while a menu is open, so this is mostly "the menu was up for a
  // while" - and a negative reading is the same thing after 35 minutes of it,
  // which is where a 1 MHz counter overruns what an int can hold. Either way
  // every column on screen is older than the screen, and shifting 300 empty
  // ones through one at a time only takes longer to say so.
  if (elapsed < 0 || elapsed > GRID_WIDTH * g_roll_col_us)
  {
    roll_clear();
    return;
  }

  g_roll_acc_us += elapsed;

  while (g_roll_acc_us >= g_roll_col_us)
  {
    g_roll_acc_us -= g_roll_col_us;
    roll_commit();
    columns++;
  }

  if (columns > 0)
  {
    // The sweep paints one column per pass and everything moved, so this is
    // a full repaint - ~11 ms of LCD writes spread over the next 300 passes,
    // which is why the roll threshold stops where it does
    roll_publish();
  }
}

//-----------------------------------------------------------------------------
// Enter or leave the view. Only the timebase decides this, so it is called
// from wherever the timebase or a view flag can have changed rather than from
// a key.
static void roll_set_active(bool active)
{
  if (active == g_roll_active)
  {
    // Same view, different timebase: a column covers a new amount of time,
    // and what is already plotted gets rescaled onto it
    if (active && g_roll_col_us != roll_column_us())
    {
      int old_us = g_roll_col_us;

      g_roll_col_us = roll_column_us();
      roll_retime(old_us, g_roll_col_us);
    }

    return;
  }

  g_roll_active = active;
  g_state = -1; // the state slot says ROLL or does not

  if (active)
  {
    int old_us = g_roll_col_us;
    int stale_min, stale_max;

    // Panning has nothing to pan: the newest column is the present, and
    // there is no record either side of the screen to walk into
    config.horizontal_position = 0;
    config.horizontal_position_px = 0;

    g_roll_col_us = roll_column_us();

    // The ring has been turning the whole time this view was not up, and the
    // fold's read point with it. Take that stretch and throw it away, or the
    // first column back would carry up to a ring of signal in it.
    capture_fold_samples(&stale_min, &stale_max);

    // Whatever is stored is still a strip of the same input at the same
    // sample rate; it is only OLDER, and roll_task shifts blank columns in
    // for the time spent away (or clears it, past a screenful). A timebase
    // moved while another view was up gets the same rescaling as one moved
    // in here - the stored columns just cover a different amount of time.
    if (old_us > 0 && old_us != g_roll_col_us)
      roll_retime(old_us, g_roll_col_us);
    else if (old_us <= 0)
      roll_clear(); // first entry since boot: there is no strip yet
    else
      roll_publish();
  }

  // AUTO while rolling whatever the trigger says, and the user's mode back
  // on the way out. A NORMAL sweep that never triggers would leave the
  // measurements frozen on whatever record was last published, and SINGLE
  // stops the ring outright - which would look exactly like a hung scope.
  //
  // Not capture_start(): a stop is the user's, and a timebase key is not a
  // request to undo one. Rolling into a stopped acquisition shows the frozen
  // strip and STOP in the state slot, and RUN starts it moving.
  capture_set_trigger_mode(active ? TRIGGER_MODE_AUTO : config.trigger_mode);
}

//-----------------------------------------------------------------------------
// Re-decide the view after something OTHER than the timebase changed what it
// should be - a view toggle, the threshold setting. The timebase key does
// this itself, in the middle of its own rescaling; everyone else calls here.
void roll_sync(void)
{
  bool want = roll_wanted();

  if (want == g_roll_active)
    return;

  roll_set_active(want);
  update_sample_rate(); // the acquisition is timed differently either way
}

//-----------------------------------------------------------------------------
// How much time the screen holds, for the readout that shows the pan offset
// on a swept trace - there is no pan here, and the span is what a strip chart
// is read against
int64_t roll_screen_ns(void)
{
  return hs_div_value[config.horizontal_scale] * GRID_DIVS_H;
}

//-----------------------------------------------------------------------------
// How tall the bit grid's lines are: exactly what the waveform spans under
// the decoded bytes, top pixel to bottom pixel. Measured over the decoded
// columns and not the whole screen, because those are the only columns the
// grid is drawn in - a marker pulse somewhere else in the record has nothing
// to do with how tall a line over THIS byte should be.
//
// Per frame, not per decode: the bytes move when the record changes, the
// signal's height moves whenever the vertical scale, the position or the
// signal itself does.
static void dbit_span_update(void)
{
  int top = GRID_HEIGHT, bot = -1;

  if (!g_dbit_on)
    return;

  for (int c = 0; c < GRID_WIDTH; c++)
  {
    if (g_dband_byte[c] < 0)
      continue;

    if (!(g_display_buffer.flags[c] & SAMPLE_FLAG_VALID))
      continue;

    if (g_display_buffer.min[c] < top)
      top = g_display_buffer.min[c];

    if (g_display_buffer.max[c] > bot)
      bot = g_display_buffer.max[c];
  }

  g_dbit_top = top;
  g_dbit_bot = (bot > GRID_HEIGHT - 2) ? GRID_HEIGHT - 2 : bot;

  // ...and it stops at the band. The line is a ruler for the WAVEFORM - that
  // is the whole argument for drawing it over the trace - and the band is not
  // waveform, it is screen the band took away from it. Run on into the band
  // and the ruler becomes a hairline down the middle of the text there, which
  // the band can only dim and not remove.
  if (g_dband_rows && g_dbit_bot >= g_dband_row0)
    g_dbit_bot = g_dband_row0 - 1;

  // The numbers go in the middle of that span. A digital trace lives at its
  // two rails and crosses the middle only on its edges - which is exactly
  // where the lines are and exactly where the numbers are not.
  int row = (g_dbit_top + g_dbit_bot) / 2 - 4;

  if (row < 0)
    row = 0;

  if (row + 8 > GRID_HEIGHT)
    row = GRID_HEIGHT - 8;

  g_dbit_text_row = row;
}

//-----------------------------------------------------------------------------
void update_display(void)
{
  int scale = vs_mv_px(config.vertical_scale);

  // The display buffer is the trend plot while that view is up: a frame
  // landing (or a pan-settle refresh) must not overwrite it with the trace
  if (g_trend_mode)
  {
    trend_view_update();
    return;
  }

  // ...and it is the roll strip while THAT view is up, which the columns of
  // a record would wipe out. Nothing to rebuild: roll_task() keeps the strip
  // in its own store and this only puts it back on screen.
  if (g_roll_active)
  {
    roll_publish();
    return;
  }

  g_data_buffer.size = GRID_WIDTH;
  capture_get_data(&g_data_buffer);

  // Whether what was just drawn between the samples is still worth believing.
  // Only the reconstruction can overstate itself this way - straight lines
  // never claimed to be the signal - so the strain is only reported when it
  // is switched on.
  {
    ScopeMeasure sm;
    const uint8_t *data;
    int size, offset, period_ns, trig;

    g_recon_strained = false;

    if (config.draw_mode == DRAW_SINC &&
        capture_get_record(&data, &size, &offset, &period_ns, &trig) &&
        period_ns > 0 && capture_get_measurements(&sm) && sm.frequency > 0)
    {
      g_recon_strained =
          (sm.period_med_ns < RECON_MIN_SAMPLES * period_ns);
    }
  }

  for (int i = 0; i < GRID_WIDTH; i++)
  {
    int min = (g_data_buffer.min[i] - g_data_buffer.vertical_position) / scale + config.vertical_position;
    int max = (g_data_buffer.max[i] - g_data_buffer.vertical_position) / scale + config.vertical_position;

    g_display_buffer.min[i]   = clip_for_display(max);
    g_display_buffer.max[i]   = clip_for_display(min);
    g_display_buffer.flags[i] = g_data_buffer.flags[i];
  }

  display_average();            // thin running-mean trace instead of the band
  close_gaps(&g_display_buffer);
  display_persist_accum();      // envelope of what is actually displayed

  dbit_span_update();           // the bit grid is as tall as the trace is

  redraw_trace();
}

//-----------------------------------------------------------------------------
// Repaint whatever the trace area currently shows. Vertical and timebase
// changes are meaningful in the spectrum view too (they set the ADC range
// and the record length), but they must not paint a waveform over it.
void refresh_view(void)
{
  if (g_fft_mode)
    fft_update();
  else
    update_display();
}

//-----------------------------------------------------------------------------
void change_horizontal_scale(int delta)
{
  if ((delta < 0 && config.horizontal_scale == 0) ||
      (delta > 0 && config.horizontal_scale == HS_LAST))
    return;

  config.horizontal_scale += delta;
  config.horizontal_period = hs_px_value[config.horizontal_scale];

  // Roll is a property of the timebase and this is the only key that changes
  // it. Ahead of the pan being rescaled, because entering roll zeroes it.
  roll_set_active(roll_wanted());

  config.horizontal_position_px = config.horizontal_position / hs_px_value[config.horizontal_scale];

  draw_horizontal_scale();
  draw_horizontal_position();
  update_sample_rate();
  refresh_view();
}

//-----------------------------------------------------------------------------
static void change_horizontal_position_ex(int delta, bool repeat)
{
  int div = hs_px_value[config.horizontal_scale];

  // Rolling: the right-hand column is now and there is nothing either side of
  // the screen to walk into - the record a pan moves through is the very
  // thing roll does without
  if (g_roll_active)
    return;

  if ((delta < 0 && config.horizontal_position <= MIN_HORIZONTAL_POSITION) ||
      (delta > 0 && config.horizontal_position >= MAX_HORIZONTAL_POSITION))
    return;

  config.horizontal_position_px = config.horizontal_position / div + delta;
  config.horizontal_position = (int64_t)config.horizontal_position_px * div;

  draw_horizontal_position();

  // While the key is held, restarting the DMA and repainting the miniview
  // on every 10-50 ms repeat tick is what makes panning feel sticky: do the
  // heavy part every 4th tick and settle once after release
  if (repeat)
  {
    g_pan_settle_timer = 200;

    if (++g_pan_repeat_count % 4 == 0)
      update_sample_rate();
  }
  else
  {
    g_pan_repeat_count = 0;
    update_sample_rate();
  }

  refresh_view();
}

//-----------------------------------------------------------------------------
void change_vertical_scale(int delta)
{
  // Clamped, not just guarded at the ends: the auto-setup jumps straight to
  // the scale it computed, so |delta| is not always 1
  int scale = config.vertical_scale + delta;

  if (scale < 0)
    scale = 0;
  else if (scale > VS_LAST)
    scale = VS_LAST;

  if (scale == config.vertical_scale)
    return;

  int old_scale = vs_mv_px(config.vertical_scale);

  config.vertical_scale = scale;
  config.vertical_mult = config.calib_vs_mult[config.vertical_scale] * config_probe_mult();
  config.vertical_position_mv = config.vertical_position * vs_mv_px(config.vertical_scale);

  config.trigger_level_mv = config.trigger_level * vs_mv_px(config.vertical_scale);

  scope_display_settings_changed(); // px-to-mV mapping changed under them
  roll_rescale(old_scale, config.vertical_position); // ...including the plot's
  capture_set_vertical_parameters();
  capture_set_trigger_level(config.trigger_level_mv);
  draw_vertical_scale();
  draw_trigger_level();
  refresh_view();
}

//-----------------------------------------------------------------------------
static void change_vertical_position_ex(int delta, bool repeat)
{
  if ((delta < 0 && config.vertical_position <= MIN_VERTICAL_POSITION) ||
      (delta > 0 && config.vertical_position >= MAX_VERTICAL_POSITION))
    return;

  int old_vpos = config.vertical_position;

  config.vertical_position += delta;
  config.vertical_position_mv = config.vertical_position * vs_mv_px(config.vertical_scale);

  scope_display_settings_changed(); // every column moved on screen
  roll_rescale(vs_mv_px(config.vertical_scale), old_vpos); // the plot too

  // Same held-key coalescing as horizontal pan: the DAC/DMA restart runs
  // every 4th repeat tick, the settle timer finalizes after release
  if (repeat)
  {
    g_pan_settle_timer = 200;

    if (++g_pan_repeat_count % 4 == 0)
      capture_set_vertical_parameters();
  }
  else
  {
    g_pan_repeat_count = 0;
    capture_set_vertical_parameters();
  }

  draw_vertical_position(true);
  draw_trigger_level();
  refresh_view();
}

//-----------------------------------------------------------------------------
// Push config.trigger_level (px) to hardware. Deliberately draw-free: also
// called from the system menu, which overlays the marker gutter (the screen
// is repainted in full when the menu closes).
void scope_apply_trigger_level(void)
{
  if (config.trigger_level < MIN_TRIGGER_LEVEL)
    config.trigger_level = MIN_TRIGGER_LEVEL;
  else if (config.trigger_level > MAX_TRIGGER_LEVEL)
    config.trigger_level = MAX_TRIGGER_LEVEL;

  config.trigger_level_mv = config.trigger_level * vs_mv_px(config.vertical_scale);

  capture_set_trigger_level(config.trigger_level_mv);
}

//-----------------------------------------------------------------------------
static void change_trigger_level(int delta)
{
  if ((delta < 0 && config.trigger_level == MIN_TRIGGER_LEVEL) ||
      (delta > 0 && config.trigger_level == MAX_TRIGGER_LEVEL))
    return;

  config.trigger_level += delta;
  scope_apply_trigger_level();
  draw_trigger_level();
}

//-----------------------------------------------------------------------------
static void change_sample_rate_limit(int delta)
{
  if ((delta < 0 && config.sample_rate_limit == 0) ||
      (delta > 0 && config.sample_rate_limit == MAX_SAMPLE_RATE_LIMIT))
    return;

  config.sample_rate_limit += delta;

  update_sample_rate();
}

//-----------------------------------------------------------------------------
// Map every trace column to the decoded byte its center sample belongs to
//-----------------------------------------------------------------------------
// The timebase that holds a whole message at this rate.
//
// The record is 24576 samples of whatever the timebase makes them, and the
// window on the screen is what picks the sample rate, so the two are the same
// question. The trigger sits in the middle of the record, so only the second
// half carries a message that starts at it - hence the doubling. And a result
// holds LOGIC_MAX_BYTES, so there is no point in a window wider than that
// many frames: past it the record grows and the decode does not.
static int decode_fit_scale(int baud)
{
  if (baud <= 0)
    return -1;

  // start + 8 data + stop, twice over for the pre-trigger half of the record
  int64_t want_ns = (int64_t)LOGIC_MAX_BYTES * 10 * 2 * 1000000000ll / baud;

  // Sweep timebases only: the fit exists to put a whole message in a RECORD,
  // and past HS_SWEEP_LAST there is no record to put it in
  for (int hs = 0; hs <= HS_SWEEP_LAST; hs++)
  {
    if (hs_div_value[hs] * GRID_DIVS_H >= want_ns)
      return hs;
  }

  return HS_SWEEP_LAST;
}

//-----------------------------------------------------------------------------
// Put the whole message on the screen once the rate is known, so reading a
// UART does not start with hunting for a timebase. Done once per lock, not
// per decode: after this the timebase is the user's again, to zoom into a
// byte with, and nothing takes it back.
static void decode_fit_window(int baud)
{
  int hs = decode_fit_scale(baud);

  if (config.decoder_fit_mode != 0 || hs < 0 || hs == config.horizontal_scale)
    return;

  config.horizontal_scale = hs;
  config.horizontal_position_px = config.horizontal_position / hs_px_value[hs];
  config.horizontal_period = hs_px_value[hs];

  // The frames on the panel were measured in a record this has just thrown
  // away; they stay up as held until the next decode replaces them
  g_decode_held = true;
  g_decode_force = true;

  draw_horizontal_scale();
  draw_horizontal_position();
  update_sample_rate();
  refresh_view();

  char msg[48];

  snprintf(msg, sizeof(msg), "Timebase set for %d baud", baud);
  toast_show();
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
}

//-----------------------------------------------------------------------------
// Hunt for the head of a message.
//
// There is no edge that means "start of message". The hardware triggers on an
// edge and every bit of a frame is one, so the trigger cannot be asked for it
// and never could be. What CAN be recognised, after the fact, is the record
// that caught the line RESTING before its first frame - that rest is the only
// thing that distinguishes the head of a message from its middle.
//
// So the hunt is: trigger on the falling edge that a start bit is, put that
// edge one division from the left so the record runs forward from it instead
// of spending half of itself on what came before, then decode every record
// that arrives - not one every 250 ms - until one decodes cleanly with an
// idle in front of it. That record is the head of a message, and acquisition
// freezes on it.
//
// The trigger goes to NORMAL for the duration: in AUTO the acquisition gives
// up waiting after 100 ms and hands over a record of the idle line, and a
// message every 200 ms means most of the hunt would be spent on those.
static void trigger_set_50_percent(void); // defined with the other trigger helpers

static void decode_arm_hunt(void)
{
  int baud = decoder_baud_value();
  char msg[48];

  if (baud <= 0 && g_logic_have && decode_proto_is_serial(g_logic.proto))
    baud = g_logic.rate; // not told, but a record has already said

  if (baud > 0)
  {
    g_decode_fitted = true;
    decode_fit_window(baud);
  }

  config.trigger_mode = TRIGGER_MODE_NORMAL;
  capture_set_trigger_mode(config.trigger_mode);

  // A start bit pulls an idle-high line down. Inverted logic is rare enough
  // that the only evidence worth trusting is a decode that already said so.
  config.trigger_edge = (g_logic_have && decode_proto_is_serial(g_logic.proto) &&
      !g_logic.idle_high) ? TRIGGER_EDGE_RISE : TRIGGER_EDGE_FALL;
  capture_set_trigger_edge(config.trigger_edge);

  trigger_set_50_percent();

  // One division of what came before is all the hunt needs - the test is for
  // a frame time of idle - and the other eleven go to the message
  int div = hs_div_value[config.horizontal_scale];
  int64_t hpos = (int64_t)div * (GRID_DIVS_H / 2 - 1);

  if (hpos > MAX_HORIZONTAL_POSITION)
    hpos = MAX_HORIZONTAL_POSITION;

  config.horizontal_position = hpos;
  config.horizontal_position_px = (int)(hpos / hs_px_value[config.horizontal_scale]);

  draw_trigger_mode();
  draw_trigger_edge();
  draw_horizontal_position();
  update_sample_rate();
  capture_start();
  refresh_view();

  g_decode_hunt = true;
  g_decode_force = true;

  if (baud > 0)
    snprintf(msg, sizeof(msg), "Waiting for a message start, %d baud", baud);
  else
    snprintf(msg, sizeof(msg), "Waiting for a message start");

  toast_show();
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
}

//-----------------------------------------------------------------------------
// The bit grid draws over the whole height of the grid, not just over the
// band, so switching it off has to repaint everything - clearing the band
// alone would leave the hairlines standing on a trace that no longer has a
// reason for them.
void scope_decode_redraw(void)
{
  mpanel_invalidate();
  g_shadow_valid = false;   // every column has to be composed again
}

//-----------------------------------------------------------------------------
// The first half of reading an SPI bus with one probe: while the probe is
// still on SCK, write down the frequency the scope is already measuring. The
// second half is moving the probe to MOSI, and by then this number is the
// only thing that says how wide a bit is.
//
// Taken from the RAW measurement rather than from the decoder: the clock line
// is a clock, the frequency counter is what reads clocks, and the decoder has
// nothing to say about a signal it is not being pointed at.
void scope_spi_clock_capture(void)
{
  g_spi_clock_request = true;
}

//-----------------------------------------------------------------------------
// The same two-pass workflow for SWD: the probe is on SWCLK now, SWDIO next.
// Its own stored number rather than a share of the SPI one, because a bench
// with both buses on it would otherwise re-measure every time the probe moved
// from one to the other.
void scope_swd_clock_capture(void)
{
  g_swd_clock_request = true;
}

//-----------------------------------------------------------------------------
void scope_decode_catch_start(void)
{
  g_decode_hunt_request = true;
}

//-----------------------------------------------------------------------------
// Run the protocol auto-decoder over the current record; flag the panel for
// a repaint only when the result actually changed. Optionally freeze the
// capture the moment a structured protocol is caught ("trigger on decode").
static void decode_update(void)
{
  const uint8_t *data;
  int size, offset, period_ns, trig_pos;
  // Static, not automatic: a LogicResult is over half a kilobyte now that a
  // whole console line fits in one, and logic_decode() has one of its own
  static LogicResult res;
  static uint32_t last_ms = 0;

  // Decoding a 24K record (worse: the whole auto-detect cascade) on every
  // acquisition starves the button path at fast frame rates; 4 Hz reading
  // of a serial console is indistinguishable from instant. The throttle is
  // on time alone: keying it off "have a result" let a decoder that never
  // matches anything run flat out, which is exactly the case that hurts.
  //
  // The hunt is the one case that cannot afford it. A message every 200 ms
  // against four looks a second is a coin toss repeated slowly: minutes to
  // catch what the acquisition is handing over sixty times a second.
  if (!g_decode_force && !g_decode_hunt && timer_ms() - last_ms < 250)
    return;

  g_decode_force = false;
  last_ms = timer_ms();

  if (!capture_get_record(&data, &size, &offset, &period_ns, &trig_pos))
    return;

  uart_decode_set_baud(decoder_baud_value());
  spi_decode_set_clock(config.spi_clock_hz);
  spi_decode_set_order(config.spi_order);
  swd_decode_set_clock(config.swd_clock_hz);
  manchester_decode_set_rate(decoder_man_rate_value());
  manchester_decode_set_polarity(config.man_polarity);

  // Sticky protocol: once something matched, try it alone first and only
  // fall back to the full auto cascade when it stops matching
  proto_t forced = (proto_t)config.decoder_proto;

  if (forced == PROTO_AUTO && g_logic_have && g_logic.proto != PROTO_NONE &&
      g_logic.count > 0)
  {
    logic_decode(data, size, offset, period_ns, g_logic.proto,
        (LogicScratch *)CAPTURE_SPARE_RAM, &res);

    if (res.count == 0)
      logic_decode(data, size, offset, period_ns, PROTO_AUTO,
          (LogicScratch *)CAPTURE_SPARE_RAM, &res);
  }
  else
  {
    logic_decode(data, size, offset, period_ns, forced,
        (LogicScratch *)CAPTURE_SPARE_RAM, &res);
  }

  // A record can decode as UART and be, sample for sample, a square wave:
  // 0x55 back to back at 115200 IS a 57.6 kHz square wave, and the "UUUU"
  // preamble of a test message is exactly that. Nothing in such a record
  // settles the question, so it is taken only once another record has - one
  // that caught the line resting between frames. That first record is the
  // proof; after it, the preamble decodes like everything else.
  if (res.count > 0 && res.proto == PROTO_UART)
  {
    if (!res.ambiguous)
      g_uart_locked = true;
    else if (!g_uart_locked)
      res.count = 0;
  }

  // Hunting: a record that is not the head of a message is not shown at all.
  // Taking it would fill the panel with the middle of one, decoded from
  // whatever phase the record happened to start in - which is the very thing
  // the hunt is here to avoid. So it is dropped, the panel keeps what it had,
  // and the next record gets its turn. Only the serial ones need the test:
  // 1-Wire, WS2812 and NEC each match from their own leader, so a match is
  // already a head.
  if (g_decode_hunt)
  {
    if (res.count == 0 || res.proto == PROTO_RAW ||
        (decode_proto_is_serial(res.proto) &&
         !(res.burst_start && res.errors == 0)))
      return;

    g_decode_hunt = false;

    if (capture_get_state() != CAPTURE_STATE_STOP)
      capture_stop();

    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Message start caught (STOP resumes)");
  }

  // Most records hold nothing to decode. A 200 ms burst is 2% of the time,
  // and AUTO force-triggers on the idle line whenever the traffic keeps it
  // waiting - so the acquisition hands over one empty record after another
  // and only occasionally one with a message in it. Blanking the panel on
  // every empty one is what makes a working decoder look like a broken one:
  // the frames flash up for a quarter of a second and are gone. So the last
  // frames stay on the panel, marked as held, until new ones replace them.
  // Only STOP mode shows the record itself, and there the panel is live.
  if (res.count == 0 && g_logic_have && g_logic.count > 0)
  {
    if (!g_decode_held)
    {
      g_decode_held = true;
      g_decode_panel_pending = true;
      redraw_miniview();   // the byte map belongs to a record that is gone
      decode_band_build(); // and so do the marks on the trace
    }

    return;
  }

  g_decode_period_ns = period_ns;
  g_decode_trig_pos = trig_pos;
  g_decode_size = size;

  bool changed = !g_logic_have || g_decode_held || res.count != g_logic.count ||
      res.proto != g_logic.proto || res.rate != g_logic.rate ||
      memcmp(res.bytes, g_logic.bytes, (size_t)res.count) != 0;

  if (changed)
  {
    g_logic = res;

    if (res.proto == PROTO_ONEWIRE)
      g_pana.ow = *onewire_analysis();
    else if (res.proto == PROTO_CAN)
      g_pana.can = *can_analysis();
    else if (res.proto == PROTO_DHT)
      g_pana.dht = *dht_analysis();
    else if (res.proto == PROTO_SENT)
      g_pana.sent = *sent_analysis();
    else if (res.proto == PROTO_MIDI)
      g_pana.midi = *midi_analysis();
    else if (res.proto == PROTO_LIN)
      g_pana.lin = *lin_analysis();
    else if (res.proto == PROTO_EV1527)
      g_pana.ev = *ev1527_analysis();
    else if (res.proto == PROTO_DSHOT)
      g_pana.dshot = *dshot_analysis();
    else if (res.proto == PROTO_SPI)
      g_pana.spi = *spi_analysis();
    else if (res.proto == PROTO_MANCH)
      g_pana.man = *manchester_analysis();
    else if (res.proto == PROTO_RC5)
      g_pana.rc5 = *rc5_analysis();
    else if (res.proto == PROTO_DALI)
      g_pana.dali = *dali_analysis();
    else if (res.proto == PROTO_KNX)
      g_pana.knx = *knx_analysis();
    else if (res.proto == PROTO_WS2812)
      g_pana.ws = *ws2812_analysis();
    else if (res.proto == PROTO_SWO)
      g_pana.swo = *swo_analysis();
    else if (res.proto == PROTO_SWD)
      g_pana.swd = *swd_analysis();
    else if (res.proto == PROTO_USB)
      g_pana.usb = *usb_analysis();
    else if (res.proto == PROTO_PD)
      g_pana.pd = *pd_analysis();
    else if (res.proto == PROTO_SIRC)
      g_pana.sirc = *sirc_analysis();
    else if (res.proto == PROTO_PPM)
      g_pana.ppm = *ppm_analysis();

    g_logic_have = true;
    g_decode_held = false;
    g_decode_panel_pending = true;

    // First lock on a rate: put the whole message on the screen. Once only -
    // g_decode_fitted stays set until the decoder view is re-entered, so
    // zooming into a byte afterwards sticks.
    if (!g_decode_fitted && decode_proto_is_serial(res.proto) && res.rate > 0)
    {
      g_decode_fitted = true;
      decode_fit_window(res.rate);
    }

    if (g_decode_sel >= res.count)
      g_decode_sel = (res.count > 0) ? res.count - 1 : 0;

    // Stop-on-decode: RAW matches nearly anything, so it only counts as a
    // catch when the user explicitly selected it.
    //
    // "At message start" narrows it further. The hardware triggers on an
    // edge, and every bit in a frame is an edge, so a plain edge trigger
    // freezes on whatever byte the line happened to be sending - a record
    // out of the middle of a message, with its first frames missing. What
    // marks a real start is what comes BEFORE it: a line at rest for longer
    // than a frame. The decoder reports that as burst_start, so waiting for
    // it turns "stop on frames" into "stop on the head of a message" and the
    // record fills up forwards from there. Only the serial ones need asking:
    // 1-Wire, WS2812 and NEC each match from their own leader or reset
    // pulse, so a match already is the head of a message.
    if (config.decoder_stop && res.count > 0 &&
        (res.proto != PROTO_RAW || (proto_t)config.decoder_proto == PROTO_RAW) &&
        (!config.decoder_stop_start || !decode_proto_is_serial(res.proto) ||
            res.burst_start) &&
        capture_get_state() != CAPTURE_STATE_STOP)
      capture_stop();
  }

  // The byte map lives in the miniview, the byte marks on the trace itself:
  // repaint both when the result changed
  if (changed)
  {
    redraw_miniview();
    decode_band_build();
  }
}

//-----------------------------------------------------------------------------
// Center the display on the selected decoded byte (LEFT/RIGHT-style pan,
// but in whole bytes)
static void decode_jump_to_selected(void)
{
  if (!g_logic_have || g_logic.count == 0 || g_decode_period_ns <= 0)
    return;

  int64_t n_center = ((int64_t)g_logic.pos[g_decode_sel] +
      g_logic.end[g_decode_sel]) / 2 - g_decode_trig_pos;
  int64_t hpos = n_center * g_decode_period_ns;
  int div = hs_px_value[config.horizontal_scale];

  if (hpos < MIN_HORIZONTAL_POSITION)
    hpos = MIN_HORIZONTAL_POSITION;
  else if (hpos > MAX_HORIZONTAL_POSITION)
    hpos = MAX_HORIZONTAL_POSITION;

  config.horizontal_position_px = (int)(hpos / div);
  config.horizontal_position = (int64_t)config.horizontal_position_px * div;

  draw_horizontal_position();
  update_sample_rate(); // also repaints the miniview with the new selection
  update_display();

  g_decode_panel_pending = true; // the selection highlight moved
}

//-----------------------------------------------------------------------------
// First byte of a `width`-wide window over `count` bytes that keeps `sel`
// in view, centred where it can be and clamped at either end
static int window_base(int sel, int width, int count)
{
  int base = sel - width / 2;

  if (base > count - width)
    base = count - width;

  return (base < 0) ? 0 : base;
}

//-----------------------------------------------------------------------------
// Decoded-bytes overlay, drawn after a trace sweep completes so the sweep
// does not immediately paint over it (unchanged columns are skipped by the
// dirty-column logic, which keeps the panel intact on a stable signal)
#define DECODE_HEX_COLS   8   // per row, two rows of them
#define DECODE_CHAR_W     6   // FONT_SMALL is 6x8

//-----------------------------------------------------------------------------
// What byte `idx` is, in a word, for the row under the hex. Never empty: a
// byte the protocol has no name for still holds its place in the row, or the
// labels stop lining up with the numbers they belong to.
static void decode_byte_label(char *buf, int size, int idx)
{
  dband_meaning_text(buf, size, idx, g_logic.bytes[idx]);

  if (!buf[0])
    snprintf(buf, size, "..");
}

//-----------------------------------------------------------------------------
// How many character cells the label takes, swatch included. The swatch is a
// cell wide by design, so this row's arithmetic stays in characters and the
// pass that decides how many labels fit cannot drift from the pass that
// draws them.
static int decode_label_cells(const char *lab, int idx)
{
  uint16_t col;

  return (int)strlen(lab) + (decode_group_color(idx, &col) ? 1 : 0);
}

// The text row is 30 characters wide and the outer two are the < > that say
// the message runs on past what is shown
#define DECODE_TEXT_MAX  26

static void draw_decode_panel(void)
{
  char line[36];

  g_decode_panel_pending = false;

  lcd_fill_rect(DECODE_PANEL_X, DECODE_PANEL_Y, DECODE_PANEL_W, DECODE_PANEL_H, BG_COLOR);
  lcd_draw_rect(DECODE_PANEL_X, DECODE_PANEL_Y, DECODE_PANEL_W, DECODE_PANEL_H, GRID_FG_COLOR);

  lcd_set_font(FONT_SMALL);
  lcd_set_color(BG_COLOR, TOAST_COLOR);

  if (!g_logic_have || g_logic.count == 0)
  {
    lcd_puts(DECODE_PANEL_X + 4, DECODE_PANEL_Y + 4, "Decoder: no frames");
  }
  else
  {
    // The panel is a window onto the result, not the result: two rows of
    // eight is all the hex that fits. That looked exactly like a 16-byte
    // limit until the header started saying how many bytes there are, so
    // "0123456789ABCDEF" out of a 32-byte message reads as a window with
    // more to its right, and TRIG_UP/DN is visibly the way to it.
    int hex_base = window_base(g_decode_sel, DECODE_HEX_COLS * 2, g_logic.count);
    bool text = decode_proto_is_text();
    int sel_start = g_decode_sel, sel_len = 1;

    decode_group_at(g_decode_sel, &sel_start, &sel_len);
    // Text is what a console line is for and this row is 30 characters wide,
    // so it shows nearly twice the bytes the hex rows do. On a protocol that
    // carries no text the same row says what the bytes ARE instead - ADDR,
    // ~ADDR, CMD, ~CMD for a remote control - and that is written out in
    // words, so it follows the hex window and fits as many as it can.
    int txt_show = (g_logic.count < DECODE_TEXT_MAX) ? g_logic.count : DECODE_TEXT_MAX;
    int txt_base = text ? window_base(g_decode_sel, DECODE_TEXT_MAX, g_logic.count)
        : hex_base;

    if (!text)
    {
      // The label row is a flow of words, not eight fixed columns, so it
      // holds far fewer of them than there are numbers above it, and how many
      // depends on how long the words are. Two things follow from that, and
      // both were missing.
      //
      // It has to SCROLL with the selection. Eight nibbles all fit the hex
      // rows, so hex_base never moves, so a label row anchored to it would
      // never draw the last labels of the frame however far the cursor
      // walked - the CRC of a SENT frame was unreachable.
      //
      // And it has to SAY SO. A row that stops after "S1=543" with nothing
      // to mark the edge reads as a decoder that ran out of things to name,
      // which is the one reading it must never invite: the frame was whole,
      // the panel was narrow.
      int budget = DECODE_PANEL_W - 8 - 2 * DECODE_CHAR_W;  // the < > columns
      int limit = hex_base + DECODE_HEX_COLS * 2;

      txt_base = hex_base;

      for (int guard = 0; guard < DECODE_HEX_COLS * 2; guard++)
      {
        int px = 0;
        int i = txt_base;

        // One label per GROUP, not per byte: three nibbles that are one
        // signal get one "S1=543" across them
        while (i < g_logic.count && i < limit)
        {
          char lab[16];
          int gs, gl, w;

          decode_group_at(i, &gs, &gl);
          decode_byte_label(lab, sizeof(lab), gs + gl - 1);
          w = (decode_label_cells(lab, gs + gl - 1) +
              (i > txt_base ? 1 : 0)) * DECODE_CHAR_W;

          if (px + w > budget)
            break;

          px += w;
          i = gs + gl;   // always past i: idx lies inside its own group
        }

        txt_show = i - txt_base;

        if (txt_show < 1)
          txt_show = 1;  // a label wider than the panel still holds its place

        if (g_decode_sel < txt_base + txt_show ||
            txt_base + txt_show >= limit || txt_base + txt_show >= g_logic.count)
          break;

        txt_base++;
      }
    }

    // The header's range describes the hex rows, which are what its numbers
    // are counting. The label row keeps its own markers, because it is a
    // narrower window onto the same bytes and the two edges do not coincide.
    int win_show = txt_show;

    if (!text)
    {
      win_show = g_logic.count - hex_base;

      if (win_show > DECODE_HEX_COLS * 2)
        win_show = DECODE_HEX_COLS * 2;
    }

    bool more_left = text ? (txt_base > 0) : (hex_base > 0);
    bool more_right = text ? (txt_base + win_show < g_logic.count)
        : (hex_base + win_show < g_logic.count);
    // ...and separately, whether the labels under them ran out of width
    bool lab_more_left = !text && (txt_base > 0);
    bool lab_more_right = !text && (txt_base + txt_show < g_logic.count);

    // The header counts what was decoded; when the panel cannot show all of
    // it, it counts what is on the panel out of it instead. "45B" and
    // "12-37/45" are the difference between a complete dump and a view of
    // one, and the panel is a view of one far more often than not.
    if (more_left || more_right)
      snprintf(line, sizeof(line), "%.20s %d-%d/%d", g_logic.info,
          txt_base + 1, txt_base + win_show, g_logic.count);
    else
      snprintf(line, sizeof(line), "%.24s %dB", g_logic.info, g_logic.count);

    lcd_puts(DECODE_PANEL_X + 4, DECODE_PANEL_Y + 4, line);

    for (int row = 0; row < 2; row++)
    {
      for (int i = 0; i < DECODE_HEX_COLS; i++)
      {
        int idx = hex_base + row * DECODE_HEX_COLS + i;

        if (idx >= g_logic.count)
          break;

        // The selected byte, and lit behind it the others that make up the
        // same character. Two bytes for one letter is a thing to be SHOWN;
        // a word in the corner saying "UTF-8" leaves the reader to work out
        // which of the numbers in front of them go together.
        bool grouped = (sel_len > 1 && idx >= sel_start && idx < sel_start + sel_len);

        snprintf(line, sizeof(line), "%02X", g_logic.bytes[idx]);
        lcd_set_color(BG_COLOR, (idx == g_decode_sel) ? DSTRIP_SEL :
            grouped ? DSTRIP_GROUP : LCD_WHITE_COLOR);
        lcd_puts(DECODE_PANEL_X + 4 + i * 18, DECODE_PANEL_Y + 14 + row * 9, line);
      }
    }

    if (text)
    {
      // The text, between the markers that say which way the rest of it lies.
      // The left one holds its column whether or not it is needed, so the
      // text does not shift sideways by a character as the window scrolls.
      //
      // One place per CHARACTER, not per byte: a two-byte UTF-8 character
      // gets a single '?' rather than two dots. The font cannot draw it - it
      // is 6x8 ASCII - but the count is the point. Four dots where two
      // characters were sent reads as a decode that went wrong; two
      // placeholders read as text this screen cannot show, which is what it
      // is.
      char *p = line;

      *p++ = more_left ? '<' : ' ';

      for (int i = 0; i < txt_show; )
      {
        uint8_t ch = g_logic.bytes[txt_base + i];
        int len = utf8_len(&g_logic.bytes[txt_base + i], txt_show - i);

        if (len > 1)
        {
          *p++ = '?'; // one character, however many bytes it took
          i += len;
        }
        else
        {
          *p++ = (ch >= 32 && ch < 127) ? (char)ch : '.';
          i++;
        }
      }

      if (more_right)
        *p++ = '>';

      *p = 0;
      lcd_set_color(BG_COLOR, MEASURE_FREQ_COLOR);
      lcd_puts(DECODE_PANEL_X + 4, DECODE_PANEL_Y + 34, line);
    }
    else
    {
      // What the bytes above ARE. The decoder worked this out to decode them
      // at all - which byte of a NEC frame is the command, which byte of a
      // WS2812 pixel is green - and it was only being said on the trace, at
      // zooms where a byte is wide enough to write in. Here it costs a row
      // that was showing four full stops.
      //
      // Written label by label rather than as one string, so the selected one
      // can be lit the way its number is lit above it.
      //
      // And shaded by the parity of the byte it belongs to, because a space
      // between labels cannot be told from a space inside one: "READ ROM" and
      // "ID=123" are single labels, and a row that reads them as two puts
      // every label after them under the wrong number. The row is one flow of
      // text - it cannot line up with the eight fixed hex columns above it -
      // so the shade is what says where one label ends.
      int x = DECODE_PANEL_X + 4;

      // The marker column is held whether or not it is needed, so the labels
      // do not shift sideways by a character as the row scrolls
      lcd_set_color(BG_COLOR, CAPTURE_WAIT_COLOR);
      lcd_puts(x, DECODE_PANEL_Y + 34, lab_more_left ? "<" : " ");
      x += DECODE_CHAR_W;

      for (int i = txt_base; i < txt_base + txt_show; )
      {
        char lab[16];
        int gs, gl;

        decode_group_at(i, &gs, &gl);
        decode_byte_label(lab, sizeof(lab), gs + gl - 1);

        if (i > txt_base)
          x += DECODE_CHAR_W; // the space between labels

        // Lit for the group the cursor is in, whichever of its bytes that is
        bool sel = (g_decode_sel >= gs && g_decode_sel < gs + gl);
        uint16_t swc;

        // The colour itself where the label names one, in the character cell
        // the fit arithmetic already reserved for it
        if (decode_group_color(gs + gl - 1, &swc))
        {
          int sy = DECODE_PANEL_Y + 34 + (8 - DBAND_SW_H) / 2;

          lcd_fill_rect(x, sy, DBAND_SW_W, DBAND_SW_H, swc);
          lcd_draw_rect(x, sy, DBAND_SW_W, DBAND_SW_H, DBAND_SW_EDGE);
          x += DECODE_CHAR_W;
        }

        lcd_set_color(BG_COLOR, sel ? DSTRIP_SEL :
            (gs & 1) ? MPANEL_DIM(MEASURE_FREQ_COLOR) : MEASURE_FREQ_COLOR);
        lcd_puts(x, DECODE_PANEL_Y + 34, lab);
        x += (int)strlen(lab) * DECODE_CHAR_W;

        i = gs + gl;
      }

      if (lab_more_right)
      {
        lcd_set_color(BG_COLOR, CAPTURE_WAIT_COLOR);
        lcd_puts(x + DECODE_CHAR_W, DECODE_PANEL_Y + 34, ">");
      }
    }

    // Bottom row: why what is above may not be the whole story.
    //
    // "hold" - the frames are older than the record on the screen, so a panel
    // that stopped changing reads as "nothing new since" and not as "this is
    // what is on the wire now".
    //
    // "cut" - a record is a window, and a message wider than the window comes
    // out cut. Four bytes of forty look like a broken decoder right up until
    // the record length is on the screen next to them.
    // The row is 29 characters and these compete for it, so they go in in
    // order of what would mislead most if it were left out: frames that are
    // not from this record, then a record that did not hold the message,
    // then a panel that is only a view of what it did hold. The last one has
    // its own < > markers on the text row and can afford to lose the words.
    // Any protocol can be cut off by the end of the record - a NEC frame is
    // 67.5 ms of it - so overrun speaks for all of them. Only the serial ones
    // also report a record that started mid-message, because only they have
    // an idle line to have started in the middle of.
    bool cut = g_logic.overrun ||
        (decode_proto_is_serial(g_logic.proto) && !g_logic.burst_start);
    // Fewer characters than bytes is what "not plain ASCII" actually means,
    // and it is a count rather than a claim: "18 bytes, 12 characters" says
    // the line is mostly ASCII with some of it not, which is what a mixed
    // message is. A bare "UTF-8" says nothing about how much, or where.
    int chars = text ? utf8_char_count(g_logic.bytes, g_logic.count) : g_logic.count;
    bool utf8 = (chars < g_logic.count);

    // The label row runs out of width long before the hex rows run out of
    // columns, and TRIG_UP/DN is the way to the rest of it just the same
    bool more = more_left || more_right || lab_more_left || lab_more_right;

    if (g_decode_held || cut || more || utf8)
    {
      int64_t span_us = ((int64_t)g_decode_size * g_decode_period_ns) / 1000;
      char *q = line;
      size_t left = sizeof(line);

      if (g_decode_held)
        q += snprintf(q, left, "hold ");

      left = sizeof(line) - (size_t)(q - line);

      if (cut)
      {
        q += snprintf(q, left, "cut: rec %d.%02dms", (int)(span_us / 1000),
            (int)(span_us % 1000) / 10);
        left = sizeof(line) - (size_t)(q - line);
      }

      // The encoding goes before the navigation hint: it changes how what is
      // already on the screen should be read, which the hint does not.
      // Sitting on a multi-byte character it says which one - the same
      // U+xxxx the band writes on the trace, so the two views agree instead
      // of one being specific and the other waving at the topic.
      if (sel_len > 1 && (q - line) + 11 <= 29)
      {
        q += snprintf(q, left, "%sU+%04X %dB", (q == line) ? "" : " ",
            (unsigned)utf8_code_point(&g_logic.bytes[sel_start], sel_len), sel_len);
        left = sizeof(line) - (size_t)(q - line);
      }
      else if (utf8 && (q - line) + 15 <= 29)
      {
        q += snprintf(q, left, "%sUTF-8 %dch/%dB", (q == line) ? "" : " ",
            chars, g_logic.count);
        left = sizeof(line) - (size_t)(q - line);
      }

      // 29 characters at 6 px each is the panel's width
      if (more && (q - line) + 17 <= 29)
        snprintf(q, left, "%smore: TRIG_UP/DN", (q == line) ? "" : " ");

      lcd_set_color(BG_COLOR, CAPTURE_WAIT_COLOR);
      lcd_puts(DECODE_PANEL_X + 4, DECODE_PANEL_Y + 43, line);
    }
  }

  lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
// SHIFT+50P: find the narrowest pulse anywhere in the record and pan to it.
//
// The screen shows 300 of 98304 samples; a runt that fires once per record
// is practically invisible on the live trace and an edge trigger never
// singles it out. Scanning the whole record after the fact does. Runs on the
// STOPPED record - on a live one the glitch would be overwritten before the
// pan lands on it - so it stops acquisition first, exactly like STOP.
static void find_glitch(void)
{
  Measure m;
  const uint8_t *data;
  int size, offset, period_ns, trigger_timepos;
  int width = 0, count = 0, pos, mid, hyst;
  int64_t t_ns;
  char msg[48], wtxt[12];
  bool high = false;

  if (capture_get_state() != CAPTURE_STATE_STOP)
  {
    capture_stop();
    draw_capture_state();
  }

  if (!capture_get_raw_measure_fresh(&m) || !capture_get_record(&data, &size,
      &offset, &period_ns, &trigger_timepos))
    return;

  if ((m.pk_hi - m.pk_lo) < 6)
  {
    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, "No signal to search");
    return;
  }

  // The frequency counter's own Schmitt pair, so "a pulse" means the same
  // thing here as it does there
  mid = (m.pk_lo + m.pk_hi) / 2;
  hyst = (m.pk_hi - m.pk_lo) * 20 / 100;

  if (hyst < 2)
    hyst = 2;

  pos = measure_find_min_pulse(data, size, offset, mid - hyst, mid + hyst,
      &width, &high, &count);

  if (pos < 0)
  {
    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, "No pulses in the record");
    return;
  }

  // Pan the view onto the pulse: its center in trigger-relative time is
  // exactly what horizontal_position means
  t_ns = (int64_t)(pos - trigger_timepos) * period_ns;

  if (t_ns < MIN_HORIZONTAL_POSITION)
    t_ns = MIN_HORIZONTAL_POSITION;
  else if (t_ns > MAX_HORIZONTAL_POSITION)
    t_ns = MAX_HORIZONTAL_POSITION;

  config.horizontal_position = t_ns;
  config.horizontal_position_px =
      (int)(config.horizontal_position / hs_px_value[config.horizontal_scale]);

  draw_horizontal_position();
  update_sample_rate();
  refresh_view();

  {
    int64_t w_ps = (int64_t)width * period_ns * 1000;

    format_ps((w_ps > 0x7fffffff) ? 0x7fffffff : (int)w_ps, wtxt, sizeof(wtxt));
  }

  // The tie count is the verdict: "x1" is a genuine outlier worth staring
  // at, "x2481" is the signal's own half-periods quantized by the sample
  // clock - nothing anomalous in the record at all
  if (count > 1)
    snprintf(msg, sizeof(msg), "Min %s pulse: %s x%d (STOP resumes)",
        high ? "high" : "low", wtxt, count);
  else
    snprintf(msg, sizeof(msg), "Min %s pulse: %s (STOP resumes)",
        high ? "high" : "low", wtxt);

  toast_show();
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
}

//-----------------------------------------------------------------------------
// Put the trigger level at the mid point between the measured signal
// extremes (the classic 50% button). Draw-free public form for the system
// menu; the scope's own button path wraps it with the marker redraw.
void scope_trigger_50_percent(void)
{
  ScopeMeasure sm;

  if (!capture_get_measurements_fresh(&sm))
    return;

  config.trigger_level = sm.vmid_mv / vs_mv_px(config.vertical_scale);

  scope_apply_trigger_level();
}

//-----------------------------------------------------------------------------
static void trigger_set_50_percent(void)
{
  scope_trigger_50_percent();
  draw_trigger_level();
}

//-----------------------------------------------------------------------------
// Enter the decoder view: drop every conclusion the last session reached, put
// the trigger where a logic signal wants it, and set the scope up from the
// rate when the rate is already known.
//
// Told BOTH the protocol and the rate, there is nothing left to work out from
// a record, so there is no reason to look at a random one: the window, the
// trigger and the hunt for a message start can all be set before the first
// record arrives. That is the difference between reading a message and first
// having to catch one.
static void decode_mode_enter(void)
{
  g_decode_mode = true;
  g_fft_mode = false;
  g_trend_mode = false;
  g_logic_have = false;
  g_decode_held = false;
  g_uart_locked = false;
  g_decode_fitted = false;
  g_decode_hunt = false;
  g_decode_sel = 0;
  g_decode_force = true; // show something without waiting for the throttle
  // Before anything re-times the acquisition: the window and the trigger set
  // up below are a record's, and roll pins the rate to something else
  roll_sync();
  trigger_set_50_percent();

  if (decoder_baud_value() > 0 && config.decoder_fit_mode == 0 &&
      (proto_t)config.decoder_proto == PROTO_UART)
  {
    decode_arm_hunt(); // window, trigger and the hunt in one
  }
  else if (decoder_baud_value() > 0)
  {
    g_decode_fitted = true;
    decode_fit_window(decoder_baud_value());
  }

  decode_update();
  g_decode_panel_pending = true;
  g_shadow_valid = false;
  g_sweep_force = true;
  update_display();
}

//-----------------------------------------------------------------------------
// Auto-setup.
//
// Amplitude and frequency want opposite records. A frequency needs several
// periods inside the record; an amplitude only needs the envelope, which
// undersampling leaves intact. So the run is split into phases, each on the
// record that suits what it measures:
//
//   SURVEY - park on a ~100 ms record: mains hum and up shows several
//            periods there, and the envelope of a signal far above that
//            sample rate is still correct
//   VERT   - converge the vertical on that record: step out of clipping,
//            center the DC, then jump straight to the scale the measured
//            voltage asks for
//   FAST   - the maximum sample rate answers "is there something far too
//            fast for the survey record?", by crossings first and by the
//            spectrum of the raw ring second
//   REFINE - park on the timebase the estimate asks for and re-measure. An
//            aliased estimate cannot survive this: the new record samples
//            the signal thousands of times per period.
//
// Doing the vertical on a record that held a fraction of one period was the
// old bug: 50 Hz looked like a few counts of drift, the loop zoomed into the
// millivolt ranges and locked onto the noise floor.
//-----------------------------------------------------------------------------
// Spectral frequency estimate from the full-rate ring: crossing counting
// dies at ~10-20 MHz on the decimated record, the FFT peak survives up to
// the Nyquist edge of the raw ring (~62 MHz)
static int autoset_fft_probe(void)
{
  int period_ns = 0;

  if (capture_read_fast_samples(g_fft_samples, FFT_SIZE, &period_ns) < FFT_SIZE)
    return 0;

  if (period_ns <= 0)
    return 0;

  fft_spectrum(g_fft_samples, FFT_SIZE, 0, g_fft_mag);

  return fft_peak_frequency(g_fft_mag, period_ns);
}

//-----------------------------------------------------------------------------
void autoset_set_horizontal(int hs)
{
  if (hs < 0)
    hs = 0;
  else if (hs > HS_LAST)
    hs = HS_LAST;

  config.horizontal_scale = hs;
  config.horizontal_position = 0;
  config.horizontal_position_px = 0;
  config.horizontal_period = hs_px_value[hs];

  draw_horizontal_scale();
  draw_horizontal_position();
  update_sample_rate();
}

//-----------------------------------------------------------------------------
// Timebase that puts periods_x10/10 whole periods across the screen. Sweep
// timebases only - the hunt found a signal by measuring records, and landing
// it in a view that draws none would throw that away.
static void autoset_set_timebase(int freq, int periods_x10)
{
  int64_t need = (100000000ll * periods_x10) / freq / GRID_DIVS_H;
  int hs = 0;

  while (hs < HS_SWEEP_LAST && hs_div_value[hs] < need)
    hs++;

  autoset_set_horizontal(hs);
}

//-----------------------------------------------------------------------------
// A frequency is only believable when the record held several similar
// periods: noise crosses the mid level all the time and a bare "frequency"
// reading cannot tell the two apart
static bool autoset_freq_ok(const ScopeMeasure *sm)
{
  if (sm->frequency <= 0 || sm->periods < AUTOSET_MIN_PERIODS)
    return false;

  // Regular repetition is the strong case: a waveform generator, a clock,
  // mains hum.
  if (sm->period_good_pct >= AUTOSET_MIN_GOOD_PCT)
    return true;

  // But most things worth probing do NOT repeat evenly — UART frames, bus
  // traffic, an SWD line, a burst of packets. Their intervals are as
  // irregular as noise, and demanding regularity made the auto-setup walk
  // straight past them. What noise cannot fake is sitting AT two levels:
  // a logic line does it ~90% of the time, noise a few %. So an irregular
  // record still counts when it is unmistakably digital, and the median
  // interval is then a rough but usable timebase for it.
  return sm->level_pct >= AUTOSET_MIN_LEVEL_PCT;
}

//-----------------------------------------------------------------------------
// Position pixels that cancel a DC error of mid_err ADC counts. The offset
// DAC is calibrated so one position pixel moves the trace exactly one pixel.
static int autoset_center_step(int mid_err)
{
  int64_t mv = (int64_t)mid_err * config.calib_vs_mult[config.vertical_scale] /
      CALIB_MULTIPLIER;
  int step = (int)(mv / vs_mv_px(config.vertical_scale));

  if (step == 0)
    step = (mid_err > 0) ? 1 : -1;

  return step;
}

//-----------------------------------------------------------------------------
// The most sensitive scale that still keeps the whole trace on screen.
// Deliberately in PIXELS, not ADC counts: the 50 and 100 mV ranges share one
// analog gain and differ only in display zoom, so a counts-based target
// silently refuses to zoom at the sensitive end.
static int autoset_target_scale(int vamp_mv)
{
  int s;

  for (s = 0; s < VS_LAST; s++)
  {
    if (vamp_mv / vs_mv_px(s) <= AUTOSET_TARGET_PX)
      break;
  }

  if (s < g_autoset_zoom_floor)
    s = g_autoset_zoom_floor;

  return s;
}

//-----------------------------------------------------------------------------
void autoset_set_scale(int scale)
{
  change_vertical_scale(scale - config.vertical_scale); // clamps and redraws
}

//-----------------------------------------------------------------------------
// One vertical convergence step. Returns true when there is nothing left
// worth doing; every other path consumes the frame through update_display().
static bool autoset_vertical(const ScopeMeasure *sm)
{
  bool clip_hi = (sm->top_raw >= 254);
  bool clip_lo = (sm->base_raw <= 1);
  bool center_blocked = false;
  int mid_err = (sm->top_raw + sm->base_raw) / 2 - ZERO_POINT;
  int abs_err = (mid_err < 0) ? -mid_err : mid_err;
  int target;

  if (g_autoset_vert_tries >= AUTOSET_VERT_TRIES)
    return true; // budget spent: show what we have rather than hunt forever

  // 1) Off-center DC. Clipping on ONE rail is an offset problem, not a gain
  //    problem, so this runs before any sensitivity is given away.
  if (abs_err > AUTOSET_CENTER_TOL && !(clip_hi && clip_lo))
  {
    int abs_prev = (g_autoset_cent_prev < 0) ? -g_autoset_cent_prev :
        g_autoset_cent_prev;
    int want;

    // The step assumes the offset DAC pushes the reading up; if the error
    // grew instead, this analog chain works the other way round
    if (g_autoset_cent_prev != AUTOSET_NO_ERR && abs_err > abs_prev + 2)
      g_autoset_cent_dir = -g_autoset_cent_dir;

    want = config.vertical_position -
        g_autoset_cent_dir * autoset_center_step(mid_err);

    // Past the end of the offset range the DAC has nothing left to give at
    // this gain (a few volts of DC at 50 mV/div): the fix is coarser, not
    // more clamped steps that burn the whole budget without moving
    if (want >= MIN_VERTICAL_POSITION && want <= MAX_VERTICAL_POSITION)
    {
      config.vertical_position = want;
      config.vertical_position_mv =
          config.vertical_position * vs_mv_px(config.vertical_scale);

      g_autoset_cent_prev = mid_err;
      g_autoset_vert_tries++;

      capture_set_vertical_parameters();
      draw_vertical_position(false);
      update_display();
      return false;
    }

    center_blocked = true;
  }

  // 2) Too much gain: both rails pinned, or the offset DAC out of range.
  //    Never zoom back into a scale that clipped during this run.
  if ((clip_hi || clip_lo || center_blocked) && config.vertical_scale < VS_LAST)
  {
    g_autoset_zoom_floor = config.vertical_scale + 1;
    g_autoset_cent_prev = AUTOSET_NO_ERR;
    g_autoset_vert_tries++;
    change_vertical_scale(1);
    return false;
  }

  if (clip_hi || clip_lo)
    return true; // pinned at the coarsest scale: nothing else to try

  // 3) The swing is a voltage, so the scale that displays it well can be
  //    computed outright instead of felt out one step per acquisition
  target = autoset_target_scale(sm->vamp_mv);

  if (target != config.vertical_scale)
  {
    // A swing that lands right on the boundary between two ranges maps back
    // to the one we just left; settle for the current one instead of
    // alternating until the budget runs out
    if (target == g_autoset_prev_scale)
      return true;

    g_autoset_prev_scale = config.vertical_scale;
    g_autoset_cent_prev = AUTOSET_NO_ERR; // a gain change moves the DC too
    g_autoset_vert_tries++;
    autoset_set_scale(target);
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
static void autoset_end(void)
{
  g_autoset_active = false;
  g_autoset_timer = TIMER_DISABLE;

  // An abort restored a timebase that may have been a roll one
  roll_sync();

  // The hunt rewrote the scale, the position, the timebase and the trigger
  // while a toast was up, and a toast suppresses every status-line element.
  // The measurements panel is the one part that is not toast-gated, so put
  // it back straight away; the rest follows when the toast expires.
  mpanel_invalidate();
}

//-----------------------------------------------------------------------------
static void autoset_finish(int freq)
{
  char str[32];

  if (freq > 0)
    autoset_set_timebase(freq, AUTOSET_VIEW_PERIODS);

  trigger_set_50_percent();
  draw_vertical_position(false);
  update_display();
  autoset_end();

  // A toast owns the whole status line and every element on it returns early
  // while one is up — including the measurements. Announcing a frequency the
  // measurements are about to show anyway is not worth blanking them for a
  // second and a half, so the result is only announced when there is nothing
  // else down there.
  if (config.measure_display)
    return;

  toast_show();

  if (freq >= 1000000)
    snprintf(str, sizeof(str), "Auto-setup: %d.%02d MHz", freq / 1000000,
        freq % 1000000 / 10000);
  else if (freq >= 1000)
    snprintf(str, sizeof(str), "Auto-setup: %d.%02d kHz", freq / 1000,
        freq % 1000 / 10);
  else
    snprintf(str, sizeof(str), "Auto-setup: %d Hz", freq);

  lcd_puts(GRID_LEFT, STATUS_LINE_Y, str);
}

//-----------------------------------------------------------------------------
// Nothing periodic at any sample rate. Leaving the scope parked on whatever
// the noise floor looked like is worse than useless, so the user's own
// settings come back.
static void autoset_abort(void)
{
  bool bottomed = (config.vertical_scale == 0) &&
      (g_autoset_vamp_mv < AUTOSET_MIN_SIGNAL_MV);

  autoset_set_horizontal(g_autoset_saved_hs);

  // A DC level or a one-shot is still worth showing at the scale we found;
  // a scale that bottomed out on a few counts of noise is not
  if (bottomed)
  {
    config.vertical_position = g_autoset_saved_vpos;
    config.vertical_scale = g_autoset_saved_vs;
    config.vertical_mult = config.calib_vs_mult[config.vertical_scale] * config_probe_mult();
    config.vertical_position_mv =
        config.vertical_position * vs_mv_px(config.vertical_scale);
    config.trigger_level_mv =
        config.trigger_level * vs_mv_px(config.vertical_scale);

    capture_set_vertical_parameters();
    capture_set_trigger_level(config.trigger_level_mv);
    draw_vertical_scale();
    draw_vertical_position(false);
    draw_trigger_level();
  }

  update_display();
  autoset_end();

  toast_show();
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Auto-setup: no signal found");
}

//-----------------------------------------------------------------------------
// One auto-setup iteration, run on each fresh acquisition while active.
// Every path consumes the frame (update_display() directly or through
// change_*) so the next acquisition keeps flowing.
static void autoset_step(void)
{
  ScopeMeasure sm;

  // Fresh (unthrottled) measurements: this is a feedback loop, and steering
  // on a reading cached from before the previous step's own change is what
  // made it oscillate between vertical scales
  if (!capture_get_measurements_fresh(&sm))
  {
    update_display();
    return;
  }

  g_autoset_vamp_mv = sm.vamp_mv;

  switch (g_autoset_phase)
  {
    case AS_SURVEY:
      // ~100 ms of record: >= 5 periods of mains hum, and the envelope of
      // anything faster is still intact here
      if (config.horizontal_scale != AUTOSET_SURVEY_HS)
      {
        autoset_set_horizontal(AUTOSET_SURVEY_HS);
        update_display();
        return;
      }

      g_autoset_phase = AS_VERT;
      /* fall through: the record in hand is already a survey record */

    case AS_VERT:
      if (!autoset_vertical(&sm))
        return;

      // The long record is the honest witness: it holds many periods, its
      // crossings are gated by 20% of the DOMINANT swing (so small fast
      // junk riding on a big slow signal cannot claim the answer), and its
      // sample rate still resolves everything below ~100 kHz.
      if (autoset_freq_ok(&sm))
      {
        g_autoset_freq = sm.frequency;
        g_autoset_phase = AS_REFINE;
        autoset_set_timebase(g_autoset_freq, AUTOSET_VIEW_PERIODS);
        update_display();
        return;
      }

      // Nothing regular in 100 ms: whatever is out there is too fast for
      // this record, so go and ask the maximum sample rate
      g_autoset_phase = AS_FAST;
      autoset_set_horizontal(AUTOSET_FAST_HS);
      update_display();
      return;

    case AS_FAST:
    {
      // Maximum sample rate. A slow signal is a fraction of one period here,
      // so only a fast result counts: anything under the floor would have
      // been seen by the survey record, and was not, so it is junk.
      int freq = 0;

      if (autoset_freq_ok(&sm) && sm.frequency >= AUTOSET_FAST_MIN_HZ)
        freq = sm.frequency;

      if (freq == 0)
      {
        // Crossings die once a period is only a couple of samples wide; the
        // spectrum of the raw ring survives to the Nyquist edge
        int spectral = autoset_fft_probe();

        if (spectral >= AUTOSET_FAST_MIN_HZ)
          freq = spectral;
      }

      if (freq == 0)
      {
        // Not fast, and the survey record saw nothing either: it may simply
        // be slower than 100 ms of record can resolve
        g_autoset_phase = AS_SLOW;
        autoset_set_horizontal(AUTOSET_SLOW_HS);
        update_display();
        return;
      }

      g_autoset_freq = freq;
      g_autoset_phase = AS_REFINE;
      autoset_set_timebase(freq, AUTOSET_VIEW_PERIODS);
      update_display();
      return;
    }

    case AS_SLOW:
      // Eight times the record: the last chance for anything below the
      // ~30 Hz the survey record can resolve
      if (autoset_freq_ok(&sm))
      {
        g_autoset_freq = sm.frequency;
        g_autoset_phase = AS_REFINE;
        autoset_set_timebase(g_autoset_freq, AUTOSET_VIEW_PERIODS);
        update_display();
        return;
      }

      autoset_abort();
      return;

    case AS_REFINE:
    default:
    {
      // The timebase now matches the estimate, so this record samples the
      // signal thousands of times per period. If the estimate came from an
      // aliased record, the disagreement shows up right here.
      int freq = sm.frequency;
      int tol = (int)((int64_t)g_autoset_freq * AUTOSET_FREQ_TOL_PCT / 100);

      if (tol < 1)
        tol = 1;

      if (!autoset_freq_ok(&sm))
      {
        // Too few periods left on screen to re-measure: the estimate that
        // picked this timebase stands
        autoset_finish(g_autoset_freq);
        return;
      }

      if (g_autoset_refine_tries >= AUTOSET_REFINE_TRIES ||
          (freq >= g_autoset_freq - tol && freq <= g_autoset_freq + tol))
      {
        autoset_finish(freq);
        return;
      }

      g_autoset_refine_tries++;
      g_autoset_freq = freq;
      autoset_set_timebase(freq, AUTOSET_VIEW_PERIODS);
      update_display();
      return;
    }
  }
}

//-----------------------------------------------------------------------------
static void autoset_start(void)
{
  g_autoset_active = true;
  g_autoset_timer = AUTOSET_TIMEOUT;

  // The hunt steers on records, so it cannot run inside a view that has none.
  // It ends on a swept timebase whatever it finds, and "no signal found" puts
  // the roll timebase - and the roll view with it - back.
  roll_sync();

  g_autoset_phase = AS_SURVEY;
  g_autoset_saved_hs = config.horizontal_scale;
  g_autoset_saved_vs = config.vertical_scale;
  g_autoset_saved_vpos = config.vertical_position;
  g_autoset_zoom_floor = 0;
  g_autoset_prev_scale = -1;
  g_autoset_cent_dir = 1;
  g_autoset_cent_prev = AUTOSET_NO_ERR;
  g_autoset_vert_tries = 0;
  g_autoset_refine_tries = 0;
  g_autoset_freq = 0;
  g_autoset_vamp_mv = 0;

  // AUTO trigger mode guarantees a steady stream of frames to iterate on
  if (config.trigger_mode != TRIGGER_MODE_AUTO)
  {
    config.trigger_mode = TRIGGER_MODE_AUTO;
    capture_set_trigger_mode(config.trigger_mode);
    draw_trigger_mode();
  }

  capture_start();

  toast_show();
  lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Auto-setup...");
}

//-----------------------------------------------------------------------------
// Auto-calibration against an OPEN input.
//
// Three of the four calibration parameters are derivable with nothing
// connected, because each one is a statement about the instrument rather than
// about a signal:
//
//   Z (calib_dac_zero)     the resting level has to read ZERO_POINT
//   D (calib_channel_delta) the two converters have to agree on that level
//   O (calib_dac_mult)     one position pixel has to move the trace one pixel
//
// The fourth, S (calib_vs_mult), is a volts-per-count claim and cannot be
// checked without a reference amplitude - there is no calibrator output on
// this board (doc/Hardware.md), so it stays manual.
//
// Every phase changes hardware and then has to wait for the change to reach a
// record: capture_get_*_fresh() bypasses the display throttle but still hands
// back whatever acquisition last completed, which for the first frames after
// a DAC or attenuator change is the state from before it.
//-----------------------------------------------------------------------------
// Set the vertical range from the menu, so calibration values can be edited
// per range without leaving it
void scope_set_vertical_scale(int scale)
{
  change_vertical_scale(scale - config.vertical_scale);
}

//-----------------------------------------------------------------------------
void scope_buttons_handler(int buttons)
{
  bool shift  = (buttons & BTN_SHIFT);
  bool repeat = (buttons & BTN_REPEAT);

  // The AC/DC hold is armed again only once the key is let go - the release
  // arrives as an event with no keys in it, which is exactly what this reads
  if (0 == (buttons & BTN_AC_DC))
    g_acdc_held = false;

  // ...and so does the layout editor, for a simpler reason: every key on this
  // screen means something else than it does over a live trace
  if (g_layout_edit)
  {
    if (!repeat || (buttons & (BTN_LEFT | BTN_RIGHT | BTN_UP | BTN_DOWN)))
      layout_edit_keys(buttons);

    return;
  }

  // While auto-calibration is up it owns the keyboard: it is rewriting the
  // vertical settings underneath, so letting anything else change them
  // mid-run would silently corrupt the values it is deriving.
  if (g_autocal_active)
  {
    // The driver reports a release as an event with no keys in it, and the
    // "MODE closes" message at the end is dismissed by ANY event - so a STOP
    // that ends the run put the message up on the press and took it away again
    // on the release, about sixty milliseconds later. Nothing else here reads
    // a bare event as a keypress, so the guard belongs at the door.
    if (0 == (buttons & ~(BTN_SHIFT | BTN_REPEAT | BTN_SHIFT_TAP)))
      return;

    // The gain pass is the exception, and only at its prompt: the expected
    // level and the range are the two things the user is holding, so those
    // four keys keep their normal meaning and their auto-repeat. Everything
    // else in a run is a one-shot answer to a question that was asked.
    if (ACAL_ASK_REF == g_autocal_phase)
    {
      if (buttons & (BTN_LEFT | BTN_RIGHT))
      {
        gain_ref_step((buttons & BTN_RIGHT) ? 1 : -1, repeat);
        return;
      }

      if (shift && (buttons & (BTN_UP | BTN_DOWN)))
      {
        gain_range_step((buttons & BTN_UP) ? 1 : -1);
        return;
      }
    }

    if (repeat)
      return;

    if (ACAL_DONE == g_autocal_phase)
      autocal_close();
    else if (buttons & BTN_STOP)
    {
      // Stopping at the gain prompt is not a failure: zero, delta and the DAC
      // step are already found and worth keeping
      if (ACAL_ASK_REF == g_autocal_phase)
        autocal_finish(true);
      else
        autocal_finish(false);
    }
    else if (buttons & BTN_MODE)
    {
      if (ACAL_ASK == g_autocal_phase)
        autocal_begin();
      else if (ACAL_ASK_REF == g_autocal_phase)
      {
        g_autocal_phase = ACAL_SCALE;
        g_autocal_timer = AUTOCAL_TIMEOUT;
        g_gain_note_until = 0;
        autocal_mark();
        autocal_say(g_autocal_gain_only ? "Measuring this range" :
            "Step 5 of 5: gain", "Hold the level steady");
      }
    }

    return;
  }

  if (g_fft_mode && !scope_calibration_mode)
  {
    if (fft_buttons(buttons, shift, repeat))
      return;
  }

  if (g_trend_mode && !scope_calibration_mode)
  {
    if (trend_buttons(buttons, shift, repeat))
      return;
  }

  if (g_cursor_sel && !g_fft_mode && !g_decode_mode && !g_trend_mode &&
      !g_roll_active && !scope_calibration_mode)
  {
    if (cursor_buttons(buttons, shift, repeat))
      return;
  }

  if ((buttons & BTN_UP) && (buttons & BTN_DOWN))
  {
    int old_vpos = config.vertical_position;

    config.vertical_position = 0;
    config.vertical_position_mv = 0;

    capture_set_vertical_parameters();
    roll_rescale(vs_mv_px(config.vertical_scale), old_vpos);
    draw_vertical_position(true);
    refresh_view();
  }
  else if (buttons & BTN_UP)
  {
    if (shift)
      change_vertical_scale(1);
    else
      change_vertical_position_ex(1, repeat);
  }
  else if (buttons & BTN_DOWN)
  {
    if (shift)
      change_vertical_scale(-1);
    else
      change_vertical_position_ex(-1, repeat);
  }

  else if ((buttons & BTN_LEFT) && (buttons & BTN_RIGHT))
  {
    if (g_roll_active)
      return; // already zero, and nothing to re-time

    config.horizontal_position = 0;
    config.horizontal_position_px = 0;

    draw_horizontal_position();
    update_sample_rate();
    update_display();
  }
  else if (buttons & BTN_LEFT)
  {
    if (shift)
      change_horizontal_scale(-1);
    else
      change_horizontal_position_ex(1, repeat);
  }
  else if (buttons & BTN_RIGHT)
  {
    if (shift)
      change_horizontal_scale(1);
    else
      change_horizontal_position_ex(-1, repeat);
  }

  else if (buttons & BTN_TRIG)
  {
    if (repeat || scope_calibration_mode)
      return;

    if (config.trigger_mode == TRIGGER_MODE_SINGLE)
      config.trigger_mode = TRIGGER_MODE_AUTO;
    else
      config.trigger_mode++;

    // Set here, applied on the way out of roll: the strip is drawn from the
    // ring and holding the acquisition in AUTO is what keeps the ring turning
    if (!g_roll_active)
    {
      capture_set_trigger_mode(config.trigger_mode);
      capture_start();
    }

    draw_trigger_mode();
  }
  else if (buttons & BTN_EDGE)
  {
    if (repeat || scope_calibration_mode)
      return;

    if (shift)
    {
      // SHIFT+EDGE: toggle the protocol decoder view
      if (!g_decode_mode)
      {
        decode_mode_enter();
      }
      else
      {
        g_decode_mode = false;
        g_decode_hunt = false;
        g_shadow_valid = false;
        g_sweep_force = true;
        roll_sync();
        update_display();
        redraw_miniview();   // restore the decorative wave over the byte map
        decode_band_build(); // and clear the byte marks off the trace
      }

      draw_status_line();
      return;
    }

    if (config.trigger_edge == TRIGGER_EDGE_BOTH)
      config.trigger_edge = TRIGGER_EDGE_RISE;
    else
      config.trigger_edge++;

    capture_set_trigger_edge(config.trigger_edge);
    draw_trigger_edge();
  }
  else if (buttons & BTN_TRIG_UP)
  {
    if (scope_calibration_mode)
      change_calibration_value(1, shift, repeat);
    else if (shift)
      change_sample_rate_limit(1);
    else if (g_decode_mode && g_logic_have && g_logic.count > 0)
    {
      // In decoder view these keys step through the decoded bytes
      if (g_decode_sel + 1 < g_logic.count)
      {
        g_decode_sel++;
        decode_jump_to_selected();
      }
    }
    else
      change_trigger_level(1);
  }
  else if (buttons & BTN_TRIG_DOWN)
  {
    if (scope_calibration_mode)
      change_calibration_value(-1, shift, repeat);
    else if (shift)
      change_sample_rate_limit(-1);
    else if (g_decode_mode && g_logic_have && g_logic.count > 0)
    {
      if (g_decode_sel > 0)
      {
        g_decode_sel--;
        decode_jump_to_selected();
      }
    }
    else
      change_trigger_level(-1);
  }

  else if (buttons & BTN_AC_DC)
  {
    // HELD: the other thing this key is about. Coupling decides whether there
    // is a DC level at all, so the switch for "show it in the bar instead of a
    // peak-to-peak of noise" belongs on the same key rather than four rows deep
    // in a menu - and it only does anything on the side of the key that has a
    // level to show. One toast, because a setting toggled by a long press with
    // no visible result is a setting nobody trusts.
    if (repeat)
    {
      char msg[48];

      if (g_acdc_held)
        return; // one toggle per hold, not one per repeat tick

      g_acdc_held = true;

      // The press that began this hold has already flipped the coupling. Put it
      // back: a hold means the OTHER thing on this key, not both things, and
      // the user who wants the coupling changed has not lifted their finger yet
      // to say so.
      config.ac_coupling = !config.ac_coupling;
      scope_display_settings_changed();
      capture_set_vertical_parameters();
      draw_ac_dc();

      config.vpp_dc_off = !config.vpp_dc_off;

      snprintf(msg, sizeof(msg), "Flat DC in the bar: %s%s",
          config.vpp_dc_off ? "OFF" : "ON",
          config.ac_coupling ? "  (DC coupling only)" : "");
      toast_show();
      lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
      return;
    }

    config.ac_coupling = !config.ac_coupling;

    scope_display_settings_changed(); // the DC level just moved
    capture_set_vertical_parameters();
    draw_ac_dc();
  }

  else if (buttons & BTN_MODE)
  {
    if (repeat)
      return;

    if (scope_calibration_mode)
    {
      // MODE hides the hint band. It comes back on the next parameter -
      // see g_calib_hint.
      g_calib_hint = !g_calib_hint;
      g_calib_hint_param = -1;
      calib_hint_update();
      g_shadow_valid = false;
      g_sweep_force = true;
      return;
    }

    if (shift)
    {
      // SHIFT+MODE: toggle the FFT spectrum view
      g_fft_mode = !g_fft_mode;
      g_decode_mode = false;
      g_trend_mode = false;
      g_shadow_valid = false;
      g_sweep_force = true;
      roll_sync(); // a spectrum has no time axis to roll along
      draw_status_line();

      if (g_fft_mode)
      {
        g_fft_cursor = -1;
        g_fft_band_auto = true;
        g_fft_auto_span = 0;
        g_fft_have = false;
        g_fft_timer = TIMER_DISABLE;
        g_fft_panel_on = true;
        g_fft_panel_pending = true; // draw it even if no record exists yet
        fft_update();
      }
      else
      {
        update_display();
      }
    }
    else
    {
      config.measure_display = !config.measure_display;
      g_measure_timer = config.measure_display ? MEASURE_UPDATE_TIMEOUT : TIMER_DISABLE;
      draw_status_line();
    }
  }

  // The legend on the shift key. A tap of it - no chord, no hold - is the one
  // press that was free to mean 1X/10X, see BTN_SHIFT_TAP.
  else if (buttons & BTN_SHIFT_TAP)
  {
    if (scope_calibration_mode)
      return;   // calibrating through an attenuator is not a calibration

    // Switching a 1x probe on would do nothing at all, and this key's legend
    // promises 10x: an instrument nobody has told about its probe gets the one
    // nearly everybody has, and the menu is where an odd ratio is said.
    if (!config.x10 && 1 == config_probe_ratio())
      config.probe_ratio = PROBE_RATIO_10X;

    config.x10 = !config.x10;
    scope_probe_changed();

    // Said out loud, because every reading on screen just changed by that
    // factor and the tag in the top bar is easy to miss on a key that used to
    // do nothing
    {
      char msg[40];

      snprintf(msg, sizeof(msg), config.x10 ?
          "Probe %s - readings scaled" : "Probe off - readings direct",
          probe_ratio_labels[((unsigned)config.probe_ratio < PROBE_RATIO_COUNT) ?
              config.probe_ratio : 0]);
      toast_show();
      lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
    }
  }

  // Straight to what the two large readouts show, one key each. Which two
  // measurements are worth the big font changes while probing, and reaching it
  // through Menu > Measurements > Status line > Left was four levels of menu
  // for a question asked that often.
  else if (buttons & (BTN_F1 | BTN_F2))
  {
    if (repeat || scope_calibration_mode)
      return;

    scope_menu_open_slot((buttons & BTN_F1) ? 0 : 1);
  }

  else if (buttons & BTN_AUTO)
  {
    if (repeat || g_fft_mode)
      return;

    // Same promise as everywhere else on this key - "work it out yourself" -
    // and in calibration mode the thing to work out is the calibration.
    // Gain (S) is the one parameter it cannot do; the hint says so.
    if (scope_calibration_mode)
    {
      scope_autocal_start();
      return;
    }

    autoset_start();
  }

  else if (buttons & BTN_50P)
  {
    if (repeat || scope_calibration_mode)
      return;

    // Both work on a record: the glitch finder scans one, and a 50% level is
    // a trigger setting for a sweep that is not running here
    if (g_roll_active)
    {
      toast_show();
      lcd_puts(GRID_LEFT, STATUS_LINE_Y, "Not while rolling - zoom in first");
      return;
    }

    if (shift)
      find_glitch();
    else
      trigger_set_50_percent();
  }

  else if (buttons & BTN_SAVE)
  {
    if (repeat || scope_calibration_mode)
      return;

    if (shift)
    {
      trend_toggle();
      return;
    }

    // Cursors live on the waveform view only
    if (g_fft_mode || g_decode_mode || g_trend_mode)
      return;

    if (g_cursor_sel == 0 && g_cursor_t[0] == g_cursor_t[1])
    {
      // Never used yet: park the pairs two divisions around the view centre
      int64_t div_ns = hs_div_value[config.horizontal_scale];
      int div_mv = vs_mv_px(config.vertical_scale) * GRID_DIV_PX;

      g_cursor_t[0] = config.horizontal_position - 2 * div_ns;
      g_cursor_t[1] = config.horizontal_position + 2 * div_ns;
      g_cursor_v[0] = config.vertical_position_mv + 2 * div_mv;
      g_cursor_v[1] = config.vertical_position_mv - 2 * div_mv;
    }

    g_cursor_sel = (g_cursor_sel + 1) % 5;
    g_cursor_dirty = true;
    draw_status_line(); // hands the line to the readout - or back
  }

  else if (buttons & BTN_STOP)
  {
    if (capture_get_state() == CAPTURE_STATE_STOP)
      capture_start();
    else
      capture_stop_view(); // freeze what the display shows, not the live ring
  }
}

//-----------------------------------------------------------------------------
void scope_init(bool calibration_mode)
{
  scope_calibration_mode = calibration_mode;

  config.horizontal_period = hs_px_value[config.horizontal_scale];
  config.vertical_mult = config.calib_vs_mult[config.vertical_scale] * config_probe_mult();

  // How the screen is divided up, before anything is laid out against it
  scope_ui_scale_changed();

  grid_init();
  persist_build_ramp();

  // The screen was just cleared: previous-frame caches are stale
  g_shadow_valid = false;
  g_sweep_force = true;
  g_vpos_marker.valid = false;
  g_trig_marker.valid = false;

  // The stored timebase decides the view, and update_sample_rate() at the
  // bottom of this has to already know which one it is timing the ADC for.
  // Not roll_sync(): nothing is on screen yet for it to compare against.
  g_roll_active = false;
  roll_set_active(roll_wanted());

  // Settle the panel before anything paints. It decides who owns the status
  // line, and it is a static that outlives the application: entering the scope
  // a second time painted that line for the panel state of the previous
  // session. The cells go with it - the readings in them are the old session's,
  // and they would be composited into the first sweep.
  g_mpanel_active = mpanel_wanted();
  g_line_owner = measure_owns_status_line();
  memset(g_mpanel_cell, 0, sizeof(g_mpanel_cell));
  g_mpanel_is_text = false;
  mpanel_invalidate();

  draw_grid_frame();
  draw_vertical_position(false);
  draw_trigger_mode();
  draw_trigger_edge();
  draw_capture_state();
  draw_status_line();
  redraw_trace();

  if (scope_calibration_mode)
  {
    capture_set_trigger_edge(TRIGGER_EDGE_RISE);
    capture_set_trigger_mode(TRIGGER_MODE_AUTO);
    capture_set_trigger_level(0);
  }
  else
  {
    capture_set_trigger_edge(config.trigger_edge);
    // AUTO while rolling whatever the stored mode says - see roll_set_active
    capture_set_trigger_mode(g_roll_active ? TRIGGER_MODE_AUTO : config.trigger_mode);
    capture_set_trigger_level(config.trigger_level_mv);
  }

  timer_add(&g_toast_timer);
  timer_add(&g_state_timer);
  timer_add(&g_measure_timer);
  timer_add(&g_autoset_timer);
  timer_add(&g_autocal_timer);
  timer_add(&g_pan_settle_timer);
  timer_add(&g_fps_timer);
  timer_add(&g_fft_timer);
  timer_add(&g_trend_timer);
  timer_add(&g_mv_timer);
  g_fps_timer = 1000;
  g_trend_timer = TREND_INTERVAL_MS;
  g_mv_timer = MINIVIEW_UPDATE_TIMEOUT;

  g_measure_timer = config.measure_display ? MEASURE_UPDATE_TIMEOUT : TIMER_DISABLE;

  update_sample_rate();
  capture_set_vertical_parameters();
  capture_start();
}

//-----------------------------------------------------------------------------
// Repaint everything the scope owns. Menus and modal dialogs paint over the
// whole screen and clear it on close, so the incremental renderer has to be
// told that what it thinks is on screen is gone.
void scope_redraw_all(void)
{
  // The menu that was up may have been the one that changes the text size, and
  // that decides how the screen is divided before any of it is drawn
  if (g_geom.bar_scale != ((UI_SCALE_LARGE == config.ui_scale) ? 2 : 1))
    scope_ui_scale_changed();

  g_shadow_valid = false;
  g_sweep_force = true;
  g_vpos_marker.valid = false;
  g_trig_marker.valid = false;
  g_state = -1;
  g_toast_active = false;
  g_toast_timer = TIMER_DISABLE;

  lcd_set_font(FONT_LARGE);
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_COLOR);

  // The menu that was up may have moved the roll threshold under the current
  // timebase, and update_sample_rate() below times the ADC for one view or
  // the other
  roll_set_active(roll_wanted());

  draw_grid_frame();
  draw_vertical_position(false);
  draw_trigger_mode();
  draw_trigger_edge();
  draw_capture_state();
  draw_status_line();
  battery_redraw();
  update_sample_rate();
  redraw_trace();

  /*
   * ...and run the sweep out here, instead of leaving it to paint a column a
   * main-loop pass.
   *
   * What this function repaints is a screen something else covered, and the
   * usual something else is a menu - which is on top of the scope, so the
   * scope is not ticked at all while it is up. A trace left to the sweep is
   * then not a trace that arrives a moment later: it is a hole in the display
   * that stays there until the last menu closes. It showed as a black grid
   * behind an open menu every time a submenu was closed.
   *
   * It is the same three hundred columns either way, and the sweep that
   * follows redraws none of them: this leaves the shadows describing exactly
   * what was painted, which is what tells it there is nothing to do.
   */
  g_trace_column = 0;

  while (!trace_ready())
    draw_trace();

  // The editor's backdrop is its own, and this has just cleared the screen and
  // re-armed the sweep - which is exactly the path taken when the menu that
  // opened the editor closes over it
  if (g_layout_edit)
    layout_edit_refresh();

  // The panels are holes in the sweep, so what is under a closed menu comes
  // back only when they are painted - here, for the same reason as the trace.
  // The snapshot tag is normally put back by scope_task() when it sees a sweep
  // finish; the sweep above finished before it could see it start.
  if (g_snap_tag)
    snap_tag_paint();

  if (g_decode_mode)
    draw_decode_panel();

  if (g_fft_mode)
  {
    g_fft_panel_pending = true;

    if (g_fft_panel_on)
      draw_fft_panel();
  }
}

//-----------------------------------------------------------------------------
int scope_get_fps(void)
{
  return g_fps_value;
}

//-----------------------------------------------------------------------------
// Why the measurements panel is (or is not) on screen. Read this straight
// after the panel goes blank: `act` says whether the sweep composites it at
// all, `bld`/`pnt` say whether it is still being rebuilt and repainted, and
// `cell` is how many readings it last composed - zero with `act1` is the
// combination that means "the band is being drawn, and it is empty".
void scope_get_panel_state(char *buf, int size)
{
  snprintf(buf, (size_t)size, "d%d m%d act%d fft%d t%d bld%u pnt%u cell%d%s",
      config.measure_display ? 1 : 0, config.measure_panel_mode,
      g_mpanel_active ? 1 : 0, g_fft_mode ? 1 : 0, g_measure_timer,
      (unsigned)g_mpanel_builds, (unsigned)g_mpanel_paints,
      g_mpanel_cells, g_mpanel_is_text ? " text" : "");
}

//-----------------------------------------------------------------------------
void scope_task(void)
{
  if (trace_ready())
  {
    if (capture_buffer_updated())
    {
      g_fps_counter++;

      // The editor's trace is its own: a mock waveform where update_display()
      // would put whatever is on the probe. Re-laid every frame rather than
      // once, because a sweep that began before the editor opened paints its
      // first columns from the buffer as it was then - which showed as half a
      // screen of real trace beside half a screen of mock. The frame still has
      // to be consumed, or acquisition stalls behind it.
      if (g_layout_edit)
      {
        capture_consume_frame();
        layout_mock_trace();
        g_sweep_force = true;
      }
      else if (g_autocal_active)
        autocal_step();
      else if (scope_calibration_mode)
        draw_calibration_info();
      else if (g_autoset_active)
        autoset_step();
      else if (g_fft_mode)
        fft_tick();
      else if (g_trend_mode || g_roll_active)
        capture_consume_frame(); // keep acquisition flowing under the plot
      else
      {
        update_display();

        if (g_decode_mode)
          decode_update();
      }
    }
  }

  // The strip advances on wall clock, not on records: every pass, so a
  // column is closed within a main-loop pass of the time it is due
  if (g_roll_active)
    roll_task();

  // A cursor moved: repaint the sweep once the current one is done. The
  // overlay is composited into the columns, so the shadow cache is stale.
  if (g_cursor_dirty && trace_ready() && !g_fft_mode && !g_trend_mode)
  {
    g_cursor_dirty = false;
    g_shadow_valid = false;
    update_display();
  }

  // The trend logger ticks whether or not its view is open. A stopped scope
  // logs nothing: its record is frozen and the "measurements" would flatline.
  if (g_trend_timer == 0)
  {
    ScopeMeasure sm;

    g_trend_timer = TREND_INTERVAL_MS;

    if (capture_get_state() != CAPTURE_STATE_STOP && !scope_calibration_mode &&
        !g_autocal_active && capture_get_measurements(&sm))
    {
      trend_push(sm.frequency, sm.vrms_mv, sm.duty_x10);

      if (g_trend_mode)
      {
        trend_view_update();
        trend_readout();
      }
    }
  }

  // A RUN/STOP transition switches the active record (decimated storage vs
  // full-rate capture buffer): the decode and its positions must follow.
  // Only STOP matters here — capture_get_state() also flips WAIT<->TRIG on
  // every single acquisition, and reacting to that ran the whole decoder
  // cascade dozens of times a second with the throttle bypassed.
  //
  // Asked for from the menu, done here: the menu is a screen over the scope
  // and the scope does not draw while it is open
  if (g_spi_clock_request)
  {
    Measure m;
    char msg[48];

    g_spi_clock_request = false;

    // From the RAW measurement, not from the decoder: the probe is on a
    // clock line, a frequency counter is what reads clocks, and the decoder
    // has nothing to say about a signal nobody pointed it at.
    if (capture_get_raw_measure(&m) && m.frequency > 0)
    {
      config.spi_clock_hz = m.frequency;
      snprintf(msg, sizeof(msg), "SPI clock stored: %d Hz", m.frequency);
    }
    else
    {
      snprintf(msg, sizeof(msg), "SPI clock: nothing periodic to measure");
    }

    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
    mpanel_invalidate();
  }

  if (g_swd_clock_request)
  {
    Measure m;
    char msg[48];

    g_swd_clock_request = false;

    if (capture_get_raw_measure(&m) && m.frequency > 0)
    {
      config.swd_clock_hz = m.frequency;
      snprintf(msg, sizeof(msg), "SWD clock stored: %d Hz", m.frequency);
    }
    else
    {
      snprintf(msg, sizeof(msg), "SWD clock: nothing periodic to measure");
    }

    toast_show();
    lcd_puts(GRID_LEFT, STATUS_LINE_Y, msg);
    mpanel_invalidate();
  }

  if (g_decode_hunt_request)
  {
    g_decode_hunt_request = false;

    if (!g_decode_mode)
    {
      decode_mode_enter(); // which arms the hunt itself when it can
      draw_status_line();
    }

    if (!g_decode_hunt)
      decode_arm_hunt();
  }

  // A stop may swap the record under the frozen trace (an AUTO stop
  // publishes the live ring, up to one sweep newer than the drawn frame;
  // see capture_stop_view): repaint from the actual frozen record NOW,
  // so the first pan/zoom does not silently morph the trace into it.
  {
    static int prev_stopped = -1;
    int stopped = (capture_get_state() == CAPTURE_STATE_STOP);

    if (stopped != prev_stopped && trace_ready())
    {
      prev_stopped = stopped;

      if (stopped && !g_fft_mode && !g_trend_mode && !scope_calibration_mode)
      {
        g_shadow_valid = false;
        update_display();
      }
    }
  }

  if (g_decode_mode)
  {
    static int prev_stopped = -1;
    int stopped = (capture_get_state() == CAPTURE_STATE_STOP);

    if (stopped != prev_stopped)
    {
      prev_stopped = stopped;
      g_decode_force = true;
      decode_update();
    }
  }

  // Same for the spectrum: STOP is where the analysis matters most. A
  // stopped acquisition delivers no more frames, so nothing else would ever
  // re-run it — and the record it freezes may have swapped (an AUTO stop
  // publishes the raw ring, four times longer than the storage record the
  // running view analyzes; a NORMAL/SINGLE one keeps the snapshot).
  if (g_fft_mode)
  {
    static int prev_stopped = -1;
    int stopped = (capture_get_state() == CAPTURE_STATE_STOP);

    if (stopped != prev_stopped)
    {
      prev_stopped = stopped;
      g_fft_timer = TIMER_DISABLE;
      fft_update();
    }
  }

  // The snapshot tag's lifecycle. On/off transitions cover stop/start from
  // every path (the button, the decode hunt, the glitch finder); the
  // sweep-completion repaint self-heals after anything that repainted the
  // whole grid over it (a view switch, the menu closing) - any such repaint
  // runs a sweep, and the sweep's hole spares the tag columns from then on.
  {
    static bool was_ready = true;
    bool ready = trace_ready();
    bool want = capture_stopped_on_snapshot() && !g_fft_mode &&
        !g_trend_mode && !g_roll_active && !scope_calibration_mode;

    if (want != g_snap_tag)
    {
      g_snap_tag = want;

      if (want)
        snap_tag_paint();
      else
        overlay_repaint_region(0, SNAP_TAG_H); // put the trace back
    }
    else if (want && ready && !was_ready)
      snap_tag_paint();

    was_ready = ready;
  }

  if (g_autoset_active && g_autoset_timer == 0)
    autoset_abort(); // ran out of time: put the user's settings back

  if (g_autocal_active && g_autocal_timer == 0)
    autocal_finish(false); // same, and say so rather than claim success

  if (g_fps_timer == 0)
  {
    g_fps_timer = 1000;
    g_fps_value = g_fps_counter;
    g_fps_counter = 0;
  }

  // A held pan key has been released: apply the coalesced heavy updates
  if (g_pan_settle_timer == 0)
  {
    g_pan_settle_timer = TIMER_DISABLE;
    g_pan_repeat_count = 0;
    capture_set_vertical_parameters();
    update_sample_rate();
    refresh_view();
  }

  draw_trace();

  // Sweeps no longer touch the panel area, so a redraw is only needed when
  // the decoded content itself changed — and can happen mid-sweep
  if (g_decode_mode && g_decode_panel_pending)
    draw_decode_panel();

  if (g_fft_mode && g_fft_panel_on && g_fft_panel_pending)
    draw_fft_panel();

  // A filling record delivers no frames while it fills, so fft_tick() - which
  // is what normally repaints the panel - does not run at all, and the
  // progress line would sit frozen at whatever it read when the timebase
  // moved. Tick that ONE row here instead, off the wall clock rather than off
  // frames, and leave the rest of the panel where it is.
  if (g_fft_mode && g_fft_panel_on && !g_fft_panel_pending && fft_fill_pending())
  {
    static uint32_t last_ms = 0;

    if ((timer_ms() - last_ms) >= FFT_FILL_TICK_MS)
    {
      last_ms = timer_ms();
      lcd_set_font(FONT_SMALL);
      fft_fill_row_draw();
      lcd_set_font(FONT_LARGE);
    }
  }

  if (CAPTURE_STATE_WAIT == capture_get_state())
  {
    if (g_state_timer == TIMER_DISABLE)
    {
      g_state_timer = WAIT_STATE_HOLDOFF;
    }
    else if (g_state_timer == 0)
    {
      g_state_timer = TIMER_DISABLE;
      draw_capture_state();
    }
  }
  else
  {
    g_state_timer = TIMER_DISABLE;
    draw_capture_state();
  }

  if (g_toast_active)
  {
    if (g_toast_timer == 0)
    {
      g_toast_timer = TIMER_DISABLE;
      g_toast_active = false;

      // Every status-line element returns early while a toast is up, so the
      // bottom of the screen has been frozen for as long as it was shown:
      // rebuild all of it, panel included
      draw_status_line();
      mpanel_invalidate();
    }
  }

  mpanel_set_active(mpanel_wanted());

  // The view, the metric selection and the spectrum key can all hand the
  // right-hand end of the status line from the trigger readouts to the
  // measurements and back. Neither side repaints on its own, so watch the
  // ownership itself: whoever holds it now has to paint it, and the loser's
  // glyphs have to go. A toast owns the whole line while it is up and
  // rebuilds it from scratch when it expires.
  if (measure_owns_status_line() != g_line_owner)
  {
    g_line_owner = measure_owns_status_line();

    if (!g_toast_active)
      draw_status_line();
  }

  // Put the current record in the record map. Only when the acquisition has
  // actually moved on: the strip costs a 24 KB scan and four blits, and at a
  // fast timebase this timer would otherwise pay both to redraw the same
  // picture. MV_Y < 0 is the Large layout, where there is no strip at all.
  if (g_mv_timer == 0)
  {
    g_mv_timer = MINIVIEW_UPDATE_TIMEOUT;

    // Rolling reads the on-screen strip rather than a record, and that strip
    // advances on the fold's schedule with nothing in the acquisition
    // generation to mark it: there the tick itself is the trigger to repaint
    if (MV_Y >= 0 && (g_roll_active || capture_get_generation() != g_mv_env_gen))
    {
      g_mv_env_valid = false;
      redraw_miniview();
    }
  }

  // measure_display can also be flipped from the system menu, which does not
  // know about our timer: keep the timer state in sync here
  if (config.measure_display && TIMER_DISABLE == g_measure_timer)
    g_measure_timer = 0;
  else if (!config.measure_display && TIMER_DISABLE != g_measure_timer)
    g_measure_timer = TIMER_DISABLE;

  // In calibration mode the panel band carries the hint instead, so the
  // measurement builder must not overwrite it
  if (config.measure_display && !g_fft_mode && !scope_calibration_mode &&
      !g_autocal_active)
  {
    if (g_measure_timer == 0)
    {
      g_measure_timer = MEASURE_UPDATE_TIMEOUT;

      // The two views are independent and can both be on: the panel lists a
      // set of metrics compactly, the status line puts two of them in the
      // large font. Repeating a value between them is the user's call.
      if (g_mpanel_active)
        mpanel_update();

      // ...but not over the editor's hint line, which is the only thing on
      // screen saying what the keys do there
      if (measure_owns_status_line() && !g_toast_active && !g_layout_edit)
        draw_measure();
    }
  }
}

