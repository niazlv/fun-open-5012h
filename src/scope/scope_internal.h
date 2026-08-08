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

#ifndef _SCOPE_INTERNAL_H_
#define _SCOPE_INTERNAL_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"
#include "common.h"
#include "config.h"
#include "images.h"
#include "logic_decode.h"

/*- Definitions -------------------------------------------------------------*/
#define ZERO_POINT             0x80

/*
 * THE SCREEN, and the one number in it that may not move.
 *
 * 240 rows, and they are spent three ways: the top bar, the graticule, the
 * status line. Horizontally nothing changes - 12 divisions of 25 px, the full
 * width - and neither does the division PITCH, ever: 25 px is what makes
 * vs_px_value[] read 50 mV, 100 mV, 200 mV ... per division, so a grid with a
 * different pitch would silently relabel every vertical range the instrument
 * has. What can change is HOW MANY divisions there are down the screen, and
 * that is the whole of what a bigger interface costs.
 *
 *   Normal   top bar 20, grid 8 x 25 = 200, status line 20.  8x16 in the bars.
 *   Large    top bar 41, grid 6 x 25 = 150, status line 48. 16x32 in the bars,
 *            over two rows, and two divisions fewer to put the trace in.
 *
 * That is a quarter of the vertical RANGE - not of the resolution, which is
 * still 25 px to a division at every setting, so a division is worth what its
 * label says and a measurement is what it always was. It buys the readings and
 * the settings at four times the area, which on a 2.4" panel is the difference
 * between reading the instrument and leaning into it.
 *
 * These are runtime values behind their old names, so that the ~130 places that
 * lay something out against the grid did not each have to be found and changed;
 * what the compiler DID find is every place that needed a constant, which is
 * the array sizes below and one static initialiser. Those use the _MAX forms.
 */
#define GRID_CENTER_X          160
#define GRID_LEFT              10
#define GRID_RIGHT             310
#define GRID_WIDTH             300
#define GRID_DIV_PX            25
#define GRID_DIVS_H            12

#define GRID_HEIGHT_MAX        200   // buffers are sized for the biggest grid
#define GRID_TOP               (g_geom.grid_top)
#define GRID_HEIGHT            (g_geom.grid_h)
#define GRID_BOTTOM            (g_geom.grid_top + g_geom.grid_h)
#define GRID_CENTER_Y          (g_geom.grid_top + g_geom.grid_h / 2)
#define GRID_DIVS_V            (g_geom.grid_h / GRID_DIV_PX)

// The bottom bar: where it starts and how tall it is. At the large size it is
// two rows - the readings at 16x32 above, the scale settings at 8x16 below -
// and STATUS_LINE_Y is the first of them, which is what every other owner of
// this line (toasts, cursors, the trend readout, the editor's key hints) draws
// its own first row at.
#define STATUS_LINE_Y          (g_geom.status_y)
#define STATUS_LINE_HEIGHT     (g_geom.status_h)

// Inside a reading: how far below the digits' top the name and the unit sit, so
// that they line up with the figures' baseline rather than their tops. Zero at
// the normal size, where a reading is one row of one font.
#define STATUS_ROW2_Y_OFS      (g_geom.status_row2)

typedef struct
{
  int16_t grid_top;
  int16_t grid_h;
  int16_t status_y;      // the readings' row: the digits' own origin
  int16_t status_h;      // ...and the whole bottom bar, which is what is cleared
  int16_t status_row2;   // offset of the second row, 0 when there is not one
  int16_t slot0_x;       // the two readings, and how much room each one has
  int16_t slot1_x;
  int16_t slot_w;
  int16_t vscale_x;      // volts per division, and the probe ratio beside it
  int16_t vscale_y;
  int16_t acdc_x;        // the coupling glyph
  int16_t acdc_y;
  int16_t hscale_x;      // time per division
  int16_t hscale_y;
  int16_t mode_x;        // AUTO / NORM / SNGL
  int16_t mode_y;
  int16_t state_x;       // STOP / WAIT / TRIG / ROLL
  int16_t state_y;
  int16_t mv_y;          // the record map, or -1 where there is no room for it
  int8_t  bar_scale;     // glyph scale for the readings: 1, or 2 at Large
} ScopeGeom;

// Glyph scale for the readings. One place, because every field on the bar has
// to agree about how tall the row is.
#define BAR_SCALE              (g_geom.bar_scale)
#define MV_Y                   (g_geom.mv_y)

#define MINIVIEW_WIDTH         160
#define MINIVIEW_UPDATE_TIMEOUT 100  // envelope rebuild cap, ms
#define MV_ENV_NONE            0xff  // no envelope byte for this column

// Decoded bytes are marked in the miniview (the whole-record map at the top
// of the screen): alternating tints per byte, bright for the selected one
#define DSTRIP_EVEN    LCD_COLOR(0, 150, 200)
#define DSTRIP_ODD     LCD_COLOR(0, 90, 130)
#define DSTRIP_SEL     LCD_COLOR(255, 255, 0)
// The other bytes of the character the selected byte belongs to. Lit, but not
// as the selection: "this one, and these came with it".
#define DSTRIP_GROUP   LCD_COLOR(150, 150, 0)
#define FORMAT_PS_W            5     // format_ps() field width

// Top bar, right of the record map and left of the sample rates. Nine free
// columns, 243..251: the map's own clear reaches 242 (its trigger marker
// overhangs the frame by two), and the rates start at 252. The icon is 8x16
// with a blank first column, so its ink lands 244..250.
//
// The rates cannot move over to give it more room. They are 27 px wide (three
// digits, a half space and the unit) and end at 278, and input.c puts the
// shift arrow at 280 - between that and the battery frame at 289 there is
// nothing to take.
#define TRIGGER_EDGE_X         243
#define TRIGGER_EDGE_Y         2

// The vertical scale field: x=10 up to the AC/DC glyph at 54. Six characters of
// the large font, or two rows of the small one when a probe is being counted in
// - see draw_vertical_scale().
#define VSCALE_FIELD_W         44
#define PROBE_TAG_COLOR        LCD_COLOR(255, 170, 0)
#define SAMPLE_RATE_X          252

#define CALIB_AREA_LEFT        140
#define CALIB_AREA_WIDTH       (LCD_WIDTH - CALIB_AREA_LEFT)

#define MAX_SAMPLE_RATE_LIMIT  13

#define TOAST_TIMEOUT          1500
#define TOAST_COLOR            LCD_COLOR(255, 255, 0)

#define WAIT_STATE_HOLDOFF     100

#define BG_COLOR               LCD_COLOR(0, 0, 0)
#define TRACE_COLOR            LCD_COLOR(255, 255, 0)
#define TRACE_FILLED_COLOR     LCD_COLOR(0, 255, 0)
// ...and the same reconstructed pixels once the record no longer supports
// them. sin(x)/x is exact down to about three samples per period and then
// falls apart fast - measured against the true waveform, the worst error over
// a 200-count swing goes 1.2 counts at 3.1 samples, 7 at 2.31, 64 at 2.08.
// The colour is the only warning there can be, because the curve itself stays
// perfectly smooth and confident all the way down. See sinc_between().
#define TRACE_RECON_WARN_COLOR LCD_COLOR(255, 150, 0)
#define TRACE_CLIP_COLOR       LCD_COLOR(255, 0, 0)
#define TRACE_INVALID_COLOR    LCD_COLOR(255, 0, 0)

// Persistence: the envelope accumulated across frames, drawn behind the
// live trace in a dimmed tone so the current sweep stays readable over it
#define TRACE_PERSIST_COLOR    LCD_COLOR(120, 120, 24)

// ...and in decay mode, the brightest step of a ramp down to black. A fresh
// glow starts above the flat tone infinite persistence uses and falls through
// it, but stays well under TRACE_COLOR: the beam has to read as the brightest
// thing on the grid. Taken up to (210,210,60) it did not, and the band that
// trigger jitter legitimately paints across a steep edge - the beam really
// does sweep all of it, every frame - stopped looking like a glow and started
// looking like a solid slab of trace. 16 steps is finer than the eye
// separates on a dim colour and costs 32 bytes of table.
//
// As components, because the ramp has to scale them: an LCD_COLOR() here
// would have to be unpacked again to be dimmed, and a second copy of the
// numbers written out beside it is a constant that can be edited without
// changing anything - which is exactly what happened.
#define PERSIST_GLOW_R         165
#define PERSIST_GLOW_G         165
#define PERSIST_GLOW_B         40
#define PERSIST_RAMP_STEPS     16
// How long a glow takes to reach black. Wall clock, not frames: the frame
// rate here runs from hundreds a second to one per six seconds, and a decay
// counted in frames would be a different decay on every timebase.
#define PERSIST_DECAY_MS       1500
// Persistence modes, in the order config.persist_mode stores them - see the
// note on the field for why 1 is the infinite one
enum
{
  PERSIST_OFF = 0,
  PERSIST_INFINITE,
  PERSIST_DECAY,
};

// Measurement cursors: the active one is solid, its partner dashed
#define CURSOR_T_COLOR         LCD_COLOR(0, 200, 255)
#define CURSOR_V_COLOR         LCD_COLOR(255, 120, 255)

#define TREND_INTERVAL_MS      1000  // logger tick; the ring compresses 2:1
                                     // when full, so the span only grows
#define TREND_MARGIN_PX        12    // headroom above/below the plotted curve

// Roll view. The sample rate is pinned rather than derived from the timebase:
// a roll screen spans minutes, the record that would have to span it does not
// exist, and the only thing the rate still decides is how narrow a spike the
// peak fold can catch and how far behind the main loop may fall before the
// ring laps it. Divider 10 is 8.192 us per sample, so the ring holds 805 ms
// of history - longer than any main-loop stall - and a 20 us runt still lands
// in the envelope.
#define ROLL_SR_DIVIDER        10
#define ROLL_STATE_COLOR       LCD_COLOR(255, 180, 50)
// Pseudo capture state: rolling is neither WAIT nor TRIG, and saying so in
// the state slot is the only place the mode announces itself
#define CAPTURE_STATE_ROLL     3

#define GRID_BG_COLOR          LCD_COLOR(0, 0, 0)
#define GRID_FG_COLOR          LCD_COLOR(200, 200, 200)
#define MV_FRAME_COLOR         LCD_COLOR(230, 230, 230)

#define HSCALE_COLOR           LCD_COLOR(255, 255, 255)
#define HPOS_COLOR             LCD_COLOR(255, 180, 50)
#define VSCALE_COLOR           LCD_COLOR(255, 255, 0)

#define TRIGGER_LEVEL_COLOR    LCD_COLOR(50, 255, 255)
#define TRIGGER_MODE_COLOR     LCD_COLOR(0, 255, 0)

#define MEASURE_MODE_COLOR     LCD_COLOR(50, 255, 255)
#define MEASURE_VOLTAGE_COLOR  LCD_COLOR(255, 255, 0)
// A reading the record cannot support, and one that is only just supported.
// Red is the same red the trace clips in, which is usually the reason for it.
#define MEASURE_BAD_COLOR      LCD_COLOR(255, 48, 48)
#define MEASURE_WEAK_COLOR     LCD_COLOR(255, 132, 0)
#define MEASURE_FREQ_COLOR     LCD_COLOR(255, 255, 255)

#define CAPTURE_STOP_COLOR     LCD_COLOR(255, 0, 0)
#define CAPTURE_WAIT_COLOR     LCD_COLOR(255, 180, 50)
#define CAPTURE_TRIG_COLOR     LCD_COLOR(0, 255, 0)

#define SR_LIMIT_COLOR         LCD_COLOR(250, 50, 50)
#define SR_COLOR               LCD_COLOR(0, 230, 0)

#define MIN_TRIGGER_LEVEL      -100 // px
#define MAX_TRIGGER_LEVEL       100 // px

#define MIN_HORIZONTAL_POSITION -2000000000 // 2 seconds
#define MAX_HORIZONTAL_POSITION  2000000000 // 2 seconds

#define MIN_VERTICAL_POSITION  (-10 * GRID_DIV_PX)
#define MAX_VERTICAL_POSITION  ( 10 * GRID_DIV_PX)

#define MEASURE_UPDATE_TIMEOUT 100
#define MEASURE_ITEMS_MAX      (MEASURE_COUNT - 1) // every metric but NONE

// Status-line layout: a one-character tag and a value per slot. Values are
// right-aligned in a fixed width by format_*(), which comes to 76 px, so a
// tagged slot is 84 and the two of them run 140..312.
//
// At the large size they are twice that and own a row to themselves - 152 px
// each, from x=2 and x=164 - with the settings (V/div, AC/DC, s/div) on the row
// below. Two readings is what 320 px holds at 16 px a character, which is why
// the settings did not double with them.
#define MEASURE_SLOT_0_X       (g_geom.slot0_x)
#define MEASURE_SLOT_1_X       (g_geom.slot1_x)
#define MEASURE_TAG_W          8     // one glyph of the large font, both sizes

// "There is no reading", at the width format_time() and format_voltage() come
// out at, so it covers the number it replaces instead of leaving its tail on
// screen. A time metric prints this rather than a zero: a period of 0 ns is
// not a measurement of anything.
#define PERIOD_NONE            "   --.--  "

// The decode panel is a hole in the trace area: columns it covers only
// paint their part BELOW it, so neither grid nor trace ever touches the
// panel pixels and it needs no repainting while it is on screen.
// Aligned to the column grid: col N sits at screen x = GRID_LEFT+1+N.
#define DECODE_PANEL_COL0      2
#define DECODE_PANEL_W         180
#define DECODE_PANEL_COL1      (DECODE_PANEL_COL0 + DECODE_PANEL_W - 1)
#define DECODE_PANEL_X         (GRID_LEFT + 1 + DECODE_PANEL_COL0)
#define DECODE_PANEL_Y         (GRID_TOP + 1)
#define DECODE_PANEL_H         52

// Translucent measurements panel: composited INTO the trace columns (trace
// dimmed to 50%, text overlaid), sitting just above the status line
/*
 * The measurements panel: a band along the bottom of the trace area holding
 * the selected readings, two rows deep.
 *
 * What made the old band hard to read was not where the readings sat but that
 * they MOVED: it packed them into two lines of text, and a value free to change
 * width - a jitter figure going from "31ps~1.2ns" to "1.2ns~15ns", a signal type
 * from "Sine" to "Square" - shoved everything behind it along the line. So every
 * value is formatted to a constant width for its metric now (see format_ps and
 * MEASURE_TYPE), which is what actually holds the band still.
 *
 * With that fixed the packing can be tight, and tight is what reads: a name, one
 * space, its number, and two spaces before the next pair. A cell whose value was
 * right-aligned in a fixed column instead put five blanks between "duty" and
 * "50.0%", and a name that far from its number stops looking like it belongs to
 * it - which is worse than the problem it was solving.
 *
 * A reading therefore only ever changes its own digits. Where the pairs sit
 * changes when the SET changes, never because a number did.
 */
#define MPANEL_PAD_X           4
#define MPANEL_PAD_Y           3
#define MPANEL_ROWS            2
#define MPANEL_GAP_Y           2     // between the two rows
#define MPANEL_GUTTER          2     // characters between one pair and the next

// Room for the longest pair there is: a four-character name, its space, and a
// jitter reading (two five-character times and a tilde)
#define MPANEL_CHARS_MAX       20
#define MPANEL_CELLS_MAX       8

// Free text (the calibration hints, the auto-calibration's running commentary)
// is sentences rather than readings, so it keeps the small font and the full
// width of the band whatever the panel is set to.
#define MPANEL_TEXT_MAX        50    // chars per line at 6 px/char
#define MPANEL_HEARTBEAT       4     // forced repaints: every 4th update (~2 s)
#define MPANEL_DIM(p)          (((p) >> 1) & 0x7BEF) // RGB565 half brightness

// The hairline along the top of a solid band, so it reads as a panel over the
// trace rather than as a hole in it
#define MPANEL_EDGE_COLOR      LCD_COLOR(70, 70, 70)

// A reading that has nothing to report: grey, name and all. THD with no
// spectrum behind it, jitter with no periods - the cell stays where it is so
// the ones that DO have a number do not move, and the grey says which is which
// without having to read the dashes.
#define MPANEL_NONE_COLOR      LCD_COLOR(130, 130, 130)

// Small dim "rec <span>" tag in the top-right trace corner while the frozen
// record is only the 24K storage snapshot (capture_stopped_on_snapshot):
// zooming a stopped NORMAL/SINGLE catch is limited to that window, and the
// tag says how much record there is. Same opaque-hole trick as the decode
// panel, clear of it (the panel ends at column 181) and of the FFT panel
// (the tag never shows in the spectrum view).
#define SNAP_TAG_W             64    // 10 chars of the small font + margins
#define SNAP_TAG_COL1          (GRID_WIDTH - 3)
#define SNAP_TAG_COL0          (SNAP_TAG_COL1 - SNAP_TAG_W + 1)
#define SNAP_TAG_H             10
#define SNAP_TAG_COLOR         LCD_COLOR(110, 110, 110)

#define FFT_DB_RANGE           60.0f // spectrum display dynamic range

// Spectrum rendering: a continuous curve over a heat-gradient fill (the
// classic analyzer look — quiet bins glow blue-green, strong peaks run
// through yellow into red). Detected peaks carry a floating tick in their
// class colour, the cursor is a hairline standing on the curve.
#define FFT_LINE_COLOR         LCD_COLOR(255, 255, 210)
#define FFT_PEAK_H_COLOR       LCD_COLOR(255, 255, 255) // harmonic of F0
#define FFT_PEAK_X_COLOR       LCD_COLOR(255, 128, 32)  // foreign peak
#define FFT_CURSOR_COLOR       LCD_COLOR(0, 190, 190)

// Spectrum analysis panel: same "hole in the sweep" trick as the decode
// panel, parked on the right where the high-frequency end of a spectrum is
// almost always empty. Eight rows of the small font plus margins.
#define FFT_PANEL_W            168   // 26 chars of the small font + margins
#define FFT_PANEL_COL0         (GRID_WIDTH - 2 - FFT_PANEL_W)
#define FFT_PANEL_COL1         (FFT_PANEL_COL0 + FFT_PANEL_W - 1)
#define FFT_PANEL_X            (GRID_LEFT + 1 + FFT_PANEL_COL0)
#define FFT_PANEL_Y            (GRID_TOP + 1)
#define FFT_PANEL_H            80
#define FFT_PANEL_ROWS         5     // peak rows

#define FFT_UPDATE_TIMEOUT     100   // recompute the spectrum at most at 10 Hz
#define FFT_FILL_MIN_MS        1000  // shorter than this, the record fills
                                     // faster than the readout could be read
#define FFT_FILL_TICK_MS       500   // ...and it ticks at 2 Hz while it does
#define FFT_AUTO_BAND_MARGIN   6     // keep the measured frequency this far
                                     // below the analysis Nyquist

#define AUTOSET_TIMEOUT        8000  // give up on auto-setup after 8 s
#define AUTOSET_SURVEY_HS      HS_5_ms   // ~100 ms record: >= 5 periods of 50 Hz
#define AUTOSET_FAST_HS        HS_10_us  // maximum sample rate, ~800 us record
#define AUTOSET_SLOW_HS        HS_50_ms  // ~800 ms record: down to a few Hz
#define AUTOSET_FAST_MIN_HZ    50000 // below this the survey record knows better
#define AUTOSET_TARGET_PX      (6 * GRID_DIV_PX) // fill 6 of the 8 divisions
#define AUTOSET_VIEW_PERIODS   25    // periods across the screen, x10
#define AUTOSET_CENTER_TOL     8     // counts of DC offset we accept as centered
#define AUTOSET_VERT_TRIES     10    // vertical actions before we settle
#define AUTOSET_REFINE_TRIES   4
#define AUTOSET_FREQ_TOL_PCT   30    // estimate vs re-measurement agreement
#define AUTOSET_MIN_PERIODS    3     // periods a believable frequency needs
#define AUTOSET_MIN_GOOD_PCT   60    // ... of that regularity
#define AUTOSET_MIN_LEVEL_PCT  65    // ... or, when irregular, this digital
#define AUTOSET_MIN_SIGNAL_MV  100   // less than this at 50 mV/div is noise
#define AUTOSET_NO_ERR         0x7fffffff

// Auto-calibration against an OPEN input. Three of the four parameters are
// derivable that way; the fourth (gain) needs a reference amplitude and stays
// manual. See autocal_step().
#define AUTOCAL_TIMEOUT        20000 // give up after 20 s
#define AUTOCAL_SETTLE         6     // records to drop after a hardware change,
                                     // enough for the 50/100 mV relay to land
#define AUTOCAL_ZERO_TOL       25    // counts*100 we call centred (1/4 count)
#define AUTOCAL_DAC_PX         60    // position step the DAC calibration uses:
                                     // large enough to measure, far from clipping
#define AUTOCAL_HS             HS_10_us // short records, maximum sample rate

// Offset DAC zero: the window the hardware can actually be centred in
#define CALIB_DAC_ZERO_MIN     1900
#define CALIB_DAC_ZERO_MAX     2200

// Gain reference: the level you apply and can measure independently
#define CALIB_REF_MIN_MV       50
#define CALIB_REF_MAX_MV       40000
#define CALIB_REF_DEFAULT_MV   1000
// Reject a reading too small to divide by, or one that has run off the ADC
#define CALIB_REF_MIN_COUNTS   20
// Largest correction the gain step will commit, in 0.1%. An instrument that
// needs more than half a range is not out of calibration, it is being told
// the wrong thing - and the factory numbers it would be overwriting came off
// a real unit.
#define GAIN_TRIM_LIMIT        500

enum
{
  CALIB_ZERO,
  CALIB_DELTA,
  CALIB_SCALE,
  CALIB_OFFSET,
};

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  uint8_t  min[GRID_WIDTH];
  uint8_t  max[GRID_WIDTH];
  uint8_t  flags[GRID_WIDTH];
} DisplayBuffer;

// What a reading is OF, which is how the panel colours it: the same yellow the
// vertical scale and the first status slot use for volts, the same white the
// second uses for frequency. A panel where every number is white is a panel
// you have to read the labels of to know what you are looking at.
typedef enum
{
  MK_VOLT = 0,  // volts
  MK_TIME,      // periods, widths, frequencies - anything the timebase decides
  MK_OTHER,     // ratios, the signal type, the alias candidate
} MeasureKind;

// One selected metric, rendered for both views at once: the panel puts `label`
// and `value` in a cell of its grid, the status line prints `tag` and `value`
// in the large font, where a single character is all the room there is for a
// name. `present` is false when the metric has nothing to report right now -
// no spectrum for a THD, no periods for a jitter - and the panel greys the
// whole cell rather than dropping it, so the layout does not move under the
// reading that is still there.
/*
 * How much the RECORD supports the number, which is not the same question as
 * whether the signal is any good - and the instrument can only answer the
 * first one. It does not know what this signal is for: 1% of jitter is broken
 * on a clock and normal on a servo, and a firmware that painted the second one
 * red would be inventing a fault. What it does know exactly is when its own
 * record cannot carry the reading it just printed.
 */
typedef enum
{
  MQ_OK = 0,
  MQ_WEAK,   // usable, but against a limit: coarse, or few cycles, or the floor
  MQ_BAD,    // the record cannot support this number at all
} MeasureQuality;

typedef struct
{
  char label[6];
  char tag[2];
  char value[16];
  uint8_t metric;   // which MEASURE_* this is, for the layout editor
  uint8_t kind;
  uint8_t quality;  // MeasureQuality, and what colours the reading
  bool present;
} MeasureItem;

/*- The chrome: status bars, formatting, utf8, the record map ---------------*/
// Drawn by scope_chrome.c; state shared with the rest of the scope.

typedef struct
{
  int  x, y, w, h;
  bool valid;
} MarkerRect;

extern MarkerRect g_vpos_marker;
extern MarkerRect g_trig_marker;
extern bool g_toast_active;
extern int g_toast_timer;
extern int g_state;              // capture state as the top bar last drew it
extern int g_state_timer;
extern int g_sample_rate;
extern bool g_snap_tag;
extern uint8_t g_mv_env[];       // miniview envelope; sized by scope_chrome.c
extern bool g_mv_env_valid;
extern int g_mv_timer;
extern uint32_t g_mv_env_gen;

extern const char *const hs_str[HS_COUNT];
extern const int64_t hs_div_value[HS_COUNT];
extern const int hs_px_value[HS_COUNT];
extern const char *const vs_str[VS_COUNT];
extern const int vs_px_value[VS_COUNT];

int vs_mv_px(int scale);
const char *vs_label(int scale);
void toast_show(void);
void draw_grid_frame(void);
void draw_ac_dc(void);
void draw_horizontal_scale(void);
void draw_horizontal_position(void);
void draw_marker_image(MarkerRect *prev, int x, int y, const Image *image);
void draw_vertical_position(bool toast);
void draw_vertical_scale(void);
void draw_trigger_level(void);
void draw_trigger_edge(void);
void draw_trigger_mode(void);
char *format_duty(int duty_x10);
void format_ps(int ps, char *out, int size);
void format_ns(int64_t ns, char *out, int size);
void format_hz_mhz(int64_t mhz, char *out, int size);
void format_mv(int mv, char *out, int size);
void overlay_repaint_region(int row0, int rows);
void snap_tag_paint(void);
int utf8_len(const uint8_t *b, int n);
uint32_t utf8_code_point(const uint8_t *b, int len);
int utf8_char_count(const uint8_t *bytes, int count);
void utf8_group_at(const uint8_t *bytes, int count, int idx,
                   int *start, int *len);
int text_width(const char *str);
void draw_capture_state(void);
void draw_miniview(int trigger_offset, int window_offset, int window_width);
void redraw_miniview(void);
void draw_sample_rates(int sample_rate_limit, int sample_rate);
void update_sample_rate(void);
int64_t roll_screen_ns(void);   // scope.c (roll)
void draw_status_line(void);

// Still owned by scope.c, called from scope_chrome.c:
int roll_row(int raw);
void build_trace_column(int c, uint16_t *column);
bool measure_owns_status_line(void);
void measure_slot(int slot, int x, const char *tag, const char *value,
    int color);
void decode_band_build(void);
void layout_edit_footer(void);
void draw_measure(void);
void cursor_readout(void);
void trend_readout(void);

// Auto-calibration phases, in the order autocal_step() walks them
enum
{
  ACAL_ASK,    // waiting for the user to prepare the input and confirm
  ACAL_ZERO,   // single channel: put ADC B's mean on ZERO_POINT
  ACAL_DELTA,  // dual channel: match ADC A's mean to ADC B's
  ACAL_DAC,    // per range: one position pixel must move the trace one pixel
  ACAL_LIN,    // sweep the offset DAC across the window and fit the bend in it
  ACAL_ASK_REF,// waiting for the reference voltage to be connected
  ACAL_SCALE,  // gain: make the reading equal that reference
  ACAL_DONE,
};

// The calibration flows (scope_calib.c), driven from scope.c's task/buttons:
void autocal_mark(void);
void autocal_say(const char *l0, const char *l1);
void autocal_begin(void);
void autocal_close(void);
void autocal_finish(bool ok);
void autocal_step(void);
void calib_hint_update(void);
void change_calibration_value(int delta, bool shift, bool repeat);
void draw_calibration_info(void);
void gain_ref_step(int dir, bool repeat);
void gain_range_step(int delta);

/*- Shared state (definitions stay with their owner file) ------------------*/
extern LogicResult g_logic;
extern ScopeGeom g_geom;
extern bool g_calibration_dual_channel;
extern bool g_decode_held;
extern bool g_decode_mode;
extern bool g_fft_mode;
extern bool g_layout_edit;
extern bool g_logic_have;
extern bool g_roll_active;
extern bool g_trend_mode;
extern int     g_cursor_sel;
extern int g_decode_sel;
extern int g_decode_size;
extern int g_trace_column;
extern uint8_t g_roll_row_flags[GRID_WIDTH];
extern uint8_t g_roll_row_max[GRID_WIDTH];
extern uint8_t g_roll_row_min[GRID_WIDTH];

/*- Shared state (definitions stay with their owner file) ------------------*/
extern bool g_autocal_active;
extern bool g_autocal_gain_only;
extern bool g_calib_hint;
extern int g_autocal_phase;
extern int g_autocal_timer;
extern int g_calib_hint_param;
extern uint32_t g_gain_note_until;

// Cross-file (owner: scope.c):
void autoset_set_horizontal(int hs);
void autoset_set_scale(int scale);
void change_vertical_scale(int delta);
int clip_for_display(int value);
void mpanel_invalidate(void);
void mpanel_set_active(bool active);
void mpanel_set_lines(const char *l0, const char *l1);
bool mpanel_wanted(void);
void redraw_trace(void);
void refresh_view(void);
void update_display(void);

/*- Shared state (definitions stay with their owner file) ------------------*/
extern DataBuffer g_data_buffer;
extern DisplayBuffer g_display_buffer;
extern bool g_line_owner;
extern bool g_mpanel_active;

#endif // _SCOPE_INTERNAL_H_
