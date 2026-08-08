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

/*- Variables and implementations -------------------------------------------*/

bool g_mpanel_active = false;
static bool g_mpanel_force = false;   // rebuild the panel text on the next tick
uint32_t g_mpanel_builds = 0;  // texts built / bands repainted, for
uint32_t g_mpanel_paints = 0;  // the System Info diagnostic
MPanelCell g_mpanel_cell[MPANEL_CELLS_MAX];
int g_mpanel_cells = 0;
// ...or two lines of free text instead of the grid, for the things the band is
// borrowed to SAY rather than to measure (calibration hints, the
// auto-calibration's report). Sentences, so they keep the small font.
static char g_mpanel_text[2][MPANEL_TEXT_MAX + 2];
bool g_mpanel_is_text = false;

/*
 * ...or readings placed where the user put them, which is the other layout.
 *
 * Same idea as a band cell and the same painter, with the position in pixels
 * instead of character columns and the font per widget rather than per panel.
 * Built from config.measure_widget[] at the same 2 Hz the band is.
 */

PanelPlaced g_placed[PANEL_WIDGETS_MAX];
int g_placed_n = 0;

// The layout editor. It owns the screen and the keyboard while it is up: the
// trace is a mock waveform and the readings are mock values, so that arranging
// them is not a moving target and every one of them has something to show
// whatever is on the probe.
bool g_layout_edit = false;

// Logic decoder view; the run tables live in the spare main-SRAM block
bool g_decode_mode = false;
bool g_logic_have = false;
// The frames on the panel are older than the record on the screen: kept
// because the records since then had nothing in them (see decode_update)
bool g_decode_held = false;
LogicResult g_logic;
int g_decode_sel = 0;              // selected byte (jump target)
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
// Set when the record has fewer than RECON_MIN_SAMPLES samples per period, so
// the reconstruction drawn between them is past what it can honestly do.
// Recomputed once per display rebuild, read by every column.
bool g_recon_strained = false;
// Persistence and averaging state, all in display (column) space: persist is
// the accumulated envelope, avg the per-column EMA of the trace midpoint in
// px*64 fixed point. Both forget everything on any pan/zoom/scale change -
// the column-to-time mapping they accumulated under is gone.
static uint8_t  g_persist_min[GRID_WIDTH];
static uint8_t  g_persist_max[GRID_WIDTH];
// How brightly this column's envelope still glows, 0 = nothing there. Was a
// plain have/have-not flag; a byte was already being spent on it, so the
// decay mode costs no RAM at all - infinite persistence simply pins it full.
uint8_t  g_persist_lvl[GRID_WIDTH];
static uint16_t g_persist_ramp[PERSIST_RAMP_STEPS];
uint32_t g_persist_stamp;
static uint16_t g_avg_acc[GRID_WIDTH];
uint8_t  g_avg_have[GRID_WIDTH];
static uint32_t g_avg_gen;
//-----------------------------------------------------------------------------
// The panel's font and the geometry that follows from it. Everything that
// touches the band asks these rather than reading a constant: the band is
// taller in the 8x16 font, and what stacks on top of it (the decoder's byte
// strip) has to follow.
const Font *mpanel_font(void)
{
  return (PANEL_FONT_SMALL == config.measure_panel_font) ?
      FONT_SMALL : FONT_LARGE;
}

// ...and how many times over each of its pixels is drawn. There is no 16x32 font
// in the image, so the biggest band is the 8x16 one doubled - see PANEL_FONT_HUGE.
int mpanel_scale(void)
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
int mpanel_chars(void)
{
  return (GRID_WIDTH - 2 * MPANEL_PAD_X) / mpanel_cell_w();
}

int mpanel_row_h(void)
{
  return mpanel_font()->height * mpanel_scale() + MPANEL_GAP_Y;
}

// Two rows of readings, or one at the doubled size: two would be 72 px of the
// 200 the trace area has, and a band that owns a third of the screen is not a
// band any more. One row of 16x32 is 36 px - the same share the 8x16 band has.
int mpanel_rows(void)
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

int mpanel_row0(void)
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

//-----------------------------------------------------------------------------
void grid_init(void)
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
bool trace_ready(void)
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
void persist_build_ramp(void)
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
int widget_size_of(uint8_t flags)
{
  if (flags & PW_HUGE)
    return 2;

  return (flags & PW_LARGE) ? 1 : 0;
}

const Font *widget_font(int size)
{
  return size ? FONT_LARGE : FONT_SMALL;
}

int widget_scale(int size)
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
void widget_pos(const PanelWidget *w, int wide, int gh, int *px, int *py)
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
void widget_set_pos(PanelWidget *w, int x, int y, int wide, int gh)
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
void draw_trace(void)
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
// Build the placed readings from config.measure_widget[]. Slot for slot, not
// compacted: the editor selects by slot, and a hole in the middle of the array
// is a slot the user emptied rather than a reason to renumber the rest.
void widgets_update(const ScopeMeasure *sm)
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
void mpanel_update(void)
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

//-----------------------------------------------------------------------------
// Acquisition averaging, display space: an EMA of each column's midpoint
// across trigger-aligned frames, drawn as a thin trace instead of the
// min/max band. At fast timebases a column IS one sample, so this is
// textbook average mode there (~sqrt(N) noise reduction); at slow timebases
// it steadies the centre of the band. Only NEW frames advance the average -
// a pan redraw of the same record must not multiply-count it.
void display_average(void)
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
void display_persist_accum(void)
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



