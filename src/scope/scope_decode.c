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
int g_dband_rows = 0;             // 0 = no band; bottom stays put
// Where the band starts, set with its height in decode_band_build(). Not
// DBAND_BOTTOM here: that follows the measurements panel, whose height is a
// setting now and not a constant an initializer could use.
int g_dband_row0 = 0;

// The protocol-level read of the same record - which device answered on the
// 1-Wire bus, which CAN frames went past, what the MIDI link said. Kept next
// to the result and not fetched on demand: the decoder's own copy belongs to
// whatever record it was last handed, and the panel can be holding frames
// several records old.
//
// One union, because only one of them is ever live: the snapshot is taken for
// the protocol the record decoded AS, and every reader dispatches on that same
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

bool g_decode_panel_pending = false;
bool g_decode_force = false;   // run the next decode past the throttle
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
bool g_decode_hunt = false;
// Arm the hunt on the next scope tick. The menu is a screen over the scope
// and the scope is not drawing while it is open, so a menu action leaves the
// work here rather than doing it under the menu.
bool g_decode_hunt_request = false;
bool g_spi_clock_request = false;
bool g_swd_clock_request = false;
static int g_decode_period_ns = 0;        // record metrics the decode ran on
static int g_decode_trig_pos = 0;
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

int decoder_man_rate_value(void)
{
  int i = config.man_rate;

  if (i < 0 || i >= DECODER_MAN_RATE_COUNT)
    return 0;

  return decoder_man_rate_values[i];
}

//-----------------------------------------------------------------------------
int decoder_baud_value(void)
{
  int i = config.decoder_baud;

  if (i < 0 || i >= DECODER_BAUD_COUNT)
    return 0;

  return decoder_baud_values[i];
}

//-----------------------------------------------------------------------------
// The same four questions, asked of whichever analysis is live in g_pana:
// snapshot it, name a byte, name a byte's field, find a byte's group. Each
// adapter binds one protocol's answer to one signature, so the sites that ask
// look the protocol up in the table below instead of switching on it.
static void pres_ow_capture(void) { g_pana.ow = *onewire_analysis(); }
static void pres_ow_byte(int idx, uint8_t v, char *buf, int n) { onewire_byte_label(&g_pana.ow, idx, v, buf, n); }
static void pres_ow_field(int idx, uint8_t v, char *buf, int n) { onewire_field_label(&g_pana.ow, idx, v, buf, n); }
static void pres_ow_group(int idx, int *start, int *len) { onewire_group_at(&g_pana.ow, idx, start, len); }

static void pres_can_capture(void) { g_pana.can = *can_analysis(); }
static void pres_can_byte(int idx, uint8_t v, char *buf, int n) { can_byte_label(&g_pana.can, idx, v, buf, n); }
static void pres_can_field(int idx, uint8_t v, char *buf, int n) { can_field_label(&g_pana.can, idx, v, buf, n); }
static void pres_can_group(int idx, int *start, int *len) { can_group_at(&g_pana.can, idx, start, len); }

static void pres_dht_capture(void) { g_pana.dht = *dht_analysis(); }
static void pres_dht_byte(int idx, uint8_t v, char *buf, int n) { dht_byte_label(&g_pana.dht, idx, v, buf, n); }
static void pres_dht_field(int idx, uint8_t v, char *buf, int n) { dht_field_label(&g_pana.dht, idx, v, buf, n); }
static void pres_dht_group(int idx, int *start, int *len) { dht_group_at(&g_pana.dht, idx, start, len); }

static void pres_sent_capture(void) { g_pana.sent = *sent_analysis(); }
static void pres_sent_byte(int idx, uint8_t v, char *buf, int n) { sent_byte_label(&g_pana.sent, idx, v, buf, n); }
static void pres_sent_field(int idx, uint8_t v, char *buf, int n) { sent_field_label(&g_pana.sent, idx, v, buf, n); }
static void pres_sent_group(int idx, int *start, int *len) { sent_group_at(&g_pana.sent, idx, start, len); }

static void pres_midi_capture(void) { g_pana.midi = *midi_analysis(); }
static void pres_midi_byte(int idx, uint8_t v, char *buf, int n) { midi_byte_label(&g_pana.midi, idx, v, buf, n); }
static void pres_midi_field(int idx, uint8_t v, char *buf, int n) { midi_field_label(&g_pana.midi, idx, v, buf, n); }
static void pres_midi_group(int idx, int *start, int *len) { midi_group_at(&g_pana.midi, idx, start, len); }

static void pres_lin_capture(void) { g_pana.lin = *lin_analysis(); }
static void pres_lin_byte(int idx, uint8_t v, char *buf, int n) { lin_byte_label(&g_pana.lin, idx, v, buf, n); }

static void pres_ev_capture(void) { g_pana.ev = *ev1527_analysis(); }
static void pres_ev_byte(int idx, uint8_t v, char *buf, int n) { ev1527_byte_label(&g_pana.ev, idx, v, buf, n); }
static void pres_ev_field(int idx, uint8_t v, char *buf, int n) { ev1527_field_label(&g_pana.ev, idx, v, buf, n); }
static void pres_ev_group(int idx, int *start, int *len) { ev1527_group_at(&g_pana.ev, idx, start, len); }

static void pres_dshot_capture(void) { g_pana.dshot = *dshot_analysis(); }
static void pres_dshot_byte(int idx, uint8_t v, char *buf, int n) { dshot_byte_label(&g_pana.dshot, idx, v, buf, n); }
static void pres_dshot_field(int idx, uint8_t v, char *buf, int n) { dshot_field_label(&g_pana.dshot, idx, v, buf, n); }
static void pres_dshot_group(int idx, int *start, int *len) { dshot_group_at(&g_pana.dshot, idx, start, len); }

static void pres_spi_capture(void) { g_pana.spi = *spi_analysis(); }
static void pres_spi_byte(int idx, uint8_t v, char *buf, int n) { spi_byte_label(&g_pana.spi, idx, v, buf, n); }
static void pres_spi_field(int idx, uint8_t v, char *buf, int n) { spi_field_label(&g_pana.spi, idx, v, buf, n); }
static void pres_spi_group(int idx, int *start, int *len) { spi_group_at(&g_pana.spi, idx, start, len); }

static void pres_man_capture(void) { g_pana.man = *manchester_analysis(); }
static void pres_man_byte(int idx, uint8_t v, char *buf, int n) { manchester_byte_label(&g_pana.man, idx, v, buf, n); }
static void pres_man_field(int idx, uint8_t v, char *buf, int n) { manchester_field_label(&g_pana.man, idx, v, buf, n); }
static void pres_man_group(int idx, int *start, int *len) { manchester_group_at(&g_pana.man, idx, start, len); }

static void pres_rc5_capture(void) { g_pana.rc5 = *rc5_analysis(); }
static void pres_rc5_byte(int idx, uint8_t v, char *buf, int n) { rc5_byte_label(&g_pana.rc5, idx, v, buf, n); }
static void pres_rc5_field(int idx, uint8_t v, char *buf, int n) { rc5_field_label(&g_pana.rc5, idx, v, buf, n); }
static void pres_rc5_group(int idx, int *start, int *len) { rc5_group_at(&g_pana.rc5, idx, start, len); }

static void pres_dali_capture(void) { g_pana.dali = *dali_analysis(); }
static void pres_dali_byte(int idx, uint8_t v, char *buf, int n) { dali_byte_label(&g_pana.dali, idx, v, buf, n); }
static void pres_dali_field(int idx, uint8_t v, char *buf, int n) { dali_field_label(&g_pana.dali, idx, v, buf, n); }
static void pres_dali_group(int idx, int *start, int *len) { dali_group_at(&g_pana.dali, idx, start, len); }

static void pres_knx_capture(void) { g_pana.knx = *knx_analysis(); }
static void pres_knx_byte(int idx, uint8_t v, char *buf, int n) { knx_byte_label(&g_pana.knx, idx, v, buf, n); }
static void pres_knx_field(int idx, uint8_t v, char *buf, int n) { knx_field_label(&g_pana.knx, idx, v, buf, n); }
static void pres_knx_group(int idx, int *start, int *len) { knx_group_at(&g_pana.knx, idx, start, len); }

static void pres_ws_capture(void) { g_pana.ws = *ws2812_analysis(); }
static void pres_ws_byte(int idx, uint8_t v, char *buf, int n) { ws2812_byte_label(&g_pana.ws, idx, v, buf, n); }
static void pres_ws_field(int idx, uint8_t v, char *buf, int n) { ws2812_field_label(&g_pana.ws, idx, v, buf, n); }
static void pres_ws_group(int idx, int *start, int *len) { ws2812_group_at(&g_pana.ws, idx, start, len); }

static void pres_swo_capture(void) { g_pana.swo = *swo_analysis(); }
static void pres_swo_byte(int idx, uint8_t v, char *buf, int n) { swo_byte_label(&g_pana.swo, idx, v, buf, n); }
static void pres_swo_field(int idx, uint8_t v, char *buf, int n) { swo_field_label(&g_pana.swo, idx, v, buf, n); }
static void pres_swo_group(int idx, int *start, int *len) { swo_group_at(&g_pana.swo, idx, start, len); }

static void pres_swd_capture(void) { g_pana.swd = *swd_analysis(); }
static void pres_swd_byte(int idx, uint8_t v, char *buf, int n) { swd_byte_label(&g_pana.swd, idx, v, buf, n); }
static void pres_swd_field(int idx, uint8_t v, char *buf, int n) { swd_field_label(&g_pana.swd, idx, v, buf, n); }
static void pres_swd_group(int idx, int *start, int *len) { swd_group_at(&g_pana.swd, idx, start, len); }

static void pres_usb_capture(void) { g_pana.usb = *usb_analysis(); }
static void pres_usb_byte(int idx, uint8_t v, char *buf, int n) { usb_byte_label(&g_pana.usb, idx, v, buf, n); }
static void pres_usb_field(int idx, uint8_t v, char *buf, int n) { usb_field_label(&g_pana.usb, idx, v, buf, n); }
static void pres_usb_group(int idx, int *start, int *len) { usb_group_at(&g_pana.usb, idx, start, len); }

static void pres_pd_capture(void) { g_pana.pd = *pd_analysis(); }
static void pres_pd_byte(int idx, uint8_t v, char *buf, int n) { pd_byte_label(&g_pana.pd, idx, v, buf, n); }
static void pres_pd_field(int idx, uint8_t v, char *buf, int n) { pd_field_label(&g_pana.pd, idx, v, buf, n); }
static void pres_pd_group(int idx, int *start, int *len) { pd_group_at(&g_pana.pd, idx, start, len); }

static void pres_sirc_capture(void) { g_pana.sirc = *sirc_analysis(); }
static void pres_sirc_byte(int idx, uint8_t v, char *buf, int n) { sirc_byte_label(&g_pana.sirc, idx, v, buf, n); }
static void pres_sirc_field(int idx, uint8_t v, char *buf, int n) { sirc_field_label(&g_pana.sirc, idx, v, buf, n); }
static void pres_sirc_group(int idx, int *start, int *len) { sirc_group_at(&g_pana.sirc, idx, start, len); }

static void pres_ppm_capture(void) { g_pana.ppm = *ppm_analysis(); }
static void pres_ppm_byte(int idx, uint8_t v, char *buf, int n) { ppm_byte_label(&g_pana.ppm, idx, v, buf, n); }
static void pres_ppm_field(int idx, uint8_t v, char *buf, int n) { ppm_field_label(&g_pana.ppm, idx, v, buf, n); }

//-----------------------------------------------------------------------------
// One row per protocol that has an analysis to present. A NULL member means
// the protocol has no such labels; the site's own fallback applies there,
// exactly as its default arm always did.
typedef struct
{
  proto_t proto;
  void (*capture)(void);
  void (*byte_label)(int idx, uint8_t v, char *buf, int n);
  void (*field_label)(int idx, uint8_t v, char *buf, int n);
  void (*group_at)(int idx, int *start, int *len);
} DecodePres;

const DecodePres g_decode_pres[] =
{
  { PROTO_ONEWIRE, pres_ow_capture,    pres_ow_byte,    pres_ow_field,    pres_ow_group    },
  { PROTO_CAN,     pres_can_capture,   pres_can_byte,   pres_can_field,   pres_can_group   },
  { PROTO_DHT,     pres_dht_capture,   pres_dht_byte,   pres_dht_field,   pres_dht_group   },
  { PROTO_SENT,    pres_sent_capture,  pres_sent_byte,  pres_sent_field,  pres_sent_group  },
  { PROTO_MIDI,    pres_midi_capture,  pres_midi_byte,  pres_midi_field,  pres_midi_group  },
  { PROTO_LIN,     pres_lin_capture,   pres_lin_byte,   NULL,             NULL             },
  { PROTO_EV1527,  pres_ev_capture,    pres_ev_byte,    pres_ev_field,    pres_ev_group    },
  { PROTO_DSHOT,   pres_dshot_capture, pres_dshot_byte, pres_dshot_field, pres_dshot_group },
  { PROTO_SPI,     pres_spi_capture,   pres_spi_byte,   pres_spi_field,   pres_spi_group   },
  { PROTO_MANCH,   pres_man_capture,   pres_man_byte,   pres_man_field,   pres_man_group   },
  { PROTO_RC5,     pres_rc5_capture,   pres_rc5_byte,   pres_rc5_field,   pres_rc5_group   },
  { PROTO_DALI,    pres_dali_capture,  pres_dali_byte,  pres_dali_field,  pres_dali_group  },
  { PROTO_KNX,     pres_knx_capture,   pres_knx_byte,   pres_knx_field,   pres_knx_group   },
  { PROTO_WS2812,  pres_ws_capture,    pres_ws_byte,    pres_ws_field,    pres_ws_group    },
  { PROTO_SWO,     pres_swo_capture,   pres_swo_byte,   pres_swo_field,   pres_swo_group   },
  { PROTO_SWD,     pres_swd_capture,   pres_swd_byte,   pres_swd_field,   pres_swd_group   },
  { PROTO_USB,     pres_usb_capture,   pres_usb_byte,   pres_usb_field,   pres_usb_group   },
  { PROTO_PD,      pres_pd_capture,    pres_pd_byte,    pres_pd_field,    pres_pd_group    },
  { PROTO_SIRC,    pres_sirc_capture,  pres_sirc_byte,  pres_sirc_field,  pres_sirc_group  },
  { PROTO_PPM,     pres_ppm_capture,   pres_ppm_byte,   pres_ppm_field,   NULL             },
};

//-----------------------------------------------------------------------------
static const DecodePres *pres_of(proto_t p)
{
  for (int i = 0; i < ARRAY_SIZE(g_decode_pres); i++)
  {
    if (g_decode_pres[i].proto == p)
      return &g_decode_pres[i];
  }

  return NULL;
}

// The character the selection is inside, so its bytes light up together on
// the trace exactly as they do in the panel
int g_dband_sel_start = 0;
int g_dband_sel_len = 1;
int8_t g_dband_byte[GRID_WIDTH];  // decoded byte under each column, -1 none
uint8_t g_dband_edge[(GRID_WIDTH + 7) / 8];             // byte starts
// ...and where a GROUP starts, which is a subset of them. A byte boundary is
// a fact about the rows that show bytes, and only about those: the bottom row
// holds one value written across every byte of its group, so the boundaries
// INSIDE that group divide nothing there. Ticking them anyway ran a coloured
// hairline down the middle of "#221133" and "S1=543" - the row whose whole
// point is that those bytes are one thing was the row being cut into three.
uint8_t g_dband_gedge[(GRID_WIDTH + 7) / 8];            // group starts
// Columns no byte covers that were filled in to make a group one strip. The
// fill is for the group's row and for nothing else: the rows that show BYTES
// keep their boundaries exactly as the record drew them, gaps included.
uint8_t g_dband_gap[(GRID_WIDTH + 7) / 8];
// The first row of the group's value. Above it a tick marks a byte, at it and
// below a tick marks a group; DBAND_H1 has no such row and never reaches it.
int g_dband_group_y = DBAND_H3;
uint8_t g_dband_mask[DBAND_H3][(GRID_WIDTH + 7) / 8];   // the text
// Which columns a swatch covers, and nothing else: the box's own edges are
// its outermost columns, and a column knows it is one by its neighbour not
// being in the box. Boxes are always separated by the text they sit beside,
// so two of them can never touch and be read as one.
uint8_t g_dband_sw[(GRID_WIDTH + 7) / 8];
bool g_dbit_on = false;
// The rows the grid lives between, and where the numbers sit inside them.
// Recomputed every frame from the trace's own envelope, because that is what
// they follow - the decode changes rarely, the signal's height every frame.
int g_dbit_top = 0;
int g_dbit_bot = -1;
// Middle of the grid until a decode places it; not GRID_HEIGHT/2-4 written out
// here, because the grid's height is a runtime value now and this is a file
// scope initialiser. decode_bit_rows() sets it before anything reads it.
int g_dbit_text_row = GRID_HEIGHT_MAX / 2 - 4;
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
bool decode_proto_is_serial(proto_t proto)
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
    // PPM has no group_at, deliberately: its channels each stand alone, and
    // a frame of eight of them is not one value written across eight bytes.
    // It and every other protocol without one keep the one-byte group above.
    const DecodePres *pres = pres_of(g_logic.proto);

    if (pres && pres->group_at)
      pres->group_at(idx, start, len);
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
bool decode_group_color(int idx, uint16_t *col)
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

  if (g_logic.proto == PROTO_NEC)
  {
    snprintf(buf, size, "%s", nec_field[idx & 3]);
    return;
  }

  if (g_logic.proto == PROTO_SERVO)
  {
    // The byte IS the width, in tens of microseconds; put it back into the
    // units the servo is commanded in, which is the only form anyone reads
    snprintf(buf, size, "%d.%02dms", v / 100, v % 100);
    return;
  }

  const DecodePres *pres = pres_of(g_logic.proto);

  if (pres && pres->byte_label)
  {
    pres->byte_label(idx, v, buf, size);

    // On SPI and USB a byte the transaction structure never reached is just
    // a byte, and the character it stands for is as much as can honestly be
    // said about it, exactly as on a serial line
    if (0 == buf[0] &&
        (g_logic.proto == PROTO_SPI || g_logic.proto == PROTO_USB))
      dband_ascii_text(buf, size, v);

    return;
  }

  // UART and the raw bit stream: text is what they usually carry
  dband_ascii_text(buf, size, v);
}

//-----------------------------------------------------------------------------
// What byte `idx` is ON ITS OWN - the field it belongs to, never the value
// its group adds up to. That value spans several bytes and is written once
// across them a row lower; this is the row that still says which byte of the
// three you are looking at, the way a protocol analyser numbers bit0 bit1
// bit2 above the bracket labelled "command".
static void dband_field_text(char *buf, int size, int idx, uint8_t v)
{
  const DecodePres *pres = pres_of(g_logic.proto);

  if (pres && pres->field_label)
  {
    pres->field_label(idx, v, buf, size);

    // On SPI a byte outside any transaction is just a byte, exactly as on
    // the meaning row below it
    if (0 == buf[0] && g_logic.proto == PROTO_SPI)
      dband_ascii_text(buf, size, v);

    return;
  }

  // Everything else names its bytes one at a time already
  dband_meaning_text(buf, size, idx, v);
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
int decode_bit_slots(int *data0, bool *msb_first)
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
// How tall the bit grid's lines are: exactly what the waveform spans under
// the decoded bytes, top pixel to bottom pixel. Measured over the decoded
// columns and not the whole screen, because those are the only columns the
// grid is drawn in - a marker pulse somewhere else in the record has nothing
// to do with how tall a line over THIS byte should be.
//
// Per frame, not per decode: the bytes move when the record changes, the
// signal's height moves whenever the vertical scale, the position or the
// signal itself does.
void dbit_span_update(void)
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
void decode_update(void)
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

  // g_logic.pos/end and the three anchors below must always describe the SAME
  // record: band_column() evaluates (pos - trig_pos) * period_ns, and a pan on
  // the full-rate window path moves the trigger's index inside the record
  // without changing a single decoded byte. Publishing new anchors while
  // keeping old positions shifted every mark by exactly the pan - which is
  // what byte-stepping does, cumulatively, until the marks sat on idle line.
  bool moved = (period_ns != g_decode_period_ns) ||
      (trig_pos != g_decode_trig_pos) || (size != g_decode_size);

  g_decode_period_ns = period_ns;
  g_decode_trig_pos = trig_pos;
  g_decode_size = size;

  bool changed = !g_logic_have || g_decode_held || moved ||
      res.count != g_logic.count ||
      res.proto != g_logic.proto || res.rate != g_logic.rate ||
      memcmp(res.bytes, g_logic.bytes, (size_t)res.count) != 0 ||
      memcmp(res.pos, g_logic.pos, sizeof(res.pos[0]) * (size_t)res.count) != 0;

  if (changed)
  {
    g_logic = res;

    const DecodePres *pres = pres_of(res.proto);

    if (pres)
      pres->capture();

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
void decode_jump_to_selected(void)
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

  // The pan above moved the record window under the marks; without a forced
  // re-decode the band is drawn from the pre-pan record for up to the whole
  // decode throttle interval
  g_decode_force = true;
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

void draw_decode_panel(void)
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
// Enter the decoder view: drop every conclusion the last session reached, put
// the trigger where a logic signal wants it, and set the scope up from the
// rate when the rate is already known.
//
// Told BOTH the protocol and the rate, there is nothing left to work out from
// a record, so there is no reason to look at a random one: the window, the
// trigger and the hunt for a message start can all be set before the first
// record arrives. That is the difference between reading a message and first
// having to catch one.
void decode_mode_enter(void)
{
  decode_trigger_save();
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
  // Before anything re-times the acquisition: the window set up below is a
  // record's, and roll pins the rate to something else. The trigger is NOT
  // auto-levelled here: the hunt path below does its own 50% where catching
  // a burst asks for it, and a level the user set on purpose survives a
  // plain look at the decoder view.
  roll_sync();

  if (decoder_baud_value() > 0 && config.decoder_fit_mode == 0 &&
      (proto_t)config.decoder_proto == PROTO_UART)
  {
    decode_arm_hunt(); // window, trigger and the hunt in one
  }
  else if (decoder_baud_value() > 0 &&
      decode_proto_is_serial((proto_t)config.decoder_proto))
  {
    // Only for a protocol the baud setting describes: fitting a PD or CAN
    // window to the UART baud number retimes the acquisition ~28x off
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

// The user's trigger, parked while the decoder view owns the acquisition.
// The hunt deliberately re-arms mode, edge and level to catch a burst; what
// was there before has to come back when the view is left, or every look at
// the decoder costs the trigger setup.
static int  g_decode_saved_level;
static int  g_decode_saved_mode;
static int  g_decode_saved_edge;
static bool g_decode_saved_valid = false;

void decode_trigger_save(void)
{
  if (g_decode_saved_valid) // re-entry (the catch action) keeps the original
    return;

  g_decode_saved_level = config.trigger_level;
  g_decode_saved_mode = config.trigger_mode;
  g_decode_saved_edge = config.trigger_edge;
  g_decode_saved_valid = true;
}

void decode_trigger_restore(void)
{
  if (!g_decode_saved_valid)
    return;

  config.trigger_level = g_decode_saved_level;
  config.trigger_mode = g_decode_saved_mode;
  config.trigger_edge = g_decode_saved_edge;
  g_decode_saved_valid = false;

  scope_apply_trigger_level();
  capture_set_trigger_mode(config.trigger_mode);
  capture_set_trigger_edge(config.trigger_edge);
  draw_trigger_level();
  draw_trigger_mode();
  draw_trigger_edge();
}

void decode_arm_hunt(void)
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

