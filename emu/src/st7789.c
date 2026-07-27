/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The ST7789 panel, on the far side of the parallel bus lcd.c bit-bangs.
 *
 * Nothing above this file knows the display exists: board.c watches the WR
 * pin, and when it rises it hands over whatever byte the firmware had put on
 * PE[7:0], plus the RS pin that says command or data. So the whole of lcd.c -
 * the window setup, the font rasteriser, the clipping, the RGB565 packing -
 * runs exactly as it does on the device, and a bug in any of it shows up here
 * as the wrong pixels rather than being papered over.
 */

#include <string.h>

#include "emu.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  ST7789_NOP     = 0x00,
  ST7789_SWRESET = 0x01,
  ST7789_SLPIN   = 0x10,
  ST7789_SLPOUT  = 0x11,
  ST7789_INVOFF  = 0x20,
  ST7789_INVON   = 0x21,
  ST7789_DISPOFF = 0x28,
  ST7789_DISPON  = 0x29,
  ST7789_CASET   = 0x2a,
  ST7789_RASET   = 0x2b,
  ST7789_RAMWR   = 0x2c,
  ST7789_MADCTL  = 0x36,
  ST7789_COLMOD  = 0x3a,
};

/*- Variables ---------------------------------------------------------------*/
uint16_t st7789_fb[LCD_W * LCD_H];
uint64_t st7789_writes;
uint64_t st7789_sweep_end;
bool st7789_display_on;

/* The scope paints its trace one column per pass of the main loop, so any
 * single frame of the panel is a mixture of the sweep in progress and the one
 * before it. Watching the bus for the moment a run of single-column windows
 * jumps back to the left tells us a sweep has just finished - the one instant
 * at which the panel holds a whole coherent frame. Nothing here knows what
 * the firmware is drawing; it only knows the shape of the accesses. */
static int g_last_col_x = -1;

static uint8_t g_cmd;
static int g_param;          /* how many data bytes this command has taken */

static int g_col_start, g_col_end;
static int g_row_start, g_row_end;
static int g_x, g_y;         /* the address counter                        */
static bool g_in_ramwr;
static uint8_t g_pixel_hi;
static bool g_have_hi;
static uint8_t g_madctl;
static bool g_reset_level = true;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void st7789_init(void)
{
  memset(st7789_fb, 0, sizeof(st7789_fb));

  g_cmd = ST7789_NOP;
  g_param = 0;
  g_col_start = g_row_start = 0;
  g_col_end = LCD_W - 1;
  g_row_end = LCD_H - 1;
  g_x = g_y = 0;
  g_in_ramwr = false;
  g_have_hi = false;
  g_madctl = 0;
  g_last_col_x = -1;
  st7789_display_on = false;
  st7789_writes = 0;
}

//-----------------------------------------------------------------------------
// A low pulse on the RST pin restarts the controller. lcd_init() uses it
// before it sends a single command.
void st7789_reset_pin(bool level)
{
  if (g_reset_level && !level)
  {
    bool on = st7789_display_on;
    uint64_t writes = st7789_writes;

    st7789_init();

    // The panel keeps its pixels across a reset pulse; only the state machine
    // restarts. Leaving them means the emulator's window does not flash black
    // every time the firmware reboots.
    st7789_display_on = on;
    st7789_writes = writes;
  }

  g_reset_level = level;
}

//-----------------------------------------------------------------------------
static void ramwr_byte(uint8_t value)
{
  if (!g_have_hi)
  {
    g_pixel_hi = value;
    g_have_hi = true;
    return;
  }

  g_have_hi = false;

  if (g_x >= 0 && g_x < LCD_W && g_y >= 0 && g_y < LCD_H)
    st7789_fb[g_y * LCD_W + g_x] = (uint16_t)((g_pixel_hi << 8) | value);

  st7789_writes++;

  // The controller walks the window it was given, wrapping column-first. Off
  // the end of the window it starts again at the top-left, which is exactly
  // the behaviour lcd.c's clipping comments warn about.
  if (++g_x > g_col_end)
  {
    g_x = g_col_start;

    if (++g_y > g_row_end)
      g_y = g_row_start;
  }
}

//-----------------------------------------------------------------------------
void st7789_bus_write(uint8_t value, bool is_command)
{
  if (is_command)
  {
    g_cmd = value;
    g_param = 0;
    g_in_ramwr = (value == ST7789_RAMWR);
    g_have_hi = false;

    if (g_in_ramwr)
    {
      g_x = g_col_start;
      g_y = g_row_start;

      if (g_col_end == g_col_start)
      {
        if (g_col_start <= g_last_col_x)
          st7789_sweep_end++;

        g_last_col_x = g_col_start;
      }
    }

    switch (value)
    {
      case ST7789_DISPON:  st7789_display_on = true;  break;
      case ST7789_DISPOFF: st7789_display_on = false; break;
      case ST7789_SWRESET: st7789_init();             break;
      default: break;
    }

    return;
  }

  if (g_in_ramwr)
  {
    ramwr_byte(value);
    return;
  }

  switch (g_cmd)
  {
    case ST7789_CASET:
      // Column address: two 16-bit values, big endian
      if      (g_param == 0) g_col_start = value << 8;
      else if (g_param == 1) g_col_start |= value;
      else if (g_param == 2) g_col_end = value << 8;
      else if (g_param == 3) g_col_end |= value;
      break;

    case ST7789_RASET:
      if      (g_param == 0) g_row_start = value << 8;
      else if (g_param == 1) g_row_start |= value;
      else if (g_param == 2) g_row_end = value << 8;
      else if (g_param == 3) g_row_end |= value;
      break;

    case ST7789_MADCTL:
      // The firmware sets 0x60 - column/row exchange plus column mirror -
      // which is what turns the 240x320 panel into the 320x240 landscape
      // surface every coordinate in the firmware assumes. The framebuffer
      // here is that surface, so the addresses arrive already in it.
      g_madctl = value;
      break;

    default:
      break;
  }

  g_param++;
}
