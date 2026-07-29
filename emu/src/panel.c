/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The front panel: the body, the eighteen keys, and which keyboard key each
 * one answers to. See panel.h for what the coordinates mean.
 *
 * The arrangement follows the instrument: MENU/SAVE/TRIG down the left,
 * MODE/AUTO/STOP down the right, the cursor cross between them, and the two
 * rows of grey keys along the bottom - the trigger level on the left of those,
 * the way it is on the case.
 */

#ifndef EMU_NO_SDL

#include <string.h>

#include "emu.h"
#include "panel.h"

/*- Definitions -------------------------------------------------------------*/
#define KEY_COUNT  (int)(sizeof(g_panel) / sizeof(g_panel[0]))

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int         x, y, w, h;   /* panel units                                   */
  uint32_t    btn;          /* buttons.h BTN_*                               */
  const char *label;        /* NULL when the key is an arrow                 */
  const char *hint;         /* the keyboard key that works it                */
  char        arrow;        /* U D L R, or 0                                 */
  bool        amber;        /* the yellow keys, as opposed to the grey ones  */
} PanelKey;

/*- Constants ---------------------------------------------------------------*/
/* A 5x7 cell, one byte per column, bit 0 at the top. Enough of ASCII for the
 * labels and the key hints, which are all upper case. */
static const uint8_t g_font[][5] =
{
  { 0x00,0x00,0x00,0x00,0x00 }, /* space */
  { 0x23,0x13,0x08,0x64,0x62 }, /* %     */
  { 0x20,0x10,0x08,0x04,0x02 }, /* /     */
  { 0x3E,0x51,0x49,0x45,0x3E }, /* 0     */
  { 0x00,0x42,0x7F,0x40,0x00 },
  { 0x42,0x61,0x51,0x49,0x46 },
  { 0x21,0x41,0x45,0x4B,0x31 },
  { 0x18,0x14,0x12,0x7F,0x10 },
  { 0x27,0x45,0x45,0x45,0x39 },
  { 0x3C,0x4A,0x49,0x49,0x30 },
  { 0x01,0x71,0x09,0x05,0x03 },
  { 0x36,0x49,0x49,0x49,0x36 },
  { 0x06,0x49,0x49,0x29,0x1E }, /* 9     */
  { 0x7E,0x11,0x11,0x11,0x7E }, /* A     */
  { 0x7F,0x49,0x49,0x49,0x36 },
  { 0x3E,0x41,0x41,0x41,0x22 },
  { 0x7F,0x41,0x41,0x22,0x1C },
  { 0x7F,0x49,0x49,0x49,0x41 },
  { 0x7F,0x09,0x09,0x09,0x01 },
  { 0x3E,0x41,0x49,0x49,0x7A },
  { 0x7F,0x08,0x08,0x08,0x7F },
  { 0x00,0x41,0x7F,0x41,0x00 },
  { 0x20,0x40,0x41,0x3F,0x01 },
  { 0x7F,0x08,0x14,0x22,0x41 },
  { 0x7F,0x40,0x40,0x40,0x40 },
  { 0x7F,0x02,0x0C,0x02,0x7F },
  { 0x7F,0x04,0x08,0x10,0x7F },
  { 0x3E,0x41,0x41,0x41,0x3E },
  { 0x7F,0x09,0x09,0x09,0x06 },
  { 0x3E,0x41,0x51,0x21,0x5E },
  { 0x7F,0x09,0x19,0x29,0x46 },
  { 0x46,0x49,0x49,0x49,0x31 },
  { 0x01,0x01,0x7F,0x01,0x01 },
  { 0x3F,0x40,0x40,0x40,0x3F },
  { 0x1F,0x20,0x40,0x20,0x1F },
  { 0x3F,0x40,0x38,0x40,0x3F },
  { 0x63,0x14,0x08,0x14,0x63 },
  { 0x07,0x08,0x70,0x08,0x07 },
  { 0x61,0x51,0x49,0x45,0x43 }, /* Z     */
};

/* The panel, in panel units. The two columns and the cursor cross sit in the
 * band under the screen; the grey keys fill the two rows below that. */
static const PanelKey g_panel[] =
{
  {  16, 284, 64, 30, 1u <<  9, "MENU",  "ENTER", 0,   true  },
  {  16, 320, 64, 30, 1u << 13, "SAVE",  "S",     0,   true  },
  {  16, 356, 64, 30, 1u << 14, "TRIG",  "T",     0,   true  },

  { 280, 284, 64, 30, 1u <<  3, "MODE",  "M",     0,   true  },
  { 280, 320, 64, 30, 1u <<  7, "AUTO",  "A",     0,   true  },
  { 280, 356, 64, 30, 1u <<  0, "STOP",  "SPACE", 0,   true  },

  { 162, 286, 36, 28, 1u <<  1, NULL,    NULL,    'U', false },
  { 120, 320, 38, 28, 1u << 11, NULL,    NULL,    'L', false },
  { 202, 320, 38, 28, 1u <<  5, NULL,    NULL,    'R', false },
  { 162, 354, 36, 28, 1u << 15, NULL,    NULL,    'D', false },

  {  11, 396, 74, 32, 1u << 10, NULL,    "PGUP",  'U', false },
  {  99, 396, 74, 32, 1u <<  8, "50%",   "5",     0,   false },
  { 187, 396, 74, 32, 1u <<  6, "AC/DC", "C",     0,   false },
  { 275, 396, 74, 32, 1u << 17, "1X10X", "SHIFT", 0,   false },

  {  11, 434, 74, 32, 1u << 12, NULL,    "PGDN",  'D', false },
  {  99, 434, 74, 32, 1u <<  2, "EDGE",  "E",     0,   false },
  { 187, 434, 74, 32, 1u <<  4, "F1",    "F1",    0,   false },
  { 275, 434, 74, 32, 1u << 16, "F2",    "F2",    0,   false },
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static const uint8_t *glyph(char c)
{
  if (c >= 'a' && c <= 'z')
    c = (char)(c - 'a' + 'A');

  if (c == '%')            return g_font[1];
  if (c == '/')            return g_font[2];
  if (c >= '0' && c <= '9') return g_font[3 + (c - '0')];
  if (c >= 'A' && c <= 'Z') return g_font[13 + (c - 'A')];

  return g_font[0];
}

//-----------------------------------------------------------------------------
static void set_colour(SDL_Renderer *r, uint32_t rgb)
{
  SDL_SetRenderDrawColor(r, (uint8_t)(rgb >> 16), (uint8_t)(rgb >> 8),
      (uint8_t)rgb, 255);
}

//-----------------------------------------------------------------------------
static void fill(SDL_Renderer *r, int x, int y, int w, int h, uint32_t rgb)
{
  SDL_Rect rect = { x, y, w, h };

  set_colour(r, rgb);
  SDL_RenderFillRect(r, &rect);
}

//-----------------------------------------------------------------------------
// Text in window pixels, `px` being how big one font cell pixel comes out.
// Returns nothing; the caller has already worked out where it wants it.
static void text(SDL_Renderer *r, int x, int y, const char *s, int px,
    uint32_t rgb)
{
  set_colour(r, rgb);

  for (; *s; s++, x += 6 * px)
  {
    const uint8_t *g = glyph(*s);

    for (int col = 0; col < 5; col++)
    {
      for (int row = 0; row < 7; row++)
      {
        if (g[col] & (1u << row))
        {
          SDL_Rect p = { x + col * px, y + row * px, px, px };
          SDL_RenderFillRect(r, &p);
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
static int text_width(const char *s, int px)
{
  return (int)strlen(s) * 6 * px - px;
}

//-----------------------------------------------------------------------------
// A filled triangle, for the four cursor keys and the trigger level. Drawn as
// rows rather than with a polygon, which SDL's plain renderer has no call for.
static void triangle(SDL_Renderer *r, int cx, int cy, int size, char dir,
    uint32_t rgb)
{
  set_colour(r, rgb);

  for (int i = 0; i < size; i++)
  {
    int span = 2 * i + 1;
    SDL_Rect p;

    switch (dir)
    {
      case 'U': p = (SDL_Rect){ cx - i, cy - size / 2 + i, span, 1 }; break;
      case 'D': p = (SDL_Rect){ cx - i, cy + size / 2 - i, span, 1 }; break;
      case 'L': p = (SDL_Rect){ cx - size / 2 + i, cy - i, 1, span }; break;
      default:  p = (SDL_Rect){ cx + size / 2 - i, cy - i, 1, span }; break;
    }

    SDL_RenderFillRect(r, &p);
  }
}

//-----------------------------------------------------------------------------
void panel_draw(SDL_Renderer *r, int scale, uint32_t pressed)
{
  const int glyph_px = scale > 2 ? scale - 1 : 1;
  const int hint_px  = scale > 3 ? scale - 2 : 1;

  // The case: a dark body inside the yellow bumper the instrument is wrapped
  // in, with the screen sunk into a black bezel
  fill(r, 0, 0, PANEL_W * scale, PANEL_H * scale, 0xE8C317);
  fill(r, 4 * scale, 4 * scale, (PANEL_W - 8) * scale, (PANEL_H - 8) * scale,
      0x2A2E33);
  fill(r, (PANEL_SCREEN_X - 6) * scale, (PANEL_SCREEN_Y - 6) * scale,
      (LCD_W + 12) * scale, (LCD_H + 12) * scale, 0x101214);

  for (int i = 0; i < KEY_COUNT; i++)
  {
    const PanelKey *k = &g_panel[i];
    bool down = (pressed & k->btn) != 0;
    int x = k->x * scale, y = k->y * scale;
    int w = k->w * scale, h = k->h * scale;
    uint32_t face, ink;

    if (k->amber)
    {
      face = down ? 0xFFDD44 : 0xD8A81B;
      ink  = 0x201A05;
    }
    else
    {
      face = down ? 0xF2F4F6 : 0xB9BEC4;
      ink  = 0x1A1D20;
    }

    // A hint of relief: the shadow under the key, then the key on top of it
    fill(r, x, y + scale, w, h, 0x15181B);
    fill(r, x, y + (down ? scale : 0), w, h - (down ? scale : 0), face);

    if (k->arrow)
    {
      int cy = y + h / 2 + (down ? scale : 0);
      int size = (k->hint ? 5 : 7) * scale;

      triangle(r, x + w / 2, cy, size, k->arrow, ink);
    }

    if (k->label)
    {
      int tw = text_width(k->label, glyph_px);
      int ty = y + h / 2 - 4 * glyph_px + (down ? scale : 0);

      if (k->hint)
        ty -= 3 * glyph_px;

      text(r, x + (w - tw) / 2, ty, k->label, glyph_px, ink);
    }

    // The keyboard key that does the same thing, under the label
    if (k->hint)
    {
      int hw = text_width(k->hint, hint_px);
      int hy = y + h - 4 * hint_px - 3 * scale + (down ? scale : 0);

      text(r, x + (w - hw) / 2, hy, k->hint, hint_px,
          k->amber ? 0x6B5A18 : 0x63696F);
    }
  }
}

//-----------------------------------------------------------------------------
uint32_t panel_hit(int scale, int x, int y)
{
  for (int i = 0; i < KEY_COUNT; i++)
  {
    const PanelKey *k = &g_panel[i];

    if (x >= k->x * scale && x < (k->x + k->w) * scale &&
        y >= k->y * scale && y < (k->y + k->h) * scale)
      return k->btn;
  }

  return 0;
}

#endif /* EMU_NO_SDL */
