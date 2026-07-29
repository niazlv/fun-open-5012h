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

/* The keys, in panel units, arranged the way they are on the case: two
 * columns of yellow ones down the sides with the cursor cross between them,
 * then the grey ones in two rows underneath. They are small and far apart -
 * most of the lower half of the instrument is bare plastic, and the panel
 * looks wrong without that space. */
static const PanelKey g_panel[] =
{
  {  46, 356, 62, 28, 1u <<  9, "MENU",  "ESC",     0, true  },
  {  46, 416, 62, 28, 1u << 13, "SAVE",  "S",       0, true  },
  {  46, 476, 62, 28, 1u << 14, "TRIG",  "T",       0, true  },

  { 330, 356, 62, 28, 1u <<  3, "MODE",  "ENTER M", 0, true  },
  { 330, 416, 62, 28, 1u <<  7, "AUTO",  "A",       0, true  },
  { 330, 476, 62, 28, 1u <<  0, "STOP",  "SPACE",   0, true  },

  { 186, 352, 66, 26, 1u <<  1, NULL,    NULL,    'U', false },
  { 145, 390, 26, 66, 1u << 11, NULL,    NULL,    'L', false },
  { 267, 390, 26, 66, 1u <<  5, NULL,    NULL,    'R', false },
  { 186, 468, 66, 26, 1u << 15, NULL,    NULL,    'D', false },

  {  46, 548, 62, 30, 1u << 10, NULL,    "PGUP",  'U', false },
  { 141, 548, 62, 30, 1u <<  8, "50%",   "5",     0,   false },
  { 236, 548, 62, 30, 1u <<  6, "AC/DC", "C",     0,   false },
  { 330, 548, 62, 30, 1u << 17, "1X10X", "SHIFT", 0,   false },

  {  46, 610, 62, 30, 1u << 12, NULL,    "PGDN",  'D', false },
  { 141, 610, 62, 30, 1u <<  2, "EDGE",  "E",     0,   false },
  { 236, 610, 62, 30, 1u <<  4, "F1",    "F1",    0,   false },
  { 330, 610, 62, 30, 1u << 16, "F2",    "F2",    0,   false },
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
// Nothing on this instrument has a square corner. Drawn a row at a time, with
// the corners inset by however far the radius has left to go.
static void round_rect(SDL_Renderer *r, int x, int y, int w, int h, int rad,
    uint32_t rgb)
{
  if (rad * 2 > w) rad = w / 2;
  if (rad * 2 > h) rad = h / 2;

  set_colour(r, rgb);

  for (int row = 0; row < h; row++)
  {
    int inset = 0;

    if (row < rad || row >= h - rad)
    {
      int dy = (row < rad) ? (rad - row - 1) : (row - (h - rad));
      int dx = (int)(SDL_sqrtf((float)(rad * rad - (dy + 1) * (dy + 1))) + 0.5f);

      inset = rad - dx;
    }

    {
      SDL_Rect span = { x + inset, y + row, w - 2 * inset, 1 };
      SDL_RenderFillRect(r, &span);
    }
  }
}

//-----------------------------------------------------------------------------
// The four cursor keys are petals rather than rectangles - pointed at both
// ends, widest in the middle. An ellipse is close enough at this size, and it
// is what makes the cross read as this instrument's cross.
static void petal(SDL_Renderer *r, int x, int y, int w, int h, uint32_t rgb)
{
  set_colour(r, rgb);

  for (int row = 0; row < h; row++)
  {
    float t = (2.0f * (row + 0.5f) / (float)h) - 1.0f;
    int half = (int)(w * 0.5f * SDL_sqrtf(1.0f - t * t) + 0.5f);
    SDL_Rect span = { x + w / 2 - half, y + row, 2 * half, 1 };

    if (half > 0)
      SDL_RenderFillRect(r, &span);
  }
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
  const int glyph_px = scale;
  const int hint_px  = scale > 1 ? scale - 1 : 1;

  // The case, outside in: the rubber bumper the instrument is wrapped in, the
  // dark body inside it, the recess the display sits in, and the black bezel
  // around the glass itself.
  set_colour(r, 0x1B1D1F);
  SDL_RenderClear(r);

  round_rect(r, 0, 0, PANEL_W * scale, PANEL_H * scale, 26 * scale, 0xEFC71B);
  round_rect(r, 14 * scale, 14 * scale, (PANEL_W - 28) * scale,
      (PANEL_H - 28) * scale, 16 * scale, 0x33383D);
  round_rect(r, 34 * scale, 52 * scale, (PANEL_W - 68) * scale, 292 * scale,
      6 * scale, 0x24282C);
  fill(r, (PANEL_SCREEN_X - 5) * scale, (PANEL_SCREEN_Y - 5) * scale,
      (LCD_W + 10) * scale, (LCD_H + 10) * scale, 0x0B0C0D);

  // The bandwidth marking, where the case carries it
  text(r, (PANEL_W - 66) * scale, 30 * scale, "100MHZ",
      scale > 2 ? scale - 1 : 1, 0xD6DADE);

  for (int i = 0; i < KEY_COUNT; i++)
  {
    const PanelKey *k = &g_panel[i];
    bool down = (pressed & k->btn) != 0;
    int x = k->x * scale, y = k->y * scale;
    int w = k->w * scale, h = k->h * scale;
    int sink = down ? scale : 0;
    uint32_t face, ink;

    if (k->amber)
    {
      face = down ? 0xFFE258 : 0xE3B41F;
      ink  = 0x1E1806;
    }
    else
    {
      face = down ? 0xFAFBFC : 0xC6CBD1;
      ink  = 0x17191C;
    }

    // The key stands a little proud of the case: its shadow, then its face
    if (k->arrow && !k->hint)
    {
      petal(r, x, y + 2 * scale, w, h, 0x1A1D20);
      petal(r, x, y + sink, w, h, face);
    }
    else
    {
      int rad = (k->w > k->h ? k->h : k->w) / 3 * scale;

      round_rect(r, x, y + 2 * scale, w, h, rad, 0x1A1D20);
      round_rect(r, x, y + sink, w, h, rad, face);
    }

    if (k->arrow)
    {
      int cy = y + h / 2 + sink;
      int size = (k->hint ? 5 : 7) * scale;

      triangle(r, x + w / 2, cy, size, k->arrow, ink);
    }

    if (k->label)
    {
      int tw = text_width(k->label, glyph_px);
      int ty = y + h / 2 - 4 * glyph_px + sink;

      if (k->hint)
        ty -= 3 * glyph_px;

      text(r, x + (w - tw) / 2, ty, k->label, glyph_px, ink);
    }

    // The keyboard key that does the same thing, under the label
    if (k->hint)
    {
      int hw = text_width(k->hint, hint_px);
      int hy = y + h - 4 * hint_px - 3 * scale + sink;

      text(r, x + (w - hw) / 2, hy, k->hint, hint_px,
          k->amber ? 0x6E5B17 : 0x666C72);
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
