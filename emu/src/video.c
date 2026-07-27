/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The window, the keyboard, and PNG capture of the panel.
 *
 * The framebuffer this shows is the one the ST7789 model holds, so a
 * screenshot is the pixels the firmware actually pushed over the bus - not a
 * re-rendering of them. Headless mode skips the window entirely, which is what
 * makes the screenshot script runnable in CI.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zlib.h>

#include "emu.h"

#ifndef EMU_NO_SDL
#include <SDL.h>
#endif

/*- Variables ---------------------------------------------------------------*/
static bool g_headless = true;
static int g_scale = 3;

#ifndef EMU_NO_SDL
static SDL_Window *g_window;
static SDL_Renderer *g_renderer;
static SDL_Texture *g_texture;
#endif

/* The panel keys, laid out the way the instrument's front panel is. The bit
 * numbers are buttons.h's BTN_* values. */
typedef struct { int key; uint32_t btn; const char *label; } KeyMap;

#ifndef EMU_NO_SDL
static const KeyMap g_keys[] =
{
  { SDLK_SPACE,     1u << 0,  "STOP/RUN"  },
  { SDLK_UP,        1u << 1,  "UP"        },
  { SDLK_e,         1u << 2,  "EDGE"      },
  { SDLK_m,         1u << 3,  "MODE"      },
  { SDLK_F1,        1u << 4,  "F1"        },
  { SDLK_RIGHT,     1u << 5,  "RIGHT"     },
  { SDLK_c,         1u << 6,  "AC/DC"     },
  { SDLK_a,         1u << 7,  "AUTO"      },
  { SDLK_5,         1u << 8,  "50%"       },
  { SDLK_RETURN,    1u << 9,  "MENU"      },
  { SDLK_PAGEUP,    1u << 10, "TRIG UP"   },
  { SDLK_LEFT,      1u << 11, "LEFT"      },
  { SDLK_PAGEDOWN,  1u << 12, "TRIG DOWN" },
  { SDLK_s,         1u << 13, "SAVE"      },
  { SDLK_t,         1u << 14, "TRIG"      },
  { SDLK_DOWN,      1u << 15, "DOWN"      },
  { SDLK_F2,        1u << 16, "F2"        },
  { SDLK_LSHIFT,    1u << 17, "SHIFT"     },
  { SDLK_RSHIFT,    1u << 17, "SHIFT"     },
};
#endif

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
bool video_init(int scale, bool headless, const char *title)
{
  g_headless = headless;
  g_scale = scale > 0 ? scale : 3;

#ifdef EMU_NO_SDL
  (void)title;
  g_headless = true;
  return true;
#else
  if (g_headless)
    return true;

  if (SDL_Init(SDL_INIT_VIDEO) != 0)
  {
    emu_log("SDL_Init: %s - falling back to headless", SDL_GetError());
    g_headless = true;
    return true;
  }

  g_window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED, LCD_W * g_scale, LCD_H * g_scale, 0);

  if (!g_window)
  {
    emu_log("SDL_CreateWindow: %s", SDL_GetError());
    g_headless = true;
    return true;
  }

  g_renderer = SDL_CreateRenderer(g_window, -1, SDL_RENDERER_ACCELERATED);
  g_texture = SDL_CreateTexture(g_renderer, SDL_PIXELFORMAT_RGB565,
      SDL_TEXTUREACCESS_STREAMING, LCD_W, LCD_H);

  return true;
#endif
}

//-----------------------------------------------------------------------------
bool video_pump(void)
{
#ifndef EMU_NO_SDL
  SDL_Event ev;
  uint32_t buttons;

  if (g_headless)
    return true;

  buttons = board_get_buttons();

  while (SDL_PollEvent(&ev))
  {
    if (ev.type == SDL_QUIT)
      return false;

    if (ev.type == SDL_KEYDOWN || ev.type == SDL_KEYUP)
    {
      bool down = (ev.type == SDL_KEYDOWN);

      if (down && ev.key.keysym.sym == SDLK_ESCAPE)
        return false;

      if (down && ev.key.keysym.sym == SDLK_F12)
      {
        char path[64];
        static int n = 0;
        snprintf(path, sizeof(path), "screenshot-%03d.png", n++);
        if (video_save_png(path, 2))
          emu_log("wrote %s", path);
        continue;
      }

      for (unsigned i = 0; i < sizeof(g_keys) / sizeof(g_keys[0]); i++)
      {
        if (g_keys[i].key == ev.key.keysym.sym)
        {
          if (down)
            buttons |= g_keys[i].btn;
          else
            buttons &= ~g_keys[i].btn;
        }
      }
    }
  }

  board_set_buttons(buttons);
#endif

  return true;
}

//-----------------------------------------------------------------------------
void video_present(void)
{
#ifndef EMU_NO_SDL
  if (g_headless)
    return;

  SDL_UpdateTexture(g_texture, NULL, st7789_fb, LCD_W * (int)sizeof(uint16_t));
  SDL_RenderClear(g_renderer);
  SDL_RenderCopy(g_renderer, g_texture, NULL, NULL);
  SDL_RenderPresent(g_renderer);
#endif
}

//-----------------------------------------------------------------------------
void video_shutdown(void)
{
#ifndef EMU_NO_SDL
  if (g_headless)
    return;

  SDL_DestroyTexture(g_texture);
  SDL_DestroyRenderer(g_renderer);
  SDL_DestroyWindow(g_window);
  SDL_Quit();
#endif
}

/*- PNG ---------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void put_be32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

//-----------------------------------------------------------------------------
static void png_chunk(FILE *f, const char *type, const uint8_t *data, size_t len)
{
  uint8_t hdr[8];
  uint32_t crc;

  put_be32(hdr, (uint32_t)len);
  memcpy(hdr + 4, type, 4);
  fwrite(hdr, 1, 8, f);

  if (len)
    fwrite(data, 1, len, f);

  crc = (uint32_t)crc32(0, (const Bytef *)type, 4);

  if (len)
    crc = (uint32_t)crc32(crc, (const Bytef *)data, (uInt)len);

  put_be32(hdr, crc);
  fwrite(hdr, 1, 4, f);
}

//-----------------------------------------------------------------------------
// Writes the panel exactly as it stands, expanded from RGB565 to 8-bit RGB and
// nearest-neighbour scaled so the pixel grid stays crisp in the README.
bool video_save_png(const char *path, int scale)
{
  const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n' };
  int w = LCD_W * scale, h = LCD_H * scale;
  size_t raw_len = (size_t)h * (1 + (size_t)w * 3);
  uint8_t *raw, *ihdr_buf, *comp;
  uLongf comp_len;
  FILE *f;

  if (scale < 1)
    scale = 1;

  raw = malloc(raw_len);

  if (!raw)
    return false;

  {
    uint8_t *p = raw;

    for (int y = 0; y < h; y++)
    {
      *p++ = 0; // filter type: none

      for (int x = 0; x < w; x++)
      {
        uint16_t c = st7789_fb[(y / scale) * LCD_W + (x / scale)];
        int r = (c >> 11) & 0x1f, g = (c >> 5) & 0x3f, b = c & 0x1f;

        // 5/6-bit to 8-bit with the top bits replicated, so full-scale
        // stays full-scale
        *p++ = (uint8_t)((r << 3) | (r >> 2));
        *p++ = (uint8_t)((g << 2) | (g >> 4));
        *p++ = (uint8_t)((b << 3) | (b >> 2));
      }
    }
  }

  comp_len = compressBound((uLong)raw_len);
  comp = malloc(comp_len);

  if (!comp || compress2(comp, &comp_len, raw, (uLong)raw_len, 9) != Z_OK)
  {
    free(raw);
    free(comp);
    return false;
  }

  free(raw);

  f = fopen(path, "wb");

  if (!f)
  {
    free(comp);
    return false;
  }

  fwrite(sig, 1, sizeof(sig), f);

  ihdr_buf = malloc(13);
  put_be32(ihdr_buf, (uint32_t)w);
  put_be32(ihdr_buf + 4, (uint32_t)h);
  ihdr_buf[8] = 8;    // bit depth
  ihdr_buf[9] = 2;    // colour type: truecolour
  ihdr_buf[10] = 0;
  ihdr_buf[11] = 0;
  ihdr_buf[12] = 0;
  png_chunk(f, "IHDR", ihdr_buf, 13);
  png_chunk(f, "IDAT", comp, comp_len);
  png_chunk(f, "IEND", NULL, 0);

  fclose(f);
  free(ihdr_buf);
  free(comp);

  return true;
}
