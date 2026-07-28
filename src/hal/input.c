/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Input translation layer: key remapping and sticky shift mode
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "config.h"
#include "input.h"

/*- Definitions -------------------------------------------------------------*/
#define SHIFT_DOUBLE_CLICK_MS   500
#define REMAP_KEY_COUNT         12

// The armed marker goes in the gap the status corner leaves between the
// rightmost readout (the scope's sample rates end at x 278) and the battery
// frame at x 289, so it sits next to the charge indicator instead of on top
// of it. Amber, not red: red in this corner already means a flat battery.
#define SHIFT_FLAG_X            280
#define SHIFT_FLAG_Y            4
#define SHIFT_FLAG_W            8
#define SHIFT_FLAG_H            8
#define SHIFT_FLAG_COLOR        LCD_COLOR(255, 200, 0)
#define SHIFT_FLAG_REFRESH_MS   100

/*- Constants ---------------------------------------------------------------*/
// Shift arrow, one bit per pixel, MSB leftmost
static const uint8_t g_shift_glyph[SHIFT_FLAG_H] =
{
  0x18, 0x3c, 0x7e, 0xff, 0x3c, 0x3c, 0x3c, 0x00,
};

/*- Variables ---------------------------------------------------------------*/
static struct
{
  bool active;      // armed, waiting for the key it applies to
  bool held;        // that key is down; keep shifting it until it is released
  bool down;        // SHIFT itself is down, so a repeat event is not a new tap
  uint32_t last_tap;
  uint32_t taps;
} g_shift;

// Keys eligible for remapping; navigation and system keys are excluded
static const uint32_t g_remap_keys[REMAP_KEY_COUNT] =
{
  BTN_F1, BTN_F2, BTN_SAVE, BTN_AUTO, BTN_AC_DC, BTN_1X_10X,
  BTN_STOP, BTN_EDGE, BTN_50P, BTN_TRIG_UP, BTN_TRIG_DOWN, BTN_TRIG,
};

#define SYSTEM_KEYS_MASK  (BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT | \
    BTN_MODE | BTN_MENU | BTN_SHIFT)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void input_init(void)
{
  memset(&g_shift, 0, sizeof(g_shift));

  for (int i = 0; i < REMAP_KEY_COUNT; i++)
  {
    if (0 == config.key_mapping[i])
      config.key_mapping[i] = g_remap_keys[i];
  }
}

//-----------------------------------------------------------------------------
static uint32_t key_remapping_translate(uint32_t buttons)
{
  if (!config.key_remapping_enabled)
    return buttons;

  if (buttons & SYSTEM_KEYS_MASK)
    return buttons;

  uint32_t result = buttons;

  for (int i = 0; i < REMAP_KEY_COUNT; i++)
  {
    if (buttons & g_remap_keys[i])
    {
      result &= ~g_remap_keys[i];
      result |= config.key_mapping[i];
    }
  }

  return result;
}

//-----------------------------------------------------------------------------
// Double-click on SHIFT arms sticky shift; the next regular key press gets
// BTN_SHIFT OR-ed in and disarms it. Another double-click cancels.
//
// What arrives here is the whole debounced key state, not one key event, so an
// ordinary SHIFT+key chord also has the SHIFT bit set, and so does the event
// that reports the other key being released. Counting those as taps was enough
// to arm sticky shift off a single deliberate tap that followed a chord, and
// the key after it went through shifted with nothing on screen to say why.
// Only a solo tap of SHIFT, with a release before it, counts.
static void shift_mode_track(int buttons)
{
  if (!config.shift_mode_enabled)
  {
    g_shift.active = false;
    g_shift.held = false;
    g_shift.down = false;
    g_shift.taps = 0;
    return;
  }

  if (buttons & BTN_REPEAT)
    return; // auto-repeat of a held key, not a new press

  if (!(buttons & BTN_SHIFT))
  {
    g_shift.down = false;
    return;
  }

  // SHIFT held together with something else is a plain chord, and it voids
  // whatever double click was in progress
  if (buttons & ~(BTN_SHIFT | BTN_REPEAT))
  {
    g_shift.down = true;
    g_shift.taps = 0;
    return;
  }

  if (g_shift.down)
    return; // same press, seen again because another key went up

  g_shift.down = true;

  uint32_t now = timer_ms();

  if (g_shift.taps && now - g_shift.last_tap < SHIFT_DOUBLE_CLICK_MS)
  {
    g_shift.active = !g_shift.active;
    g_shift.taps = 0;
  }
  else
  {
    g_shift.taps = 1;
  }

  g_shift.last_tap = now;
}

//-----------------------------------------------------------------------------
int input_translate(int buttons)
{
  shift_mode_track(buttons);

  buttons = key_remapping_translate(buttons);

  int keys = buttons & ~(BTN_SHIFT | BTN_REPEAT);

  // Arming applies to one key, but for as long as that key stays down: the
  // repeats have to carry SHIFT too, or holding the key would act shifted once
  // and then unshifted, which for the arrows means changing the timebase and
  // then panning
  if (g_shift.active && keys)
  {
    g_shift.active = false;
    g_shift.held = true;
  }

  if (g_shift.held)
  {
    if (keys)
      buttons |= BTN_SHIFT;
    else
      g_shift.held = false;
  }

  return buttons;
}

//-----------------------------------------------------------------------------
bool shift_mode_is_active(void)
{
  return g_shift.active;
}

//-----------------------------------------------------------------------------
void shift_mode_reset(void)
{
  g_shift.active = false;
  g_shift.held = false;
}

//-----------------------------------------------------------------------------
// Blitted as pixels rather than drawn as text on purpose: lcd_set_font() and
// lcd_set_color() are global, every other drawing site leaves them set to what
// it needs and expects to find them that way next time, and this runs between
// their frames. The badge used to set the small font and a red background here
// and never put them back, which is what made the fonts fall apart everywhere.
static void shift_flag_blit(int fg)
{
  uint16_t buf[SHIFT_FLAG_W * SHIFT_FLAG_H];

  for (int y = 0; y < SHIFT_FLAG_H; y++)
  {
    for (int x = 0; x < SHIFT_FLAG_W; x++)
    {
      buf[y * SHIFT_FLAG_W + x] =
          (g_shift_glyph[y] & (0x80 >> x)) ? fg : LCD_BLACK_COLOR;
    }
  }

  lcd_draw_buf(SHIFT_FLAG_X, SHIFT_FLAG_Y, SHIFT_FLAG_W, SHIFT_FLAG_H, buf);
}

//-----------------------------------------------------------------------------
void shift_mode_task(void)
{
  static bool was_active = false;
  static uint32_t last_draw = 0;

  if (g_shift.active)
  {
    uint32_t now = timer_ms();

    // Refresh at 10 Hz so the flag survives screen repaints without
    // bit-banging the LCD on every main loop pass
    if (!was_active || now - last_draw >= SHIFT_FLAG_REFRESH_MS)
    {
      shift_flag_blit(SHIFT_FLAG_COLOR);
      last_draw = now;
    }
  }
  else if (was_active)
  {
    // Erase to black, the same background the battery block next door paints
    // its own interior on. The slot belongs to the status corner and no screen
    // draws in it, so there is nothing underneath to restore.
    shift_flag_blit(LCD_BLACK_COLOR);
  }

  was_active = g_shift.active;
}
