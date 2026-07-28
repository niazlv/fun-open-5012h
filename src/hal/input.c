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
#define REMAP_KEY_COUNT         11
#define KEY_MAPPING_SLOTS       ((int)(sizeof(config.key_mapping) / \
                                       sizeof(config.key_mapping[0])))

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

static bool g_capture;

// Keys eligible for remapping, in the order the editor lists them. Navigation
// and system keys are absent by construction, and that is the whole
// protection: nothing here can rewrite the keys the menus are driven with, so
// no mapping can lock the user out of the editor that made it.
//
// 1X/10X is absent for a different reason - on this hardware it IS the shift
// key. PE12 is read as BTN_1X_10X, and BTN_1X_10X and BTN_SHIFT are the same
// bit (17), so an entry for it was only ever an entry for SHIFT, and the
// system-key check below threw it out again on every press.
static const uint32_t g_remap_keys[REMAP_KEY_COUNT] =
{
  BTN_F1, BTN_F2, BTN_SAVE, BTN_AUTO, BTN_AC_DC,
  BTN_STOP, BTN_EDGE, BTN_50P, BTN_TRIG_UP, BTN_TRIG_DOWN, BTN_TRIG,
};

static const char *const g_remap_names[REMAP_KEY_COUNT] =
{
  "F1", "F2", "SAVE", "AUTO", "AC/DC",
  "STOP", "EDGE", "50%", "TRIG_UP", "TRIG_DN", "TRIG",
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int key_remap_count(void)
{
  return REMAP_KEY_COUNT;
}

//-----------------------------------------------------------------------------
const char *key_remap_name(int index)
{
  if (index < 0 || index >= REMAP_KEY_COUNT)
    return "";

  return g_remap_names[index];
}

//-----------------------------------------------------------------------------
uint32_t key_remap_button(int index)
{
  if (index < 0 || index >= REMAP_KEY_COUNT)
    return 0;

  return g_remap_keys[index];
}

//-----------------------------------------------------------------------------
int key_remap_index_of(uint32_t button)
{
  if (0 == button)
    return -1;

  for (int i = 0; i < REMAP_KEY_COUNT; i++)
  {
    if (button == g_remap_keys[i])
      return i;
  }

  return -1;
}

//-----------------------------------------------------------------------------
// A slot holds a button, KEY_MAP_NONE for a key that has been turned off, or
// zero for one that was never touched - which is the identity, so a default
// config needs nothing written into it and the first boot no longer dirties
// the settings store just by looking at the table.
uint32_t key_remap_target(int index)
{
  uint32_t value;

  if (index < 0 || index >= REMAP_KEY_COUNT)
    return 0;

  value = config.key_mapping[index];

  if (KEY_MAP_NONE == value)
    return 0;

  return value ? value : g_remap_keys[index];
}

//-----------------------------------------------------------------------------
void key_remap_set(int index, uint32_t target)
{
  if (index < 0 || index >= REMAP_KEY_COUNT)
    return;

  if (0 == target)
    config.key_mapping[index] = KEY_MAP_NONE;
  else if (key_remap_index_of(target) < 0)
    return; // only a remappable key can be a target
  else if (target == g_remap_keys[index])
    config.key_mapping[index] = 0; // itself again: back to the unset state
  else
    config.key_mapping[index] = target;
}

//-----------------------------------------------------------------------------
void key_remap_reset(void)
{
  memset(config.key_mapping, 0, sizeof(config.key_mapping));
}

//-----------------------------------------------------------------------------
bool key_remap_is_default(void)
{
  for (int i = 0; i < REMAP_KEY_COUNT; i++)
  {
    if (key_remap_target(i) != g_remap_keys[i])
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// A stored table this firmware could not have written is not read at all.
//
// The firmware before this one had a twelfth entry - 1X/10X, the shift key's
// own bit - and filled every slot with the identity on first boot. Read
// against this table, its entries are one place out from index five down, and
// EDGE would have come up meaning STOP. Nothing but the identity could ever be
// stored back then, so recognising the old table and clearing it loses
// nothing: what it meant was "no remapping", and that is what zero means here.
static bool mapping_is_valid(void)
{
  for (int i = 0; i < REMAP_KEY_COUNT; i++)
  {
    uint32_t value = config.key_mapping[i];

    if (0 == value || KEY_MAP_NONE == value)
      continue;

    if (key_remap_index_of(value) < 0)
      return false;
  }

  // Slots past the table belong to no key, so anything in one came from a
  // firmware whose table was longer
  for (int i = REMAP_KEY_COUNT; i < KEY_MAPPING_SLOTS; i++)
  {
    if (config.key_mapping[i])
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
void input_init(void)
{
  memset(&g_shift, 0, sizeof(g_shift));
  g_capture = false;

  if (!mapping_is_valid())
    key_remap_reset();
}

//-----------------------------------------------------------------------------
void input_capture_set(bool enable)
{
  g_capture = enable;

  if (enable)
    shift_mode_reset();
}

//-----------------------------------------------------------------------------
// Every remappable bit is cleared first and the targets are OR-ed in after, so
// the result does not depend on the order of the table. Rewriting the bits in
// place did: a key mapped onto another key's button had that bit cleared again
// when the other key's turn came, and a swap of two keys held together came
// out as one of them.
//
// System keys are not in the table and so are never rewritten - and a chord
// holding one now remaps the rest of it, which is what makes SHIFT+F1 follow
// F1 wherever it was sent. Bailing out on the whole chord instead meant a
// remapped key quietly reverted for as long as SHIFT was down.
static uint32_t key_remapping_translate(uint32_t buttons)
{
  uint32_t result = buttons;

  if (!config.key_remapping_enabled)
    return buttons;

  for (int i = 0; i < REMAP_KEY_COUNT; i++)
    result &= ~g_remap_keys[i];

  for (int i = 0; i < REMAP_KEY_COUNT; i++)
  {
    if (buttons & g_remap_keys[i])
      result |= key_remap_target(i);
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
  // A capture reads the keyboard, not what the keyboard currently means:
  // neither the mapping being edited nor a sticky shift may touch it
  if (g_capture)
    return buttons;

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
