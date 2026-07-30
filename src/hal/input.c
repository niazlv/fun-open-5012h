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

// Holding SHIFT on its own this long latches it, and it stays on until it is
// tapped again. Well clear of the 250 ms it takes auto-repeat to start, so a
// key held down deliberately still gets its repeats before anything latches.
#define SHIFT_HOLD_LOCK_MS      700
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
#define SHIFT_FLAG_H            10
#define SHIFT_FLAG_BAR_ROW      8
#define SHIFT_FLAG_COLOR        LCD_COLOR(255, 200, 0)
#define SHIFT_FLAG_REFRESH_MS   100

/*- Constants ---------------------------------------------------------------*/
// Shift arrow, one bit per pixel, MSB leftmost. The last two rows are the lock
// bar, drawn only while shift is latched: the bare arrow is one key's worth of
// shift, the underlined arrow is shift until it is switched off, and the two
// have to be told apart at a glance because they end very differently.
static const uint8_t g_shift_glyph[SHIFT_FLAG_H] =
{
  0x18, 0x3c, 0x7e, 0xff, 0x3c, 0x3c, 0x3c, 0x00,
  0xff, 0xff,
};

/*- Variables ---------------------------------------------------------------*/
static struct
{
  bool active;      // armed, waiting for the key it applies to
  bool held;        // that key is down; keep shifting it until it is released
  bool locked;      // latched by a long hold; every key is shifted until a tap
  bool down;        // SHIFT itself is down, so a repeat event is not a new tap
  bool lock_hold;   // the latch came from the hold that is still in progress
  bool no_lock;     // this hold cleared a latch and must not put one back
  uint32_t press_at;
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

/*
 * The 1X/10X tap, tracked beside the shift logic and independent of it.
 *
 * The key is the shift key on this hardware - same pin, same bit - so the
 * function printed on it had nowhere to live. A modifier is used by being HELD,
 * though, which leaves the tap free: pressed and released on its own, with
 * nothing else down and not long enough to latch, it is the legend rather than
 * the modifier. Shift itself is untouched by any of this.
 *
 * Three presses deliberately do not produce a tap. One that had another key
 * down at any point during it was a chord. One long enough to latch shift was a
 * hold. And one that RELEASED a latch was the user putting shift away, which is
 * why this runs before shift_mode_track() - afterwards the latch is already
 * gone and the press looks like any other.
 *
 * Double-click sticky shift is not one of them, and does not need to be: its
 * first tap toggles the probe, its second toggles it back, so a double click
 * arms sticky shift and leaves the probe exactly as it was.
 */
#define SHIFT_TAP_MAX_MS        400

static struct
{
  bool down;
  bool chord;
  bool was_locked;   // this press found a latch on, so it is releasing one
  bool pending;      // a clean tap has completed and not been reported yet
  uint32_t press_at;
} g_tap;

static void shift_tap_track(int buttons)
{
  uint32_t now = timer_ms();

  if (buttons & BTN_SHIFT)
  {
    if (!g_tap.down)
    {
      g_tap.down = true;
      g_tap.chord = false;
      g_tap.was_locked = g_shift.locked;
      g_tap.press_at = now;
    }

    // Any other key at any time during the press makes the whole press a
    // modifier, including the poll that reports that key being released
    if (buttons & ~(BTN_SHIFT | BTN_REPEAT))
      g_tap.chord = true;

    return;
  }

  if (!g_tap.down)
    return;

  g_tap.down = false;

  if (g_tap.chord || g_tap.was_locked ||
      (now - g_tap.press_at) >= SHIFT_TAP_MAX_MS)
    return;

  g_tap.pending = true;
}

//-----------------------------------------------------------------------------
// Two ways to get shift without holding it, and they end differently:
//
//   double click  arms it for one key, then it is gone      (shift_mode_enabled)
//   long hold     latches it until SHIFT is tapped again    (shift_hold_lock)
//
// What arrives here is the whole debounced key state, not one key event, so an
// ordinary SHIFT+key chord also has the SHIFT bit set, and so does the event
// that reports the other key being released. Counting those as taps was enough
// to arm sticky shift off a single deliberate tap that followed a chord, and
// the key after it went through shifted with nothing on screen to say why.
// Only a solo tap of SHIFT, with a release before it, counts.
static void shift_mode_track(int buttons)
{
  uint32_t now = timer_ms();

  if (!config.shift_mode_enabled && !config.shift_hold_lock)
  {
    shift_mode_reset();
    g_shift.down = false;
    g_shift.taps = 0;
    return;
  }

  if (!(buttons & BTN_SHIFT))
  {
    g_shift.down = false;
    g_shift.lock_hold = false;
    g_shift.no_lock = false;
    return;
  }

  // SHIFT held together with something else is a plain chord. It voids
  // whatever double click was in progress, and takes back a latch this same
  // hold had just produced: dwelling on SHIFT before pressing the other key is
  // how a chord is normally typed, and it must not leave shift stuck on.
  if (buttons & ~(BTN_SHIFT | BTN_REPEAT))
  {
    g_shift.down = true;
    g_shift.taps = 0;
    g_shift.no_lock = true; // and it may not latch again before it is released

    if (g_shift.lock_hold)
    {
      g_shift.locked = false;
      g_shift.lock_hold = false;
    }

    return;
  }

  // SHIFT on its own, still down - the press was seen already, this is either
  // an auto-repeat or the event that reported another key going up. Time it:
  // past the threshold the hold latches.
  if (g_shift.down)
  {
    if (config.shift_hold_lock && !g_shift.locked && !g_shift.no_lock &&
        now - g_shift.press_at >= SHIFT_HOLD_LOCK_MS)
    {
      g_shift.locked = true;
      g_shift.lock_hold = true;
      g_shift.active = false;
      g_shift.held = false;
      g_shift.taps = 0;
    }

    return;
  }

  // A new press of SHIFT, alone
  g_shift.down = true;
  g_shift.press_at = now;
  g_shift.lock_hold = false;
  g_shift.no_lock = false;

  // A latch is released by tapping SHIFT, and that tap is not also the first
  // half of a double click. no_lock keeps the same press from latching again
  // the moment it crosses the threshold.
  if (g_shift.locked)
  {
    g_shift.locked = false;
    g_shift.no_lock = true;
    g_shift.taps = 0;
    return;
  }

  if (!config.shift_mode_enabled)
    return;

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
  shift_tap_track(buttons);

  // The tap is reported on the release, which is the poll where the key state
  // is empty - so it rides out on its own and no application can mistake it
  // for a modifier on something else
  if (g_tap.pending)
  {
    g_tap.pending = false;
    buttons |= BTN_SHIFT_TAP;
  }

  buttons = key_remapping_translate(buttons);

  int keys = buttons & ~(BTN_SHIFT | BTN_REPEAT | BTN_SHIFT_TAP);

  // A latch is shift on the keyboard, not shift on the next key: everything
  // gets it, repeats included, until it is switched off again
  if (g_shift.locked)
  {
    if (keys)
      buttons |= BTN_SHIFT;

    return buttons;
  }

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
bool shift_mode_is_locked(void)
{
  return g_shift.locked;
}

//-----------------------------------------------------------------------------
void shift_mode_reset(void)
{
  g_shift.active = false;
  g_shift.held = false;
  g_shift.locked = false;
  g_shift.lock_hold = false;
}

//-----------------------------------------------------------------------------
// Blitted as pixels rather than drawn as text on purpose: lcd_set_font() and
// lcd_set_color() are global, every other drawing site leaves them set to what
// it needs and expects to find them that way next time, and this runs between
// their frames. The badge used to set the small font and a red background here
// and never put them back, which is what made the fonts fall apart everywhere.
static void shift_flag_blit(int fg, bool bar)
{
  uint16_t buf[SHIFT_FLAG_W * SHIFT_FLAG_H];

  for (int y = 0; y < SHIFT_FLAG_H; y++)
  {
    int row = (!bar && y >= SHIFT_FLAG_BAR_ROW) ? 0 : g_shift_glyph[y];

    for (int x = 0; x < SHIFT_FLAG_W; x++)
      buf[y * SHIFT_FLAG_W + x] = (row & (0x80 >> x)) ? fg : LCD_BLACK_COLOR;
  }

  lcd_draw_buf(SHIFT_FLAG_X, SHIFT_FLAG_Y, SHIFT_FLAG_W, SHIFT_FLAG_H, buf);
}

//-----------------------------------------------------------------------------
void shift_mode_task(void)
{
  static bool was_shown = false;
  static bool was_locked = false;
  static uint32_t last_draw = 0;

  bool shown = g_shift.active || g_shift.locked;

  if (shown)
  {
    uint32_t now = timer_ms();

    // Refresh at 10 Hz so the flag survives screen repaints without
    // bit-banging the LCD on every main loop pass. A change of state does not
    // wait for the tick: the bar going up is the whole feedback that the hold
    // has taken, and it has to land while the key is still down.
    if (!was_shown || g_shift.locked != was_locked ||
        now - last_draw >= SHIFT_FLAG_REFRESH_MS)
    {
      shift_flag_blit(SHIFT_FLAG_COLOR, g_shift.locked);
      last_draw = now;
    }
  }
  else if (was_shown)
  {
    // Erase to black, the same background the battery block next door paints
    // its own interior on. The slot belongs to the status corner and no screen
    // draws in it, so there is nothing underneath to restore.
    shift_flag_blit(LCD_BLACK_COLOR, false);
  }

  was_shown = shown;
  was_locked = g_shift.locked;
}
