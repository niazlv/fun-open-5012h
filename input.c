/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
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

/*- Variables ---------------------------------------------------------------*/
static struct
{
  bool active;
  uint32_t last_press;
  uint32_t press_count;
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
static void shift_mode_track(int buttons)
{
  if (!config.shift_mode_enabled)
  {
    g_shift.active = false;
    return;
  }

  if ((buttons & BTN_SHIFT) && !(buttons & BTN_REPEAT))
  {
    uint32_t now = timer_ms();

    if (now - g_shift.last_press < SHIFT_DOUBLE_CLICK_MS)
    {
      if (++g_shift.press_count >= 2)
      {
        g_shift.active = !g_shift.active;
        g_shift.press_count = 0;
      }
    }
    else
    {
      g_shift.press_count = 1;
    }

    g_shift.last_press = now;
  }
}

//-----------------------------------------------------------------------------
int input_translate(int buttons)
{
  shift_mode_track(buttons);

  buttons = key_remapping_translate(buttons);

  if (g_shift.active && (buttons & ~(BTN_SHIFT | BTN_REPEAT)))
  {
    buttons |= BTN_SHIFT;

    if (!(buttons & BTN_REPEAT))
      g_shift.active = false;
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
}

//-----------------------------------------------------------------------------
void shift_mode_task(void)
{
  static bool was_active = false;
  static uint32_t last_draw = 0;

  if (g_shift.active)
  {
    uint32_t now = timer_ms();

    // Refresh at 10 Hz so the badge survives screen repaints without
    // bit-banging the LCD on every main loop pass
    if (!was_active || now - last_draw >= 100)
    {
      lcd_fill_rect(LCD_WIDTH - 50, 5, 45, 15, LCD_COLOR(255, 0, 0));
      lcd_set_color(LCD_COLOR(255, 0, 0), LCD_COLOR(255, 255, 255));
      lcd_set_font(FONT_SMALL);
      lcd_puts(LCD_WIDTH - 45, 8, "SHIFT");
      last_draw = now;
    }
  }
  else if (was_active)
  {
    // Erase the badge; the area is repainted by whatever draws next
    lcd_fill_rect(LCD_WIDTH - 50, 5, 45, 15, LCD_BLACK_COLOR);
  }

  was_active = g_shift.active;
}
