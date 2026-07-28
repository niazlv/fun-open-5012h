/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * System menu: the popup opened with the MENU key over whatever is running.
 *
 * The menu is assembled when it opens: the running application's own settings
 * come first (the application owns that table, see launcher_app_menu()),
 * followed by the settings that belong to the device itself. Nothing in here
 * is application specific.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "config.h"
#include "common.h"
#include "utils.h"
#include "scope.h"
#include "ui.h"
#include "menu_widget.h"
#include "input.h"
#include "launcher.h"
#include "system_menu.h"

/*- Definitions -------------------------------------------------------------*/
// Hardware register addresses
#define DBG_ID_REG        0xE0042000 // Debug ID register
#define FLASH_SIZE_REG    0x1FFF7A22 // Flash size register (16-bit, KB)
#define UID_REG_BASE      0x1FFF7A10 // Unique ID base address
#define FLASH_MEM_BASE    0x08000000
#define TCM_MEM_BASE      0x10000000

#define TCM_SIZE_KB       64
#define SRAM_SIZE_KB      128

// A popup has no scrolling, so the assembled menu has to fit on the screen:
// 11 rows x 20 px + margins = 224 px, opened at y = 10. MAIN_FIXED counts the
// rows this file always contributes (3 submenus, 2 separators, Exit); the
// rest is the budget for the application's own rows.
#define MAIN_MAX_ITEMS    11
#define MAIN_FIXED        6
#define MAIN_APP_BUDGET   (MAIN_MAX_ITEMS - MAIN_FIXED)

// Help: the application's own pages, then the ones that belong to the device.
// HELP_FIXED is the separator plus Key Bindings.
#define HELP_MAX_ITEMS    11
#define HELP_FIXED        2
#define HELP_APP_BUDGET   (HELP_MAX_ITEMS - HELP_FIXED)

// Linker script symbols for memory usage calculation
extern uint32_t _etext;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;
extern uint32_t _end;
extern uint32_t __etext; // end of all flash data, including .data image

/*- Variables ---------------------------------------------------------------*/
static menu_item_t g_main_items[MAIN_MAX_ITEMS];
static menu_def_t g_main_menu;

// Assembled next to the main menu, and alive for as long as it is: the Help
// row points straight at this array
static menu_item_t g_help_items[HELP_MAX_ITEMS];
static int g_help_count;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Firmware hash: CRC32 (IEEE 802.3) over everything in flash
//-----------------------------------------------------------------------------
static uint32_t crc32_ieee(const uint8_t *data, uint32_t length)
{
  uint32_t crc = 0xFFFFFFFF;

  for (uint32_t i = 0; i < length; i++)
  {
    crc ^= data[i];

    for (int j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xEDB88320;
      else
        crc >>= 1;
    }
  }

  return ~crc;
}

//-----------------------------------------------------------------------------
static uint32_t calculate_firmware_hash(void)
{
  uint32_t flash_used = (uint32_t)&__etext - FLASH_MEM_BASE;

  return crc32_ieee((uint8_t *)FLASH_MEM_BASE, flash_used);
}

//-----------------------------------------------------------------------------
// Info page bodies: the frame, font and colors are already set up by the
// info page, these only add the computed lines
//-----------------------------------------------------------------------------
static void body_device_info(void)
{
  char buf[80];
  char mcu_buf[32];

  uint32_t device_id = *(uint32_t *)DBG_ID_REG;
  uint16_t dev_id = device_id & 0xFFF;
  uint16_t rev_id = (device_id >> 16) & 0xFFFF;

  const char *mcu_name;

  switch (dev_id)
  {
    case 0x413: mcu_name = "GD32F407VE"; break;
    case 0x414: mcu_name = "GD32F405/407"; break;
    default:
      snprintf(mcu_buf, sizeof(mcu_buf), "Unknown GD32 (0x%03X)", dev_id);
      mcu_name = mcu_buf;
      break;
  }

  snprintf(buf, sizeof(buf), "MCU: %s (ARM Cortex-M4)", mcu_name);
  lcd_puts(INFO_X, 45, buf);

  sprintf(buf, "DevID: 0x%03X, Rev: 0x%04X", dev_id, rev_id);
  lcd_puts(INFO_X, 60, buf);

  uint16_t flash_size = *(uint16_t *)FLASH_SIZE_REG;

  if (0xFFFF == flash_size || 0 == flash_size)
    flash_size = 512;

  sprintf(buf, "Flash: %u KB, SRAM: %u KB + TCM: %u KB",
      flash_size, SRAM_SIZE_KB, TCM_SIZE_KB);
  lcd_puts(INFO_X, 75, buf);

  sprintf(buf, "CPU Freq: %lu MHz", F_CPU / 1000000UL);
  lcd_puts(INFO_X, 90, buf);

  uint32_t uid0 = *(uint32_t *)(UID_REG_BASE + 0);
  uint32_t uid1 = *(uint32_t *)(UID_REG_BASE + 4);
  uint32_t uid2 = *(uint32_t *)(UID_REG_BASE + 8);
  sprintf(buf, "UID: %08lX-%08lX-%08lX", uid0, uid1, uid2);
  lcd_puts(INFO_X, 105, buf);

#ifdef FIRMWARE_NAME
  sprintf(buf, "%s v%s", FIRMWARE_NAME, FIRMWARE_VERSION);
  lcd_puts(INFO_X, 120, buf);
#endif

#ifdef BUILD_DATE
  sprintf(buf, "Built: %s %s", BUILD_DATE, BUILD_TIME);
#else
  sprintf(buf, "Built: %s %s", __DATE__, __TIME__);
#endif
  lcd_puts(INFO_X, 135, buf);

#ifdef GIT_HASH
  sprintf(buf, "Git: %s", GIT_HASH);
  lcd_puts(INFO_X, 150, buf);
#endif

  sprintf(buf, "FW Hash(CRC32): %08lX", calculate_firmware_hash());
  lcd_puts(INFO_X, 165, buf);
}

//-----------------------------------------------------------------------------
static void body_system_info(void)
{
  char buf[80];

  uint32_t flash_used = (uint32_t)&_etext - FLASH_MEM_BASE;
  uint32_t data_size = (uint32_t)&_edata - (uint32_t)&_data;
  uint32_t bss_size = (uint32_t)&_ebss - (uint32_t)&_bss;
  uint32_t ram_used = (uint32_t)&_end - TCM_MEM_BASE;

  uint16_t flash_total = *(uint16_t *)FLASH_SIZE_REG;

  if (0xFFFF == flash_total || 0 == flash_total)
    flash_total = 512;

  // Percentages in integer tenths: newlib's %f pulls in the malloc-backed
  // _dtoa path, and the heap shares scarce TCM with the stack
  uint32_t flash_pct10 = flash_used * 1000ul / (flash_total * 1024ul);
  uint32_t ram_pct10 = ram_used * 1000ul / (TCM_SIZE_KB * 1024ul);

  sprintf(buf, "Flash: %lu KB / %u KB (%lu.%lu%%)",
      flash_used / 1024, flash_total, flash_pct10 / 10, flash_pct10 % 10);
  lcd_puts(INFO_X, 45, buf);

  sprintf(buf, "TCM RAM: %lu KB / %u KB (%lu.%lu%%)",
      ram_used / 1024, TCM_SIZE_KB, ram_pct10 / 10, ram_pct10 % 10);
  lcd_puts(INFO_X, 60, buf);

  sprintf(buf, "Data: %lu B, BSS: %lu B", data_size, bss_size);
  lcd_puts(INFO_X, 75, buf);

  sprintf(buf, "CPU Freq: %lu MHz", F_CPU / 1000000UL);
  lcd_puts(INFO_X, 90, buf);

  uint32_t uptime_ms = timer_ms();
  uint32_t uptime_sec = uptime_ms / 1000;
  uint32_t hours = uptime_sec / 3600;
  uint32_t minutes = (uptime_sec % 3600) / 60;
  uint32_t seconds = uptime_sec % 60;

  if (hours > 0)
    sprintf(buf, "Uptime: %luh %lum %lus", hours, minutes, seconds);
  else if (minutes > 0)
    sprintf(buf, "Uptime: %lum %lus", minutes, seconds);
  else
    sprintf(buf, "Uptime: %lu.%03lus", seconds, uptime_ms % 1000);
  lcd_puts(INFO_X, 105, buf);

#ifdef GIT_HASH
  sprintf(buf, "Git: %s (%s)", GIT_HASH, GIT_BRANCH);
  lcd_puts(INFO_X, 120, buf);
#endif

#ifdef GIT_USER
  sprintf(buf, "Built by: %s", GIT_USER);
  lcd_puts(INFO_X, 135, buf);
#endif

  // Worst main-loop stall since this dialog was last opened: the practical
  // "is it responsive" metric (a button press can wait at most this long)
  sprintf(buf, "Loop max: %d ms   Scope FPS: %d",
      timer_get_max_delta(), scope_get_fps());
  lcd_puts(INFO_X, 150, buf);

  // Measurements panel state, for diagnosing a blank one: d=enabled,
  // m=panel/rotate, act=composited into the sweep, t=update timer,
  // bld/pnt=texts built and bands repainted, len=length of the current text
  scope_get_panel_state(buf, sizeof(buf));
  lcd_puts(INFO_X, 165, buf);

  // Settings store health. "saved" with a rising entry number is a store that
  // is persisting; "NO STORED CONFIG" on a device that has been used before
  // means the last session's settings were lost on the way to flash.
  config_get_state(buf, sizeof(buf));
  lcd_puts(INFO_X, 180, buf);
}

//-----------------------------------------------------------------------------
// Info pages
//-----------------------------------------------------------------------------
static const char *const g_key_bindings_lines[] =
{
  INFO_HEAD "Menus",
  "UP/DOWN     - Navigate",
  "LEFT/RIGHT  - Adjust / enter submenu",
  "MODE        - Select / confirm",
  "MENU        - System menu over any application",
  "SHIFT+MENU  - Leave the application",
  "SHIFT x2    - Sticky shift (if enabled)",
  "",
  "The rows above the first separator belong to",
  "the running application. Its keys, and every",
  "other read-only page, are under Help.",
  "",
  INFO_HEAD "Text pages",
  "UP/DOWN     - One line",
  "LEFT/RIGHT  - One screen",
  "TRIG_UP/DN  - Previous / next section",
  "MODE        - Close, and so does any other key",
  "",
  "A page that fits on the screen has nothing to",
  "scroll and closes on any key at all.",
  "",
  INFO_HEAD "Remappable keys",
  "  F1 F2 SAVE AUTO AC/DC 1X/10X",
  "  STOP EDGE 50% TRIG TRIG_UP/DOWN",
  "",
  INFO_HEAD "Protected system keys",
  "  UP DOWN LEFT RIGHT MODE MENU SHIFT",
};

static const info_page_t g_page_key_bindings =
{
  .title = "Key Bindings",
  .lines = g_key_bindings_lines,
  .count = ARRAY_SIZE(g_key_bindings_lines),
};

static const info_page_t g_page_device_info =
{
  .title = "Device Information",
  .body = body_device_info,
};

static const info_page_t g_page_system_info =
{
  .title = "System Information",
  .body = body_system_info,
};

//-----------------------------------------------------------------------------
// Menu actions
//-----------------------------------------------------------------------------
static void action_exit_app(const void *arg)
{
  (void)arg;
  ui_pop_to_root();
}

//-----------------------------------------------------------------------------
static void action_reboot(const void *arg)
{
  (void)arg;

  // The settings store saves on a 1 s timer and then takes a few passes of
  // the main loop to finish writing. Resetting straight from the menu threw
  // both away, so a reboot was the one action guaranteed to lose the setting
  // the user had just changed.
  config_flush();

  NVIC_SystemReset();
}

//-----------------------------------------------------------------------------
static void apply_backlight(int value)
{
  lcd_set_backlight_level(value);
}

//-----------------------------------------------------------------------------
static void shift_mode_changed(void)
{
  if (!config.shift_mode_enabled)
    shift_mode_reset();
}

//-----------------------------------------------------------------------------
// Menu tables (const, in flash). config changes are persisted automatically
// by config_task once the struct CRC goes stale.
//-----------------------------------------------------------------------------
static const menu_item_t g_general_items[] =
{
  { .kind = MI_NUMBER, .label = "Brightness",
    .u.number = { &config.lcd_bl_level, 10, 100, 5, 10, "%", apply_backlight } },
  { .kind = MI_TOGGLE, .label = "Shift Mode",
    .u.toggle = { &config.shift_mode_enabled, shift_mode_changed } },
  { .kind = MI_TOGGLE, .label = "Key Remap",
    .u.toggle = { &config.key_remapping_enabled, NULL } },
};

static const menu_item_t g_advanced_items[] =
{
  { .kind = MI_ACTION, .label = "Device Info",
    .u.action = { menu_action_info, &g_page_device_info } },
  { .kind = MI_ACTION, .label = "System Info",
    .u.action = { menu_action_info, &g_page_system_info } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "Reboot System",
    .u.action = { action_reboot, NULL } },
};

//-----------------------------------------------------------------------------
// Help: one section for every read-only page in the firmware.
//
// Applications keep their text out of their own settings tables and hand it
// over as a second table (launcher_app_help), so a page is always one place
// away no matter which application is running: MENU > Help. The running
// application's pages come first, the device's own below the separator.
//-----------------------------------------------------------------------------
static void help_menu_build(void)
{
  const menu_def_t *app_help = launcher_app_help();
  int n = 0;

  if (app_help && app_help->items)
  {
    for (int i = 0; i < app_help->count && n < HELP_APP_BUDGET; i++)
      g_help_items[n++] = app_help->items[i];
  }

  if (n > 0)
    g_help_items[n++] = (menu_item_t){ .kind = MI_SEPARATOR };

  g_help_items[n++] = (menu_item_t)
  {
    .kind = MI_ACTION, .label = "Key Bindings",
    .u.action = { menu_action_info, &g_page_key_bindings },
  };

  g_help_count = n;
}

//-----------------------------------------------------------------------------
void system_menu_open(void)
{
  const menu_def_t *app_menu = launcher_app_menu();
  int n = 0;

  help_menu_build();

  // The running application's own settings come first: spliced in directly
  // when they fit, otherwise nested under the application name
  if (app_menu && app_menu->count > 0)
  {
    if (app_menu->count <= MAIN_APP_BUDGET)
    {
      for (int i = 0; i < app_menu->count; i++)
        g_main_items[n++] = app_menu->items[i];
    }
    else
    {
      g_main_items[n++] = (menu_item_t)
      {
        .kind = MI_SUBMENU,
        .label = app_menu->title ? app_menu->title : launcher_app_name(),
        .u.submenu = { app_menu->items, app_menu->count },
      };
    }

    g_main_items[n++] = (menu_item_t){ .kind = MI_SEPARATOR };
  }

  g_main_items[n++] = (menu_item_t)
  {
    .kind = MI_SUBMENU, .label = "General Settings",
    .u.submenu = { g_general_items, ARRAY_SIZE(g_general_items) },
  };

  g_main_items[n++] = (menu_item_t)
  {
    .kind = MI_SUBMENU, .label = "Advanced",
    .u.submenu = { g_advanced_items, ARRAY_SIZE(g_advanced_items) },
  };

  g_main_items[n++] = (menu_item_t)
  {
    .kind = MI_SUBMENU, .label = "Help",
    .u.submenu = { g_help_items, g_help_count },
  };

  if (launcher_app_running())
  {
    g_main_items[n++] = (menu_item_t){ .kind = MI_SEPARATOR };
    g_main_items[n++] = (menu_item_t)
    {
      .kind = MI_ACTION, .label = "Exit Application",
      .u.action = { action_exit_app, NULL },
    };
  }

  g_main_menu = (menu_def_t){ .items = g_main_items, .count = n };

  menu_open_popup(&g_main_menu, 10, 10);
}
