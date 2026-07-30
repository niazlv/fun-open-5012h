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
#include "flash.h"
#include "battery.h"
#include "common.h"
#include "utils.h"
#include "scope.h"
#include "ui.h"
#include "menu_widget.h"
#include "input.h"
#include "key_remap.h"
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
// Info page content.
//
// Every page here renders ONE line at a time, on request, rather than painting
// itself at coordinates it picked. That is not a style choice: a page that
// draws itself has to know where the bottom of the panel is, and it silently
// loses whatever it puts below it. This file had three of them, and the
// System Information page was already one line off the edge - the next field
// anybody added to it would simply not have appeared.
//
// Handing lines back instead gives the widget what it needs to lay the page
// out: it picks the roomy grid when the content fits, the tight one when it
// does not, and adds scrolling and a scrollbar past that. A page can now grow
// without anyone having to check arithmetic against INFO_BODY_BOTTOM.
//
// The shared buffer only has to survive until the next call, which is the
// contract info_page_t::line_fn states.
//-----------------------------------------------------------------------------
static char g_info_line[80];

//-----------------------------------------------------------------------------
// Line numbering, with the conditional rows in place. An #ifdef'd entry
// removes itself from the count as well as from the switch, so the two cannot
// disagree about how long the page is.
enum
{
  DI_MCU, DI_DEVID, DI_MEM, DI_CPU, DI_UID,
#ifdef FIRMWARE_NAME
  DI_FIRMWARE,
#endif
  DI_BUILT,
#ifdef GIT_HASH
  DI_GIT,
#endif
  DI_HASH,
  DI_COUNT,
};

static const char *device_info_line(int index)
{
  char *buf = g_info_line;
  int size = sizeof(g_info_line);

  switch (index)
  {
    case DI_MCU:
    {
      uint32_t device_id = *(uint32_t *)DBG_ID_REG;
      uint16_t dev_id = device_id & 0xFFF;

      switch (dev_id)
      {
        case 0x413:
          snprintf(buf, size, "MCU: GD32F407VE (ARM Cortex-M4)");
          break;

        case 0x414:
          snprintf(buf, size, "MCU: GD32F405/407 (ARM Cortex-M4)");
          break;

        default:
          snprintf(buf, size, "MCU: Unknown GD32 (0x%03X) (ARM Cortex-M4)",
              dev_id);
          break;
      }

      break;
    }

    case DI_DEVID:
    {
      uint32_t device_id = *(uint32_t *)DBG_ID_REG;

      snprintf(buf, size, "DevID: 0x%03X, Rev: 0x%04X",
          (unsigned)(device_id & 0xFFF), (unsigned)((device_id >> 16) & 0xFFFF));
      break;
    }

    case DI_MEM:
    {
      uint16_t flash_size = *(uint16_t *)FLASH_SIZE_REG;

      if (0xFFFF == flash_size || 0 == flash_size)
        flash_size = 512;

      snprintf(buf, size, "Flash: %u KB, SRAM: %u KB + TCM: %u KB",
          flash_size, SRAM_SIZE_KB, TCM_SIZE_KB);
      break;
    }

    case DI_CPU:
      snprintf(buf, size, "CPU Freq: %lu MHz", F_CPU / 1000000UL);
      break;

    case DI_UID:
      snprintf(buf, size, "UID: %08lX-%08lX-%08lX",
          *(uint32_t *)(UID_REG_BASE + 0),
          *(uint32_t *)(UID_REG_BASE + 4),
          *(uint32_t *)(UID_REG_BASE + 8));
      break;

#ifdef FIRMWARE_NAME
    case DI_FIRMWARE:
      snprintf(buf, size, "%s v%s", FIRMWARE_NAME, FIRMWARE_VERSION);
      break;
#endif

    case DI_BUILT:
#ifdef BUILD_DATE
      snprintf(buf, size, "Built: %s %s", BUILD_DATE, BUILD_TIME);
#else
      snprintf(buf, size, "Built: %s %s", __DATE__, __TIME__);
#endif
      break;

#ifdef GIT_HASH
    case DI_GIT:
      snprintf(buf, size, "Git: %s", GIT_HASH);
      break;
#endif

    case DI_HASH:
      snprintf(buf, size, "FW Hash(CRC32): %08lX", calculate_firmware_hash());
      break;

    default:
      return "";
  }

  return buf;
}

//-----------------------------------------------------------------------------
enum
{
  SI_FLASH, SI_TCM, SI_DATA, SI_CPU, SI_UPTIME,
#ifdef GIT_HASH
  SI_GIT,
#endif
#ifdef GIT_USER
  SI_BUILT_BY,
#endif
  SI_LOOP, SI_PANEL, SI_BATTERY, SI_STORE, SI_CALIB, SI_SPI, SI_SPI_HEAD,
  SI_COUNT,
};

static const char *system_info_line(int index)
{
  char *buf = g_info_line;
  int size = sizeof(g_info_line);

  uint16_t flash_total = *(uint16_t *)FLASH_SIZE_REG;

  if (0xFFFF == flash_total || 0 == flash_total)
    flash_total = 512;

  switch (index)
  {
    case SI_FLASH:
    {
      uint32_t flash_used = (uint32_t)&_etext - FLASH_MEM_BASE;

      // Percentages in integer tenths: newlib's %f pulls in the malloc-backed
      // _dtoa path, and the heap shares scarce TCM with the stack
      uint32_t pct10 = flash_used * 1000ul / (flash_total * 1024ul);

      snprintf(buf, size, "Flash: %lu KB / %u KB (%lu.%lu%%)",
          flash_used / 1024, flash_total, pct10 / 10, pct10 % 10);
      break;
    }

    case SI_TCM:
    {
      uint32_t ram_used = (uint32_t)&_end - TCM_MEM_BASE;
      uint32_t pct10 = ram_used * 1000ul / (TCM_SIZE_KB * 1024ul);

      snprintf(buf, size, "TCM RAM: %lu KB / %u KB (%lu.%lu%%)",
          ram_used / 1024, TCM_SIZE_KB, pct10 / 10, pct10 % 10);
      break;
    }

    case SI_DATA:
      snprintf(buf, size, "Data: %lu B, BSS: %lu B",
          (uint32_t)&_edata - (uint32_t)&_data,
          (uint32_t)&_ebss - (uint32_t)&_bss);
      break;

    case SI_CPU:
      snprintf(buf, size, "CPU Freq: %lu MHz", F_CPU / 1000000UL);
      break;

    case SI_UPTIME:
    {
      uint32_t uptime_ms = timer_ms();
      uint32_t uptime_sec = uptime_ms / 1000;
      uint32_t hours = uptime_sec / 3600;
      uint32_t minutes = (uptime_sec % 3600) / 60;
      uint32_t seconds = uptime_sec % 60;

      if (hours > 0)
        snprintf(buf, size, "Uptime: %luh %lum %lus", hours, minutes, seconds);
      else if (minutes > 0)
        snprintf(buf, size, "Uptime: %lum %lus", minutes, seconds);
      else
        snprintf(buf, size, "Uptime: %lu.%03lus", seconds, uptime_ms % 1000);

      break;
    }

#ifdef GIT_HASH
    case SI_GIT:
      snprintf(buf, size, "Git: %s (%s)", GIT_HASH, GIT_BRANCH);
      break;
#endif

#ifdef GIT_USER
    case SI_BUILT_BY:
      snprintf(buf, size, "Built by: %s", GIT_USER);
      break;
#endif

    // Worst main-loop stall since this dialog was last opened: the practical
    // "is it responsive" metric (a button press can wait at most this long)
    case SI_LOOP:
      snprintf(buf, size, "Loop max: %d ms   Scope FPS: %d",
          timer_get_max_delta(), scope_get_fps());
      break;

    // Measurements panel state, for diagnosing a blank one: d=enabled,
    // m=panel/rotate, act=composited into the sweep, t=update timer,
    // bld/pnt=texts built and bands repainted, len=length of the current text
    case SI_PANEL:
      scope_get_panel_state(buf, size);
      break;

    // Charge, cell voltage and what the charger pin says. The icon in the
    // corner is 16 pixels wide and cannot be read to better than 6 %; this is
    // the same reading in figures, and Advanced > Battery breaks it down.
    case SI_BATTERY:
      battery_get_state(buf, size);
      break;

    // Settings store health. "saved" with a rising entry number is a store
    // that is persisting; "NO STORED CONFIG" on a device that has been used
    // before means the last session's settings were lost on the way to flash.
    case SI_STORE:
      config_get_state(buf, size);
      break;

    // Where the calibration came from. "DEFAULTS" here means the numbers
    // belong to another unit and every voltage on screen is approximate - the
    // one failure in this dialog that a working-looking scope can be hiding.
    case SI_CALIB:
      config_get_calib_state(buf, size);
      break;

    // The SPI NOR part doc/Hardware.md puts on PA3/PA5-PA7 - 8 MB that this
    // firmware does not use for anything yet. These two lines exist to settle
    // whether it is actually on the board: the JEDEC ID, then the first bytes
    // it hands back, which is a read of the array rather than of an ID
    // register and so also says the wiring works.
    case SI_SPI:
      flash_get_state(buf, size);
      break;

    case SI_SPI_HEAD:
      flash_get_head(buf, size);
      break;

    default:
      return "";
  }

  return buf;
}

//-----------------------------------------------------------------------------
// The calibration itself, in full. This page exists to be copied onto paper:
// flash is only a cache for these numbers, and the one thing that survives the
// storage sector wearing out, being erased by a layout change, or the silicon
// failing outright is somebody having written them down.
#define CAL_HEADER  2

static const char *calibration_line(int index)
{
  switch (index)
  {
    case 0:
      return "Measured on THIS unit. Copy them somewhere.";

    case 1:
      config_get_calib_state(g_info_line, sizeof(g_info_line));
      return g_info_line;

    default:
      break;
  }

  if (!config_get_calib_line(index - CAL_HEADER, g_info_line,
        sizeof(g_info_line)))
    return "";

  return g_info_line;
}

//-----------------------------------------------------------------------------
// Asked rather than hard-coded, so that a range added to VS_COUNT lengthens
// the page instead of falling off the end of it
static int calibration_lines(void)
{
  char buf[64];
  int n = 0;

  while (config_get_calib_line(n, buf, sizeof(buf)))
    n++;

  return n + CAL_HEADER;
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
  "SHIFT x2    - Sticky shift, for the next key",
  "SHIFT hold  - Shift latched on; tap SHIFT off",
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
  "  F1 F2 SAVE AUTO AC/DC",
  "  STOP EDGE 50% TRIG TRIG_UP/DOWN",
  "",
  "MENU > General Settings > Remap Keys sets",
  "what each of them does, and can turn one off",
  "altogether. A key pointed at another key stays",
  "pointed there under SHIFT too.",
  "",
  INFO_HEAD "Protected system keys",
  "  UP DOWN LEFT RIGHT MODE MENU SHIFT",
  "",
  "The menus are driven with these, so no mapping",
  "can take away the way back out of one. 1X/10X",
  "is among them without looking like it: on this",
  "board it is the SHIFT key, one wire and one bit",
  "under two names.",
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
  .count = DI_COUNT,
  .line_fn = device_info_line,
};

static const info_page_t g_page_system_info =
{
  .title = "System Information",
  .count = SI_COUNT,
  .line_fn = system_info_line,
};

// count is filled in when it opens, see action_calibration: the number of
// ranges is config.c's to decide, not this file's
static info_page_t g_page_calibration =
{
  .title = "Calibration",
  .line_fn = calibration_line,
};

//-----------------------------------------------------------------------------
// The gauge in figures: the charge the icon is showing, the voltage it came
// from, the filtering between the two, and what the reading is worth. Rendered
// by battery.c, which owns every one of those numbers - this file only asks
// how many lines there are, the same way the calibration page does.
static const char *battery_page_line(int index)
{
  if (!battery_get_line(index, g_info_line, sizeof(g_info_line)))
    return "";

  return g_info_line;
}

static int battery_page_lines(void)
{
  char buf[80];
  int n = 0;

  while (battery_get_line(n, buf, sizeof(buf)))
    n++;

  return n;
}

static info_page_t g_page_battery =
{
  .title = "Battery",
  .line_fn = battery_page_line,
};

static void action_battery(const void *arg)
{
  (void)arg;

  g_page_battery.count = battery_page_lines();

  menu_action_info(&g_page_battery);
}

static void action_calibration(const void *arg)
{
  (void)arg;

  g_page_calibration.count = calibration_lines();

  menu_action_info(&g_page_calibration);
}

//-----------------------------------------------------------------------------
// The config as the running scope holds it, field by field.
//
// This is the RAM side of the pair: the flash viewer's Config view decodes
// what is on the flash, and this decodes what is in use. They are supposed to
// agree, and every interesting bug in this store is a case where they do not -
// a change that never reached a slot, or a slot that loaded as something else.
//
// Rendered a line at a time through the page's line_fn rather than built into
// an array: eighty lines of text is 3 KB, and TCM keeps 16 KB back for stack
// and heap - a buffer that size does not link. One line of scratch does.
//-----------------------------------------------------------------------------
// Where the config came from, ahead of what it holds: a config that quietly
// fell back to defaults reads perfectly ordinary field by field
#define CFG_DUMP_HEADER   3

static char g_cfg_line[52];

static const char *config_ram_line(int index)
{
  switch (index)
  {
    case 0:
      config_get_state(g_cfg_line, sizeof(g_cfg_line));
      return g_cfg_line;

    case 1:
      config_get_calib_state(g_cfg_line, sizeof(g_cfg_line));
      return g_cfg_line;

    case 2:
      return "";

    default:
      break;
  }

  if (!config_describe(&config, index - CFG_DUMP_HEADER,
        g_cfg_line, sizeof(g_cfg_line)))
    return "";

  return g_cfg_line;
}

//-----------------------------------------------------------------------------
// The page needs its length up front, and config_describe is what knows it
static int config_ram_lines(void)
{
  char buf[52];
  int n = 0;

  while (config_describe(&config, n, buf, sizeof(buf)))
    n++;

  return n + CFG_DUMP_HEADER;
}

static info_page_t g_page_config_ram =
{
  .title = "Config (RAM)",
  .line_fn = config_ram_line,
};

static void action_config_ram(const void *arg)
{
  (void)arg;

  g_page_config_ram.count = config_ram_lines();

  menu_action_info(&g_page_config_ram);
}

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
// Either switch, either way: whatever shift was pending was pending under the
// old setting, and leaving it armed or latched across the change would outlive
// the thing that put it there
static void shift_mode_changed(void)
{
  shift_mode_reset();
}

//-----------------------------------------------------------------------------
// The menus this row is on are drawn at the size this row sets, so they have to
// be laid out again around it - see menu_relayout()
static void ui_scale_changed(void)
{
  menu_relayout();
}

//-----------------------------------------------------------------------------
static void action_key_remap(const void *arg)
{
  (void)arg;
  key_remap_open();
}

//-----------------------------------------------------------------------------
// Menu tables (const, in flash). config changes are persisted automatically
// by config_task once the struct CRC goes stale.
//-----------------------------------------------------------------------------
// What the device comes up in. A choice rather than a toggle because both
// answers are screens with names, and "Startup: ON" would say neither of them.
// config.startup_app_mode is read once, by main.c, at boot - there is nothing
// to apply here, which is why the row has no on_change.
//
// "Startup" rather than the "Start With" it reads as, for the room: a popup row
// puts the label at the left edge and the value hard against the right one, and
// "Oscilloscope" is the longest value in this menu.
static const char *const g_startup_labels[STARTUP_APP_COUNT] =
{
  "Oscilloscope",   // STARTUP_APP_SCOPE, the default
  "App Menu",       // STARTUP_APP_LAUNCHER
};

static const menu_item_t g_general_items[] =
{
  { .kind = MI_CHOICE, .label = "Startup",
    .u.choice = { &config.startup_app_mode, g_startup_labels,
        STARTUP_APP_COUNT, NULL } },
  { .kind = MI_NUMBER, .label = "Brightness",
    .u.number = { &config.lcd_bl_level, 10, 100, 5, 10, "%", apply_backlight } },
  // The menus' own text size, and it applies to the menu it is being changed
  // from: the row grows under the cursor as the value turns, which is the whole
  // preview this setting needs. It does not touch the oscilloscope's bars -
  // see UI_SCALE_* in config.h for why those cannot grow, and README for the
  // readings that can.
  // "Menu Text" and not "Text Size": it is the menus, the popups and the text
  // pages. The oscilloscope's two bars are 20 and 16 px tall and full end to
  // end, so 8x16 is their ceiling whatever this says - what makes the
  // instrument's own numbers big is the panel's Size (up to 16x32) and the
  // layout editor. A row that promised "text size" promised the bars too.
  { .kind = MI_CHOICE, .label = "Menu Text",
    .desc = "Menus and text pages, not the scope's bars",
    .u.choice = { &config.ui_scale, ui_scale_labels, UI_SCALE_COUNT,
        ui_scale_changed } },
  // Dims to a quarter of the brightness above after this long without a key,
  // and comes straight back on the next one. Not while the oscilloscope is on
  // screen - see launcher_app_may_dim.
  { .kind = MI_CHOICE, .label = "Auto Dim",
    .u.choice = { &config.lcd_dim_timeout, lcd_dim_labels, LCD_DIM_COUNT,
        NULL } },
  { .kind = MI_TOGGLE, .label = "Shift x2 Sticky",
    .u.toggle = { &config.shift_mode_enabled, shift_mode_changed } },
  { .kind = MI_TOGGLE, .label = "Shift Hold Lock",
    .u.toggle = { &config.shift_hold_lock, shift_mode_changed } },
  { .kind = MI_TOGGLE, .label = "Key Remap",
    .u.toggle = { &config.key_remapping_enabled, NULL } },
  { .kind = MI_ACTION, .label = "Remap Keys...",
    .u.action = { action_key_remap, NULL } },
};

static const menu_item_t g_advanced_items[] =
{
  { .kind = MI_ACTION, .label = "Device Info",
    .u.action = { menu_action_info, &g_page_device_info } },
  { .kind = MI_ACTION, .label = "System Info",
    .u.action = { menu_action_info, &g_page_system_info } },
  { .kind = MI_ACTION, .label = "Battery",
    .u.action = { action_battery, NULL } },
  { .kind = MI_ACTION, .label = "Calibration",
    .u.action = { action_calibration, NULL } },
  { .kind = MI_ACTION, .label = "Config (RAM)",
    .u.action = { action_config_ram, NULL } },
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
