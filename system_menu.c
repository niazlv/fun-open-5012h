/*
 * System menu: settings popup and info dialogs
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
#include "capture.h"
#include "scope.h"
#include "ui.h"
#include "menu_widget.h"
#include "input.h"
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

#define DLG_BG            LCD_COLOR(240, 240, 240)
#define DLG_FG            LCD_COLOR(0, 0, 0)

// Linker script symbols for memory usage calculation
extern uint32_t _etext;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;
extern uint32_t _end;
extern uint32_t __etext; // end of all flash data, including .data image

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  void (*draw)(void);
} dialog_def_t;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Modal info dialog: fullscreen, any button closes it
//-----------------------------------------------------------------------------
static void dialog_draw(void *ctx, bool full)
{
  (void)full;
  ((const dialog_def_t *)ctx)->draw();
}

//-----------------------------------------------------------------------------
static bool dialog_input(void *ctx, int buttons)
{
  (void)ctx;

  if (buttons && !(buttons & BTN_REPEAT))
    ui_pop();

  return true;
}

static const ui_screen_t dialog_screen =
{
  .draw   = dialog_draw,
  .input  = dialog_input,
  .opaque = true,
};

//-----------------------------------------------------------------------------
static void dialog_frame(const char *title)
{
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, DLG_BG);
  lcd_draw_rect(10, 10, LCD_WIDTH - 21, LCD_HEIGHT - 21, DLG_FG);

  lcd_set_color(DLG_BG, DLG_FG);
  lcd_set_font(FONT_LARGE);
  lcd_puts(20, 20, title);

  lcd_set_font(FONT_SMALL);
  lcd_puts(20, LCD_HEIGHT - 25, "Press any button to close");
}

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
// Dialog bodies
//-----------------------------------------------------------------------------
static void draw_device_info(void)
{
  char buf[80];
  char mcu_buf[32];

  dialog_frame("Device Information");

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
  lcd_puts(20, 45, buf);

  sprintf(buf, "DevID: 0x%03X, Rev: 0x%04X", dev_id, rev_id);
  lcd_puts(20, 60, buf);

  uint16_t flash_size = *(uint16_t *)FLASH_SIZE_REG;

  if (0xFFFF == flash_size || 0 == flash_size)
    flash_size = 512;

  sprintf(buf, "Flash: %u KB, SRAM: %u KB + TCM: %u KB",
      flash_size, SRAM_SIZE_KB, TCM_SIZE_KB);
  lcd_puts(20, 75, buf);

  sprintf(buf, "CPU Freq: %lu MHz", F_CPU / 1000000UL);
  lcd_puts(20, 90, buf);

  uint32_t uid0 = *(uint32_t *)(UID_REG_BASE + 0);
  uint32_t uid1 = *(uint32_t *)(UID_REG_BASE + 4);
  uint32_t uid2 = *(uint32_t *)(UID_REG_BASE + 8);
  sprintf(buf, "UID: %08lX-%08lX-%08lX", uid0, uid1, uid2);
  lcd_puts(20, 105, buf);

#ifdef FIRMWARE_NAME
  sprintf(buf, "%s v%s", FIRMWARE_NAME, FIRMWARE_VERSION);
  lcd_puts(20, 120, buf);
#endif

#ifdef BUILD_DATE
  sprintf(buf, "Built: %s %s", BUILD_DATE, BUILD_TIME);
#else
  sprintf(buf, "Built: %s %s", __DATE__, __TIME__);
#endif
  lcd_puts(20, 135, buf);

#ifdef GIT_HASH
  sprintf(buf, "Git: %s", GIT_HASH);
  lcd_puts(20, 150, buf);
#endif

  sprintf(buf, "FW Hash(CRC32): %08lX", calculate_firmware_hash());
  lcd_puts(20, 165, buf);
}

//-----------------------------------------------------------------------------
static void draw_system_info(void)
{
  char buf[80];

  dialog_frame("System Information");

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
  lcd_puts(20, 45, buf);

  sprintf(buf, "TCM RAM: %lu KB / %u KB (%lu.%lu%%)",
      ram_used / 1024, TCM_SIZE_KB, ram_pct10 / 10, ram_pct10 % 10);
  lcd_puts(20, 60, buf);

  sprintf(buf, "Data: %lu B, BSS: %lu B", data_size, bss_size);
  lcd_puts(20, 75, buf);

  sprintf(buf, "CPU Freq: %lu MHz", F_CPU / 1000000UL);
  lcd_puts(20, 90, buf);

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
  lcd_puts(20, 105, buf);

#ifdef GIT_HASH
  sprintf(buf, "Git: %s (%s)", GIT_HASH, GIT_BRANCH);
  lcd_puts(20, 120, buf);
#endif

#ifdef GIT_USER
  sprintf(buf, "Built by: %s", GIT_USER);
  lcd_puts(20, 135, buf);
#endif

  // Worst main-loop stall since this dialog was last opened: the practical
  // "is it responsive" metric (a button press can wait at most this long)
  sprintf(buf, "Loop max: %d ms   Scope FPS: %d",
      timer_get_max_delta(), scope_get_fps());
  lcd_puts(20, 150, buf);
}

//-----------------------------------------------------------------------------
static void draw_key_bindings(void)
{
  dialog_frame("Key Bindings");

  lcd_puts(20, 45, "UP/DOWN     - Navigate");
  lcd_puts(20, 60, "LEFT/RIGHT  - Adjust / submenu");
  lcd_puts(20, 75, "MODE        - Select / confirm");
  lcd_puts(20, 90, "MENU        - System menu / back");
  lcd_puts(20, 105, "SHIFT x2    - Sticky shift (if enabled)");

  lcd_puts(20, 130, "Remappable keys:");
  lcd_puts(30, 145, "F1 F2 SAVE AUTO AC/DC 1X/10X");
  lcd_puts(30, 160, "STOP EDGE 50% TRIG TRIG_UP/DOWN");

  lcd_puts(20, 180, "Protected system keys:");
  lcd_puts(30, 195, "UP DOWN LEFT RIGHT MODE MENU SHIFT");
}

//-----------------------------------------------------------------------------
static void draw_scope_guide(void)
{
  dialog_frame("Scope Functions");

  lcd_puts(20, 45,  "MODE        - Measurements on/off:");
  lcd_puts(30, 57,  "[M] Vpp+freq  [R] Vrms+duty  [S] type+THD");
  lcd_puts(20, 72,  "SHIFT+MODE  - FFT spectrum view");
  lcd_puts(30, 84,  "see the Spectrum (FFT) page for details");
  lcd_puts(20, 99,  "SHIFT+EDGE  - Decoder: UART/1-Wire/");
  lcd_puts(30, 111, "WS2812/NEC/raw; TRIG_UP/DN jump bytes");
  lcd_puts(20, 126, "AUTO        - Auto-setup (scale/trigger)");
  lcd_puts(20, 141, "50%         - Trigger level to mid-signal");
  lcd_puts(20, 156, "STOP        - Freeze: pan/zoom/measure/");
  lcd_puts(30, 168, "decode/spectrum on the frozen record");
  lcd_puts(20, 183, "TRIG / EDGE - Trigger mode / edge");
  lcd_puts(20, 195, "TRIG_UP/DN  - Trigger level");
}

//-----------------------------------------------------------------------------
// The spectrum view has its own key map and its own trap (resolution comes
// from the timebase, not from the sample rate), so it gets its own page
static void draw_spectrum_guide(void)
{
  dialog_frame("Spectrum (FFT)");

  lcd_puts(20, 42,  "SHIFT+MODE - enter/leave   MODE - panel");
  lcd_puts(20, 54,  "LEFT/RIGHT - cursor   UP/DOWN - peaks");
  lcd_puts(20, 66,  "TRIG_UP/DN - wider span / finer df");
  lcd_puts(20, 78,  "SHIFT+L/R - timebase  SHIFT+U/D - volts");

  lcd_puts(20, 96,  "The whole record is transformed, so the");
  lcd_puts(20, 108, "resolution df = 1/record time and the");
  lcd_puts(20, 120, "TIMEBASE sets it, not the sample rate.");
  lcd_puts(20, 132, "df is on the panel: mains 50/100 Hz");
  lcd_puts(20, 144, "needs 5 ms/div or slower to separate.");

  lcd_puts(20, 162, "F0 comes from the harmonic comb, not");
  lcd_puts(20, 174, "from the tallest peak. h1..hN mark its");
  lcd_puts(20, 186, "harmonics, '-' marks peaks that belong");
  lcd_puts(20, 198, "to something else: interference, noise.");
}

static const dialog_def_t dlg_device_info = { draw_device_info };
static const dialog_def_t dlg_system_info = { draw_system_info };
static const dialog_def_t dlg_key_bindings = { draw_key_bindings };
static const dialog_def_t dlg_scope_guide = { draw_scope_guide };
static const dialog_def_t dlg_spectrum_guide = { draw_spectrum_guide };

//-----------------------------------------------------------------------------
// Menu actions
//-----------------------------------------------------------------------------
static void action_dialog(const void *arg)
{
  ui_push(&dialog_screen, (void *)(uintptr_t)arg);
}

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
// Trigger settings: values live in config, hardware follows immediately.
// The scope screen underneath repaints in full when the menu closes.
//-----------------------------------------------------------------------------
static void trigger_mode_changed(void)
{
  capture_set_trigger_mode(config.trigger_mode);
  capture_start(); // arms SINGLE, resumes after a stopped capture
}

//-----------------------------------------------------------------------------
static void trigger_edge_changed(void)
{
  capture_set_trigger_edge(config.trigger_edge);
}

//-----------------------------------------------------------------------------
static void trigger_level_apply(int value)
{
  (void)value; // the widget already stored it in config.trigger_level
  scope_apply_trigger_level();
}

//-----------------------------------------------------------------------------
static void action_trigger_50p(const void *arg)
{
  (void)arg;
  scope_trigger_50_percent();
  ui_request_redraw(); // the Level row shows the new value
}

static const char *const g_trigger_mode_labels[] = { "Auto", "Normal", "Single" };
static const char *const g_trigger_edge_labels[] = { "Rise", "Fall", "Both" };

static const char *const g_measure_view_labels[] = { "Panel", "Rotate" };

static const char *const g_decoder_proto_labels[] =
{
  "Auto", "UART", "1-Wire", "WS2812", "NEC IR", "Raw",
};

//-----------------------------------------------------------------------------
// Menu tables (const, in flash). config changes are persisted automatically
// by config_task once the struct CRC goes stale.
//-----------------------------------------------------------------------------
static const menu_item_t g_trigger_items[] =
{
  { .kind = MI_CHOICE, .label = "Mode",
    .u.choice = { &config.trigger_mode, g_trigger_mode_labels, 3, trigger_mode_changed } },
  { .kind = MI_CHOICE, .label = "Edge",
    .u.choice = { &config.trigger_edge, g_trigger_edge_labels, 3, trigger_edge_changed } },
  { .kind = MI_NUMBER, .label = "Level",
    .u.number = { &config.trigger_level, -100, 100, 1, 5, "px", trigger_level_apply } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "Level to 50%",
    .u.action = { action_trigger_50p, NULL } },
};

// Measurements display: values live in config; the scope picks them up on
// its next tick, no callbacks needed
static const menu_item_t g_measure_items[] =
{
  { .kind = MI_TOGGLE, .label = "Show (MODE)",
    .u.toggle = { &config.measure_display, NULL } },
  { .kind = MI_CHOICE, .label = "View",
    .u.choice = { &config.measure_panel_mode, g_measure_view_labels, 2, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_TOGGLE, .label = "Vpp",
    .u.toggle = { &config.show_vpp, NULL } },
  { .kind = MI_TOGGLE, .label = "Frequency",
    .u.toggle = { &config.show_freq, NULL } },
  { .kind = MI_TOGGLE, .label = "Duty cycle",
    .u.toggle = { &config.show_duty, NULL } },
  { .kind = MI_TOGGLE, .label = "Vrms",
    .u.toggle = { &config.show_vrms, NULL } },
  { .kind = MI_TOGGLE, .label = "Vavg",
    .u.toggle = { &config.show_vavg, NULL } },
  { .kind = MI_TOGGLE, .label = "Signal type",
    .u.toggle = { &config.show_type, NULL } },
  { .kind = MI_TOGGLE, .label = "THD",
    .u.toggle = { &config.show_thd, NULL } },
};

static const menu_item_t g_decoder_items[] =
{
  { .kind = MI_CHOICE, .label = "Protocol",
    .u.choice = { &config.decoder_proto, g_decoder_proto_labels, 6, NULL } },
  { .kind = MI_TOGGLE, .label = "Stop on frames",
    .u.toggle = { &config.decoder_stop, NULL } },
};

static const menu_item_t g_general_items[] =
{
  { .kind = MI_NUMBER, .label = "Brightness",
    .u.number = { &config.lcd_bl_level, 10, 100, 5, 10, "%", apply_backlight } },
  { .kind = MI_TOGGLE, .label = "Shift Mode",
    .u.toggle = { &config.shift_mode_enabled, shift_mode_changed } },
  { .kind = MI_TOGGLE, .label = "Key Remap",
    .u.toggle = { &config.key_remapping_enabled, NULL } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "Key Bindings",
    .u.action = { action_dialog, &dlg_key_bindings } },
};

static const menu_item_t g_advanced_items[] =
{
  { .kind = MI_ACTION, .label = "Device Info",
    .u.action = { action_dialog, &dlg_device_info } },
  { .kind = MI_ACTION, .label = "System Info",
    .u.action = { action_dialog, &dlg_system_info } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "Reboot System",
    .u.action = { action_reboot, NULL } },
};

static const menu_item_t g_main_items[] =
{
  { .kind = MI_SUBMENU, .label = "Trigger Settings",
    .u.submenu = { g_trigger_items, ARRAY_SIZE(g_trigger_items) } },
  { .kind = MI_SUBMENU, .label = "Measurements",
    .u.submenu = { g_measure_items, ARRAY_SIZE(g_measure_items) } },
  { .kind = MI_SUBMENU, .label = "Decoder",
    .u.submenu = { g_decoder_items, ARRAY_SIZE(g_decoder_items) } },
  { .kind = MI_ACTION, .label = "Scope Functions",
    .u.action = { action_dialog, &dlg_scope_guide } },
  { .kind = MI_ACTION, .label = "Spectrum (FFT)",
    .u.action = { action_dialog, &dlg_spectrum_guide } },
  { .kind = MI_SUBMENU, .label = "General Settings",
    .u.submenu = { g_general_items, ARRAY_SIZE(g_general_items) } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_SUBMENU, .label = "Advanced",
    .u.submenu = { g_advanced_items, ARRAY_SIZE(g_advanced_items) } },
  { .kind = MI_SEPARATOR },
  { .kind = MI_ACTION, .label = "Exit Application",
    .u.action = { action_exit_app, NULL } },
};

static const menu_def_t g_main_menu =
{
  .items = g_main_items,
  .count = ARRAY_SIZE(g_main_items),
};

//-----------------------------------------------------------------------------
void system_menu_open(void)
{
  menu_open_popup(&g_main_menu, 10, 10);
}
