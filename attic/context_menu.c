/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Context Menu System
 * Windows-style context menu with system information and settings
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
#include "context_menu.h"
#include "menu_system.h"
#include "capture.h"
#include "config.h"
#include "utils.h"

// External variables for memory calculation from linker script
extern uint32_t _etext;   // End of text section (flash)
extern uint32_t _data;    // Start of data section in RAM
extern uint32_t _edata;   // End of data section in RAM
extern uint32_t _bss;     // Start of BSS section
extern uint32_t _ebss;    // End of BSS section
extern uint32_t _end;     // End of used RAM
extern uint32_t __etext;  // End of all flash data (including .data section)

// Hardware register addresses
#define DBG_ID_REG              0xE0042000  // Debug ID register
#define FLASH_SIZE_REG          0x1FFF7A22  // Flash size register (16-bit)
#define UID_BASE                0x1FFF7A10  // Unique ID base address
#define FLASH_BASE              0x08000000  // Flash memory base
#define TCM_BASE                0x10000000  // TCM memory base

// Memory sizes (can be read from registers or calculated)
#define FLASH_SIZE_KB           (*(uint16_t*)FLASH_SIZE_REG)
#define TCM_SIZE_KB             64
#define SRAM_SIZE_KB            128

/*- Definitions -------------------------------------------------------------*/
#define MENU_BG_COLOR           LCD_COLOR(240, 240, 240)  // Light gray background
#define MENU_BORDER_COLOR       LCD_COLOR(128, 128, 128)  // Gray border
#define MENU_TEXT_COLOR         LCD_COLOR(0, 0, 0)        // Black text
#define MENU_SELECTED_BG_COLOR  LCD_COLOR(0, 120, 215)    // Blue selection
#define MENU_SELECTED_TEXT_COLOR LCD_COLOR(255, 255, 255) // White selected text
#define MENU_SEPARATOR_COLOR    LCD_COLOR(160, 160, 160)  // Gray separator
#define MENU_SHADOW_COLOR       LCD_COLOR(64, 64, 64)     // Dark shadow

/*- Variables ---------------------------------------------------------------*/
static context_menu_t g_context_menu;
static context_menu_item_t g_main_menu_items[CONTEXT_MENU_MAX_ITEMS];
static context_menu_item_t g_advanced_submenu[CONTEXT_MENU_MAX_SUBITEMS];
static context_menu_item_t g_general_submenu[CONTEXT_MENU_MAX_SUBITEMS];

// Shift mode state
static struct {
    bool enabled;
    bool active;
    uint32_t last_shift_press;
    uint32_t shift_press_count;
    bool double_click_detected;
} g_shift_mode;

// Key remapping state
static struct {
    bool in_remap_mode;
    int current_key_index;
    const char* key_names[18];
    uint32_t original_keys[18];
    uint32_t system_keys_mask;  // Mask of keys that cannot be remapped
} g_key_remap;

// Settings UI state
static settings_ui_t g_settings_ui;

/*- Forward Declarations ----------------------------------------------------*/
static void draw_context_menu(void);
static void draw_menu_item(context_menu_item_t *item, int x, int y, int width, bool selected);
static void calculate_menu_size(void);
static void setup_menu_items(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void context_menu_init(void)
{
    memset(&g_context_menu, 0, sizeof(g_context_menu));
    g_context_menu.items = g_main_menu_items;
    g_context_menu.visible = false;
    g_context_menu.dirty = false;
    g_context_menu.in_submenu = false;
    g_context_menu.modal_dialog_active = false;
    
    setup_menu_items();
}

//-----------------------------------------------------------------------------
static void setup_menu_items(void)
{
    // Setup general settings submenu
    g_general_submenu[0] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "Screen Brightness",
        .action = context_menu_action_brightness_settings
    };
    
    g_general_submenu[1] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "Key Bindings",
        .action = context_menu_action_key_bindings
    };
    
    g_general_submenu[2] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "Key Remapping",
        .action = context_menu_action_key_remapping
    };
    
    g_general_submenu[3] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_SEPARATOR,
        .text = NULL
    };
    
    g_general_submenu[4] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = config.shift_mode_enabled ? "Shift Mode: ON" : "Shift Mode: OFF",
        .action = context_menu_action_shift_mode_toggle
    };

    // Setup advanced submenu
    g_advanced_submenu[0] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "Device Info",
        .action = context_menu_action_device_info
    };
    
    g_advanced_submenu[1] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "System Info",
        .action = context_menu_action_system_info
    };
    
    g_advanced_submenu[2] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_SEPARATOR,
        .text = NULL
    };
    
    g_advanced_submenu[3] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "Reboot to Firmware",
        .action = context_menu_action_reboot_firmware
    };
    
    g_advanced_submenu[4] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "Reboot System",
        .action = context_menu_action_reboot_system
    };

    // Setup main menu
    g_main_menu_items[0] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "Trigger Settings",
        .action = context_menu_action_trigger_settings
    };
    
    g_main_menu_items[1] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_SUBMENU,
        .text = "General Settings >",
        .submenu = g_general_submenu,
        .submenu_count = 5
    };
    
    g_main_menu_items[2] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_SEPARATOR,
        .text = NULL
    };
    
    g_main_menu_items[3] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_SUBMENU,
        .text = "Advanced >",
        .submenu = g_advanced_submenu,
        .submenu_count = 5
    };
    
    g_main_menu_items[4] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_SEPARATOR,
        .text = NULL
    };
    
    g_main_menu_items[5] = (context_menu_item_t){
        .type = CONTEXT_MENU_ITEM_NORMAL,
        .text = "Exit Application",
        .action = context_menu_action_exit_app
    };
    
    g_context_menu.item_count = 6;
}

//-----------------------------------------------------------------------------
static void calculate_menu_size(void)
{
    g_context_menu.width = CONTEXT_MENU_WIDTH;
    g_context_menu.height = g_context_menu.item_count * CONTEXT_MENU_ITEM_HEIGHT + 2 * CONTEXT_MENU_MARGIN;
    
    // Ensure menu fits on screen
    if (g_context_menu.x + g_context_menu.width > LCD_WIDTH) {
        g_context_menu.x = LCD_WIDTH - g_context_menu.width;
    }
    if (g_context_menu.y + g_context_menu.height > LCD_HEIGHT) {
        g_context_menu.y = LCD_HEIGHT - g_context_menu.height;
    }
}

//-----------------------------------------------------------------------------
void context_menu_show(int x, int y)
{
    g_context_menu.x = x;
    g_context_menu.y = y;
    g_context_menu.selected_item = 0;
    g_context_menu.in_submenu = false;
    g_context_menu.visible = true;
    g_context_menu.dirty = true;
    
    calculate_menu_size();
}

//-----------------------------------------------------------------------------
void context_menu_hide(void)
{
    g_context_menu.visible = false;
    g_context_menu.in_submenu = false;
    g_context_menu.dirty = false;
}

//-----------------------------------------------------------------------------
bool context_menu_is_visible(void)
{
    return g_context_menu.visible;
}

//-----------------------------------------------------------------------------
bool context_menu_is_modal_active(void)
{
    return g_context_menu.modal_dialog_active;
}

//-----------------------------------------------------------------------------
void context_menu_close_modal(void)
{
    g_context_menu.modal_dialog_active = false;
    
    // Clear screen and restore font settings
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    lcd_set_font(FONT_LARGE);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_menu_item(context_menu_item_t *item, int x, int y, int width, bool selected)
{
    uint16_t bg_color = selected ? MENU_SELECTED_BG_COLOR : MENU_BG_COLOR;
    uint16_t text_color = selected ? MENU_SELECTED_TEXT_COLOR : MENU_TEXT_COLOR;
    
    if (item->type == CONTEXT_MENU_ITEM_SEPARATOR) {
        // Draw separator line
        lcd_fill_rect(x + 5, y + CONTEXT_MENU_ITEM_HEIGHT/2 - 1, width - 10, 1, MENU_SEPARATOR_COLOR);
        return;
    }
    
    // Draw item background
    lcd_fill_rect(x, y, width, CONTEXT_MENU_ITEM_HEIGHT, bg_color);
    
    // Draw text
    if (item->text) {
        lcd_set_font(FONT_SMALL);
        lcd_set_color(bg_color, text_color);
        lcd_puts(x + 8, y + 4, item->text);
    }
}

//-----------------------------------------------------------------------------
static void draw_context_menu(void)
{
    if (!g_context_menu.visible) return;
    
    int menu_x = g_context_menu.x;
    int menu_y = g_context_menu.y;
    int menu_w = g_context_menu.width;
    int menu_h = g_context_menu.height;
    
    // Draw shadow
    lcd_fill_rect(menu_x + 2, menu_y + 2, menu_w, menu_h, MENU_SHADOW_COLOR);
    
    // Draw background
    lcd_fill_rect(menu_x, menu_y, menu_w, menu_h, MENU_BG_COLOR);
    
    // Draw border
    lcd_draw_rect(menu_x, menu_y, menu_w - 1, menu_h - 1, MENU_BORDER_COLOR);
    
    // Draw menu items
    for (int i = 0; i < g_context_menu.item_count; i++) {
        int item_y = menu_y + CONTEXT_MENU_MARGIN + i * CONTEXT_MENU_ITEM_HEIGHT;
        bool selected = (i == g_context_menu.selected_item) && !g_context_menu.in_submenu;
        
        draw_menu_item(&g_context_menu.items[i], menu_x + CONTEXT_MENU_MARGIN, 
                      item_y, menu_w - 2 * CONTEXT_MENU_MARGIN, selected);
    }
    
    // Draw submenu if active
    if (g_context_menu.in_submenu && g_context_menu.current_submenu) {
        int submenu_w = CONTEXT_MENU_WIDTH - 20;
        int submenu_h = g_context_menu.current_submenu->submenu_count * CONTEXT_MENU_ITEM_HEIGHT + 2 * CONTEXT_MENU_MARGIN;
        
        // Draw submenu shadow
        lcd_fill_rect(g_context_menu.submenu_x + 2, g_context_menu.submenu_y + 2, submenu_w, submenu_h, MENU_SHADOW_COLOR);
        
        // Draw submenu background
        lcd_fill_rect(g_context_menu.submenu_x, g_context_menu.submenu_y, submenu_w, submenu_h, MENU_BG_COLOR);
        
        // Draw submenu border
        lcd_draw_rect(g_context_menu.submenu_x, g_context_menu.submenu_y, submenu_w - 1, submenu_h - 1, MENU_BORDER_COLOR);
        
        // Draw submenu items
        for (int i = 0; i < g_context_menu.current_submenu->submenu_count; i++) {
            int item_y = g_context_menu.submenu_y + CONTEXT_MENU_MARGIN + i * CONTEXT_MENU_ITEM_HEIGHT;
            bool selected = (i == g_context_menu.submenu_selected);
            
            draw_menu_item(&g_context_menu.current_submenu->submenu[i], 
                          g_context_menu.submenu_x + CONTEXT_MENU_MARGIN, 
                          item_y, submenu_w - 2 * CONTEXT_MENU_MARGIN, selected);
        }
    }
}

//-----------------------------------------------------------------------------
void context_menu_task(void)
{
    if (g_context_menu.visible && g_context_menu.dirty) {
        draw_context_menu();
        g_context_menu.dirty = false;
    }
}

//-----------------------------------------------------------------------------
void context_menu_buttons_handler(int buttons)
{
    if (!g_context_menu.visible) return;
    
    bool repeat = (buttons & BTN_REPEAT);
    
    if (buttons & BTN_UP) {
        if (g_context_menu.in_submenu) {
            if (g_context_menu.submenu_selected > 0) {
                g_context_menu.submenu_selected--;
                // Skip separators
                while (g_context_menu.submenu_selected >= 0 && 
                       g_context_menu.current_submenu->submenu[g_context_menu.submenu_selected].type == CONTEXT_MENU_ITEM_SEPARATOR) {
                    g_context_menu.submenu_selected--;
                }
                if (g_context_menu.submenu_selected < 0) {
                    g_context_menu.submenu_selected = 0;
                }
                g_context_menu.dirty = true;
            }
        } else {
            if (g_context_menu.selected_item > 0) {
                g_context_menu.selected_item--;
                // Skip separators
                while (g_context_menu.selected_item >= 0 && 
                       g_context_menu.items[g_context_menu.selected_item].type == CONTEXT_MENU_ITEM_SEPARATOR) {
                    g_context_menu.selected_item--;
                }
                if (g_context_menu.selected_item < 0) {
                    g_context_menu.selected_item = 0;
                }
                g_context_menu.dirty = true;
            }
        }
    }
    else if (buttons & BTN_DOWN) {
        if (g_context_menu.in_submenu) {
            if (g_context_menu.submenu_selected < g_context_menu.current_submenu->submenu_count - 1) {
                g_context_menu.submenu_selected++;
                // Skip separators
                while (g_context_menu.submenu_selected < g_context_menu.current_submenu->submenu_count && 
                       g_context_menu.current_submenu->submenu[g_context_menu.submenu_selected].type == CONTEXT_MENU_ITEM_SEPARATOR) {
                    g_context_menu.submenu_selected++;
                }
                if (g_context_menu.submenu_selected >= g_context_menu.current_submenu->submenu_count) {
                    g_context_menu.submenu_selected = g_context_menu.current_submenu->submenu_count - 1;
                }
                g_context_menu.dirty = true;
            }
        } else {
            if (g_context_menu.selected_item < g_context_menu.item_count - 1) {
                g_context_menu.selected_item++;
                // Skip separators
                while (g_context_menu.selected_item < g_context_menu.item_count && 
                       g_context_menu.items[g_context_menu.selected_item].type == CONTEXT_MENU_ITEM_SEPARATOR) {
                    g_context_menu.selected_item++;
                }
                if (g_context_menu.selected_item >= g_context_menu.item_count) {
                    g_context_menu.selected_item = g_context_menu.item_count - 1;
                }
                g_context_menu.dirty = true;
            }
        }
    }
    else if (buttons & BTN_RIGHT) {
        if (!repeat && !g_context_menu.in_submenu) {
            context_menu_item_t *item = &g_context_menu.items[g_context_menu.selected_item];
            if (item->type == CONTEXT_MENU_ITEM_SUBMENU && item->submenu) {
                g_context_menu.in_submenu = true;
                g_context_menu.current_submenu = item;
                g_context_menu.submenu_selected = 0;
                
                // Position submenu
                g_context_menu.submenu_x = g_context_menu.x + g_context_menu.width - 5;
                g_context_menu.submenu_y = g_context_menu.y + CONTEXT_MENU_MARGIN + 
                                          g_context_menu.selected_item * CONTEXT_MENU_ITEM_HEIGHT;
                
                // Skip separators in submenu
                while (g_context_menu.submenu_selected < item->submenu_count && 
                       item->submenu[g_context_menu.submenu_selected].type == CONTEXT_MENU_ITEM_SEPARATOR) {
                    g_context_menu.submenu_selected++;
                }
                g_context_menu.dirty = true;
            }
        }
    }
    else if (buttons & BTN_LEFT) {
        if (!repeat && g_context_menu.in_submenu) {
            g_context_menu.in_submenu = false;
            g_context_menu.dirty = true;
        }
    }
    else if (buttons & BTN_MODE) {
        if (!repeat) {
            if (g_context_menu.in_submenu) {
                context_menu_item_t *item = &g_context_menu.current_submenu->submenu[g_context_menu.submenu_selected];
                if (item->action && item->type == CONTEXT_MENU_ITEM_NORMAL) {
                    context_menu_hide();
                    item->action();
                }
            } else {
                context_menu_item_t *item = &g_context_menu.items[g_context_menu.selected_item];
                if (item->action && item->type == CONTEXT_MENU_ITEM_NORMAL) {
                    context_menu_hide();
                    item->action();
                } else if (item->type == CONTEXT_MENU_ITEM_SUBMENU) {
                    // Same as RIGHT button
                    g_context_menu.in_submenu = true;
                    g_context_menu.current_submenu = item;
                    g_context_menu.submenu_selected = 0;
                    
                    g_context_menu.submenu_x = g_context_menu.x + g_context_menu.width - 5;
                    g_context_menu.submenu_y = g_context_menu.y + CONTEXT_MENU_MARGIN + 
                                              g_context_menu.selected_item * CONTEXT_MENU_ITEM_HEIGHT;
                    
                    while (g_context_menu.submenu_selected < item->submenu_count && 
                           item->submenu[g_context_menu.submenu_selected].type == CONTEXT_MENU_ITEM_SEPARATOR) {
                        g_context_menu.submenu_selected++;
                    }
                }
            }
        }
    }
    else if (buttons & BTN_MENU) {
        if (!repeat) {
            context_menu_hide();
        }
    }
}

//-----------------------------------------------------------------------------
// Firmware hash calculation
//-----------------------------------------------------------------------------

// Standard CRC32 implementation (IEEE 802.3 polynomial)
static uint32_t crc32_ieee(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;  // IEEE 802.3 polynomial
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}

static uint32_t calculate_firmware_hash(void)
{
    // Calculate CRC32 of the entire firmware in flash memory (including .data section)
    // Use __etext which points to the end of all flash data (after .data section)
    uint32_t flash_used = (uint32_t)&__etext - FLASH_BASE;
    uint8_t *flash_start = (uint8_t*)FLASH_BASE;
    
    return crc32_ieee(flash_start, flash_used);
}

//-----------------------------------------------------------------------------
// Menu Actions
//-----------------------------------------------------------------------------

void context_menu_action_exit_app(void)
{
    menu_system_return_to_menu();
}

void context_menu_action_trigger_settings(void)
{
    // TODO: Implement trigger settings dialog
    // For now, just show a simple message
    lcd_fill_rect(50, 100, 220, 40, LCD_COLOR(255, 255, 0));
    lcd_set_color(LCD_COLOR(255, 255, 0), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_LARGE);
    lcd_puts(60, 110, "Trigger Settings");
    lcd_puts(80, 125, "(Not implemented)");
}

void context_menu_action_general_settings(void)
{
    // TODO: Implement general settings dialog
    lcd_fill_rect(50, 100, 220, 40, LCD_COLOR(255, 255, 0));
    lcd_set_color(LCD_COLOR(255, 255, 0), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_LARGE);
    lcd_puts(60, 110, "General Settings");
    lcd_puts(80, 125, "(Not implemented)");
}

void context_menu_action_reboot_firmware(void)
{
    // TODO: Implement reboot to firmware mode
    lcd_fill_rect(50, 100, 220, 40, LCD_COLOR(255, 0, 0));
    lcd_set_color(LCD_COLOR(255, 0, 0), LCD_COLOR(255, 255, 255));
    lcd_set_font(FONT_LARGE);
    lcd_puts(60, 110, "Reboot Firmware");
    lcd_puts(80, 125, "(Not implemented)");
}

void context_menu_action_reboot_system(void)
{
    // Perform system reset
    NVIC_SystemReset();
}

void context_menu_action_device_info(void)
{
    // Activate modal dialog mode
    g_context_menu.modal_dialog_active = true;
    
    // Clear screen first
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_COLOR(240, 240, 240));
    
    // Show device information
    lcd_draw_rect(10, 10, LCD_WIDTH - 21, LCD_HEIGHT - 21, LCD_COLOR(0, 0, 0));
    
    lcd_set_color(LCD_COLOR(240, 240, 240), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_LARGE);
    lcd_puts(20, 20, "Device Information");
    
    lcd_set_font(FONT_SMALL);
    
    char buf[80];
    char mcu_buf[32];

    // Read device ID and determine MCU type dynamically
    uint32_t device_id = *(uint32_t*)DBG_ID_REG;
    uint16_t dev_id = (device_id & 0xFFF);
    uint16_t rev_id = (device_id >> 16) & 0xFFFF;
    
    // Determine MCU name from device ID
    const char* mcu_name = "Unknown MCU";
    const char* core_name = "Unknown Core";
    
    switch (dev_id) {
        case 0x413:
            mcu_name = "GD32F407VE";
            core_name = "ARM Cortex-M4";
            break;
        case 0x414:
            mcu_name = "GD32F405/407";
            core_name = "ARM Cortex-M4";
            break;
        default:
            // Not buf: the next sprintf reads mcu_name while writing buf
            snprintf(mcu_buf, sizeof(mcu_buf), "Unknown GD32 (0x%03X)", dev_id);
            mcu_name = mcu_buf;
            core_name = "ARM Cortex-M4";
            break;
    }

    snprintf(buf, sizeof(buf), "MCU: %s (%s)", mcu_name, core_name);
    lcd_puts(20, 45, buf);
    sprintf(buf, "DevID: 0x%03X, Rev: 0x%04X", dev_id, rev_id);
    lcd_puts(20, 60, buf);
    
    // Read flash size from register (if available)
    uint16_t flash_size = FLASH_SIZE_KB;
    if (flash_size == 0xFFFF || flash_size == 0) {
        flash_size = 512; // Default fallback
    }
    
    sprintf(buf, "Flash: %u KB, SRAM: %u KB + TCM: %u KB", 
            flash_size, SRAM_SIZE_KB, TCM_SIZE_KB);
    lcd_puts(20, 75, buf);
    
    // CPU Frequency
    sprintf(buf, "CPU Freq: %lu MHz", F_CPU / 1000000UL);
    lcd_puts(20, 90, buf);
    
    // Unique Device ID (96-bit)
    uint32_t uid0 = *(uint32_t*)(UID_BASE + 0);
    uint32_t uid1 = *(uint32_t*)(UID_BASE + 4);
    uint32_t uid2 = *(uint32_t*)(UID_BASE + 8);
    sprintf(buf, "UID: %08lX-%08lX-%08lX", uid0, uid1, uid2);
    lcd_puts(20, 105, buf);
    
    // Firmware information
    #ifdef FIRMWARE_NAME
    sprintf(buf, "%s v%s", FIRMWARE_NAME, FIRMWARE_VERSION);
    lcd_puts(20, 120, buf);
    #else
    lcd_puts(20, 120, "Firmware: Unknown");
    #endif
    
    // Build information
    #ifdef BUILD_DATE
    sprintf(buf, "Built: %s %s", BUILD_DATE, BUILD_TIME);
    lcd_puts(20, 135, buf);
    #else
    sprintf(buf, "Built: %s %s", __DATE__, __TIME__);
    lcd_puts(20, 135, buf);
    #endif
    
    // Git information
    #ifdef GIT_HASH
    sprintf(buf, "Git: %s", GIT_HASH);
    lcd_puts(20, 150, buf);
    #endif
    
    // Calculate and show firmware hash
    uint32_t fw_hash = calculate_firmware_hash();
    sprintf(buf, "FW Hash(CRC32): %08lX", fw_hash);
    lcd_puts(20, 165, buf);
    
    lcd_set_font(FONT_LARGE);
    lcd_puts(20, 185, "Press any button to close");
}

void context_menu_action_system_info(void)
{
    // Activate modal dialog mode
    g_context_menu.modal_dialog_active = true;
    
    // Clear screen first
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_COLOR(240, 240, 240));
    
    // Show system information
    lcd_draw_rect(10, 10, LCD_WIDTH - 21, LCD_HEIGHT - 21, LCD_COLOR(0, 0, 0));
    
    lcd_set_color(LCD_COLOR(240, 240, 240), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_LARGE);
    lcd_puts(20, 20, "System Information");
    
    lcd_set_font(FONT_SMALL);
    
    char buf[80];
    
    // Calculate real memory usage using defines
    uint32_t flash_used = (uint32_t)&_etext - FLASH_BASE;
    uint32_t data_size = (uint32_t)&_edata - (uint32_t)&_data;
    uint32_t bss_size = (uint32_t)&_ebss - (uint32_t)&_bss;
    uint32_t ram_used = (uint32_t)&_end - TCM_BASE;
    
    // Get flash size dynamically
    uint16_t flash_total = FLASH_SIZE_KB;
    if (flash_total == 0xFFFF || flash_total == 0) {
        flash_total = 512; // Fallback
    }
    
    // Percentages in integer tenths: newlib's %f pulls in the malloc-backed
    // _dtoa path, and the heap shares scarce TCM with the stack
    uint32_t flash_pct10 = flash_used * 1000ul / (flash_total * 1024ul);
    uint32_t ram_pct10 = ram_used * 1000ul / (TCM_SIZE_KB * 1024ul);

    // Flash usage with dynamic total size
    sprintf(buf, "Flash: %lu KB / %u KB (%lu.%lu%%)",
            flash_used / 1024,
            flash_total,
            flash_pct10 / 10, flash_pct10 % 10);
    lcd_puts(20, 45, buf);

    // RAM usage (TCM memory) with dynamic size
    sprintf(buf, "TCM RAM: %lu KB / %u KB (%lu.%lu%%)",
            ram_used / 1024,
            TCM_SIZE_KB,
            ram_pct10 / 10, ram_pct10 % 10);
    lcd_puts(20, 60, buf);
    
    // Data and BSS sections
    sprintf(buf, "Data: %lu B, BSS: %lu B", data_size, bss_size);
    lcd_puts(20, 75, buf);
    
    // CPU Frequency (real value)
    sprintf(buf, "CPU Freq: %lu MHz", F_CPU / 1000000UL);
    lcd_puts(20, 90, buf);
    
    // Show uptime
    uint32_t uptime_ms = timer_ms();
    uint32_t uptime_sec = uptime_ms / 1000;
    uint32_t hours = uptime_sec / 3600;
    uint32_t minutes = (uptime_sec % 3600) / 60;
    uint32_t seconds = uptime_sec % 60;
    
    if (hours > 0) {
        sprintf(buf, "Uptime: %luh %lum %lus", hours, minutes, seconds);
    } else if (minutes > 0) {
        sprintf(buf, "Uptime: %lum %lus", minutes, seconds);
    } else {
        sprintf(buf, "Uptime: %lu.%03lus", seconds, uptime_ms % 1000);
    }
    lcd_puts(20, 105, buf);
    
    // Build information
    #ifdef GIT_HASH
    sprintf(buf, "Git: %s (%s)", GIT_HASH, GIT_BRANCH);
    lcd_puts(20, 120, buf);
    #endif
    
    #ifdef GIT_USER
    sprintf(buf, "Built by: %s", GIT_USER);
    lcd_puts(20, 135, buf);
    #else
    lcd_puts(20, 135, "Built by: Unknown Developer");
    #endif
    
    lcd_set_font(FONT_LARGE);
    lcd_puts(20, 155, "Press any button to close");
}

//-----------------------------------------------------------------------------
// General Settings Implementation
//-----------------------------------------------------------------------------

void context_menu_action_brightness_settings(void)
{
    settings_ui_show(SETTINGS_UI_BRIGHTNESS);
}

void context_menu_action_key_bindings(void)
{
    settings_ui_show(SETTINGS_UI_KEY_BINDINGS);
}

void context_menu_action_key_remapping(void)
{
    // Activate modal dialog mode
    g_context_menu.modal_dialog_active = true;
    
    // Clear screen first
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_COLOR(240, 240, 240));
    
    // Show key remapping interface
    lcd_draw_rect(10, 10, LCD_WIDTH - 21, LCD_HEIGHT - 21, LCD_COLOR(0, 0, 0));
    
    lcd_set_color(LCD_COLOR(240, 240, 240), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_LARGE);
    lcd_puts(20, 20, "Key Remapping");
    
    lcd_set_font(FONT_SMALL);
    
    lcd_puts(20, 45, "Remappable Keys:");
    lcd_puts(20, 65, "F1, F2 - Function keys");
    lcd_puts(20, 80, "SAVE, AUTO - Action keys");
    lcd_puts(20, 95, "AC_DC, 1X_10X - Toggle keys");
    
    lcd_puts(20, 115, "Protected System Keys:");
    lcd_puts(20, 135, "UP, DOWN, LEFT, RIGHT");
    lcd_puts(20, 150, "MODE, MENU, SHIFT");
    
    if (config.key_remapping_enabled) {
        lcd_puts(20, 170, "Key Remapping: ENABLED");
    } else {
        lcd_puts(20, 170, "Key Remapping: DISABLED");
    }
    
    lcd_puts(20, 195, "Press MODE to configure");
    lcd_puts(20, 210, "Press any other button to close");
}

void context_menu_action_shift_mode_toggle(void)
{
    // Toggle shift mode
    config.shift_mode_enabled = !config.shift_mode_enabled;
    
    // If disabling, also deactivate
    if (!config.shift_mode_enabled) {
        g_shift_mode.active = false;
    }
    
    // Update the menu text
    g_general_submenu[4].text = config.shift_mode_enabled ? "Shift Mode: ON" : "Shift Mode: OFF";
    
    // Save config
    // config_save(); // This would be called if config saving is implemented
    
    // Show confirmation
    lcd_fill_rect(50, 100, 220, 60, LCD_COLOR(0, 255, 0));
    lcd_set_color(LCD_COLOR(0, 255, 0), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_LARGE);
    lcd_puts(60, 110, "Shift Mode");
    if (config.shift_mode_enabled) {
        lcd_puts(80, 130, "ENABLED");
        lcd_puts(60, 145, "Double-click SHIFT");
    } else {
        lcd_puts(80, 130, "DISABLED");
    }
}

//-----------------------------------------------------------------------------
// Shift Mode Implementation
//-----------------------------------------------------------------------------

void shift_mode_init(void)
{
    memset(&g_shift_mode, 0, sizeof(g_shift_mode));
    g_shift_mode.enabled = config.shift_mode_enabled;
    // Shift state is per-session and deliberately NOT mirrored into the
    // persisted config: writing it there dirtied the config CRC on every
    // toggle, multiplying flash saves and the sector erases that stall the
    // CPU (and used to white-screen the device)
    g_shift_mode.active = false;
}

void shift_mode_task(void)
{
    // Draw shift mode indicator if active
    if (g_shift_mode.active && g_shift_mode.enabled) {
        shift_mode_draw_indicator();
    }
}

void shift_mode_handle_button(int button)
{
    if (!g_shift_mode.enabled) return;
    
    if (button & BTN_SHIFT) {
        uint32_t current_time = timer_ms();
        
        // Check for double-click (within 500ms)
        if (current_time - g_shift_mode.last_shift_press < 500) {
            g_shift_mode.shift_press_count++;
            
            if (g_shift_mode.shift_press_count >= 2) {
                // Double-click detected - toggle shift mode
                g_shift_mode.active = !g_shift_mode.active;
                g_shift_mode.double_click_detected = true;
                g_shift_mode.shift_press_count = 0;
            }
        } else {
            // Reset counter for new sequence
            g_shift_mode.shift_press_count = 1;
        }
        
        g_shift_mode.last_shift_press = current_time;
    }
    
    // If shift mode is active and any other button is pressed, deactivate
    if (g_shift_mode.active && (button & ~BTN_SHIFT) && !(button & BTN_REPEAT)) {
        // Single click on shift while active deactivates it
        if (button & BTN_SHIFT) {
            g_shift_mode.active = false;
        }
    }
}

bool shift_mode_is_active(void)
{
    return g_shift_mode.enabled && g_shift_mode.active;
}

void shift_mode_draw_indicator(void)
{
    // Draw shift mode indicator in top-right corner
    if (g_shift_mode.active && g_shift_mode.enabled) {
        lcd_fill_rect(LCD_WIDTH - 50, 5, 45, 15, LCD_COLOR(255, 0, 0));
        lcd_set_color(LCD_COLOR(255, 0, 0), LCD_COLOR(255, 255, 255));
        lcd_set_font(FONT_SMALL);
        lcd_puts(LCD_WIDTH - 45, 8, "SHIFT");
    }
}

//-----------------------------------------------------------------------------
// Key Remapping Implementation
//-----------------------------------------------------------------------------

void key_remapping_init(void)
{
    memset(&g_key_remap, 0, sizeof(g_key_remap));
    
    // Initialize key names
    g_key_remap.key_names[0] = "F1";
    g_key_remap.key_names[1] = "F2";
    g_key_remap.key_names[2] = "SAVE";
    g_key_remap.key_names[3] = "AUTO";
    g_key_remap.key_names[4] = "AC_DC";
    g_key_remap.key_names[5] = "1X_10X";
    g_key_remap.key_names[6] = "STOP";
    g_key_remap.key_names[7] = "EDGE";
    g_key_remap.key_names[8] = "50P";
    g_key_remap.key_names[9] = "TRIG_UP";
    g_key_remap.key_names[10] = "TRIG_DOWN";
    g_key_remap.key_names[11] = "TRIG";
    
    // Initialize original key values
    g_key_remap.original_keys[0] = BTN_F1;
    g_key_remap.original_keys[1] = BTN_F2;
    g_key_remap.original_keys[2] = BTN_SAVE;
    g_key_remap.original_keys[3] = BTN_AUTO;
    g_key_remap.original_keys[4] = BTN_AC_DC;
    g_key_remap.original_keys[5] = BTN_1X_10X;
    g_key_remap.original_keys[6] = BTN_STOP;
    g_key_remap.original_keys[7] = BTN_EDGE;
    g_key_remap.original_keys[8] = BTN_50P;
    g_key_remap.original_keys[9] = BTN_TRIG_UP;
    g_key_remap.original_keys[10] = BTN_TRIG_DOWN;
    g_key_remap.original_keys[11] = BTN_TRIG;
    
    // Set system keys mask (keys that cannot be remapped)
    g_key_remap.system_keys_mask = BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT | 
                                   BTN_MODE | BTN_MENU | BTN_SHIFT;
    
    // Initialize key mapping from config
    for (int i = 0; i < 12; i++) {
        if (config.key_mapping[i] == 0) {
            config.key_mapping[i] = g_key_remap.original_keys[i];
        }
    }
}

uint32_t key_remapping_translate(uint32_t button)
{
    if (!config.key_remapping_enabled) {
        return button;
    }
    
    // Don't remap system keys
    if (button & g_key_remap.system_keys_mask) {
        return button;
    }
    
    // Translate remapped keys
    uint32_t result = button;
    
    for (int i = 0; i < 12; i++) {
        if (button & g_key_remap.original_keys[i]) {
            // Remove original key and add mapped key
            result &= ~g_key_remap.original_keys[i];
            result |= config.key_mapping[i];
        }
    }
    
    return result;
}

//-----------------------------------------------------------------------------
// Settings UI System Implementation
//-----------------------------------------------------------------------------

void settings_ui_init(void)
{
    memset(&g_settings_ui, 0, sizeof(g_settings_ui));
    g_settings_ui.current_mode = SETTINGS_UI_NONE;
    g_settings_ui.active = false;
    g_settings_ui.close_timer = TIMER_DISABLE;
    timer_add(&g_settings_ui.close_timer);
}

void settings_ui_show(settings_ui_mode_t mode)
{
    g_settings_ui.current_mode = mode;
    g_settings_ui.active = true;
    g_settings_ui.dirty = true;
    g_settings_ui.close_timer = TIMER_DISABLE;
    g_settings_ui.selected_item = 0;
    
    // Initialize mode-specific values
    switch (mode) {
        case SETTINGS_UI_BRIGHTNESS:
            g_settings_ui.brightness_value = config.screen_brightness;
            g_settings_ui.brightness_changed = false;
            break;
        case SETTINGS_UI_KEY_REMAPPING:
            g_settings_ui.key_remap_mode = false;
            g_settings_ui.key_remap_index = 0;
            break;
        default:
            break;
    }
    
    // Clear screen and draw initial UI
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_COLOR(245, 245, 245));
}

void settings_ui_hide(void)
{
    g_settings_ui.active = false;
    g_settings_ui.current_mode = SETTINGS_UI_NONE;
    
    // Clear screen and restore default colors
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    lcd_set_font(FONT_LARGE);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
}

bool settings_ui_is_active(void)
{
    return g_settings_ui.active;
}

static void draw_brightness_ui(void)
{
    // Modern iOS/Android style brightness settings
    lcd_set_color(LCD_COLOR(245, 245, 245), LCD_COLOR(0, 0, 0));
    
    // Header
    lcd_fill_rect(0, 0, LCD_WIDTH, 50, LCD_COLOR(0, 122, 255));
    lcd_set_color(LCD_COLOR(0, 122, 255), LCD_COLOR(255, 255, 255));
    lcd_set_font(FONT_LARGE);
    lcd_puts(20, 15, "Screen Brightness");
    
    // Back button indicator
    lcd_set_font(FONT_SMALL);
    lcd_puts(10, 20, "<");
    
    // Main content area
    lcd_set_color(LCD_COLOR(245, 245, 245), LCD_COLOR(0, 0, 0));
    
    // Current value display
    lcd_fill_rect(20, 70, LCD_WIDTH - 40, 60, LCD_COLOR(255, 255, 255));
    lcd_draw_rect(20, 70, LCD_WIDTH - 41, 59, LCD_COLOR(200, 200, 200));
    
    lcd_set_color(LCD_COLOR(255, 255, 255), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_LARGE);
    
    char buf[32];
    sprintf(buf, "%d%%", g_settings_ui.brightness_value);
    
    // Center the percentage text
    int text_width = strlen(buf) * 8;
    int center_x = (LCD_WIDTH - text_width) / 2;
    lcd_puts(center_x, 90, buf);
    
    // Brightness slider
    int slider_x = 30;
    int slider_y = 150;
    int slider_w = LCD_WIDTH - 60;
    int slider_h = 20;
    
    // Slider track
    lcd_fill_rect(slider_x, slider_y, slider_w, slider_h, LCD_COLOR(220, 220, 220));
    lcd_draw_rect(slider_x, slider_y, slider_w - 1, slider_h - 1, LCD_COLOR(180, 180, 180));
    
    // Slider fill
    int fill_w = (slider_w - 4) * g_settings_ui.brightness_value / 100;
    if (fill_w > 0) {
        lcd_fill_rect(slider_x + 2, slider_y + 2, fill_w, slider_h - 4, LCD_COLOR(0, 122, 255));
    }
    
    // Slider thumb
    int thumb_x = slider_x + (slider_w * g_settings_ui.brightness_value / 100) - 8;
    lcd_fill_rect(thumb_x, slider_y - 5, 16, slider_h + 10, LCD_COLOR(255, 255, 255));
    lcd_draw_rect(thumb_x, slider_y - 5, 15, slider_h + 9, LCD_COLOR(0, 122, 255));
    
    // Instructions
    lcd_set_color(LCD_COLOR(245, 245, 245), LCD_COLOR(100, 100, 100));
    lcd_set_font(FONT_SMALL);
    lcd_puts(30, 190, "Use LEFT/RIGHT to adjust");
    lcd_puts(30, 205, "MODE - save, MENU - cancel");
    
    // Show if changed
    if (g_settings_ui.brightness_changed) {
        lcd_set_color(LCD_COLOR(245, 245, 245), LCD_COLOR(255, 149, 0));
        lcd_puts(200, 190, "Changed");
    }
}

static void draw_key_bindings_ui(void)
{
    // Modern settings list style
    lcd_set_color(LCD_COLOR(245, 245, 245), LCD_COLOR(0, 0, 0));
    
    // Header
    lcd_fill_rect(0, 0, LCD_WIDTH, 50, LCD_COLOR(0, 122, 255));
    lcd_set_color(LCD_COLOR(0, 122, 255), LCD_COLOR(255, 255, 255));
    lcd_set_font(FONT_LARGE);
    lcd_puts(20, 15, "Key Bindings");
    
    // Back button
    lcd_set_font(FONT_SMALL);
    lcd_puts(10, 20, "<");
    
    // Settings list
    lcd_set_color(LCD_COLOR(245, 245, 245), LCD_COLOR(0, 0, 0));
    
    const char* bindings[] = {
        "UP/DOWN - Menu navigation",
        "LEFT/RIGHT - Submenu navigation", 
        "MODE - Select/Confirm",
        "MENU - Context menu",
        "SHIFT - Modifier key",
        "F1/F2 - Function keys"
    };
    
    for (int i = 0; i < 6; i++) {
        int y = 70 + i * 25;
        
        // List item background
        lcd_fill_rect(10, y, LCD_WIDTH - 20, 23, LCD_COLOR(255, 255, 255));
        if (i < 5) {
            lcd_hline(10, LCD_WIDTH - 10, y + 23, LCD_COLOR(230, 230, 230));
        }
        
        lcd_set_color(LCD_COLOR(255, 255, 255), LCD_COLOR(0, 0, 0));
        lcd_set_font(FONT_SMALL);
        lcd_puts(20, y + 6, bindings[i]);
    }
    
    // Shift mode status
    int status_y = 220;
    lcd_fill_rect(10, status_y, LCD_WIDTH - 20, 23, LCD_COLOR(255, 255, 255));
    
    lcd_set_color(LCD_COLOR(255, 255, 255), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_SMALL);
    
    if (config.shift_mode_enabled) {
        lcd_puts(20, status_y + 6, "Shift Mode: ENABLED");
        if (shift_mode_is_active()) {
            lcd_set_color(LCD_COLOR(255, 255, 255), LCD_COLOR(255, 0, 0));
            lcd_puts(200, status_y + 6, "ACTIVE");
        }
    } else {
        lcd_puts(20, status_y + 6, "Shift Mode: DISABLED");
    }
}

static void draw_key_remapping_ui(void)
{
    // Header
    lcd_fill_rect(0, 0, LCD_WIDTH, 50, LCD_COLOR(0, 122, 255));
    lcd_set_color(LCD_COLOR(0, 122, 255), LCD_COLOR(255, 255, 255));
    lcd_set_font(FONT_LARGE);
    lcd_puts(20, 15, "Key Remapping");
    
    lcd_set_font(FONT_SMALL);
    lcd_puts(10, 20, "<");
    
    // Content
    lcd_set_color(LCD_COLOR(245, 245, 245), LCD_COLOR(0, 0, 0));
    
    // Status
    int status_y = 70;
    lcd_fill_rect(10, status_y, LCD_WIDTH - 20, 30, LCD_COLOR(255, 255, 255));
    lcd_set_color(LCD_COLOR(255, 255, 255), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_SMALL);
    
    if (config.key_remapping_enabled) {
        lcd_puts(20, status_y + 8, "Key Remapping: ENABLED");
        lcd_set_color(LCD_COLOR(255, 255, 255), LCD_COLOR(0, 200, 0));
        lcd_puts(220, status_y + 8, "ON");
    } else {
        lcd_puts(20, status_y + 8, "Key Remapping: DISABLED");
        lcd_set_color(LCD_COLOR(255, 255, 255), LCD_COLOR(200, 0, 0));
        lcd_puts(220, status_y + 8, "OFF");
    }
    
    // Available keys
    lcd_set_color(LCD_COLOR(245, 245, 245), LCD_COLOR(0, 0, 0));
    lcd_set_font(FONT_SMALL);
    lcd_puts(20, 120, "Available for remapping:");
    
    const char* remappable[] = {
        "F1, F2 - Function keys",
        "SAVE, AUTO - Action keys", 
        "AC_DC, 1X_10X - Toggle keys"
    };
    
    for (int i = 0; i < 3; i++) {
        lcd_puts(30, 140 + i * 15, remappable[i]);
    }
    
    lcd_puts(20, 200, "Protected system keys:");
    lcd_puts(30, 215, "UP/DOWN/LEFT/RIGHT MODE MENU SHIFT");
}

void settings_ui_task(void)
{
    if (!g_settings_ui.active) return;

    if (g_settings_ui.close_timer == 0) {
        g_settings_ui.close_timer = TIMER_DISABLE;
        settings_ui_hide();
        return;
    }

    // Redraw only after a state change: a full redraw bit-bangs ~30-40k
    // pixels (~5 ms) and used to run on every main-loop pass
    if (!g_settings_ui.dirty) return;
    g_settings_ui.dirty = false;

    switch (g_settings_ui.current_mode) {
        case SETTINGS_UI_BRIGHTNESS:
            draw_brightness_ui();
            break;
        case SETTINGS_UI_KEY_BINDINGS:
            draw_key_bindings_ui();
            break;
        case SETTINGS_UI_KEY_REMAPPING:
            draw_key_remapping_ui();
            break;
        default:
            break;
    }
}

void settings_ui_buttons_handler(int buttons)
{
    if (!g_settings_ui.active) return;
    
    bool repeat = (buttons & BTN_REPEAT);
    
    // Common navigation - MENU to go back
    if (buttons & BTN_MENU && !repeat) {
        settings_ui_hide();
        return;
    }
    
    switch (g_settings_ui.current_mode) {
        case SETTINGS_UI_BRIGHTNESS:
            if (buttons & BTN_LEFT) {
                if (g_settings_ui.brightness_value > 10) {
                    g_settings_ui.brightness_value -= repeat ? 5 : 1;
                    if (g_settings_ui.brightness_value < 10) {
                        g_settings_ui.brightness_value = 10;
                    }
                    g_settings_ui.brightness_changed = true;
                    g_settings_ui.dirty = true;

                    // Apply immediately for preview
                    lcd_set_backlight_level(g_settings_ui.brightness_value);
                }
            }
            else if (buttons & BTN_RIGHT) {
                if (g_settings_ui.brightness_value < 100) {
                    g_settings_ui.brightness_value += repeat ? 5 : 1;
                    if (g_settings_ui.brightness_value > 100) {
                        g_settings_ui.brightness_value = 100;
                    }
                    g_settings_ui.brightness_changed = true;
                    g_settings_ui.dirty = true;

                    // Apply immediately for preview
                    lcd_set_backlight_level(g_settings_ui.brightness_value);
                }
            }
            else if (buttons & BTN_MODE && !repeat) {
                // Save settings
                config.screen_brightness = g_settings_ui.brightness_value;
                lcd_set_backlight_level(config.screen_brightness);

                // Show confirmation briefly, then close from settings_ui_task
                // when the soft timer expires (a busy-wait here blocked the
                // whole main loop for ~30 ms)
                lcd_fill_rect(50, 100, 220, 40, LCD_COLOR(0, 200, 0));
                lcd_set_color(LCD_COLOR(0, 200, 0), LCD_COLOR(255, 255, 255));
                lcd_set_font(FONT_LARGE);
                lcd_puts(80, 110, "Saved!");

                g_settings_ui.close_timer = 500;
            }
            break;
            
        case SETTINGS_UI_KEY_BINDINGS:
            // Just informational, any button closes
            if (buttons && !repeat) {
                settings_ui_hide();
            }
            break;
            
        case SETTINGS_UI_KEY_REMAPPING:
            if (buttons & BTN_MODE && !repeat) {
                // Toggle key remapping
                config.key_remapping_enabled = !config.key_remapping_enabled;
                g_settings_ui.dirty = true;
            }
            else if (buttons && !repeat && !(buttons & BTN_MODE)) {
                settings_ui_hide();
            }
            break;
            
        default:
            break;
    }
}
