/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Menu System for Multiple Applications
 * Scalable menu system for selecting different sub-applications
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "menu_system.h"

/*- Definitions -------------------------------------------------------------*/
#define MENU_BG_COLOR           LCD_COLOR(20, 20, 40)     // Dark blue background
#define MENU_TEXT_COLOR         LCD_COLOR(255, 255, 255)  // White text
#define MENU_SELECTED_BG_COLOR  LCD_COLOR(0, 120, 215)    // Blue selection
#define MENU_SELECTED_TEXT_COLOR LCD_COLOR(255, 255, 255) // White selected text
#define MENU_BORDER_COLOR       LCD_COLOR(100, 100, 100)  // Gray border
#define MENU_TITLE_COLOR        LCD_COLOR(255, 200, 0)    // Yellow title

#define MENU_TITLE_HEIGHT       40
#define MENU_FOOTER_HEIGHT      30
#define MENU_CONTENT_Y          MENU_TITLE_HEIGHT
#define MENU_CONTENT_HEIGHT     (LCD_HEIGHT - MENU_TITLE_HEIGHT - MENU_FOOTER_HEIGHT)
#define MENU_VISIBLE_ITEMS      (MENU_CONTENT_HEIGHT / MENU_ITEM_HEIGHT)

/*- Variables ---------------------------------------------------------------*/
static menu_system_t g_menu;
static int g_menu_timer = TIMER_DISABLE;

/*- Forward Declarations ----------------------------------------------------*/
static void draw_menu_background(void);
static void draw_menu_title(void);
static void draw_menu_items(void);
static void draw_menu_footer(void);
static void draw_menu_item(int index, bool selected);
static void update_scroll_position(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void menu_system_init(void)
{
    memset(&g_menu, 0, sizeof(g_menu));
    
    g_menu.item_count = 0;
    g_menu.selected_item = 0;
    g_menu.scroll_offset = 0;
    g_menu.current_app = APP_MENU;
    g_menu.in_menu = true;
    g_menu.menu_dirty = true;
    
    // Initialize timer for menu updates
    timer_add(&g_menu_timer);
    g_menu_timer = 100; // 10 FPS for menu
}

//-----------------------------------------------------------------------------
void menu_system_register_app(app_id_t id, const char *name, const char *description,
                             void (*init_func)(void), void (*task_func)(void),
                             void (*buttons_func)(int), void (*cleanup_func)(void))
{
    if (g_menu.item_count >= MAX_MENU_ITEMS) {
        return; // Menu is full
    }
    
    menu_item_t *item = &g_menu.items[g_menu.item_count];
    item->id = id;
    item->name = name;
    item->description = description;
    item->init_func = init_func;
    item->task_func = task_func;
    item->buttons_func = buttons_func;
    item->cleanup_func = cleanup_func;
    
    g_menu.item_count++;
    g_menu.menu_dirty = true;
}

//-----------------------------------------------------------------------------
static void draw_menu_background(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, MENU_BG_COLOR);
    
    // Draw border
    lcd_draw_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, MENU_BORDER_COLOR);
    lcd_draw_rect(1, 1, LCD_WIDTH - 3, LCD_HEIGHT - 3, MENU_BORDER_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_menu_title(void)
{
    lcd_set_font(FONT_LARGE);
    lcd_set_color(MENU_BG_COLOR, MENU_TITLE_COLOR);
    
    // Center the title
    const char *title = "SELECT APPLICATION";
    int title_width = strlen(title) * 8; // Assuming 8 pixels per character for large font
    int title_x = (LCD_WIDTH - title_width) / 2;
    
    lcd_puts(title_x, 10, title);
    
    // Draw separator line
    lcd_fill_rect(MENU_MARGIN, MENU_TITLE_HEIGHT - 2, 
                  LCD_WIDTH - 2 * MENU_MARGIN, 2, MENU_BORDER_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_menu_footer(void)
{
    int footer_y = LCD_HEIGHT - MENU_FOOTER_HEIGHT;
    
    // Draw separator line
    lcd_fill_rect(MENU_MARGIN, footer_y, 
                  LCD_WIDTH - 2 * MENU_MARGIN, 2, MENU_BORDER_COLOR);
    
    lcd_set_font(FONT_SMALL);
    lcd_set_color(MENU_BG_COLOR, MENU_TEXT_COLOR);
    
    // Controls help
    lcd_puts(MENU_MARGIN, footer_y + 5, "UP/DOWN: Navigate");
    lcd_puts(MENU_MARGIN, footer_y + 15, "MODE: Select  STOP: Exit");
}

//-----------------------------------------------------------------------------
static void draw_menu_item(int index, bool selected)
{
    if (index >= g_menu.item_count) return;
    
    int item_y = MENU_CONTENT_Y + (index - g_menu.scroll_offset) * MENU_ITEM_HEIGHT;
    
    // Skip if item is not visible
    if (item_y < MENU_CONTENT_Y || item_y >= MENU_CONTENT_Y + MENU_CONTENT_HEIGHT) {
        return;
    }
    
    menu_item_t *item = &g_menu.items[index];
    
    // Draw item background
    uint16_t bg_color = selected ? MENU_SELECTED_BG_COLOR : MENU_BG_COLOR;
    uint16_t text_color = selected ? MENU_SELECTED_TEXT_COLOR : MENU_TEXT_COLOR;
    
    lcd_fill_rect(MENU_MARGIN, item_y, 
                  LCD_WIDTH - 2 * MENU_MARGIN, MENU_ITEM_HEIGHT, bg_color);
    
    if (selected) {
        // Draw selection border
        lcd_draw_rect(MENU_MARGIN, item_y, 
                      LCD_WIDTH - 2 * MENU_MARGIN - 1, MENU_ITEM_HEIGHT - 1, 
                      MENU_SELECTED_TEXT_COLOR);
    }
    
    // Draw item text
    lcd_set_font(FONT_LARGE);
    lcd_set_color(bg_color, text_color);
    lcd_puts(MENU_MARGIN + 5, item_y + 2, item->name);
    
    // Draw description if there's space
    if (item->description && strlen(item->description) > 0) {
        lcd_set_font(FONT_SMALL);
        lcd_set_color(bg_color, text_color);
        lcd_puts(MENU_MARGIN + 5, item_y + 16, item->description);
    }
}

//-----------------------------------------------------------------------------
static void draw_menu_items(void)
{
    // Calculate visible range
    int start_item = g_menu.scroll_offset;
    int end_item = start_item + MENU_VISIBLE_ITEMS;
    if (end_item > g_menu.item_count) {
        end_item = g_menu.item_count;
    }
    
    // Clear content area
    lcd_fill_rect(MENU_MARGIN, MENU_CONTENT_Y, 
                  LCD_WIDTH - 2 * MENU_MARGIN, MENU_CONTENT_HEIGHT, MENU_BG_COLOR);
    
    // Draw visible items
    for (int i = start_item; i < end_item; i++) {
        draw_menu_item(i, i == g_menu.selected_item);
    }
    
    // Draw scroll indicators if needed
    if (g_menu.item_count > MENU_VISIBLE_ITEMS) {
        lcd_set_font(FONT_SMALL);
        lcd_set_color(MENU_BG_COLOR, MENU_TEXT_COLOR);
        
        if (g_menu.scroll_offset > 0) {
            lcd_puts(LCD_WIDTH - 20, MENU_CONTENT_Y + 5, "^");
        }
        
        if (g_menu.scroll_offset + MENU_VISIBLE_ITEMS < g_menu.item_count) {
            lcd_puts(LCD_WIDTH - 20, MENU_CONTENT_Y + MENU_CONTENT_HEIGHT - 15, "v");
        }
    }
}

//-----------------------------------------------------------------------------
static void update_scroll_position(void)
{
    // Ensure selected item is visible
    if (g_menu.selected_item < g_menu.scroll_offset) {
        g_menu.scroll_offset = g_menu.selected_item;
        g_menu.menu_dirty = true;
    } else if (g_menu.selected_item >= g_menu.scroll_offset + MENU_VISIBLE_ITEMS) {
        g_menu.scroll_offset = g_menu.selected_item - MENU_VISIBLE_ITEMS + 1;
        g_menu.menu_dirty = true;
    }
}

//-----------------------------------------------------------------------------
void menu_system_show(void)
{
    g_menu.in_menu = true;
    g_menu.menu_dirty = true;
    g_menu_timer = 50; // Faster refresh when showing menu
}

//-----------------------------------------------------------------------------
void menu_system_hide(void)
{
    g_menu.in_menu = false;
    g_menu_timer = TIMER_DISABLE;
}

//-----------------------------------------------------------------------------
void menu_system_launch_app(app_id_t app_id)
{
    // Find the app in menu items
    for (int i = 0; i < g_menu.item_count; i++) {
        if (g_menu.items[i].id == app_id) {
            g_menu.current_app = app_id;
            g_menu.in_menu = false;
            
            // Initialize the app
            if (g_menu.items[i].init_func) {
                g_menu.items[i].init_func();
            }
            
            g_menu_timer = TIMER_DISABLE;
            return;
        }
    }
}

//-----------------------------------------------------------------------------
void menu_system_return_to_menu(void)
{
    // Cleanup current app
    for (int i = 0; i < g_menu.item_count; i++) {
        if (g_menu.items[i].id == g_menu.current_app) {
            if (g_menu.items[i].cleanup_func) {
                g_menu.items[i].cleanup_func();
            }
            break;
        }
    }
    
    g_menu.current_app = APP_MENU;
    menu_system_show();
}

//-----------------------------------------------------------------------------
app_id_t menu_system_get_current_app(void)
{
    return g_menu.current_app;
}

//-----------------------------------------------------------------------------
bool menu_system_is_in_menu(void)
{
    return g_menu.in_menu;
}

//-----------------------------------------------------------------------------
void menu_system_task(void)
{
    if (!g_menu.in_menu) {
        // Run current app task
        for (int i = 0; i < g_menu.item_count; i++) {
            if (g_menu.items[i].id == g_menu.current_app) {
                if (g_menu.items[i].task_func) {
                    g_menu.items[i].task_func();
                }
                break;
            }
        }
        return;
    }
    
    // Menu is active - only redraw when needed
    if (g_menu.menu_dirty) {
        draw_menu_background();
        draw_menu_title();
        draw_menu_items();
        draw_menu_footer();
        g_menu.menu_dirty = false;
    }
}

//-----------------------------------------------------------------------------
void menu_system_buttons_handler(int buttons)
{
    if (!g_menu.in_menu) {
        // Pass buttons to current app
        for (int i = 0; i < g_menu.item_count; i++) {
            if (g_menu.items[i].id == g_menu.current_app) {
                if (g_menu.items[i].buttons_func) {
                    g_menu.items[i].buttons_func(buttons);
                }
                break;
            }
        }
        return;
    }
    
    // Handle menu navigation
    bool repeat = (buttons & BTN_REPEAT);
    
    if (buttons & BTN_UP) {
        if (g_menu.selected_item > 0) {
            g_menu.selected_item--;
            update_scroll_position();
            g_menu.menu_dirty = true;
        }
    }
    else if (buttons & BTN_DOWN) {
        if (g_menu.selected_item < g_menu.item_count - 1) {
            g_menu.selected_item++;
            update_scroll_position();
            g_menu.menu_dirty = true;
        }
    }
    else if (buttons & BTN_MODE) {
        if (!repeat && g_menu.selected_item < g_menu.item_count) {
            // Launch selected app
            app_id_t selected_app = g_menu.items[g_menu.selected_item].id;
            menu_system_launch_app(selected_app);
        }
    }
    else if (buttons & BTN_STOP) {
        if (!repeat) {
            // Exit to first app or do nothing if already in menu
            if (g_menu.item_count > 0) {
                menu_system_launch_app(g_menu.items[0].id);
            }
        }
    }
}
