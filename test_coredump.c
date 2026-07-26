/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Test file to demonstrate coredump functionality
 * This file contains functions to trigger different types of errors for testing
 */

#include "debug_coredump.h"
#include "timer.h"
#include "lcd.h"
#include "buttons.h"

/*- Test Functions ----------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Test function to trigger a timer overflow error
void test_timer_overflow(void)
{
    COREDUMP_CAPTURE(ERROR_TYPE_TIMER_OVERFLOW, "Test timer overflow");
    error("Test Timer Overflow");
}

//-----------------------------------------------------------------------------
// Test function to trigger a general error
void test_general_error(void)
{
    COREDUMP_CAPTURE(ERROR_TYPE_GENERAL_ERROR, "Test general error");
    error("Test General Error");
}

//-----------------------------------------------------------------------------
// Test function to trigger a memory error
void test_memory_error(void)
{
    COREDUMP_CAPTURE(ERROR_TYPE_MEMORY_ERROR, "Test memory access violation");
    error("Test Memory Error");
}

//-----------------------------------------------------------------------------
// Test function to trigger an assertion failure
void test_assertion_failure(void)
{
    COREDUMP_CAPTURE(ERROR_TYPE_ASSERTION_FAILED, "Test assertion failed");
    error("Test Assertion Failed");
}

//-----------------------------------------------------------------------------
// Test function to trigger a hard fault (dangerous - use with caution!)
void test_hard_fault(void)
{
    // This will cause a hard fault by accessing invalid memory
    volatile uint32_t *invalid_ptr = (uint32_t*)0x00000000;
    *invalid_ptr = 0xDEADBEEF;  // This should trigger a hard fault
}

//-----------------------------------------------------------------------------
// Test function to show coredump viewer with sample data
void test_show_coredump_viewer(void)
{
    // Create some test coredump entries
    COREDUMP_CAPTURE(ERROR_TYPE_TIMER_OVERFLOW, "Sample timer overflow");
    COREDUMP_CAPTURE(ERROR_TYPE_GENERAL_ERROR, "Sample general error");
    COREDUMP_CAPTURE(ERROR_TYPE_MEMORY_ERROR, "Sample memory error");
    
    // Show the viewer
    debug_coredump_show_viewer();
}

//-----------------------------------------------------------------------------
// Test menu to select different error types
void test_coredump_menu(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    lcd_set_font(FONT_LARGE);
    
    lcd_puts(10, 10, "CoreDump Test Menu");
    
    lcd_set_font(FONT_SMALL);
    lcd_puts(10, 40, "1. Timer Overflow (UP)");
    lcd_puts(10, 55, "2. General Error (DOWN)");
    lcd_puts(10, 70, "3. Memory Error (LEFT)");
    lcd_puts(10, 85, "4. Assertion Failed (RIGHT)");
    lcd_puts(10, 100, "5. Hard Fault (DANGEROUS!) (F2)");
    lcd_puts(10, 115, "6. Show Viewer (F1)");
    lcd_puts(10, 130, "7. Exit (MENU)");
    
    lcd_puts(10, LCD_HEIGHT - 15, "Select test with buttons");
}

//-----------------------------------------------------------------------------
// Handle buttons for test menu
void test_coredump_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    
    if (!repeat) {
        if (buttons & BTN_UP) {
            test_timer_overflow();
        }
        else if (buttons & BTN_DOWN) {
            test_general_error();
        }
        else if (buttons & BTN_LEFT) {
            test_memory_error();
        }
        else if (buttons & BTN_RIGHT) {
            test_assertion_failure();
        }
        else if (buttons & BTN_F2) {
            // Warning: This will cause a hard fault!
            lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_RED_COLOR);
            lcd_set_color(LCD_WHITE_COLOR, LCD_RED_COLOR);
            lcd_puts(10, 50, "WARNING: Hard Fault Test!");
            lcd_puts(10, 70, "Press F2 again to confirm");
            lcd_puts(10, 90, "or any other key to cancel");
            
            // Wait for confirmation
            while (1) {
                buttons_task();
                int new_buttons = buttons_state();
                if (new_buttons & BTN_F2) {
                    test_hard_fault();  // This will cause a hard fault
                }
                else if (new_buttons) {
                    test_coredump_menu();  // Redraw menu
                    break;
                }
            }
        }
        else if (buttons & BTN_F1) {
            test_show_coredump_viewer();
        }
        else if (buttons & BTN_MENU) {
            // Exit test menu - return to main menu
            return;
        }
    }
}
