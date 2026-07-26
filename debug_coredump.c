/*
 * Debug Coredump System Implementation
 * Provides stack trace and error analysis functionality
 */

#include <stdio.h>
#include <string.h>
#include "debug_coredump.h"
#include "lcd.h"
#include "buttons.h"
#include "timer.h"
#include "launcher.h"
#include "gd32f4xx.h"

/*- Variables ---------------------------------------------------------------*/
static coredump_entry_t g_coredump_entries[MAX_COREDUMP_ENTRIES];
static int g_coredump_count = 0;
static int g_coredump_write_index = 0;
static bool g_viewer_active = false;
static int g_viewer_current_entry = 0;
static int g_viewer_current_page = 0;
static int g_viewer_timer = 0;

// Stack/heap bounds provided by the linker (both in TCM)
extern char _end;
extern char _stack_top;

/*- Local Prototypes --------------------------------------------------------*/
static void capture_stack_trace(stack_frame_t* trace, int* depth, uint32_t sp, uint32_t lr);
static uint32_t get_stack_free_space(void);
static uint32_t get_heap_free_space(void);
static void draw_coredump_list(void);
static void draw_coredump_details(int entry_index);
static void draw_stack_trace(int entry_index);
static void draw_registers(int entry_index);
static const char* get_error_type_string(error_type_t type);
static void format_hex_value(char* buffer, uint32_t value);

// Flash storage functions
static bool flash_erase_sector(uint32_t sector);
static bool flash_write_data(uint32_t addr, const void* data, uint32_t size);
static bool flash_read_data(uint32_t addr, void* data, uint32_t size);
static void save_coredumps_to_flash(void);
static void load_coredumps_from_flash(void);
static uint32_t calculate_crc32(const void* data, uint32_t size);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void debug_coredump_init(void)
{
    g_coredump_count = 0;
    g_coredump_write_index = 0;
    g_viewer_active = false;
    g_viewer_current_entry = 0;
    g_viewer_current_page = 0;
    
    memset(g_coredump_entries, 0, sizeof(g_coredump_entries));
    
    timer_add(&g_viewer_timer);
}

//-----------------------------------------------------------------------------
static void capture_stack_trace(stack_frame_t* trace, int* depth, uint32_t sp, uint32_t lr)
{
    *depth = 0;
    
    // Simple stack unwinding - this is basic and may need improvement
    uint32_t* stack_ptr = (uint32_t*)sp;
    uint32_t current_lr = lr;
    
    for (int i = 0; i < MAX_STACK_TRACE_DEPTH && *depth < MAX_STACK_TRACE_DEPTH; i++) {
        // The stack lives in TCM (see linker script), not in main SRAM;
        // checking the SRAM range made every trace come back empty
        if ((uint32_t)stack_ptr < (uint32_t)&_end || (uint32_t)stack_ptr >= (uint32_t)&_stack_top) {
            break;
        }
        
        trace[*depth].pc = current_lr;
        trace[*depth].lr = current_lr;
        trace[*depth].sp = (uint32_t)stack_ptr;
        trace[*depth].function = current_lr & 0xFFFFFFFE; // Clear thumb bit
        
        (*depth)++;
        
        // Try to find next frame (this is simplified)
        // In a real implementation, you'd need proper frame pointer unwinding
        stack_ptr += 4; // Move up the stack
        
        // Look for a potential return address
        for (int j = 0; j < 8 && (uint32_t)stack_ptr < 0x20020000; j++) {
            uint32_t potential_lr = *stack_ptr;
            // Check if this looks like a valid code address
            if (potential_lr >= 0x08000000 && potential_lr < 0x08100000) {
                current_lr = potential_lr;
                break;
            }
            stack_ptr++;
        }
    }
}

//-----------------------------------------------------------------------------
static uint32_t get_stack_free_space(void)
{
    // Simple stack usage estimation
    uint32_t current_sp;
    __asm volatile ("mov %0, sp" : "=r" (current_sp));

    // Free space is what is left between the top of the heap and the
    // current stack pointer (both in TCM)
    if (current_sp < (uint32_t)&_end)
        return 0;

    return current_sp - (uint32_t)&_end;
}

//-----------------------------------------------------------------------------
static uint32_t get_heap_free_space(void)
{
    // This would need to be implemented based on your heap allocator
    // For now, return a placeholder
    return 0x8000; // 32KB placeholder
}

//-----------------------------------------------------------------------------
void debug_coredump_capture(error_type_t type, const char* message, 
                           uint32_t pc, uint32_t lr, uint32_t sp)
{
    coredump_entry_t* entry = &g_coredump_entries[g_coredump_write_index];
    
    // Clear the entry
    memset(entry, 0, sizeof(coredump_entry_t));
    
    // Basic information
    entry->type = type;
    strncpy(entry->message, message, MAX_ERROR_MESSAGE_LEN - 1);
    entry->message[MAX_ERROR_MESSAGE_LEN - 1] = '\0';
    entry->timestamp = timer_ms();
    entry->error_pc = pc;
    entry->error_lr = lr;
    entry->error_sp = sp;
    
    // Capture CPU registers (simplified - in real handler you'd get these from stack)
    entry->r0 = 0;  // Would be captured from exception stack frame
    entry->r1 = 0;
    entry->r2 = 0;
    entry->r3 = 0;
    entry->r12 = 0;
    entry->psr = 0;
    
    // Fault status registers
    entry->cfsr = SCB->CFSR;
    entry->hfsr = SCB->HFSR;
    entry->dfsr = SCB->DFSR;
    entry->afsr = SCB->AFSR;
    entry->mmar = SCB->MMFAR;
    entry->bfar = SCB->BFAR;
    
    // Capture stack trace
    capture_stack_trace(entry->stack_trace, &entry->stack_depth, sp, lr);
    
    // System context
    entry->heap_free = get_heap_free_space();
    entry->stack_free = get_stack_free_space();
    entry->active_timers = 0; // Would need timer system integration
    entry->system_state = 0;  // Could capture current app state, etc.
    
    // Update counters
    g_coredump_write_index = (g_coredump_write_index + 1) % MAX_COREDUMP_ENTRIES;
    if (g_coredump_count < MAX_COREDUMP_ENTRIES) {
        g_coredump_count++;
    }
    
    // Save to flash (currently just placeholder)
    save_coredumps_to_flash();
}

//-----------------------------------------------------------------------------
void debug_coredump_capture_hard_fault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t *sp = (uint32_t *)((lr & 4) ? psp : msp);
    coredump_entry_t* entry = &g_coredump_entries[g_coredump_write_index];
    
    // Clear the entry
    memset(entry, 0, sizeof(coredump_entry_t));
    
    // Basic information
    entry->type = ERROR_TYPE_HARD_FAULT;
    strcpy(entry->message, "Hard Fault Exception");
    entry->timestamp = timer_ms();
    entry->error_pc = sp[6];  // PC from exception stack frame
    entry->error_lr = lr;
    entry->error_sp = (lr & 4) ? psp : msp;
    
    // CPU registers from exception stack frame
    entry->r0 = sp[0];
    entry->r1 = sp[1];
    entry->r2 = sp[2];
    entry->r3 = sp[3];
    entry->r12 = sp[4];
    entry->psr = sp[7];
    
    // Fault status registers
    entry->cfsr = SCB->CFSR;
    entry->hfsr = SCB->HFSR;
    entry->dfsr = SCB->DFSR;
    entry->afsr = SCB->AFSR;
    entry->mmar = SCB->MMFAR;
    entry->bfar = SCB->BFAR;
    
    // Capture stack trace
    capture_stack_trace(entry->stack_trace, &entry->stack_depth, 
                       (lr & 4) ? psp : msp, sp[5]);
    
    // System context
    entry->heap_free = get_heap_free_space();
    entry->stack_free = get_stack_free_space();
    entry->active_timers = 0;
    entry->system_state = 0;
    
    // Update counters
    g_coredump_write_index = (g_coredump_write_index + 1) % MAX_COREDUMP_ENTRIES;
    if (g_coredump_count < MAX_COREDUMP_ENTRIES) {
        g_coredump_count++;
    }
    
    // Save to flash (currently just placeholder)
    save_coredumps_to_flash();
}

//-----------------------------------------------------------------------------
void debug_coredump_show_viewer(void)
{
    g_viewer_active = true;
    g_viewer_current_entry = 0;
    g_viewer_current_page = 0;
    g_viewer_timer = 100; // Refresh rate
    
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    draw_coredump_list();
}

//-----------------------------------------------------------------------------
static const char* get_error_type_string(error_type_t type)
{
    switch (type) {
        case ERROR_TYPE_TIMER_OVERFLOW: return "Timer Overflow";
        case ERROR_TYPE_HARD_FAULT: return "Hard Fault";
        case ERROR_TYPE_GENERAL_ERROR: return "General Error";
        case ERROR_TYPE_ASSERTION_FAILED: return "Assertion Failed";
        case ERROR_TYPE_MEMORY_ERROR: return "Memory Error";
        default: return "Unknown Error";
    }
}

//-----------------------------------------------------------------------------
static void format_hex_value(char* buffer, uint32_t value)
{
    sprintf(buffer, "0x%08lX", value);
}

//-----------------------------------------------------------------------------
static void draw_coredump_list(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    lcd_set_font(FONT_LARGE);
    
    char title[32];
    sprintf(title, "CoreDump (%d entries)", g_coredump_count);
    lcd_puts(10, 5, title);
    
    if (g_coredump_count == 0) {
        lcd_puts(10, 30, "No crash dumps available");
        lcd_puts(10, 50, "F1: Exit");
        return;
    }
    
    lcd_set_font(FONT_SMALL);
    
    int y = 25;
    for (int i = 0; i < g_coredump_count && y < LCD_HEIGHT - 20; i++) {
        int entry_idx = (g_coredump_write_index - g_coredump_count + i + MAX_COREDUMP_ENTRIES) % MAX_COREDUMP_ENTRIES;
        coredump_entry_t* entry = &g_coredump_entries[entry_idx];
        
        if (i == g_viewer_current_entry) {
            lcd_set_color(LCD_WHITE_COLOR, LCD_BLUE_COLOR);
        } else {
            lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
        }
        
        char line[64];
        sprintf(line, "%d. %s", i + 1, get_error_type_string(entry->type));
        lcd_puts(10, y, line);
        
        sprintf(line, "   PC:0x%08lX T:%lu", entry->error_pc, entry->timestamp);
        lcd_puts(10, y + 10, line);
        
        y += 25;
    }
    
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    lcd_puts(10, LCD_HEIGHT - 25, "UP/DOWN: Select  RIGHT: Details");
    lcd_puts(10, LCD_HEIGHT - 15, "F1: Exit  F2: Add Test Dump");
}

//-----------------------------------------------------------------------------
static void draw_coredump_details(int entry_index)
{
    if (entry_index >= g_coredump_count) return;
    
    int entry_idx = (g_coredump_write_index - g_coredump_count + entry_index + MAX_COREDUMP_ENTRIES) % MAX_COREDUMP_ENTRIES;
    coredump_entry_t* entry = &g_coredump_entries[entry_idx];
    
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    lcd_set_font(FONT_LARGE);
    
    char title[32];
    sprintf(title, "Crash #%d Details", entry_index + 1);
    lcd_puts(10, 5, title);
    
    lcd_set_font(FONT_SMALL);
    
    int y = 25;
    
    // Error type and message
    lcd_puts(10, y, "Type:"); y += 10;
    lcd_puts(20, y, get_error_type_string(entry->type)); y += 15;
    
    lcd_puts(10, y, "Message:"); y += 10;
    lcd_puts(20, y, entry->message); y += 15;
    
    // Timing and location
    char buffer[64];
    sprintf(buffer, "Time: %lu ms", entry->timestamp);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "PC:   0x%08lX", entry->error_pc);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "LR:   0x%08lX", entry->error_lr);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "SP:   0x%08lX", entry->error_sp);
    lcd_puts(10, y, buffer); y += 15;
    
    // System state
    sprintf(buffer, "Stack Free: %lu bytes", entry->stack_free);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "Heap Free:  %lu bytes", entry->heap_free);
    lcd_puts(10, y, buffer); y += 15;
    
    lcd_puts(10, LCD_HEIGHT - 25, "LEFT: Back  UP/DOWN: Regs/Stack");
    lcd_puts(10, LCD_HEIGHT - 15, "F1: Exit");
}

//-----------------------------------------------------------------------------
static void draw_registers(int entry_index)
{
    if (entry_index >= g_coredump_count) return;
    
    int entry_idx = (g_coredump_write_index - g_coredump_count + entry_index + MAX_COREDUMP_ENTRIES) % MAX_COREDUMP_ENTRIES;
    coredump_entry_t* entry = &g_coredump_entries[entry_idx];
    
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    lcd_set_font(FONT_LARGE);
    
    char title[32];
    sprintf(title, "Crash #%d Registers", entry_index + 1);
    lcd_puts(10, 5, title);
    
    lcd_set_font(FONT_SMALL);
    
    int y = 25;
    char buffer[32];
    
    // CPU Registers
    sprintf(buffer, "R0:  0x%08lX", entry->r0);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "R1:  0x%08lX", entry->r1);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "R2:  0x%08lX", entry->r2);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "R3:  0x%08lX", entry->r3);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "R12: 0x%08lX", entry->r12);
    lcd_puts(10, y, buffer); y += 10;
    
    sprintf(buffer, "PSR: 0x%08lX", entry->psr);
    lcd_puts(10, y, buffer); y += 15;
    
    // Fault Status Registers
    lcd_puts(10, y, "Fault Status:"); y += 10;
    
    sprintf(buffer, "CFSR: 0x%08lX", entry->cfsr);
    lcd_puts(20, y, buffer); y += 10;
    
    sprintf(buffer, "HFSR: 0x%08lX", entry->hfsr);
    lcd_puts(20, y, buffer); y += 10;
    
    sprintf(buffer, "MMAR: 0x%08lX", entry->mmar);
    lcd_puts(20, y, buffer); y += 10;
    
    sprintf(buffer, "BFAR: 0x%08lX", entry->bfar);
    lcd_puts(20, y, buffer); y += 10;
    
    lcd_puts(10, LCD_HEIGHT - 25, "LEFT: Back  UP/DOWN: Details/Stack");
    lcd_puts(10, LCD_HEIGHT - 15, "F1: Exit");
}

//-----------------------------------------------------------------------------
static void draw_stack_trace(int entry_index)
{
    if (entry_index >= g_coredump_count) return;
    
    int entry_idx = (g_coredump_write_index - g_coredump_count + entry_index + MAX_COREDUMP_ENTRIES) % MAX_COREDUMP_ENTRIES;
    coredump_entry_t* entry = &g_coredump_entries[entry_idx];
    
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    lcd_set_font(FONT_LARGE);
    
    char title[32];
    sprintf(title, "Crash #%d Stack", entry_index + 1);
    lcd_puts(10, 5, title);
    
    lcd_set_font(FONT_SMALL);
    
    int y = 25;
    char buffer[64];
    
    sprintf(buffer, "Stack Depth: %d frames", entry->stack_depth);
    lcd_puts(10, y, buffer); y += 15;
    
    for (int i = 0; i < entry->stack_depth && y < LCD_HEIGHT - 30; i++) {
        sprintf(buffer, "#%d PC: 0x%08lX", i, entry->stack_trace[i].pc);
        lcd_puts(10, y, buffer); y += 10;
        
        sprintf(buffer, "   SP: 0x%08lX", entry->stack_trace[i].sp);
        lcd_puts(10, y, buffer); y += 12;
    }
    
    lcd_puts(10, LCD_HEIGHT - 25, "LEFT: Back  UP/DOWN: Details/Regs");
    lcd_puts(10, LCD_HEIGHT - 15, "F1: Exit");
}

//-----------------------------------------------------------------------------
void debug_coredump_viewer_buttons_handler(int buttons)
{
    if (!g_viewer_active) return;
    
    bool repeat = (buttons & BTN_REPEAT);
    
    if (buttons & BTN_F1) {
        if (!repeat) {
            g_viewer_active = false;
            return;
        }
    }
    
    // F2 to create a test coredump
    if (buttons & BTN_F2) {
        if (!repeat) {
            uint32_t pc, lr, sp;
            __asm volatile ("mov %0, pc" : "=r" (pc));
            __asm volatile ("mov %0, lr" : "=r" (lr));
            __asm volatile ("mov %0, sp" : "=r" (sp));
            
            // Create different types of test errors
            static int test_type = 0;
            switch (test_type % 4) {
                case 0:
                    debug_coredump_capture(ERROR_TYPE_TIMER_OVERFLOW, "Test timer overflow", pc, lr, sp);
                    break;
                case 1:
                    debug_coredump_capture(ERROR_TYPE_MEMORY_ERROR, "Test memory error", pc, lr, sp);
                    break;
                case 2:
                    debug_coredump_capture(ERROR_TYPE_ASSERTION_FAILED, "Test assertion failed", pc, lr, sp);
                    break;
                case 3:
                    debug_coredump_capture(ERROR_TYPE_GENERAL_ERROR, "Test general error", pc, lr, sp);
                    break;
            }
            test_type++;
            
            // Refresh the list view
            if (g_viewer_current_page == 0) {
                draw_coredump_list();
            }
        }
    }
    
    if (g_viewer_current_page == 0) { // List view
        if (buttons & BTN_UP) {
            if (!repeat || g_viewer_timer <= 0) {
                if (g_viewer_current_entry > 0) {
                    g_viewer_current_entry--;
                    draw_coredump_list();
                }
                g_viewer_timer = repeat ? 5 : 20;
            }
        }
        
        if (buttons & BTN_DOWN) {
            if (!repeat || g_viewer_timer <= 0) {
                if (g_viewer_current_entry < g_coredump_count - 1) {
                    g_viewer_current_entry++;
                    draw_coredump_list();
                }
                g_viewer_timer = repeat ? 5 : 20;
            }
        }
        
        if (buttons & BTN_RIGHT) {
            if (!repeat && g_coredump_count > 0) {
                g_viewer_current_page = 1; // Details view
                draw_coredump_details(g_viewer_current_entry);
            }
        }
    } else { // Detail views
        if (buttons & BTN_LEFT) {
            if (!repeat) {
                g_viewer_current_page = 0; // Back to list
                draw_coredump_list();
            }
        }
        
        if (buttons & BTN_UP) {
            if (!repeat) {
                g_viewer_current_page = (g_viewer_current_page == 1) ? 3 : g_viewer_current_page - 1;
                if (g_viewer_current_page == 1) draw_coredump_details(g_viewer_current_entry);
                else if (g_viewer_current_page == 2) draw_registers(g_viewer_current_entry);
                else if (g_viewer_current_page == 3) draw_stack_trace(g_viewer_current_entry);
            }
        }
        
        if (buttons & BTN_DOWN) {
            if (!repeat) {
                g_viewer_current_page = (g_viewer_current_page == 3) ? 1 : g_viewer_current_page + 1;
                if (g_viewer_current_page == 1) draw_coredump_details(g_viewer_current_entry);
                else if (g_viewer_current_page == 2) draw_registers(g_viewer_current_entry);
                else if (g_viewer_current_page == 3) draw_stack_trace(g_viewer_current_entry);
            }
        }
    }
}

//-----------------------------------------------------------------------------
bool debug_coredump_viewer_is_active(void)
{
    return g_viewer_active;
}

//-----------------------------------------------------------------------------
void debug_coredump_viewer_task(void)
{
    // g_viewer_timer is registered with timer_add() and already counts down
    // in milliseconds; decrementing it once per loop pass here made the
    // repeat throttle expire in microseconds
    (void)g_viewer_active;
}

//-----------------------------------------------------------------------------
int debug_coredump_get_count(void)
{
    return g_coredump_count;
}

//-----------------------------------------------------------------------------
void debug_coredump_clear_all(void)
{
    g_coredump_count = 0;
    g_coredump_write_index = 0;
    memset(g_coredump_entries, 0, sizeof(g_coredump_entries));
}

//-----------------------------------------------------------------------------
// Application wrapper functions for menu system
//-----------------------------------------------------------------------------

void coredump_app_init(void)
{
    // If no coredumps exist, create a sample one for demonstration
    if (g_coredump_count == 0) {
        // Create a sample coredump entry
        uint32_t pc, lr, sp;
        __asm volatile ("mov %0, pc" : "=r" (pc));
        __asm volatile ("mov %0, lr" : "=r" (lr));
        __asm volatile ("mov %0, sp" : "=r" (sp));
        
        debug_coredump_capture(ERROR_TYPE_GENERAL_ERROR, "Sample error for demo", pc, lr, sp);
    }
    
    debug_coredump_show_viewer();
}

//-----------------------------------------------------------------------------
void coredump_app_task(void)
{
    debug_coredump_viewer_task();
}

//-----------------------------------------------------------------------------
void coredump_app_buttons_handler(int buttons)
{
    debug_coredump_viewer_buttons_handler(buttons);
    
    // If viewer is no longer active, return to menu
    if (!debug_coredump_viewer_is_active()) {
        launcher_exit_app();
    }
}

//-----------------------------------------------------------------------------
void coredump_app_cleanup(void)
{
    // Cleanup is handled by the viewer itself
    g_viewer_active = false;
}

//-----------------------------------------------------------------------------
// Flash storage functions
//-----------------------------------------------------------------------------

static uint32_t calculate_crc32(const void* data, uint32_t size)
{
    // Simple checksum instead of CRC32 for now
    const uint8_t* bytes = (const uint8_t*)data;
    uint32_t checksum = 0;
    
    for (uint32_t i = 0; i < size; i++) {
        checksum += bytes[i];
        checksum = (checksum << 1) | (checksum >> 31); // Rotate left
    }
    
    return checksum;
}

//-----------------------------------------------------------------------------
static bool flash_erase_sector(uint32_t sector)
{
    // For now, just return true - flash operations would need proper implementation
    // with correct GD32F4xx flash controller registers
    (void)sector;
    return true;
}

//-----------------------------------------------------------------------------
static bool flash_write_data(uint32_t addr, const void* data, uint32_t size)
{
    // For now, just return true - flash operations would need proper implementation
    // with correct GD32F4xx flash controller registers
    (void)addr;
    (void)data;
    (void)size;
    return true;
}

//-----------------------------------------------------------------------------
static bool flash_read_data(uint32_t addr, void* data, uint32_t size)
{
    memcpy(data, (const void*)addr, size);
    return true;
}

//-----------------------------------------------------------------------------
static void save_coredumps_to_flash(void)
{
    // static, not on the stack: this runs during fault handling, where the
    // remaining stack headroom is exactly what cannot be trusted
    static coredump_flash_storage_t storage;

    // Prepare storage structure
    storage.magic = COREDUMP_MAGIC;
    storage.count = g_coredump_count;
    storage.write_index = g_coredump_write_index;
    
    // Copy entries
    memcpy(storage.entries, g_coredump_entries, sizeof(g_coredump_entries));
    
    // Calculate CRC32 (excluding the CRC field itself)
    storage.crc32 = calculate_crc32(&storage, sizeof(storage) - sizeof(storage.crc32));
    
    // Erase flash sector
    if (!flash_erase_sector(COREDUMP_FLASH_SECTOR)) {
        return; // Failed to erase
    }
    
    // Write to flash
    flash_write_data(COREDUMP_FLASH_ADDR, &storage, sizeof(storage));
}

//-----------------------------------------------------------------------------
static void load_coredumps_from_flash(void)
{
    coredump_flash_storage_t storage;
    
    // Read from flash
    if (!flash_read_data(COREDUMP_FLASH_ADDR, &storage, sizeof(storage))) {
        return; // Failed to read
    }
    
    // Check magic number
    if (storage.magic != COREDUMP_MAGIC) {
        return; // Invalid data
    }
    
    // Verify CRC32
    uint32_t calculated_crc = calculate_crc32(&storage, sizeof(storage) - sizeof(storage.crc32));
    if (calculated_crc != storage.crc32) {
        return; // Corrupted data
    }
    
    // Validate counts
    if (storage.count > MAX_COREDUMP_ENTRIES || storage.write_index >= MAX_COREDUMP_ENTRIES) {
        return; // Invalid data
    }
    
    // Load data
    g_coredump_count = storage.count;
    g_coredump_write_index = storage.write_index;
    memcpy(g_coredump_entries, storage.entries, sizeof(g_coredump_entries));
}
