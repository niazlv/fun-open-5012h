/*
 * Flash Memory Viewer Implementation
 * Interactive viewer for flash memory contents with structure analysis
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "flash_viewer.h"
#include "lcd.h"
#include "buttons.h"
#include "timer.h"
#include "launcher.h"

/*- Variables ---------------------------------------------------------------*/
static bool g_flash_viewer_active = false;
static uint32_t g_current_address = FLASH_START_ADDR;
static flash_view_mode_t g_view_mode = FLASH_VIEW_HEX;
static int g_refresh_timer = 0;
static int g_section_index = 0;

// Flash memory sections (will be analyzed dynamically)
static flash_section_t g_flash_sections[16];
static int g_section_count = 0;

/*- External symbols from linker script ------------------------------------*/
extern uint32_t _etext;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;
extern uint32_t _stack_top;

/*- Local Prototypes --------------------------------------------------------*/
static void draw_hex_view(void);
static void draw_ascii_view(void);
static void draw_structure_view(void);
static void draw_disasm_view(void);
static void draw_header(void);
static void draw_footer(void);
static const char* get_view_mode_name(flash_view_mode_t mode);
static void analyze_flash_sections(void);
static const char* get_section_name(uint32_t addr);
static uint32_t read_flash_word(uint32_t addr);
static uint8_t read_flash_byte(uint32_t addr);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void flash_viewer_init(void)
{
    g_flash_viewer_active = true;
    g_current_address = FLASH_START_ADDR;
    g_view_mode = FLASH_VIEW_HEX;
    g_refresh_timer = 100;
    g_section_index = 0;
    
    // Analyze flash structure
    analyze_flash_sections();
    
    timer_add(&g_refresh_timer);
    
    // Clear screen and draw initial view
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    draw_hex_view();
}

//-----------------------------------------------------------------------------
void flash_viewer_cleanup(void)
{
    g_flash_viewer_active = false;
    timer_remove(&g_refresh_timer);
    
    // Clear screen
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
}

//-----------------------------------------------------------------------------
bool flash_viewer_is_active(void)
{
    return g_flash_viewer_active;
}

//-----------------------------------------------------------------------------
void flash_viewer_task(void)
{
    if (!g_flash_viewer_active) return;
    
    if (g_refresh_timer <= 0) {
        // Redraw current view
        switch (g_view_mode) {
            case FLASH_VIEW_HEX:
                draw_hex_view();
                break;
            case FLASH_VIEW_ASCII:
                draw_ascii_view();
                break;
            case FLASH_VIEW_STRUCTURE:
                draw_structure_view();
                break;
            case FLASH_VIEW_DISASM:
                draw_disasm_view();
                break;
            case FLASH_VIEW_COUNT:
            default:
                draw_hex_view();
                break;
        }
        g_refresh_timer = 100; // Refresh every 100ms
    }
}

//-----------------------------------------------------------------------------
static void analyze_flash_sections(void)
{
    g_section_count = 0;
    
    // Add known sections from linker script
    g_flash_sections[g_section_count++] = (flash_section_t){
        "Vector Table", FLASH_START_ADDR, 0x400, "Interrupt vectors"
    };
    
    g_flash_sections[g_section_count++] = (flash_section_t){
        "Text Section", FLASH_START_ADDR + 0x400, 
        (uint32_t)&_etext - (FLASH_START_ADDR + 0x400), "Program code"
    };
    
    g_flash_sections[g_section_count++] = (flash_section_t){
        "Data Section", (uint32_t)&_data,
        (uint32_t)&_edata - (uint32_t)&_data, "Initialized data"
    };
    
    g_flash_sections[g_section_count++] = (flash_section_t){
        "BSS Section", (uint32_t)&_bss,
        (uint32_t)&_ebss - (uint32_t)&_bss, "Uninitialized data"
    };
    
    // Scan for other patterns
    uint32_t addr = FLASH_START_ADDR + 0x1000; // Skip first 4KB
    while (addr < FLASH_START_ADDR + FLASH_SIZE && g_section_count < 15) {
        uint32_t word = read_flash_word(addr);
        
        // Look for string tables (sequences of printable strings)
        if (flash_is_ascii_printable(word & 0xFF) && 
            flash_is_ascii_printable((word >> 8) & 0xFF)) {
            uint32_t start = addr;
            uint32_t end = addr;
            
            // Find end of string section
            while (end < FLASH_START_ADDR + FLASH_SIZE) {
                uint8_t byte = read_flash_byte(end);
                if (!flash_is_ascii_printable(byte) && byte != 0) {
                    break;
                }
                end++;
                if (end - start > 1024) break; // Limit section size
            }
            
            if (end - start > 32) { // Only add if significant size
                g_flash_sections[g_section_count++] = (flash_section_t){
                    "String Data", start, end - start, "ASCII strings"
                };
                addr = end;
                continue;
            }
        }
        
        addr += 0x100; // Skip ahead
    }
}

//-----------------------------------------------------------------------------
static uint32_t read_flash_word(uint32_t addr)
{
    if (addr < FLASH_START_ADDR || addr >= FLASH_START_ADDR + FLASH_SIZE) {
        return 0xFFFFFFFF;
    }
    return *(volatile uint32_t*)addr;
}

//-----------------------------------------------------------------------------
static uint8_t read_flash_byte(uint32_t addr)
{
    if (addr < FLASH_START_ADDR || addr >= FLASH_START_ADDR + FLASH_SIZE) {
        return 0xFF;
    }
    return *(volatile uint8_t*)addr;
}

//-----------------------------------------------------------------------------
bool flash_is_ascii_printable(uint8_t byte)
{
    return (byte >= 0x20 && byte <= 0x7E) || byte == 0;
}

//-----------------------------------------------------------------------------
static const char* get_section_name(uint32_t addr)
{
    for (int i = 0; i < g_section_count; i++) {
        if (addr >= g_flash_sections[i].address && 
            addr < g_flash_sections[i].address + g_flash_sections[i].size) {
            return g_flash_sections[i].name;
        }
    }
    return "Unknown";
}

//-----------------------------------------------------------------------------
static void draw_header(void)
{
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    lcd_set_font(FONT_LARGE);
    
    char title[32];
    sprintf(title, "Flash Viewer - %s", get_view_mode_name(g_view_mode));
    lcd_puts(5, 2, title);
    
    lcd_set_font(FONT_SMALL);
    char addr_str[32];
    sprintf(addr_str, "Addr: 0x%08lX", g_current_address);
    lcd_puts(5, 20, addr_str);
    
    const char* section = get_section_name(g_current_address);
    sprintf(addr_str, "Section: %s", section);
    lcd_puts(150, 20, addr_str);
}

//-----------------------------------------------------------------------------
static void draw_footer(void)
{
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    lcd_set_font(FONT_SMALL);
    
    lcd_puts(5, LCD_HEIGHT - 25, "UP/DOWN: Scroll  LEFT/RIGHT: Page");
    lcd_puts(5, LCD_HEIGHT - 15, "F1: Mode  F2: Sections  MENU: Exit");
}

//-----------------------------------------------------------------------------
static const char* get_view_mode_name(flash_view_mode_t mode)
{
    switch (mode) {
        case FLASH_VIEW_HEX: return "Hex";
        case FLASH_VIEW_ASCII: return "ASCII";
        case FLASH_VIEW_STRUCTURE: return "Structure";
        case FLASH_VIEW_DISASM: return "Disasm";
        default: return "Unknown";
    }
}

//-----------------------------------------------------------------------------
static void draw_hex_view(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    draw_header();
    
    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    
    int y = 40;
    uint32_t addr = g_current_address;
    
    for (int line = 0; line < FLASH_LINES_PER_PAGE && y < LCD_HEIGHT - 40; line++) {
        char line_str[80];
        char hex_part[50] = "";
        char ascii_part[20] = "";
        
        // Address
        sprintf(line_str, "%08lX: ", addr);
        
        // Hex bytes
        for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
            uint8_t byte = read_flash_byte(addr + i);
            char hex_byte[4];
            sprintf(hex_byte, "%02X ", byte);
            strcat(hex_part, hex_byte);
            
            // ASCII representation
            char ascii_char = (byte >= 0x20 && byte <= 0x7E) ? byte : '.';
            ascii_part[i] = ascii_char;
        }
        ascii_part[FLASH_PAGE_SIZE] = '\0';
        
        strcat(line_str, hex_part);
        strcat(line_str, " ");
        strcat(line_str, ascii_part);
        
        lcd_puts(5, y, line_str);
        
        addr += FLASH_PAGE_SIZE;
        y += 10;
    }
    
    draw_footer();
}

//-----------------------------------------------------------------------------
static void draw_ascii_view(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    draw_header();
    
    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    
    int y = 40;
    uint32_t addr = g_current_address;
    
    for (int line = 0; line < FLASH_LINES_PER_PAGE && y < LCD_HEIGHT - 40; line++) {
        char line_str[80];
        sprintf(line_str, "%08lX: ", addr);
        
        // ASCII characters
        for (int i = 0; i < FLASH_PAGE_SIZE * 2 && strlen(line_str) < 70; i++) {
            uint8_t byte = read_flash_byte(addr + i);
            if (byte >= 0x20 && byte <= 0x7E) {
                char ascii_char[2] = {byte, '\0'};
                strcat(line_str, ascii_char);
            } else if (byte == 0) {
                strcat(line_str, "\\0");
            } else {
                strcat(line_str, ".");
            }
        }
        
        lcd_puts(5, y, line_str);
        
        addr += FLASH_PAGE_SIZE * 2;
        y += 10;
    }
    
    draw_footer();
}

//-----------------------------------------------------------------------------
static void draw_structure_view(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    draw_header();
    
    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    
    int y = 40;
    
    lcd_puts(5, y, "Flash Memory Structure:"); y += 15;
    
    for (int i = 0; i < g_section_count && y < LCD_HEIGHT - 40; i++) {
        char line[80];
        
        if (i == g_section_index) {
            lcd_set_color(LCD_WHITE_COLOR, LCD_BLUE_COLOR);
        } else {
            lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
        }
        
        sprintf(line, "%s", g_flash_sections[i].name);
        lcd_puts(10, y, line); y += 10;
        
        sprintf(line, "  Addr: 0x%08lX  Size: %lu bytes", 
                g_flash_sections[i].address, g_flash_sections[i].size);
        lcd_puts(10, y, line); y += 10;
        
        sprintf(line, "  %s", g_flash_sections[i].description);
        lcd_puts(10, y, line); y += 15;
    }
    
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    draw_footer();
}

//-----------------------------------------------------------------------------
static void draw_disasm_view(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    draw_header();
    
    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    
    int y = 40;
    uint32_t addr = g_current_address;
    
    lcd_puts(5, y, "Disassembly View (simplified):"); y += 15;
    
    for (int line = 0; line < 10 && y < LCD_HEIGHT - 40; line++) {
        char line_str[80];
        uint32_t instruction = read_flash_word(addr);
        
        sprintf(line_str, "%08lX: %08lX", addr, instruction);
        
        // Simple instruction analysis
        if ((instruction & 0xF000F000) == 0xF000F000) {
            strcat(line_str, "  ; 32-bit instruction");
        } else if ((instruction & 0xE000) == 0xE000) {
            strcat(line_str, "  ; Branch instruction");
        } else if ((instruction & 0xF800) == 0x4800) {
            strcat(line_str, "  ; Load from PC-relative");
        } else {
            strcat(line_str, "  ; ARM Thumb instruction");
        }
        
        lcd_puts(5, y, line_str);
        
        addr += 4;
        y += 12;
    }
    
    draw_footer();
}

//-----------------------------------------------------------------------------
void flash_viewer_buttons_handler(int buttons)
{
    if (!g_flash_viewer_active) return;
    
    bool repeat = (buttons & BTN_REPEAT);
    
    // Exit to menu
    if (buttons & BTN_MENU) {
        if (!repeat) {
            launcher_exit_app();
            return;
        }
    }
    
    // Change view mode
    if (buttons & BTN_F1) {
        if (!repeat) {
            g_view_mode = (g_view_mode + 1) % FLASH_VIEW_COUNT;
            g_refresh_timer = 0; // Force immediate refresh
        }
    }
    
    // Jump to sections
    if (buttons & BTN_F2) {
        if (!repeat) {
            g_view_mode = FLASH_VIEW_STRUCTURE;
            g_refresh_timer = 0;
        }
    }
    
    // Navigation
    if (g_view_mode == FLASH_VIEW_STRUCTURE) {
        // Navigate sections
        if (buttons & BTN_UP) {
            if (!repeat || g_refresh_timer <= 0) {
                if (g_section_index > 0) {
                    g_section_index--;
                    g_refresh_timer = repeat ? 5 : 20;
                }
            }
        }
        
        if (buttons & BTN_DOWN) {
            if (!repeat || g_refresh_timer <= 0) {
                if (g_section_index < g_section_count - 1) {
                    g_section_index++;
                    g_refresh_timer = repeat ? 5 : 20;
                }
            }
        }
        
        if (buttons & BTN_RIGHT) {
            if (!repeat) {
                // Jump to selected section
                g_current_address = g_flash_sections[g_section_index].address;
                g_view_mode = FLASH_VIEW_HEX;
                g_refresh_timer = 0;
            }
        }
    } else {
        // Navigate memory
        if (buttons & BTN_UP) {
            if (!repeat || g_refresh_timer <= 0) {
                if (g_current_address >= FLASH_START_ADDR + FLASH_PAGE_SIZE) {
                    g_current_address -= FLASH_PAGE_SIZE;
                    g_refresh_timer = repeat ? 5 : 20;
                }
            }
        }
        
        if (buttons & BTN_DOWN) {
            if (!repeat || g_refresh_timer <= 0) {
                if (g_current_address < FLASH_START_ADDR + FLASH_SIZE - FLASH_BYTES_PER_PAGE) {
                    g_current_address += FLASH_PAGE_SIZE;
                    g_refresh_timer = repeat ? 5 : 20;
                }
            }
        }
        
        if (buttons & BTN_LEFT) {
            if (!repeat || g_refresh_timer <= 0) {
                if (g_current_address >= FLASH_START_ADDR + FLASH_BYTES_PER_PAGE) {
                    g_current_address -= FLASH_BYTES_PER_PAGE;
                    g_refresh_timer = repeat ? 5 : 20;
                }
            }
        }
        
        if (buttons & BTN_RIGHT) {
            if (!repeat || g_refresh_timer <= 0) {
                if (g_current_address < FLASH_START_ADDR + FLASH_SIZE - FLASH_BYTES_PER_PAGE) {
                    g_current_address += FLASH_BYTES_PER_PAGE;
                    g_refresh_timer = repeat ? 5 : 20;
                }
            }
        }
    }
}

//-----------------------------------------------------------------------------
const flash_section_t* flash_get_sections(int* count)
{
    if (count) *count = g_section_count;
    return g_flash_sections;
}
