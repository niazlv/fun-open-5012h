/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Flash Memory Viewer
 *
 * Hex, ASCII, layout and Thumb views over the internal flash. The screen is
 * repainted only when something changed: the first version re-rendered all
 * 320x240 pixels ten times a second whether or not anything had moved.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "buttons.h"
#include "timer.h"
#include "utils.h"
#include "ui.h"
#include "menu_widget.h"
#include "flash_viewer.h"

/*- Definitions -------------------------------------------------------------*/
#define FLASH_SIZE_REG      0x1FFF7A22  // device flash size in KB, 16-bit

#define HEADER_H            30
#define FOOTER_H            22
#define BODY_Y              32
#define BODY_H              (LCD_HEIGHT - BODY_Y - FOOTER_H)
#define LINE_H              10
#define LINES_PER_PAGE      (BODY_H / LINE_H)

#define HEX_BYTES_PER_LINE  8           // 43 chars: fits the 320 px panel
#define ASCII_CHARS_PER_LINE 40
#define THUMB_BYTES_PER_LINE 2

#define BG                  LCD_BLACK_COLOR
#define FG                  LCD_COLOR(220, 220, 220)
#define HEADER_BG           LCD_COLOR(20, 20, 40)
#define ACCENT              LCD_COLOR(255, 200, 0)
#define SEL_BG              LCD_COLOR(0, 120, 215)

/*- Variables ---------------------------------------------------------------*/
static uint32_t g_current_address = FLASH_START_ADDR;
static uint32_t g_flash_end = FLASH_START_ADDR + FLASH_SIZE;
static int g_view_mode = FLASH_VIEW_HEX;
static int g_section_index = 0;
static int g_section_scroll = 0;
static bool g_dirty = true;

static flash_section_t g_flash_sections[FLASH_MAX_SECTIONS];
static int g_section_count = 0;

/*- External symbols from linker script ------------------------------------*/
extern uint32_t _etext;   // end of .text in flash
extern uint32_t __etext;  // end of everything in flash, .data image included

/*- Local Prototypes --------------------------------------------------------*/
static void analyze_flash_sections(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint8_t read_flash_byte(uint32_t addr)
{
    if (addr < FLASH_START_ADDR || addr >= g_flash_end)
        return 0xFF;

    return *(volatile uint8_t *)addr;
}

//-----------------------------------------------------------------------------
static uint16_t read_flash_half(uint32_t addr)
{
    return (uint16_t)(read_flash_byte(addr) | (read_flash_byte(addr + 1) << 8));
}

//-----------------------------------------------------------------------------
static uint32_t read_flash_word(uint32_t addr)
{
    return (uint32_t)read_flash_half(addr) | ((uint32_t)read_flash_half(addr + 2) << 16);
}

//-----------------------------------------------------------------------------
static bool is_printable(uint8_t byte)
{
    return byte >= 0x20 && byte <= 0x7E;
}

//-----------------------------------------------------------------------------
static int bytes_per_line(void)
{
    switch (g_view_mode) {
        case FLASH_VIEW_ASCII: return ASCII_CHARS_PER_LINE;
        case FLASH_VIEW_THUMB: return THUMB_BYTES_PER_LINE;
        default:               return HEX_BYTES_PER_LINE;
    }
}

//-----------------------------------------------------------------------------
// Signed, so that stepping back from the first page clamps to the start
// instead of wrapping around to the end of the array
static void set_address(int64_t addr)
{
    int64_t step = bytes_per_line();
    int64_t last = (int64_t)g_flash_end - step * LINES_PER_PAGE;
    uint32_t offset;

    if (last < FLASH_START_ADDR)
        last = FLASH_START_ADDR;

    if (addr < FLASH_START_ADDR)
        addr = FLASH_START_ADDR;

    if (addr > last)
        addr = last;

    offset = (uint32_t)addr - FLASH_START_ADDR;

    g_current_address = FLASH_START_ADDR + (offset / (uint32_t)step) * (uint32_t)step;

    g_dirty = true;
}

//-----------------------------------------------------------------------------
static const char *view_mode_name(void)
{
    switch (g_view_mode) {
        case FLASH_VIEW_HEX:       return "Hex";
        case FLASH_VIEW_ASCII:     return "ASCII";
        case FLASH_VIEW_STRUCTURE: return "Layout";
        case FLASH_VIEW_THUMB:     return "Thumb";
        default:                   return "?";
    }
}

//-----------------------------------------------------------------------------
static const char *section_name_at(uint32_t addr)
{
    for (int i = 0; i < g_section_count; i++) {
        if (addr >= g_flash_sections[i].address &&
            addr < g_flash_sections[i].address + g_flash_sections[i].size)
            return g_flash_sections[i].name;
    }

    return "-";
}

//-----------------------------------------------------------------------------
// The vector table is the run of plausible Thumb handler addresses at the
// very start of flash; the linker script does not give it a symbol
static uint32_t detect_vector_table_end(void)
{
    uint32_t addr = FLASH_START_ADDR + 4; // [0] is the initial stack pointer

    while (addr < FLASH_START_ADDR + 0x800) {
        uint32_t v = read_flash_word(addr);

        if (0 != v && !(v >= FLASH_START_ADDR && v < g_flash_end && (v & 1)))
            break;

        addr += 4;
    }

    return addr;
}

//-----------------------------------------------------------------------------
static void add_section(const char *name, uint32_t addr, uint32_t size,
    const char *desc)
{
    if (g_section_count >= FLASH_MAX_SECTIONS || 0 == size)
        return;

    g_flash_sections[g_section_count].name = name;
    g_flash_sections[g_section_count].address = addr;
    g_flash_sections[g_section_count].size = size;
    g_flash_sections[g_section_count].description = desc;
    g_section_count++;
}

//-----------------------------------------------------------------------------
static void analyze_flash_sections(void)
{
    uint32_t text_end = (uint32_t)&_etext;
    uint32_t flash_end = (uint32_t)&__etext;
    uint32_t vt_end = detect_vector_table_end();

    g_section_count = 0;

    // Descriptions are kept short: the second row of a layout entry is
    // 22 characters of address before the text, and the panel holds 52
    add_section("Vector table", FLASH_START_ADDR, vt_end - FLASH_START_ADDR,
        "SP and IRQ handlers");

    if (text_end > vt_end)
        add_section("Code + rodata", vt_end, text_end - vt_end,
            ".text and constants");

    if (flash_end > text_end)
        add_section(".data image", text_end, flash_end - text_end,
            "Initial RAM values");

    if (g_flash_end > flash_end)
        add_section("Erased", flash_end, g_flash_end - flash_end,
            "Blank, reads as 0xFF");

    // String pools inside the image: a run of at least 24 printable
    // characters. Treating 0x00 as printable, as the first version did,
    // matched every stretch of zero padding.
    uint32_t addr = vt_end;

    while (addr < flash_end && g_section_count < FLASH_MAX_SECTIONS) {
        if (!is_printable(read_flash_byte(addr))) {
            addr += 4;
            continue;
        }

        uint32_t start = addr;
        uint32_t end = addr;
        int printable = 0;

        while (end < flash_end && end - start < 4096) {
            uint8_t b = read_flash_byte(end);

            if (is_printable(b))
                printable++;
            else if (0 != b)
                break;

            end++;
        }

        if (printable >= 24)
            add_section("String data", start, end - start, "ASCII text pool");

        addr = (end > start) ? end : start + 4;
    }
}

//-----------------------------------------------------------------------------
static void draw_header(void)
{
    char buf[64];

    lcd_fill_rect(0, 0, LCD_WIDTH, HEADER_H, HEADER_BG);
    lcd_fill_rect(0, HEADER_H - 1, LCD_WIDTH, 1, FG);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(HEADER_BG, ACCENT);
    snprintf(buf, sizeof(buf), "FLASH VIEWER - %s", view_mode_name());
    lcd_puts(5, 5, buf);

    lcd_set_color(HEADER_BG, FG);

    if (FLASH_VIEW_STRUCTURE == g_view_mode) {
        snprintf(buf, sizeof(buf), "%d regions   %lu KB total", g_section_count,
            (unsigned long)((g_flash_end - FLASH_START_ADDR) / 1024));
    }
    else {
        snprintf(buf, sizeof(buf), "0x%08lX   %s",
            (unsigned long)g_current_address, section_name_at(g_current_address));
    }

    lcd_puts(5, 17, buf);
}

//-----------------------------------------------------------------------------
static void draw_footer(const char *l1, const char *l2)
{
    lcd_fill_rect(0, LCD_HEIGHT - FOOTER_H, LCD_WIDTH, FOOTER_H, HEADER_BG);
    lcd_fill_rect(0, LCD_HEIGHT - FOOTER_H, LCD_WIDTH, 1, FG);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(HEADER_BG, FG);
    lcd_puts(5, LCD_HEIGHT - FOOTER_H + 4, l1);
    lcd_puts(5, LCD_HEIGHT - FOOTER_H + 13, l2);
}

//-----------------------------------------------------------------------------
// Clears from the header down, not from BODY_Y: the two rows between them
// belong to no draw path and would keep whatever an overlay left there
static void clear_body(void)
{
    lcd_fill_rect(0, HEADER_H, LCD_WIDTH, LCD_HEIGHT - HEADER_H - FOOTER_H, BG);
    lcd_set_font(FONT_SMALL);
    lcd_set_color(BG, FG);
}

//-----------------------------------------------------------------------------
static void draw_hex_view(void)
{
    uint32_t addr = g_current_address;
    int y = BODY_Y;

    clear_body();

    for (int line = 0; line < LINES_PER_PAGE; line++) {
        char text[64];
        char ascii[HEX_BYTES_PER_LINE + 1];
        int n = snprintf(text, sizeof(text), "%08lX: ", (unsigned long)addr);

        for (int i = 0; i < HEX_BYTES_PER_LINE; i++) {
            uint8_t byte = read_flash_byte(addr + i);

            n += snprintf(text + n, sizeof(text) - n, "%02X ", byte);
            ascii[i] = is_printable(byte) ? (char)byte : '.';
        }

        ascii[HEX_BYTES_PER_LINE] = 0;
        snprintf(text + n, sizeof(text) - n, " %s", ascii);

        lcd_puts(5, y, text);

        addr += HEX_BYTES_PER_LINE;
        y += LINE_H;
    }

    draw_footer("UP/DN: line   LEFT/RIGHT: page   TRIG_UP/DN: 4 KB",
        "MODE: next view   MENU: settings and help");
}

//-----------------------------------------------------------------------------
static void draw_ascii_view(void)
{
    uint32_t addr = g_current_address;
    int y = BODY_Y;

    clear_body();

    for (int line = 0; line < LINES_PER_PAGE; line++) {
        char text[ASCII_CHARS_PER_LINE + 16];
        int n = snprintf(text, sizeof(text), "%04lX ",
            (unsigned long)((addr - FLASH_START_ADDR) & 0xFFFF));

        for (int i = 0; i < ASCII_CHARS_PER_LINE; i++) {
            uint8_t byte = read_flash_byte(addr + i);

            text[n + i] = is_printable(byte) ? (char)byte : '.';
        }

        text[n + ASCII_CHARS_PER_LINE] = 0;
        lcd_puts(5, y, text);

        addr += ASCII_CHARS_PER_LINE;
        y += LINE_H;
    }

    draw_footer("UP/DN: line   LEFT/RIGHT: page   TRIG_UP/DN: 4 KB",
        "MODE: next view   MENU: settings and help");
}

//-----------------------------------------------------------------------------
static void draw_structure_view(void)
{
    int rows = LINES_PER_PAGE / 2;
    int y = BODY_Y;

    clear_body();

    if (g_section_index < g_section_scroll)
        g_section_scroll = g_section_index;
    else if (g_section_index >= g_section_scroll + rows)
        g_section_scroll = g_section_index - rows + 1;

    for (int i = g_section_scroll; i < g_section_count && i < g_section_scroll + rows; i++) {
        const flash_section_t *s = &g_flash_sections[i];
        bool selected = (i == g_section_index);
        uint16_t bg = selected ? SEL_BG : BG;
        char text[64];

        lcd_fill_rect(0, y, LCD_WIDTH, 2 * LINE_H, bg);
        lcd_set_color(bg, FG);

        snprintf(text, sizeof(text), "%-14s %luK", s->name,
            (unsigned long)((s->size + 1023) / 1024));
        lcd_puts(5, y, text);

        snprintf(text, sizeof(text), "  %08lX..%08lX  %s",
            (unsigned long)s->address, (unsigned long)(s->address + s->size - 1),
            s->description);
        lcd_puts(5, y + LINE_H, text);

        y += 2 * LINE_H;
    }

    lcd_set_color(BG, FG);

    draw_footer("UP/DN: select   RIGHT: open in hex view",
        "MODE: next view   MENU: settings and help");
}

//-----------------------------------------------------------------------------
// Not a disassembler: it splits the stream into 16 and 32 bit Thumb
// instructions and names the encoding group. That is enough to tell code from
// data, which is what this view is for.
static const char *thumb_class(uint16_t hw, bool wide, uint16_t hw2)
{
    if (wide) {
        if ((hw2 & 0xD000) == 0xD000)
            return "bl";

        if ((hw & 0xFE40) == 0xE800)
            return "ldm/stm.w";

        if ((hw & 0xFE00) == 0xF800)
            return "ldr/str.w";

        return "32-bit";
    }

    if (hw < 0x2000) return "lsl/lsr/asr/add/sub";
    if (hw < 0x4000) return "mov/cmp/add/sub imm";
    if (hw < 0x4400) return "data processing";
    if (hw < 0x4800) return "bx/blx/hi-reg";
    if (hw < 0x5000) return "ldr literal";
    if (hw < 0xA000) return "load/store";
    if (hw < 0xB000) return "adr / add sp";
    if (hw < 0xC000) return "push/pop/misc";
    if (hw < 0xD000) return "ldm/stm";
    if (hw < 0xDF00) return "b<cond>";
    if (hw < 0xE000) return "svc";

    return "b";
}

//-----------------------------------------------------------------------------
static void draw_thumb_view(void)
{
    uint32_t addr = g_current_address;
    int y = BODY_Y;

    clear_body();

    for (int line = 0; line < LINES_PER_PAGE && addr < g_flash_end; line++) {
        uint16_t hw = read_flash_half(addr);
        bool wide = (hw & 0xF800) == 0xE800 || (hw & 0xF000) == 0xF000;
        uint16_t hw2 = wide ? read_flash_half(addr + 2) : 0;
        char text[64];

        if (wide) {
            snprintf(text, sizeof(text), "%08lX: %04X %04X  %s",
                (unsigned long)addr, hw, hw2, thumb_class(hw, true, hw2));
        }
        else {
            snprintf(text, sizeof(text), "%08lX: %04X       %s",
                (unsigned long)addr, hw, thumb_class(hw, false, 0));
        }

        lcd_puts(5, y, text);

        addr += wide ? 4 : 2;
        y += LINE_H;
    }

    draw_footer("UP/DN: halfword  LEFT/RIGHT: page  TRIG_UP/DN: 4K",
        "MODE: next view   MENU: settings and help");
}

//-----------------------------------------------------------------------------
static void draw_view(void)
{
    draw_header();

    switch (g_view_mode) {
        case FLASH_VIEW_ASCII:     draw_ascii_view(); break;
        case FLASH_VIEW_STRUCTURE: draw_structure_view(); break;
        case FLASH_VIEW_THUMB:     draw_thumb_view(); break;
        default:                   draw_hex_view(); break;
    }
}

//-----------------------------------------------------------------------------
void flash_viewer_init(void)
{
    uint16_t size_kb = *(uint16_t *)FLASH_SIZE_REG;

    if (0 == size_kb || 0xFFFF == size_kb)
        size_kb = FLASH_SIZE / 1024;

    g_flash_end = FLASH_START_ADDR + (uint32_t)size_kb * 1024;

    g_current_address = FLASH_START_ADDR;
    g_view_mode = FLASH_VIEW_HEX;
    g_section_index = 0;
    g_section_scroll = 0;

    analyze_flash_sections();

    g_dirty = true;
    draw_view();
}

//-----------------------------------------------------------------------------
// Paints straight away rather than raising the dirty flag: an overlay can
// close while another one is still open, and only the top screen is ticked,
// so a deferred repaint would not happen until the whole stack unwound
void flash_viewer_redraw(void)
{
    g_dirty = false;
    draw_view();
}

//-----------------------------------------------------------------------------
void flash_viewer_task(void)
{
    if (!g_dirty)
        return;

    g_dirty = false;
    draw_view();
}

//-----------------------------------------------------------------------------
void flash_viewer_cleanup(void)
{
    // Nothing to release: no timers, no hardware
}

//-----------------------------------------------------------------------------
void flash_viewer_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    int line = bytes_per_line();
    int page = line * LINES_PER_PAGE;

    if ((buttons & BTN_MODE) && !repeat) {
        g_view_mode = (g_view_mode + 1) % FLASH_VIEW_COUNT;
        set_address(g_current_address); // re-align to the new line size
        g_dirty = true;
        return;
    }

    if (FLASH_VIEW_STRUCTURE == g_view_mode) {
        if ((buttons & BTN_UP) && g_section_index > 0) {
            g_section_index--;
            g_dirty = true;
        }

        if ((buttons & BTN_DOWN) && g_section_index < g_section_count - 1) {
            g_section_index++;
            g_dirty = true;
        }

        if ((buttons & BTN_RIGHT) && !repeat && g_section_count > 0) {
            g_view_mode = FLASH_VIEW_HEX;
            set_address(g_flash_sections[g_section_index].address);
        }

        return;
    }

    int64_t addr = g_current_address;

    if (buttons & BTN_UP)
        set_address(addr - line);

    if (buttons & BTN_DOWN)
        set_address(addr + line);

    if (buttons & BTN_LEFT)
        set_address(addr - page);

    if (buttons & BTN_RIGHT)
        set_address(addr + page);

    if (buttons & BTN_TRIG_UP)
        set_address(addr - 4096);

    if (buttons & BTN_TRIG_DOWN)
        set_address(addr + 4096);
}

//-----------------------------------------------------------------------------
const flash_section_t *flash_get_sections(int *count)
{
    if (count)
        *count = g_section_count;

    return g_flash_sections;
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void view_mode_changed(void)
{
    set_address(g_current_address);
    g_dirty = true;
}

//-----------------------------------------------------------------------------
static void action_goto_start(const void *arg)
{
    (void)arg;
    set_address(FLASH_START_ADDR);
    menu_close_popups();
}

//-----------------------------------------------------------------------------
static void action_goto_image_end(const void *arg)
{
    (void)arg;
    set_address((uint32_t)&__etext);
    menu_close_popups();
}

static const char *const g_help_lines[] =
{
    "UP/DOWN     - One line",
    "LEFT/RIGHT  - One page",
    "TRIG_UP/DN  - 4 KB",
    "MODE        - Next view",
    "MENU        - This menu",
    "SHIFT+MENU  - Back to the launcher",
    "",
    "Layout shows the regions of the image;",
    "RIGHT opens the selected one in hex.",
    "Thumb splits the stream into 16 and 32 bit",
    "instructions and names the encoding group -",
    "enough to tell code from data.",
};

static const info_page_t g_help_page =
{
    .title = "Flash Viewer",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const char *const g_view_labels[] = { "Hex", "ASCII", "Layout", "Thumb" };

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "View",
      .u.choice = { &g_view_mode, g_view_labels, FLASH_VIEW_COUNT,
                    view_mode_changed } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Go to start",
      .u.action = { action_goto_start, NULL } },
    { .kind = MI_ACTION, .label = "Go to image end",
      .u.action = { action_goto_image_end, NULL } },
    { .kind = MI_ACTION, .label = "Help",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t flash_viewer_menu =
{
    .title = "Flash Viewer",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};
