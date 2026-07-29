/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
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
#include "config.h"
#include "flash.h"
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

/*
 * The address spaces the viewer walks. Sizes are the real ones for this part:
 * 128 KB of main SRAM at 0x20000000, 64 KB of tightly coupled memory at
 * 0x10000000 (where the stack and heap live, see the linker script), and the
 * 32 KB system region - bootloader ROM, then the OTP block, and at the top of
 * it the unique id and flash size registers this file already reads.
 *
 * Flash's length is fixed up at init from the size register: it is the one
 * that varies by part, and reading past the end of the array bus-faults. The
 * SPI part's length comes from its JEDEC capacity byte, and is zero when there
 * is no chip - the source stays in the list and says so rather than vanishing,
 * because a missing entry reads as a bug in the viewer.
 */
static struct {
    const char *name;
    uint32_t base;
    uint32_t size;
    bool serial;            // reached by a bus command, not by a pointer
} g_regions[MEM_REGION_COUNT] = {
    [MEM_REGION_FLASH]  = { "Flash",  FLASH_START_ADDR, FLASH_SIZE, false },
    [MEM_REGION_SRAM]   = { "SRAM",   0x20000000u, 128 * 1024, false },
    [MEM_REGION_TCM]    = { "TCM",    0x10000000u,  64 * 1024, false },
    [MEM_REGION_SYSTEM] = { "System", 0x1FFF0000u,  32 * 1024, false },
    [MEM_REGION_SPI]    = { "SPI",    0x00000000u,  0,         true  },
};

static int g_region = MEM_REGION_FLASH;

/*
 * One page of the serial part, held back so that a screenful of hex is a
 * couple of bus transactions instead of one per byte. Every view reads through
 * read_flash_byte, and a hex page alone would otherwise be 144 commands.
 *
 * Invalidated whenever the source changes, and by anything that writes.
 */
#define SPI_CACHE_SIZE      FLASH_PAGE_SIZE
#define SPI_CACHE_EMPTY     0xFFFFFFFFu

static uint8_t g_spi_cache[SPI_CACHE_SIZE];
static uint32_t g_spi_cache_base = SPI_CACHE_EMPTY;

/*
 * The used/blank map behind the Map view. One bit per cell, filled in a little
 * at a time from flash_viewer_task: reading all 8 MB of the serial part takes
 * about four seconds at 15.6 MHz, and doing it in one go would freeze the main
 * loop - buttons, backlight and all - for exactly that long.
 *
 * A cell is "used" if it holds any byte that is neither 0x00 nor 0xFF. That is
 * the honest test across all five sources: 0xFF is erased NOR, 0x00 is
 * untouched RAM, and anything else is somebody's data.
 */
#define MAP_CELLS_MAX       2048
#define MAP_SCAN_CHUNK      256         // bytes per read, and the stack cost
#define MAP_SCAN_BUDGET_US  15000       // per pass of the main loop

static struct {
    bool running;
    bool done;
    uint32_t cell_bytes;                // how much of the source one cell is
    uint32_t cells;
    uint32_t next_cell;
    uint32_t used;
    uint32_t first_used;
    uint8_t bits[MAP_CELLS_MAX / 8];
} g_map;

static int g_map_cursor = 0;

// Config store browser: which entry is selected, and whether the decoded field
// list for it is open
static int g_cfg_index = 0;
static int g_cfg_scroll = 0;
static bool g_cfg_detail = false;

/*- External symbols from linker script ------------------------------------*/
extern uint32_t _etext;   // end of .text in flash
extern uint32_t __etext;  // end of everything in flash, .data image included

/*- Local Prototypes --------------------------------------------------------*/
static void analyze_flash_sections(void);
static void map_scan_restart(void);
static bool map_scan_step(void);
static bool map_cell_used(uint32_t cell);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint32_t region_base(void)
{
    return g_regions[g_region].base;
}

//-----------------------------------------------------------------------------
static uint32_t region_end(void)
{
    return g_regions[g_region].base + g_regions[g_region].size;
}

//-----------------------------------------------------------------------------
// Bounds-checked against the CURRENT region, which is what keeps the viewer
// from wandering into an unmapped hole and taking the scope down with a bus
// fault. Out of range reads as erased rather than faulting.
//
// A serial source cannot be dereferenced at all, so it comes through a cached
// page read instead. Everything above this line is written as if memory were
// memory.
static uint8_t read_flash_byte(uint32_t addr)
{
    if (addr < region_base() || addr >= region_end())
        return 0xFF;

    if (g_regions[g_region].serial) {
        uint32_t page = addr & ~(SPI_CACHE_SIZE - 1);

        if (page != g_spi_cache_base) {
            if (!flash_read(page, g_spi_cache, SPI_CACHE_SIZE))
                return 0xFF;

            g_spi_cache_base = page;
        }

        return g_spi_cache[addr - page];
    }

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
    int64_t base = region_base();
    int64_t last = (int64_t)region_end() - step * LINES_PER_PAGE;
    uint32_t offset;

    if (last < base)
        last = base;

    if (addr < base)
        addr = base;

    if (addr > last)
        addr = last;

    offset = (uint32_t)addr - (uint32_t)base;

    g_current_address = (uint32_t)base + (offset / (uint32_t)step) * (uint32_t)step;

    g_dirty = true;
}

//-----------------------------------------------------------------------------
// Switching regions moves the cursor with it, since an address from the old
// one is meaningless in the new
static void set_region(int region)
{
    if (region < 0 || region >= MEM_REGION_COUNT)
        return;

    g_region = region;
    g_spi_cache_base = SPI_CACHE_EMPTY;

    map_scan_restart();
    set_address(region_base());
}

//-----------------------------------------------------------------------------
// A cell is the smallest erasable unit of the source where that means
// something, and whatever divides the source into at most MAP_CELLS_MAX cells
// where it does not. Both come out a power of two, so the arithmetic below is
// shifts and masks.
static void map_scan_restart(void)
{
    uint32_t size = g_regions[g_region].size;
    uint32_t cell = FLASH_SECTOR_SIZE;

    memset(&g_map, 0, sizeof(g_map));

    g_map.first_used = SPI_CACHE_EMPTY;

    if (0 == size)
        return;

    while (size / cell > MAP_CELLS_MAX)
        cell *= 2;

    // A small source would otherwise be one cell wide and tell nobody
    // anything: 128 KB of SRAM is 32 sectors, and the screen has room for
    // hundreds of cells
    while (cell > FLASH_PAGE_SIZE && size / cell < MAP_CELLS_MAX / 4)
        cell /= 2;

    g_map.cell_bytes = cell;
    g_map.cells = (size + cell - 1) / cell;
    g_map.running = true;
}

//-----------------------------------------------------------------------------
static bool map_cell_used(uint32_t cell)
{
    return 0 != (g_map.bits[cell >> 3] & (1 << (cell & 7)));
}

//-----------------------------------------------------------------------------
// Runs for a fixed slice of wall clock and returns, so a source that takes
// seconds to read costs a few frames of drawing rather than the whole main
// loop. timer_us and not timer_ms: the millisecond counter only moves in
// timer_task, which does not run while this does.
static bool map_scan_step(void)
{
    uint32_t start = timer_us();

    if (!g_map.running)
        return false;

    while (g_map.next_cell < g_map.cells) {
        uint32_t base = region_base() + g_map.next_cell * g_map.cell_bytes;
        uint32_t end = base + g_map.cell_bytes;
        bool used = false;

        if (end > region_end())
            end = region_end();

        for (uint32_t addr = base; addr < end && !used; addr++) {
            uint8_t byte = read_flash_byte(addr);

            if (0xFF != byte && 0x00 != byte)
                used = true;
        }

        if (used) {
            g_map.bits[g_map.next_cell >> 3] |= 1 << (g_map.next_cell & 7);
            g_map.used++;

            if (SPI_CACHE_EMPTY == g_map.first_used)
                g_map.first_used = base;
        }

        g_map.next_cell++;

        if (timer_us() - start > MAP_SCAN_BUDGET_US)
            return true;                // more to do, but not this pass
    }

    g_map.running = false;
    g_map.done = true;

    return true;
}

//-----------------------------------------------------------------------------
// What is being read, in the few characters the header has room for. The size
// is part of the name because "SPI" alone does not say whether a chip
// answered, and that is the first thing anybody wants to know here.
static const char *source_summary(void)
{
    static char buf[32];
    uint32_t size = g_regions[g_region].size;

    if (0 == size)
        return (MEM_REGION_SPI == g_region) ? "SPI - no device" : "empty";

    if (size >= 1024 * 1024)
        snprintf(buf, sizeof(buf), "%s  %lu MB", g_regions[g_region].name,
            (unsigned long)(size >> 20));
    else
        snprintf(buf, sizeof(buf), "%s  %lu KB", g_regions[g_region].name,
            (unsigned long)(size >> 10));

    return buf;
}

//-----------------------------------------------------------------------------
static const char *view_mode_name(void)
{
    switch (g_view_mode) {
        case FLASH_VIEW_MAP:       return "Map";
        case FLASH_VIEW_HEX:       return "Hex";
        case FLASH_VIEW_ASCII:     return "ASCII";
        case FLASH_VIEW_STRUCTURE: return "Layout";
        case FLASH_VIEW_THUMB:     return "Thumb";
        case FLASH_VIEW_CONFIG:    return "Config";
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

    // The source is the title now, and the view is a subtitle of it: which
    // memory you are looking at is the thing that is easy to lose track of
    // once there are five of them
    lcd_set_color(HEADER_BG, ACCENT);
    snprintf(buf, sizeof(buf), "%s", source_summary());
    lcd_puts(5, 5, buf);

    lcd_set_color(HEADER_BG, FG);
    snprintf(buf, sizeof(buf), "%s", view_mode_name());
    lcd_puts(LCD_WIDTH - 5 - 6 * (int)strlen(buf), 5, buf);

    if (FLASH_VIEW_STRUCTURE == g_view_mode) {
        snprintf(buf, sizeof(buf), "%d regions   %lu KB total", g_section_count,
            (unsigned long)((g_flash_end - FLASH_START_ADDR) / 1024));
    }
    else if (FLASH_VIEW_CONFIG == g_view_mode) {
        snprintf(buf, sizeof(buf), "Settings store   %s 0x%08lX",
            config_store_is_external() ? "SPI" : "flash",
            (unsigned long)config_store_base());
    }
    else if (FLASH_VIEW_MAP == g_view_mode) {
        snprintf(buf, sizeof(buf), "whole source, %lu cells",
            (unsigned long)g_map.cells);
    }
    else {
        snprintf(buf, sizeof(buf), "0x%08lX   %s",
            (unsigned long)g_current_address,
            (MEM_REGION_FLASH == g_region) ?
                section_name_at(g_current_address) : "-");
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
// Biggest cell that still fits the whole source on one screen, and how many of
// them go across. Nothing scrolls in the map on purpose: a map you have to
// page through is a dump. Shared with the button handler so that UP and DOWN
// move by exactly one visible row.
static int map_grid(int *cell_px_out)
{
    int grid_h = LCD_HEIGHT - BODY_Y - FOOTER_H - LINE_H - 2;
    int cell_px = 10;
    int cols = 1;

    while (cell_px > 2) {
        cols = (LCD_WIDTH - 10) / cell_px;

        if ((((int)g_map.cells + cols - 1) / cols) * cell_px <= grid_h)
            break;

        cell_px--;
    }

    cols = (LCD_WIDTH - 10) / cell_px;

    if (cell_px_out)
        *cell_px_out = cell_px;

    return cols;
}

//-----------------------------------------------------------------------------
// The whole source at once, one cell per erase unit. This is the view that
// answers the questions an 8 MB part actually raises - is there anything on
// it, where does it start, how much is left - none of which a hex dump can
// answer in under an hour of paging.
//
// Cells are laid out left to right, top to bottom, so the picture reads like
// the address space: the top left is offset zero.
static void draw_map_view(void)
{
    int grid_y = BODY_Y + LINE_H + 2;
    int cell_px;
    int cols = map_grid(&cell_px);
    char text[64];

    clear_body();

    if (0 == g_map.cells) {
        lcd_puts(5, BODY_Y + 20, "Nothing to map here.");

        if (MEM_REGION_SPI == g_region)
            lcd_puts(5, BODY_Y + 20 + LINE_H * 2,
                "No SPI device answered at boot.");

        draw_footer("F1: next memory   MODE: next view",
            "MENU: settings and help");
        return;
    }

    // Progress, then what the picture is made of
    if (g_map.running) {
        snprintf(text, sizeof(text), "scanning %lu%%   %luK per cell",
            (unsigned long)(100ull * g_map.next_cell / g_map.cells),
            (unsigned long)(g_map.cell_bytes / 1024));
    }
    else if (0 == g_map.used) {
        snprintf(text, sizeof(text), "all blank   %luK per cell",
            (unsigned long)(g_map.cell_bytes / 1024));
    }
    else {
        snprintf(text, sizeof(text), "%lu of %lu used   from %08lX",
            (unsigned long)g_map.used, (unsigned long)g_map.cells,
            (unsigned long)g_map.first_used);
    }

    lcd_set_color(BG, FG);
    lcd_puts(5, BODY_Y, text);

    for (uint32_t i = 0; i < g_map.cells; i++) {
        int x = 5 + (int)(i % (uint32_t)cols) * cell_px;
        int y = grid_y + (int)(i / (uint32_t)cols) * cell_px;
        int color;

        if (g_map.running && i >= g_map.next_cell)
            color = LCD_COLOR(40, 40, 40);          // not looked at yet
        else if (map_cell_used(i))
            color = ACCENT;
        else
            color = LCD_COLOR(0, 70, 110);          // blank, but accounted for

        lcd_fill_rect(x, y, cell_px - 1, cell_px - 1, color);
    }

    if (g_map_cursor >= (int)g_map.cells)
        g_map_cursor = (int)g_map.cells - 1;

    {
        int x = 5 + (g_map_cursor % cols) * cell_px;
        int y = grid_y + (g_map_cursor / cols) * cell_px;

        lcd_draw_rect(x - 1, y - 1, cell_px + 1, cell_px + 1, LCD_WHITE_COLOR);
    }

    snprintf(text, sizeof(text), "%08lX  %s",
        (unsigned long)(region_base() + (uint32_t)g_map_cursor * g_map.cell_bytes),
        map_cell_used((uint32_t)g_map_cursor) ? "used" : "blank");

    draw_footer(text, "arrows: move   TRIG: open in hex   F1: next memory");
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
// The settings store, one row per rotation slot. Reading it as hex is possible
// and useless: what matters about an entry is whether it validates and why
// not, and that is four checks over 416 bytes rather than something the eye
// picks out of a dump.
static void draw_config_list(void)
{
    char buf[64];
    int count = config_store_entries();
    int live = config_store_live_index();
    int rows = LINES_PER_PAGE - 1;
    int y = BODY_Y;
    int valid = 0;

    clear_body();

    for (int i = 0; i < count; i++) {
        if (CONFIG_ENTRY_VALID == config_store_entry_state(i))
            valid++;
    }

    lcd_set_color(BG, ACCENT);
    snprintf(buf, sizeof(buf), "%d/%d valid   live %d   %d B/entry",
        valid, count, live, (int)sizeof(Config));
    lcd_puts(5, y, buf);
    y += LINE_H;

    // Keep the selection on screen without a separate scroll key
    if (g_cfg_index < g_cfg_scroll)
        g_cfg_scroll = g_cfg_index;
    else if (g_cfg_index >= g_cfg_scroll + rows)
        g_cfg_scroll = g_cfg_index - rows + 1;

    for (int row = 0; row < rows && g_cfg_scroll + row < count; row++) {
        int i = g_cfg_scroll + row;
        ConfigEntryState state = config_store_entry_state(i);
        const Config *entry = config_store_entry(i);
        bool sel = (i == g_cfg_index);

        if (sel)
            lcd_fill_rect(0, y - 1, LCD_WIDTH, LINE_H, SEL_BG);

        lcd_set_color(sel ? SEL_BG : BG,
            (CONFIG_ENTRY_VALID == state) ? FG : ACCENT);

        // count is only meaningful in an entry that validates; in one that
        // does not it is whatever survived, so it is left off
        if (CONFIG_ENTRY_VALID == state)
            snprintf(buf, sizeof(buf), "%c%3d  %-11s cnt %-7d %s",
                (i == live) ? '>' : ' ', i,
                config_entry_state_name(state), entry->count,
                config_store_entry_calib_ok(i) ? "cal ok" : "cal --");
        else
            snprintf(buf, sizeof(buf), "%c%3d  %-11s         %s",
                (i == live) ? '>' : ' ', i,
                config_entry_state_name(state),
                config_store_entry_calib_ok(i) ? "cal ok" : "");

        lcd_puts(5, y, buf);
        y += LINE_H;
    }

    lcd_set_color(BG, FG);
    draw_footer("U/D entry   RIGHT decode   MODE next view",
        "'>' is the live slot; 'cal ok' = calibration seals");
}

//-----------------------------------------------------------------------------
// The selected entry decoded field by field, which is the view that answers
// "is what I set actually what got stored". An entry that failed its CRC is
// still shown - the fields are readable, they simply cannot be trusted, and
// seeing which one looks wrong is usually the whole diagnosis.
static void draw_config_detail(void)
{
    char buf[64];
    const Config *entry = config_store_entry(g_cfg_index);
    ConfigEntryState state = config_store_entry_state(g_cfg_index);
    int rows = LINES_PER_PAGE - 1;
    int y = BODY_Y;

    clear_body();

    lcd_set_color(BG, ACCENT);
    snprintf(buf, sizeof(buf), "entry %d  %s%s", g_cfg_index,
        config_entry_state_name(state),
        (CONFIG_ENTRY_VALID == state) ? "" : "  - values are suspect");
    lcd_puts(5, y, buf);
    y += LINE_H;

    lcd_set_color(BG, FG);

    for (int row = 0; row < rows; row++) {
        if (!config_describe(entry, g_cfg_scroll + row, buf, sizeof(buf)))
            break;

        lcd_puts(5, y, buf);
        y += LINE_H;
    }

    draw_footer("U/D line   L/R page   LEFT at top: back",
        "MODE next view");
}

//-----------------------------------------------------------------------------
static void draw_config_view(void)
{
    if (g_cfg_detail)
        draw_config_detail();
    else
        draw_config_list();
}

//-----------------------------------------------------------------------------
// Scrolling the detail list needs its length, and config_describe is the only
// thing that knows it
static int config_detail_lines(void)
{
    char buf[64];
    int n = 0;

    while (config_describe(config_store_entry(g_cfg_index), n, buf, sizeof(buf)))
        n++;

    return n;
}

//-----------------------------------------------------------------------------
static void config_buttons(int buttons)
{
    int rows = LINES_PER_PAGE - 1;

    if (g_cfg_detail) {
        int last = config_detail_lines() - rows;

        if (last < 0)
            last = 0;

        if ((buttons & BTN_LEFT) && 0 == g_cfg_scroll) {
            // Only from the top, so that paging back through a long entry
            // does not fall out of it by surprise
            g_cfg_detail = false;
            g_cfg_scroll = 0;
            g_dirty = true;
            return;
        }

        if (buttons & BTN_UP)
            g_cfg_scroll--;

        if (buttons & BTN_DOWN)
            g_cfg_scroll++;

        if (buttons & BTN_LEFT)
            g_cfg_scroll -= rows;

        if (buttons & BTN_RIGHT)
            g_cfg_scroll += rows;

        if (g_cfg_scroll > last)
            g_cfg_scroll = last;

        if (g_cfg_scroll < 0)
            g_cfg_scroll = 0;

        g_dirty = true;
        return;
    }

    if (buttons & BTN_UP) {
        if (g_cfg_index > 0)
            g_cfg_index--;

        g_dirty = true;
    }

    if (buttons & BTN_DOWN) {
        if (g_cfg_index < config_store_entries() - 1)
            g_cfg_index++;

        g_dirty = true;
    }

    if (buttons & BTN_TRIG_UP) {
        g_cfg_index = 0;
        g_dirty = true;
    }

    if (buttons & BTN_TRIG_DOWN) {
        g_cfg_index = config_store_live_index();
        g_dirty = true;
    }

    if (buttons & BTN_RIGHT) {
        g_cfg_detail = true;
        g_cfg_scroll = 0;
        g_dirty = true;
    }
}

//-----------------------------------------------------------------------------
static void draw_view(void)
{
    draw_header();

    switch (g_view_mode) {
        case FLASH_VIEW_MAP:       draw_map_view(); break;
        case FLASH_VIEW_ASCII:     draw_ascii_view(); break;
        case FLASH_VIEW_STRUCTURE: draw_structure_view(); break;
        case FLASH_VIEW_THUMB:     draw_thumb_view(); break;
        case FLASH_VIEW_CONFIG:    draw_config_view(); break;
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

    // The bound read_flash_byte enforces, not just the one the layout scan
    // uses: leaving it at the 512 KB fallback would let the cursor walk off
    // the end of a smaller part's array, which bus-faults
    g_regions[MEM_REGION_FLASH].size = g_flash_end - FLASH_START_ADDR;

    // Whatever the part answered at boot. Zero when nothing did, and the
    // source then says so instead of offering 8 MB of 0xFF.
    g_regions[MEM_REGION_SPI].size = flash_size();

    // Back to flash whatever the last session left selected - the layout scan
    // below reads through read_flash_byte, and that answers for the region
    // that is current
    g_region = MEM_REGION_FLASH;

    g_current_address = FLASH_START_ADDR;
    g_view_mode = FLASH_VIEW_MAP;
    g_section_index = 0;
    g_section_scroll = 0;
    g_cfg_index = 0;
    g_cfg_scroll = 0;
    g_cfg_detail = false;
    g_map_cursor = 0;
    g_spi_cache_base = SPI_CACHE_EMPTY;

    analyze_flash_sections();
    map_scan_restart();

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
    // The map fills itself in from here, a slice per pass. Repainting on every
    // slice would cost more than the scan does - the grid is a couple of
    // thousand filled rectangles over a bit-banged bus - so the picture
    // catches up four times a second and once more when it finishes.
    if (FLASH_VIEW_MAP == g_view_mode && g_map.running) {
        static uint32_t last_paint;

        map_scan_step();

        if (!g_map.running || timer_ms() - last_paint > 250) {
            last_paint = timer_ms();
            g_dirty = true;
        }
    }

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

        // Leaving the store view half way into an entry and cycling back to
        // it should land on the list, not on whatever was being read before
        g_cfg_detail = false;
        g_cfg_scroll = 0;

        g_dirty = true;
        return;
    }

    // One key for the thing this viewer is now mostly about: which memory.
    // It used to be two levels down a menu, which is a long way to go to
    // compare a byte in flash with the copy of it in RAM.
    if ((buttons & BTN_F1) && !repeat) {
        set_region((g_region + 1) % MEM_REGION_COUNT);
        g_dirty = true;
        return;
    }

    if (FLASH_VIEW_MAP == g_view_mode) {
        int cols = map_grid(NULL);      // one visible row per UP/DOWN

        if (g_map.cells > 0) {
            int last = (int)g_map.cells - 1;

            if (buttons & BTN_LEFT)
                g_map_cursor--;

            if (buttons & BTN_RIGHT)
                g_map_cursor++;

            if (buttons & BTN_UP)
                g_map_cursor -= cols;

            if (buttons & BTN_DOWN)
                g_map_cursor += cols;

            if (g_map_cursor < 0)
                g_map_cursor = 0;

            if (g_map_cursor > last)
                g_map_cursor = last;

            // The point of the map: pick a cell, read what is in it
            if ((buttons & BTN_TRIG) && !repeat) {
                g_view_mode = FLASH_VIEW_HEX;
                set_address(region_base() +
                    (uint32_t)g_map_cursor * g_map.cell_bytes);
            }

            g_dirty = true;
        }

        return;
    }

    if (FLASH_VIEW_CONFIG == g_view_mode) {
        config_buttons(buttons);
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

    g_cfg_detail = false;
    g_cfg_scroll = 0;

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

//-----------------------------------------------------------------------------
// The map is a snapshot, and the sources it is taken over change underneath
// it - SRAM constantly, the serial part whenever something writes to it
static void action_rescan(const void *arg)
{
    (void)arg;

    g_spi_cache_base = SPI_CACHE_EMPTY;
    map_scan_restart();
    g_view_mode = FLASH_VIEW_MAP;
    g_dirty = true;
    menu_close_popups();
}

//-----------------------------------------------------------------------------
static void region_changed(void)
{
    // The choice item has already written g_region; this puts the cursor
    // somewhere that exists in it
    set_region(g_region);
    g_dirty = true;
}

//-----------------------------------------------------------------------------
// Straight to a slot of the settings store as raw bytes, for when the decoded
// view is not what is wanted - a slot that fails its CRC is sometimes only
// explicable by looking at what is actually in it.
//
// It asks config.c where the store is rather than taking the address of an
// entry, because the answer stopped being an address: on the SPI part an entry
// is fetched into a buffer, and the pointer to that buffer says nothing about
// where the store lives. These two used to jump to the RAM cache.
static void goto_store_entry(int index)
{
    set_region(config_store_is_external() ? MEM_REGION_SPI : MEM_REGION_FLASH);

    g_view_mode = FLASH_VIEW_HEX;
    set_address(config_store_base() + (uint32_t)index * config_store_entry_size());

    menu_close_popups();
}

//-----------------------------------------------------------------------------
static void action_goto_store(const void *arg)
{
    (void)arg;
    goto_store_entry(0);
}

//-----------------------------------------------------------------------------
// ...and to the live one, which is the entry the running scope came from
static void action_goto_live_entry(const void *arg)
{
    (void)arg;
    goto_store_entry(config_store_live_index());
}

//-----------------------------------------------------------------------------
static void action_open_store(const void *arg)
{
    (void)arg;
    g_view_mode = FLASH_VIEW_CONFIG;
    g_cfg_detail = false;
    g_cfg_index = config_store_live_index();
    g_cfg_scroll = 0;
    g_dirty = true;
    menu_close_popups();
}

static const char *const g_help_lines[] =
{
    "F1          - Next memory",
    "MODE        - Next view",
    "UP/DOWN     - One line",
    "LEFT/RIGHT  - One page",
    "TRIG_UP/DN  - 4 KB",
    "MENU        - This menu",
    "SHIFT+MENU  - Back to the launcher",
    "",
    "Map is the whole memory on one screen, one",
    "cell per erase sector: amber holds data, blue",
    "is blank. Arrows move the cursor and TRIG",
    "opens that cell in hex. The scan runs in the",
    "background because reading all 8 MB of the",
    "SPI part takes about four seconds.",
    "",
    "Layout shows the regions of the image;",
    "RIGHT opens the selected one in hex.",
    "Thumb splits the stream into 16 and 32 bit",
    "instructions and names the encoding group -",
    "enough to tell code from data.",
    "",
    "Memory picks what is being read: flash, SRAM,",
    "TCM (stack and heap), the system ROM, or the",
    "8 MB SPI part. The first four are whitelisted",
    "because an unmapped address bus-faults and a",
    "peripheral register can change state just",
    "from being read. The SPI part is not in the",
    "address space at all - its bytes arrive over",
    "the bus, a page at a time.",
    "",
    "Config decodes the settings store instead of",
    "dumping it: one row per rotation slot, why an",
    "entry is not usable, and RIGHT to read its",
    "fields. '>' marks the slot in use, and the",
    "calibration column is its own seal - it can",
    "hold in an entry that otherwise failed.",
};

static const info_page_t g_help_page =
{
    .title = "Flash Viewer",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const char *const g_view_labels[] =
    { "Map", "Hex", "ASCII", "Layout", "Thumb", "Config" };

static const char *const g_region_labels[] =
    { "Flash", "SRAM", "TCM", "System", "SPI flash" };

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "Memory",
      .u.choice = { &g_region, g_region_labels, MEM_REGION_COUNT,
                    region_changed } },
    { .kind = MI_CHOICE, .label = "View",
      .u.choice = { &g_view_mode, g_view_labels, FLASH_VIEW_COUNT,
                    view_mode_changed } },
    { .kind = MI_ACTION, .label = "Rescan map",
      .u.action = { action_rescan, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Go to start",
      .u.action = { action_goto_start, NULL } },
    { .kind = MI_ACTION, .label = "Go to image end",
      .u.action = { action_goto_image_end, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Settings store...",
      .u.action = { action_open_store, NULL } },
    { .kind = MI_ACTION, .label = "Store as hex",
      .u.action = { action_goto_store, NULL } },
    { .kind = MI_ACTION, .label = "Live entry as hex",
      .u.action = { action_goto_live_entry, NULL } },
};

const menu_def_t flash_viewer_menu =
{
    .title = "Flash Viewer",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};

// Read-only pages: the system menu shows them under Help, not among the
// settings above
static const menu_item_t g_help_items[] =
{
    { .kind = MI_ACTION, .label = "Controls",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t flash_viewer_help_menu =
{
    .title = "Flash Viewer",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
