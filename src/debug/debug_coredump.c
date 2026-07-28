/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Debug Coredump System Implementation
 * Provides stack trace and error analysis functionality
 */

#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "debug_coredump.h"
#include "capture.h"
#include "lcd.h"
#include "buttons.h"
#include "timer.h"
#include "utils.h"
#include "ui.h"
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
// The ring lives in the retained SRAM block, addressed the same way capture.c
// addresses its buffers. Nothing is linked there, so the startup code does
// not touch it and the contents survive a reset - which is the whole point:
// both capture paths stop the device, and the dump has to still be there
// after the reboot that follows.
static coredump_store_t *const g_store = (coredump_store_t *)COREDUMP_RAM_BASE;

_Static_assert(sizeof(coredump_store_t) <= COREDUMP_RAM_SIZE,
    "coredump ring does not fit the retained SRAM block");

#define g_coredump_entries     (g_store->entries)
#define g_coredump_count       (g_store->count)
#define g_coredump_write_index (g_store->write_index)

static bool g_ring_retained = false; // ring came from a previous boot
static int g_viewer_current_entry = 0;
static int g_viewer_current_page = 0;
static bool g_viewer_dirty = false;

// Stack/heap bounds provided by the linker (both in TCM)
extern char _end;
extern char _stack_top;

// End of everything in flash, used to recognise return addresses
extern uint32_t __etext;

/*- Local Prototypes --------------------------------------------------------*/
static void capture_stack_trace(stack_frame_t* trace, int* depth, uint32_t sp, uint32_t lr);
static uint32_t get_stack_free_space(void);
static const char* get_error_type_string(error_type_t type);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Plain CRC32 over the ring, excluding the crc field itself. Deliberately
// self-contained: this runs from the fault handler, where the less machinery
// is involved the better.
static uint32_t store_crc(void)
{
    const uint8_t *data = (const uint8_t *)g_store;
    uint32_t length = sizeof(coredump_store_t) - sizeof(g_store->crc);
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
    }

    return ~crc;
}

//-----------------------------------------------------------------------------
static void store_seal(void)
{
    g_store->crc = store_crc();
}

//-----------------------------------------------------------------------------
static bool store_is_valid(void)
{
    if (COREDUMP_MAGIC != g_store->magic)
        return false;

    if (g_store->count < 0 || g_store->count > MAX_COREDUMP_ENTRIES)
        return false;

    if (g_store->write_index < 0 || g_store->write_index >= MAX_COREDUMP_ENTRIES)
        return false;

    return g_store->crc == store_crc();
}

//-----------------------------------------------------------------------------
static void store_reset(uint32_t boot_id)
{
    memset(g_store, 0, sizeof(coredump_store_t));

    g_store->magic = COREDUMP_MAGIC;
    g_store->boot_id = boot_id;

    store_seal();
}

//-----------------------------------------------------------------------------
void debug_coredump_init(void)
{
    g_viewer_current_entry = 0;
    g_viewer_current_page = 0;
    g_ring_retained = false;

    if (store_is_valid())
    {
        // Survived the reset: keep the dumps and start a new boot id
        g_store->boot_id++;

        // "Retained" means specifically that the boot which just ended
        // recorded a dump, i.e. this boot is the one following a crash. Any
        // older dump left in the ring does not count, or the viewer would
        // open itself on every boot until the ring is cleared by hand.
        for (int i = 0; i < g_store->count; i++)
        {
            if (g_store->entries[i].boot_id + 1 == g_store->boot_id)
                g_ring_retained = true;
        }

        store_seal();
    }
    else
    {
        // Cold start, corrupted ring, or a firmware with a different layout
        g_ring_retained = false;
        store_reset(1);
    }
}

//-----------------------------------------------------------------------------
uint32_t debug_coredump_boot_id(void)
{
    return g_store->boot_id;
}

//-----------------------------------------------------------------------------
bool debug_coredump_ring_retained(void)
{
    return g_ring_retained;
}

//-----------------------------------------------------------------------------
// There are no frame pointers in this build, so the trace is a scan: every
// word still on the stack that looks like a Thumb return address into flash
// is reported, newest first. It over-reports (stale slots from earlier calls
// show up too), but it cannot miss the call chain and, unlike the previous
// version, it cannot run off the end of memory either - that one bounded its
// inner scan against 0x20020000, an SRAM address, while walking a stack that
// lives in TCM, so it read past the top of TCM and risked faulting inside the
// fault handler.
static void capture_stack_trace(stack_frame_t* trace, int* depth, uint32_t sp, uint32_t lr)
{
    uint32_t stack_lo = (uint32_t)&_end;
    uint32_t stack_hi = (uint32_t)&_stack_top;
    uint32_t text_lo = 0x08000000;
    uint32_t text_hi = (uint32_t)&__etext;

    *depth = 0;

    // Frame 0 is where the faulting function would have returned to
    if ((lr & 1) && lr >= text_lo && lr < text_hi) {
        trace[0].pc = lr & ~1u;
        trace[0].lr = lr;
        trace[0].sp = sp;
        trace[0].function = lr & ~1u;
        *depth = 1;
    }

    if (sp < stack_lo || sp >= stack_hi)
        return; // stack pointer itself is not plausible, nothing safe to walk

    for (uint32_t addr = (sp + 3) & ~3u;
         addr + 3 < stack_hi && *depth < MAX_STACK_TRACE_DEPTH;
         addr += 4) {
        uint32_t v = *(volatile uint32_t *)addr;

        if ((v & 1) && v >= text_lo && v < text_hi) {
            trace[*depth].pc = v & ~1u;
            trace[*depth].lr = v;
            trace[*depth].sp = addr;
            trace[*depth].function = v & ~1u;
            (*depth)++;
        }
    }
}

//-----------------------------------------------------------------------------
static uint32_t get_stack_free_space(void)
{
    // The address of a local is the current stack pointer, near enough, and
    // unlike inline asm it also compiles for the host tests
    uint32_t here;
    uint32_t current_sp = (uint32_t)(uintptr_t)&here;

    // Free space is what is left between the top of the heap and the
    // current stack pointer (both in TCM)
    if (current_sp < (uint32_t)&_end)
        return 0;

    return current_sp - (uint32_t)&_end;
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
    entry->boot_id = g_store->boot_id;
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

    entry->stack_free = get_stack_free_space();

    // Update counters
    g_coredump_write_index = (g_coredump_write_index + 1) % MAX_COREDUMP_ENTRIES;
    if (g_coredump_count < MAX_COREDUMP_ENTRIES) {
        g_coredump_count++;
    }

    // Seal before the caller halts the device: the ring is only recognised
    // after the reset if its CRC matches
    store_seal();

    g_viewer_dirty = true;
}

//-----------------------------------------------------------------------------
void debug_coredump_capture_hard_fault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t frame = (lr & 4) ? psp : msp;
    coredump_entry_t* entry = &g_coredump_entries[g_coredump_write_index];

    // A stack overflow is one of the likeliest causes of getting here, and
    // then the frame pointer is not a valid address. Dereferencing it would
    // fault again inside the fault handler, which escalates to lockup and
    // loses the dump entirely - so it is checked before it is used.
    bool frame_ok = (frame >= (uint32_t)&_end) &&
                    (frame + 8 * sizeof(uint32_t) <= (uint32_t)&_stack_top);
    const uint32_t *sp = (const uint32_t *)frame;

    // Clear the entry
    memset(entry, 0, sizeof(coredump_entry_t));

    // Basic information
    entry->type = ERROR_TYPE_HARD_FAULT;
    strcpy(entry->message, frame_ok ? "Hard Fault Exception"
                                    : "Hard Fault, exception frame lost");
    entry->boot_id = g_store->boot_id;
    entry->timestamp = timer_ms();
    entry->error_lr = lr;
    entry->error_sp = frame;

    if (frame_ok) {
        entry->error_pc = sp[6];  // PC from exception stack frame
        entry->r0 = sp[0];
        entry->r1 = sp[1];
        entry->r2 = sp[2];
        entry->r3 = sp[3];
        entry->r12 = sp[4];
        entry->psr = sp[7];
    }

    // Fault status registers
    entry->cfsr = SCB->CFSR;
    entry->hfsr = SCB->HFSR;
    entry->dfsr = SCB->DFSR;
    entry->afsr = SCB->AFSR;
    entry->mmar = SCB->MMFAR;
    entry->bfar = SCB->BFAR;

    // Capture stack trace
    capture_stack_trace(entry->stack_trace, &entry->stack_depth,
                       frame, frame_ok ? sp[5] : 0);

    entry->stack_free = get_stack_free_space();

    // Update counters
    g_coredump_write_index = (g_coredump_write_index + 1) % MAX_COREDUMP_ENTRIES;
    if (g_coredump_count < MAX_COREDUMP_ENTRIES) {
        g_coredump_count++;
    }

    // Seal it: main.c halts on the fault screen and the user reboots from
    // there, so this ring has to validate on the way back up
    store_seal();
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
// Viewer
//
// Pages: the list, and three detail pages for the selected dump. Only what
// changed is repainted - moving the selection touches two rows, not the whole
// 320x240 panel.
//-----------------------------------------------------------------------------
#define PAGE_LIST       0
#define PAGE_DETAILS    1
#define PAGE_REGS       2
#define PAGE_STACK      3
#define PAGE_COUNT      4

#define HEADER_H        26
#define FOOTER_H        22
#define BODY_Y          HEADER_H
#define ROW_H           22
#define LINE_H          10

#define CD_BG           LCD_BLACK_COLOR
#define CD_FG           LCD_COLOR(220, 220, 220)
#define CD_PANEL        LCD_COLOR(20, 20, 40)
#define CD_ACCENT       LCD_COLOR(255, 200, 0)
#define CD_SEL          LCD_COLOR(0, 120, 215)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Entries are shown oldest first; the ring's newest entry sits at
// write_index - 1
static coredump_entry_t *entry_at(int index)
{
    int idx = (g_coredump_write_index - g_coredump_count + index +
        2 * MAX_COREDUMP_ENTRIES) % MAX_COREDUMP_ENTRIES;

    return &g_coredump_entries[idx];
}

//-----------------------------------------------------------------------------
static void draw_header(const char *title)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, HEADER_H, CD_PANEL);
    lcd_fill_rect(0, HEADER_H - 1, LCD_WIDTH, 1, CD_FG);

    lcd_set_font(FONT_LARGE);
    lcd_set_color(CD_PANEL, CD_ACCENT);
    lcd_puts(8, 5, title);
}

//-----------------------------------------------------------------------------
static void draw_footer(const char *l1, const char *l2)
{
    lcd_fill_rect(0, LCD_HEIGHT - FOOTER_H, LCD_WIDTH, FOOTER_H, CD_PANEL);
    lcd_fill_rect(0, LCD_HEIGHT - FOOTER_H, LCD_WIDTH, 1, CD_FG);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(CD_PANEL, CD_FG);
    lcd_puts(8, LCD_HEIGHT - FOOTER_H + 4, l1);
    lcd_puts(8, LCD_HEIGHT - FOOTER_H + 13, l2);
}

//-----------------------------------------------------------------------------
static void draw_list_row(int index)
{
    coredump_entry_t *entry = entry_at(index);
    int y = BODY_Y + 4 + index * ROW_H;
    bool selected = (index == g_viewer_current_entry);
    uint16_t bg = selected ? CD_SEL : CD_BG;
    char line[64];

    lcd_fill_rect(0, y - 2, LCD_WIDTH, ROW_H, bg);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(bg, CD_FG);

    snprintf(line, sizeof(line), "%d. %s%s", index + 1,
        get_error_type_string(entry->type),
        (entry->boot_id == g_store->boot_id) ? "" : "  (before reset)");
    lcd_puts(8, y, line);

    snprintf(line, sizeof(line), "   PC 0x%08lX  t=%lu ms  boot %lu",
        (unsigned long)entry->error_pc, (unsigned long)entry->timestamp,
        (unsigned long)entry->boot_id);
    lcd_puts(8, y + LINE_H, line);
}

//-----------------------------------------------------------------------------
static void draw_coredump_list(void)
{
    char title[32];

    snprintf(title, sizeof(title), "CRASH DUMPS (%d)", (int)g_coredump_count);
    draw_header(title);

    lcd_fill_rect(0, BODY_Y, LCD_WIDTH, LCD_HEIGHT - BODY_Y - FOOTER_H, CD_BG);

    if (0 == g_coredump_count) {
        char buf[64];

        lcd_set_font(FONT_SMALL);
        lcd_set_color(CD_BG, CD_FG);
        lcd_puts(8, BODY_Y + 20, "No crash dumps recorded.");
        lcd_puts(8, BODY_Y + 38, "Dumps are written by the hard fault handler");
        lcd_puts(8, BODY_Y + 50, "and by error(). They live in retained SRAM,");
        lcd_puts(8, BODY_Y + 62, "so after a crash you reboot from the fault");
        lcd_puts(8, BODY_Y + 74, "screen and the dump is waiting here.");

        // Accented, because this is the sentence people come back about: a
        // test dump added here is gone after the next power cycle, and that
        // is the storage working as built, not a dump that failed to save
        lcd_set_color(CD_BG, CD_ACCENT);
        lcd_puts(8, BODY_Y + 92, "Cutting the power clears them. Read a dump");
        lcd_puts(8, BODY_Y + 104, "before switching off, or it is gone.");
        lcd_set_color(CD_BG, CD_FG);

        snprintf(buf, sizeof(buf), "This is boot %lu.",
            (unsigned long)g_store->boot_id);
        lcd_puts(8, BODY_Y + 122, buf);

        lcd_puts(8, BODY_Y + 140, "MENU > Add test dump records a synthetic one;");
        lcd_puts(8, BODY_Y + 152, "it survives Advanced > Reboot, not a power cut.");

        draw_footer("MENU: settings and help",
            "SHIFT+MENU: back to the launcher");
        return;
    }

    for (int i = 0; i < g_coredump_count; i++)
        draw_list_row(i);

    // The volatility belongs on the screen that shows the dumps, not only on
    // the empty one and in Help. A list of recorded crashes reads as something
    // that was stored; these are in RAM, and switching the device off ends
    // them. Anyone who needs one kept has to read it before powering down.
    draw_footer("UP/DOWN: select  MODE: open  MENU: more",
        "In RAM: kept over a reboot, lost on power off.");
}

//-----------------------------------------------------------------------------
static void draw_detail_header(const char *what)
{
    char title[32];

    snprintf(title, sizeof(title), "DUMP %d - %s",
        g_viewer_current_entry + 1, what);
    draw_header(title);

    lcd_fill_rect(0, BODY_Y, LCD_WIDTH, LCD_HEIGHT - BODY_Y - FOOTER_H, CD_BG);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(CD_BG, CD_FG);
}

//-----------------------------------------------------------------------------
static void draw_detail_footer(void)
{
    draw_footer("LEFT: back to the list   UP/DOWN: page",
        "In RAM: kept over a reboot, lost on power off.");
}

//-----------------------------------------------------------------------------
static void draw_coredump_details(void)
{
    coredump_entry_t *entry = entry_at(g_viewer_current_entry);
    char buf[64];
    int y = BODY_Y + 8;

    draw_detail_header("Overview");

    lcd_set_color(CD_BG, CD_ACCENT);
    lcd_puts(8, y, get_error_type_string(entry->type));
    lcd_set_color(CD_BG, CD_FG);
    y += 14;

    // The message can be longer than the panel: cut it into 52 char rows
    for (int i = 0; i < MAX_ERROR_MESSAGE_LEN && entry->message[i]; i += 52) {
        strncpy(buf, &entry->message[i], 52);
        buf[52] = 0;
        lcd_puts(8, y, buf);
        y += LINE_H;
    }

    y += 6;

    snprintf(buf, sizeof(buf), "Boot  %lu%s", (unsigned long)entry->boot_id,
        (entry->boot_id == g_store->boot_id) ? " (this session)"
                                             : " (before the last reset)");
    lcd_puts(8, y, buf); y += LINE_H;

    snprintf(buf, sizeof(buf), "Time  %lu ms after that boot",
        (unsigned long)entry->timestamp);
    lcd_puts(8, y, buf); y += LINE_H;

    snprintf(buf, sizeof(buf), "PC    0x%08lX", (unsigned long)entry->error_pc);
    lcd_puts(8, y, buf); y += LINE_H;

    snprintf(buf, sizeof(buf), "LR    0x%08lX", (unsigned long)entry->error_lr);
    lcd_puts(8, y, buf); y += LINE_H;

    snprintf(buf, sizeof(buf), "SP    0x%08lX", (unsigned long)entry->error_sp);
    lcd_puts(8, y, buf); y += LINE_H + 6;

    snprintf(buf, sizeof(buf), "Stack headroom  %lu bytes",
        (unsigned long)entry->stack_free);
    lcd_puts(8, y, buf); y += LINE_H;

    snprintf(buf, sizeof(buf), "Stack frames    %d", entry->stack_depth);
    lcd_puts(8, y, buf);

    draw_detail_footer();
}

//-----------------------------------------------------------------------------
static void draw_registers(void)
{
    coredump_entry_t *entry = entry_at(g_viewer_current_entry);
    char buf[48];
    int y = BODY_Y + 8;

    draw_detail_header("Registers");

    static const char *const names[] = { "R0", "R1", "R2", "R3", "R12", "PSR" };
    uint32_t values[6];

    values[0] = entry->r0;
    values[1] = entry->r1;
    values[2] = entry->r2;
    values[3] = entry->r3;
    values[4] = entry->r12;
    values[5] = entry->psr;

    // Two columns of three
    for (int i = 0; i < 6; i++) {
        snprintf(buf, sizeof(buf), "%-4s 0x%08lX", names[i],
            (unsigned long)values[i]);
        lcd_puts((i < 3) ? 8 : 168, y + (i % 3) * LINE_H, buf);
    }

    y += 3 * LINE_H + 8;

    lcd_set_color(CD_BG, CD_ACCENT);
    lcd_puts(8, y, "Fault status");
    lcd_set_color(CD_BG, CD_FG);
    y += 14;

    static const char *const fnames[] = { "CFSR", "HFSR", "DFSR", "AFSR", "MMAR", "BFAR" };
    uint32_t fvalues[6];

    fvalues[0] = entry->cfsr;
    fvalues[1] = entry->hfsr;
    fvalues[2] = entry->dfsr;
    fvalues[3] = entry->afsr;
    fvalues[4] = entry->mmar;
    fvalues[5] = entry->bfar;

    for (int i = 0; i < 6; i++) {
        snprintf(buf, sizeof(buf), "%-4s 0x%08lX", fnames[i],
            (unsigned long)fvalues[i]);
        lcd_puts((i < 3) ? 8 : 168, y + (i % 3) * LINE_H, buf);
    }

    draw_detail_footer();
}

//-----------------------------------------------------------------------------
static void draw_stack_trace(void)
{
    coredump_entry_t *entry = entry_at(g_viewer_current_entry);
    char buf[48];
    int y = BODY_Y + 8;

    draw_detail_header("Stack");

    snprintf(buf, sizeof(buf), "%d frames recovered", entry->stack_depth);
    lcd_puts(8, y, buf);
    y += 16;

    if (0 == entry->stack_depth) {
        lcd_puts(8, y, "The unwinder found no return addresses.");
        draw_detail_footer();
        return;
    }

    // Two columns of eight, two lines each: all MAX_STACK_TRACE_DEPTH frames
    // fit on one page, so there is nothing left to scroll to
    for (int i = 0; i < entry->stack_depth && i < MAX_STACK_TRACE_DEPTH; i++) {
        int col = (i < 8) ? 8 : 168;
        int ry = y + (i % 8) * 2 * LINE_H;

        snprintf(buf, sizeof(buf), "%2d LR 0x%08lX", i,
            (unsigned long)entry->stack_trace[i].lr);
        lcd_puts(col, ry, buf);

        snprintf(buf, sizeof(buf), "   SP 0x%08lX",
            (unsigned long)entry->stack_trace[i].sp);
        lcd_puts(col, ry + LINE_H, buf);
    }

    draw_detail_footer();
}

//-----------------------------------------------------------------------------
static void draw_page(void)
{
    switch (g_viewer_current_page) {
        case PAGE_DETAILS: draw_coredump_details(); break;
        case PAGE_REGS:    draw_registers(); break;
        case PAGE_STACK:   draw_stack_trace(); break;
        default:           draw_coredump_list(); break;
    }
}

//-----------------------------------------------------------------------------
void coredump_app_init(void)
{
    g_viewer_current_entry = 0;
    g_viewer_current_page = PAGE_LIST;

    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, CD_BG);
    draw_page();
}

//-----------------------------------------------------------------------------
// Paints straight away rather than raising the dirty flag: an overlay can
// close while another one is still open, and only the top screen is ticked,
// so a deferred repaint would not happen until the whole stack unwound
void coredump_app_redraw(void)
{
    g_viewer_dirty = false;
    draw_page();
}

//-----------------------------------------------------------------------------
void coredump_app_task(void)
{
    if (!g_viewer_dirty)
        return;

    g_viewer_dirty = false;
    draw_page();
}

//-----------------------------------------------------------------------------
void coredump_app_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);

    if (PAGE_LIST == g_viewer_current_page) {
        if (0 == g_coredump_count)
            return;

        if ((buttons & BTN_UP) && g_viewer_current_entry > 0) {
            int prev = g_viewer_current_entry--;

            draw_list_row(prev);            // repaint the two rows that
            draw_list_row(g_viewer_current_entry); // changed, not the screen
        }

        if ((buttons & BTN_DOWN) && g_viewer_current_entry < g_coredump_count - 1) {
            int prev = g_viewer_current_entry++;

            draw_list_row(prev);
            draw_list_row(g_viewer_current_entry);
        }

        if ((buttons & (BTN_RIGHT | BTN_MODE)) && !repeat) {
            g_viewer_current_page = PAGE_DETAILS;
            g_viewer_dirty = true;
        }

        return;
    }

    if ((buttons & BTN_LEFT) && !repeat) {
        g_viewer_current_page = PAGE_LIST;
        g_viewer_dirty = true;
        return;
    }

    if ((buttons & BTN_UP) && !repeat) {
        g_viewer_current_page = (PAGE_DETAILS == g_viewer_current_page)
            ? PAGE_COUNT - 1 : g_viewer_current_page - 1;
        g_viewer_dirty = true;
    }

    if ((buttons & BTN_DOWN) && !repeat) {
        g_viewer_current_page = (PAGE_COUNT - 1 == g_viewer_current_page)
            ? PAGE_DETAILS : g_viewer_current_page + 1;
        g_viewer_dirty = true;
    }
}

//-----------------------------------------------------------------------------
void coredump_app_cleanup(void)
{
    // Nothing to release: the ring belongs to the coredump system, not to the
    // viewer, and outlives it
}

//-----------------------------------------------------------------------------
int debug_coredump_get_count(void)
{
    return g_coredump_count;
}

//-----------------------------------------------------------------------------
void debug_coredump_clear_all(void)
{
    uint32_t boot_id = g_store->boot_id;

    store_reset(boot_id);
    g_ring_retained = false;
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void action_add_test_dump(const void *arg)
{
    uint32_t here; // its address is the current stack pointer

    (void)arg;

    debug_coredump_capture(ERROR_TYPE_GENERAL_ERROR,
        "Synthetic dump from the viewer menu",
        (uint32_t)(uintptr_t)&action_add_test_dump,
        (uint32_t)(uintptr_t)&action_add_test_dump,
        (uint32_t)(uintptr_t)&here);

    g_viewer_current_page = PAGE_LIST;
    g_viewer_current_entry = g_coredump_count - 1;

    menu_close_popups();
}

//-----------------------------------------------------------------------------
static void action_clear_all(const void *arg)
{
    (void)arg;

    debug_coredump_clear_all();

    g_viewer_current_page = PAGE_LIST;
    g_viewer_current_entry = 0;

    menu_close_popups();
}

static const char *const g_help_lines[] =
{
    "UP/DOWN        - Select dump / change page",
    "MODE or RIGHT  - Open the selected dump",
    "LEFT           - Back to the list",
    "MENU           - This menu",
    "SHIFT+MENU     - Back to the launcher",
    "",
    "Dumps are written by the hard fault handler",
    "and by error(), then the device stops on its",
    "error screen. Rebooting from there brings you",
    "back here with the dump intact: the ring sits",
    "in a part of SRAM that nothing is linked into,",
    "so reset does not clear it.",
    "",
    "SRAM is not storage. Switching the device off",
    "ends every dump in the list, including one you",
    "added with Add test dump - that is the design,",
    "not a save that failed. Read what you need off",
    "the screen before you power down.",
    "",
    "Each entry records which boot it came from.",
};

static const info_page_t g_help_page =
{
    .title = "CoreDump Viewer",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_ACTION, .label = "Add test dump",
      .u.action = { action_add_test_dump, NULL } },
    { .kind = MI_ACTION, .label = "Clear all dumps",
      .u.action = { action_clear_all, NULL } },
};

const menu_def_t coredump_menu =
{
    .title = "CoreDump Viewer",
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

const menu_def_t coredump_help_menu =
{
    .title = "CoreDump Viewer",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
