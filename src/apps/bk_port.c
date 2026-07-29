/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Электроника БК-0010-01.
 *
 * A 1985 Soviet home computer, whole: a К1801ВМ1 - which is a PDP-11 - with
 * 32 KB of RAM, 32 KB of ROM, a screen that is 16 KB read straight out by the
 * video hardware, and five registers at the top of memory for everything else.
 *
 * The emulator is in src/bk/, split so that the parts that are the machine are
 * separate from the parts that are this instrument: bk_cpu.c knows nothing
 * about a panel, bk_video.c knows nothing about SPI flash, and this file is
 * the only one that includes lcd.h and buttons.h. Everything about it is
 * switchable from bk_config.h, because a firmware image with 10% of its flash
 * left cannot afford a floating point unit for a machine that never had one.
 *
 * Where things live while this is on screen:
 *
 *   the 64 KB address space   the SRAM the capture ring is not using
 *   ROM images and programs   the 8 MB SPI part, through spifs
 *   the screen                converted a line at a time, only where it changed
 *
 * What the machine is loaded with is a question with three answers, and the
 * picker on the way in is where it gets asked. There is no monitor ROM in this
 * repository and there is not going to be - it is the machine's firmware, not
 * ours - so a device with nothing on its chip gets the stand-in in bk_load.c,
 * which is enough to start a self-contained program and honest about being
 * nothing more.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "utils.h"
#include "ui.h"
#include "capture.h"
#include "spifs.h"
#include "menu_widget.h"
#include "bk_port.h"
#include "bk_config.h"
#include "bk_cpu.h"
#include "bk_mem.h"
#include "bk_io.h"
#include "bk_video.h"
#include "bk_load.h"

#if BK_DISASM
#include "bk_disasm.h"
#endif

/*- Definitions -------------------------------------------------------------*/
/*
 * The machine's whole address space, in the SRAM the capture ring occupies
 * while the oscilloscope is running. Acquisition is stopped on the way in, the
 * same trade DOOM makes, and the instrument rebuilds itself when it is next
 * opened.
 */
#define BK_RAM_ADDR             0x20000000u
#define BK_RAM_LIMIT            (CAPTURE_SPARE_RAM - BK_RAM_ADDR)

/*
 * Everything else the emulator needs goes immediately above the machine, in
 * the same block and for the same reason.
 *
 * .bss on this firmware means the 64 KB of tightly coupled memory, and the
 * instrument had 300 bytes of it left before this application existed. The
 * main SRAM is the other way round: 120 KB of it, all of it dead while an
 * application is on screen, none of it linked - which is why the emulated
 * machine is already there and why its scratch follows it. DOOM answers the
 * same question the same way; see doom_mem_t.
 *
 * What stays in .bss is the settings, thirty bytes of them, because those are
 * the user's choices and have to survive leaving the application.
 */
#define BK_BLIT_ADDR            (BK_RAM_ADDR + BK_MEM_SIZE)
#define BK_RUN_ADDR             (BK_BLIT_ADDR + BK_VIDEO_BUFFER_SIZE)

// 50 Hz off the video timing, in processor cycles
#define EVNT_PERIOD             (BK_CPU_HZ / 50)

/*
 * How often the panel is repainted, in milliseconds.
 *
 * A BK refreshes fifty times a second and that is what its programs are
 * written against, so anything more often is a picture nobody can see being
 * paid for in bit-banged pixels. Painting every pass of the task loop came to
 * seventeen thousand repaints a second, most of them one row - all of it time
 * the emulated processor could have had.
 */
#define REPAINT_MS              20

// How often a held key is presented to the machine again. The BK's own repeat
// is a key on the keyboard; this is the rate that makes walking feel right.
#define KEY_REPEAT_MS           60

/*
 * The picker's list is two sources and a tail: the loose .bin files spifs can
 * name, then the programs inside bkgames.pak, then "run nothing".
 *
 * MAX_FILES bounds only the first of those, because those are the ones whose
 * names are held in RAM. The pack's index stays on the chip and is read a
 * record at a time, so how many programs are in it costs nothing here - which
 * is why the list is a window onto the names rather than all of them: PICK_ROWS
 * lines between the heading and the footer, scrolled to keep the selection
 * inside it.
 */
#define MAX_FILES               SPIFS_MAX_FILES
#define PICK_ROWS               12
#define PICK_TOP                58
#define PICK_ROW_H              12
#define PICK_NAME_MAX           (BK_PACK_NAME + 1)

/*
 * The multiple of a real machine, in hundredths.
 *
 * The interesting number when the clock comes off. A БК-0010-01 ran its
 * К1801ВМ1 at 3 MHz in 1985; this part is a 250 MHz Cortex-M4 from 2014
 * pretending to be one, and how many times over it manages that is the whole
 * comparison in one figure.
 */
#define TIMES_REAL(khz)         ((khz) * 100u / (BK_CPU_HZ / 1000u))

#define HUD_FG                  LCD_COLOR(200, 200, 200)
#define HUD_HI                  LCD_COLOR(255, 190, 60)
#define HUD_BAD                 LCD_COLOR(230, 70, 50)

/*- Types -------------------------------------------------------------------*/
typedef enum
{
    ST_PICK = 0,        // which program to run
    ST_RUN,
    ST_FAULT,           // the machine stopped, and why
#if BK_DEBUGGER
    ST_DEBUG,
#endif
} state_t;

// Only meaningful while this application is the one on screen, so it lives
// where the machine does rather than in .bss. See BK_RUN_ADDR above.
typedef struct
{
    const char *files[MAX_FILES];   // pointers into spifs's own table
    int file_count;
    int pack_count;                 // programs in bkgames.pak, 0 if none
    bool have_roms;

    uint32_t last_ms;
    uint32_t evnt_left;
    uint32_t draw_ms;

    uint8_t key_code;
    uint32_t key_ms;

#if BK_STATS
    uint16_t fps;
    uint32_t frames, fps_ms;
    uint32_t cycles_mark;
    uint32_t khz;
    uint32_t rows;
    uint32_t status_ms;
    char statline[64];
#endif
} bk_run_t;

_Static_assert(BK_MEM_SIZE + BK_VIDEO_BUFFER_SIZE + sizeof(bk_run_t) <=
    BK_RAM_LIMIT, "the BK working set does not fit below the capture spare RAM");

/*- Variables ---------------------------------------------------------------*/
static bk_run_t *rt;

static state_t g_state;
static int g_file_sel;
static int g_file_top;              // first name on screen, see draw_picker

// Settings, all reachable from the application menu
static int g_screen_mode;           // BK_SCREEN_*
/*
 * Squash by default, not crop.
 *
 * 256 lines have to become 240 somehow. Crop keeps every line its own size and
 * loses sixteen of them off the top and bottom - which is exactly where a BK
 * game puts its border, its score line and its status text, because that is
 * where a television has overscan. Boulder Dash loses the top and bottom walls
 * of the cave that way.
 *
 * Squash drops one line in sixteen instead. The seam is there if you look for
 * it; a missing wall is there whether you look or not.
 */
static int g_fit_mode = BK_FIT_SQUASH;
static int g_top_line = 8;
static int g_pan;
static int g_mono_fit;              // BK_MONO_*
/*
 * How fast the machine runs, as a preset and a trim.
 *
 * The preset is what the emulated processor is nominally clocked at; the trim
 * is the correction on top, and it goes down to 5% because the cycle counts in
 * bk_cpu.c are the shape of the 1801's timings rather than measured silicon.
 * If a game runs wrong, this is the knob - and the status column shows the
 * rate actually achieved, so it can be dialled in against a number.
 *
 * Unlimited is the last preset: no clock at all, just as much of the machine
 * as fits between two repaints. What that comes to depends entirely on what
 * the program is doing.
 */
static int g_speed_preset;          // index into g_speed_labels
static int g_speed_trim = 100;      // per cent
static bool g_show_status;
static bool g_joystick = true;

// The multipliers behind the preset names, and Unlimited as the last of them
static const char *const g_speed_labels[] =
{
    "Original 3 MHz", "x2", "x4", "Unlimited"
};

#define SPEED_UNLIMITED         (ARRAY_SIZE(g_speed_labels) - 1)

static const uint8_t g_speed_mult[] = { 1, 2, 4, 1 };

// How long a pass may spend inside the machine when nothing is throttling it.
// The panel still has to be painted and the keys still have to be read.
#define UNLIMITED_BUDGET_US     8000u

static const char *const g_screen_labels[] = { "Colour 256", "Mono 512" };
static const char *const g_fit_labels[] = { "Crop", "Squash" };
static const char *const g_mono_labels[] = { "Whole 512", "1:1 window" };

/*- Local prototypes --------------------------------------------------------*/
static void draw_picker(void);
static void draw_fault(void);
static void start_machine(void);
static void apply_video(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void apply_video(void)
{
    bk_video_set_screen((bk_screen_t)g_screen_mode);
    bk_video_set_fit((bk_fit_t)g_fit_mode);
    bk_video_set_top(g_top_line);
    bk_video_set_mono((bk_mono_t)g_mono_fit);
    bk_video_set_pan(g_pan);
    bk_video_set_status(g_show_status);
}

//-----------------------------------------------------------------------------
#if BK_IO_KEYBOARD
// The front panel, as a BK keyboard.
//
// The instrument reads its keys through two priority encoders and only one key
// per group is visible at a time, which is why the arrows are the four they
// are: UP and RIGHT share a group and so do DOWN and LEFT, so of the four
// diagonals two are reachable and two are not. No BK game needs a diagonal -
// this machine's own arrow keys are on one row - so nothing is lost.
static uint8_t map_key(int buttons)
{
    if (buttons & BTN_UP)
        return BK_KEY_UP;

    if (buttons & BTN_DOWN)
        return BK_KEY_DOWN;

    if (buttons & BTN_LEFT)
        return BK_KEY_LEFT;

    if (buttons & BTN_RIGHT)
        return BK_KEY_RIGHT;

    if (buttons & BTN_MODE)
        return BK_KEY_ENTER;

    if (buttons & BTN_F1)
        return BK_KEY_SPACE;

    if (buttons & BTN_AUTO)
        return BK_KEY_TAB;

    return 0;
}
#endif

//-----------------------------------------------------------------------------
#if BK_IO_JOYSTICK
static uint8_t map_joystick(int buttons)
{
    uint8_t mask = 0;

    if (buttons & BTN_UP)
        mask |= BK_JOY_UP;

    if (buttons & BTN_DOWN)
        mask |= BK_JOY_DOWN;

    if (buttons & BTN_LEFT)
        mask |= BK_JOY_LEFT;

    if (buttons & BTN_RIGHT)
        mask |= BK_JOY_RIGHT;

    if (buttons & BTN_F1)
        mask |= BK_JOY_BUTTON1;

    if (buttons & BTN_MODE)
        mask |= BK_JOY_BUTTON2;

    return mask;
}
#endif

//-----------------------------------------------------------------------------
// Present the keyboard to the machine.
//
// A held key is offered again every KEY_REPEAT_MS, because most BK games read
// the code register and clear it, then read it again next frame - a key
// presented once would move a character one square however long it was held.
// The "a key is down" bit of 0177716 is asserted for as long as it is, which
// is the other idiom, and both work without the game having to be told which.
static void poll_keys(void)
{
    int buttons = buttons_state();

#if BK_IO_JOYSTICK
    bk_io_joystick(g_joystick ? map_joystick(buttons) : 0);
#endif

#if BK_IO_KEYBOARD
    {
        uint8_t code = map_key(buttons);
        uint32_t now = timer_ms();

        if (0 == code)
        {
            if (rt->key_code)
            {
                bk_io_key_release();
                rt->key_code = 0;
            }

            return;
        }

        if (code != rt->key_code)
        {
            // A different key is down now, so whatever is still sitting in the
            // register unread is not what the player means any more
            bk_io_key_flush();
            bk_io_key(code, false);
            rt->key_code = code;
            rt->key_ms = now;
        }
        else if (now - rt->key_ms >= KEY_REPEAT_MS)
        {
            bk_io_key(code, false);
            rt->key_ms = now;
        }
    }
#else
    (void)buttons;
#endif
}

//-----------------------------------------------------------------------------
// One chunk of the machine, stopping at each 50 Hz edge to raise the line the
// whole tempo comes from. Returns the cycles spent.
static uint32_t run_cycles(uint32_t budget)
{
    uint32_t spent = 0;

    while (budget > 0 && !bk_cpu.stopped)
    {
        uint32_t chunk = (budget < rt->evnt_left) ? budget : rt->evnt_left;
        uint32_t used;

        if (0 == chunk)
            chunk = 1;

        used = bk_cpu_run(chunk);
        spent += used;

        budget -= (used > budget) ? budget : used;
        rt->evnt_left -= (used > rt->evnt_left) ? rt->evnt_left : used;

        if (0 == rt->evnt_left)
        {
            bk_io_frame();
            rt->evnt_left = EVNT_PERIOD;
        }
    }

    return spent;
}

//-----------------------------------------------------------------------------
// Run for as much emulated time as has really passed - or, with no clock at
// all, for as much of the panel's time as can be spared.
static void run_slice(uint32_t ms)
{
    uint32_t budget;
    uint32_t spent;

    if (bk_cpu.stopped)
        return;

    if (SPEED_UNLIMITED == (unsigned)g_speed_preset)
    {
        uint32_t start = timer_us();

        spent = 0;

        do
        {
            spent += run_cycles(BK_CPU_HZ / 1000u);
        }
        while (!bk_cpu.stopped && timer_us() - start < UNLIMITED_BUDGET_US);

        // Time still passes for the machine, at whatever rate it managed
        bk_io_advance(spent / (BK_CPU_HZ / 1000000u));
        return;
    }

    budget = ms * (BK_CPU_HZ / 1000u) * g_speed_mult[g_speed_preset];
    budget = budget / 100u * (uint32_t)g_speed_trim;

    run_cycles(budget);

    bk_io_advance(ms * 1000u);
}

//-----------------------------------------------------------------------------
#if BK_STATS
//-----------------------------------------------------------------------------
// "x1.0" for a machine keeping the original's time, "x340" for one that is
// not. Two significant figures below ten, none above - five characters is what
// the margin holds, and a hundredth of a times-real is nobody's question.
static void times_real(char *buf, int size)
{
    uint32_t pct = TIMES_REAL(rt->khz);

    if (pct >= 1000u)
        snprintf(buf, size, "x%-4lu", (unsigned long)(pct / 100u));
    else
        snprintf(buf, size, "x%lu.%lu", (unsigned long)(pct / 100u),
            (unsigned long)((pct % 100u) / 10u));
}

/*
 * The reporting column.
 *
 * It goes in the 64 pixels of panel the picture does not reach, not under the
 * picture: 256 dots in a 320 pixel display leave a margin that was black, and
 * taking sixteen rows off a machine that already cannot fit its 256 lines into
 * 240 is paying twice for the same thing. In mono there is no margin - 512
 * dots windowed to 320 fills the panel - and there it is a line at the bottom,
 * because there is nowhere else it could be.
 *
 * Rows, not just frames per second, because the panel is bit-banged and a row
 * is the unit it is slow in: 240 rows and 40 rows at the same frame rate are
 * two different machines.
 *
 * kHz is what the emulated processor actually achieved. That is the number the
 * Speed setting is dialled against - a real BK-0010-01 reads 3000.
 */
static void draw_status(bool force)
{
    char buf[16];
    uint32_t now = timer_ms();
    int x, y, w, h;

    if (!g_show_status)
        return;

    if (!force && now - rt->status_ms < 250)
        return;

    rt->status_ms = now;

    bk_video_spare(&x, &y, &w, &h);

    if (w <= 0)
        return;

    lcd_set_font(FONT_SMALL);

    if (w < LCD_WIDTH)
    {
        /*
         * The margin beside the picture: 32 pixels, which is five characters
         * of the small font, so each number gets a label above it and the
         * program counter is not here at all - SAVE opens the register view
         * for that.
         *
         * Every value is padded to a fixed width rather than the column being
         * erased first: a fill here is bit-banged panel like everything else,
         * and doing it four times a second to make room for five characters
         * would cost more than the emulator saves by not repainting clean
         * lines.
         */
        static const int line_h = 9;
        int row = y + 2;

        if (force)
            lcd_fill_rect(x, y, w, h, LCD_BLACK_COLOR);

        lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
        lcd_puts(x + 2, row, "fps");
        lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
        snprintf(buf, sizeof(buf), "%-5u", rt->fps);
        lcd_puts(x + 2, row + line_h, buf);
        row += 2 * line_h + 7;

        lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
        lcd_puts(x + 2, row, "kHz");
        lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
        snprintf(buf, sizeof(buf), "%-5lu", (unsigned long)rt->khz);
        lcd_puts(x + 2, row + line_h, buf);
        row += 2 * line_h + 7;

        lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
        lcd_puts(x + 2, row, "rows");
        lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
        snprintf(buf, sizeof(buf), "%-5lu", (unsigned long)rt->rows);
        lcd_puts(x + 2, row + line_h, buf);
        row += 2 * line_h + 7;

        // How many real BKs this is worth, which is the number the whole
        // exercise is really about
        lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
        lcd_puts(x + 2, row, "vs BK");
        lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
        times_real(buf, sizeof(buf));
        lcd_puts(x + 2, row + line_h, buf);

        return;
    }

    // Mono: one line across the bottom, which is all there is room for
    {
        char line[sizeof(rt->statline)];

        {
            char mult[16];

            times_real(mult, sizeof(mult));
            snprintf(line, sizeof(line), "%u fps  %lu kHz  %s  %lu rows",
                rt->fps, (unsigned long)rt->khz, mult,
                (unsigned long)rt->rows);
        }

        if (!force && 0 == strcmp(line, rt->statline))
            return;

        {
            int len = strlen(line);

            while (len < (int)sizeof(line) - 1)
                line[len++] = ' ';

            line[len] = 0;
        }

        lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
        lcd_puts(x + 2, y + 4, line);

        memcpy(rt->statline, line, sizeof(rt->statline));
    }
}
#endif

//-----------------------------------------------------------------------------
// How many programs are on offer, the "boot with nothing" tail not counted.
static int pick_total(void)
{
    return rt->file_count + rt->pack_count;
}

//-----------------------------------------------------------------------------
// The name of entry `index`. A loose file's name is already in RAM; a pack
// entry's is read off the chip into `buf`, which is why this returns a pointer
// rather than filling one in - only twelve of them are ever on screen.
static const char *pick_name(int index, char *buf, uint32_t max)
{
    if (index < rt->file_count)
        return rt->files[index];

#if BK_LOAD_SPIFS
    if (index < pick_total() &&
        bk_load_pack_name(index - rt->file_count, buf, max))
        return buf;
#else
    (void)buf;
    (void)max;
#endif

    return (index < pick_total()) ? "(unreadable)" : "(nothing - just boot)";
}

//-----------------------------------------------------------------------------
// The names, and only the names: everything else on the picker is fixed, and
// repainting the whole screen for each press of DOWN is a bit-banged 320x240
// clear that the list does not need.
static void draw_picker_list(void)
{
    int last = pick_total();            // the extra entry: boot with nothing
    int y = PICK_TOP;
    char name[PICK_NAME_MAX];
    char pos[24];

    // Scroll only when the selection would leave the window, so a list that
    // fits never moves and a long one moves a line at a time
    if (g_file_sel < g_file_top)
        g_file_top = g_file_sel;
    else if (g_file_sel >= g_file_top + PICK_ROWS)
        g_file_top = g_file_sel - PICK_ROWS + 1;

    if (g_file_top > last - PICK_ROWS + 1)
        g_file_top = last - PICK_ROWS + 1;

    if (g_file_top < 0)
        g_file_top = 0;

    lcd_fill_rect(0, PICK_TOP - 2, LCD_WIDTH, PICK_ROWS * PICK_ROW_H + 4,
        LCD_BLACK_COLOR);

    lcd_set_font(FONT_SMALL);

    for (int row = 0; row < PICK_ROWS; row++, y += PICK_ROW_H)
    {
        int i = g_file_top + row;

        if (i > last)
            break;

        lcd_set_color(LCD_BLACK_COLOR, (i == g_file_sel) ? HUD_HI : HUD_FG);
        lcd_puts(24, y, (i == g_file_sel) ? ">" : " ");
        lcd_puts(40, y, pick_name(i, name, sizeof(name)));
    }

    /*
     * Where in the list this is, for a window that shows a tenth of it. It
     * sits on the footer line rather than in the rows, so it gets its own
     * clear - the rows' one stops above it and a shrinking number would
     * otherwise leave its old digits behind. FONT_SMALL is font_6x8.
     */
    snprintf(pos, sizeof(pos), "%d/%d", g_file_sel + 1, last + 1);

    lcd_fill_rect(LCD_WIDTH - 72, 204, 72, 10, LCD_BLACK_COLOR);
    lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
    lcd_puts(LCD_WIDTH - 8 - (int)strlen(pos) * 6, 206, pos);
}

//-----------------------------------------------------------------------------
static void draw_picker(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);

    lcd_set_font(FONT_LARGE);
    lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
    lcd_puts(84, 12, "BK-0010-01");

    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
    lcd_puts(20, 36, rt->have_roms ? "monitor ROM found on the SPI flash"
                                 : "no monitor ROM - using the stand-in");

    lcd_puts(20, 206, "UP/DOWN choose, LEFT/RIGHT page");
    lcd_puts(20, 218, "MODE starts. Put .bin tape images on");
    lcd_puts(20, 230, "the chip with tools/spiflash.py add");

    draw_picker_list();
}

//-----------------------------------------------------------------------------
static void draw_fault(void)
{
    char buf[64];

    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);

    lcd_set_font(FONT_LARGE);
    lcd_set_color(LCD_BLACK_COLOR, HUD_BAD);
    lcd_puts(20, 20, "THE MACHINE STOPPED");

    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
    lcd_puts(20, 50, bk_cpu.stopped ? bk_cpu.stopped : bk_load_error());

    lcd_set_color(LCD_BLACK_COLOR, HUD_FG);

    snprintf(buf, sizeof(buf), "PC %06lo  PSW %06lo  SP %06lo",
        (unsigned long)bk_cpu.pc_insn, (unsigned long)bk_cpu.psw,
        (unsigned long)bk_cpu.r[6]);
    lcd_puts(20, 74, buf);

    snprintf(buf, sizeof(buf), "last trap through %06lo",
        (unsigned long)bk_cpu.last_trap);
    lcd_puts(20, 86, buf);

#if BK_DISASM
    {
        char text[40];

        bk_disasm(bk_cpu.pc_insn, text, sizeof(text));
        snprintf(buf, sizeof(buf), "%06lo: %s",
            (unsigned long)bk_cpu.pc_insn, text);
        lcd_puts(20, 104, buf);
    }
#endif

    lcd_puts(20, 130, "A program with no monitor behind it");
    lcd_puts(20, 142, "stops like this when it calls one. If");
    lcd_puts(20, 154, "that is what happened, put a real ROM");
    lcd_puts(20, 166, "image on the chip as bk10mon.rom.");

    lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
    lcd_puts(20, 190, "STOP restarts, SHIFT+MENU leaves.");
}

//-----------------------------------------------------------------------------
#if BK_DEBUGGER
// Registers and the next few instructions, over whatever is on screen. Reached
// with SAVE, and the only way to see what a program that has wandered off is
// actually doing.
static void draw_debug(void)
{
    char buf[52];
    char text[40];
    uint16_t pc = bk_cpu.pc_insn;
    int y = 30;

    lcd_fill_rect(20, 20, 280, 200, LCD_COLOR(8, 8, 16));
    lcd_draw_rect(20, 20, 280, 200, HUD_HI);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_COLOR(8, 8, 16), HUD_FG);

    for (int i = 0; i < 4; i++)
    {
        snprintf(buf, sizeof(buf), "R%d %06lo   R%d %06lo",
            i, (unsigned long)bk_cpu.r[i], i + 4,
            (unsigned long)bk_cpu.r[i + 4]);
        lcd_puts(30, y, buf);
        y += 11;
    }

    snprintf(buf, sizeof(buf), "PSW %06lo  %c%c%c%c%c",
        (unsigned long)bk_cpu.psw,
        (bk_cpu.psw & PSW_T) ? 'T' : '-',
        (bk_cpu.psw & PSW_N) ? 'N' : '-',
        (bk_cpu.psw & PSW_Z) ? 'Z' : '-',
        (bk_cpu.psw & PSW_V) ? 'V' : '-',
        (bk_cpu.psw & PSW_C) ? 'C' : '-');
    lcd_puts(30, y, buf);
    y += 16;

    for (int i = 0; i < 6; i++)
    {
        int len = bk_disasm(pc, text, sizeof(text));

        snprintf(buf, sizeof(buf), "%c%06lo: %s", (0 == i) ? '>' : ' ',
            (unsigned long)pc, text);
        lcd_set_color(LCD_COLOR(8, 8, 16), (0 == i) ? HUD_HI : HUD_FG);
        lcd_puts(30, y, buf);

        pc = (uint16_t)(pc + len);
        y += 11;
    }

    y += 6;
    lcd_set_color(LCD_COLOR(8, 8, 16), HUD_FG);

    snprintf(buf, sizeof(buf), "%lu insns  %lu cycles  %lu unknown I/O",
        (unsigned long)bk_cpu.insns, (unsigned long)bk_cpu.cycles,
        (unsigned long)bk_io_unknown);
    lcd_puts(30, y, buf);
    y += 14;

    lcd_set_color(LCD_COLOR(8, 8, 16), HUD_HI);
    lcd_puts(30, y, "MODE steps, SAVE returns");
}
#endif

//-----------------------------------------------------------------------------
static void start_machine(void)
{
    bk_mem_attach((uint8_t *)BK_RAM_ADDR);
    bk_video_set_buffer((uint8_t *)BK_BLIT_ADDR);

#if BK_LOAD_SPIFS
    // This rescans the part, which rebuilds the table the pack was found in,
    // so the pack is resolved again against the table as it now stands
    rt->have_roms = bk_load_roms();
    rt->pack_count = bk_load_pack_open();
#else
    rt->have_roms = false;
#endif

    bk_io_reset();

    if (g_file_sel < pick_total())
    {
        bk_bin_t bin;

#if BK_LOAD_SPIFS
        bool ok = (g_file_sel < rt->file_count)
            ? bk_load_bin(rt->files[g_file_sel], &bin)
            : bk_load_pack_run(g_file_sel - rt->file_count, &bin);

        if (!ok)
        {
            bk_cpu.stopped = bk_load_error();
            g_state = ST_FAULT;
            draw_fault();
            return;
        }
#else
        memset(&bin, 0, sizeof(bin));
#endif

        /*
         * With a real monitor the program is entered through it, so that its
         * screen and its stack are the ones the program was written against.
         * Without one the stand-in does that job and then jumps straight to
         * the load address, which is what the monitor's own start command
         * would have done.
         */
#if BK_STUB_ROM
        if (!rt->have_roms)
            bk_load_stub_rom(bin.start);
#endif
    }
#if BK_STUB_ROM
    else if (!rt->have_roms)
    {
        // Nothing to run: the stand-in clears the screen and stops, which at
        // least says that the processor and the panel both work
        bk_load_stub_rom(0);
    }
#endif

    bk_load_boot();

    rt->evnt_left = EVNT_PERIOD;
    rt->last_ms = timer_ms();
    rt->draw_ms = rt->last_ms;
    rt->key_code = 0;

#if BK_STATS
    rt->frames = 0;
    rt->fps_ms = rt->last_ms;
    rt->cycles_mark = 0;
    rt->statline[0] = 0;
#endif

    apply_video();

    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);

    g_state = ST_RUN;
}

//-----------------------------------------------------------------------------
void bk_port_init(void)
{
    // The address space lives in the capture ring, so acquisition stops first
    capture_stop();

    // Whatever the last application left here is not ours
    rt = (bk_run_t *)BK_RUN_ADDR;
    memset(rt, 0, sizeof(*rt));

#if BK_LOAD_SPIFS
    spifs_scan();
    rt->file_count = bk_load_list_bins(rt->files, MAX_FILES);
    rt->pack_count = bk_load_pack_open();
    rt->have_roms = (NULL != spifs_find(BK_ROM_MONITOR_FILE));
#else
    rt->file_count = 0;
    rt->pack_count = 0;
    rt->have_roms = false;
#endif

    g_file_sel = 0;
    g_file_top = 0;
    g_state = ST_PICK;

    draw_picker();
}

//-----------------------------------------------------------------------------
void bk_port_redraw(void)
{
    switch (g_state)
    {
    case ST_PICK:
        draw_picker();
        return;

    case ST_FAULT:
        draw_fault();
        return;

#if BK_DEBUGGER
    case ST_DEBUG:
        lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
        bk_video_draw(true);
        draw_debug();
        return;
#endif

    default:
        lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
        apply_video();
        bk_video_draw(true);
#if BK_STATS
        rt->statline[0] = 0;
        draw_status(true);
#endif
        return;
    }
}

//-----------------------------------------------------------------------------
void bk_port_task(void)
{
    uint32_t now;
    uint32_t elapsed;

    if (ST_RUN != g_state)
        return;

    now = timer_ms();
    elapsed = now - rt->last_ms;
    rt->last_ms = now;

    /*
     * After a stall - a menu open over the top of this, or the launcher
     * switching applications - the emulated machine does not try to catch up.
     * A game that lost half a second would replay it at ten times speed, which
     * looks exactly like a bug in the emulator and is worse than a pause.
     */
    if (elapsed > 100)
        elapsed = 100;

    poll_keys();
    run_slice(elapsed);

    if (bk_cpu.stopped)
    {
        g_state = ST_FAULT;
        draw_fault();
        return;
    }

    /*
     * Paint at the machine's own rate, not the loop's.
     *
     * The frame counter used to count passes of this function, which read
     * seventeen thousand and looked exactly like a machine running four
     * hundred times too fast - the emulated processor was at 3005 kHz the
     * whole time, a real BK to within a fifth of a per cent. Only the number
     * was wrong, and then only because the panel really was being repainted
     * that often.
     *
     * Now it is fifty a second, which is what a BK does, and the number means
     * something: fifty is keeping up, less is the panel unable to.
     */
    if (now - rt->draw_ms >= REPAINT_MS)
    {
        rt->draw_ms = now;

#if BK_STATS
        rt->rows = (uint32_t)bk_video_draw(false);
        rt->frames++;
#else
        bk_video_draw(false);
#endif
    }

#if BK_STATS

    if (now - rt->fps_ms >= 1000)
    {
        rt->fps = (uint16_t)(rt->frames * 1000 / (now - rt->fps_ms));
        rt->khz = (bk_cpu.cycles - rt->cycles_mark) / (now - rt->fps_ms);
        rt->cycles_mark = bk_cpu.cycles;
        rt->frames = 0;
        rt->fps_ms = now;
    }

    draw_status(false);
#else
    bk_video_draw(false);
#endif
}

//-----------------------------------------------------------------------------
void bk_port_buttons_handler(int buttons)
{
    bool edge = 0 == (buttons & BTN_REPEAT);

    switch (g_state)
    {
    case ST_PICK:
    {
        // A chip with a hundred programs on it is a list nobody wants to walk
        // a line at a time, so LEFT and RIGHT take it a screen at a time
        int step = 0;

        if (buttons & BTN_UP)
            step = -1;
        else if (buttons & BTN_DOWN)
            step = 1;
        else if (buttons & BTN_LEFT)
            step = -PICK_ROWS;
        else if (buttons & BTN_RIGHT)
            step = PICK_ROWS;
        else if ((buttons & BTN_MODE) && edge)
            start_machine();

        if (step)
        {
            int was = g_file_sel;

            g_file_sel += step;

            if (g_file_sel < 0)
                g_file_sel = 0;
            else if (g_file_sel > pick_total())
                g_file_sel = pick_total();

            if (g_file_sel != was)
                draw_picker_list();
        }

        return;
    }

    case ST_FAULT:
        if ((buttons & BTN_STOP) && edge)
            start_machine();
        return;

#if BK_DEBUGGER
    case ST_DEBUG:
        if ((buttons & BTN_MODE) && edge)
        {
            bk_cpu_step();
            bk_video_draw(false);
            draw_debug();
        }
        else if ((buttons & BTN_SAVE) && edge)
        {
            g_state = ST_RUN;
            rt->last_ms = timer_ms();
            bk_port_redraw();
        }
        return;
#endif

    default:
        // Walking is polled in the task, because the repeat events are far too
        // coarse to steer with. Only the keys that toggle something are edges.
        if ((buttons & BTN_STOP) && edge)
        {
            bk_io_stop_key();
        }
#if BK_DEBUGGER
        else if ((buttons & BTN_SAVE) && edge)
        {
            g_state = ST_DEBUG;
            draw_debug();
        }
#endif
        return;
    }
}

//-----------------------------------------------------------------------------
void bk_port_cleanup(void)
{
    g_state = ST_PICK;
    bk_mem_attach(NULL);
}

/*- Application menu --------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void on_video_change(void)
{
    apply_video();
}

//-----------------------------------------------------------------------------
static void on_number_change(int value)
{
    (void)value;
    apply_video();
}

//-----------------------------------------------------------------------------
static void action_reset(const void *arg)
{
    (void)arg;

    start_machine();
    menu_close_popups();
}

//-----------------------------------------------------------------------------
static void action_pick(const void *arg)
{
    (void)arg;

    g_state = ST_PICK;

#if BK_LOAD_SPIFS
    rt->file_count = bk_load_list_bins(rt->files, MAX_FILES);
    rt->pack_count = bk_load_pack_open();
#endif

    if (g_file_sel > pick_total())
        g_file_sel = pick_total();

    menu_close_popups();
    draw_picker();
}

//-----------------------------------------------------------------------------
static const char *const g_help_lines[] =
{
    INFO_HEAD "CONTROLS",
    "UP DOWN LEFT RIGHT - the BK's arrow keys",
    "MODE               - the BK's Enter key",
    "F1                 - space",
    "AUTO               - tab",
    "STOP               - the BK's STOP key",
    "SAVE               - registers and disassembly",
    "SHIFT+MENU         - back to the launcher",
    "",
    "The panel is read through two priority",
    "encoders, so UP and RIGHT cannot be held",
    "together and neither can DOWN and LEFT.",
    "No BK game needs a diagonal.",
    "",
    INFO_HEAD "WHAT IT NEEDS",
    "Nothing is built into the firmware: the",
    "monitor ROM is the machine's own firmware",
    "and not ours to ship. Put your own on the",
    "SPI flash and it is used:",
    "",
    "  bk10mon.rom   8 KB, goes to 0100000",
    "  bk10bas.rom   BASIC, goes to 0120000",
    "  anything.bin  a tape image, in the picker",
    "  bkgames.pak   many of them in one file",
    "",
    "  python3 tools/spiflash.py add game.bin",
    "",
    "spifs holds sixteen files, so a collection",
    "goes in a pack instead - its index is read",
    "off the chip and costs no RAM at all:",
    "",
    "  python3 tools/bkpack.py *.bin -o b.pak",
    "  python3 tools/spiflash.py add b.pak",
    "",
    "Without a monitor a stand-in runs instead:",
    "it sets the screen registers, points every",
    "vector at an RTI and jumps to the program.",
    "Enough for a self-contained game, and not",
    "a monitor - an EMT returns without doing",
    "anything at all.",
    "",
    INFO_HEAD "THE SCREEN",
    "256 lines do not fit in 240 rows. Squash",
    "drops one line in sixteen and shows all",
    "of them - it is the default because a BK",
    "game puts its border and its score where",
    "a television had overscan, which is the",
    "first thing Crop throws away. Crop keeps",
    "every line its own size and loses sixteen,",
    "split top and bottom by Top line.",
    "",
    "Mono is 512 dots wide and the panel is",
    "320. Whole 512 takes the dots in pairs,",
    "as the colour mode does, so both modes",
    "show the same area at the same size; a",
    "pair with one dot lit is drawn grey,",
    "which keeps small text readable. 1:1",
    "window is one dot to one pixel over 320",
    "of the 512, and Pan chooses which 320.",
    "",
    INFO_HEAD "SPEED",
    "The processor is emulated against real",
    "time at a nominal 3 MHz, which is what a",
    "БК-0010-01 ran at. The cycle counts are",
    "the shape of the 1801's timings and not",
    "measured silicon, so Speed trim is here",
    "to correct for it if a game runs wrong.",
    "",
    "Unlimited takes the clock off entirely -",
    "the machine gets whatever is left between",
    "two repaints of the panel.",
    "",
    "The status column reports kHz, which a",
    "real BK reads 3000 of, and vs BK, which",
    "is the same thing as a multiple. That is",
    "the number to look at with the clock off:",
    "a 250 MHz Cortex-M4 from 2014 against a",
    "3 MHz К1801ВМ1 from 1985, in one figure.",
};

static const info_page_t g_help_page =
{
    .title = "BK-0010-01",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

//-----------------------------------------------------------------------------
static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "Screen",
      .u.choice = { &g_screen_mode, g_screen_labels,
          ARRAY_SIZE(g_screen_labels), on_video_change } },
    { .kind = MI_CHOICE, .label = "256 lines into 240",
      .u.choice = { &g_fit_mode, g_fit_labels, ARRAY_SIZE(g_fit_labels),
          on_video_change } },
    { .kind = MI_NUMBER, .label = "Top line",
      .u.number = { &g_top_line, 0, 16, 1, 2, NULL, on_number_change } },
    { .kind = MI_CHOICE, .label = "512 dots into 320",
      .u.choice = { &g_mono_fit, g_mono_labels, ARRAY_SIZE(g_mono_labels),
          on_video_change } },
    { .kind = MI_NUMBER, .label = "Pan",
      .u.number = { &g_pan, 0, 192, 8, 32, " dots", on_number_change } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_CHOICE, .label = "Speed",
      .u.choice = { &g_speed_preset, g_speed_labels,
          ARRAY_SIZE(g_speed_labels), NULL } },
    { .kind = MI_NUMBER, .label = "Speed trim",
      .u.number = { &g_speed_trim, 5, 400, 5, 25, "%", NULL } },
    { .kind = MI_TOGGLE, .label = "Joystick from the arrows",
      .u.toggle = { &g_joystick, NULL } },
    { .kind = MI_TOGGLE, .label = "Status line",
      .u.toggle = { &g_show_status, on_video_change } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Load another program",
      .u.action = { action_pick, NULL } },
    { .kind = MI_ACTION, .label = "Reset",
      .u.action = { action_reset, NULL } },
};

const menu_def_t bk_port_menu =
{
    .title = "BK-0010-01",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};

//-----------------------------------------------------------------------------
static const menu_item_t g_help_items[] =
{
    { .kind = MI_ACTION, .label = "Controls and files",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t bk_port_help_menu =
{
    .title = "BK-0010-01",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
