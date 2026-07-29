/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host tests for the БК-0010-01 emulator.
 *
 * A processor is not a thing you can tell is right by reading it. Every one of
 * these runs actual PDP-11 code through the actual emulator and checks what
 * came out, because the failures this catches are all of the same shape - a
 * condition code set from the wrong operand, an autoincrement that stepped by
 * one where it should have stepped by two - and every one of them produces a
 * machine that runs thousands of instructions perfectly and then draws the
 * wrong thing on the screen.
 *
 * The programs are assembled here, by hand, out of the octal the machine is
 * documented in. That is deliberate too: an assembler written for the test
 * would share the encoding assumptions with the thing under test, and then
 * both could be wrong together.
 *
 * Build & run, from the repository root:
 *   make test
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "lcd.h"
#include "bk_config.h"
#include "bk_cpu.h"
#include "bk_mem.h"
#include "bk_io.h"
#include "bk_video.h"
#include "bk_load.h"
#include "bk_disasm.h"

/*- The panel, as far as bk_video.c can tell --------------------------------*/
/*
 * The real lcd.h, and stubs for the six calls behind it. bk_video.c is built
 * unmodified against the same header the firmware uses, so the geometry and
 * the colour packing under test are the ones that ship.
 */
#define PANEL_W     320
#define PANEL_H     240

static uint16_t g_panel[PANEL_H][PANEL_W];
static int g_offscreen;

void lcd_set_font(const Font *font) { (void)font; }
void lcd_set_color(int bg, int fg) { (void)bg; (void)fg; }
void lcd_puts(int x, int y, const char *s) { (void)x; (void)y; (void)s; }
void lcd_draw_rect(int x, int y, int w, int h, int c)
{
    (void)x; (void)y; (void)w; (void)h; (void)c;
}

void lcd_fill_rect(int x, int y, int w, int h, int color)
{
    for (int j = 0; j < h; j++)
    {
        for (int i = 0; i < w; i++)
        {
            if (x + i < 0 || x + i >= PANEL_W || y + j < 0 || y + j >= PANEL_H)
            {
                g_offscreen++;
                continue;
            }

            g_panel[y + j][x + i] = (uint16_t)color;
        }
    }
}

void lcd_draw_indexed(int x, int y, int w, int h, const uint8_t *pix,
    const uint16_t *palette)
{
    for (int j = 0; j < h; j++)
    {
        for (int i = 0; i < w; i++)
        {
            if (x + i < 0 || x + i >= PANEL_W || y + j < 0 || y + j >= PANEL_H)
            {
                g_offscreen++;
                continue;
            }

            g_panel[y + j][x + i] = palette[pix[j * w + i]];
        }
    }
}

/*- Test plumbing -----------------------------------------------------------*/
static int g_failures;
static uint8_t g_ram[BK_MEM_SIZE];

static void check(const char *name, long got, long want)
{
    if (got == want)
    {
        printf("  PASS %-46s %ld\n", name, got);
    }
    else
    {
        printf("  FAIL %-46s got %ld  want %ld\n", name, got, want);
        g_failures++;
    }
}

static void check_oct(const char *name, unsigned got, unsigned want)
{
    if (got == want)
    {
        printf("  PASS %-46s %06o\n", name, got);
    }
    else
    {
        printf("  FAIL %-46s got %06o  want %06o\n", name, got, want);
        g_failures++;
    }
}

static void check_true(const char *name, bool ok)
{
    check(name, ok ? 1 : 0, 1);
}

/*- A hand assembler --------------------------------------------------------*/
/*
 * The six-bit operand field, spelled the way the manuals spell it: the top
 * three bits are the mode, the bottom three the register.
 */
#define REG(r)      (unsigned)(r)               // Rn
#define IND(r)      (unsigned)(010 | (r))       // (Rn)
#define PI(r)       (unsigned)(020 | (r))       // (Rn)+
#define PD(r)       (unsigned)(040 | (r))       // -(Rn)
#define IDX(r)      (unsigned)(060 | (r))       // x(Rn)
#define PID(r)      (unsigned)(030 | (r))       // @(Rn)+
#define IMM         027u                        // #x, which is (PC)+
#define ABS         037u                        // @#x

#define SP_         6
#define PC_         7

static uint16_t g_at;

static void org(uint16_t addr)
{
    g_at = addr;
}

static void w(uint16_t word)
{
    g_ram[g_at] = (uint8_t)word;
    g_ram[g_at + 1] = (uint8_t)(word >> 8);
    g_at = (uint16_t)(g_at + 2);
}

static void op2(uint16_t base, unsigned src, unsigned dst)
{
    w((uint16_t)(base | (src << 6) | dst));
}

static void op1(uint16_t base, unsigned dst)
{
    w((uint16_t)(base | dst));
}

// MOV #value, Rn - three words often enough to be worth naming
static void movi(uint16_t value, unsigned reg)
{
    op2(0010000u, IMM, REG(reg));
    w(value);
}

/*- Running -----------------------------------------------------------------*/
static void reset_machine(void)
{
    bk_mem_attach(g_ram);
    bk_io_reset();
    bk_cpu_reset(01000, 0340);
    memset(g_panel, 0, sizeof(g_panel));
    g_offscreen = 0;
}

// Run until HALT or the budget runs out. Returns the instructions executed.
static uint32_t run(uint32_t budget)
{
    uint32_t before = bk_cpu.insns;

    bk_cpu_run(budget);

    return bk_cpu.insns - before;
}

// The commonest shape: assemble at 01000, run, stop at the HALT
static void go(void)
{
    w(0000000u);            // HALT
    bk_cpu_reset(01000, 0340);
    run(100000);
}

#define FLAGS() (bk_cpu.psw & 017u)

/*- Moving and arithmetic ---------------------------------------------------*/
static void t_arithmetic(void)
{
    printf("moving and arithmetic:\n");

    reset_machine();
    org(01000);
    movi(0123456u, 0);
    go();
    check_oct("MOV puts the whole word in", bk_cpu.r[0], 0123456u);
    check("and sets N from its top bit", 0 != (bk_cpu.psw & PSW_N), 1);
    check("and leaves V alone", 0 != (bk_cpu.psw & PSW_V), 0);

    reset_machine();
    org(01000);
    movi(077777u, 0);
    movi(1, 1);
    op2(0060000u, REG(1), REG(0));      // ADD R1, R0
    go();
    check_oct("the largest positive plus one", bk_cpu.r[0], 0100000u);
    check("overflows", 0 != (bk_cpu.psw & PSW_V), 1);
    check("without carrying", 0 != (bk_cpu.psw & PSW_C), 0);

    reset_machine();
    org(01000);
    movi(0177777u, 0);
    movi(1, 1);
    op2(0060000u, REG(1), REG(0));      // ADD R1, R0
    go();
    check_oct("minus one plus one is zero", bk_cpu.r[0], 0);
    check("carrying", 0 != (bk_cpu.psw & PSW_C), 1);
    check("and setting Z", 0 != (bk_cpu.psw & PSW_Z), 1);

    reset_machine();
    org(01000);
    movi(1, 0);
    movi(2, 1);
    op2(0160000u, REG(1), REG(0));      // SUB R1, R0
    go();
    check_oct("one take two is minus one", bk_cpu.r[0], 0177777u);
    check("and borrows", 0 != (bk_cpu.psw & PSW_C), 1);

    // CMP is source minus destination, which is the way round that catches
    // people out: CMP R0, R1 with R0 < R1 sets carry
    reset_machine();
    org(01000);
    movi(1, 0);
    movi(2, 1);
    op2(0020000u, REG(0), REG(1));      // CMP R0, R1
    go();
    check("CMP borrows when source is smaller", 0 != (bk_cpu.psw & PSW_C), 1);
    check("and does not disturb either operand", bk_cpu.r[1], 2);

    reset_machine();
    org(01000);
    movi(0177777u, 0);
    op1(0005400u, REG(0));              // NEG R0
    go();
    check_oct("NEG of minus one is one", bk_cpu.r[0], 1);
    check("and carries because it is not zero", 0 != (bk_cpu.psw & PSW_C), 1);

    reset_machine();
    org(01000);
    movi(0000402u, 0);
    op1(0000300u, REG(0));              // SWAB R0
    go();
    check_oct("SWAB exchanges the halves", bk_cpu.r[0], 0001001u);
    check("and reports on the new low byte", 0 != (bk_cpu.psw & PSW_Z), 0);
}

/*- Bytes -------------------------------------------------------------------*/
static void t_bytes(void)
{
    printf("bytes:\n");

    reset_machine();
    org(01000);
    movi(0, 0);
    op2(0110000u, IMM, REG(0));         // MOVB #377, R0
    w(0377u);
    go();
    check_oct("MOVB into a register sign extends", bk_cpu.r[0], 0177777u);

    reset_machine();
    org(01000);
    movi(0177400u, 0);
    op2(0150000u, IMM, REG(0));         // BISB #17, R0
    w(0017u);
    go();
    check_oct("but BISB leaves the high half alone", bk_cpu.r[0], 0177417u);

    /*
     * The stepping rule, which is the one that quietly ruins a program: a byte
     * autoincrement moves by one, except through R6 and R7, because what those
     * two point at is always a word.
     */
    reset_machine();
    org(01000);
    movi(02000u, 1);
    op2(0110000u, PI(1), REG(0));       // MOVB (R1)+, R0
    go();
    check_oct("a byte autoincrement steps by one", bk_cpu.r[1], 02001u);

    reset_machine();
    org(01000);
    movi(02000u, 1);
    op2(0010000u, PI(1), REG(0));       // MOV (R1)+, R0
    go();
    check_oct("a word autoincrement steps by two", bk_cpu.r[1], 02002u);

    reset_machine();
    org(01000);
    movi(02000u, SP_);
    op2(0110000u, PD(SP_), REG(0));     // MOVB -(SP), R0
    go();
    check_oct("but the stack pointer always by two", bk_cpu.r[6], 01776u);

    // A byte written to memory must not disturb its neighbour
    reset_machine();
    org(01000);
    movi(0177777u, 0);
    movi(02000u, 1);
    op2(0010000u, REG(0), IND(1));      // MOV R0, (R1)
    op2(0110000u, IMM, IND(1));         // MOVB #0, (R1)
    w(0);
    go();
    check_oct("a byte store leaves the other half", bk_read16(02000u), 0177400u);
}

/*- Addressing --------------------------------------------------------------*/
static void t_addressing(void)
{
    printf("addressing:\n");

    reset_machine();
    org(02000);
    w(0123456u);
    org(01000);
    op2(0010000u, ABS, REG(0));         // MOV @#2000, R0
    w(02000u);
    go();
    check_oct("absolute reaches a fixed address", bk_cpu.r[0], 0123456u);

    reset_machine();
    org(02010);
    w(0007070u);
    org(01000);
    movi(02000u, 1);
    op2(0010000u, IDX(1), REG(0));      // MOV 10(R1), R0
    w(010u);
    go();
    check_oct("indexed adds the offset to the register", bk_cpu.r[0], 07070u);

    // Index off the program counter is how every assembler writes a label, so
    // it is the one that has to be right about where the counter is by then
    reset_machine();
    org(01000);
    op2(0010000u, IDX(PC_), REG(0));    // MOV label(PC), R0
    w(4);                               // PC is 01006 here, so 01012
    w(0000000u);                        // HALT
    w(0000000u);
    w(0135713u);                        // the label
    bk_cpu_reset(01000, 0340);
    run(1000);
    check_oct("and off the counter it lands after the word",
        bk_cpu.r[0], 0135713u);

    reset_machine();
    org(02000);
    w(03000u);
    org(03000);
    w(0111111u);
    org(01000);
    movi(02000u, 1);
    op2(0010000u, PID(1), REG(0));      // MOV @(R1)+, R0
    go();
    check_oct("deferred autoincrement goes through", bk_cpu.r[0], 0111111u);
    check_oct("and still steps by two", bk_cpu.r[1], 02002u);
}

/*- Branches, calls and loops -----------------------------------------------*/
static void t_flow(void)
{
    printf("branches, calls and loops:\n");

    // Sum 1..10 with SOB, which is this machine's whole idea of a loop
    reset_machine();
    org(01000);
    movi(0, 0);                         // total
    movi(10, 1);                        // counter
    movi(0, 2);                         // running value
    // loop:
    {
        uint16_t loop = g_at;

        op1(0005200u, REG(2));          // INC R2
        op2(0060000u, REG(2), REG(0));  // ADD R2, R0
        w((uint16_t)(0077100u | ((g_at + 2 - loop) / 2)));  // SOB R1, loop
    }
    go();
    check("SOB counts a loop out", bk_cpu.r[0], 55);
    check("and leaves the register at zero", bk_cpu.r[1], 0);

    // A subroutine, called and returned from
    reset_machine();
    org(01000);
    movi(01000u, SP_);
    movi(7, 0);
    op2(0004000u, REG(5), ABS);         // JSR R5, @#2000
    w(02000u);
    w(0000000u);                        // HALT
    org(02000);
    op2(0060000u, IMM, REG(0));         // ADD #3, R0
    w(3);
    op1(0000200u, 5);                   // RTS R5
    bk_cpu_reset(01000, 0340);
    run(1000);
    check("JSR and RTS get there and back", bk_cpu.r[0], 10);
    check_oct("with the stack put back", bk_cpu.r[6], 01000u);

    // Every branch, taken and not
    reset_machine();
    org(01000);
    movi(0, 0);
    op1(0005700u, REG(0));              // TST R0 - sets Z
    w(0001404u);                        // BEQ over the next four words
    movi(0111u, 1);                     // skipped
    movi(0222u, 2);                     // skipped
    movi(0333u, 3);
    go();
    check_oct("BEQ jumps over what it should", bk_cpu.r[1], 0);
    check_oct("and over all of it", bk_cpu.r[2], 0);
    check_oct("and lands where it should", bk_cpu.r[3], 0333u);

    reset_machine();
    org(01000);
    movi(0177777u, 0);                  // -1
    movi(1, 1);
    op2(0020000u, REG(0), REG(1));      // CMP R0, R1: -1 against 1
    w(0002402u);                        // BLT over the next two words
    movi(0666u, 2);                     // skipped when the branch is taken
    movi(0777u, 3);
    go();
    check_oct("BLT knows minus one is less than one", bk_cpu.r[2], 0);
    check_oct("and carries on afterwards", bk_cpu.r[3], 0777u);
}

/*- Traps and interrupts ----------------------------------------------------*/
static void t_traps(void)
{
    printf("traps and interrupts:\n");

    // EMT is how a BK program calls the monitor, so it is the trap that
    // matters most. The handler here counts itself and returns.
    reset_machine();
    org(0);
    org(030);
    w(03000u);                          // the EMT vector
    w(0000u);                           // at priority 0
    org(03000);
    op1(0005200u, REG(1));              // INC R1
    w(0000002u);                        // RTI
    org(01000);
    movi(01000u, SP_);
    movi(0, 1);
    w(0104007u);                        // EMT 7
    movi(0555u, 2);
    go();
    check("EMT reaches its handler", bk_cpu.r[1], 1);
    check_oct("and RTI comes back to the next instruction",
        bk_cpu.r[2], 0555u);

    // An instruction this processor does not have
    reset_machine();
    org(010);
    w(03000u);
    w(0);
    org(03000);
    op1(0005200u, REG(1));              // INC R1
    w(0000000u);                        // HALT
    org(01000);
    movi(01000u, SP_);
    movi(0, 1);
    w(0070001u);                        // MUL R0, R1 - not on a 1801VM1
    bk_cpu_reset(01000, 0340);
    run(1000);
    check("MUL traps as a reserved instruction", bk_cpu.r[1], 1);
    check_oct("through vector 10", bk_cpu.last_trap, 010u);

    // A word read from an odd address
    reset_machine();
    org(04);
    w(03000u);
    w(0);
    org(03000);
    op1(0005200u, REG(1));              // INC R1
    w(0000000u);                        // HALT
    org(01000);
    movi(01000u, SP_);
    movi(0, 1);
    movi(0001001u, 0);                  // an odd address
    op2(0010000u, IND(0), REG(2));      // MOV (R0), R2
    bk_cpu_reset(01000, 0340);
    run(1000);
    check("an odd word address is a bus error", bk_cpu.r[1], 1);
    check_oct("through vector 4", bk_cpu.last_trap, 04u);

    /*
     * An interrupt raised while the processor is at priority waits, and
     * arrives when it comes down. This is what makes a key pressed inside a
     * critical section arrive late rather than not at all.
     */
    reset_machine();
    org(060);
    w(03000u);
    w(0);
    org(03000);
    op1(0005200u, REG(1));              // INC R1
    w(0000002u);                        // RTI
    org(01000);
    movi(01000u, SP_);
    movi(0, 1);
    movi(0, 2);
    op1(0005200u, REG(2));              // INC R2, still at priority 7
    w(0106427u);                        // MTPS #0
    w(0);
    movi(0444u, 3);
    bk_cpu_reset(01000, 0340);
    bk_cpu_virq(060);
    run(1000);
    check("an interrupt at priority is held", bk_cpu.r[2], 1);
    check("and delivered when priority drops", bk_cpu.r[1], 1);
    check_oct("and the program carries on", bk_cpu.r[3], 0444u);
}

/*- The registers at the top of memory --------------------------------------*/
static void t_io(void)
{
    printf("the registers:\n");

    reset_machine();
    org(01000);
    movi(0001330u, 0);
    op2(0010000u, REG(0), ABS);         // MOV R0, @#177664
    w(0177664u);
    go();
    check_oct("a program can set the scroll register",
        bk_io_scroll(), 0001330u);

    // The keyboard: a code arrives, the ready flag comes up, reading the code
    // takes it down again
    reset_machine();
    bk_io_key(BK_KEY_RIGHT, false);
    check("a key raises the ready flag",
        0 != (bk_io_reg(BK_REG_KBD_STATUS) & BK_KBD_READY), 1);

    org(01000);
    op2(0010000u, ABS, REG(0));         // MOV @#177660, R0 - the status
    w(0177660u);
    op2(0010000u, ABS, REG(1));         // MOV @#177662, R1 - the code
    w(0177662u);
    op2(0010000u, ABS, REG(2));         // and the status again
    w(0177660u);
    go();
    check_oct("the program reads the code it was sent",
        bk_cpu.r[1], BK_KEY_RIGHT);
    check("the flag was up before the read",
        0 != (bk_cpu.r[0] & BK_KBD_READY), 1);
    check("and down after it", 0 != (bk_cpu.r[2] & BK_KBD_READY), 0);

    // 0177716 bit 6 clear means a key is down, which is the other idiom
    reset_machine();
    bk_io_key(BK_KEY_SPACE, false);
    check("a key down shows in the system register",
        0 != (bk_io_reg(BK_REG_SYSTEM) & 0100u), 0);
    bk_io_key_release();
    check("and lets go when it is released",
        0 != (bk_io_reg(BK_REG_SYSTEM) & 0100u), 1);

    // Writing above 0100000 does nothing, which is what makes it a ROM
    reset_machine();
    g_ram[0100000] = 0252;
    org(01000);
    movi(0, 0);
    op2(0010000u, REG(0), ABS);         // MOV R0, @#100000
    w(0100000u);
    go();
    check_oct("a write into the ROM window is ignored",
        g_ram[0100000], 0252u);
}

/*- The screen --------------------------------------------------------------*/
static void t_video(void)
{
    static uint8_t buf[BK_VIDEO_BUFFER_SIZE];
    const uint16_t *pal;

    printf("the screen:\n");

    reset_machine();
    bk_video_set_buffer(buf);
    bk_video_init();
    bk_video_set_screen(BK_SCREEN_COLOR);
    bk_video_set_fit(BK_FIT_CROP);
    bk_video_set_top(0);
    bk_video_set_status(false);

    // The four colours of one byte, lowest bits leftmost.
    //
    // The panel is filled with something that is not a BK colour first, so
    // that "the first dot is black" is a dot that was painted black and not a
    // dot nobody painted at all.
    memset(g_panel, 0xab, sizeof(g_panel));
    bk_write16(BK_VRAM_BASE, 0000344u);     // low byte 0344 = 11 10 01 00
    bk_video_draw(true);

    pal = bk_video_palette();
    check_oct("the leftmost dot is the lowest two bits",
        g_panel[0][32], pal[0]);
    check_oct("then the next pair", g_panel[0][33], pal[1]);
    check_oct("then the next", g_panel[0][34], pal[2]);
    check_oct("and the top pair last", g_panel[0][35], pal[3]);
    check("palette zero starts black", pal[0], 0);
    check_oct("and its third colour is red", pal[3], 0xf800u);
    check("nothing was drawn off the panel", g_offscreen, 0);

    /*
     * The scroll register holds the first line to show, counted from 0330.
     * With 0331 the picture moves up by one, so what was on line 1 is now on
     * the top row.
     */
    reset_machine();
    bk_video_set_buffer(buf);
    bk_video_init();
    bk_video_set_top(0);
    memset(g_panel, 0, sizeof(g_panel));
    bk_write16(BK_VRAM_BASE + BK_VRAM_STRIDE, 0000003u);    // line 1, dot 0
    bk_io_write(BK_REG_SCROLL, 01331u);
    bk_video_draw(true);
    check_oct("scrolling by one lifts line one to the top",
        g_panel[0][32], bk_video_palette()[3]);

    // Only what changed is repainted
    reset_machine();
    bk_video_set_buffer(buf);
    bk_video_init();
    bk_video_set_top(0);
    check("a fresh screen needs every row", bk_video_draw(false), 240);
    check("and nothing at all the next time", bk_video_draw(false), 0);
    bk_write16(BK_VRAM_BASE + 10 * BK_VRAM_STRIDE, 0177777u);
    check("one line written is one row painted", bk_video_draw(false), 1);
}

/*- The stand-in ROM --------------------------------------------------------*/
static void t_stub_rom(void)
{
    printf("the stand-in ROM:\n");

    bk_mem_attach(g_ram);
    bk_io_reset();

    // A program that marks the screen and then sits still
    org(01000);
    op2(0010000u, IMM, ABS);            // MOV #125252, @#40000
    w(0125252u);
    w(BK_VRAM_BASE);
    w(0000777u);                        // BR . - go no further

    bk_load_stub_rom(01000);
    bk_load_boot();

    check_oct("the processor starts where the ВМ1 starts",
        bk_cpu.r[7], BK_MON_BASE);

    // Long enough for the 8192 word screen clear and then some
    run(400000);

    check_true("it is still running", NULL == bk_cpu.stopped);
    check_oct("the stack is where the monitor leaves it",
        bk_cpu.r[6], 01000u);
    check_oct("the scroll register is set up", bk_io_scroll(), 01330u);
    check_oct("the EMT vector points somewhere", bk_read16(030u),
        (unsigned)(BK_MON_BASE + BK_MON_SIZE - 2));
    check_oct("and the program is running", bk_read16(BK_VRAM_BASE), 0125252u);
    check_true("in its own loop", bk_cpu.r[7] >= 01000 && bk_cpu.r[7] < 01020);

    // The screen was cleared on the way past: the word after the one the
    // program wrote is still zero, and it was not zero to begin with
    check_oct("and the screen was cleared first",
        bk_read16(BK_VRAM_BASE + 2), 0);

    // An EMT with nothing behind it returns instead of ending the world
    check_oct("nothing trapped", bk_cpu.last_trap, 0);
}

/*- Reading it back ---------------------------------------------------------*/
static void check_text(const char *name, const char *got, const char *want)
{
    if (0 == strcmp(got, want))
    {
        printf("  PASS %-46s %s\n", name, got);
    }
    else
    {
        printf("  FAIL %-46s got \"%s\"  want \"%s\"\n", name, got, want);
        g_failures++;
    }
}

//-----------------------------------------------------------------------------
// One instruction assembled, then read back.
//
// The listing is what the fault screen shows and what the debug overlay is,
// so a disassembler that is confidently wrong is worse than none: it does not
// look broken, it looks like a different program. The first version of this
// one read every extension word two bytes late, which turned MOV #1000,SP into
// MOV #104260,SP and an EMT that was not in the program at all - and nothing
// noticed, because nothing checked.
static void t_disasm(void)
{
    static const struct
    {
        const char *text;
        int len;
        uint16_t words[3];
    } cases[] =
    {
        { "MOV  #001000, SP",     4, { 0012706u, 0001000u } },
        { "MOV  R2, (R0)+",       2, { 0010220u } },
        { "MOVB #000377, R0",     4, { 0112700u, 0000377u } },
        { "MOV  @#177664, R0",    4, { 0013700u, 0177664u } },
        { "MOV  R5, @#177662",    4, { 0010537u, 0177662u } },
        { "ADD  #000330, R5",     4, { 0062705u, 0000330u } },
        { "MOV  000010(R1), R0",  4, { 0016100u, 0000010u } },
        { "CLR  (R0)+",           2, { 0005020u } },
        { "SOB  R1, 001000",      2, { 0077101u } },
        { "BEQ  001006",          2, { 0001402u } },
        { "JSR  PC, @#002000",    4, { 0004737u, 0002000u } },
        { "RTS  PC",              2, { 0000207u } },
        { "EMT  007",             2, { 0104007u } },
        { "MTPS #000000",         4, { 0106427u, 0000000u } },
        { "HALT",                 2, { 0000000u } },
        { "NOP",                  2, { 0000240u } },
        { "SEC",                  2, { 0000261u } },
        { "XOR  R1, R2",          2, { 0074102u } },
        { "MOV  001004, R0",      4, { 0016700u, 0000000u } },
    };

    printf("reading it back:\n");

    reset_machine();

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        char text[48];
        int len;

        org(01000);

        for (int k = 0; k < cases[i].len / 2; k++)
            w(cases[i].words[k]);

        len = bk_disasm(01000, text, sizeof(text));

        check_text(cases[i].text, text, cases[i].text);

        if (len != cases[i].len)
            check("  ...and its length", len, cases[i].len);
    }
}

/*- A program that does something -------------------------------------------*/
static void t_program(void)
{
    printf("a whole program:\n");

    /*
     * The machine has no MUL, so this is how every BK program multiplies: shift
     * and add, seventeen instructions where a PDP-11/23 has one. Running it is
     * a test of nearly everything at once - the shifts, the carry out of ASL,
     * the branch on it, and SOB holding the whole thing together.
     *
     * 0263 * 0155 = 043637 octal.
     */
    reset_machine();
    org(01000);
    movi(0263u, 0);                     // multiplicand, 179
    movi(0155u, 1);                     // multiplier, 109
    movi(0, 2);                         // product
    movi(16, 3);                        // bits to go
    {
        uint16_t loop = g_at;

        op1(0006000u, REG(1));          // ROR R1 - low bit falls into carry
        w(0103001u);                    // BCC over the add
        op2(0060000u, REG(0), REG(2));  // ADD R0, R2
        op1(0006300u, REG(0));          // ASL R0
        w((uint16_t)(0077300u | ((g_at + 2 - loop) / 2)));  // SOB R3, loop
    }
    go();

    check_oct("shift-and-add multiply", bk_cpu.r[2], 046067u);
    check_true("in a sensible number of instructions",
        bk_cpu.insns > 60 && bk_cpu.insns < 200);
}

/*- Main --------------------------------------------------------------------*/
int main(void)
{
    printf("БК-0010-01 emulator\n\n");

    t_arithmetic();
    printf("\n");

    t_bytes();
    printf("\n");

    t_addressing();
    printf("\n");

    t_flow();
    printf("\n");

    t_traps();
    printf("\n");

    t_io();
    printf("\n");

    t_video();
    printf("\n");

    t_stub_rom();
    printf("\n");

    t_program();
    printf("\n");

    t_disasm();

    printf("\n%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED",
        g_failures);

    return g_failures ? 1 : 0;
}
