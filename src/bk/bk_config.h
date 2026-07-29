/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * What gets built into the BK-0010-01 emulator.
 *
 * The instrument's image is the reason this file exists. The firmware is
 * already 89% of the 512 KB the part has, and an emulator is the kind of thing
 * that grows without anyone deciding it should: a disassembler here, a
 * floating point unit there, a tape deck nobody will ever plug a cable into.
 *
 * So every part of the machine that is not the machine is a switch, and the
 * switches are all here rather than scattered through the sources. Turn one
 * off and the code behind it is not compiled at all - not compiled and then
 * discarded by --gc-sections, which is what a runtime flag would get you.
 *
 * The defaults describe a real BK-0010-01 as closely as this is going to get:
 * what the К1801ВМ1 actually implements is on, what it does not is off, and
 * the extras that only exist for a person looking at the screen (the
 * disassembler, the register view) are on because there is currently room.
 *
 * To build a smaller one, define the switches on the command line:
 *
 *   make BK_CFLAGS="-DBK_DISASM=0 -DBK_DEBUGGER=0 -DBK_VIDEO_PALETTES=0"
 *
 * Every switch is wrapped in #ifndef, so a -D on the command line wins and the
 * default here fills in otherwise. Two consequences worth stating:
 *
 *   - a switch is ALWAYS defined by the time anything uses it, so test it with
 *     #if and never with #ifdef. A misspelt name is then a compile error where
 *     it is read, instead of a feature that quietly turns itself off.
 *   - a misspelt name on the COMMAND line is not caught by anything. Check the
 *     size afterwards: a switch that did nothing shows up as an image that did
 *     not shrink.
 *
 * What each is worth, measured rather than guessed - the difference in the
 * flashed image, everything else held still:
 *
 *   BK_DEBUGGER=0 BK_DISASM=0        -2800    the overlay and the listing
 *   BK_VIDEO_DIRTY=0                 -1984    and the frame rate with it
 *   BK_CPU_TBIT=0 BK_CPU_BUS_ERROR=0 -1856    no tracing, no odd-address trap
 *   BK_IO_TIMER=0                     -528
 *   BK_STUB_ROM=0                     -384    then a real ROM is required
 *   BK_STATS=0                        -368
 *   BK_VIDEO_PALETTES=0               -192    palette zero only
 *   BK_VIDEO_MONO=0                   -128
 *   BK_IO_JOYSTICK=0                  -128
 *   BK_CPU_EIS=1                      +704    a machine this never was
 *
 * The whole emulator is 21,868 bytes of code and 164 of .bss; all of the above
 * together take it to 17,564. The 64 KB of emulated machine is not in either
 * number - it lives in the SRAM the capture ring is not using.
 */

#ifndef _BK_CONFIG_H_
#define _BK_CONFIG_H_

/*- The machine -------------------------------------------------------------*/
/*
 * BK-0010-01: 32 KB of RAM, 32 KB of ROM, no memory management, one screen.
 *
 * The BK-0011M is the same processor behind a bank switcher - eight 16 KB
 * pages of RAM through two windows, two screen buffers, a second palette
 * register. Nothing here assumes there is only one bank, but nothing
 * implements the switching either; the hooks are marked with this switch so
 * that adding it later is an edit in three files rather than a search.
 */
#ifndef BK_MODEL_0010_01
#define BK_MODEL_0010_01        1
#endif
#ifndef BK_MODEL_0011M
#define BK_MODEL_0011M          0
#endif

/*- Processor ---------------------------------------------------------------*/
/*
 * EIS - MUL, DIV, ASH, ASHC.
 *
 * The К1801ВМ1 does not have them. This is not a simplification on our part:
 * BK software multiplies in a loop because it had to, and a program that
 * executes 070000 on real hardware takes a reserved-instruction trap through
 * vector 010. Turning this on makes the emulator faster than the machine it
 * emulates in a way that no BK program can ask for, so it is off - it exists
 * because the same core would be a fine PDP-11/23 with it on.
 *
 * FIS - the 075 floating point group - has no switch at all. It is not here,
 * it traps as a reserved instruction, and there is nothing to turn on.
 */
#ifndef BK_CPU_EIS
#define BK_CPU_EIS              0
#endif

/*
 * MFPI/MTPI/MFPD/MTPD - the 0065xx/0066xx/1065xx/1066xx group.
 *
 * These move data between address spaces, and the BK has one address space and
 * no memory management to give it another. On real hardware they trap. Turning
 * this on implements them as the PDP-11/34 does with the MMU disabled, which
 * is a push and a pop through the current stack pointer.
 */
#ifndef BK_CPU_MFPX
#define BK_CPU_MFPX             0
#endif

// SPL, 00023N. A supervisor-mode instruction on machines that have modes.
#ifndef BK_CPU_SPL
#define BK_CPU_SPL              0
#endif

/*
 * The T bit: after an instruction executed with PSW bit 4 set, trap through
 * vector 014. This is how every PDP-11 debugger single-steps, the BK monitor
 * included, so it stays on even though nothing in a game will use it. It costs
 * one test per instruction.
 */
#ifndef BK_CPU_TBIT
#define BK_CPU_TBIT             1
#endif

/*
 * Traps taken from instructions: BPT, IOT, EMT, TRAP, and the
 * reserved-instruction trap that catches everything not decoded.
 *
 * EMT is how BK programs call the monitor. Without this a program that prints
 * a character wanders off into whatever 000030 happens to contain, so this is
 * only really a switch for a build running one known self-contained binary.
 */
#ifndef BK_CPU_TRAPS
#define BK_CPU_TRAPS            1
#endif

/*
 * Bus errors: trap 4 on a word access to an odd address.
 *
 * Cheap and worth having - it turns a program that has lost its stack into a
 * reported fault at a known PC instead of quiet nonsense on the screen.
 */
#ifndef BK_CPU_BUS_ERROR
#define BK_CPU_BUS_ERROR        1
#endif

/*
 * Cycle accounting.
 *
 * Without it the emulator runs the processor as fast as the Cortex can, which
 * for a game written against a 3 MHz machine means unplayable. The counts are
 * the shape of the 1801 timings - a base cost plus the cost of reaching each
 * operand - not measured silicon, and the application has a speed control
 * because of that. Off only makes sense for a test harness that wants
 * instructions rather than time.
 */
#ifndef BK_CPU_CYCLES
#define BK_CPU_CYCLES           1
#endif

// Nominal processor clock. The real BK-0010-01 runs its ВМ1 at 3 MHz.
#ifndef BK_CPU_HZ
#define BK_CPU_HZ               3000000
#endif

/*- Memory ------------------------------------------------------------------*/
/*
 * Write protection above 0100000.
 *
 * A BK with no ROM cartridge fitted still answers there; what it does not do
 * is remember. Off, the region is plain RAM, which is occasionally what you
 * want when loading a binary that was linked to live in the ROM window.
 */
#ifndef BK_MEM_ROM_READONLY
#define BK_MEM_ROM_READONLY     1
#endif

/*- Screen ------------------------------------------------------------------*/
/*
 * The two ways of reading the same 16 KB.
 *
 * The video hardware has no mode bit. It always sends 512 dots a line and
 * pairs of bits always drive the colour outputs; "64 symbols" is the monitor
 * drawing narrow characters, and users switched their television to black and
 * white to stop the fringes. So this is a decision about the viewer, not about
 * the machine, and both belong in the same build unless space says otherwise.
 */
#ifndef BK_VIDEO_COLOR
#define BK_VIDEO_COLOR          1
#endif
#ifndef BK_VIDEO_MONO
#define BK_VIDEO_MONO           1
#endif

/*
 * The sixteen palettes.
 *
 * A BK-0010 has one: black, blue, green, red. The palette register at 0177662
 * is a BK-0011 feature, and software written for it is on the chip alongside
 * software that is not, so the table is here and the register is decoded. It
 * costs 128 bytes of .rodata and nothing at runtime.
 */
#ifndef BK_VIDEO_PALETTES
#define BK_VIDEO_PALETTES       1
#endif

/*
 * The scroll register, 0177664. Games use it - it is the only way the machine
 * moves a whole screen without moving 16 KB - so this is on. Off pins the
 * display to line zero.
 */
#ifndef BK_VIDEO_SCROLL
#define BK_VIDEO_SCROLL         1
#endif

/*
 * Repaint only the lines the program wrote to.
 *
 * The panel is bit-banged at about 200 ns a pixel, so a full 256x240 frame is
 * 12 ms whatever the emulator does. Most frames of most games touch a few
 * dozen lines. The bitmap is 32 bytes and the marking is two instructions on
 * the store path; it is the single largest thing between this and a playable
 * frame rate, and off it exists only to prove that.
 */
#ifndef BK_VIDEO_DIRTY
#define BK_VIDEO_DIRTY          1
#endif

/*- Peripherals -------------------------------------------------------------*/
// Keyboard: 0177660/0177662 and the two interrupt vectors, 060 and 0274.
#ifndef BK_IO_KEYBOARD
#define BK_IO_KEYBOARD          1
#endif

/*
 * EVNT, the 50 Hz line from the video timing, vector 0100.
 *
 * It is what a BK game's clock is: the whole tempo of the thing is one screen
 * refresh. Off, anything that waits for it hangs.
 */
#ifndef BK_IO_EVNT
#define BK_IO_EVNT              1
#endif

/*
 * The programmable timer, 0177706-0177712.
 *
 * A down counter with a four-way prescaler that the BK-0010-01 has and most
 * software ignores in favour of counting screen refreshes.
 */
#ifndef BK_IO_TIMER
#define BK_IO_TIMER             1
#endif

/*
 * The 0177714 port, which is where a joystick appears - eight bits, four
 * directions and four buttons. The instrument has no joystick; this is on
 * because the front panel keys can drive it, and some games read only this.
 */
#ifndef BK_IO_JOYSTICK
#define BK_IO_JOYSTICK          1
#endif

/*
 * There is no switch for the tape or the loudspeaker. Neither is emulated:
 * there is nothing to load from and the instrument has no speaker an
 * application can reach. 0177716 is decoded anyway, because programs read it
 * for the "a key is down" flag, and its tape bits simply never change.
 */

/*
 * The ВМ1's own registers at 0177700-0177704 - mode, interrupt address, error.
 * Constants, and the monitor reads them on the way up.
 */
#ifndef BK_IO_1801_REGS
#define BK_IO_1801_REGS         1
#endif

/*- Loading -----------------------------------------------------------------*/
/*
 * ROM images and programs off the 8 MB SPI part, through spifs.
 *
 * The tape format has no switch of its own: two words of header - load address
 * and length - then the bytes, and every BK program that survived is in one of
 * these. There is no second format to choose between.
 */
#ifndef BK_LOAD_SPIFS
#define BK_LOAD_SPIFS           1
#endif

/*
 * The stand-in ROM.
 *
 * Without a monitor image there is nothing at 0100000, and a machine whose
 * reset vector points at erased memory does not get far. This is a few dozen
 * bytes that set the screen registers up the way the monitor leaves them, hand
 * control to whatever was loaded, and return from any EMT rather than
 * following vector 030 into the weeds.
 *
 * It is not a monitor. A program that calls EMT 6 to read a key gets nothing
 * back. It is enough for a self-contained game and enough to see that the
 * processor and the screen work, which is what it is for.
 */
#ifndef BK_STUB_ROM
#define BK_STUB_ROM             1
#endif

/*- Looking at it -----------------------------------------------------------*/
// Disassembly, one instruction at a time. Used by the debug overlay and by
// the fault report, which is worth more than the register dump on its own.
#ifndef BK_DISASM
#define BK_DISASM               1
#endif

// The overlay: registers, the instruction at PC, single step, breakpoint.
#ifndef BK_DEBUGGER
#define BK_DEBUGGER             1
#endif

// Frames, emulated cycles and the ratio to real time, on the status line.
#ifndef BK_STATS
#define BK_STATS                1
#endif

/*- Consistency -------------------------------------------------------------*/
#if BK_DEBUGGER && !BK_DISASM
#error "BK_DEBUGGER needs BK_DISASM: the overlay's main line is the instruction"
#endif

#if !BK_VIDEO_COLOR && !BK_VIDEO_MONO
#error "BK_VIDEO_COLOR or BK_VIDEO_MONO - with neither there is no picture"
#endif

#if BK_MODEL_0011M || !BK_MODEL_0010_01
#error "only the BK-0010-01: no bank switching, no second screen, one palette register"
#endif

#endif // _BK_CONFIG_H_
