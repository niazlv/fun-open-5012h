/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The hundred and twenty-eight bytes at the top of the address space.
 */

#ifndef _BK_IO_H_
#define _BK_IO_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "bk_config.h"

/*- Definitions -------------------------------------------------------------*/
#define BK_REG_KBD_STATUS       0177660u
#define BK_REG_KBD_DATA         0177662u    // read; write is the palette
#define BK_REG_SCROLL           0177664u
#define BK_REG_TIMER_RELOAD     0177706u
#define BK_REG_TIMER_COUNT      0177710u
#define BK_REG_TIMER_CTL        0177712u
#define BK_REG_PARALLEL         0177714u    // joystick lives here
#define BK_REG_SYSTEM           0177716u

// 0177660 bit 6 masks the keyboard interrupt, bit 7 says a code is waiting.
// The mask is inverted from what you would expect: set means held off.
#define BK_KBD_MASK             0000100u
#define BK_KBD_READY            0000200u

// 0177664: the low byte is the first line to display, bit 9 is the full screen
#define BK_SCROLL_LINE          0000377u
#define BK_SCROLL_FULL          0001000u
#define BK_SCROLL_HOME          0000330u    // what the monitor leaves it at

// 0177714, read. Four directions and four buttons.
#define BK_JOY_BUTTON1          0x01u
#define BK_JOY_BUTTON2          0x02u
#define BK_JOY_BUTTON3          0x04u
#define BK_JOY_BUTTON4          0x08u
#define BK_JOY_LEFT             0x10u
#define BK_JOY_DOWN             0x20u
#define BK_JOY_RIGHT            0x40u
#define BK_JOY_UP               0x80u

/*
 * The key codes a BK keyboard produces. Seven bits, KOI-7, and the arrows are
 * where the control characters would be - which is why a BK program reading
 * "backspace" from the keyboard is reading the left arrow key.
 */
#define BK_KEY_LEFT             0010u
#define BK_KEY_TAB              0011u
#define BK_KEY_ENTER            0012u
#define BK_KEY_RIGHT            0031u
#define BK_KEY_UP               0032u
#define BK_KEY_DOWN             0033u
#define BK_KEY_SPACE            0040u

/*- Prototypes --------------------------------------------------------------*/
void bk_io_reset(void);

#if BK_IO_KEYBOARD
/*
 * A key went down. `ar2` is the shift-like key that turns the seven bit code
 * into a different eight bit one - and it does that by choosing the interrupt
 * vector, 0274 instead of 060, rather than by changing the code.
 */
void bk_io_key(uint8_t code, bool ar2);
void bk_io_key_release(void);

/*
 * Throw away a code nobody has read yet.
 *
 * The hardware has one register and one ready flag, and a controller that
 * refuses to overwrite an unread code - so a keystroke the program has not got
 * round to reading blocks the next one. That is faithful, and on the real
 * machine it is nearly invisible: a BK game reads the keyboard every 50 Hz
 * frame, and nobody presses two keys inside twenty milliseconds.
 *
 * It is not invisible here, because a front panel gives state and not events:
 * the emulator has to invent a repeat to make a held key work, and a repeat
 * that fires just before the player lets go leaves a stale code in the
 * register. The player then presses left and walks up.
 *
 * So when the key that is physically down changes, the old one goes. Losing a
 * press the program never read costs a move; delivering it costs the move the
 * player actually asked for, and in a game like Boulder Dash that is a death.
 */
void bk_io_key_flush(void);
#endif

// The STOP key. Not a code at all: it is a line into the processor.
void bk_io_stop_key(void);

#if BK_IO_JOYSTICK
void bk_io_joystick(uint8_t mask);          // BK_JOY_*, the whole state
#endif

/*
 * Time passing. The 50 Hz call is the screen refresh, which is what a BK game
 * counts; the microsecond call drives the programmable timer and may be left
 * out of a build that does not have one.
 */
void bk_io_frame(void);
void bk_io_advance(uint32_t us);

// What the video output needs to know
uint16_t bk_io_scroll(void);
unsigned bk_io_palette(void);

// For the debug page, and for the read-modify-write a byte store does: the
// register as it stands, with none of the side effects a read has.
uint16_t bk_io_reg(uint16_t addr);

// The last word written to 0177714. Nothing drives a printer or a Covox yet,
// and this is where one would read what a program sent them.
uint16_t bk_io_parallel_out(void);

#endif // _BK_IO_H_
