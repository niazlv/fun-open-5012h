/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The registers.
 *
 * A BK's peripherals are five words at the top of memory, and most of what a
 * program does with the machine it does through two of them. The keyboard is
 * one code at a time behind a ready flag; the screen is a first line number;
 * the rest is a timer nobody used and a parallel port that a joystick plugs
 * into.
 *
 * The one piece of it that is not obvious is the keyboard interrupt mask,
 * which is inverted: bit 6 of 0177660 SET means interrupts are held off. BASIC
 * writes zero there on its way up, and an emulator that reads the bit the
 * natural way round has a machine that never delivers a keystroke.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "bk_config.h"
#include "bk_io.h"
#include "bk_cpu.h"
#include "bk_mem.h"

/*- Definitions -------------------------------------------------------------*/
/*
 * The programmable timer counts at 23437.5 Hz on a BK-0010 - one tick every
 * 42.67 microseconds - and divides that by 1, 4, 16 or 64 according to two
 * bits of its control register. Kept as the reciprocal in tenths of a
 * microsecond so that the accumulator stays in integers.
 */
#define TIMER_TICK_NS           42667u

// 0177712, and every bit of it matters to something
#define TIMER_STOP              0000001u
#define TIMER_WRAPAROUND        0000002u
#define TIMER_EXPENABLE         0000004u
#define TIMER_ONESHOT           0000010u
#define TIMER_RUN               0000020u
#define TIMER_PRESCALE          0000140u
#define TIMER_EXPIRY            0000200u

// 0177716, the four bits of it that are not the boot address
#define SYS_READY               0000004u    // something happened since you read
#define SYS_TAPE_OUT            0000020u
#define SYS_TAPE_IN             0000040u
#define SYS_SOUND               0000100u    // and "no key is down", shared
#define SYS_MOTOR               0000200u

/*- Variables ---------------------------------------------------------------*/
static uint16_t g_kbd_status;
static uint8_t g_kbd_data;
static uint16_t g_kbd_deferred;             // a vector waiting for the mask

static uint16_t g_palette;                  // 0177662 written
static uint16_t g_scroll;

#if BK_IO_JOYSTICK
static uint16_t g_parallel_in;
#endif
static uint16_t g_parallel_out;

static uint16_t g_system;

#if BK_IO_TIMER
static uint16_t g_timer_reload;
static uint16_t g_timer;
static uint16_t g_timer_flags;
static uint16_t g_timer_prediv;
static uint32_t g_timer_ns;
#endif

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void bk_io_reset(void)
{
    // Bit 6 set: the keyboard interrupt starts held off, and software lifts it
    // when it has somewhere for a keystroke to go
    g_kbd_status = BK_KBD_MASK;
    g_kbd_data = 0;
    g_kbd_deferred = 0;

    g_palette = 0;

    // What the monitor leaves behind: full screen, first line at the top
    g_scroll = BK_SCROLL_FULL | BK_SCROLL_HOME;

#if BK_IO_JOYSTICK
    g_parallel_in = 0;
#endif
    g_parallel_out = 0;

    // No key down, and the tape motor stopped
    g_system = SYS_SOUND | SYS_MOTOR;

#if BK_IO_TIMER
    g_timer_reload = 0;
    g_timer = 0;
    g_timer_flags = 0177400u | TIMER_STOP;
    g_timer_prediv = 0;
    g_timer_ns = 0;
#endif
}

//-----------------------------------------------------------------------------
#if BK_IO_KEYBOARD
void bk_io_key(uint8_t code, bool ar2)
{
    uint16_t vector = ar2 ? BK_VEC_KEY_AR2 : BK_VEC_KEY;

    g_system &= (uint16_t)~SYS_SOUND;       // a key is down
    g_system |= SYS_READY;

    /*
     * A code that has not been read yet is not replaced. That is the hardware:
     * one register, one flag, and the second key pressed before a program got
     * round to the first is simply lost. Overwriting instead would make the
     * emulator drop the OLDER of the two, which is the one the player pressed
     * first and the one they are waiting to see happen.
     */
    if (g_kbd_status & BK_KBD_READY)
        return;

    g_kbd_data = (uint8_t)(code & 0177u);
    g_kbd_status |= BK_KBD_READY;

    if (0 == (g_kbd_status & BK_KBD_MASK))
        bk_cpu_virq(vector);
    else
        g_kbd_deferred = vector;            // until software lifts the mask
}

//-----------------------------------------------------------------------------
void bk_io_key_release(void)
{
    g_system |= SYS_SOUND | SYS_READY;
}
#endif

//-----------------------------------------------------------------------------
// STOP is not a key code. It is a line into the processor, and on real
// hardware it drops into halt mode - which this does not have. Vectoring
// through 004 is what the monitor's own handler is reached by on a machine
// with one, and it is the closest useful thing: a program that installs a
// handler gets control, and one that does not stops with a message.
void bk_io_stop_key(void)
{
    bk_cpu_virq(BK_VEC_BUS);
}

//-----------------------------------------------------------------------------
#if BK_IO_JOYSTICK
void bk_io_joystick(uint8_t mask)
{
    g_parallel_in = mask;
}
#endif

//-----------------------------------------------------------------------------
// One screen refresh. This is the clock every BK game is written against - not
// the processor's, which nobody could count on, but the 50 Hz line off the
// video timing.
void bk_io_frame(void)
{
    bk_cpu_evnt();
}

//-----------------------------------------------------------------------------
#if BK_IO_TIMER
static void timer_tick(void)
{
    unsigned prescale;

    if (g_timer_flags & TIMER_STOP)
    {
        g_timer = g_timer_reload;
        return;
    }

    if (0 == (g_timer_flags & TIMER_RUN))
        return;

    g_timer_prediv++;

    switch ((g_timer_flags & TIMER_PRESCALE) >> 5)
    {
    case 1:  prescale = 16; break;
    case 2:  prescale = 4;  break;
    case 3:  prescale = 64; break;
    default: prescale = 1;  break;
    }

    if (g_timer_prediv < prescale)
        return;

    g_timer_prediv = 0;
    g_timer--;

    if (0 != g_timer)
        return;

    // Wraparound is the mode where the counter simply runs on and the expiry
    // bit is never raised, whatever EXPENABLE says
    if (g_timer_flags & TIMER_WRAPAROUND)
        return;

    if (g_timer_flags & TIMER_ONESHOT)
        g_timer_flags &= (uint16_t)~TIMER_RUN;

    g_timer = g_timer_reload;

    if (g_timer_flags & TIMER_EXPENABLE)
        g_timer_flags |= TIMER_EXPIRY;
}
#endif

//-----------------------------------------------------------------------------
// Microseconds of emulated time. Only the programmable timer cares, and it is
// the one peripheral here that a build can leave out entirely.
void bk_io_advance(uint32_t us)
{
#if BK_IO_TIMER
    g_timer_ns += us * 1000u;

    // Capped so that a long stall - the launcher opening a menu over this -
    // does not turn into thousands of catch-up ticks in one pass
    for (int guard = 0; guard < 4096 && g_timer_ns >= TIMER_TICK_NS; guard++)
    {
        g_timer_ns -= TIMER_TICK_NS;
        timer_tick();
    }

    if (g_timer_ns >= TIMER_TICK_NS)
        g_timer_ns = 0;
#else
    (void)us;
#endif
}

//-----------------------------------------------------------------------------
uint16_t bk_io_scroll(void)
{
#if BK_VIDEO_SCROLL
    return g_scroll;
#else
    return BK_SCROLL_FULL | BK_SCROLL_HOME;
#endif
}

//-----------------------------------------------------------------------------
unsigned bk_io_palette(void)
{
#if BK_VIDEO_PALETTES
    // The palette number is the top nibble of what was written to 0177662
    return (g_palette >> 8) & 017u;
#else
    return 0;
#endif
}

//-----------------------------------------------------------------------------
// The register as it stands, with none of the side effects a read has. For the
// debug page, and for the read-modify-write a byte store to a register does.
uint16_t bk_io_reg(uint16_t addr)
{
    switch (addr)
    {
    case BK_REG_KBD_STATUS: return g_kbd_status;
    case BK_REG_KBD_DATA:   return g_kbd_data;
    case BK_REG_SCROLL:     return g_scroll;
#if BK_IO_TIMER
    case BK_REG_TIMER_RELOAD: return g_timer_reload;
    case BK_REG_TIMER_COUNT:  return g_timer;
    case BK_REG_TIMER_CTL:    return g_timer_flags;
#endif
#if BK_IO_JOYSTICK
    case BK_REG_PARALLEL:   return g_parallel_in;
#else
    case BK_REG_PARALLEL:   return 0;
#endif
    case BK_REG_SYSTEM:     return g_system;
    default:                return 0;
    }
}

//-----------------------------------------------------------------------------
uint16_t bk_io_read(uint16_t addr)
{
    switch (addr)
    {
#if BK_IO_KEYBOARD
    case BK_REG_KBD_STATUS:
        return g_kbd_status;

    case BK_REG_KBD_DATA:
        // Reading the code is what acknowledges it. A program that polls
        // rather than taking the interrupt has consumed the key by doing so,
        // so any interrupt still owed for it is cancelled here too.
        g_kbd_status &= (uint16_t)~BK_KBD_READY;
        g_kbd_deferred = 0;
        return g_kbd_data;
#endif

    case BK_REG_SCROLL:
        return g_scroll;

#if BK_IO_TIMER
    case BK_REG_TIMER_RELOAD:
        return g_timer_reload;

    case BK_REG_TIMER_COUNT:
        return g_timer;

    case BK_REG_TIMER_CTL:
        return g_timer_flags;
#endif

#if BK_IO_JOYSTICK
    case BK_REG_PARALLEL:
        return g_parallel_in;
#endif

    case BK_REG_SYSTEM:
    {
        uint16_t value = g_system;

        // Bit 2 says "something changed since you last looked", so looking
        // clears it
        g_system &= (uint16_t)~SYS_READY;
        return value;
    }

#if BK_IO_1801_REGS
    /*
     * The processor's own three registers. Constants - the mode register
     * reports a ВМ1 with no cache and no floating point, the error register
     * reports no error - and the monitor reads them on its way up.
     */
    case 0177700: return 0177740u;
    case 0177702: return 0177777u;
    case 0177704: return 0177440u;
#endif

    default:
        /*
         * Everything else in the page. A real bus answers here - there are
         * boards that put a disk controller at 0177130 and a serial line at
         * 0177560 - and a machine that took a bus error instead would stop on
         * software that is only asking whether the hardware is fitted.
         *
         * So: zero, and a count, which is the number worth seeing on the debug
         * page when a program is talking to something that is not here.
         */
        bk_io_unknown++;
        return 0;
    }
}

//-----------------------------------------------------------------------------
void bk_io_write(uint16_t addr, uint16_t value)
{
    switch (addr)
    {
#if BK_IO_KEYBOARD
    case BK_REG_KBD_STATUS:
    {
        bool was_masked = (0 != (g_kbd_status & BK_KBD_MASK));

        // Bit 6 is the only thing software owns here; the ready flag belongs
        // to the hardware and the rest is not wired
        g_kbd_status = (uint16_t)((g_kbd_status & ~BK_KBD_MASK) |
            (value & BK_KBD_MASK));

        if (was_masked && 0 == (g_kbd_status & BK_KBD_MASK) && g_kbd_deferred)
        {
            // A key arrived while the mask was up and has not been read.
            // Lifting the mask is when its interrupt happens.
            bk_cpu_virq(g_kbd_deferred);
            g_kbd_deferred = 0;
        }
        return;
    }
#endif

    case BK_REG_KBD_DATA:
        // Read it is the keyboard, written it is the palette. Two registers
        // sharing an address, which is what happens when a machine grows a
        // feature after its address map was fixed.
        g_palette = value;
        bk_vram_mark_all();
        return;

    case BK_REG_SCROLL:
        // Only the nine bits that exist. The rest read back as zero on real
        // hardware and a program that writes them expects that.
        value &= (BK_SCROLL_LINE | BK_SCROLL_FULL);

        if (value != g_scroll)
        {
            g_scroll = value;
            bk_vram_mark_all();
        }
        return;

#if BK_IO_TIMER
    case BK_REG_TIMER_RELOAD:
        g_timer_reload = value;
        return;

    case BK_REG_TIMER_COUNT:
        return;                 // the counter itself is read only

    case BK_REG_TIMER_CTL:
        // Writing the control register restarts the count, whatever was in it
        g_timer = g_timer_reload;
        g_timer_prediv = 0;
        g_timer_flags = (uint16_t)(0177400u | value);
        return;
#endif

    case BK_REG_PARALLEL:
        g_parallel_out = value;
        return;

    case BK_REG_SYSTEM:
        g_system |= SYS_READY;

        /*
         * Bit 11 chooses what the write means. Set, it is the memory manager
         * of a BK-0011, which this machine has not got; clear, it is the tape
         * and the loudspeaker, which it has and this build does not drive.
         *
         * Nothing is stored either way, deliberately: what a program reads
         * back from here is the state of the hardware, and reflecting a write
         * would be inventing a tape that answered.
         */
        return;

    default:
        bk_io_unknown++;
        return;
    }
}

//-----------------------------------------------------------------------------
// Kept so that the parallel output is not merely written into a void the
// compiler can see through, and because a printer or a Covox is the obvious
// next thing to hang off it
uint16_t bk_io_parallel_out(void)
{
    return g_parallel_out;
}
