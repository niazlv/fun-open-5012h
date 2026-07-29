/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * К1801ВМ1 - the PDP-11 the BK is built around.
 */

#ifndef _BK_CPU_H_
#define _BK_CPU_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "bk_config.h"

/*- Definitions -------------------------------------------------------------*/
#define PSW_C           0000001u
#define PSW_V           0000002u
#define PSW_Z           0000004u
#define PSW_N           0000010u
#define PSW_T           0000020u
#define PSW_PRIO        0000200u    // set: vectored interrupts are held off
#define PSW_HALT        0000400u    // the 1801's halt mode

/*
 * The vectors, all of them, in one place.
 *
 * 060 and 0274 are both the keyboard: the controller produces seven-bit codes,
 * and the eighth bit - which is what АР2 and the lowercase latch supply - is
 * carried by the choice of vector rather than by the code. A monitor that only
 * hooks 060 sees half the keyboard.
 */
#define BK_VEC_HALT     0000002u
#define BK_VEC_BUS      0000004u    // odd address, or nothing answered
#define BK_VEC_RESERVED 0000010u    // an opcode this processor does not have
#define BK_VEC_TBIT     0000014u    // BPT, and the T bit
#define BK_VEC_IOT      0000020u
#define BK_VEC_POWER    0000024u
#define BK_VEC_EMT      0000030u
#define BK_VEC_TRAP     0000034u
#define BK_VEC_KEY      0000060u
#define BK_VEC_EVNT     0000100u    // 50 Hz, off the video timing
#define BK_VEC_KEY_AR2  0000274u

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    uint16_t r[8];              // r[6] is SP, r[7] is PC
    uint16_t psw;

    uint32_t cycles;            // emulated processor cycles since reset
    uint32_t insns;

    uint16_t pc_insn;           // where the instruction being run started
    uint16_t last_trap;         // the vector last taken, for the debug page

    /*
     * Why the processor is not running. NULL while it is.
     *
     * Set by HALT, by a trap taken with no vector installed, and by the
     * runaway guard. The application puts it on the screen verbatim, because
     * "HALT" and "trap through an empty vector" are different problems and a
     * blank screen looks the same either way.
     */
    const char *stopped;

    bool waiting;               // in WAIT, until something interrupts
} bk_cpu_t;

/*- Variables ---------------------------------------------------------------*/
extern bk_cpu_t bk_cpu;

/*- Prototypes --------------------------------------------------------------*/
void bk_cpu_reset(uint16_t pc, uint16_t psw);

/*
 * Run for about `budget` processor cycles and return what was actually spent -
 * an instruction is never cut in half, so the answer overshoots by up to the
 * length of one.
 *
 * Returns early, having spent the whole budget, if the processor stops: check
 * bk_cpu.stopped rather than the return value.
 */
uint32_t bk_cpu_run(uint32_t budget);

// One instruction, whatever it costs. For the debugger's step key.
uint32_t bk_cpu_step(void);

/*
 * Raise an interrupt. Both are held off while PSW_PRIO is set; the request
 * stays pending until it can be taken, which is what lets a key pressed inside
 * a critical section still arrive.
 *
 * The queue is four deep. It overflows only if something raises interrupts
 * faster than the processor can take them, and then the oldest is kept - a
 * dropped keystroke beats a keyboard that stops answering.
 */
void bk_cpu_virq(uint16_t vector);
void bk_cpu_evnt(void);

// Everything pending, thrown away. Part of a reset, and of loading a program.
void bk_cpu_clear_irq(void);

#endif // _BK_CPU_H_
