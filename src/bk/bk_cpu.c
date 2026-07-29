/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * К1801ВМ1 - the processor in the BK-0010-01.
 *
 * It is a PDP-11: the same eight registers with the program counter as one of
 * them, the same eight addressing modes on both operands of every instruction,
 * the same four condition codes. What it is not is a whole PDP-11. There is no
 * memory management, no supervisor mode, and - the omission that shaped every
 * program ever written for this machine - no MUL and no DIV. BK code multiplies
 * in a loop because it had no choice, and an emulator that quietly provides the
 * instruction is emulating a different computer. See BK_CPU_EIS.
 *
 * Two decisions worth knowing about before reading:
 *
 * A trap raised inside an instruction is taken when that instruction ends,
 * rather than aborting it where it stands. Real hardware aborts. The
 * difference is visible only to a program that takes a bus error and then
 * inspects what the faulting instruction managed to do first, and paying for
 * it would mean a check after every single memory access in the file. What
 * makes it affordable here is that on this machine every address answers -
 * RAM to 077777, ROM to 0177577, registers above that - so the only bus error
 * a BK can raise is a word access to an odd address, which is a program that
 * has already lost its footing.
 *
 * Cycle counts are a shape, not a measurement. A base per instruction class
 * plus the cost of reaching each operand, which is how the family's timing
 * tables are built, but the constants have not been checked against silicon.
 * They are here so a game written for 3 MHz runs at something like 3 MHz; the
 * application has a speed control because they are approximate.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bk_config.h"
#include "bk_cpu.h"
#include "bk_mem.h"

/*- Definitions -------------------------------------------------------------*/
// An operand that turned out to be a register rather than an address. 64 KB of
// address space leaves the whole top half of a uint32_t free to say so.
#define OPR_REG         0x10000u

#define VIRQ_DEPTH      4

/*- Variables ---------------------------------------------------------------*/
bk_cpu_t bk_cpu;

#define R               bk_cpu.r
#define PC              bk_cpu.r[7]
#define SP              bk_cpu.r[6]
#define PSW             bk_cpu.psw

// Set during an instruction, taken when it ends. Zero means nothing pending.
static uint16_t g_trap;

// Cycles charged for the instruction being executed
static uint32_t g_cycles;

static uint16_t g_virq[VIRQ_DEPTH];
static uint8_t g_virq_count;
static bool g_evnt;

// RTT leaves the T bit set but must not trap on the instruction it returns to,
// which is the whole difference between it and RTI
static bool g_defer_tbit;

#if BK_CPU_CYCLES
/*
 * What each addressing mode costs on top of the instruction.
 *
 * Register is free, one indirection is a read, the deferred and indexed modes
 * pay for the extra word they fetch and the extra read they do. The shape is
 * the family's; the numbers are not measured.
 */
static const uint8_t g_mode_cost[8] = { 0, 3, 3, 7, 4, 8, 7, 11 };

#define CHARGE(n)       (g_cycles += (n))
#else
#define CHARGE(n)       ((void)0)
#endif

/*- Local prototypes --------------------------------------------------------*/
static void execute(uint16_t op);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Condition codes. Written out rather than packed because every one of them is
// read individually by the branches, and the compiler folds the sequences.
#define SET_N(c)        (PSW = (c) ? (PSW | PSW_N) : (PSW & (uint16_t)~PSW_N))
#define SET_Z(c)        (PSW = (c) ? (PSW | PSW_Z) : (PSW & (uint16_t)~PSW_Z))
#define SET_V(c)        (PSW = (c) ? (PSW | PSW_V) : (PSW & (uint16_t)~PSW_V))
#define SET_C(c)        (PSW = (c) ? (PSW | PSW_C) : (PSW & (uint16_t)~PSW_C))

#define GET_N()         (0 != (PSW & PSW_N))
#define GET_Z()         (0 != (PSW & PSW_Z))
#define GET_V()         (0 != (PSW & PSW_V))
#define GET_C()         (0 != (PSW & PSW_C))

//-----------------------------------------------------------------------------
// Word access with the odd-address check the processor owes the bus. The
// instruction carries on with a zero and the trap lands when it finishes; see
// the note at the top of the file.
static inline uint16_t rdw(uint16_t addr)
{
#if BK_CPU_BUS_ERROR
    if (addr & 1)
    {
        g_trap = BK_VEC_BUS;
        return 0;
    }
#endif

    return bk_read16(addr);
}

//-----------------------------------------------------------------------------
static inline void wrw(uint16_t addr, uint16_t value)
{
#if BK_CPU_BUS_ERROR
    if (addr & 1)
    {
        g_trap = BK_VEC_BUS;
        return;
    }
#endif

    bk_write16(addr, value);
}

//-----------------------------------------------------------------------------
static inline uint16_t fetch(void)
{
    uint16_t w = rdw(PC);

    PC = (uint16_t)(PC + 2);

    return w;
}

//-----------------------------------------------------------------------------
static inline void push(uint16_t value)
{
    SP = (uint16_t)(SP - 2);
    wrw(SP, value);
}

//-----------------------------------------------------------------------------
static inline uint16_t pop(void)
{
    uint16_t v = rdw(SP);

    SP = (uint16_t)(SP + 2);

    return v;
}

//-----------------------------------------------------------------------------
// Where an operand is. Either an address, or OPR_REG and a register number -
// the two cases have to stay distinguishable to the end, because a byte write
// to a register is not the same thing as a byte write to memory.
//
// The side effects happen here and only here: this is what advances an
// autoincrement, and it advances by one for a byte operand except through the
// stack pointer and the program counter, which always move by two because the
// things they point at are always words.
static uint32_t operand(unsigned spec, unsigned sz)
{
    unsigned reg = spec & 7;
    unsigned mode = (spec >> 3) & 7;
    unsigned step = (1 == sz && reg < 6) ? 1 : 2;
    uint16_t a;

    CHARGE(g_mode_cost[mode]);

    switch (mode)
    {
    case 0:
        return OPR_REG | reg;

    case 1:
        return R[reg];

    case 2:
        a = R[reg];
        R[reg] = (uint16_t)(R[reg] + step);
        return a;

    case 3:
        a = R[reg];
        R[reg] = (uint16_t)(R[reg] + 2);
        return rdw(a);

    case 4:
        R[reg] = (uint16_t)(R[reg] - step);
        return R[reg];

    case 5:
        R[reg] = (uint16_t)(R[reg] - 2);
        return rdw(R[reg]);

    case 6:
        a = fetch();
        return (uint16_t)(a + R[reg]);

    default:
        a = fetch();
        return rdw((uint16_t)(a + R[reg]));
    }
}

//-----------------------------------------------------------------------------
static inline uint16_t op_read(uint32_t o, unsigned sz)
{
    if (o & OPR_REG)
        return (1 == sz) ? (uint16_t)(R[o & 7] & 0xff) : R[o & 7];

    return (1 == sz) ? bk_read8((uint16_t)o) : rdw((uint16_t)o);
}

//-----------------------------------------------------------------------------
// A byte written to a register leaves the high half alone. MOVB is the one
// exception in the whole instruction set and has its own function below.
static inline void op_write(uint32_t o, unsigned sz, uint16_t value)
{
    if (o & OPR_REG)
    {
        if (1 == sz)
            R[o & 7] = (uint16_t)((R[o & 7] & 0xff00) | (value & 0xff));
        else
            R[o & 7] = value;

        return;
    }

    if (1 == sz)
        bk_write8((uint16_t)o, (uint8_t)value);
    else
        wrw((uint16_t)o, value);
}

//-----------------------------------------------------------------------------
static inline void op_write_movb(uint32_t o, uint16_t value)
{
    if (o & OPR_REG)
    {
        // Sign extended, so that MOVB #-1,R0 leaves -1 and not 255
        R[o & 7] = (uint16_t)(int16_t)(int8_t)value;
        return;
    }

    bk_write8((uint16_t)o, (uint8_t)value);
}

//-----------------------------------------------------------------------------
// Push PC and PSW, load the pair at the vector.
//
// A vector that was never filled in is a dead end - the processor would run
// whatever erased memory decodes to - so it stops instead and says so. On real
// hardware that is a machine that has gone quiet and needs the reset button;
// here it is a line on the screen naming the vector, which is the difference
// between "it does not work" and "there is no ROM loaded".
static void take_trap(uint16_t vector)
{
    uint16_t newpc = bk_read16(vector);
    uint16_t newpsw = bk_read16((uint16_t)(vector + 2));
    uint16_t oldpsw = PSW;

    bk_cpu.last_trap = vector;

    if (0 == newpc || 0177777u == newpc)
    {
        bk_cpu.stopped = "trap through an empty vector";
        return;
    }

    PSW &= (uint16_t)~PSW_HALT;

    push(oldpsw);
    push(PC);

    PC = newpc;
    PSW = (uint16_t)(newpsw & 0377u);

    bk_cpu.waiting = false;
}

//-----------------------------------------------------------------------------
void bk_cpu_virq(uint16_t vector)
{
    if (g_virq_count < VIRQ_DEPTH)
        g_virq[g_virq_count++] = vector;
}

//-----------------------------------------------------------------------------
void bk_cpu_evnt(void)
{
#if BK_IO_EVNT
    g_evnt = true;
#endif
}

//-----------------------------------------------------------------------------
void bk_cpu_clear_irq(void)
{
    g_virq_count = 0;
    g_evnt = false;
}

//-----------------------------------------------------------------------------
// EVNT first, then the vectored requests in the order they arrived. Both wait
// while the processor is running at priority - which is what makes a keystroke
// pressed during a critical section arrive after it rather than not at all.
//
// Returns the cycles the entry cost, so that a machine spending its whole time
// in interrupts still advances the caller's budget and does not spin.
static uint32_t service_interrupts(void)
{
    if (PSW & PSW_PRIO)
        return 0;

#if BK_IO_EVNT
    if (g_evnt)
    {
        g_evnt = false;
        take_trap(BK_VEC_EVNT);
        return 24;
    }
#endif

    if (g_virq_count > 0)
    {
        uint16_t vector = g_virq[0];

        g_virq_count--;

        for (unsigned i = 0; i < g_virq_count; i++)
            g_virq[i] = g_virq[i + 1];

        take_trap(vector);
        return 24;
    }

    return 0;
}

//-----------------------------------------------------------------------------
void bk_cpu_reset(uint16_t pc, uint16_t psw)
{
    memset(&bk_cpu, 0, sizeof(bk_cpu));

    PC = pc;
    PSW = psw;

    g_trap = 0;
    g_defer_tbit = false;

    bk_cpu_clear_irq();
}

//-----------------------------------------------------------------------------
// One instruction, plus whatever trap it earned.
static uint32_t run_one(void)
{
    bool tbit;
    uint16_t op;

    g_cycles = 8;
    g_trap = 0;

    tbit = (0 != (PSW & PSW_T)) && !g_defer_tbit;
    g_defer_tbit = false;

    bk_cpu.pc_insn = PC;

    op = fetch();
    execute(op);

    bk_cpu.insns++;
    bk_cpu.cycles += g_cycles;

    if (g_trap)
    {
        take_trap(g_trap);
        g_trap = 0;
    }
#if BK_CPU_TBIT
    else if (tbit)
    {
        take_trap(BK_VEC_TBIT);
    }
#else
    (void)tbit;
#endif

    return g_cycles;
}

//-----------------------------------------------------------------------------
uint32_t bk_cpu_step(void)
{
    uint32_t entry;

    if (bk_cpu.stopped)
        return 0;

    entry = service_interrupts();

    if (bk_cpu.stopped)
        return entry;

    bk_cpu.waiting = false;

    return entry + run_one();
}

//-----------------------------------------------------------------------------
uint32_t bk_cpu_run(uint32_t budget)
{
    uint32_t spent = 0;

    if (bk_cpu.stopped)
        return budget;

    while (spent < budget)
    {
        spent += service_interrupts();

        if (bk_cpu.stopped)
            return budget;

        if (bk_cpu.waiting)
        {
            /*
             * WAIT means the program has nothing to do until the hardware says
             * otherwise, and on this machine that is the 50 Hz line. Spinning
             * through the rest of the budget one no-op at a time would cost
             * real time on the Cortex for emulated time that does not pass, so
             * the budget is simply consumed.
             */
            bk_cpu.cycles += budget - spent;
            return budget;
        }

        spent += run_one();
    }

    return spent;
}

/*- The instruction set -----------------------------------------------------*/

//-----------------------------------------------------------------------------
static void reserved_instruction(void)
{
#if BK_CPU_TRAPS
    g_trap = BK_VEC_RESERVED;
#else
    bk_cpu.stopped = "reserved instruction";
#endif
}

//-----------------------------------------------------------------------------
static void branch(uint16_t op, bool taken)
{
    if (taken)
    {
        PC = (uint16_t)(PC + 2 * (int16_t)(int8_t)(op & 0377u));
        CHARGE(4);
    }
}

//-----------------------------------------------------------------------------
// 01SSDD..06SSDD and their byte forms. Source is resolved before destination,
// which matters whenever both use the same register with a side effect.
static void double_operand(uint16_t op)
{
    unsigned kind = (op >> 12) & 7;

    /*
     * 06 and 16 are ADD and SUB, not ADD and ADDB: the byte bit is spent on
     * naming a second word instruction. Resolving their operands as bytes
     * would step an autoincrement by one and quietly corrupt the pointer.
     */
    unsigned sz = ((op & 0100000u) && 6 != kind) ? 1 : 2;
    uint32_t mask = (1 == sz) ? 0xffu : 0xffffu;
    uint32_t msb = (1 == sz) ? 0x80u : 0x8000u;

    uint32_t so = operand((unsigned)(op >> 6) & 077u, sz);
    uint16_t s = op_read(so, sz);
    uint32_t dop = operand(op & 077u, sz);
    uint16_t d;
    uint32_t res;

    switch (kind)
    {
    case 1:     // MOV, MOVB
        if (1 == sz)
            op_write_movb(dop, s);
        else
            op_write(dop, sz, s);

        SET_N(0 != (s & msb));
        SET_Z(0 == (s & mask));
        SET_V(false);
        return;

    case 2:     // CMP, CMPB - source minus destination, and nothing stored
        d = op_read(dop, sz);
        res = ((uint32_t)s - d) & mask;

        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(0 != ((((uint32_t)s ^ d) & (~(uint32_t)d ^ res)) & msb));
        SET_C((s & mask) < (d & mask));
        return;

    case 3:     // BIT, BITB
        d = op_read(dop, sz);
        res = (uint32_t)s & d & mask;

        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(false);
        return;

    case 4:     // BIC, BICB
        d = op_read(dop, sz);
        res = (uint32_t)d & ~(uint32_t)s & mask;
        op_write(dop, sz, (uint16_t)res);

        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(false);
        return;

    case 5:     // BIS, BISB
        d = op_read(dop, sz);
        res = ((uint32_t)d | s) & mask;
        op_write(dop, sz, (uint16_t)res);

        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(false);
        return;

    default:    // 06 ADD, 16 SUB
        d = op_read(dop, sz);

        if (op & 0100000u)
        {
            res = ((uint32_t)d - s) & mask;
            op_write(dop, sz, (uint16_t)res);

            SET_V(0 != ((((uint32_t)d ^ s) & (~(uint32_t)s ^ res)) & msb));
            SET_C((d & mask) < (s & mask));
        }
        else
        {
            res = ((uint32_t)d + s) & mask;
            op_write(dop, sz, (uint16_t)res);

            SET_V(0 != (((~(uint32_t)d ^ s) & ((uint32_t)d ^ res)) & msb));
            SET_C(((uint32_t)(d & mask) + (s & mask)) > mask);
        }

        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        return;
    }
}

//-----------------------------------------------------------------------------
// 0050DD..0067DD and 1050DD..1067DD
static void single_operand(uint16_t op, unsigned sz)
{
    unsigned kind = (op >> 6) & 077u;
    uint32_t mask = (1 == sz) ? 0xffu : 0xffffu;
    uint32_t msb = (1 == sz) ? 0x80u : 0x8000u;
    uint32_t dop;
    uint32_t src, res;
    bool oldc;

    /*
     * 064-067 are four instructions in the word column and four completely
     * different ones in the byte column, so they are decoded before the
     * destination is resolved - MARK does not have a destination at all, and
     * resolving one would advance a register that MARK is about to overwrite.
     */
    if (kind >= 064u)
    {
        if (2 == sz)
        {
            switch (kind)
            {
            case 064:   // MARK - unwind the frame a subroutine's caller built
                SP = (uint16_t)(PC + 2 * (op & 077u));
                PC = R[5];
                R[5] = pop();
                return;

            case 067:   // SXT
                dop = operand(op & 077u, 2);
                op_write(dop, 2, GET_N() ? 0177777u : 0);
                SET_Z(!GET_N());
                SET_V(false);
                return;

#if BK_CPU_MFPX
            case 065:   // MFPI - one address space, so: read and push
                dop = operand(op & 077u, 2);
                res = (dop & OPR_REG) ? R[dop & 7] : rdw((uint16_t)dop);
                push((uint16_t)res);
                SET_N(0 != (res & 0100000u));
                SET_Z(0 == res);
                SET_V(false);
                return;

            case 066:   // MTPI - pop and write
                dop = operand(op & 077u, 2);
                res = pop();
                op_write(dop, 2, (uint16_t)res);
                SET_N(0 != (res & 0100000u));
                SET_Z(0 == res);
                SET_V(false);
                return;
#endif

            default:
                reserved_instruction();
                return;
            }
        }
        else
        {
            switch (kind)
            {
            case 064:   // MTPS - the low half of the PSW, T excepted
                dop = operand(op & 077u, 1);
                res = op_read(dop, 1);
                PSW = (uint16_t)((PSW & 0420u) | (res & 0357u));
                return;

            case 067:   // MFPS - and into a register it sign extends
                dop = operand(op & 077u, 1);
                res = PSW & 0377u;

                if (dop & OPR_REG)
                    R[dop & 7] = (uint16_t)(int16_t)(int8_t)res;
                else
                    bk_write8((uint16_t)dop, (uint8_t)res);

                SET_N(0 != (res & 0200u));
                SET_Z(0 == res);
                SET_V(false);
                return;

            default:    // 1065 MFPD, 1066 MTPD - a data space this has not got
                reserved_instruction();
                return;
            }
        }
    }

    dop = operand(op & 077u, sz);

    switch (kind)
    {
    case 050:   // CLR
        op_write(dop, sz, 0);
        PSW = (uint16_t)((PSW & ~(PSW_N | PSW_V | PSW_C)) | PSW_Z);
        return;

    case 051:   // COM
        res = ~(uint32_t)op_read(dop, sz) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(false);
        SET_C(true);
        return;

    case 052:   // INC - and note that carry is left alone
        res = ((uint32_t)op_read(dop, sz) + 1) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(res == msb);
        return;

    case 053:   // DEC
        res = ((uint32_t)op_read(dop, sz) - 1) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(res == msb - 1);
        return;

    case 054:   // NEG
        res = (0 - (uint32_t)op_read(dop, sz)) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(res == msb);
        SET_C(0 != res);
        return;

    case 055:   // ADC
        oldc = GET_C();
        res = ((uint32_t)op_read(dop, sz) + (oldc ? 1u : 0u)) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(oldc && res == msb);
        SET_C(oldc && 0 == res);
        return;

    case 056:   // SBC
        oldc = GET_C();
        res = ((uint32_t)op_read(dop, sz) - (oldc ? 1u : 0u)) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(oldc && res == msb - 1);
        SET_C(oldc && res == mask);
        return;

    case 057:   // TST
        res = op_read(dop, sz) & mask;
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_V(false);
        SET_C(false);
        return;

    case 060:   // ROR - through carry, so it is a 17 bit rotate
        src = op_read(dop, sz) & mask;
        res = ((src >> 1) | (GET_C() ? msb : 0)) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_C(0 != (src & 1));
        SET_V(GET_N() != GET_C());
        return;

    case 061:   // ROL
        src = op_read(dop, sz) & mask;
        res = ((src << 1) | (GET_C() ? 1u : 0u)) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_C(0 != (src & msb));
        SET_V(GET_N() != GET_C());
        return;

    case 062:   // ASR - arithmetic, so the sign is duplicated
        src = op_read(dop, sz) & mask;
        res = ((src >> 1) | (src & msb)) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_C(0 != (src & 1));
        SET_V(GET_N() != GET_C());
        return;

    default:    // 063 ASL
        src = op_read(dop, sz) & mask;
        res = (src << 1) & mask;
        op_write(dop, sz, (uint16_t)res);
        SET_N(0 != (res & msb));
        SET_Z(0 == res);
        SET_C(0 != (src & msb));
        SET_V(GET_N() != GET_C());
        return;
    }
}

//-----------------------------------------------------------------------------
#if BK_CPU_EIS
// 070RSS..073RSS. Not on this processor - see BK_CPU_EIS - and here because
// the same core is a PDP-11/23 with them switched on.
static void extended(uint16_t op, unsigned sub)
{
    unsigned reg = (op >> 6) & 7;
    uint32_t sop = operand(op & 077u, 2);
    int32_t src = (int16_t)op_read(sop, 2);
    int32_t acc, res;
    unsigned shift;

    CHARGE(20);

    switch (sub)
    {
    case 0:     // MUL - 16 by 16 into a register pair, or into one register
        res = (int32_t)(int16_t)R[reg] * src;

        if (0 == (reg & 1))
        {
            R[reg] = (uint16_t)((uint32_t)res >> 16);
            R[reg | 1] = (uint16_t)res;
        }
        else
        {
            R[reg] = (uint16_t)res;
        }

        SET_N(res < 0);
        SET_Z(0 == res);
        SET_V(false);
        SET_C(res < -32768 || res > 32767);
        return;

    case 1:     // DIV - 32 by 16, quotient and remainder
        if (0 != (reg & 1))
        {
            reserved_instruction();
            return;
        }

        acc = (int32_t)(((uint32_t)R[reg] << 16) | R[reg | 1]);

        if (0 == src)
        {
            SET_V(true);
            SET_C(true);
            return;
        }

        res = acc / src;

        if (res > 32767 || res < -32768)
        {
            SET_V(true);
            return;
        }

        R[reg] = (uint16_t)res;
        R[reg | 1] = (uint16_t)(acc % src);

        SET_N(res < 0);
        SET_Z(0 == res);
        SET_V(false);
        SET_C(false);
        return;

    case 2:     // ASH - shift a register by the low six bits, signed
        shift = (unsigned)src & 077u;
        acc = (int16_t)R[reg];

        if (0 == shift)
        {
            res = acc;
            SET_C(false);
        }
        else if (shift < 32)
        {
            res = acc << shift;
            SET_C(0 != (((uint32_t)acc << shift) & 0x10000u));
        }
        else
        {
            shift = 64 - shift;
            res = acc >> (shift > 15 ? 15 : shift);
            SET_C(0 != ((acc >> ((shift > 16 ? 16 : shift) - 1)) & 1));
        }

        SET_V(((acc ^ res) & 0x8000) != 0);
        R[reg] = (uint16_t)res;
        SET_N(0 != (R[reg] & 0100000u));
        SET_Z(0 == R[reg]);
        return;

    default:    // 3, ASHC - the same over a register pair
        shift = (unsigned)src & 077u;
        acc = (int32_t)(((uint32_t)R[reg] << 16) | R[reg | 1]);

        if (0 == shift)
            res = acc;
        else if (shift < 32)
            res = acc << shift;
        else
            res = acc >> (64 - shift);

        R[reg] = (uint16_t)((uint32_t)res >> 16);
        R[reg | 1] = (uint16_t)res;

        SET_N(res < 0);
        SET_Z(0 == res);
        SET_V(((acc ^ res) & 0x80000000u) != 0);
        SET_C(false);
        return;
    }
}
#endif

//-----------------------------------------------------------------------------
// 0070000-0077777: the arithmetic this processor mostly does not have, plus
// the two instructions from that block that it does - XOR and SOB.
static void eis_group(uint16_t op)
{
    unsigned sub = (op >> 9) & 7;
    unsigned reg = (op >> 6) & 7;

    switch (sub)
    {
    case 4:     // XOR
    {
        uint32_t dop = operand(op & 077u, 2);
        uint16_t res = (uint16_t)(op_read(dop, 2) ^ R[reg]);

        op_write(dop, 2, res);

        SET_N(0 != (res & 0100000u));
        SET_Z(0 == res);
        SET_V(false);
        return;
    }

    case 7:     // SOB - decrement and loop, the only backward branch that is
        R[reg] = (uint16_t)(R[reg] - 1);

        if (R[reg])
        {
            PC = (uint16_t)(PC - 2 * (op & 077u));
            CHARGE(4);
        }
        return;

    default:
#if BK_CPU_EIS
        if (sub <= 3)
        {
            extended(op, sub);
            return;
        }
#endif
        reserved_instruction();
        return;
    }
}

//-----------------------------------------------------------------------------
// 0000000-0007777
static void group_zero(uint16_t op)
{
    if (op < 0000010u)
    {
        switch (op)
        {
        case 0000000:   // HALT
            /*
             * On real hardware this drops into the 1801's halt mode, which
             * saves the state at 0177674 and runs the debug monitor. There
             * isn't one here, and a program that reaches a HALT has almost
             * always got there by accident - so it stops, with the address,
             * which is the thing worth knowing.
             */
            bk_cpu.stopped = "HALT";
            return;

        case 0000001:   // WAIT
            bk_cpu.waiting = true;
            return;

        case 0000002:   // RTI
            PC = pop();
            PSW = (uint16_t)((PSW & PSW_HALT) | (pop() & 0377u));
            return;

        case 0000003:   // BPT
#if BK_CPU_TRAPS
            g_trap = BK_VEC_TBIT;
#else
            reserved_instruction();
#endif
            return;

        case 0000004:   // IOT
#if BK_CPU_TRAPS
            g_trap = BK_VEC_IOT;
#else
            reserved_instruction();
#endif
            return;

        case 0000005:   // RESET - nothing on this bus needs telling
            return;

        case 0000006:   // RTT - like RTI, but does not trace the instruction
            PC = pop();       // it returns to
            PSW = (uint16_t)((PSW & PSW_HALT) | (pop() & 0377u));
            g_defer_tbit = true;
            return;

        default:
            reserved_instruction();
            return;
        }
    }

    if (op < 0000100u)
    {
        reserved_instruction();
        return;
    }

    if (op < 0000200u)          // JMP
    {
        uint32_t dop = operand(op & 077u, 2);

        // JMP to a register is a jump to a value, which is not a place
        if (dop & OPR_REG)
        {
            reserved_instruction();
            return;
        }

        PC = (uint16_t)dop;
        return;
    }

    if (op < 0000210u)          // RTS
    {
        unsigned reg = op & 7;

        PC = R[reg];
        R[reg] = pop();
        return;
    }

    if (op < 0000240u)          // 000210-000237, SPL among them
    {
#if BK_CPU_SPL
        if (op >= 0000230u)
            return;             // a priority this machine does not have
#endif
        reserved_instruction();
        return;
    }

    if (op < 0000300u)          // condition code set and clear, NOP included
    {
        uint16_t bits = op & 017u;

        if (op & 020u)
            PSW |= bits;
        else
            PSW &= (uint16_t)~bits;

        return;
    }

    if (op < 0000400u)          // SWAB
    {
        uint32_t dop = operand(op & 077u, 2);
        uint16_t src = op_read(dop, 2);
        uint16_t res = (uint16_t)((src >> 8) | (src << 8));

        op_write(dop, 2, res);

        // The condition codes describe the low byte after the swap, which is
        // what the instruction exists to look at
        SET_N(0 != (res & 0200u));
        SET_Z(0 == (res & 0377u));
        SET_V(false);
        SET_C(false);
        return;
    }

    if (op < 0004000u)          // the seven unsigned branches
    {
        switch (op >> 8)
        {
        case 1: branch(op, true); return;                       // BR
        case 2: branch(op, !GET_Z()); return;                    // BNE
        case 3: branch(op, GET_Z()); return;                     // BEQ
        case 4: branch(op, GET_N() == GET_V()); return;           // BGE
        case 5: branch(op, GET_N() != GET_V()); return;           // BLT
        case 6: branch(op, !GET_Z() && GET_N() == GET_V()); return; // BGT
        default: branch(op, GET_Z() || GET_N() != GET_V()); return; // BLE
        }
    }

    if (op < 0005000u)          // JSR
    {
        unsigned reg = (op >> 6) & 7;
        uint32_t dop = operand(op & 077u, 2);

        if (dop & OPR_REG)
        {
            reserved_instruction();
            return;
        }

        push(R[reg]);
        R[reg] = PC;
        PC = (uint16_t)dop;

        CHARGE(6);
        return;
    }

    if (op < 0007000u)
    {
        single_operand(op, 2);
        return;
    }

    reserved_instruction();     // 007000-007777, the CIS block
}

//-----------------------------------------------------------------------------
// 0100000-0107777
static void group_one(uint16_t op)
{
    if (op < 0104000u)          // the eight condition branches
    {
        switch ((op >> 8) & 7)
        {
        case 0: branch(op, !GET_N()); return;                   // BPL
        case 1: branch(op, GET_N()); return;                    // BMI
        case 2: branch(op, !GET_C() && !GET_Z()); return;         // BHI
        case 3: branch(op, GET_C() || GET_Z()); return;           // BLOS
        case 4: branch(op, !GET_V()); return;                   // BVC
        case 5: branch(op, GET_V()); return;                    // BVS
        case 6: branch(op, !GET_C()); return;                   // BCC
        default: branch(op, GET_C()); return;                   // BCS
        }
    }

    if (op < 0105000u)          // EMT and TRAP
    {
#if BK_CPU_TRAPS
        g_trap = (op < 0104400u) ? BK_VEC_EMT : BK_VEC_TRAP;
#else
        reserved_instruction();
#endif
        return;
    }

    if (op < 0107000u)
    {
        single_operand(op, 1);
        return;
    }

    reserved_instruction();
}

//-----------------------------------------------------------------------------
static void execute(uint16_t op)
{
    switch (op >> 12)
    {
    case 000:
        group_zero(op);
        return;

    case 007:
        eis_group(op);
        return;

    case 010:
        group_one(op);
        return;

    case 017:
        // 0170000-0177777, the floating point unit. There isn't one.
        reserved_instruction();
        return;

    default:
        double_operand(op);
        return;
    }
}
