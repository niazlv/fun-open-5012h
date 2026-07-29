/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Disassembly.
 *
 * Not for reverse engineering a game - for the two lines under the register
 * dump when something has gone wrong, which is where a fault stops being a
 * number and becomes a fact. "Stopped at 0001234" says nothing; "0001234: MOV
 * (R2)+, -(SP)" says the program was pushing something and the stack was in
 * the wrong place.
 *
 * Octal throughout, and deliberately: this machine has been written about in
 * octal since 1985, its addresses group into threes, and its addressing modes
 * are literally the digits of the operand field. 0177664 is a register anyone
 * who has used a BK recognises on sight; 0xFFB4 is not.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bk_config.h"

#if BK_DISASM

#include "bk_disasm.h"
#include "bk_mem.h"

/*- Variables ---------------------------------------------------------------*/
static const char *const g_regs[8] =
{
    "R0", "R1", "R2", "R3", "R4", "R5", "SP", "PC"
};

// 01SSDD..06SSDD, and the same list again with a B for the byte forms
static const char *const g_double[7] =
{
    "", "MOV", "CMP", "BIT", "BIC", "BIS", "ADD"
};

// 0050DD..0063DD. The four above these are two different sets depending on
// the byte bit, and are named where they are decoded.
static const char *const g_single[12] =
{
    "CLR", "COM", "INC", "DEC", "NEG", "ADC", "SBC", "TST",
    "ROR", "ROL", "ASR", "ASL"
};

/*- Local prototypes --------------------------------------------------------*/
static char *put(char *p, const char *end, const char *s);
static char *put_octal(char *p, const char *end, uint32_t value, int digits);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static char *put(char *p, const char *end, const char *s)
{
    while (*s && p < end)
        *p++ = *s++;

    return p;
}

//-----------------------------------------------------------------------------
static char *put_octal(char *p, const char *end, uint32_t value, int digits)
{
    char tmp[8];
    int n = 0;

    do
    {
        tmp[n++] = (char)('0' + (value & 7));
        value >>= 3;
    }
    while (value && n < (int)sizeof(tmp));

    while (n < digits)
        tmp[n++] = '0';

    while (n-- > 0 && p < end)
        *p++ = tmp[n];

    return p;
}

//-----------------------------------------------------------------------------
// One operand, and the extra word it may have eaten. `pc` is the address the
// next word would come from, which is what a PC-relative operand is measured
// against; *len grows by two if a word is consumed.
static char *put_operand(char *p, const char *end, unsigned spec, uint16_t pc,
    int *len)
{
    unsigned reg = spec & 7;
    unsigned mode = (spec >> 3) & 7;
    uint16_t extra = 0;

    if (mode >= 6)
    {
        extra = bk_read16((uint16_t)(pc + *len));
        *len += 2;
    }

    switch (mode)
    {
    case 0:
        return put(p, end, g_regs[reg]);

    case 1:
        p = put(p, end, "(");
        p = put(p, end, g_regs[reg]);
        return put(p, end, ")");

    case 2:
        if (7 == reg)
        {
            // #n: the word after the instruction is the value, not a place
            extra = bk_read16((uint16_t)(pc + *len));
            *len += 2;
            p = put(p, end, "#");
            return put_octal(p, end, extra, 6);
        }
        p = put(p, end, "(");
        p = put(p, end, g_regs[reg]);
        return put(p, end, ")+");

    case 3:
        if (7 == reg)
        {
            extra = bk_read16((uint16_t)(pc + *len));
            *len += 2;
            p = put(p, end, "@#");
            return put_octal(p, end, extra, 6);
        }
        p = put(p, end, "@(");
        p = put(p, end, g_regs[reg]);
        return put(p, end, ")+");

    case 4:
        p = put(p, end, "-(");
        p = put(p, end, g_regs[reg]);
        return put(p, end, ")");

    case 5:
        p = put(p, end, "@-(");
        p = put(p, end, g_regs[reg]);
        return put(p, end, ")");

    default:
        if (7 == reg)
        {
            // Relative to the program counter, so show where it lands rather
            // than the offset - which is the only part anybody wants
            if (7 == mode)
                p = put(p, end, "@");

            return put_octal(p, end, (uint16_t)(pc + *len + extra), 6);
        }

        if (7 == mode)
            p = put(p, end, "@");

        p = put_octal(p, end, extra, 6);
        p = put(p, end, "(");
        p = put(p, end, g_regs[reg]);
        return put(p, end, ")");
    }
}

//-----------------------------------------------------------------------------
static char *put_branch(char *p, const char *end, const char *name, uint16_t op,
    uint16_t pc)
{
    p = put(p, end, name);
    p = put(p, end, "  ");

    return put_octal(p, end, (uint16_t)(pc + 2 * (int16_t)(int8_t)(op & 0377u)),
        6);
}

//-----------------------------------------------------------------------------
static const char *branch_name(uint16_t op)
{
    if (op < 0100000u)
    {
        static const char *const n[8] =
        {
            "", "BR", "BNE", "BEQ", "BGE", "BLT", "BGT", "BLE"
        };

        return n[op >> 8];
    }

    {
        static const char *const n[8] =
        {
            "BPL", "BMI", "BHI", "BLOS", "BVC", "BVS", "BCC", "BCS"
        };

        return n[(op >> 8) & 7];
    }
}

//-----------------------------------------------------------------------------
int bk_disasm(uint16_t addr, char *buf, int size)
{
    char *p = buf;
    const char *end = buf + size - 1;
    uint16_t op = bk_read16(addr);
    uint16_t pc = (uint16_t)(addr + 2);
    unsigned top = op >> 12;
    unsigned kind;
    int len = 2;

    if (size < 8)
    {
        if (size > 0)
            buf[0] = 0;

        return 2;
    }

    // 01SSDD..06SSDD and 11SSDD..16SSDD
    if ((top >= 1 && top <= 6) || (top >= 011 && top <= 016))
    {
        kind = top & 7;

        p = put(p, end, (6 == kind && top >= 010) ? "SUB" : g_double[kind]);

        if (top >= 010 && 6 != kind)
            p = put(p, end, "B");

        p = put(p, end, "  ");
        p = put_operand(p, end, (op >> 6) & 077u, pc, &len);
        p = put(p, end, ", ");
        p = put_operand(p, end, op & 077u, pc, &len);

        *p = 0;
        return len;
    }

    // The branches, in both groups
    if ((op >= 0000400u && op < 0004000u) ||
        (op >= 0100000u && op < 0104000u))
    {
        p = put_branch(p, end, branch_name(op), op, pc);
        *p = 0;
        return 2;
    }

    // 0050DD..0067DD and 1050DD..1067DD
    if ((op >= 0005000u && op < 0007000u) ||
        (op >= 0105000u && op < 0107000u))
    {
        bool byteop = 0 != (op & 0100000u);

        kind = ((op >> 6) & 077u) - 050u;

        if (kind < 12)
        {
            p = put(p, end, g_single[kind]);

            if (byteop)
                p = put(p, end, "B");
        }
        else if (byteop)
        {
            // The byte column of 064-067 is four different instructions
            static const char *const n[4] = { "MTPS", "MFPD", "MTPD", "MFPS" };

            p = put(p, end, n[kind - 12]);
        }
        else
        {
            static const char *const n[4] = { "MARK", "MFPI", "MTPI", "SXT" };

            p = put(p, end, n[kind - 12]);
        }

        p = put(p, end, "  ");

        if (!byteop && 12 == kind)
            p = put_octal(p, end, op & 077u, 2);      // MARK takes a count
        else
            p = put_operand(p, end, op & 077u, pc, &len);

        *p = 0;
        return len;
    }

    // The 07 block: XOR and SOB are the two of these that exist here
    if (070000u == (op & 0170000u))
    {
        unsigned sub = (op >> 9) & 7;
        unsigned reg = (op >> 6) & 7;

        if (4 == sub)
        {
            p = put(p, end, "XOR  ");
            p = put(p, end, g_regs[reg]);
            p = put(p, end, ", ");
            p = put_operand(p, end, op & 077u, pc, &len);
        }
        else if (7 == sub)
        {
            p = put(p, end, "SOB  ");
            p = put(p, end, g_regs[reg]);
            p = put(p, end, ", ");
            p = put_octal(p, end, (uint16_t)(pc - 2 * (op & 077u)), 6);
        }
        else
        {
            static const char *const n[8] =
            {
                "MUL", "DIV", "ASH", "ASHC", "", "FIS", "CIS", ""
            };

            p = put(p, end, n[sub]);
            p = put(p, end, "  ");
            p = put(p, end, g_regs[reg]);
            p = put(p, end, ", ");
            p = put_operand(p, end, op & 077u, pc, &len);
        }

        *p = 0;
        return len;
    }

    if (op >= 0004000u && op < 0005000u)        // JSR
    {
        p = put(p, end, "JSR  ");
        p = put(p, end, g_regs[(op >> 6) & 7]);
        p = put(p, end, ", ");
        p = put_operand(p, end, op & 077u, pc, &len);
        *p = 0;
        return len;
    }

    if (op >= 0000100u && op < 0000200u)        // JMP
    {
        p = put(p, end, "JMP  ");
        p = put_operand(p, end, op & 077u, pc, &len);
        *p = 0;
        return len;
    }

    if (op >= 0000200u && op < 0000210u)        // RTS
    {
        p = put(p, end, "RTS  ");
        p = put(p, end, g_regs[op & 7]);
        *p = 0;
        return 2;
    }

    if (op >= 0000300u && op < 0000400u)        // SWAB
    {
        p = put(p, end, "SWAB ");
        p = put_operand(p, end, op & 077u, pc, &len);
        *p = 0;
        return len;
    }

    if (op >= 0104000u && op < 0105000u)        // EMT and TRAP
    {
        p = put(p, end, (op < 0104400u) ? "EMT  " : "TRAP ");
        p = put_octal(p, end, op & 0377u, 3);
        *p = 0;
        return 2;
    }

    if (op >= 0000240u && op < 0000300u)        // the condition code group
    {
        if (0000240u == op)
        {
            p = put(p, end, "NOP");
        }
        else
        {
            static const char g_cc[4] = { 'C', 'V', 'Z', 'N' };

            p = put(p, end, (op & 020u) ? "SE" : "CL");

            for (int i = 3; i >= 0; i--)
            {
                if (op & (1u << i))
                {
                    char one[2] = { g_cc[i], 0 };

                    p = put(p, end, one);
                }
            }
        }

        *p = 0;
        return 2;
    }

    if (op < 0000010u)
    {
        static const char *const n[8] =
        {
            "HALT", "WAIT", "RTI", "BPT", "IOT", "RESET", "RTT", ".WORD 7"
        };

        p = put(p, end, n[op]);
        *p = 0;
        return 2;
    }

    p = put(p, end, ".WORD ");
    p = put_octal(p, end, op, 6);
    *p = 0;

    return 2;
}

#endif // BK_DISASM
