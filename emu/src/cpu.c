/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The Cortex-M4 core, its memory map, and ARMv7-M exception entry/return.
 *
 * Unicorn (QEMU) executes the Thumb-2 instruction stream, DSP extensions and
 * VFP included - which matters here, because this firmware's hot paths are
 * hand-written SIMD assembly (uqsub8/sel/rbit in trigger.c and buffer.c) and
 * DOOM's renderer is built for the hard-float ABI. What Unicorn does not
 * provide is the NVIC: it will run a handler, but it will not decide to enter
 * one and it cannot return from one. Both are implemented here against the
 * ARMv7-M architecture reference, so the firmware's interrupt-driven capture
 * runs the way it does on the part.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unicorn/unicorn.h>

#include "emu.h"

/*- Definitions -------------------------------------------------------------*/
/* Instructions per uc_emu_start call. Virtual time advances a slice at a
 * time, so this is also the timing granularity the guest can observe: 256
 * instructions at 250 MHz is roughly a microsecond, fine enough that the
 * firmware's busy-waits on TIMER1 and on the DMA counter behave, and coarse
 * enough that the emulator still runs at a useful fraction of real speed. */
#define SLICE_INSNS      256

/* One instruction per cycle. The real M4 is close to that for the Thumb-2
 * these hot loops are made of (single-cycle ALU, ldm/stm bursts amortised),
 * and every timebase the scope displays comes from the modeled peripherals'
 * own clocks rather than from this number. */
#define NS_PER_INSN      (1000000000.0 / (double)EMU_CPU_HZ)

/* An EXC_RETURN value in the PC is how ARMv7-M leaves a handler. Unicorn
 * reports the branch as an unhandled exception and parks the PC there. */
#define EXC_RETURN_MASK  0xFFFFFFF0u

/*- Variables ---------------------------------------------------------------*/
uint8_t *emu_flash;
uint8_t *emu_tcm;
uint8_t *emu_sram;

static uc_engine *g_uc;
static uint64_t g_insns;
static uint64_t g_time_ns;
static bool g_wedged;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint32_t reg_read(int reg)
{
  uint32_t v = 0;
  uc_reg_read(g_uc, reg, &v);
  return v;
}

//-----------------------------------------------------------------------------
static void reg_write(int reg, uint32_t v)
{
  uc_reg_write(g_uc, reg, &v);
}

//-----------------------------------------------------------------------------
uint32_t cpu_pc(void)          { return reg_read(UC_ARM_REG_PC); }
uint64_t cpu_now_ns(void)      { return g_time_ns; }
uint64_t cpu_instructions(void){ return g_insns; }

//-----------------------------------------------------------------------------
// IPSR == 0 means thread mode: no exception is active, so one can be taken.
bool cpu_in_thread_mode(void)
{
  return (reg_read(UC_ARM_REG_XPSR) & 0x1ff) == 0;
}

//-----------------------------------------------------------------------------
bool cpu_irq_masked(void)
{
  return (reg_read(UC_ARM_REG_PRIMASK) & 1) != 0;
}

//-----------------------------------------------------------------------------
// Peripheral space. Every access lands in board.c, which owns the register
// file - there is no backing RAM here, so a read the board does not model is
// visible rather than silently plausible.
static uint64_t mmio_read(uc_engine *uc, uint64_t offset, unsigned size, void *ud)
{
  (void)uc; (void)ud;
  return board_mmio_read(EMU_PERIPH_BASE + offset, size);
}

//-----------------------------------------------------------------------------
static void mmio_write(uc_engine *uc, uint64_t offset, unsigned size,
    uint64_t value, void *ud)
{
  (void)uc; (void)ud;
  board_mmio_write(EMU_PERIPH_BASE + offset, size, value);
}

//-----------------------------------------------------------------------------
static uint64_t scs_read(uc_engine *uc, uint64_t offset, unsigned size, void *ud)
{
  (void)uc; (void)ud;
  return scs_mmio_read(EMU_SCS_BASE + offset, size);
}

//-----------------------------------------------------------------------------
static void scs_write(uc_engine *uc, uint64_t offset, unsigned size,
    uint64_t value, void *ud)
{
  (void)uc; (void)ud;
  scs_mmio_write(EMU_SCS_BASE + offset, size, value);
}

//-----------------------------------------------------------------------------
// The device signature page: read-only, and the only field the firmware wants
// from it is the flash size in KB at 0x1FFF7A22 (flash_viewer.c).
static uint64_t sig_read(uc_engine *uc, uint64_t offset, unsigned size, void *ud)
{
  (void)uc; (void)ud;

  if (EMU_SIG_BASE + offset == 0x1FFF7A22u)
    return EMU_FLASH_SIZE / 1024;

  return 0;
}

//-----------------------------------------------------------------------------
static void sig_write(uc_engine *uc, uint64_t offset, unsigned size,
    uint64_t value, void *ud)
{
  (void)uc; (void)offset; (void)size; (void)value; (void)ud;
}

//-----------------------------------------------------------------------------
// Flash is mapped without write permission, which is what NOR flash looks like
// to the core: a store only takes effect through the FMC. The fault handler
// hands the store to board.c, which checks the FMC is unlocked and programming,
// and applies the AND-only semantics of a NOR cell.
static bool flash_prot_hook(uc_engine *uc, uc_mem_type type, uint64_t addr,
    int size, int64_t value, void *ud)
{
  (void)uc; (void)type; (void)ud;

  board_flash_write((uint32_t)addr, (uint64_t)value, (unsigned)size);

  return true; // handled: resume at the next instruction
}

//-----------------------------------------------------------------------------
// Anything outside the modeled map. Reported once with the PC, then tolerated,
// so a stray access shows up as a diagnostic instead of killing the session.
static bool unmapped_hook(uc_engine *uc, uc_mem_type type, uint64_t addr,
    int size, int64_t value, void *ud)
{
  static int reported = 0;

  (void)uc; (void)ud;

  if (reported < 16)
  {
    emu_log("unmapped %s at 0x%08llx (size %d, value 0x%llx) from pc 0x%08x",
        (type == UC_MEM_READ_UNMAPPED) ? "read" :
        (type == UC_MEM_WRITE_UNMAPPED) ? "write" : "fetch",
        (unsigned long long)addr, size, (unsigned long long)value, cpu_pc());
    reported++;
  }

  return false; // let it fault; the run loop reports where
}

//-----------------------------------------------------------------------------
bool cpu_init(void)
{
  uc_err err;
  uc_hook hook;

  // uc_mem_map_ptr needs page-aligned host memory, and macOS on arm64 uses
  // 16 KB pages
  if (posix_memalign((void **)&emu_flash, 16384, EMU_FLASH_SIZE) ||
      posix_memalign((void **)&emu_tcm, 16384, EMU_TCM_SIZE) ||
      posix_memalign((void **)&emu_sram, 16384, EMU_SRAM_SIZE))
    return false;

  // Erased NOR reads as ones; SRAM and TCM come up indeterminate, and zero is
  // the honest stand-in for "whatever was there"
  memset(emu_flash, 0xff, EMU_FLASH_SIZE);
  memset(emu_tcm, 0, EMU_TCM_SIZE);
  memset(emu_sram, 0, EMU_SRAM_SIZE);

  err = uc_open(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS, &g_uc);

  if (err != UC_ERR_OK)
  {
    emu_log("uc_open: %s", uc_strerror(err));
    return false;
  }

  err = uc_ctl_set_cpu_model(g_uc, UC_CPU_ARM_CORTEX_M4);

  if (err != UC_ERR_OK)
  {
    emu_log("uc_ctl_set_cpu_model: %s", uc_strerror(err));
    return false;
  }

  uc_mem_map_ptr(g_uc, EMU_FLASH_BASE, EMU_FLASH_SIZE,
      UC_PROT_READ | UC_PROT_EXEC, emu_flash);
  uc_mem_map_ptr(g_uc, EMU_TCM_BASE, EMU_TCM_SIZE, UC_PROT_ALL, emu_tcm);
  uc_mem_map_ptr(g_uc, EMU_SRAM_BASE, EMU_SRAM_SIZE, UC_PROT_ALL, emu_sram);

  err = uc_mmio_map(g_uc, EMU_PERIPH_BASE, EMU_PERIPH_SIZE,
      mmio_read, NULL, mmio_write, NULL);

  if (err != UC_ERR_OK)
  {
    emu_log("uc_mmio_map(periph): %s", uc_strerror(err));
    return false;
  }

  uc_mmio_map(g_uc, EMU_SCS_BASE, EMU_SCS_SIZE, scs_read, NULL, scs_write, NULL);
  uc_mmio_map(g_uc, EMU_SIG_BASE, EMU_SIG_SIZE, sig_read, NULL, sig_write, NULL);

  uc_hook_add(g_uc, &hook, UC_HOOK_MEM_WRITE_PROT, (void *)flash_prot_hook,
      NULL, EMU_FLASH_BASE, EMU_FLASH_BASE + EMU_FLASH_SIZE - 1);

  uc_hook_add(g_uc, &hook, UC_HOOK_MEM_UNMAPPED, (void *)unmapped_hook,
      NULL, 1, 0);

  return true;
}

//-----------------------------------------------------------------------------
void cpu_load_image(const uint8_t *image, size_t size)
{
  if (size > EMU_FLASH_SIZE)
    size = EMU_FLASH_SIZE;

  memset(emu_flash, 0xff, EMU_FLASH_SIZE);
  memcpy(emu_flash, image, size);
}

//-----------------------------------------------------------------------------
// Power-on reset: MSP and the reset vector come out of the image, exactly as
// the core fetches them from 0x08000000.
void cpu_reset(void)
{
  uint32_t sp, pc, zero = 0;

  memcpy(&sp, emu_flash + 0, 4);
  memcpy(&pc, emu_flash + 4, 4);

  reg_write(UC_ARM_REG_SP, sp);
  reg_write(UC_ARM_REG_PC, pc & ~1u);
  reg_write(UC_ARM_REG_LR, 0xFFFFFFFFu);
  reg_write(UC_ARM_REG_XPSR, 0x01000000u); // Thumb, thread mode
  uc_reg_write(g_uc, UC_ARM_REG_PRIMASK, &zero);

  g_wedged = false;

  emu_log("reset: initial MSP 0x%08x, entry 0x%08x", sp, pc);
}

//-----------------------------------------------------------------------------
// ARMv7-M exception entry (B1.5.6): stack the caller-saved registers, switch
// to handler mode with the exception number in IPSR, and branch through the
// vector table. Bit 9 of the stacked xPSR records the alignment adjustment so
// that the return below can undo it.
void cpu_enter_irq(int irqn)
{
  uint32_t frame[8];
  uint32_t sp, xpsr, handler;
  bool align;

  sp   = reg_read(UC_ARM_REG_SP);
  xpsr = reg_read(UC_ARM_REG_XPSR);

  frame[0] = reg_read(UC_ARM_REG_R0);
  frame[1] = reg_read(UC_ARM_REG_R1);
  frame[2] = reg_read(UC_ARM_REG_R2);
  frame[3] = reg_read(UC_ARM_REG_R3);
  frame[4] = reg_read(UC_ARM_REG_R12);
  frame[5] = reg_read(UC_ARM_REG_LR);
  frame[6] = reg_read(UC_ARM_REG_PC);

  align = (sp & 4) != 0;
  frame[7] = align ? (xpsr | (1u << 9)) : (xpsr & ~(1u << 9));

  sp = (sp - 32) & ~7u; // the frame is 8-byte aligned

  uc_mem_write(g_uc, sp, frame, sizeof(frame));
  reg_write(UC_ARM_REG_SP, sp);

  // Return to thread mode on the main stack, without a floating-point frame:
  // this firmware never switches to the process stack.
  reg_write(UC_ARM_REG_LR, 0xFFFFFFF9u);

  // IPSR = exception number; ICI/IT bits do not survive exception entry
  reg_write(UC_ARM_REG_XPSR, (xpsr & 0xF8000000u) | 0x01000000u |
      (uint32_t)(irqn + 16));

  uc_mem_read(g_uc, EMU_FLASH_BASE + (uint32_t)(irqn + 16) * 4, &handler, 4);

  reg_write(UC_ARM_REG_PC, handler & ~1u);
}

//-----------------------------------------------------------------------------
// ARMv7-M exception return (B1.5.8): pop the frame the entry above pushed.
static void cpu_exit_irq(void)
{
  uint32_t frame[8];
  uint32_t sp = reg_read(UC_ARM_REG_SP);

  uc_mem_read(g_uc, sp, frame, sizeof(frame));

  sp += 32;

  if (frame[7] & (1u << 9))
    sp |= 4; // undo the entry alignment

  reg_write(UC_ARM_REG_R0,  frame[0]);
  reg_write(UC_ARM_REG_R1,  frame[1]);
  reg_write(UC_ARM_REG_R2,  frame[2]);
  reg_write(UC_ARM_REG_R3,  frame[3]);
  reg_write(UC_ARM_REG_R12, frame[4]);
  reg_write(UC_ARM_REG_LR,  frame[5]);
  reg_write(UC_ARM_REG_SP,  sp);
  reg_write(UC_ARM_REG_XPSR, frame[7] & ~(1u << 9));
  reg_write(UC_ARM_REG_PC,  frame[6] & ~1u);
}

//-----------------------------------------------------------------------------
// Where the core is, in the terms that matter when it has stopped making
// progress: is it in a handler, has it masked interrupts, and where is it
// spinning.
void cpu_describe(char *buf, size_t size)
{
  uint32_t xpsr = reg_read(UC_ARM_REG_XPSR);
  uint32_t ipsr = xpsr & 0x1ff;

  snprintf(buf, size, "pc 0x%08x  sp 0x%08x  lr 0x%08x  %s  irq %s",
      reg_read(UC_ARM_REG_PC), reg_read(UC_ARM_REG_SP), reg_read(UC_ARM_REG_LR),
      ipsr ? "in handler" : "thread mode",
      (reg_read(UC_ARM_REG_PRIMASK) & 1) ? "MASKED" : "enabled");
}

//-----------------------------------------------------------------------------
bool cpu_step(void)
{
  uint32_t pc = reg_read(UC_ARM_REG_PC);
  uc_err err;

  if (g_wedged)
    return false;

  err = uc_emu_start(g_uc, pc | 1, 0, 0, SLICE_INSNS);

  g_insns += SLICE_INSNS;
  g_time_ns = (uint64_t)((double)g_insns * NS_PER_INSN);

  if (err == UC_ERR_OK)
    return true;

  // Leaving a handler: Unicorn stops on the branch to EXC_RETURN instead of
  // performing the return, so the unstacking happens here.
  pc = reg_read(UC_ARM_REG_PC);

  if (err == UC_ERR_EXCEPTION && (pc | 0xfu) == 0xFFFFFFFFu)
  {
    cpu_exit_irq();
    return true;
  }

  emu_log("core stopped: %s at pc 0x%08x lr 0x%08x sp 0x%08x r0 %08x r1 %08x r2 %08x r3 %08x",
      uc_strerror(err), pc, reg_read(UC_ARM_REG_LR), reg_read(UC_ARM_REG_SP),
      reg_read(UC_ARM_REG_R0), reg_read(UC_ARM_REG_R1),
      reg_read(UC_ARM_REG_R2), reg_read(UC_ARM_REG_R3));
  g_wedged = true;

  return false;
}
