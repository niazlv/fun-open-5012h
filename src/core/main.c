/*
 * Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
 * Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "gd32f4xx.h"
#include "hal_gpio.h"
#include "lcd.h"
#include "utils.h"
#include "flash.h"
#include "timer.h"
#include "config.h"
#include "buttons.h"
#include "battery.h"
#include "capture.h"
#include "ui.h"
#include "input.h"
#include "launcher.h"
#include "system_menu.h"
#include "debug_coredump.h"

/*- Definitions -------------------------------------------------------------*/
#define RESET_TO_DEFAULT     (BTN_SHIFT | BTN_SAVE)
#define CALIBRATION_MODE     (BTN_SHIFT | BTN_MODE)

// One flash wait state per 30 MHz of CK_SYS (2.7-3.6 V), and WSCNT is 4 bits
#define FLASH_WS_PER_HZ      30000000ul
#define FLASH_WS_RAW         ((F_CPU - 1) / FLASH_WS_PER_HZ)
#define FLASH_WAIT_STATES    (FLASH_WS_RAW > 15 ? 15 : FLASH_WS_RAW)

/*- Variables ---------------------------------------------------------------*/

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  RCU->AHB1EN |= RCU_AHB1EN_PAEN_Msk | RCU_AHB1EN_PBEN_Msk | RCU_AHB1EN_PCEN_Msk |
      RCU_AHB1EN_PDEN_Msk | RCU_AHB1EN_PEEN_Msk | RCU_AHB1EN_PFEN_Msk |
      RCU_AHB1EN_PGEN_Msk | RCU_AHB1EN_PHEN_Msk | RCU_AHB1EN_PIEN_Msk;

  // Stop execution of the program before it has a chance to configure high speed
  // clocks. This helps with programming.
  if (buttons_read() & BTN_F2)
    while (1);

  RCU->CTL_b.HXTALEN = 1;
  while (0 == RCU->CTL_b.HXTALSTB);

  // The PLL below runs the core at 250 MHz, far above the rated 168 MHz.
  // Follow the vendor procedure for >168 MHz operation: raise the LDO output
  // and enable high-drive mode before switching SYSCLK to the PLL. The waits
  // are bounded so a die that never reports ready still boots (without
  // high-drive, as before this change).
  RCU->APB1EN_b.PMUEN = 1;
  PMU->CTL_b.LDOVS = 3;

  RCU->PLL = RCU_PLL_PLLSEL_Msk | (20 << RCU_PLL_PLLPSC_Pos) | (500 << RCU_PLL_PLLN_Pos) |
      (0 << RCU_PLL_PLLP_Pos) | (15 << RCU_PLL_PLLQ_Pos);

  RCU->CTL_b.PLLEN = 1;
  while (0 == RCU->CTL_b.PLLSTB);

  PMU->CTL_b.HDEN = 1;
  for (int i = 0; i < 1000000 && 0 == PMU->CS_b.HDRF; i++);

  if (PMU->CS_b.HDRF)
  {
    PMU->CTL_b.HDS = 1;
    for (int i = 0; i < 1000000 && 0 == PMU->CS_b.HDSRF; i++);
  }

  // Flash wait states, and they have to be in place BEFORE CK_SYS speeds up.
  // At 2.7-3.6 V the flash sustains one wait state per 30 MHz of CK_SYS, so
  // 250 MHz needs 8. This was never programmed at all: WSCNT comes out of
  // reset as 0, which is only good to 30 MHz, and the part then ran a 49%
  // overclock on top of that. Code fetch mostly survived it because the
  // prefetch buffer hides sequential reads, but a cold linear sweep of flash
  // DATA is exactly the access pattern with nothing in front of it - and
  // config_init()'s find_last_entry() sweeps 128 KB of it through CRC32 on
  // every boot. One bad byte fails the CRC, the entry is rejected, and the
  // settings silently fall back to defaults.
  FMC->WS_b.WSCNT = FLASH_WAIT_STATES;
  while (FMC->WS_b.WSCNT != FLASH_WAIT_STATES);

  RCU->CFG0 = (2/*CK_PLLP*/ << RCU_CFG0_SCS_Pos) | (0/*CK_SYS*/ << RCU_CFG0_AHBPSC_Pos) |
      (5/*DIV 4*/ << RCU_CFG0_APB1PSC_Pos) | (4/*DIV 2*/ << RCU_CFG0_APB2PSC_Pos) |
      (0 << RCU_CFG0_RTCDIV_Pos);
  while (RCU->CFG0_b.SCSS != 2);
}

//-----------------------------------------------------------------------------
static void print_value(int x, int y, char *name, uint32_t value)
{
  static const char hex[] = "0123456789abcdef";
  char str[9];

  for (int i = 0; i < 8; i++)
    str[i] = hex[(value >> ((7-i)*4)) & 0xf];

  str[8] = 0;

  lcd_puts((x+1)*8, 2 + y*16, name);
  lcd_puts((x+6)*8, 2 + y*16, "= 0x");
  lcd_puts((x+10)*8, 2 + y*16, str);
}

//-----------------------------------------------------------------------------
void irq_handler_hard_fault_c(uint32_t lr, uint32_t msp, uint32_t psp)
{
  uint32_t s_r0, s_r1, s_r2, s_r3, s_r12, s_lr, s_pc, s_psr;
  uint32_t r_CFSR, r_HFSR, r_DFSR, r_AFSR, r_BFAR, r_MMAR;
  uint32_t *sp = (uint32_t *)((lr & 4) ? psp : msp);

  s_r0  = sp[0];
  s_r1  = sp[1];
  s_r2  = sp[2];
  s_r3  = sp[3];
  s_r12 = sp[4];
  s_lr  = sp[5];
  s_pc  = sp[6];
  s_psr = sp[7];

  r_CFSR = SCB->CFSR;  // Configurable Fault Status Register (MMSR, BFSR and UFSR)
  r_HFSR = SCB->HFSR;  // Hard Fault Status Register
  r_DFSR = SCB->DFSR;  // Debug Fault Status Register
  r_MMAR = SCB->MMFAR; // MemManage Fault Address Register
  r_BFAR = SCB->BFAR;  // Bus Fault Address Register
  r_AFSR = SCB->AFSR;  // Auxiliary Fault Status Register

  asm("nop"); // Setup breakpoint here

  // Capture coredump before displaying error
  debug_coredump_capture_hard_fault(lr, msp, psp);

  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
  lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
  lcd_set_font(FONT_LARGE);
  lcd_puts(120, 2, "HARD FAULT");

  print_value(0,  2, "R0",  s_r0);
  print_value(0,  3, "R1",  s_r1);
  print_value(0,  4, "R2",  s_r2);
  print_value(0,  5, "R3",  s_r3);
  print_value(0,  6, "R12", s_r12);
  print_value(0,  7, "LR",  s_lr);
  print_value(0,  8, "PC",  s_pc);
  print_value(0,  9, "PSR", s_psr);
  print_value(0, 10, "SP",  (lr & 4) ? psp : msp);
  print_value(0, 11, "MSP", msp);
  print_value(0, 12, "PSP", psp);

  print_value(20, 2, "CFSR", r_CFSR);
  print_value(20, 3, "HFSR", r_HFSR);
  print_value(20, 4, "DFSR", r_DFSR);
  print_value(20, 5, "MMAR", r_MMAR);
  print_value(20, 6, "BFAR", r_BFAR);
  print_value(20, 7, "AFSR", r_AFSR);

  // 52 glyphs is all that fits at x=8 in the 6 px font; anything longer is
  // dropped by lcd_putc, and this is the only instruction on the screen
  lcd_set_font(FONT_SMALL);
  lcd_puts(8, LCD_HEIGHT - 14,
      "Dump saved. Press any button to reboot.");

  // The dump is in retained SRAM and survives the reset, so rebooting from
  // here is what makes it readable. Halting forever, as this used to do, meant
  // every real crash was captured and then thrown away.
  // buttons_read() is a plain GPIO read: no timers, no interrupts, safe here.
  while (buttons_read());
  while (0 == buttons_read());

  NVIC_SystemReset();

  while (1);
}

//-----------------------------------------------------------------------------
__attribute__((naked)) void irq_handler_hard_fault(void)
{
  asm volatile (
    "mov    r0, lr\n"
    "mrs    r1, msp\n"
    "mrs    r2, psp\n"
    "b      irq_handler_hard_fault_c\n"
  );
}

//-----------------------------------------------------------------------------
void error(char *text)
{
  int len;

  __disable_irq();

  // Capture coredump for general errors
  COREDUMP_CAPTURE(ERROR_TYPE_GENERAL_ERROR, text);

  capture_stop();
  capture_disable_clock();
  lcd_set_backlight_level(50);

  lcd_set_font(FONT_LARGE);
  lcd_set_color(LCD_WHITE_COLOR, LCD_RED_COLOR);
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_WHITE_COLOR);

  for (len = 0; text[len]; len++);

  lcd_puts(LCD_WIDTH/2 - len*4, 112, text);

  lcd_set_font(FONT_SMALL);
  lcd_puts(8, LCD_HEIGHT - 14, "Dump saved. SHIFT+MENU: reboot to view it.");

  // A deliberate two-key combination rather than "any button": the most
  // common caller is battery_low_handler, and rebooting on a stray press
  // would just loop back into the same error.
  while (1)
  {
    if ((buttons_read() & (BTN_SHIFT | BTN_MENU)) == (BTN_SHIFT | BTN_MENU))
      NVIC_SystemReset();
  }
}

//-----------------------------------------------------------------------------
void battery_low_handler(void)
{
  error("BATTERY LOW");
}

//-----------------------------------------------------------------------------
void buttons_handler(int buttons)
{
  // Before anything is made of the press, including whether it is remapped
  // away or swallowed: a key is a key, and it is the only evidence this
  // firmware has that somebody is still in front of the screen. The press
  // itself is delivered as usual - waking the backlight is not a mode a
  // button has to be spent getting out of.
  lcd_backlight_activity();

  buttons = input_translate(buttons);

#ifdef DEBUG_F2_HALT  // Debug only, makes it easier to program things
  if (buttons & BTN_F2)
  {
    capture_disable_clock();
    while (1);
  }
#endif

  if (ui_handle_input(buttons))
    return;

  // Unconsumed MENU press opens the system menu over whatever is running
  if ((buttons & BTN_MENU) && !(buttons & BTN_REPEAT))
    system_menu_open();
}

//-----------------------------------------------------------------------------
int main(void)
{
  int buttons;

  sys_init();
  timer_init();
  lcd_init();
  crc32_init();
  flash_init();
  config_init();
  buttons_init();
  battery_init();
  capture_init();

  lcd_set_font(FONT_LARGE);
  lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
  lcd_set_backlight_level(config.lcd_bl_level);
  lcd_backlight_init();

  buttons = buttons_state();

  if ((buttons & RESET_TO_DEFAULT) == RESET_TO_DEFAULT)
    config_reset();

  ui_init();
  input_init();
  debug_coredump_init();

  // The launcher menu becomes the root of the UI stack
  launcher_start();

  // A dump left over from before the reset is the reason this boot happened,
  // so go straight to it instead of making the user find the viewer
  if (debug_coredump_ring_retained())
    launcher_start_app("CoreDump Viewer");

  while (1)
  {
    uint32_t pass_start = timer_us();

    timer_task();
    ui_task();
    shift_mode_task();
    battery_task();
    buttons_task();
    config_task();
    lcd_backlight_task(launcher_app_may_dim());

    // A pass that found nothing to do has nothing to do until the clock moves
    // either: every task above is driven by a millisecond timer or polls a
    // button behind a 20 ms debounce. So stop the core until the next
    // interrupt instead of spinning at 250 MHz between frames - measured, a
    // game leaves the loop idle for 55 to 99 % of the time it is played, and
    // this is the only part of the power budget the firmware can give back.
    //
    // Two conditions, and the point of both is that nothing which is actually
    // working ever waits on this:
    //
    //   - the application must allow it. The incremental renderers do not:
    //     for them a pass is a slice of the frame, not the whole of it (see
    //     app_desc_t::idle behind launcher_app_may_idle).
    //   - the pass must have taken less than a tick. Anything that spends
    //     longer than that in one pass - a DOOM frame, a screenful of a game -
    //     is not idle, and skips the sleep whatever it allows.
    //
    // The wake-up is the SysTick alarm at worst, and any earlier interrupt at
    // best: while the scope runs, its own DMA raises one per buffer.
    if (launcher_app_may_idle() && timer_us() - pass_start < 1000)
      timer_idle();
  }

  return 0;
}
