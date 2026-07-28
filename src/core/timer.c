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
#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "common.h"
#include "timer.h"

/*- Definitions -------------------------------------------------------------*/
#define MAX_TIMERS     32

/*
 * Timekeeping is done with TIMER1, a true 32-bit general-purpose timer on
 * APB1. main.c programs APB1PSC = DIV4, and a timer on a bus with a
 * prescaler != 1 receives 2 x PCLK, so the TIMER1 kernel clock is
 * F_CPU/2 = 125 MHz. The 16-bit prescaler divides this to a 1 MHz (1 us)
 * free-running counter.
 *
 * The counter wraps every 2^32 us ~= 71.6 minutes and all delta math on it
 * is unsigned 32-bit, so a wrap between two timer_task() calls is handled
 * transparently by modular subtraction. The hardware counter keeps running
 * while the CPU is stalled, so any stall shorter than ~71 minutes (config
 * flash sector erase ~1-2 s, long redraw, debugger halt) is absorbed with
 * zero time loss. The previous SysTick scheme killed the device with a
 * white "Timer overflow" screen after a stall of only ~0.27-0.54 s.
 *
 * No interrupts are used. TIMER0/TIMER7 belong to capture.c and TIMER2 to
 * the lcd.c backlight PWM; TIMER1 is free (TIMER4 is the other 32-bit
 * timer if TIMER1 is ever needed elsewhere).
 */
#define TIMER_CLOCK    (F_CPU / 2)
#define US_PER_SEC     1000000ul
#define US_PER_MS      1000ul

/*- Variables ---------------------------------------------------------------*/
static int *g_timer_list[MAX_TIMERS];
static int g_timer_count = 0;
static int g_timer_max_delta = 0;
static uint32_t g_timer_prev_cnt = 0;
static uint32_t g_timer_ms_counter = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void timer_init(void)
{
  g_timer_count     = 0;
  g_timer_max_delta = 0;

  RCU->APB1EN_b.TIMER1EN = 1;

  TIMER1->CTL0  = 0;
  TIMER1->PSC   = (TIMER_CLOCK / US_PER_SEC) - 1; // 125 MHz / 125 = 1 MHz tick
  TIMER1->CAR   = 0xffffffff;                     // full 32-bit period
  TIMER1->SWEVG = TIMER1_SWEVG_UPG_Msk;           // latch PSC/CAR, clear CNT
  TIMER1->CTL0  = TIMER1_CTL0_CEN_Msk;

  g_timer_prev_cnt = TIMER1->CNT;

  // SysTick is no longer used for timekeeping
  SysTick->CTRL = 0;
}

//-----------------------------------------------------------------------------
void timer_add(int *timer)
{
  for (int i = 0; i < g_timer_count; i++)
  {
    if (g_timer_list[i] == timer)
      return;
  }

  if (g_timer_count == MAX_TIMERS)
    error("Too many timers");

  g_timer_list[g_timer_count++] = timer;
}

//-----------------------------------------------------------------------------
void timer_remove(int *timer)
{
  for (int i = 0; i < g_timer_count; i++)
  {
    if (g_timer_list[i] == timer)
    {
      g_timer_list[i] = g_timer_list[g_timer_count-1];
      g_timer_count--;
      return;
    }
  }
}

//-----------------------------------------------------------------------------
int timer_get_max_delta(void)
{
  int res = g_timer_max_delta;
  g_timer_max_delta = 0;
  return res;
}

//-----------------------------------------------------------------------------
void timer_task(void)
{
  uint32_t cnt = TIMER1->CNT;
  uint32_t delta_us = cnt - g_timer_prev_cnt; // unsigned modular math: wrap-safe

  if (delta_us >= US_PER_MS)
  {
    uint32_t ms  = delta_us / US_PER_MS;
    uint32_t rem = delta_us % US_PER_MS;
    int ms_i = (int)ms; // max 2^32/1000, fits in int

    for (int i = 0; i < g_timer_count; i++)
    {
      if (*g_timer_list[i] > ms_i)
        *g_timer_list[i] -= ms_i;
      else if (*g_timer_list[i] > 0)
        *g_timer_list[i] = 0;
    }

    g_timer_prev_cnt = cnt - rem; // carry sub-ms remainder, no tick ever lost
    g_timer_ms_counter += ms;

    if (ms_i > g_timer_max_delta)
      g_timer_max_delta = ms_i;
  }
}

//-----------------------------------------------------------------------------
uint32_t timer_ms(void)
{
  return g_timer_ms_counter;
}

//-----------------------------------------------------------------------------
// The counter TIMER1 free-runs at 1 MHz and wraps every ~71 minutes; unsigned
// subtraction of two readings is correct across the wrap
uint32_t timer_us(void)
{
  return TIMER1->CNT;
}
