/*
 * Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
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
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <stddef.h>
#include "gd32f4xx.h"
#include "hal_gpio.h"
#include "utils.h"
#include "timer.h"
#include "buffer.h"
#include "common.h"
#include "config.h"
#include "trigger.h"
#include "measure.h"
#include "record_window.h"
#include "capture.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(CLK_A,    A, 8)
HAL_GPIO_PIN(CLK_B,    A, 9)

HAL_GPIO_PIN(DAC,      A, 4)

HAL_GPIO_PIN(Q0,       B, 9)
HAL_GPIO_PIN(Q1,       B, 8)
HAL_GPIO_PIN(Q2,       B, 7)
HAL_GPIO_PIN(Q3,       C, 12)
HAL_GPIO_PIN(Q4,       C, 11)
HAL_GPIO_PIN(Q5,       C, 10)
HAL_GPIO_PIN(Q6,       A, 15)
HAL_GPIO_PIN(AC_DC,    C, 15)

#define AUTO_MODE_COUNT_DIV    10

#define DMA_MAX_BUFFER_SIZE    (16 * 1024)

#define STORAGE_BUFFER_SIZE    (24 * 1024)
#define STORAGE_BUFFER_RATIO   (CAPTURE_BUFFER_SIZE / STORAGE_BUFFER_SIZE)

_Static_assert(STORAGE_BUFFER_RATIO == 4, "buffer_decimate asm is hardcoded 4:1");
_Static_assert(CAPTURE_BUFFER_SIZE % DMA_MAX_BUFFER_SIZE == 0, "ring must be a multiple of the DMA buffer");
_Static_assert(CAPTURE_BUFFER_SIZE + STORAGE_BUFFER_SIZE <= 128 * 1024, "capture + storage must fit main SRAM");
_Static_assert(CAPTURE_SPARE_RAM + CAPTURE_SPARE_RAM_SIZE + COREDUMP_RAM_SIZE ==
    0x20000000u + 128 * 1024, "spare SRAM split must cover the block exactly");
_Static_assert(0x20000000u + CAPTURE_BUFFER_SIZE + STORAGE_BUFFER_SIZE == CAPTURE_SPARE_RAM,
    "spare RAM must start right after the storage buffer");

#define ZERO_POINT             0x80

// TIMER0 output modes for the two AD9288 encode clocks (see adc_init).
// 6 = PWM mode 0, 7 = PWM mode 1 (inverted). Equal modes = both converters
// sample the same instant (the ring holds every moment twice, 62.5 MS/s of
// unique data - measured: 50 MHz reads exactly |50-62.5| = 12.5 MHz);
// opposite modes interleave them 8 ns apart for the real 125 MS/s.
//
// Interleave is how this hardware is MEANT to run. From the designer's own
// reverse-engineering thread (eevblog, ataradov): "The ADC clocks are out of
// phase (when both channels are used), but the ADC itself has a data align
// setting, which makes data from both channels available on the rising edge
// of the channel A clock." The data-align feature is the whole trick: B
// encodes half a period away, but its OUTPUT is re-timed inside the ADC to
// channel A's clock, so the data bus only ever transitions on A's edges and
// one 16-bit DMA snapshot safely carries both channels.
//
// That sentence also dictates which channel may be inverted, and getting it
// wrong is what the first attempt did (2026-07-27): inverting CH0 moved
// channel A - the alignment REFERENCE - so every bus transition shifted 8 ns
// onto the DMA snapshot instant, and with the TIMER0<->TIMER7 start skew
// being whatever the two CEN writes make it, each dma_start() randomly gave
// either a clean run or metastable odd bytes (needles, ~600 mV of garbage on
// a shorted input). Channel A's clock must never move. Inverting CH1 (ADC B)
// leaves the bus timing byte-for-byte as the always-working in-phase build
// had it.
//
// Verification on hardware, 50 MHz in at 50 ns/div:
//   ~50 MHz  - interleave works, this is the real 125 MS/s
//   ~15 MHz  - works, but the pair lands in swapped time order (B later,
//              not earlier); fix is software - swap pair interpretation
//   needles  - the board does not strap data-align after all: revert to 6/6
#define ADC_A_CLOCK_MODE       6
#define ADC_B_CLOCK_MODE       7


/*- Types -------------------------------------------------------------------*/
typedef struct
{
  bool     valid;
  int      period;
  int      offset;
  int      trigger;
  int      vpos;
  int      vs_mult;
  int      size;
  uint8_t  *data;
  int      min_index;
  int      max_index;
} BufferInfo;

/*- Variables ---------------------------------------------------------------*/
// NOTE: Some variables here do not need to be volatile, but I'm keeping them
//       like this, since everything here is either interrupt driven or not
//       critical for performance.
static volatile uint8_t *g_capture_buffer = (uint8_t *)0x20000000;
static volatile int g_dma_buffer_size;
static volatile int g_trigger_mode;
static volatile int g_trigger_edge;
static volatile int g_trigger_level;
static volatile int g_trigger_offset;
static volatile int g_sample_period;
static volatile bool g_dual_channel;
static volatile int g_last_sample = 0;
static int (*g_trigger_find)(uint32_t, uint32_t) = NULL;
static volatile int g_active_buf_ptr;
static volatile int g_next_buf_ptr;
static volatile int g_trigger_ptr;
static volatile int g_count;
static volatile int g_remaining;
static volatile int g_auto_mode_count;
static volatile bool g_auto_mode_stop;
static volatile bool g_triggered;
static volatile bool g_stopped;
// Whether the storage record is a full-rate window of the ring rather than a
// 4:1 decimation of all of it (see update_storage_window)
static volatile bool g_record_full_rate;
// Placed in main SRAM right after the capture ring (not in TCM: the 64 KB TCM
// also holds all .data/.bss and the stack, and was 99.7% full with this
// buffer there, leaving 2 KB for stack+heap)
static volatile uint8_t * const g_storage_buffer =
    (uint8_t *)(0x20000000 + CAPTURE_BUFFER_SIZE);
static volatile BufferInfo g_capture_buffer_info;
static volatile BufferInfo g_storage_buffer_info;
static volatile uint32_t g_buffer_generation = 0;

/*- Prototypes --------------------------------------------------------------*/
static inline int dma_get_count(void);
static inline void dma_wait_count(uint32_t count);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void adc_init(void)
{
  HAL_GPIO_CLK_A_alt(1);
  HAL_GPIO_CLK_B_alt(1);

  RCU->AHB1EN_b.DMA1EN = 1;
  RCU->APB2EN_b.TIMER0EN = 1;
  RCU->APB2EN_b.TIMER7EN = 1;

  // The ADC is an AD9288: TWO independent 8-bit converters in one package,
  // each with its own encode clock (PA8 = ADC A, PA9 = ADC B, see
  // doc/Hardware.md). Both are clocked at 62.5 MHz here, and the only way to
  // get the 125 MS/s this firmware has always claimed is to encode them half
  // a period apart - otherwise the two converters latch the SAME instant and
  // every 16-bit DMA snapshot of GPIOD carries one moment twice.
  //
  // That is what the original setup did: identical CHxCV, identical PWM mode,
  // untouched polarity, i.e. two clocks in phase. It cost the instrument half
  // its designed sample rate (a 50 MHz input folded to exactly 12.5 MHz) and
  // put a sample-to-sample ripple on the trace, since the duplicate came from
  // the other converter with its own gain and offset.
  //
  // PWM mode 1 (7) on CH0 inverts ADC A's clock, so A encodes 8 ns AFTER B.
  // The DMA reads GPIOD little-endian, so the low byte (ADC B) lands on even
  // ring addresses and the high byte (ADC A) on odd ones - B earlier, A later,
  // which is exactly the ring's own byte order. If the pipeline latency turns
  // out to land the other way the record reads ~15 MHz instead of ~50 for a
  // 50 MHz input; then invert CH1 rather than CH0.
  TIMER0->PSC    = 1;
  TIMER0->CAR    = 1;
  TIMER0->CNT    = 0;
  TIMER0->CH0CV  = 1;
  TIMER0->CH1CV  = 1;
  TIMER0->CHCTL2 = TIMER0_CHCTL2_CH0EN_Msk | TIMER0_CHCTL2_CH1EN_Msk;
  TIMER0->CTL1   = TIMER0_CTL1_ISO0_Msk | TIMER0_CTL1_ISO1_Msk;
  TIMER0->CCHP   = TIMER0_CCHP_POEN_Msk;
  TIMER0->CHCTL0_Output =
      (ADC_A_CLOCK_MODE << TIMER0_CHCTL0_Output_CH0COMCTL_Pos) |
      (ADC_B_CLOCK_MODE << TIMER0_CHCTL0_Output_CH1COMCTL_Pos);

  TIMER7->PSC      = 1;
  TIMER7->CAR      = 1;
  TIMER7->CNT      = 0;
  TIMER7->CH0CV    = 1;
  TIMER7->CHCTL2   = TIMER7_CHCTL2_CH0EN_Msk;
  TIMER7->DMAINTEN = TIMER7_DMAINTEN_CH0DEN_Msk;

  DMA1->CH2PADDR = (uint32_t)&GPIOD->ISTAT;
}

//-----------------------------------------------------------------------------
static void dac_init(void)
{
  HAL_GPIO_DAC_analog();
  RCU->APB1EN_b.DACEN = 1;
  DAC->CTL = DAC_CTL_DEN0_Msk;
  DAC->DAC0_R12DH = 2048;
}

//-----------------------------------------------------------------------------
static void dac_write(int value)
{
  if (value < 0)
    value = 0;
  else if (value > 4095)
    value = 4095;

  DAC->DAC0_R12DH = value;
}

//-----------------------------------------------------------------------------
static void vertical_scale_init(void)
{
  HAL_GPIO_Q0_out();
  HAL_GPIO_Q0_set();

  HAL_GPIO_Q1_out();
  HAL_GPIO_Q1_set();

  HAL_GPIO_Q2_out();
  HAL_GPIO_Q2_set();

  HAL_GPIO_Q3_out();
  HAL_GPIO_Q3_set();

  HAL_GPIO_Q4_out();
  HAL_GPIO_Q4_set();

  HAL_GPIO_Q5_out();
  HAL_GPIO_Q5_set();

  HAL_GPIO_Q6_out();
  HAL_GPIO_Q6_set();

  HAL_GPIO_AC_DC_out();
  HAL_GPIO_AC_DC_set();
}

//-----------------------------------------------------------------------------
static void set_vertical_scale(void)
{
  static int set_scale = -1;
  int scale = config.vertical_scale;

  if (VS_50_mV == scale)
    scale = VS_100_mV;

  if (scale == set_scale)
    return;

  set_scale = scale;

  HAL_GPIO_Q0_set();
  HAL_GPIO_Q1_set();
  HAL_GPIO_Q2_set();
  HAL_GPIO_Q3_set();
  HAL_GPIO_Q4_set();
  HAL_GPIO_Q5_set();
  HAL_GPIO_Q6_set();

  if (VS_100_mV == scale)
    HAL_GPIO_Q3_clr();
  else if (VS_200_mV == scale)
    HAL_GPIO_Q2_clr();
  else if (VS_500_mV == scale)
    HAL_GPIO_Q0_clr();
  else if (VS_1_V == scale)
    HAL_GPIO_Q1_clr();
  else if (VS_2_V == scale)
    HAL_GPIO_Q6_clr();
  else if (VS_5_V == scale)
    HAL_GPIO_Q5_clr();
  else if (VS_10_V ==scale)
    HAL_GPIO_Q4_clr();
  else
    while (1);
}

//-----------------------------------------------------------------------------
static void set_ac_coupling(void)
{
  HAL_GPIO_AC_DC_write(!config.ac_coupling);
}

//-----------------------------------------------------------------------------
void capture_init(void)
{
  adc_init();
  dac_init();
  vertical_scale_init();

  g_triggered      = false;
  g_stopped        = true;
  g_dual_channel   = false;
  g_trigger_offset = CAPTURE_BUFFER_SIZE / 2;

  capture_set_trigger_edge(TRIGGER_EDGE_RISE);
  capture_set_trigger_mode(TRIGGER_MODE_AUTO);

  g_capture_buffer_info.valid = false;
  g_capture_buffer_info.size  = CAPTURE_BUFFER_SIZE;
  g_capture_buffer_info.data  = (uint8_t *)g_capture_buffer;

  g_storage_buffer_info.valid = false;
  g_storage_buffer_info.size  = STORAGE_BUFFER_SIZE;
  g_storage_buffer_info.data  = (uint8_t *)g_storage_buffer;
}

//-----------------------------------------------------------------------------
void capture_disable_clock(void)
{
  TIMER0->CTL0 = 0;
  TIMER7->CTL0 = 0;
}

//-----------------------------------------------------------------------------
static bool check_trigger_condition(int prev, int new)
{
  if (TRIGGER_EDGE_RISE == g_trigger_edge)
  {
    return (prev < g_trigger_level && new > g_trigger_level);
  }
  else if (TRIGGER_EDGE_FALL == g_trigger_edge)
  {
    return (prev > g_trigger_level && new < g_trigger_level);
  }
  else
  {
    return (prev < g_trigger_level && new > g_trigger_level) ||
           (prev > g_trigger_level && new < g_trigger_level);
  }
}

//-----------------------------------------------------------------------------
static void update_capture_buffer(void)
{
  g_capture_buffer_info.offset = g_next_buf_ptr - dma_get_count();

  if (g_capture_buffer_info.offset < 0)
    g_capture_buffer_info.offset += CAPTURE_BUFFER_SIZE;
  else if (g_capture_buffer_info.offset >= CAPTURE_BUFFER_SIZE)
    g_capture_buffer_info.offset -= CAPTURE_BUFFER_SIZE;

  if (g_auto_mode_stop)
    g_trigger_ptr = (g_capture_buffer_info.offset + g_trigger_offset) % CAPTURE_BUFFER_SIZE;

  g_capture_buffer_info.trigger = g_trigger_ptr;
  g_capture_buffer_info.valid   = true;
}

//-----------------------------------------------------------------------------
static void copy_words(uint32_t dst, uint32_t src, int count)
{
  uint32_t *d = (uint32_t *)dst;
  const uint32_t *s = (const uint32_t *)src;

  for (int i = count / 4; i > 0; i--)
    *d++ = *s++;
}

//-----------------------------------------------------------------------------
// Trigger-relative sample indices, at `period` per sample, that the display
// will ask capture_get_data() for at the current timebase and pan position.
// A couple of samples of slack on each end covers the rounding there and the
// interpolation between index and index+1.
static void display_sample_range(int period, int *lo, int *hi)
{
  int64_t half = (int64_t)config.horizontal_period * (DATA_BUFFER_SIZE/2 + 2);

  *lo = (int)((config.horizontal_position - half) / period) - 2;
  *hi = (int)((config.horizontal_position + half) / period) + 2;

  // Every consumer indexes the record relative to the trigger, so it has to
  // be inside - and strictly, since trigger == offset reads as "one past the
  // newest sample" to the ring arithmetic in capture_get_data()
  if (*lo > -1)
    *lo = -1;

  if (*hi < 1)
    *hi = 1;
}

//-----------------------------------------------------------------------------
// Does a full-rate window cover what the screen is asking for?
static bool record_window_fits(int period)
{
  int lo, hi;

  display_sample_range(period, &lo, &hi);

  return ((int64_t)hi - lo + 1) <= STORAGE_BUFFER_SIZE;
}

//-----------------------------------------------------------------------------
// Full-rate record: copy a WINDOW of the ring instead of decimating all of it.
//
// The storage buffer exists because the ring is being overwritten by the DMA;
// consumers need a stable snapshot. Making that snapshot a 4:1 decimation of
// the whole 96 KB record costs a factor of four in sample rate, and that rate
// - not the ring's - is what the display, the measurements, the FFT and the
// decoders actually see while acquisition runs: 31.25 MS/s at the top of the
// timebase, a 15.6 MHz Nyquist limit, and a 50 MHz input folding down to a
// convincing ~12 MHz trace.
//
// At fast timebases the whole record is not needed - at 50 ns/div the visible
// window is 600 ns, i.e. 75 samples - so 24576 consecutive samples at the RAW
// rate cover the screen with the full 125 MS/s and a 62.5 MHz Nyquist limit.
// Past ~16 us/div the window no longer spans the screen and the decimated
// whole-record path below takes over again.
static bool update_storage_window(void)
{
  const int size = STORAGE_BUFFER_SIZE;
  RecordWindow w;
  int lo, hi, first;

  display_sample_range(g_sample_period, &lo, &hi);

  if (!record_window_place(CAPTURE_BUFFER_SIZE, g_capture_buffer_info.offset,
      g_capture_buffer_info.trigger, size, lo, hi, &w))
    return false;

  first = CAPTURE_BUFFER_SIZE - w.src;

  if (first > size)
    first = size;

  copy_words((uint32_t)g_storage_buffer, (uint32_t)g_capture_buffer + w.src, first);

  if (first < size)
    copy_words((uint32_t)g_storage_buffer + first, (uint32_t)g_capture_buffer, size - first);

  // Consecutive samples means BOTH interleaved ADCs, and the odd one is wired
  // bit-reversed - which is exactly what the decimated path sidestepped by
  // only ever picking even ring bytes, and exactly why it was stuck at one
  // 62.5 MS/s phase. dma_finish() un-reverses the ring itself only after this
  // runs, so the source here is always still raw.
  if (g_dual_channel)
    buffer_reverse((uint32_t)g_storage_buffer, size);

  // A linear window is just a ring whose oldest sample sits at index 0
  g_storage_buffer_info.period  = g_sample_period;
  g_storage_buffer_info.offset  = 0;
  g_storage_buffer_info.trigger = -w.start;

  return true;
}

//-----------------------------------------------------------------------------
static void update_storage_buffer(void)
{
  int offset = g_capture_buffer_info.trigger % STORAGE_BUFFER_RATIO;

  if (g_storage_buffer_info.valid)
    return;

  if (!g_record_full_rate || !update_storage_window())
  {
    // NOTE: Given the way dual channel trigger event search is implemented,
    //       the reverse version will never be called. I'm still keeping it
    //       here just in case things change in the future.
    if (g_dual_channel && (offset & 1) == 1)
      buffer_decimate_reverse((uint32_t)g_storage_buffer, (uint32_t)g_capture_buffer, CAPTURE_BUFFER_SIZE, offset);
    else
      buffer_decimate((uint32_t)g_storage_buffer, (uint32_t)g_capture_buffer, CAPTURE_BUFFER_SIZE, offset);

    g_storage_buffer_info.offset = g_capture_buffer_info.offset / STORAGE_BUFFER_RATIO;

    if (offset < (g_capture_buffer_info.offset % STORAGE_BUFFER_RATIO))
      g_storage_buffer_info.offset++;

    g_storage_buffer_info.period  = g_sample_period * STORAGE_BUFFER_RATIO;
    g_storage_buffer_info.trigger = g_capture_buffer_info.trigger / STORAGE_BUFFER_RATIO;
  }

  g_storage_buffer_info.vpos    = g_capture_buffer_info.vpos;
  g_storage_buffer_info.vs_mult = g_capture_buffer_info.vs_mult;
  g_storage_buffer_info.valid   = true;
}

//-----------------------------------------------------------------------------
static inline void dma_start(void)
{
  DMA1->CH2CTL_b.MBS = 0;

  DMA1->CH2M0ADDR = (uint32_t)g_capture_buffer;
  DMA1->CH2M1ADDR = (uint32_t)g_capture_buffer + g_dma_buffer_size;

  if (g_dual_channel)
  {
    DMA1->CH2CTL = (0/*P2M*/ << DMA1_CH2CTL_TM_Pos) | DMA1_CH2CTL_SBMEN_Msk |
        (1/*16-bit*/ << DMA1_CH2CTL_PWIDTH_Pos) | (1/*16-bit*/ << DMA1_CH2CTL_MWIDTH_Pos) |
        DMA1_CH2CTL_MNAGA_Msk | (3/*UltraHigh*/ << DMA1_CH2CTL_PRIO_Pos) |
        (7/*TIMER7_CH0*/ << DMA1_CH2CTL_PERIEN_Pos) | DMA1_CH2CTL_FTFIE_Msk;

    DMA1->CH2CNT = g_dma_buffer_size / 2;
  }
  else
  {
    DMA1->CH2CTL = (0/*P2M*/ << DMA1_CH2CTL_TM_Pos) | DMA1_CH2CTL_SBMEN_Msk |
        (0/*8-bit*/ << DMA1_CH2CTL_PWIDTH_Pos) | (0/*8-bit*/ << DMA1_CH2CTL_MWIDTH_Pos) |
        DMA1_CH2CTL_MNAGA_Msk | (3/*UltraHigh*/ << DMA1_CH2CTL_PRIO_Pos) |
        (7/*TIMER7_CH0*/ << DMA1_CH2CTL_PERIEN_Pos) | DMA1_CH2CTL_FTFIE_Msk;

    DMA1->CH2CNT = g_dma_buffer_size;
  }

  g_active_buf_ptr = 0;
  g_next_buf_ptr   = g_dma_buffer_size * 2;
  g_trigger_ptr    = 0;
  g_count          = 0;
  g_remaining      = 0;
  g_triggered      = false;
  g_auto_mode_stop = false;

  g_capture_buffer_info.period  = g_sample_period;
  g_capture_buffer_info.vpos    = config.vertical_position_mv;
  g_capture_buffer_info.vs_mult = config.calib_vs_mult[config.vertical_scale];
  g_capture_buffer_info.valid   = false;

  DMA1->INTC0 = DMA1_INTC0_FTFIFC2_Msk;
  NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  DMA1->CH2CTL_b.CHEN = 1;
}

//-----------------------------------------------------------------------------
static inline void dma_stop(void)
{
  NVIC_DisableIRQ(DMA1_Channel2_IRQn);

  DMA1->CH2CTL_b.CHEN = 0;
  while (DMA1->CH2CTL_b.CHEN);

  DMA1->INTC0 = DMA1_INTC0_FTFIFC2_Msk;
  NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
}

//-----------------------------------------------------------------------------
static inline int dma_get_count(void)
{
  if (g_dual_channel)
    return DMA1->CH2CNT * 2;
  else
    return DMA1->CH2CNT;
}

//-----------------------------------------------------------------------------
static inline void dma_wait_count(uint32_t count)
{
  if (g_dual_channel)
    count /= 2;

  while (DMA1->CH2CNT > count);
}

//-----------------------------------------------------------------------------
static inline void dma_finish(void)
{
  dma_stop();
  update_capture_buffer();
  update_storage_buffer();
  g_buffer_generation++;

  if (TRIGGER_MODE_SINGLE == g_trigger_mode)
  {
    if (g_dual_channel)
      buffer_reverse((uint32_t)g_capture_buffer, CAPTURE_BUFFER_SIZE);

    g_stopped = true;
  }
  else
  {
    dma_start();
  }
}

//-----------------------------------------------------------------------------
void irq_handler_dma1_channel2(void)
{
  uint8_t *active_buffer = (uint8_t *)g_capture_buffer + g_active_buf_ptr;

  DMA1->INTC0 = DMA1_INTC0_FTFIFC2_Msk;

  if (DMA1->CH2CTL_b.MBS)
    DMA1->CH2M0ADDR = (uint32_t)g_capture_buffer + g_next_buf_ptr;
  else
    DMA1->CH2M1ADDR = (uint32_t)g_capture_buffer + g_next_buf_ptr;

  if (g_triggered)
  {
    if (g_remaining >= g_dma_buffer_size)
    {
      g_remaining -= g_dma_buffer_size;
    }
    else
    {
      dma_wait_count(g_dma_buffer_size - g_remaining);
      dma_finish();
      return;
    }
  }

  else if (g_count < g_trigger_offset)
  {
    g_count += g_dma_buffer_size;
  }

  else
  {
    int trigger;

    if (check_trigger_condition(g_last_sample, active_buffer[0]))
      trigger = g_dma_buffer_size;
    else
      trigger = g_trigger_find((uint32_t)active_buffer, g_dma_buffer_size);

    if (trigger > 0)
    {
      g_triggered = true;
      g_trigger_ptr = g_active_buf_ptr + (g_dma_buffer_size - trigger);
      g_remaining = (CAPTURE_BUFFER_SIZE - g_trigger_offset) - trigger;

      if (g_remaining < 0)
      {
        dma_finish();
        return;
      }
      else if (g_remaining < g_dma_buffer_size)
      {
        dma_wait_count(g_dma_buffer_size - g_remaining);
        dma_finish();
        return;
      }
      else
      {
        g_remaining -= g_dma_buffer_size;
      }
    }
    else if (TRIGGER_MODE_AUTO == g_trigger_mode)
    {
      g_count += g_dma_buffer_size;

      if (g_count > g_auto_mode_count)
      {
        g_auto_mode_stop = true;
        dma_finish();
        return;
      }
    }
  }

  g_last_sample    = active_buffer[g_dma_buffer_size - (g_dual_channel ? 2 : 1)];
  g_next_buf_ptr   = (g_next_buf_ptr + g_dma_buffer_size) % CAPTURE_BUFFER_SIZE;
  g_active_buf_ptr = (g_active_buf_ptr + g_dma_buffer_size) % CAPTURE_BUFFER_SIZE;
}

//-----------------------------------------------------------------------------
void capture_start(void)
{
  if (!g_stopped)
    return;

  // capture_stop() and dma_finish() un-reverse the ring IN PLACE so a stopped
  // record reads straight. That used to be harmless because a running scope
  // only ever read even ring bytes, which the reversal never touches — but
  // update_storage_window() copies BOTH parities and reverses the odd ones,
  // so any byte the DMA has not overwritten yet would come back
  // double-reversed, i.e. raw garbage on every other sample. Put the wiring
  // back before the DMA starts writing raw data over it again.
  if (g_dual_channel)
    buffer_unreverse((uint32_t)g_capture_buffer, CAPTURE_BUFFER_SIZE);

  g_stopped = false;

  dma_start();
}

//-----------------------------------------------------------------------------
void capture_stop(void)
{
  if (g_stopped)
    return;

  dma_stop();

  // A stopped scope reads the RAW ring instead of the decimated storage
  // record (snapshot_active_info), and in dual-channel mode every odd byte
  // of that ring is still wired bit-reversed: the display, the decoders and
  // the spectrum would all analyze garbage on alternate samples. Undo the
  // interleave once, here, exactly like the single-shot stop in dma_finish()
  // already does.
  if (g_dual_channel)
    buffer_reverse((uint32_t)g_capture_buffer, CAPTURE_BUFFER_SIZE);

  g_stopped = true;
}

//-----------------------------------------------------------------------------
static void update_trigger_handler(void)
{
  if (g_dual_channel)
  {
    if (TRIGGER_EDGE_RISE == g_trigger_edge)
      g_trigger_find = trigger_find_rise_dual;
    else if (TRIGGER_EDGE_FALL == g_trigger_edge)
      g_trigger_find = trigger_find_fall_dual;
    else
      g_trigger_find = trigger_find_both_dual;
  }
  else
  {
    if (TRIGGER_EDGE_RISE == g_trigger_edge)
      g_trigger_find = trigger_find_rise_single;
    else if (TRIGGER_EDGE_FALL == g_trigger_edge)
      g_trigger_find = trigger_find_fall_single;
    else
      g_trigger_find = trigger_find_both_single;
  }
}

//-----------------------------------------------------------------------------
void capture_set_vertical_parameters(void)
{
  int offset = (config.vertical_position * config.calib_dac_mult[config.vertical_scale]) / 1024;

  dma_stop();

  set_ac_coupling();
  dac_write(config.calib_dac_zero + offset);
  set_vertical_scale();

  if (!g_stopped)
    dma_start();
}

//-----------------------------------------------------------------------------
void capture_set_horizontal_parameters(int sr_divider, int trigger_offset)
{
  int divider, dma_divider;

  dma_stop();

  if (sr_divider < 1)
  {
    divider = 1;
    g_dual_channel = true;
  }
  else
  {
    divider = (1 << sr_divider) - 1;
    g_dual_channel = false;
  }

  dma_divider = (sr_divider < 6) ? 1 : (1 << (sr_divider - 6));

  g_dma_buffer_size = DMA_MAX_BUFFER_SIZE / dma_divider;

  // The trigger/decimate asm loops advance 32 bytes per iteration and wrap
  // through 2^32 if the size is not a multiple of 32 (permanent hang in IRQ)
  if (g_dma_buffer_size < 32)
    g_dma_buffer_size = 32;

  TIMER0->CTL0 = 0;
  TIMER7->CTL0 = 0;

  TIMER0->PSC = (divider > 63) ? 63 : divider;
  TIMER7->PSC = divider;

  TIMER0->CNT = 0;
  TIMER7->CNT = 0;

  TIMER0->CTL0 = TIMER0_CTL0_CEN_Msk;
  TIMER7->CTL0 = TIMER7_CTL0_CEN_Msk;

  g_trigger_offset  = trigger_offset;
  g_sample_period   = BASE_SAMPLE_PERIOD * (1 << sr_divider);
  g_auto_mode_count = (BASE_SAMPLE_RATE / (1 << sr_divider)) / AUTO_MODE_COUNT_DIV;

  // Decide the record mode here rather than per acquisition: it only depends
  // on the timebase and the pan, and capture_get_record_period() has to be
  // able to answer before the first frame under the new settings lands
  g_record_full_rate = record_window_fits(g_sample_period);
  g_storage_buffer_info.period = g_record_full_rate ?
      g_sample_period : (g_sample_period * STORAGE_BUFFER_RATIO);

  if (g_auto_mode_count < CAPTURE_BUFFER_SIZE)
    g_auto_mode_count = CAPTURE_BUFFER_SIZE;

  update_trigger_handler();

  if (!g_stopped)
    dma_start();
}

//-----------------------------------------------------------------------------
void capture_set_trigger_level(int level)
{
  g_trigger_level = (level * CALIB_MULTIPLIER) / config.calib_vs_mult[config.vertical_scale] + ZERO_POINT;

  if (g_trigger_level < 20)
    g_trigger_level = 20;
  else if (g_trigger_level > 235)
    g_trigger_level = 235;

  trigger_set_levels(g_trigger_level);
}

//-----------------------------------------------------------------------------
void capture_set_trigger_edge(int edge)
{
  dma_stop();

  g_trigger_edge = edge;

  update_trigger_handler();

  if (!g_stopped)
    dma_start();
}

//-----------------------------------------------------------------------------
void capture_set_trigger_mode(int mode)
{
  dma_stop();

  g_trigger_mode = mode;

  if (!g_stopped)
    dma_start();
}

//-----------------------------------------------------------------------------
int capture_get_state(void)
{
  if (g_stopped)
    return CAPTURE_STATE_STOP;
  else if (g_triggered)
    return CAPTURE_STATE_TRIG;
  else
   return CAPTURE_STATE_WAIT;
}

//-----------------------------------------------------------------------------
bool capture_buffer_updated(void)
{
  return g_storage_buffer_info.valid;
}

//-----------------------------------------------------------------------------
// Hand the storage buffer back to the DMA IRQ without reading it through
// capture_get_data()/capture_read_samples(): update_storage_buffer() refills
// it only while valid is false, so a consumer that analyzes the record
// in place (capture_get_record) must still release the frame or acquisition
// stops at the one it is looking at.
void capture_consume_frame(void)
{
  g_storage_buffer_info.valid = false;
}

//---------------------------------------------------------------------
static void find_min_max_buf_aligned(uint32_t buf, int size, int *vmin, int *vmax)
{
  uint32_t min = *vmin;
  uint32_t max = *vmax;

  asm volatile (R"asm(
    t          .req r3
    b0         .req r5
    b1         .req r6

    bfi        %[min], %[min], #8, #8
    bfi        %[min], %[min], #16, #8
    bfi        %[min], %[min], #24, #8

    bfi        %[max], %[max], #8, #8
    bfi        %[max], %[max], #16, #8
    bfi        %[max], %[max], #24, #8

0:
    ldm        %[buf]!, { b0, b1 }

    usub8      t, %[min], b0
    sel        %[min], b0, %[min]
    usub8      t, %[min], b1
    sel        %[min], b1, %[min]

    usub8      t, %[max], b0
    sel        %[max], %[max], b0
    usub8      t, %[max], b1
    sel        %[max], %[max], b1

    subs       %[size], #8
    bne        0b

    // Finalize min value
    ubfx       b0, %[min], #16, #16
    usub8      t, %[min], b0
    sel        %[min], b0, %[min]

    ubfx       b0, %[min], #8, #8
    usub8      t, %[min], b0
    sel        %[min], b0, %[min]

    and        %[min], #0xff

    // Finalize max value
    ubfx       b0, %[max], #16, #16
    usub8      t, %[max], b0
    sel        %[max], %[max], b0

    ubfx       b0, %[max], #8, #8
    usub8      t, %[max], b0
    sel        %[max], %[max], b0

    and        %[max], #0xff
99:
    .unreq     t
    .unreq     b0
    .unreq     b1
    )asm"
    : [buf] "+r" (buf), [size] "+r" (size), [min] "+r" (min), [max] "+r" (max)
    : /* none */
    : "r3", "r5", "r6"
  );

  *vmin = min;
  *vmax = max;
}

//---------------------------------------------------------------------
static void find_min_max_buf(uint8_t *data, int size, int *vmin, int *vmax)
{
  int max = *vmax;
  int min = *vmin;

  while (size > 0 && ((uint32_t)data & 3))
  {
    int v = *data;

    if (v > max)
      max = v;

    if (v < min)
      min = v;

    size--;
    data++;
  }

  if (size >= 8)
  {
    int sz = size & ~7;

    find_min_max_buf_aligned((uint32_t)data, sz, &min, &max);

    data += sz;
    size -= sz;
  }

  while (size > 0)
  {
    int v = *data;

    if (v > max)
      max = v;

    if (v < min)
      min = v;

    size--;
    data++;
  }

  *vmax = max;
  *vmin = min;
}

//---------------------------------------------------------------------
static int clamp_index(BufferInfo *info, int index)
{
  if (index < info->min_index)
    index = info->min_index;

  if (index > info->max_index)
    index = info->max_index;

  index = info->trigger + index;

  if (index < 0)
    index += info->size;
  else if (index >= info->size)
    index -= info->size;

  return index;
}

//---------------------------------------------------------------------
static bool find_min_max(BufferInfo *info, int index0, int index1, int *vmin, int *vmax)
{
  if (index0 > info->max_index || index1 < info->min_index)
    return false;

  index0 = clamp_index(info, index0);
  index1 = clamp_index(info, index1);

  if (index0 < index1)
  {
    find_min_max_buf(&info->data[index0], index1 - index0, vmin, vmax);
  }
  else
  {
    find_min_max_buf(&info->data[index0], info->size - index0, vmin, vmax);
    find_min_max_buf(&info->data[0], index1, vmin, vmax);
  }

  return true;
}

//---------------------------------------------------------------------
// Snapshot the metadata of the active buffer with the DMA IRQ masked:
// dma_finish() rewrites these fields (and the storage buffer) from interrupt
// context, and the consumers below also run from settings-change paths
// while acquisition is live
static BufferInfo snapshot_active_info(void)
{
  BufferInfo local;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  if (g_stopped && g_capture_buffer_info.valid)
    local = *(BufferInfo *)&g_capture_buffer_info;
  else
    local = *(BufferInfo *)&g_storage_buffer_info;

  __set_PRIMASK(primask);

  return local;
}

//---------------------------------------------------------------------
// Full-record measurements (see measure.c), cached per acquisition: the
// record only changes when a new frame lands, so pan/zoom redraws and the
// periodic measurement display reuse the cached scan.
static bool measure_active_buffer_ex(Measure *out, bool fresh)
{
  static Measure cache;
  static uint32_t cache_gen = 0xffffffff;
  static uint32_t cache_ms = 0;
  static int cache_period = -1;
  BufferInfo local;
  uint32_t gen;

  if (g_buffer_generation == 0 && !(g_stopped && g_capture_buffer_info.valid))
    return false; // nothing has ever been captured

  gen = g_buffer_generation;
  local = snapshot_active_info();

  // At fast timebases acquisitions land far more often than anyone can
  // read numbers: cap the full-record rescan at 10 Hz — except for control
  // loops (fresh=true), which must see the latest acquisition or they end
  // up steering on data from before their own settings change
  if ((gen != cache_gen && (fresh || timer_ms() - cache_ms >= 100)) ||
      local.period != cache_period)
  {
    cache_ms = timer_ms();
    measure_run(local.data, local.size, local.offset, local.period, ZERO_POINT, &cache);
    cache_gen = gen;
    cache_period = local.period;
  }

  *out = cache;

  return true;
}

//---------------------------------------------------------------------
static bool get_measurements_ex(ScopeMeasure *out, bool fresh)
{
  Measure m;
  int64_t mult;

  if (!measure_active_buffer_ex(&m, fresh))
    return false;

  mult = config.calib_vs_mult[config.vertical_scale];

  out->vmin_raw  = m.vmin;
  out->vmax_raw  = m.vmax;
  // Spike-trimmed peaks, not percentiles: control loops must still see a
  // short burst (a us-scale packet in a ms-scale record) that a percentile
  // trim erases completely
  out->base_raw  = m.pk_lo;
  out->top_raw   = m.pk_hi;
  out->vpp_mv    = (int)((int64_t)(m.vmax - m.vmin) * mult / CALIB_MULTIPLIER);
  out->vamp_mv   = (int)((int64_t)(m.pk_hi - m.pk_lo) * mult / CALIB_MULTIPLIER);
  out->vrms_mv   = (int)((int64_t)m.rms_c100 * mult / CALIB_MULTIPLIER / 100);
  out->vavg_mv   = (int)((int64_t)m.mean_c100 * mult / CALIB_MULTIPLIER / 100);
  // Mid level between the spike-trimmed peaks: works for continuous signals
  // (equals the rail midpoint) and for bursts (percentiles would put the
  // "midpoint" at the baseline)
  out->vmid_mv   = (int)((int64_t)((m.pk_hi + m.pk_lo) / 2 - ZERO_POINT) * mult / CALIB_MULTIPLIER);
  out->frequency = m.frequency;
  out->duty_x10  = m.duty_x10;
  out->periods   = m.periods;
  out->period_good_pct = m.period_good_pct;
  out->level_pct = m.level_pct;

  return true;
}

//---------------------------------------------------------------------
bool capture_get_measurements(ScopeMeasure *out)
{
  return get_measurements_ex(out, false);
}

//---------------------------------------------------------------------
bool capture_get_measurements_fresh(ScopeMeasure *out)
{
  return get_measurements_ex(out, true);
}

//---------------------------------------------------------------------
bool capture_get_raw_measure(Measure *out)
{
  return measure_active_buffer_ex(out, false);
}

//---------------------------------------------------------------------
bool capture_get_raw_measure_fresh(Measure *out)
{
  return measure_active_buffer_ex(out, true);
}

//---------------------------------------------------------------------
// Mean level of each converter separately, read straight from the raw ring
// and WITHOUT config.calib_channel_delta applied - which is exactly what
// calibrating that delta needs. Even ring bytes are ADC B (straight wiring),
// odd ones ADC A (bit-reversed, see doc/Hardware.md). Only meaningful while a
// dual-channel acquisition is running: a stopped ring has already been
// un-reversed in place, and a single-channel one holds ADC B only.
bool capture_get_channel_means(int *a_x100, int *b_x100)
{
  const int count = 4096; // ring bytes averaged, i.e. 2048 per converter
  int64_t sum_a = 0, sum_b = 0;

  if (!g_dual_channel || g_stopped || g_buffer_generation == 0)
    return false;

  for (int i = 0; i < count; i += 2)
  {
    sum_b += g_capture_buffer[i];
    sum_a += (int)(__RBIT((uint32_t)g_capture_buffer[i + 1]) >> 24);
  }

  *b_x100 = (int)(sum_b * 200 / count);
  *a_x100 = (int)(sum_a * 200 / count);

  return true;
}

//---------------------------------------------------------------------
uint32_t capture_get_generation(void)
{
  return g_buffer_generation;
}

//---------------------------------------------------------------------
// Sample period of the record consumers actually see, which is the ring's
// only while the full-rate window applies (see update_storage_window); a
// stopped scope always reads the raw ring.
int capture_get_record_period(void)
{
  if (g_stopped && g_capture_buffer_info.valid)
    return g_sample_period;

  return g_storage_buffer_info.period;
}

//---------------------------------------------------------------------
// Direct (zero-copy) access to the active record for main-loop analyzers.
// Does NOT consume the pending-frame handshake; like the display path, the
// data may tear if the DMA IRQ lands a new frame mid-scan.
// trigger_timepos receives the trigger's position in time order (samples
// from the oldest sample), letting callers map record positions onto the
// trigger-relative display axis.
bool capture_get_record(const uint8_t **data, int *size, int *offset,
    int *period_ns, int *trigger_timepos)
{
  BufferInfo local;

  if (g_buffer_generation == 0 && !(g_stopped && g_capture_buffer_info.valid))
    return false;

  local = snapshot_active_info();

  *data      = local.data;
  *size      = local.size;
  *offset    = local.offset;
  *period_ns = local.period;

  *trigger_timepos = local.trigger - local.offset;

  if (*trigger_timepos < 0)
    *trigger_timepos += local.size;

  return true;
}

//---------------------------------------------------------------------
// Copy up to max_count time-ordered samples of the active record (used by
// the FFT view). With consume=true the pending-frame handshake is cleared
// exactly like capture_get_data does, so acquisition keeps flowing when
// this is the only consumer.
int capture_read_samples(uint8_t *dst, int max_count, int *period_ns, bool consume)
{
  BufferInfo local;
  int count, index;

  if (g_buffer_generation == 0 && !(g_stopped && g_capture_buffer_info.valid))
    return 0;

  local = snapshot_active_info();

  count = (max_count < local.size) ? max_count : local.size;
  index = local.offset;

  for (int i = 0; i < count; i++)
  {
    dst[i] = local.data[index];

    if (++index == local.size)
      index = 0;
  }

  *period_ns = local.period;

  if (consume)
    g_storage_buffer_info.valid = false;

  return count;
}

//---------------------------------------------------------------------
int capture_read_fast_samples(uint8_t *dst, int max_count, int *period_ns)
{
  int end, count, delta;
  bool dual;
  uint32_t primask;

  if (g_buffer_generation == 0 && !(g_stopped && g_capture_buffer_info.valid))
    return 0;

  // Freshest write position of the ring (same math as update_capture_buffer)
  primask = __get_PRIMASK();
  __disable_irq();
  end = g_next_buf_ptr - dma_get_count();
  // While stopped after a SINGLE capture the ring has already been
  // un-reversed in place by dma_finish(); only a live ring still carries
  // the raw interleave
  dual = g_dual_channel && !g_stopped;
  *period_ns = g_sample_period;
  __set_PRIMASK(primask);

  if (end < 0)
    end += CAPTURE_BUFFER_SIZE;
  else if (end >= CAPTURE_BUFFER_SIZE)
    end -= CAPTURE_BUFFER_SIZE;

  // In dual-channel mode the two interleaved ADCs alternate ring bytes and
  // the odd one is wired bit-reversed with its own offset (this is what
  // buffer_decimate_reverse undoes for the storage record). Undoing it here
  // too keeps the full 125 MS/s stream instead of one 62.5 MS/s phase,
  // which is the difference between a 31 MHz and a 62 MHz Nyquist limit
  // for the spectral probe.
  delta = config.calib_channel_delta;
  count = max_count;

  int start = end - count;

  while (start < 0)
    start += CAPTURE_BUFFER_SIZE;

  for (int i = 0; i < count; i++)
  {
    int v = g_capture_buffer[start];

    if (dual && (start & 1))
    {
      v = (int)(__RBIT((uint32_t)v) >> 24) + delta;

      if (v < 0)
        v = 0;
      else if (v > 255)
        v = 255;
    }

    dst[i] = (uint8_t)v;

    if (++start >= CAPTURE_BUFFER_SIZE)
      start -= CAPTURE_BUFFER_SIZE;
  }

  return count;
}

//---------------------------------------------------------------------
void capture_get_data(DataBuffer *db)
{
  BufferInfo local;
  BufferInfo *info = &local;
  int index_inc, error_inc, index, error, next_index, next_error;
  int istart, dx, min_value, max_value, flags;
  int64_t offs;

  local = snapshot_active_info();

  offs = config.horizontal_position - (int64_t)config.horizontal_period * (db->size/2 - 1) -
      info->period/2 - config.horizontal_period/2;

  index_inc = config.horizontal_period / info->period;
  error_inc = config.horizontal_period % info->period;
  index     = offs / info->period;
  error     = offs % info->period;
  dx        = info->period / config.horizontal_period;
  istart    = 0;

  if (error < 0)
  {
    index -= 1;
    error += info->period;
  }

  if (info->trigger > info->offset)
    info->min_index = info->offset - info->trigger;
  else
    info->min_index = info->offset - info->trigger - info->size;

  info->max_index = info->size + info->min_index - 1;

  if (index_inc == 0)
  {
    for (istart = 0; (error + istart * error_inc) > 0; istart--);
  }

  db->min_value = INT_MAX;
  db->max_value = INT_MIN;
  db->vertical_position = info->vpos;

  for (int i = 0; i < db->size; i++)
  {
    next_index = index + index_inc;
    next_error = error + error_inc;

    if (next_error >= info->period)
    {
      next_index += 1;
      next_error -= info->period;
    }

    flags = SAMPLE_FLAG_NONE;

    if (next_index == index)
    {
      int di = i - istart;
      int v = info->data[clamp_index(info, index)];
      int nv = info->data[clamp_index(info, index + 1)];
      int value = ((dx - di) * v + di * nv + dx/2) / dx;

      min_value = value;
      max_value = value;
      flags = SAMPLE_FLAG_VALID | SAMPLE_FLAG_FILLED;
    }
    else if ((next_index - index) == 1)
    {
      int idx = clamp_index(info, next_index);

      istart = i;

      min_value = info->data[idx];
      max_value = info->data[idx];
      flags = SAMPLE_FLAG_VALID;
    }
    else
    {
      istart = i;
      min_value = 255;
      max_value = 0;

      if (find_min_max(info, index, next_index-1, &min_value, &max_value))
        flags = SAMPLE_FLAG_VALID;
    }

    if (min_value == 0)
      flags |= SAMPLE_FLAG_CLIP_L;

    if (max_value == 255)
      flags |= SAMPLE_FLAG_CLIP_H;

    if (flags & SAMPLE_FLAG_VALID)
    {
      min_value = ((min_value - ZERO_POINT) * info->vs_mult + info->vs_mult/2) / CALIB_MULTIPLIER;
      max_value = ((max_value - ZERO_POINT) * info->vs_mult + info->vs_mult/2) / CALIB_MULTIPLIER;

      if (min_value < db->min_value)
        db->min_value = min_value;

      if (max_value > db->max_value)
        db->max_value = max_value;
    }

    db->min[i] = min_value;
    db->max[i] = max_value;
    db->flags[i] = flags;

    index = next_index;
    error = next_error;
  }

  // NOTE: db->frequency is fed by capture_get_measurements() consumers now;
  // running the full-record measure scan here put ~1 ms on EVERY displayed
  // frame and pan tick once measurements became default-on
  db->frequency = 0;

  // Hand the storage buffer back to the DMA IRQ: update_storage_buffer()
  // refills it only while valid is false, so this must clear
  // unconditionally or acquisition and display deadlock on one frame
  g_storage_buffer_info.valid = false;
}

//---------------------------------------------------------------------
void capture_get_raw_data(int *raw, int size)
{
  for (int i = 0; i < size; i++)
    raw[i] = g_capture_buffer[i];
}

