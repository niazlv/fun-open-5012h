/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The GD32F407VE peripherals and the FNIRSI 5012H board wired to them.
 *
 * Every register offset, bit position and mask below comes from the vendor's
 * SVD-generated include/gd32f4xx.h - the same header the firmware is compiled
 * against - so there is no second, hand-copied description of the silicon to
 * drift out of step with the first.
 *
 * What is modeled, and why each one is here:
 *   RCU/PMU  - the 250 MHz PLL bring-up in main.c polls these for readiness
 *   FMC      - the settings store erases and programs flash through it, with
 *              NOR's can-only-clear-bits semantics that config.c depends on
 *   CRC      - config entries are sealed with the hardware CRC unit
 *   GPIO     - the LCD bus, the attenuator relays, the AC/DC switch and the
 *              two button encoders are all just pins
 *   TIMER1   - the 1 MHz timebase every timeout in the firmware is measured on
 *   TIMER0/7 - the AD9288 encode clocks and the DMA request source, i.e. the
 *              sample rate
 *   DMA1 ch2 - the acquisition itself: GPIOD (the ADC data bus) into the ring
 *   DAC/ADC0 - the vertical offset, and the battery gauge
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gd32f4xx.h"
#include "emu.h"
#include "../../src/core/common.h"

/*- Definitions -------------------------------------------------------------*/
#define PERIPH_OFF(addr)   ((uint32_t)(addr) - EMU_PERIPH_BASE)

/* Bus clocks after main.c's sys_init(): CK_SYS 250 MHz, AHB /1, APB1 /4,
 * APB2 /2. A timer on a bus whose prescaler is not 1 gets 2x that bus clock. */
#define AHB_HZ             250000000ull
#define APB1_HZ            (AHB_HZ / 4)
#define APB2_HZ            (AHB_HZ / 2)
#define TIMER_APB1_HZ      (APB1_HZ * 2)   /* TIMER1 (timebase), TIMER2 (BL) */
#define TIMER_APB2_HZ      (APB2_HZ * 2)   /* TIMER0/TIMER7 (ADC + DMA)      */

/* GD32F407VE flash sector map: 4 x 16 KB, 1 x 64 KB, 3 x 128 KB */
static const uint32_t g_sector_start[8] =
{
  0x00000, 0x04000, 0x08000, 0x0C000, 0x10000, 0x20000, 0x40000, 0x60000,
};
static const uint32_t g_sector_size[8] =
{
  0x04000, 0x04000, 0x04000, 0x04000, 0x10000, 0x20000, 0x20000, 0x20000,
};

/* Flash timings, from the GD32F4 datasheet: a word program is tens of
 * microseconds, a 128 KB sector erase is on the order of a second. Both are
 * long enough that config.c times them, so the model has to take time too. */
#define FLASH_PROGRAM_NS   30000ull
#define FLASH_ERASE_NS     800000000ull

#define FMC_KEY1           0x45670123u
#define FMC_KEY2           0xCDEF89ABu

/* AD9288 converters. Even ring bytes are ADC B, odd ones ADC A - and A is
 * wired to the bus bit-reversed, with a small offset the firmware nulls out
 * through config.calib_channel_delta (see doc/Hardware.md and buffer.c). */
#define ADC_A_OFFSET       5

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  bool     enabled;       /* CHEN                                            */
  uint32_t m0, m1;
  uint32_t cnt;           /* transfers left in the active buffer             */
  uint32_t reload;        /* what CH2CNT was programmed with                 */
  int      width;         /* bytes per transfer, 1 or 2                      */
  bool     ftfie;
  bool     ftf;           /* full-transfer-finish flag, awaiting service     */
  uint64_t time_ns;       /* the model has produced samples up to here       */
} DmaChannel;

/*- Variables ---------------------------------------------------------------*/
int emu_battery_mv = 3950;
bool emu_battery_charging = false;

static uint8_t g_periph[EMU_PERIPH_SIZE];

static uint16_t g_gpio_ext[9];       /* levels driven onto pins from outside */
static uint32_t g_buttons;

static uint64_t g_timer1_origin_ns;

static bool g_fmc_key1_seen;
static bool g_fmc_unlocked;
static uint64_t g_fmc_busy_until_ns;
static uint32_t g_fmc_stat_flags;    /* sticky END and error bits            */

static uint32_t g_crc;

static DmaChannel g_dma;

static bool g_reset_requested;

/* NVIC */
static uint32_t g_nvic_enabled[8];
static uint32_t g_nvic_pending[8];

/* LCD bus state, sampled from the pins */
static bool g_lcd_wr_prev = true;

static AfeState g_afe;

/*- Prototypes --------------------------------------------------------------*/
static void gpio_outputs_changed(int port);
static void dma_sync(uint64_t now_ns);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint32_t pget(uint32_t addr)
{
  uint32_t v;
  memcpy(&v, g_periph + PERIPH_OFF(addr & ~3u), 4);
  return v;
}

//-----------------------------------------------------------------------------
static void pset(uint32_t addr, uint32_t v)
{
  memcpy(g_periph + PERIPH_OFF(addr & ~3u), &v, 4);
}

//-----------------------------------------------------------------------------
// Folds a byte/halfword access into the 32-bit word the models work with
static uint32_t merge_write(uint32_t old, uint32_t addr, unsigned size,
    uint64_t value)
{
  int shift = (int)(addr & 3) * 8;
  uint32_t mask;

  if (size >= 4)
    return (uint32_t)value;

  mask = ((1u << (size * 8)) - 1u) << shift;

  return (old & ~mask) | (((uint32_t)value << shift) & mask);
}

//-----------------------------------------------------------------------------
static uint64_t extract_read(uint32_t word, uint32_t addr, unsigned size)
{
  int shift = (int)(addr & 3) * 8;

  if (size >= 4)
    return word;

  return (word >> shift) & ((1u << (size * 8)) - 1u);
}

//-----------------------------------------------------------------------------
void board_init(void)
{
  memset(g_periph, 0, sizeof(g_periph));

  for (int i = 0; i < 9; i++)
    g_gpio_ext[i] = 0xffff;   // everything idles pulled up

  board_set_buttons(0);
  board_reset();
}

//-----------------------------------------------------------------------------
void board_reset(void)
{
  memset(&g_dma, 0, sizeof(g_dma));
  memset(g_nvic_enabled, 0, sizeof(g_nvic_enabled));
  memset(g_nvic_pending, 0, sizeof(g_nvic_pending));

  g_timer1_origin_ns = 0;
  g_fmc_key1_seen = false;
  g_fmc_unlocked = false;
  g_fmc_busy_until_ns = 0;
  g_fmc_stat_flags = 0;
  g_crc = 0xffffffff;
  g_reset_requested = false;
  g_lcd_wr_prev = true;

  g_afe.relay_scale = -1;
  g_afe.dac_code = 2048;
  g_afe.ac_coupling = false;
  g_afe.backlight_pct = 0;

  st7789_init();
}

//-----------------------------------------------------------------------------
bool board_reset_requested(void)
{
  return g_reset_requested;
}

//-----------------------------------------------------------------------------
void board_get_afe(AfeState *out)
{
  *out = g_afe;
}

/*- Buttons -----------------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The panel has two 8-input priority encoders. Each drives a group-strobe line
// low while any of its keys is down, and puts the key's index on three address
// lines - all active low. buttons.c reads the pins through ~ISTAT and rebuilds
// the index, so driving the pins the way the encoders do is all it takes for
// the firmware to see a keypress; it never learns where the press came from.
//
//   group 0 (BTN_STOP..BTN_AUTO):  GS = PE14, A0 = PB13, A1 = PB14, A2 = PE13
//   group 1 (BTN_50P..BTN_DOWN):   GS = PB10, A0 = PB11, A1 = PB12, A2 = PE15
//   PE11 = F2, PE12 = SHIFT (1X/10X), PB15 = charger present
static int highest_bit(uint32_t v)
{
  int idx = -1;

  for (int i = 0; i < 8; i++)
  {
    if (v & (1u << i))
      idx = i;
  }

  return idx;
}

//-----------------------------------------------------------------------------
void board_set_buttons(uint32_t mask)
{
  uint16_t pb = 0xffff, pe = 0xffff;
  int idx0 = highest_bit(mask & 0xff);
  int idx1 = highest_bit((mask >> 8) & 0xff);

  g_buttons = mask;

  if (idx0 >= 0)
  {
    pe &= ~(1u << 14);                            // group 0 strobe
    if (idx0 & 1) pb &= ~(1u << 13);
    if (idx0 & 2) pb &= ~(1u << 14);
    if (idx0 & 4) pe &= ~(1u << 13);
  }

  if (idx1 >= 0)
  {
    pb &= ~(1u << 10);                            // group 1 strobe
    if (idx1 & 1) pb &= ~(1u << 11);
    if (idx1 & 2) pb &= ~(1u << 12);
    if (idx1 & 4) pe &= ~(1u << 15);
  }

  if (mask & (1u << 16))                          // BTN_F2
    pe &= ~(1u << 11);

  if (mask & (1u << 17))                          // BTN_1X_10X / BTN_SHIFT
    pe &= ~(1u << 12);

  if (emu_battery_charging)
    pb &= ~(1u << 15);

  g_gpio_ext[1] = pb;   // GPIOB
  g_gpio_ext[4] = pe;   // GPIOE
}

//-----------------------------------------------------------------------------
uint32_t board_get_buttons(void)
{
  return g_buttons;
}

/*- GPIO --------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
static int gpio_index(uint32_t base)
{
  if (base < GPIOA_BASE || base > GPIOI_BASE)
    return -1;

  return (int)((base - GPIOA_BASE) / 0x400);
}

//-----------------------------------------------------------------------------
static uint32_t gpio_base(int port)
{
  return (uint32_t)GPIOA_BASE + (uint32_t)port * 0x400u;
}

//-----------------------------------------------------------------------------
static uint32_t gpio_output_mask(int port)
{
  uint32_t ctl = pget(gpio_base(port) + offsetof(GPIOA_Type, CTL));
  uint32_t mask = 0;

  for (int pin = 0; pin < 16; pin++)
  {
    if (((ctl >> (pin * 2)) & 3) == 1)  // 1 = general purpose output
      mask |= 1u << pin;
  }

  return mask;
}

//-----------------------------------------------------------------------------
// A pin reads back what drives it: the output latch for outputs, whatever is
// wired to it for inputs.
static uint32_t gpio_istat(int port)
{
  uint32_t out = gpio_output_mask(port);
  uint32_t octl = pget(gpio_base(port) + offsetof(GPIOA_Type, OCTL));

  return (octl & out) | ((uint32_t)g_gpio_ext[port] & ~out);
}

//-----------------------------------------------------------------------------
static bool pin_level(int port, int pin)
{
  return (gpio_istat(port) >> pin) & 1;
}

/*- ST7789 bus and the analog front end, both just pins ---------------------*/

//-----------------------------------------------------------------------------
// LCD_RST = PC6, LCD_RD = PB3, LCD_WR = PB4, LCD_RS = PB5, LCD_CS = PB6,
// LCD_BL = PB0, and the 8-bit data bus is PE[7:0]. The panel latches the bus
// on the rising edge of WR while CS is low, with RS telling command from data.
static void lcd_bus_poll(void)
{
  bool wr = pin_level(1, 4);
  bool cs = pin_level(1, 6);
  bool rs = pin_level(1, 5);

  if (wr && !g_lcd_wr_prev && !cs)
  {
    uint32_t octl_e = pget(gpio_base(4) + offsetof(GPIOA_Type, OCTL));
    st7789_bus_write((uint8_t)(octl_e & 0xff), !rs);
  }

  g_lcd_wr_prev = wr;

  st7789_reset_pin(pin_level(2, 6));
}

//-----------------------------------------------------------------------------
// The attenuator is a bank of relays, one per range, each pulled low to select
// it (capture.c: set_vertical_scale). 50 mV/div has no relay of its own - the
// firmware selects the 100 mV hardware range and magnifies in software, which
// is exactly why the emulated ADC gain below follows the RELAY, not the menu.
//
//   Q0 = PB9, Q1 = PB8, Q2 = PB7, Q3 = PC12, Q4 = PC11, Q5 = PC10, Q6 = PA15
static void afe_poll(void)
{
  static const struct { int port, pin, scale; } relays[] =
  {
    { 1,  9, VS_500_mV }, { 1,  8, VS_1_V   }, { 1,  7, VS_200_mV },
    { 2, 12, VS_100_mV }, { 2, 11, VS_10_V  }, { 2, 10, VS_5_V    },
    { 0, 15, VS_2_V    },
  };

  g_afe.relay_scale = -1;

  for (unsigned i = 0; i < sizeof(relays) / sizeof(relays[0]); i++)
  {
    if (!pin_level(relays[i].port, relays[i].pin))
      g_afe.relay_scale = relays[i].scale;
  }

  // AC_DC = PC15, driven low for AC coupling
  g_afe.ac_coupling = !pin_level(2, 15);

  // Backlight: PB0 driven directly at the ends of the range, or handed to
  // TIMER2 CH2 as PWM in between (lcd.c: lcd_set_backlight_level)
  {
    uint32_t ctl = pget(gpio_base(1) + offsetof(GPIOA_Type, CTL));
    int mode = (ctl >> 0) & 3;

    if (mode == 2) // alternate function: the PWM drives it
    {
      uint32_t car = pget(TIMER2_BASE + offsetof(TIMER2_Type, CAR));
      uint32_t cv  = pget(TIMER2_BASE + offsetof(TIMER2_Type, CH2CV));
      uint32_t en  = pget(TIMER2_BASE + offsetof(TIMER2_Type, CTL0));

      if ((en & TIMER2_CTL0_CEN_Msk) && car)
        g_afe.backlight_pct = (int)((uint64_t)cv * 100 / car);
    }
    else
    {
      g_afe.backlight_pct = pin_level(1, 0) ? 100 : 0;
    }
  }
}

//-----------------------------------------------------------------------------
static void gpio_outputs_changed(int port)
{
  if (port == 1 || port == 2 || port == 4)  // B, C, E carry the LCD bus
    lcd_bus_poll();

  if (port == 0 || port == 1 || port == 2)  // A, B, C carry the relays
    afe_poll();
}

/*- CRC ---------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The GD32 unit is CRC-32/MPEG-2 over whole words: polynomial 0x04C11DB7,
// seeded with all ones, MSB first, no reflection and no final inversion.
static void crc_feed(uint32_t word)
{
  g_crc ^= word;

  for (int i = 0; i < 32; i++)
    g_crc = (g_crc & 0x80000000u) ? ((g_crc << 1) ^ 0x04C11DB7u) : (g_crc << 1);
}

/*- FMC ---------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
static bool fmc_busy(void)
{
  return cpu_now_ns() < g_fmc_busy_until_ns;
}

//-----------------------------------------------------------------------------
static void fmc_write_ctl(uint32_t value)
{
  uint32_t prev = pget(FMC_BASE + offsetof(FMC_Type, CTL));

  if (g_fmc_unlocked)
    value &= ~FMC_CTL_LK_Msk;

  pset(FMC_BASE + offsetof(FMC_Type, CTL), value);

  if (value & FMC_CTL_LK_Msk)
  {
    g_fmc_unlocked = false;
    g_fmc_key1_seen = false;
  }

  // START with SER set kicks off a sector erase. Only on the 0->1 edge: the
  // firmware writes the command word first and the start bit second.
  if ((value & FMC_CTL_START_Msk) && !(prev & FMC_CTL_START_Msk) &&
      (value & FMC_CTL_SER_Msk))
  {
    uint32_t sn = (value & FMC_CTL_SN_Msk) >> FMC_CTL_SN_Pos;

    if (!g_fmc_unlocked)
    {
      g_fmc_stat_flags |= FMC_STAT_WPERR_Msk;
      return;
    }

    if (sn < 8)
    {
      memset(emu_flash + g_sector_start[sn], 0xff, g_sector_size[sn]);
      g_fmc_busy_until_ns = cpu_now_ns() + FLASH_ERASE_NS;
      g_fmc_stat_flags |= FMC_STAT_END_Msk;

      emu_log("FMC: erase sector %u (0x%08x, %u KB)", sn,
          EMU_FLASH_BASE + g_sector_start[sn], g_sector_size[sn] / 1024);
    }
    else
    {
      g_fmc_stat_flags |= FMC_STAT_PGSERR_Msk;
    }
  }
}

//-----------------------------------------------------------------------------
// A guest store into the flash array. Flash is mapped read-only, so this is
// reached from the access fault: the FMC has to be unlocked and in programming
// mode, and a NOR cell can only ever clear bits - the settings store's
// "is this slot still blank" check is built on exactly that.
bool board_flash_write(uint32_t addr, uint64_t value, unsigned size)
{
  uint32_t ctl = pget(FMC_BASE + offsetof(FMC_Type, CTL));
  uint32_t off = addr - EMU_FLASH_BASE;
  uint8_t bytes[8];

  if (off >= EMU_FLASH_SIZE || size > 8)
    return false;

  if (!g_fmc_unlocked || !(ctl & FMC_CTL_PG_Msk))
  {
    g_fmc_stat_flags |= FMC_STAT_WPERR_Msk;
    emu_log("FMC: rejected write to 0x%08x (%s)", addr,
        g_fmc_unlocked ? "not in program mode" : "locked");
    return false;
  }

  memcpy(bytes, &value, size <= 8 ? size : 8);

  for (unsigned i = 0; i < size && off + i < EMU_FLASH_SIZE; i++)
    emu_flash[off + i] &= bytes[i];

  g_fmc_busy_until_ns = cpu_now_ns() + FLASH_PROGRAM_NS;
  g_fmc_stat_flags |= FMC_STAT_END_Msk;

  return true;
}

/*- DMA + the acquisition front end -----------------------------------------*/

//-----------------------------------------------------------------------------
// One DMA request per TIMER7 update event. The firmware sets CAR = 1 and moves
// PSC to pick the timebase, so this is where "sample rate" actually comes from.
static uint64_t dma_request_period_ns(void)
{
  uint32_t ctl0 = pget(TIMER7_BASE + offsetof(TIMER7_Type, CTL0));
  uint64_t psc  = pget(TIMER7_BASE + offsetof(TIMER7_Type, PSC)) & 0xffff;
  uint64_t car  = pget(TIMER7_BASE + offsetof(TIMER7_Type, CAR)) & 0xffff;
  uint64_t ticks;

  if (!(ctl0 & TIMER7_CTL0_CEN_Msk))
    return 0;  // the timer is stopped: no requests at all

  ticks = (psc + 1) * (car + 1);

  return (ticks * 1000000000ull + TIMER_APB2_HZ / 2) / TIMER_APB2_HZ;
}

//-----------------------------------------------------------------------------
// MBS lives in the control register rather than beside it. The firmware does
// read-modify-write on CH2CTL through CMSIS bitfields (DMA1->CH2CTL_b.CHEN = 1),
// so a read that reported anything other than the register's own state would
// be written straight back - and dma_start()'s explicit "MBS = 0" would be
// undone by the very next bitfield assignment, starting the acquisition on the
// wrong half of the double buffer.
static bool dma_mbs(void)
{
  return (pget(DMA1_BASE + offsetof(DMA1_Type, CH2CTL)) & DMA1_CH2CTL_MBS_Msk) != 0;
}

//-----------------------------------------------------------------------------
static void dma_set_mbs(bool value)
{
  uint32_t addr = DMA1_BASE + offsetof(DMA1_Type, CH2CTL);
  uint32_t ctl = pget(addr);

  pset(addr, value ? (ctl | DMA1_CH2CTL_MBS_Msk) : (ctl & ~DMA1_CH2CTL_MBS_Msk));
}

//-----------------------------------------------------------------------------
static uint32_t dma_active_address(void)
{
  uint32_t base = dma_mbs() ? g_dma.m1 : g_dma.m0;

  return base + (g_dma.reload - g_dma.cnt) * (uint32_t)g_dma.width;
}

//-----------------------------------------------------------------------------
// Produces `count` transfers worth of converter output straight into the
// guest's SRAM, which is where the DMA would put them.
//
// Dual channel is the AD9288 running interleaved: the 16-bit bus snapshot
// carries both converters, B on the low byte and A on the high one, half a
// sample period apart - so one transfer is two samples and the ring runs at
// twice the request rate. A is wired to the bus bit-reversed and sits a few
// counts high; buffer.c undoes both, and it can only do so if they are
// actually there.
static void dma_produce(uint32_t addr, uint32_t count, uint64_t t0_ns,
    uint64_t period_ns)
{
  uint32_t off = addr - EMU_SRAM_BASE;
  AfeState afe;

  if (off >= EMU_SRAM_SIZE)
    return;

  board_get_afe(&afe);

  if (g_dma.width == 2)
  {
    uint64_t half = period_ns / 2;

    for (uint32_t i = 0; i < count && off + 1 < EMU_SRAM_SIZE; i++, off += 2)
    {
      uint64_t t = t0_ns + (uint64_t)i * period_ns;
      int a = sig_adc_sample(t + half, &afe, 1) + ADC_A_OFFSET;
      uint8_t rev = 0;

      if (a > 255) a = 255;

      for (int b = 0; b < 8; b++)          // the A channel's reversed wiring
        rev |= (uint8_t)(((a >> b) & 1) << (7 - b));

      emu_sram[off]     = sig_adc_sample(t, &afe, 0);
      emu_sram[off + 1] = rev;
    }
  }
  else
  {
    for (uint32_t i = 0; i < count && off < EMU_SRAM_SIZE; i++, off++)
      emu_sram[off] = sig_adc_sample(t0_ns + (uint64_t)i * period_ns, &afe, 0);
  }
}

//-----------------------------------------------------------------------------
// Catches the acquisition up to the core's clock.
//
// The one place this deliberately departs from the silicon: the model will not
// run past the end of a buffer while the completion interrupt from that buffer
// is still unserviced. Real hardware would keep going and overwrite data the
// firmware had not read yet - an overrun that depends on how fast the host
// happens to be, which is not a property of the instrument. Stalling instead
// makes the sample stream the firmware sees contiguous and reproducible, and
// it is also what keeps capture.c's dma_wait_count() spin bounded.
static void dma_sync(uint64_t now_ns)
{
  uint64_t period = dma_request_period_ns();

  if (!g_dma.enabled || period == 0)
  {
    g_dma.time_ns = now_ns;
    return;
  }

  while (g_dma.time_ns + period <= now_ns)
  {
    uint64_t available = (now_ns - g_dma.time_ns) / period;
    uint32_t n;

    if (g_dma.ftf)
      break;   // the previous completion has not been serviced yet

    n = (uint32_t)(available < g_dma.cnt ? available : g_dma.cnt);

    if (n == 0)
      break;

    dma_produce(dma_active_address(), n, g_dma.time_ns, period);

    g_dma.cnt -= n;
    g_dma.time_ns += (uint64_t)n * period;

    if (g_dma.cnt == 0)
    {
      // Full transfer finished: raise the flag, hand the other buffer to the
      // channel and let the firmware's handler re-point the one just filled
      g_dma.ftf = true;
      dma_set_mbs(!dma_mbs());
      g_dma.cnt = g_dma.reload;

      if (g_dma.ftfie)
        g_nvic_pending[DMA1_CH2_IRQN / 32] |= 1u << (DMA1_CH2_IRQN % 32);
    }
  }
}

//-----------------------------------------------------------------------------
static void dma_write_ctl(uint32_t value)
{
  bool was = g_dma.enabled;

  pset(DMA1_BASE + offsetof(DMA1_Type, CH2CTL), value);

  g_dma.enabled = (value & DMA1_CH2CTL_CHEN_Msk) != 0;
  g_dma.ftfie   = (value & DMA1_CH2CTL_FTFIE_Msk) != 0;
  g_dma.width   = ((value & DMA1_CH2CTL_MWIDTH_Msk) >> DMA1_CH2CTL_MWIDTH_Pos)
      ? 2 : 1;

  if (g_dma.enabled && !was)
  {
    g_dma.m0     = pget(DMA1_BASE + offsetof(DMA1_Type, CH2M0ADDR));
    g_dma.m1     = pget(DMA1_BASE + offsetof(DMA1_Type, CH2M1ADDR));
    g_dma.reload = pget(DMA1_BASE + offsetof(DMA1_Type, CH2CNT));
    g_dma.cnt    = g_dma.reload;
    g_dma.ftf    = false;
    g_dma.time_ns = cpu_now_ns();
  }

  if (!g_dma.enabled)
    g_dma.ftf = false;
}

/*- MMIO dispatch -----------------------------------------------------------*/

//-----------------------------------------------------------------------------
uint64_t board_mmio_read(uint64_t address, unsigned size)
{
  uint32_t addr = (uint32_t)address;
  uint32_t base = addr & 0xFFFFFC00u;
  uint32_t off  = addr & 0x3FFu;
  uint32_t word;

  if (addr < EMU_PERIPH_BASE || addr >= EMU_PERIPH_BASE + EMU_PERIPH_SIZE)
    return 0;

  word = pget(addr);

  switch (base)
  {
    case RCU_BASE:
      // The crystal and the PLL report themselves stable once enabled; the
      // clock switch reports the source it was told to select
      if (off == offsetof(RCU_Type, CTL))
      {
        if (word & RCU_CTL_HXTALEN_Msk) word |= RCU_CTL_HXTALSTB_Msk;
        if (word & RCU_CTL_PLLEN_Msk)   word |= RCU_CTL_PLLSTB_Msk;
        word |= RCU_CTL_IRC16MSTB_Msk;
      }
      else if (off == offsetof(RCU_Type, CFG0))
      {
        word = (word & ~RCU_CFG0_SCSS_Msk) |
            (((word & RCU_CFG0_SCS_Msk) >> RCU_CFG0_SCS_Pos) << RCU_CFG0_SCSS_Pos);
      }
      break;

    case PMU_BASE:
      // High-drive mode: the ready flags follow the requests main.c makes
      // before it dares run the core at 250 MHz
      if (off == offsetof(PMU_Type, CS))
      {
        uint32_t ctl = pget(PMU_BASE + offsetof(PMU_Type, CTL));

        if (ctl & PMU_CTL_HDEN_Msk) word |= PMU_CS_HDRF_Msk;
        if (ctl & PMU_CTL_HDS_Msk)  word |= PMU_CS_HDSRF_Msk;

        word |= PMU_CS_LDOVSRF_Msk;
      }
      break;

    case FMC_BASE:
      if (off == offsetof(FMC_Type, STAT))
      {
        word = g_fmc_stat_flags;

        if (fmc_busy())
          word |= FMC_STAT_BUSY_Msk;
      }
      else if (off == offsetof(FMC_Type, CTL))
      {
        word = g_fmc_unlocked ? (word & ~FMC_CTL_LK_Msk) : (word | FMC_CTL_LK_Msk);
      }
      break;

    case CRC_BASE:
      if (off == offsetof(CRC_Type, DATA))
        word = g_crc;
      else if (off == offsetof(CRC_Type, CTL))
        word = 0;  // the reset bit clears itself
      break;

    case TIMER1_BASE:
      // The 1 MHz free-running timebase. Everything the firmware times - the
      // debounce, the flash timeouts, the measurement throttle - is this
      // counter, so it is derived from the emulator's clock, not from a
      // stored value.
      if (off == offsetof(TIMER1_Type, CNT))
      {
        uint32_t ctl0 = pget(TIMER1_BASE + offsetof(TIMER1_Type, CTL0));
        uint64_t psc  = pget(TIMER1_BASE + offsetof(TIMER1_Type, PSC)) & 0xffff;

        if (ctl0 & TIMER1_CTL0_CEN_Msk)
        {
          uint64_t elapsed = cpu_now_ns() - g_timer1_origin_ns;
          uint64_t cycles = elapsed * (TIMER_APB1_HZ / 1000000ull) / 1000ull;

          word = (uint32_t)(cycles / (psc + 1));
        }
      }
      break;

    case DMA1_BASE:
      dma_sync(cpu_now_ns());

      if (off == offsetof(DMA1_Type, CH2CNT))
        word = g_dma.cnt;
      else if (off == offsetof(DMA1_Type, CH2CTL))
        word = (word & ~DMA1_CH2CTL_CHEN_Msk) |
            (g_dma.enabled ? DMA1_CH2CTL_CHEN_Msk : 0);
      else if (off == offsetof(DMA1_Type, INTF0))
        word = g_dma.ftf ? DMA1_INTF0_FTFIF2_Msk : 0;
      break;

    case ADC0_BASE:
      // The battery divider on PB1. Overs__ampling is configured but the
      // result register is what gets read (battery.c: read_battery_voltage).
      if (off == offsetof(ADC0_Type, RDATA))
        word = (uint32_t)((int64_t)emu_battery_mv * 4096 / 6600);
      else if (off == offsetof(ADC0_Type, STAT))
        word = ADC0_STAT_EOC_Msk | ADC0_STAT_EOIC_Msk;
      else if (off == offsetof(ADC0_Type, CTL1))
        word &= ~(ADC0_CTL1_RSTCLB_Msk | ADC0_CTL1_CLB_Msk | ADC0_CTL1_SWRCST_Msk);
      break;

    default:
      break;
  }

  {
    int gp = gpio_index(base);

    if (gp >= 0 && off == offsetof(GPIOA_Type, ISTAT))
      word = gpio_istat(gp);
  }

  return extract_read(word, addr, size);
}

//-----------------------------------------------------------------------------
void board_mmio_write(uint64_t address, unsigned size, uint64_t value)
{
  uint32_t addr = (uint32_t)address;
  uint32_t base = addr & 0xFFFFFC00u;
  uint32_t off  = addr & 0x3FFu;
  uint32_t word;
  int gp;

  if (addr < EMU_PERIPH_BASE || addr >= EMU_PERIPH_BASE + EMU_PERIPH_SIZE)
    return;

  word = merge_write(pget(addr), addr, size, value);

  gp = gpio_index(base);

  if (gp >= 0)
  {
    uint32_t octl_addr = gpio_base(gp) + offsetof(GPIOA_Type, OCTL);
    uint32_t octl = pget(octl_addr);

    if (off == offsetof(GPIOA_Type, BOP))
    {
      // Set the pins named in the low half, clear those in the high half
      octl = (octl | (word & 0xffff)) & ~(word >> 16);
      pset(octl_addr, octl);
      gpio_outputs_changed(gp);
      return;
    }

    if (off == offsetof(GPIOA_Type, BC))
    {
      pset(octl_addr, octl & ~(word & 0xffff));
      gpio_outputs_changed(gp);
      return;
    }

    if (off == offsetof(GPIOA_Type, TG))
    {
      pset(octl_addr, octl ^ (word & 0xffff));
      gpio_outputs_changed(gp);
      return;
    }

    pset(addr, word);

    if (off == offsetof(GPIOA_Type, OCTL) || off == offsetof(GPIOA_Type, CTL))
      gpio_outputs_changed(gp);

    return;
  }

  switch (base)
  {
    case FMC_BASE:
      if (off == offsetof(FMC_Type, KEY))
      {
        if (word == FMC_KEY1)
        {
          g_fmc_key1_seen = true;
        }
        else if (word == FMC_KEY2 && g_fmc_key1_seen)
        {
          g_fmc_unlocked = true;
          g_fmc_key1_seen = false;
        }
        else
        {
          g_fmc_key1_seen = false;
        }
        return;
      }

      if (off == offsetof(FMC_Type, CTL))
      {
        fmc_write_ctl(word);
        return;
      }

      if (off == offsetof(FMC_Type, STAT))
      {
        // END and the error bits are cleared by writing ones to them
        g_fmc_stat_flags &= ~word;
        return;
      }
      break;

    case CRC_BASE:
      if (off == offsetof(CRC_Type, DATA))
      {
        crc_feed(word);
        return;
      }

      if (off == offsetof(CRC_Type, CTL))
      {
        if (word & CRC_CTL_RST_Msk)
          g_crc = 0xffffffff;
        return;
      }
      break;

    case TIMER1_BASE:
      pset(addr, word);

      // An update event latches the prescaler and restarts the count, which is
      // how timer_init() puts the timebase at a known zero
      if (off == offsetof(TIMER1_Type, SWEVG) && (word & TIMER1_SWEVG_UPG_Msk))
        g_timer1_origin_ns = cpu_now_ns();
      else if (off == offsetof(TIMER1_Type, CNT))
        g_timer1_origin_ns = cpu_now_ns();

      return;

    case TIMER7_BASE:
      // Changing the sample clock while the channel runs must not retroactively
      // reinterpret samples already produced
      dma_sync(cpu_now_ns());
      pset(addr, word);
      return;

    case TIMER2_BASE:
      pset(addr, word);
      afe_poll();          // the backlight PWM duty lives here
      return;

    case DAC_BASE:
      pset(addr, word);

      if (off == offsetof(DAC_Type, DAC0_R12DH))
        g_afe.dac_code = (int)(word & 0xfff);

      return;

    case DMA1_BASE:
      dma_sync(cpu_now_ns());

      if (off == offsetof(DMA1_Type, CH2CTL))
      {
        dma_write_ctl(word);
        return;
      }

      if (off == offsetof(DMA1_Type, INTC0))
      {
        if (word & DMA1_INTC0_FTFIFC2_Msk)
          g_dma.ftf = false;          // serviced: the channel may run on
        pset(addr, 0);
        return;
      }

      if (off == offsetof(DMA1_Type, CH2M0ADDR))
        g_dma.m0 = word;
      else if (off == offsetof(DMA1_Type, CH2M1ADDR))
        g_dma.m1 = word;

      pset(addr, word);
      return;

    default:
      break;
  }

  pset(addr, word);
}

/*- Cortex-M system control space -------------------------------------------*/

//-----------------------------------------------------------------------------
// NVIC and SCB live at architecturally fixed addresses, so these offsets are
// part of ARMv7-M rather than of this particular vendor's header.
#define NVIC_ISER_BASE   0xE000E100u
#define NVIC_ICER_BASE   0xE000E180u
#define NVIC_ISPR_BASE   0xE000E200u
#define NVIC_ICPR_BASE   0xE000E280u
#define SCB_AIRCR        0xE000ED0Cu
#define SCB_AIRCR_KEY    0x05FA0000u
#define SCB_AIRCR_SYSRESETREQ (1u << 2)

static uint8_t g_scs[EMU_SCS_SIZE];

//-----------------------------------------------------------------------------
uint64_t scs_mmio_read(uint64_t address, unsigned size)
{
  uint32_t addr = (uint32_t)address;
  uint32_t word;

  if (addr >= NVIC_ISER_BASE && addr < NVIC_ISER_BASE + 32)
    return extract_read(g_nvic_enabled[(addr - NVIC_ISER_BASE) / 4], addr, size);

  if (addr >= NVIC_ICER_BASE && addr < NVIC_ICER_BASE + 32)
    return extract_read(g_nvic_enabled[(addr - NVIC_ICER_BASE) / 4], addr, size);

  if (addr >= NVIC_ISPR_BASE && addr < NVIC_ISPR_BASE + 32)
    return extract_read(g_nvic_pending[(addr - NVIC_ISPR_BASE) / 4], addr, size);

  if (addr >= NVIC_ICPR_BASE && addr < NVIC_ICPR_BASE + 32)
    return extract_read(g_nvic_pending[(addr - NVIC_ICPR_BASE) / 4], addr, size);

  memcpy(&word, g_scs + ((addr & ~3u) - EMU_SCS_BASE), 4);

  return extract_read(word, addr, size);
}

//-----------------------------------------------------------------------------
void scs_mmio_write(uint64_t address, unsigned size, uint64_t value)
{
  uint32_t addr = (uint32_t)address;
  uint32_t word;

  memcpy(&word, g_scs + ((addr & ~3u) - EMU_SCS_BASE), 4);
  word = merge_write(word, addr, size, value);

  if (addr >= NVIC_ISER_BASE && addr < NVIC_ISER_BASE + 32)
  {
    g_nvic_enabled[(addr - NVIC_ISER_BASE) / 4] |= word;
    return;
  }

  if (addr >= NVIC_ICER_BASE && addr < NVIC_ICER_BASE + 32)
  {
    g_nvic_enabled[(addr - NVIC_ICER_BASE) / 4] &= ~word;
    return;
  }

  if (addr >= NVIC_ISPR_BASE && addr < NVIC_ISPR_BASE + 32)
  {
    g_nvic_pending[(addr - NVIC_ISPR_BASE) / 4] |= word;
    return;
  }

  if (addr >= NVIC_ICPR_BASE && addr < NVIC_ICPR_BASE + 32)
  {
    g_nvic_pending[(addr - NVIC_ICPR_BASE) / 4] &= ~word;
    return;
  }

  if (addr == SCB_AIRCR)
  {
    if ((word & 0xFFFF0000u) == SCB_AIRCR_KEY && (word & SCB_AIRCR_SYSRESETREQ))
      g_reset_requested = true;
    return;
  }

  memcpy(g_scs + ((addr & ~3u) - EMU_SCS_BASE), &word, 4);
}

/*- What the run loop asks each slice ---------------------------------------*/

//-----------------------------------------------------------------------------
void board_sync(void)
{
  dma_sync(cpu_now_ns());
}

//-----------------------------------------------------------------------------
int board_pending_irq(void)
{
  int word = DMA1_CH2_IRQN / 32;
  uint32_t bit = 1u << (DMA1_CH2_IRQN % 32);

  if ((g_nvic_pending[word] & bit) && (g_nvic_enabled[word] & bit))
    return DMA1_CH2_IRQN;

  return -1;
}

//-----------------------------------------------------------------------------
void board_irq_accepted(int irqn)
{
  g_nvic_pending[irqn / 32] &= ~(1u << (irqn % 32));
}

//-----------------------------------------------------------------------------
// What the acquisition is doing, for hang reports. A channel that is enabled
// with an unserviced completion flag is the shape of a capture that has
// stopped being fed - which is exactly the state worth naming.
void board_describe(char *buf, size_t size)
{
  int word = DMA1_CH2_IRQN / 32;
  uint32_t bit = 1u << (DMA1_CH2_IRQN % 32);
  uint64_t period = dma_request_period_ns();

  snprintf(buf, size,
      "dma %s buf%d cnt %u/%u %s  irq %s/%s  %s  relay %d dac %d",
      g_dma.enabled ? "run" : "off", dma_mbs() ? 1 : 0,
      g_dma.cnt, g_dma.reload, g_dma.ftf ? "COMPLETION UNSERVICED" : "flowing",
      (g_nvic_enabled[word] & bit) ? "en" : "dis",
      (g_nvic_pending[word] & bit) ? "PENDING" : "idle",
      period ? "sampling" : "sample clock stopped",
      g_afe.relay_scale, g_afe.dac_code);
}
