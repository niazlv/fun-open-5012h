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

/* TIMER0/TIMER7 prescalers.
 *
 * PSC is a shadow register on this silicon. A write only parks the value; the
 * prescaler keeps dividing by the old one until an update event - SWEVG.UPG,
 * or the counter wrapping CAR - hands it over. Reading the register straight
 * back, the way this model used to derive the sample rate, skips all of that
 * and makes a timebase change look instantaneous.
 *
 * The shadow is load-bearing, which is why it is worth modeling. An update
 * event also clears the prescaler counter, so firmware that writes PSC and
 * goes straight to CEN lets TIMER0 (the AD9288 encode clock) and TIMER7 (the
 * DMA strobe) each latch the new divider at their OWN first overflow, leaving
 * the two prescalers at an arbitrary relative phase. With CAR = 1 and ADC B's
 * encode clock inverted, ADC B's sampling edge and the DMA strobe are
 * nominally the SAME timer instant, so that slip puts the bus read on the
 * converter's output transition and the ring fills with metastable bytes.
 *
 * The analog half of that is NOT modeled and cannot be here: cpu_step()
 * advances time in 256-instruction slices, ~1 us, while the slip is a few ns
 * - both CEN writes land on the same emulated instant, so the phase comes out
 * identically zero however carefully it is derived. Inventing it would be
 * fiction rather than a model. What IS exactly reproducible is the register
 * sequence that produces it, and timer_ctl0_write() reports that instead.
 */
#define LATE_LATCH_REPORTS  4

typedef struct
{
  uint32_t    base;
  const char *name;
  uint32_t    psc;      // what the prescaler is dividing by right now
  uint64_t    due_ns;   // when a parked write takes over; 0 = nothing parked
  bool        enabled;
  int         warned;
  int         late_latches;
} Prescaler;

/* TIMER0_Type and TIMER7_Type are the same advanced-timer layout, so one set
 * of offsets serves both. */
static Prescaler g_tim0 = { TIMER0_BASE, "TIMER0", 0, 0, false, 0, 0 };
static Prescaler g_tim7 = { TIMER7_BASE, "TIMER7", 0, 0, false, 0, 0 };

/*- Prototypes --------------------------------------------------------------*/
static void gpio_outputs_changed(int port);
static void dma_sync(uint64_t now_ns);
static void spif_select(bool selected);
static uint8_t spif_byte(uint8_t in);

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
  // Reset value of both prescalers is 0, and nothing is parked yet
  g_tim0.psc = g_tim7.psc = 0;
  g_tim0.due_ns = g_tim7.due_ns = 0;
  g_tim0.enabled = g_tim7.enabled = false;
  g_tim0.warned = g_tim7.warned = 0;
  g_tim0.late_latches = g_tim7.late_latches = 0;
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

  if (port == 0)                            // PA3 is the flash chip select
    spif_select(0 == pin_level(0, 3));
}

/*- SPI flash ---------------------------------------------------------------*/

/*
 * The serial NOR part on SPI0, chip select on PA3. Modelled because it is the
 * only storage on the board the firmware image cannot reach, so anything that
 * comes to live in it - asset packs, saved waveforms - has to be developable
 * without the hardware in front of you.
 *
 * The JEDEC ID answered here is the one read off the real unit on 2026-07-29
 * (c8 40 17, GigaDevice, 8 MB), not the W25Q64JV in doc/Hardware.md. A model
 * that answered ef 40 17 would hide exactly the disagreement that matters.
 *
 * Timing is not modelled: a program or an erase completes before the command
 * that started it returns, and the status register never reports itself busy.
 * The firmware's waits are bounded and tolerate that; what this model is for
 * is the protocol and the contents.
 */

#define SPIF_SIZE       (8u << 20)
#define SPIF_PAGE       256u
#define SPIF_SECTOR     4096u
#define SPIF_BLOCK      65536u

enum
{
  SPIF_CMD_PAGE_PROGRAM = 0x02,
  SPIF_CMD_READ_DATA    = 0x03,
  SPIF_CMD_READ_STATUS  = 0x05,
  SPIF_CMD_WRITE_ENABLE = 0x06,
  SPIF_CMD_ERASE_SECTOR = 0x20,
  SPIF_CMD_ERASE_BLOCK  = 0xd8,
  SPIF_CMD_ERASE_CHIP   = 0xc7,
  SPIF_CMD_JEDEC_ID     = 0x9f,
};

static const uint8_t g_spif_id[3] = { 0xc8, 0x40, 0x17 };

static struct
{
  uint8_t *data;
  bool selected;
  bool wel;                     /* write enable latch */
  int phase;                    /* bytes seen since chip select fell */
  uint8_t cmd;
  uint32_t addr;
} g_spif;

//-----------------------------------------------------------------------------
uint8_t *board_spi_flash(void)
{
  if (!g_spif.data)
  {
    g_spif.data = malloc(SPIF_SIZE);

    if (g_spif.data)
      memset(g_spif.data, 0xff, SPIF_SIZE);
  }

  return g_spif.data;
}

//-----------------------------------------------------------------------------
uint32_t board_spi_flash_size(void)
{
  return SPIF_SIZE;
}

//-----------------------------------------------------------------------------
static void spif_select(bool selected)
{
  if (selected == g_spif.selected)
    return;

  g_spif.selected = selected;

  if (selected)
  {
    g_spif.phase = 0;
    g_spif.cmd = 0;
    g_spif.addr = 0;
  }
  else if (SPIF_CMD_PAGE_PROGRAM == g_spif.cmd)
  {
    // The rising edge is what commits a page program, and the latch clears
    // with it: the next write has to arm it again.
    g_spif.wel = false;
  }
}

//-----------------------------------------------------------------------------
// One byte in, one byte out, which is what an SPI frame is. The firmware
// writes DATA and then reads it back, so the answer to byte N is what the
// read after write N returns.
static uint8_t spif_byte(uint8_t in)
{
  uint8_t *mem = board_spi_flash();
  uint8_t out = 0xff;
  int phase;

  if (!g_spif.selected || !mem)
    return 0xff;

  phase = g_spif.phase++;

  if (0 == phase)
  {
    g_spif.cmd = in;

    switch (in)
    {
      case SPIF_CMD_WRITE_ENABLE:
        g_spif.wel = true;
        break;

      case SPIF_CMD_ERASE_CHIP:
        if (g_spif.wel)
        {
          memset(mem, 0xff, SPIF_SIZE);
          g_spif.wel = false;
          emu_log("SPI flash: chip erase");
        }
        break;

      default:
        break;
    }

    return 0xff;
  }

  switch (g_spif.cmd)
  {
    case SPIF_CMD_JEDEC_ID:
      out = (phase <= 3) ? g_spif_id[phase - 1] : 0xff;
      break;

    // Bit 0 is WIP and bit 1 is WEL. Nothing here is ever in progress.
    case SPIF_CMD_READ_STATUS:
      out = g_spif.wel ? 0x02 : 0x00;
      break;

    case SPIF_CMD_READ_DATA:
    case SPIF_CMD_PAGE_PROGRAM:
    case SPIF_CMD_ERASE_SECTOR:
    case SPIF_CMD_ERASE_BLOCK:
      if (phase <= 3)
      {
        g_spif.addr = (g_spif.addr << 8) | in;

        // The erases carry an address and no data, so they happen as soon as
        // that address is complete
        if (3 == phase)
        {
          g_spif.addr %= SPIF_SIZE;

          if (SPIF_CMD_ERASE_SECTOR == g_spif.cmd && g_spif.wel)
          {
            memset(mem + (g_spif.addr & ~(SPIF_SECTOR - 1)), 0xff, SPIF_SECTOR);
            g_spif.wel = false;
          }
          else if (SPIF_CMD_ERASE_BLOCK == g_spif.cmd && g_spif.wel)
          {
            memset(mem + (g_spif.addr & ~(SPIF_BLOCK - 1)), 0xff, SPIF_BLOCK);
            g_spif.wel = false;
          }
        }

        break;
      }

      if (SPIF_CMD_READ_DATA == g_spif.cmd)
      {
        out = mem[g_spif.addr];
        g_spif.addr = (g_spif.addr + 1) % SPIF_SIZE;
      }
      else if (SPIF_CMD_PAGE_PROGRAM == g_spif.cmd && g_spif.wel)
      {
        // Programming can only clear bits, and it wraps within the 256 byte
        // page rather than running on into the next one. Both are the kind of
        // thing a driver gets wrong once.
        uint32_t page = g_spif.addr & ~(SPIF_PAGE - 1);

        mem[g_spif.addr] &= in;
        g_spif.addr = page + ((g_spif.addr + 1) % SPIF_PAGE);
      }

      break;

    default:
      break;
  }

  return out;
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

/*- TIMER0/TIMER7 prescalers ------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint64_t timer_counter_period_ns(const Prescaler *t, uint64_t psc)
{
  uint64_t car = pget(t->base + offsetof(TIMER7_Type, CAR)) & 0xffff;

  return ((psc + 1) * (car + 1) * 1000000000ull + TIMER_APB2_HZ / 2) /
      TIMER_APB2_HZ;
}

//-----------------------------------------------------------------------------
// The divider in force at `now_ns`, applying a parked write once its update
// event has come due.
static uint64_t timer_psc(Prescaler *t, uint64_t now_ns)
{
  if (t->due_ns && now_ns >= t->due_ns)
  {
    t->psc = pget(t->base + offsetof(TIMER7_Type, PSC)) & 0xffff;
    t->due_ns = 0;
  }

  return t->psc;
}

//-----------------------------------------------------------------------------
// An update event: the parked divider takes over and the prescaler counter
// restarts from zero.
static void timer_update_event(Prescaler *t)
{
  t->psc = pget(t->base + offsetof(TIMER7_Type, PSC)) & 0xffff;
  t->due_ns = 0;
}

//-----------------------------------------------------------------------------
static void timer_ctl0_write(Prescaler *t, uint32_t value, uint64_t now_ns)
{
  bool enable = (value & TIMER7_CTL0_CEN_Msk) != 0;
  uint64_t parked = pget(t->base + offsetof(TIMER7_Type, PSC)) & 0xffff;

  timer_psc(t, now_ns);

  if (enable && !t->enabled && parked != t->psc)
  {
    // Started with a different divider parked and no update event in between.
    // The prescaler serves one more counter period at the OLD value and
    // latches at its own first overflow - and clears its counter there, which
    // is where the TIMER0/TIMER7 phase is lost on hardware. Both timers doing
    // this independently is the defect; see the block comment above.
    t->due_ns = now_ns + timer_counter_period_ns(t, t->psc);
    t->late_latches++;

    // Every distinct divider change is worth naming - the one that bit was
    // PSC 1 -> 3, and only reporting the first occurrence would have shown
    // the harmless boot-time 0 -> 1 instead. Capped so a pan that retunes the
    // timebase every frame cannot bury the log.
    if (t->warned < LATE_LATCH_REPORTS)
    {
      t->warned++;
      emu_log("%s: CEN set with PSC %llu parked over %llu and no update event"
          " - the prescaler latches at its own first overflow, so TIMER0 and"
          " TIMER7 end up out of phase (needles on hardware). Raise SWEVG.UPG"
          " before CEN.%s",
          t->name, (unsigned long long)parked, (unsigned long long)t->psc,
          t->warned == LATE_LATCH_REPORTS ? " Further ones counted silently." : "");
    }
  }

  t->enabled = enable;
}

//-----------------------------------------------------------------------------
// One DMA request per TIMER7 update event. The firmware sets CAR = 1 and moves
// PSC to pick the timebase, so this is where "sample rate" actually comes from.
static uint64_t dma_request_period_ns(void)
{
  uint64_t psc = timer_psc(&g_tim7, cpu_now_ns());
  uint64_t car = pget(TIMER7_BASE + offsetof(TIMER7_Type, CAR)) & 0xffff;

  if (!g_tim7.enabled)
    return 0;  // the timer is stopped: no requests at all

  return ((psc + 1) * (car + 1) * 1000000000ull + TIMER_APB2_HZ / 2) /
      TIMER_APB2_HZ;
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

    // The controller is always ready to shift - there is no baud rate here,
    // and flash.c's bounded polls would otherwise burn their whole limit per
    // byte. DATA reads back whatever the chip answered to the byte the
    // firmware last wrote into it (see the write side).
    case SPI0_BASE:
      if (off == offsetof(SPI0_Type, STAT))
        word = SPI0_STAT_TBE_Msk | SPI0_STAT_RBNE_Msk;
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
    // A write to DATA is a frame: the byte goes to the chip and the chip's
    // answer is parked in the same register, which is where the firmware's
    // next read of it will find it.
    case SPI0_BASE:
      if (off == offsetof(SPI0_Type, DATA))
      {
        pset(addr, spif_byte((uint8_t)word));
        return;
      }
      break;

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

    case TIMER0_BASE:
      // The AD9288 encode clocks. Nothing downstream reads them - the model
      // samples the signal directly - but the prescaler is tracked so the
      // TIMER0/TIMER7 latch check has both halves to compare.
      pset(addr, word);

      if (off == offsetof(TIMER7_Type, SWEVG) && (word & TIMER0_SWEVG_UPG_Msk))
        timer_update_event(&g_tim0);
      else if (off == offsetof(TIMER7_Type, CTL0))
        timer_ctl0_write(&g_tim0, word, cpu_now_ns());

      return;

    case TIMER7_BASE:
      // Changing the sample clock while the channel runs must not retroactively
      // reinterpret samples already produced
      dma_sync(cpu_now_ns());
      pset(addr, word);

      if (off == offsetof(TIMER7_Type, SWEVG) && (word & TIMER7_SWEVG_UPG_Msk))
        timer_update_event(&g_tim7);
      else if (off == offsetof(TIMER7_Type, CTL0))
        timer_ctl0_write(&g_tim7, word, cpu_now_ns());

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
// How many times a timer was started with a divider change parked and no
// update event to hand it over. Nonzero means the firmware is leaving the
// encode clock and the DMA strobe to latch independently; see the Prescaler
// comment. Reported at the end of every run, since the symptom it causes on
// hardware is out of this model's reach.
int board_late_prescaler_latches(void)
{
  return g_tim0.late_latches + g_tim7.late_latches;
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
      "dma %s buf%d cnt %u/%u %s  irq %s/%s  %s %lluns  psc %u/%u late %d/%d"
      "  relay %d dac %d",
      g_dma.enabled ? "run" : "off", dma_mbs() ? 1 : 0,
      g_dma.cnt, g_dma.reload, g_dma.ftf ? "COMPLETION UNSERVICED" : "flowing",
      (g_nvic_enabled[word] & bit) ? "en" : "dis",
      (g_nvic_pending[word] & bit) ? "PENDING" : "idle",
      period ? "sampling" : "sample clock stopped",
      (unsigned long long)period,
      g_tim0.psc, g_tim7.psc, g_tim0.late_latches, g_tim7.late_latches,
      g_afe.relay_scale, g_afe.dac_code);
}
