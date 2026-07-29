/*
 * Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
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

/*
 * The SPI NOR part on PA3/PA5-PA7 - 8 MB the firmware image cannot reach and
 * the only storage on this board that is not already spoken for.
 *
 * What is actually fitted, read off this unit on 2026-07-29: JEDEC ID
 * c8 40 17, i.e. GigaDevice, 2^23 bytes. doc/Hardware.md says W25Q64JV, and
 * that is worth knowing: the probe this file used to carry demanded ef 40 17
 * and called error() on anything else, so it would have refused to boot here.
 * That probe was commented out of main() upstream in 2021 with no reason
 * recorded; this is the likeliest one.
 *
 * Everything below is therefore written so that a part which is absent, is
 * wired differently, or answers with an ID nobody has seen degrades to a
 * report rather than to a dead scope:
 *
 *   - every transfer is bounded. On silicon RBNE sets when the frame has been
 *     shifted whether or not anything drives MISO, but a board model that does
 *     not implement SPI0 would otherwise spin here forever;
 *   - MISO is pulled up, so an absent chip reads a definite 0xff instead of
 *     whatever noise a floating input picks up;
 *   - a timeout marks the bus dead and every later command returns straight
 *     away rather than paying the full poll limit again;
 *   - nothing here calls error(), and nothing here writes unless it was asked
 *     to. The viewer only reads.
 *
 * Erase and program are bounded by timer_us(), NOT timer_ms(): the millisecond
 * counter only advances inside timer_task(), so a blocking loop that waits on
 * it waits forever.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "gd32f4xx.h"
#include "hal_gpio.h"
#include "common.h"
#include "utils.h"
#include "timer.h"
#include "flash.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(CS,       A, 3)
HAL_GPIO_PIN(CLK,      A, 5)
HAL_GPIO_PIN(MISO,     A, 6)
HAL_GPIO_PIN(MOSI,     A, 7)

enum
{
  CMD_PAGE_PROGRAM     = 0x02,
  CMD_READ_DATA        = 0x03,
  CMD_FAST_READ        = 0x0b,
  CMD_READ_STATUS      = 0x05,
  CMD_WRITE_ENABLE     = 0x06,
  CMD_ERASE_SECTOR     = 0x20,
  CMD_ERASE_BLOCK      = 0xd8,
  CMD_ERASE_CHIP       = 0xc7,
  CMD_READ_JEDEC_ID    = 0x9f,
};

enum
{
  STATUS_WIP           = (1 << 0),   // write/erase in progress
  STATUS_WEL           = (1 << 1),   // write enable latch
};

/*
 * Two clocks, and the reason there are two.
 *
 * PSC is the SPI0 prescaler off APB2, which is SYSCLK/2 = 125 MHz at the
 * overclock: 1 is /4 = 31.25 MHz and 2 is /8 = 15.625 MHz.
 *
 * Reads run at 31.25. That was measured, not assumed - it is what took DOOM's
 * texture streaming from 17 fps to 20-55, and every read since has been
 * byte-exact, including a 100 KB pack the renderer would show the seams of.
 *
 * Writes do not. At 31.25 a page program arrives corrupted, differently every
 * time and always with bits missing rather than added, which is the signature
 * of a part sampling MOSI before it has settled. It is specific to the long
 * unbroken data phase: the three address bytes of a read go out on the same
 * wire at the same rate and land correctly every time, so this is not a bus
 * that cannot do 31.25 - it is 256 bytes of it back to back that cannot.
 *
 * Found on 2026-07-29 by pushing the same 256 byte pattern three times and
 * getting three different answers - 3, 44 and 130 bytes correct.
 *
 * Costing nothing: a page program is 256 bytes, and the part then spends up to
 * 3 ms inside itself finishing it. Halving the clock adds 4 us to that.
 */
#define SPI_PSC_READ    1u
#define SPI_PSC_WRITE   2u

// One byte at 15.6 MHz is ~0.5 us, a few hundred core cycles. Anything past
// this many polls means the peripheral is not shifting at all.
#define POLL_LIMIT      100000

// Datasheet maxima with room to spare. A part that is still busy past these is
// not coming back, and the caller gets a false rather than a hang.
#define PROGRAM_TIMEOUT_US    10000u        // one 256 byte page, max 3 ms
#define SECTOR_TIMEOUT_US     1000000u      // 4 KB, max 400 ms
#define BLOCK_TIMEOUT_US      4000000u      // 64 KB, max 2 s

#define HEAD_BYTES      8

enum
{
  STATE_UNPROBED,
  STATE_PRESENT,
  STATE_ABSENT,
  STATE_DEAD,
};

/*- Variables ---------------------------------------------------------------*/
static uint8_t g_state = STATE_UNPROBED;
static uint8_t g_id[3];
static uint32_t g_size;
static uint8_t g_head[HEAD_BYTES];
static bool g_head_valid;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static bool spi_xfer(uint8_t out, uint8_t *in)
{
  uint32_t limit = POLL_LIMIT;
  uint8_t value;

  while (0 == SPI0->STAT_b.TBE)
  {
    if (0 == --limit)
      return false;
  }

  SPI0->DATA = out;

  limit = POLL_LIMIT;

  while (0 == SPI0->STAT_b.RBNE)
  {
    if (0 == --limit)
      return false;
  }

  value = SPI0->DATA;

  if (in)
    *in = value;

  return true;
}

//-----------------------------------------------------------------------------
// The prescaler, changed between frames and never during one. SPIEN goes down
// first: the manual says the control bits are only to be written while the
// peripheral is disabled, and a prescaler that changes mid-shift is a clock
// glitch the part would read as a bit.
static void spi_clock(uint32_t psc)
{
  uint32_t ctl = SPI0->CTL0;

  if (psc == ((ctl >> SPI0_CTL0_PSC_Pos) & 7u))
    return;

  SPI0->CTL0 = ctl & ~SPI0_CTL0_SPIEN_Msk;
  SPI0->CTL0 = (ctl & ~(7u << SPI0_CTL0_PSC_Pos)) | (psc << SPI0_CTL0_PSC_Pos);
}

//-----------------------------------------------------------------------------
// One command, with an optional 24-bit address and an optional data phase in
// either direction. Chip select frames the whole thing: these parts latch a
// command on the falling edge of CS and abandon it on the rising one, which is
// also what commits a page program.
static bool flash_command(uint8_t cmd, uint32_t addr, bool has_addr,
    const uint8_t *tx, uint8_t *rx, uint32_t size)
{
  bool ok;

  if (STATE_DEAD == g_state)
    return false;

  // Only a page program has a data phase this machine drives, and it is the
  // one thing on this bus that does not survive the full rate. Everything
  // else - the reads, the erases, the one-byte commands - stays fast.
  spi_clock((CMD_PAGE_PROGRAM == cmd) ? SPI_PSC_WRITE : SPI_PSC_READ);

  HAL_GPIO_CS_clr();

  ok = spi_xfer(cmd, NULL);

  if (ok && has_addr)
  {
    ok = spi_xfer((addr >> 16) & 0xff, NULL) &&
         spi_xfer((addr >> 8) & 0xff, NULL) &&
         spi_xfer(addr & 0xff, NULL);

    // Fast read clocks one byte of nothing between the address and the data,
    // which is what buys it the higher rate: the part uses that byte to get
    // its output ready.
    if (ok && CMD_FAST_READ == cmd)
      ok = spi_xfer(0, NULL);
  }

  for (uint32_t i = 0; ok && i < size; i++)
    ok = spi_xfer(tx ? tx[i] : 0, rx ? &rx[i] : NULL);

  HAL_GPIO_CS_set();

  if (!ok)
    g_state = STATE_DEAD;

  return ok;
}

//-----------------------------------------------------------------------------
static bool read_status(uint8_t *status)
{
  return flash_command(CMD_READ_STATUS, 0, false, NULL, status, 1);
}

//-----------------------------------------------------------------------------
// The latch clears itself after every program and erase, so this has to be
// re-armed per operation. Reading it back catches a part that is write
// protected in hardware, which otherwise looks like a program that silently
// did nothing.
static bool write_enable(void)
{
  uint8_t status;

  if (!flash_command(CMD_WRITE_ENABLE, 0, false, NULL, NULL, 0))
    return false;

  if (!read_status(&status))
    return false;

  return 0 != (status & STATUS_WEL);
}

//-----------------------------------------------------------------------------
bool flash_busy(void)
{
  uint8_t status;

  if (!read_status(&status))
    return false;

  return 0 != (status & STATUS_WIP);
}

//-----------------------------------------------------------------------------
bool flash_wait_ready(uint32_t timeout_us)
{
  uint32_t start = timer_us();

  for (;;)
  {
    uint8_t status;

    if (!read_status(&status))
      return false;

    if (0 == (status & STATUS_WIP))
      return true;

    // Modular arithmetic on a free-running counter: correct across the wrap
    if (timer_us() - start > timeout_us)
      return false;
  }
}

//-----------------------------------------------------------------------------
static void flash_probe(void)
{
  if (!flash_command(CMD_READ_JEDEC_ID, 0, false, NULL, g_id, sizeof(g_id)))
    return;                             // flash_command has recorded the death

  // Nothing on the bus reads as the pull-up, or as zero if something holds the
  // line down. No real manufacturer ID is 0x00 or 0xff - JEDEC reserves both.
  if ((0x00 == g_id[0] || 0xff == g_id[0]) && g_id[0] == g_id[1] &&
      g_id[1] == g_id[2])
  {
    g_state = STATE_ABSENT;
    return;
  }

  g_state = STATE_PRESENT;

  // The capacity byte is log2 of the size in bytes on every SPI NOR part that
  // follows the convention: 0x17 is 2^23, the 8 MB this board carries.
  if (g_id[2] >= 0x10 && g_id[2] <= 0x1f)
    g_size = 1u << g_id[2];

  g_head_valid = flash_read(0, g_head, sizeof(g_head));
}

//-----------------------------------------------------------------------------
void flash_init(void)
{
  HAL_GPIO_CS_out();
  HAL_GPIO_CS_set();
  HAL_GPIO_CLK_alt(5);
  HAL_GPIO_MISO_alt(5);
  HAL_GPIO_MOSI_alt(5);
  HAL_GPIO_MISO_pullup();

  RCU->APB2EN_b.SPI0EN = 1;

  // PCLK/4 to start with, which is the rate everything but a page program
  // runs at - see SPI_PSC_READ and the note beside it. APB2 is SYSCLK/2, so
  // 31.25 MHz at the 250 MHz overclock.
  //
  // This was PCLK/8 while reads went through the plain 03h command, which
  // these parts rate for 50-80 MHz and which did not deserve the risk at a
  // clock nobody had measured. Everything that reads in bulk now uses fast
  // read (0Bh) instead, rated past 100 MHz, so the limit is the controller
  // rather than the flash - and DOOM streams its textures through here, where
  // the difference is milliseconds a frame.
  SPI0->CTL0 = SPI0_CTL0_SPIEN_Msk | SPI0_CTL0_MSTMOD_Msk |
      (SPI_PSC_READ << SPI0_CTL0_PSC_Pos) |
      SPI0_CTL0_CKPH_Msk | SPI0_CTL0_CKPL_Msk | SPI0_CTL0_SWNSSEN_Msk | SPI0_CTL0_SWNSS_Msk;

  delay_cycles(100);

  flash_probe();
}

//-----------------------------------------------------------------------------
bool flash_present(void)
{
  return STATE_PRESENT == g_state;
}

//-----------------------------------------------------------------------------
uint32_t flash_size(void)
{
  return g_size;
}

//-----------------------------------------------------------------------------
const uint8_t *flash_jedec_id(void)
{
  return g_id;
}

//-----------------------------------------------------------------------------
// One range check for every entry point, so an address past the end of the
// part can never wrap into the start of it. A part whose capacity byte made no
// sense has g_size == 0 and refuses everything: better no access than access
// to an address the chip will alias somewhere unpredictable.
static bool range_ok(uint32_t addr, uint32_t size)
{
  if (STATE_PRESENT != g_state || 0 == g_size)
    return false;

  return addr < g_size && size <= g_size - addr;
}

//-----------------------------------------------------------------------------
bool flash_read(uint32_t addr, uint8_t *data, uint32_t size)
{
  if (!range_ok(addr, size))
    return false;

  return flash_command(CMD_FAST_READ, addr, true, NULL, data, size);
}

//-----------------------------------------------------------------------------
// Page program stops at the end of a 256 byte page and wraps to its start
// rather than carrying on, so a write that straddles one has to be split. Note
// that NOR programming can only clear bits: this writes data & old, and the
// caller is responsible for the erase.
bool flash_write(uint32_t addr, const uint8_t *data, uint32_t size)
{
  if (!range_ok(addr, size))
    return false;

  while (size > 0)
  {
    uint32_t chunk = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);

    if (chunk > size)
      chunk = size;

    if (!write_enable())
      return false;

    if (!flash_command(CMD_PAGE_PROGRAM, addr, true, data, NULL, chunk))
      return false;

    if (!flash_wait_ready(PROGRAM_TIMEOUT_US))
      return false;

    addr += chunk;
    data += chunk;
    size -= chunk;
  }

  return true;
}

//-----------------------------------------------------------------------------
static bool flash_erase(uint8_t cmd, uint32_t addr, uint32_t timeout_us)
{
  if (!range_ok(addr, 1))
    return false;

  if (!write_enable())
    return false;

  if (!flash_command(cmd, addr, true, NULL, NULL, 0))
    return false;

  return flash_wait_ready(timeout_us);
}

//-----------------------------------------------------------------------------
bool flash_erase_sector(uint32_t addr)
{
  return flash_erase(CMD_ERASE_SECTOR, addr & ~(FLASH_SECTOR_SIZE - 1),
      SECTOR_TIMEOUT_US);
}

//-----------------------------------------------------------------------------
bool flash_erase_block(uint32_t addr)
{
  return flash_erase(CMD_ERASE_BLOCK, addr & ~(FLASH_BLOCK_SIZE - 1),
      BLOCK_TIMEOUT_US);
}

//-----------------------------------------------------------------------------
// Issues the erase and returns; the part is busy for tens of seconds
// afterwards. Blocking that long would freeze the main loop, so the caller
// polls flash_busy() from its own task instead.
bool flash_erase_chip_start(void)
{
  if (STATE_PRESENT != g_state)
    return false;

  if (!write_enable())
    return false;

  return flash_command(CMD_ERASE_CHIP, 0, false, NULL, NULL, 0);
}

//-----------------------------------------------------------------------------
void flash_get_state(char *buf, int size)
{
  const char *vendor = "id";

  switch (g_state)
  {
    case STATE_UNPROBED:
      snprintf(buf, size, "SPI flash: not probed");
      return;

    case STATE_DEAD:
      snprintf(buf, size, "SPI flash: SPI0 NOT SHIFTING");
      return;

    case STATE_ABSENT:
      snprintf(buf, size, "SPI flash: NO DEVICE (bus reads %02X)", g_id[0]);
      return;

    default:
      break;
  }

  switch (g_id[0])
  {
    case 0xef: vendor = "Winbond";    break;
    case 0xc8: vendor = "GigaDevice"; break;
    case 0x20: vendor = "Micron";     break;
    case 0x1c: vendor = "EON";        break;
    case 0x5e: vendor = "Zbit";       break;
    case 0x68: vendor = "Boya";       break;
    case 0x85: vendor = "Puya";       break;
    default:                          break;
  }

  if (g_size)
    snprintf(buf, size, "SPI flash: %s %02X%02X%02X, %u MB", vendor,
        g_id[0], g_id[1], g_id[2], (unsigned)(g_size >> 20));
  else
    snprintf(buf, size, "SPI flash: %s %02X%02X%02X, size ?", vendor,
        g_id[0], g_id[1], g_id[2]);
}

//-----------------------------------------------------------------------------
void flash_get_head(char *buf, int size)
{
  bool erased = true;
  int n;

  if (!g_head_valid)
  {
    snprintf(buf, size, "SPI @0: --");
    return;
  }

  n = snprintf(buf, size, "SPI @0:");

  for (int i = 0; i < HEAD_BYTES && n > 0 && n < size; i++)
  {
    n += snprintf(buf + n, size - n, " %02X", g_head[i]);

    if (0xff != g_head[i])
      erased = false;
  }

  // Distinguishes "the read worked and the chip is blank" from "the read
  // worked and there is something in there" - the stock firmware's saved
  // waveforms would show up as data.
  if (erased && n > 0 && n < size)
    snprintf(buf + n, size - n, " (erased)");
}
