/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SPI flash loader: the fast way to get a PC's bytes onto the serial part.
 *
 * The slow way is tools/spiflash.tcl, which halts the core and drives SPI0 a
 * byte at a time from the debugger - three SWD transactions per byte, about a
 * kilobyte a second. Fine for a ROM, hopeless for a WAD.
 *
 * This is the other end of the same problem. The debugger is good at exactly
 * one thing here: moving a block of bytes into SRAM quickly, which is how it
 * programs the internal flash too. So the host drops a payload into a mailbox
 * in SRAM, writes a command word last, and this application - which is already
 * running on the device, with a working driver and no SWD in the way - does
 * the shifting. The core is never halted, so what you get on screen is live
 * progress rather than a frozen scope.
 *
 * Why an application and not a background task: nothing should be able to
 * write to that chip because a debugger happened to be attached. The mailbox
 * only exists while somebody has deliberately opened this screen, and the
 * screen says what is happening the whole time.
 *
 * The mailbox lives at the bottom of main SRAM, which belongs to the capture
 * ring while the scope runs and to whatever application is open otherwise -
 * the same 120 KB DOOM takes. That is safe here for the same reason it is safe
 * there: acquisition is stopped before an application is entered.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "buttons.h"
#include "timer.h"
#include "utils.h"
#include "flash.h"
#include "capture.h"
#include "spi_tool.h"

/*- Definitions -------------------------------------------------------------*/
#define MAILBOX_BASE        0x20000000u
#define MAILBOX_MAGIC       0x4C495053u     // 'SPIL', little endian
#define MAILBOX_DATA_SIZE   (32 * 1024)

// Commands the host writes. Anything else is rejected, which is what keeps a
// half-written mailbox from being executed as something.
enum
{
  CMD_NONE        = 0,
  CMD_ERASE       = 1,        // every sector covering addr..addr+len
  CMD_PROGRAM     = 2,        // len bytes from data[] to addr
  CMD_READ        = 3,        // len bytes from addr into data[]
  CMD_CRC         = 4,        // crc32 of len bytes at addr, into result
  CMD_ERASE_CHIP  = 5,
};

enum
{
  ST_IDLE         = 0,
  ST_BUSY         = 1,
  ST_OK           = 2,
  ST_FAILED       = 3,
  ST_BAD_REQUEST  = 4,
};

// Written by the firmware, polled and filled by the host. Volatile because the
// debugger changes it behind the compiler's back - without it the poll below
// is hoisted out of the loop and this screen waits forever.
typedef struct
{
  volatile uint32_t magic;      // firmware writes it; absence means not ready
  volatile uint32_t command;
  volatile uint32_t address;
  volatile uint32_t length;
  volatile uint32_t status;
  volatile uint32_t result;     // crc, or the count of bytes moved
  volatile uint32_t sequence;   // one per completed command
  volatile uint32_t reserved;
  volatile uint8_t data[MAILBOX_DATA_SIZE];
} mailbox_t;

#define MAILBOX             ((mailbox_t *)MAILBOX_BASE)

#define FG                  LCD_COLOR(200, 200, 200)
#define DIM                 LCD_COLOR(120, 120, 120)
#define HI                  LCD_COLOR(255, 190, 60)
#define OK                  LCD_COLOR(80, 220, 120)
#define BAD                 LCD_COLOR(230, 80, 60)

/*- Variables ---------------------------------------------------------------*/
static uint32_t g_last_sequence;
static uint32_t g_commands;
static uint32_t g_bytes_written;
static uint32_t g_bytes_read;
static bool g_dirty;
static char g_last[48] = "waiting for the host";
static int g_last_color;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The zlib polynomial, bitwise: the hardware unit on this part is CRC-32/
// MPEG-2 over whole words, which is neither what a host's crc32 gives nor
// defined for a length that is not a multiple of four. Eight shifts a byte
// over 8 MB is about half a second, and it only runs when asked.
static uint32_t crc32_bytes(uint32_t crc, const uint8_t *data, uint32_t size)
{
  crc = ~crc;

  while (size--)
  {
    crc ^= *data++;

    for (int i = 0; i < 8; i++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1));
  }

  return ~crc;
}

//-----------------------------------------------------------------------------
static void note(const char *text, int color)
{
  snprintf(g_last, sizeof(g_last), "%s", text);
  g_last_color = color;
  g_dirty = true;
}

//-----------------------------------------------------------------------------
static bool range_valid(uint32_t addr, uint32_t len, bool needs_buffer)
{
  uint32_t size = flash_size();

  if (0 == size || addr >= size || len > size - addr)
    return false;

  return !needs_buffer || len <= MAILBOX_DATA_SIZE;
}

//-----------------------------------------------------------------------------
// The top of the part holds the settings store and the one page that was on
// the chip before this firmware ever wrote to it. A host that names an address
// up there is either doing something deliberate or has miscalculated an
// offset, and the second is much more likely - so it has to say which by
// putting the override in a field it could not have set by accident.
#define UNSAFE_KEY          0x0FF77E57u     // "off-test"

static bool region_writable(uint32_t addr, uint32_t len)
{
  if (addr + len <= FLASH_RESERVED_BASE)
    return true;

  return UNSAFE_KEY == MAILBOX->reserved;
}

//-----------------------------------------------------------------------------
static uint32_t do_erase(uint32_t addr, uint32_t len)
{
  uint32_t first = addr & ~(FLASH_SECTOR_SIZE - 1);
  uint32_t last = (addr + len - 1) & ~(FLASH_SECTOR_SIZE - 1);

  for (uint32_t s = first; s <= last; s += FLASH_SECTOR_SIZE)
  {
    if (!flash_erase_sector(s))
      return s;                     // where it stopped, for the report
  }

  return 0;
}

//-----------------------------------------------------------------------------
// Reads back in mailbox-sized bites rather than all at once: the buffer is
// 32 KB and the request can name the whole 8 MB.
static bool do_crc(uint32_t addr, uint32_t len, uint32_t *out)
{
  uint8_t scratch[512];
  uint32_t crc = 0;

  while (len > 0)
  {
    uint32_t chunk = (len > sizeof(scratch)) ? sizeof(scratch) : len;

    if (!flash_read(addr, scratch, chunk))
      return false;

    crc = crc32_bytes(crc, scratch, chunk);
    addr += chunk;
    len -= chunk;
  }

  *out = crc;

  return true;
}

//-----------------------------------------------------------------------------
static void run_command(uint32_t cmd)
{
  uint32_t addr = MAILBOX->address;
  uint32_t len = MAILBOX->length;
  uint32_t result = 0;
  bool ok = false;
  char text[48];

  MAILBOX->status = ST_BUSY;

  switch (cmd)
  {
    case CMD_ERASE:
      if (!range_valid(addr, len, false))
        break;

      if (!region_writable(addr, len))
      {
        note("refused: reserved region", BAD);
        MAILBOX->status = ST_BAD_REQUEST;
        MAILBOX->command = CMD_NONE;
        return;
      }

      result = do_erase(addr, len);
      ok = (0 == result);

      snprintf(text, sizeof(text), ok ? "erased %luK at %08lX" :
          "ERASE FAILED at %08lX", (unsigned long)(len / 1024),
          (unsigned long)(ok ? addr : result));
      note(text, ok ? OK : BAD);
      break;

    case CMD_PROGRAM:
      if (!range_valid(addr, len, true))
        break;

      if (!region_writable(addr, len))
      {
        note("refused: reserved region", BAD);
        MAILBOX->status = ST_BAD_REQUEST;
        MAILBOX->command = CMD_NONE;
        return;
      }

      ok = flash_write(addr, (const uint8_t *)MAILBOX->data, len);

      if (ok)
        g_bytes_written += len;

      snprintf(text, sizeof(text), ok ? "wrote %lu at %08lX" :
          "WRITE FAILED at %08lX", (unsigned long)(ok ? len : addr),
          (unsigned long)addr);
      note(text, ok ? OK : BAD);
      break;

    case CMD_READ:
      if (!range_valid(addr, len, true))
        break;

      ok = flash_read(addr, (uint8_t *)MAILBOX->data, len);

      if (ok)
        g_bytes_read += len;

      snprintf(text, sizeof(text), ok ? "read %lu at %08lX" :
          "READ FAILED at %08lX", (unsigned long)(ok ? len : addr),
          (unsigned long)addr);
      note(text, ok ? OK : BAD);
      break;

    case CMD_CRC:
      if (!range_valid(addr, len, false))
        break;

      ok = do_crc(addr, len, &result);

      snprintf(text, sizeof(text), ok ? "crc %08lX" : "CRC READ FAILED",
          (unsigned long)result);
      note(text, ok ? OK : BAD);
      break;

    case CMD_ERASE_CHIP:
      ok = flash_erase_chip_start() && flash_wait_ready(120000000u);
      note(ok ? "chip erased" : "CHIP ERASE FAILED", ok ? OK : BAD);
      break;

    default:
      note("unknown command", BAD);
      MAILBOX->status = ST_BAD_REQUEST;
      MAILBOX->command = CMD_NONE;
      return;
  }

  if (!ok && ST_BUSY == MAILBOX->status &&
      (CMD_ERASE == cmd || CMD_PROGRAM == cmd || CMD_READ == cmd ||
       CMD_CRC == cmd))
  {
    // Fell out of a range check rather than out of the part
    if (!range_valid(addr, len, CMD_PROGRAM == cmd || CMD_READ == cmd))
      note("bad address or length", BAD);
  }

  MAILBOX->result = result;
  MAILBOX->status = ok ? ST_OK : ST_FAILED;
  MAILBOX->sequence = ++g_commands;
  MAILBOX->command = CMD_NONE;        // last, so the host sees a settled reply
}

//-----------------------------------------------------------------------------
static void draw_screen(void)
{
  char buf[64];
  int y = 46;

  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);

  lcd_set_font(FONT_LARGE);
  lcd_set_color(LCD_BLACK_COLOR, HI);
  lcd_puts(10, 12, "SPI FLASH LOADER");

  lcd_set_font(FONT_SMALL);
  lcd_set_color(LCD_BLACK_COLOR, FG);

  flash_get_state(buf, sizeof(buf));
  lcd_puts(10, y, buf);
  y += 18;

  snprintf(buf, sizeof(buf), "mailbox  %08lX   buffer %lu KB",
      (unsigned long)MAILBOX_BASE, (unsigned long)(MAILBOX_DATA_SIZE / 1024));
  lcd_puts(10, y, buf);
  y += 12;

  lcd_set_color(LCD_BLACK_COLOR, DIM);
  lcd_puts(10, y, "./tools/spiflash.py push <addr> <file>");
  y += 22;

  lcd_set_color(LCD_BLACK_COLOR, FG);
  snprintf(buf, sizeof(buf), "commands %lu   written %lu   read %lu",
      (unsigned long)g_commands, (unsigned long)g_bytes_written,
      (unsigned long)g_bytes_read);
  lcd_puts(10, y, buf);
  y += 18;

  lcd_set_color(LCD_BLACK_COLOR, g_last_color ? g_last_color : FG);
  lcd_puts(10, y, g_last);

  lcd_set_color(LCD_BLACK_COLOR, DIM);
  lcd_puts(10, LCD_HEIGHT - 26,
      "The chip is only writable while this screen");
  lcd_puts(10, LCD_HEIGHT - 14,
      "is open.  SHIFT+MENU returns to the launcher.");
}

//-----------------------------------------------------------------------------
void spi_tool_init(void)
{
  // Acquisition owns this memory while the scope runs, so it has to be the
  // application that opens the mailbox, not the boot
  capture_stop();

  memset((void *)MAILBOX, 0, sizeof(mailbox_t) - MAILBOX_DATA_SIZE);

  g_last_sequence = 0;
  g_commands = 0;
  g_bytes_written = 0;
  g_bytes_read = 0;
  g_last_color = 0;
  note("waiting for the host", 0);

  // Last, so that a host polling for the magic never sees a mailbox whose
  // other fields are still whatever the last application left there
  MAILBOX->magic = MAILBOX_MAGIC;

  g_dirty = true;
  draw_screen();
}

//-----------------------------------------------------------------------------
void spi_tool_task(void)
{
  uint32_t cmd = MAILBOX->command;

  if (CMD_NONE != cmd)
    run_command(cmd);

  if (g_dirty)
  {
    g_dirty = false;
    draw_screen();
  }
}

//-----------------------------------------------------------------------------
void spi_tool_buttons_handler(int buttons)
{
  (void)buttons;
}

//-----------------------------------------------------------------------------
void spi_tool_cleanup(void)
{
  // The mailbox stops existing with the screen: no magic, no writes
  MAILBOX->magic = 0;
  MAILBOX->command = CMD_NONE;
}

//-----------------------------------------------------------------------------
void spi_tool_redraw(void)
{
  draw_screen();
}
