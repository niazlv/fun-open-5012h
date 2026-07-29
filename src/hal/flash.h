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

#ifndef _FLASH_H_
#define _FLASH_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
// Geometry every SPI NOR part of this class shares: programming is limited to
// one 256 byte page at a time, and the smallest thing that can be erased is a
// 4 KB sector. Erasing sets bits; programming only clears them, so a byte can
// go from 0xff to anything but never back without erasing what it sits in.
#define FLASH_PAGE_SIZE         256u
#define FLASH_SECTOR_SIZE       4096u
#define FLASH_BLOCK_SIZE        65536u

/*
 * The top of the part is not ours, and the map of it was measured rather than
 * assumed - the second time round.
 *
 * A full scan of all 2048 sectors (2026-07-29) found the stock firmware's data
 * living from 0x7BD000 up: twenty saved records of 4 KB each, numbered 20 down
 * to 1, holding what is plainly ADC samples - a curve, not noise - plus a few
 * scattered structures at 0x7DA000, 0x7DF000, 0x7E4000, 0x7EE000, 0x7EF000 and
 * 0x7F0000. What any of them mean is unknown; that they were deliberately
 * written is not in doubt.
 *
 * The settings store was first put at 0x7E0000, which is INSIDE that area. The
 * choice was made from a seven-point sample that happened to miss all of it,
 * and the migration erased one sector of somebody else's data before the full
 * scan found the rest. Hence the layout below, and hence the rule: an address
 * in this part is blank when a scan says so, not when a spot check does.
 *
 *   0x000000 .. 0x79FFFF   assets, growing up from zero
 *   0x7A0000 .. 0x7AFFFF   settings store
 *   0x7B0000 .. 0x7FFFFF   the stock firmware's, reserved, never written
 *
 * 0x7B0000 leaves thirteen blank sectors of margin below the first record.
 * Anything writing bulk data must stop at FLASH_RESERVED_BASE; the one caller
 * that legitimately writes above it is the settings store itself.
 */
#define FLASH_RESERVED_BASE     0x7A0000u
#define FLASH_STORE_BASE        0x7A0000u
#define FLASH_STORE_SIZE        0x10000u    // 64 KB, 16 sectors of rotation
#define FLASH_FACTORY_BASE      0x7B0000u   // do not write, ever

/*- Prototypes --------------------------------------------------------------*/
// Brings up SPI0 and asks the chip who it is. Never fails the boot: a part
// that is absent, wired differently or not shifting at all is recorded and
// reported by flash_get_state(), not turned into error().
void flash_init(void);

// True once a JEDEC ID came back that looks like a real device.
bool flash_present(void);

// Capacity in bytes, decoded from the JEDEC capacity byte. 0 if that byte
// made no sense.
uint32_t flash_size(void);

// The three raw ID bytes: manufacturer, memory type, capacity.
const uint8_t *flash_jedec_id(void);

// Reads `size` bytes at `addr`. False if there is no device, if the bus never
// answered, or if the range runs past the end of the part.
bool flash_read(uint32_t addr, uint8_t *data, uint32_t size);

// Programs `size` bytes at `addr`, splitting at page boundaries. The target
// must have been erased first: this can only clear bits, so programming over
// live data yields (old & new) and no error.
bool flash_write(uint32_t addr, const uint8_t *data, uint32_t size);

// Erase, to the granularity named. Both block until the part reports itself
// ready or the datasheet maximum passes, which is up to 400 ms for a sector
// and 2 s for a block - long enough to be visible as a stalled main loop.
bool flash_erase_sector(uint32_t addr);
bool flash_erase_block(uint32_t addr);

// Chip erase is tens of seconds, so this only issues it. Poll flash_busy()
// from a task rather than waiting here.
bool flash_erase_chip_start(void);

// True while a program or erase is still running.
bool flash_busy(void);

// Waits out a program or erase. Bounded in MICROseconds off timer_us(), since
// timer_ms() does not advance inside a blocking loop.
bool flash_wait_ready(uint32_t timeout_us);

// Two lines for the System Information page: what answered, and the first
// bytes it holds. Written to fit the panel, so they name the part rather than
// spell out a diagnosis.
void flash_get_state(char *buf, int size);
void flash_get_head(char *buf, int size);

#endif // _FLASH_H_

