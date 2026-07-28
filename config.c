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
#include <stdio.h>
#include <stddef.h>
#include "gd32f4xx.h"
#include "timer.h"
#include "utils.h"
#include "common.h"
#include "config.h"

/*- Definitions -------------------------------------------------------------*/
#define MAGIC              0x78656c41
#define VERSION            1

/*
 * Storage lives in the last 128 KB sector, and the firmware owns everything
 * below it. It used to take the last 256 KB - sectors 6 and 7 - which was free
 * when the firmware was 154 KB, but the DOOM asset pack pushed the image to
 * 372 KB and its tail landed inside sector 6. Nothing failed at build time: the
 * firmware ran, and then config_init erased the sector out from under the pack.
 * The linker script now caps the flash region at 384 KB so an overlap is a
 * build error rather than something the device discovers at runtime.
 *
 * The cost of a single sector is that rotating past the end has to erase the
 * only copy, so losing power inside that window (once every ENTRIES_COUNT
 * saves) falls back to defaults on the next boot.
 */
#define FLASH_START        0x08000000
#define ENTRY_SIZE         (1024)
#define STORAGE_SIZE       (128*1024)
#define STORAGE_OFFSET     (384*1024)
#define STORAGE_START      (FLASH_START + STORAGE_OFFSET)
#define ENTRIES_COUNT      (STORAGE_SIZE / ENTRY_SIZE)

#define SECTOR_0_INDEX     (7)
#define SECTOR_0_OFFSET    (0)

/*
 * How much of an entry the CRC covers. This is offsetof(crc), and it is NOT
 * sizeof(Config) - sizeof(uint32_t), which is what every call site used to
 * pass. int64_t horizontal_position gives Config 8-byte alignment, so with
 * crc landing at offset 408 the compiler adds four bytes of TRAILING PADDING
 * to round the struct up to 416. sizeof - 4 is therefore 412, four bytes past
 * where the crc field starts, and the checksum was being computed over its
 * own storage.
 *
 * Nothing about that fails loudly. It just makes the store impossible:
 * config_save() seals with a value derived from the previous seal and then
 * overwrites it, so is_entry_valid() recomputing the same range reads the new
 * crc and never matches - every entry ever written is rejected, find_last_entry
 * returns -1, and each boot starts from defaults. config_changed() compares the
 * same way, so it is also permanently true: the config was being rewritten
 * every second, marching through the 128 entry rotation forever.
 *
 * The assert is the point of the constant. Config's layout is edited often
 * (fields are carved out of padding[]), and any edit that shifts crc across an
 * 8-byte boundary brings the trailing padding back.
 */
#define CRC_LENGTH         offsetof(Config, crc)

_Static_assert(CRC_LENGTH % sizeof(uint32_t) == 0,
    "CRC region must be a whole number of words for the hardware CRC unit");
_Static_assert(CRC_LENGTH + sizeof(uint32_t) <= sizeof(Config),
    "crc must be the last field of Config");

// The layout is pinned ABSOLUTELY. Alignment makes it treacherous: adding a
// ninth bool to the show_* run once moved every field below it by 4 bytes
// while sizeof(Config) stayed put - the store then failed every entry's CRC
// and erased the sector, taking the calibration with it. If one of these
// fires, the new field is padding the compiler re-arranged around: carve it
// out of padding[] / decoder_reserved[] instead, and verify with a host
// build that these numbers did not move.
_Static_assert(offsetof(Config, measure_line) == 248, "layout shifted: measure_line");
_Static_assert(offsetof(Config, calib_channel_delta) == 336, "layout shifted: calibration block");
_Static_assert(offsetof(Config, crc) == 408, "layout shifted: crc / CRC_LENGTH");

#define TIMER_INTERVAL     1000

/*
 * A save used to trickle one word per config_task() call, and config_task()
 * runs once per main loop pass. A pass is one scope frame - the LCD is
 * bit-banged, so ~15 ms - which put a 416 byte config 104 passes, about 1.5
 * seconds, away from being safely on the flash. Switching the device off
 * inside that window lost the change. Programming a word takes tens of
 * microseconds, so a batch of 16 costs well under a millisecond of the frame
 * and lands the whole config in about 7 passes.
 */
#define WRITE_WORDS_PER_TASK 16

// A save that does not read back is a save that only looks like one, so a
// failed entry is rewritten into the next slot rather than being trusted
#define SAVE_MAX_ATTEMPTS  3

// Long enough for a 128 KB sector erase (1-2 s) plus the entry behind it
#define FLUSH_TIMEOUT_US   3000000u

// A word program is tens of microseconds; a millisecond means it is not coming
#define WORD_TIMEOUT_US    1000u

#define FMC_KEY_KEY1       0x45670123
#define FMC_KEY_KEY2       0xcdef89ab

#define FMC_STAT_ALL_ERRORS (FMC_STAT_OPERR_Msk | FMC_STAT_WPERR_Msk | \
    FMC_STAT_PGMERR_Msk | FMC_STAT_PGSERR_Msk | FMC_STAT_RDDERR_Msk)

// END and the error bits are sticky and are cleared by writing 1 to them.
// Nothing ever cleared them, so the first error the FMC ever raised stayed
// latched and every later operation reported it as its own.
#define FMC_STAT_ALL_FLAGS  (FMC_STAT_END_Msk | FMC_STAT_ALL_ERRORS)

/*- Variables ---------------------------------------------------------------*/
Config config;
static int g_config_timer = 0;
static int g_entry_offset;
static Config g_config_copy;
static bool g_flash_erase_busy = false;
static bool g_flash_write_busy = false;
static uint32_t *g_flash_write_addr = 0;
static uint32_t *g_flash_write_data = 0;
static uint32_t g_flash_write_size = 0;

// Store health, reported by config_get_state() on the System Information page.
// The whole point is that a store which is not persisting says so on screen
// instead of quietly handing back defaults on the next boot.
static int g_save_attempt = 0;
static ConfigStoreStatus g_status = CONFIG_STORE_FRESH;
static int g_saves_ok = 0;
static int g_saves_failed = 0;
static uint32_t g_last_fmc_stat = 0;
static uint32_t g_last_entry_word = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline Config *get_entry(int index)
{
  return (Config *)(STORAGE_START + index * ENTRY_SIZE);
}

//-----------------------------------------------------------------------------
static void flash_clear_flags(void)
{
  FMC->STAT = FMC_STAT_ALL_FLAGS;
}

//-----------------------------------------------------------------------------
// Records why a save gave up. Deliberately not error(): a scope whose settings
// store has gone bad is still a working scope, and killing it behind a red
// screen would replace one silent failure with a louder useless one.
static void flash_fail(ConfigStoreStatus status)
{
  g_last_fmc_stat = FMC->STAT;
  g_status = status;
  g_saves_failed++;

  FMC->CTL = 0;
  flash_clear_flags();

  g_flash_erase_busy = false;
  g_flash_write_busy = false;
  g_flash_write_size = 0;
  g_save_attempt = 0;
}

//-----------------------------------------------------------------------------
static void flash_unlock(void)
{
  FMC->KEY = FMC_KEY_KEY1;
  FMC->KEY = FMC_KEY_KEY2;
  FMC->CTL = 0;

  flash_clear_flags();

  if (FMC->CTL & FMC_CTL_LK_Msk)
    error("Flash unlock error");
}

//-----------------------------------------------------------------------------
static void flash_erase(int index)
{
  uint32_t cmd = FMC_CTL_SER_Msk | (index << FMC_CTL_SN_Pos);

  flash_clear_flags();

  FMC->CTL = cmd;
  FMC->CTL = cmd | FMC_CTL_START_Msk;

  g_flash_erase_busy = true;
}

//-----------------------------------------------------------------------------
static bool flash_erase_task(void)
{
  if (!g_flash_erase_busy)
    return false;

  if (FMC->STAT & FMC_STAT_BUSY_Msk)
    return true;

  if (FMC->STAT & FMC_STAT_ALL_ERRORS)
  {
    flash_fail(CONFIG_STORE_ERASE_ERROR);
    return false;
  }

  FMC->CTL = 0;
  flash_clear_flags();

  g_flash_erase_busy = false;

  return false;
}

//-----------------------------------------------------------------------------
static void flash_write(uint32_t *addr, uint32_t *data, int size)
{
  // Not while an erase is still running: those flags belong to the erase and
  // are checked when it finishes
  if (!g_flash_erase_busy)
    flash_clear_flags();

  g_flash_write_addr = addr;
  g_flash_write_data = data;
  g_flash_write_size = size;
  g_flash_write_busy = true;
}

//-----------------------------------------------------------------------------
static bool is_entry_valid(Config *entry);
static void store_entry(void);

//-----------------------------------------------------------------------------
// Called once the last word of an entry is on the flash. Reading the entry
// back is the only thing that distinguishes a save from a sequence of writes
// that went nowhere, which is what the store used to do.
static void write_verify(void)
{
  Config *entry = get_entry(g_entry_offset / ENTRY_SIZE);

  if (is_entry_valid(entry))
  {
    g_status = CONFIG_STORE_OK;
    g_saves_ok++;
    g_save_attempt = 0;
    return;
  }

  // What the slot holds, which is the question a verify failure actually
  // raises. FMC->STAT is useless here - the flags were cleared when the write
  // finished, so it always reads back zero. ffffffff means nothing was
  // programmed, MAGIC means the entry landed and the checksum is what
  // disagrees, anything else means the data is wrong.
  g_last_entry_word = *(const uint32_t *)entry;

  // The entry did not read back. One bad slot should not cost the user their
  // settings, so rewrite the same snapshot into the next one.
  if (g_save_attempt < SAVE_MAX_ATTEMPTS)
  {
    g_save_attempt++;
    store_entry();
    return;
  }

  flash_fail(CONFIG_STORE_VERIFY_ERROR);
}

//-----------------------------------------------------------------------------
static bool flash_write_task(void)
{
  if (!g_flash_write_busy)
    return false;

  if (FMC->STAT & FMC_STAT_BUSY_Msk)
    return true;

  if (FMC->STAT & FMC_STAT_ALL_ERRORS)
  {
    flash_fail(CONFIG_STORE_WRITE_ERROR);
    return false;
  }

  if (g_flash_write_size > 0)
  {
    FMC->CTL = (2/*WORD*/ << FMC_CTL_PSZ_Pos) | FMC_CTL_PG_Msk;

    for (int i = 0; i < WRITE_WORDS_PER_TASK && g_flash_write_size > 0; i++)
    {
      uint32_t start = timer_us();

      *g_flash_write_addr = *g_flash_write_data;

      // Programming a word takes tens of microseconds and stalls the flash;
      // the next store has to wait for it either way, so wait here where the
      // result can still be checked. Bounded, because this runs in the main
      // loop and an FMC that never drops BUSY would take the scope with it.
      while (FMC->STAT & FMC_STAT_BUSY_Msk)
      {
        if (timer_us() - start > WORD_TIMEOUT_US)
        {
          flash_fail(CONFIG_STORE_WRITE_ERROR);
          return false;
        }
      }

      if (FMC->STAT & FMC_STAT_ALL_ERRORS)
      {
        flash_fail(CONFIG_STORE_WRITE_ERROR);
        return false;
      }

      g_flash_write_addr++;
      g_flash_write_data++;
      g_flash_write_size -= sizeof(uint32_t);
    }

    return true;
  }

  FMC->CTL = 0;
  flash_clear_flags();

  g_flash_write_busy = false;

  write_verify();

  return false;
}

//-----------------------------------------------------------------------------
static bool is_entry_valid(Config *entry)
{
  if (entry->magic != MAGIC)
    return false;

  if (entry->version != VERSION)
    return false;

  if (entry->size != sizeof(Config))
    return false;

  if (crc32_calc((uint32_t *)entry, CRC_LENGTH) != entry->crc)
    return false;

  return true;
}

//-----------------------------------------------------------------------------
static int find_last_entry(void)
{
  int max_count = -1;
  int index = -1;

  for (int i = 0; i < ENTRIES_COUNT; i++)
  {
    Config *entry = get_entry(i);

    if (!is_entry_valid(entry))
      continue;

    if (entry->count > max_count)
    {
      index = i;
      max_count = entry->count;
    }
  }

  return index;
}

//-----------------------------------------------------------------------------
static bool config_changed(void)
{
  return crc32_calc((uint32_t *)&config, CRC_LENGTH) != config.crc;
}

//-----------------------------------------------------------------------------
// Only the bytes an entry actually occupies are ever written, so those are the
// only ones that have to be blank
static bool entry_is_blank(int index)
{
  const uint32_t *p = (const uint32_t *)get_entry(index);

  for (unsigned i = 0; i < sizeof(Config) / sizeof(uint32_t); i++)
  {
    if (0xffffffff != p[i])
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// Puts the sealed snapshot in g_config_copy into the next entry. Split out of
// config_save() so a failed read-back can retry into the following slot
// without re-sealing (and so re-counting) the config.
static void store_entry(void)
{
  g_entry_offset = (g_entry_offset + ENTRY_SIZE) % STORAGE_SIZE;

  /*
   * The rotation only erases when it wraps, which silently assumes every slot
   * ahead of the write pointer is still blank. Break that assumption once and
   * the store wedges permanently: NOR programming can only clear bits, so a
   * write into a used slot lands as (old & new), the entry never validates,
   * find_last_entry keeps returning the same older slot, and the next boot
   * pins the pointer right back in front of the same garbage. It never
   * reaches the wrap, so it never erases, so it never recovers - which is how
   * a device ends up sitting at entry 108/128 failing every save.
   *
   * Anything that leaves a used slot ahead of the pointer gets it there: a
   * write corrupted by the missing flash wait states, a power cut partway
   * through an entry, or a firmware whose sizeof(Config) differs. Checking
   * the slot is blank turns all of them into one recovery - erase and restart
   * the rotation - instead of a store that is dead until reflashed.
   */
  if (g_entry_offset == SECTOR_0_OFFSET || !entry_is_blank(g_entry_offset / ENTRY_SIZE))
  {
    flash_erase(SECTOR_0_INDEX);
    g_entry_offset = 0;
  }

  flash_write((uint32_t *)(STORAGE_START + g_entry_offset),
      (uint32_t *)&g_config_copy, sizeof(Config));
}

//-----------------------------------------------------------------------------
static void config_save(void)
{
  config.count++;
  config.crc = crc32_calc((uint32_t *)&config, CRC_LENGTH);

  g_config_copy = config;
  g_save_attempt = 1;

  store_entry();
}

//-----------------------------------------------------------------------------
void config_init(void)
{
  int index = find_last_entry();

  flash_unlock();

  if (index == -1)
  {
    flash_erase(SECTOR_0_INDEX);

    config_reset();

    config.count = 0;
    config.power_cycles = 0;
    config.charge_cycles = 0;

    g_entry_offset = 0;

    // Nothing usable was in the store. Expected exactly once, on a device
    // that has never saved; on any later boot it means the previous session's
    // settings did not survive, which is worth being able to see.
    g_status = CONFIG_STORE_FRESH;
  }
  else
  {
    Config *entry = get_entry(index);
    config = *entry;
    g_entry_offset = index * ENTRY_SIZE;

    g_status = CONFIG_STORE_LOADED;
  }

  config.power_cycles++;

  // Fields carved out of padding read as zero in a config saved before they
  // existed, and zero has to mean something safe. For the status-line slots
  // it would mean "show nothing", which is a legal choice but the wrong thing
  // to hand someone who never made it: fill them in once, and remember that
  // we did so that turning them back off afterwards sticks.
  if (!config.measure_line_set)
  {
    config.measure_line_set = true;
    config.measure_line[0] = MEASURE_VPP;
    config.measure_line[1] = MEASURE_FREQ;
  }

  // A stored slot outside the enum would index the menu's label table
  for (int i = 0; i < MEASURE_LINE_SLOTS; i++)
  {
    if ((unsigned)config.measure_line[i] >= MEASURE_COUNT)
      config.measure_line[i] = MEASURE_NONE;
  }

  timer_add(&g_config_timer);
}

//-----------------------------------------------------------------------------
// Drives the store's state machine to completion. Bounded by the clock rather
// than by a pass count, because a sector erase stalls the core for 1-2 s and
// a store that has stopped responding must not hang the caller.
static void config_drain(void)
{
  uint32_t start = timer_us();

  while (g_flash_erase_busy || g_flash_write_busy)
  {
    if (timer_us() - start > FLUSH_TIMEOUT_US)
      break;

    flash_erase_task();
    flash_write_task();
  }
}

//-----------------------------------------------------------------------------
// Saves immediately instead of waiting for the next timer tick. Call it before
// anything that ends the session on purpose - a reset from the menu used to
// drop both the pending write and any change the 1 s timer had not seen yet.
void config_flush(void)
{
  config_drain();

  if (config_changed())
  {
    config_save();
    config_drain();
  }
}

//-----------------------------------------------------------------------------
// One line of store health for the System Information page: whether the last
// save read back, where the rotation is, and how much has been written
void config_get_state(char *buf, int size)
{
  static const char *const names[] =
  {
    "NO STORED CONFIG", "loaded", "saved",
    "ERASE ERR", "WRITE ERR", "VERIFY ERR",
  };

  const char *name = ((unsigned)g_status < ARRAY_SIZE(names))
      ? names[g_status] : "?";

  if (CONFIG_STORE_VERIFY_ERROR == g_status)
    snprintf(buf, size, "Cfg: %s read=%08lX entry %d/%d fail %d",
        name, (unsigned long)g_last_entry_word,
        g_entry_offset / ENTRY_SIZE, ENTRIES_COUNT, g_saves_failed);
  else if (g_status >= CONFIG_STORE_ERASE_ERROR)
    snprintf(buf, size, "Cfg: %s stat=%08lX entry %d/%d fail %d",
        name, (unsigned long)g_last_fmc_stat,
        g_entry_offset / ENTRY_SIZE, ENTRIES_COUNT, g_saves_failed);
  else
    snprintf(buf, size, "Cfg: %s  entry %d/%d  saves %d  boot %d",
        name, g_entry_offset / ENTRY_SIZE, ENTRIES_COUNT,
        g_saves_ok, config.power_cycles);
}

//-----------------------------------------------------------------------------
void config_reset(void)
{
  config.magic   = MAGIC;
  config.size    = sizeof(Config);
  config.version = VERSION;

  // Keep config.count
  // Keep config.power_cycles
  // Keep config.charge_cycles

  config.lcd_bl_level           = 100;

  config.ac_coupling            = false;
  config.x10                    = false;

  config.trigger_mode           = TRIGGER_MODE_AUTO;
  config.trigger_edge           = TRIGGER_EDGE_RISE;
  config.trigger_level          = 0;
  config.trigger_level_mv       = 0;

  config.horizontal_scale       = HS_100_us;
  config.horizontal_position    = 0;
  config.horizontal_position_px = 0;
  config.horizontal_period      = 1; // Dummy value

  config.vertical_scale         = VS_200_mV;
  config.vertical_position      = 0;
  config.vertical_position_mv   = 0;
  config.vertical_mult          = 1; // Dummy value

  config.sample_rate_limit      = 0;

  config.measure_display        = true;

  config.measure_panel_mode     = 0; // panel on
  config.show_vpp               = true;
  config.show_freq              = true;
  config.show_duty              = true;
  config.show_vrms              = true;
  config.show_vavg              = false;
  config.show_type              = false;
  config.show_thd               = false;

  // The two the status line is worth spending its large font on
  config.measure_line_set       = true;
  config.measure_line[0]        = MEASURE_VPP;
  config.measure_line[1]        = MEASURE_FREQ;

  config.decoder_proto          = 0; // auto
  config.decoder_stop           = false;
  config.show_jitter            = false;
  config.decoder_stop_start     = false;
  config.decoder_reserved[0]    = false;

  config.display_persist        = false;
  config.display_reserved[0]    = false;
  config.display_reserved[1]    = false;
  config.display_reserved[2]    = false;
  config.average_mode           = 0;
  config.decoder_baud           = 0; // auto-detect
  config.decoder_fit_mode       = 0; // fit the timebase to the rate
  config.decoder_bits_mode      = 0; // bit grid over the trace

  // General settings defaults
  config.screen_brightness      = 80;  // 80% default brightness
  config.shift_mode_enabled     = false;
  config.shift_mode_active      = false;
  config.key_remapping_enabled  = false;
  
  // Initialize key mapping with default values
  for (int i = 0; i < 32; i++)
    config.key_mapping[i] = 0;

  for (int i = 0; i < ARRAY_SIZE(config.padding); i++)
    config.padding[i] = 0;

  config.calib_channel_delta    = -5;
  config.calib_dac_zero         = 2010;

  config.calib_dac_mult[VS_50_mV]  = 2308;
  config.calib_dac_mult[VS_100_mV] = 4548;
  config.calib_dac_mult[VS_200_mV] = 5277;
  config.calib_dac_mult[VS_500_mV] = 5384;
  config.calib_dac_mult[VS_1_V]    = 5160;
  config.calib_dac_mult[VS_2_V]    = 5082;
  config.calib_dac_mult[VS_5_V]    = 5542;
  config.calib_dac_mult[VS_10_V]   = 5630;

  config.calib_vs_mult[VS_50_mV]   = 4941;
  config.calib_vs_mult[VS_100_mV]  = 4808;
  config.calib_vs_mult[VS_200_mV]  = 8343;
  config.calib_vs_mult[VS_500_mV]  = 20449;
  config.calib_vs_mult[VS_1_V]     = 42447;
  config.calib_vs_mult[VS_2_V]     = 86232;
  config.calib_vs_mult[VS_5_V]     = 196726;
  config.calib_vs_mult[VS_10_V]    = 386029;
}

//-----------------------------------------------------------------------------
void config_task(void)
{
  if (flash_erase_task())
    return;

  if (flash_write_task())
    return;

  if (g_config_timer == 0)
  {
    if (config_changed())
      config_save();

    g_config_timer = TIMER_INTERVAL;
  }
}
