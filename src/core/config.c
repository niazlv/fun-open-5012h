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
#include <string.h>
#include "gd32f4xx.h"
#include "timer.h"
#include "utils.h"
#include "common.h"
#include "config.h"
#include "debug_coredump.h"

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

/*
 * The calibration block, sealed on its own by config.calib_crc.
 *
 * The point is a difference in what the two halves of this struct cost to
 * lose. Preferences are re-made in a minute of button pressing. Calibration is
 * a measurement of this particular analog chain - it needs a reference source
 * and a calibration run - and the fallback is another unit's numbers, so a
 * scope that has quietly lost it goes on displaying volts that are simply
 * wrong. Sharing one checksum with the preferences meant any single bad byte
 * anywhere in the entry took the calibration down with it, and the recovery
 * (config_reset) then overwrote it with the defaults.
 *
 * So the block gets a second seal covering only itself, and calib_recover()
 * can believe it in an entry that has otherwise failed. calib_crc sits outside
 * the range it covers and inside CRC_LENGTH, so a whole entry that validates
 * also vouches for the seal.
 */
#define CALIB_OFFSET       offsetof(Config, calib_channel_delta)
#define CALIB_LENGTH       (offsetof(Config, crc) - CALIB_OFFSET)

_Static_assert(CALIB_LENGTH % sizeof(uint32_t) == 0,
    "calibration block must be a whole number of words for the CRC unit");
_Static_assert(offsetof(Config, calib_crc) + sizeof(uint32_t) <= CALIB_OFFSET,
    "calib_crc must lie outside the block it seals");
_Static_assert(CALIB_OFFSET + CALIB_LENGTH == CRC_LENGTH,
    "the calibration block must run to the end of the CRC region: a field "
    "appended after it would be sealed by calib_crc without meaning to be");

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

/*
 * How long a change may wait for its flash write, by what the change is.
 *
 * The store's unit of work is the whole struct: every save writes all of
 * sizeof(Config) into a fresh slot, so what the endurance budget buys is a
 * number of SAVES, and a field that rides along in one already happening is
 * free. That is what makes the split worth having - not writing less, but
 * writing less OFTEN, by letting the cheap-to-lose fields wait for a save
 * that was going to happen anyway.
 *
 * The fast tier is everything not named in g_lazy_fields: probe attenuation,
 * coupling, the scale settings, the key map, and above all the calibration.
 * These change when somebody deliberately changes them, which is rare, and
 * each is worth a write of its own. TIMER_INTERVAL, as before.
 *
 * The lazy tier is the fields in g_lazy_fields, and the case that matters is
 * a held button: panning rewrites horizontal_position every frame, so under
 * one rule for everything the store used to spend a save per second for as
 * long as a finger was down. Every one of those intermediate positions is
 * worthless - nobody cares where the trace sat three seconds into a ten
 * second pan, only where it stopped - so they coalesce, and at most one save
 * per LAZY_INTERVAL carries whatever the latest values happen to be.
 *
 * There is no queue to build: since a save writes the entire struct, waiting
 * IS the accumulation. Whatever else changed in the meantime, lazily or not,
 * goes out in the same write.
 *
 * The cost is symmetrical and worth stating plainly: power lost inside the
 * window loses the change. For a trace position or a snake score that is a
 * fair trade at a minute. For coupling or calibration it would not be, which
 * is exactly why those are not in the table.
 */
#define LAZY_INTERVAL      60000

/*
 * What the flash endurance budget actually spends is not writes but how often
 * the config reads as changed. The rotation turns ENTRIES_COUNT saves into a
 * single sector erase, and the 128 KB sector is good for on the order of 10k
 * of them; capped at one save per TIMER_INTERVAL, the worst case is an erase
 * every ENTRIES_COUNT seconds, which spends the whole budget in about two
 * weeks of powered-on time.
 *
 * That is not a hypothetical. It is what the CRC_LENGTH bug described above
 * did: the config was sealed with a checksum that the seal then overwrote, so
 * config_changed() was true forever and the store marched through the whole
 * rotation every couple of minutes for as long as the device was on. Nothing
 * reported it, because nothing had failed - every individual save was written
 * and read back, and the System Information line said the store was healthy
 * right up until the sector was worn out.
 *
 * So the rate is watched, not just the outcome. A run of saves that keeps
 * hitting the one-per-second cap for two full rotations is recorded as a
 * dump: it is not someone changing settings - holding a pan button does
 * rewrite horizontal_position every frame, but nobody holds it for four
 * minutes - it is a config that reports itself changed when nothing did.
 *
 * The gap allows three intervals rather than one because a save that wraps
 * the rotation waits out a sector erase, and that stalls the core for 1-2 s.
 * It is measured against the tier that fired the save: back-to-back at the
 * lazy ceiling is a minute apart and still a runaway, just a sixtieth as
 * expensive, so it is allowed to take a sixtieth as long to be noticed.
 */
#define RUNAWAY_GAP_MS(lazy) (((lazy) ? LAZY_INTERVAL : TIMER_INTERVAL) * 3)
#define RUNAWAY_SAVES      (ENTRIES_COUNT * 2)

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

// Save-rate watch, see RUNAWAY_SAVES. g_save_run counts saves that followed
// the previous one inside RUNAWAY_GAP_MS; anything slower restarts it.
static uint32_t g_last_save_ms = 0;
static uint32_t g_run_start_ms = 0;
static int g_save_run = 0;

// The dump ring holds MAX_COREDUMP_ENTRIES entries and belongs to the fault
// handler as much as to the store, so each condition is recorded once per
// boot. A store that fails every save must not push out the crash that
// explains why, and the repeat is not news anyway - the System Information
// line carries the running failure count.
static unsigned g_dumped_status = 0;   // bitmask over ConfigStoreStatus
static bool g_dumped_runaway = false;

// Counts down to the earliest moment a lazy-only change may be written, see
// LAZY_INTERVAL. Armed on every save, so a lazy change that follows an
// ordinary one rides it out rather than starting its own clock.
static int g_lazy_timer = 0;

// True once g_config_copy holds what is actually in the store, which is what
// the tier comparison is against. False on a fresh store, where there is
// nothing in flash to compare to and everything is therefore a change.
static bool g_have_copy = false;

static ConfigCalibSource g_calib_source = CONFIG_CALIB_DEFAULTS;

/*
 * Fields whose newest value is not worth a flash write of its own, see
 * LAZY_INTERVAL. Two rules decide membership:
 *
 *   - it changes continuously while a button is held, so all but the last
 *     value is noise (the positions, the trigger level, and the derived
 *     multipliers that follow them), or
 *   - losing up to LAZY_INTERVAL of it costs the user nothing they will
 *     mind (a game score, a measured bus clock, the boot counters).
 *
 * Everything NOT listed here is written promptly, and that is deliberately
 * the safe default: a field added to Config and forgotten about ends up in
 * the fast tier, which costs a few extra writes rather than silently losing
 * something that mattered. The calibration block is in the fast tier for the
 * same reason it has its own seal.
 *
 * offsetof/sizeof rather than literals, so this table cannot drift out of
 * step with the struct: rename a field and the build breaks, move one and it
 * follows. Order does not matter - the lookup is a scan.
 */
#define LAZY_FIELD(f)  { offsetof(Config, f), sizeof(((Config *)0)->f) }

static const struct { uint16_t offset; uint16_t size; } g_lazy_fields[] =
{
  // Bookkeeping: interesting in aggregate, worthless one increment at a time
  LAZY_FIELD(count),
  LAZY_FIELD(power_cycles),
  LAZY_FIELD(charge_cycles),

  // Where the trace sits. The held-button case this whole split exists for.
  LAZY_FIELD(horizontal_position),
  LAZY_FIELD(horizontal_position_px),
  LAZY_FIELD(horizontal_period),
  LAZY_FIELD(vertical_position),
  LAZY_FIELD(vertical_position_mv),
  LAZY_FIELD(vertical_mult),

  // The level, not the mode or the edge: those are single deliberate presses
  // and belong in the fast tier, the level is dragged
  LAZY_FIELD(trigger_level),
  LAZY_FIELD(trigger_level_mv),

  // Measured off the probe and re-measurable in seconds
  LAZY_FIELD(spi_clock_hz),
  LAZY_FIELD(swd_clock_hz),

  // Nice to keep, not worth a write per record-breaking run - and early in a
  // session almost every run breaks the record
  LAZY_FIELD(snake_high_score),
  LAZY_FIELD(flappy_high_score),
  LAZY_FIELD(g2048_high_score),
  LAZY_FIELD(tetris_high_score),
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static bool byte_is_lazy(unsigned offset)
{
  for (int i = 0; i < ARRAY_SIZE(g_lazy_fields); i++)
  {
    if (offset >= g_lazy_fields[i].offset &&
        offset < (unsigned)(g_lazy_fields[i].offset + g_lazy_fields[i].size))
      return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
// Has anything outside the lazy table changed since the snapshot that went to
// flash? Compared against g_config_copy rather than re-checksummed, because
// the interesting answer is per-byte: only bytes that actually differ pay for
// the table scan, and in the common case none do.
static bool important_changed(void)
{
  const uint8_t *now = (const uint8_t *)&config;
  const uint8_t *was = (const uint8_t *)&g_config_copy;

  // Nothing in the store to have diverged from, so the first save of a fresh
  // device must not sit in the lazy queue for a minute
  if (!g_have_copy)
    return true;

  for (unsigned i = 0; i < CRC_LENGTH; i++)
  {
    if (now[i] != was[i] && !byte_is_lazy(i))
      return true;
  }

  return false;
}

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
// One set of names for the store's status, shared by the System Information
// line and the dump message so the two cannot drift apart
static const char *status_name(ConfigStoreStatus status)
{
  static const char *const names[] =
  {
    "NO STORED CONFIG", "loaded", "saved",
    "ERASE ERR", "WRITE ERR", "VERIFY ERR",
  };

  return ((unsigned)status < ARRAY_SIZE(names)) ? names[status] : "?";
}

//-----------------------------------------------------------------------------
// Puts a failed save in the coredump ring, where it outlives the reset that
// the user is about to perform anyway. The System Information line already
// says the store is unhealthy, but only while this boot lasts and only if
// someone opens that page; the ring is what makes "my settings keep
// disappearing" a question with an answer.
static void dump_failure(ConfigStoreStatus status, int attempt)
{
  char msg[MAX_ERROR_MESSAGE_LEN];
  unsigned bit = 1u << (unsigned)status;

  if (!debug_coredump_ready() || (g_dumped_status & bit))
    return;

  g_dumped_status |= bit;

  // Which register is worth the room differs by failure. After a verify the
  // FMC flags have already been cleared, so STAT always reads zero and the
  // useful value is what the slot actually holds: ffffffff means nothing was
  // programmed, MAGIC means the entry landed and the checksum disagrees.
  if (CONFIG_STORE_VERIFY_ERROR == status)
    snprintf(msg, sizeof(msg), "Cfg %s read=%08lX entry %d/%d try %d",
        status_name(status), (unsigned long)g_last_entry_word,
        g_entry_offset / ENTRY_SIZE, ENTRIES_COUNT, attempt);
  else
    snprintf(msg, sizeof(msg), "Cfg %s stat=%08lX entry %d/%d fail %d",
        status_name(status), (unsigned long)g_last_fmc_stat,
        g_entry_offset / ENTRY_SIZE, ENTRIES_COUNT, g_saves_failed);

  COREDUMP_CAPTURE(ERROR_TYPE_MEMORY_ERROR, msg);
}

//-----------------------------------------------------------------------------
// Records why a save gave up. Deliberately not error(): a scope whose settings
// store has gone bad is still a working scope, and killing it behind a red
// screen would replace one silent failure with a louder useless one.
static void flash_fail(ConfigStoreStatus status)
{
  // Which attempt it died on is part of the story, and the teardown below
  // clears it
  int attempt = g_save_attempt;

  g_last_fmc_stat = FMC->STAT;
  g_status = status;
  g_saves_failed++;

  FMC->CTL = 0;
  flash_clear_flags();

  g_flash_erase_busy = false;
  g_flash_write_busy = false;
  g_flash_write_size = 0;
  g_save_attempt = 0;

  // After the FMC is safely idle: the capture walks the stack and formats a
  // string, neither of which should happen with a program operation still
  // armed
  dump_failure(status, attempt);
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
// The seal over the calibration block alone, see CALIB_OFFSET
static uint32_t calib_crc_calc(const Config *cfg)
{
  return crc32_calc((uint32_t *)((uint8_t *)cfg + CALIB_OFFSET), CALIB_LENGTH);
}

//-----------------------------------------------------------------------------
// Whether an entry's calibration can be believed WITHOUT believing the entry.
// The two rejected constants are the two ways of holding no calibration at
// all: a blank slot reads 0xffffffff throughout, and an entry saved before
// calib_crc was carved out of padding reads 0 there.
static bool calib_is_valid(const Config *entry)
{
  if (0 == entry->calib_crc || 0xffffffff == entry->calib_crc)
    return false;

  return calib_crc_calc(entry) == entry->calib_crc;
}

//-----------------------------------------------------------------------------
// Copies the calibration block out of an entry, leaving the rest of config
// alone. The block is contiguous by construction (see the static asserts), so
// this is one memcpy and not a field-by-field list that could go stale.
static void calib_adopt(const Config *entry)
{
  memcpy((uint8_t *)&config + CALIB_OFFSET,
      (const uint8_t *)entry + CALIB_OFFSET, CALIB_LENGTH);
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
// Last resort for the calibration when no entry validates as a whole: sweep
// the store for one whose calibration block still seals, and take just that.
//
// MUST run before the sector is erased - the entries about to be wiped are
// the only place this unit's calibration exists.
//
// The highest valid index wins rather than the highest count, because count
// lives in the half of the entry that has just been declared untrustworthy.
// The rotation writes forward, so the last valid block in slot order is the
// most recently written one unless the rotation wrapped since - and since
// calibration changes only when somebody calibrates, an older block is very
// nearly as good as the newest anyway.
static bool calib_recover(void)
{
  int found = -1;

  for (int i = 0; i < ENTRIES_COUNT; i++)
  {
    if (calib_is_valid(get_entry(i)))
      found = i;
  }

  if (found < 0)
    return false;

  calib_adopt(get_entry(found));

  return true;
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
// Watches how fast the rotation is being consumed, see RUNAWAY_SAVES. This is
// the failure that reports itself as success: every save works, and the store
// eats a sector erase every ENTRIES_COUNT of them regardless.
static void save_rate_check(bool lazy)
{
  uint32_t now = timer_ms();

  if (g_save_run > 0 && (now - g_last_save_ms) <= (uint32_t)RUNAWAY_GAP_MS(lazy))
  {
    g_save_run++;
  }
  else
  {
    g_save_run = 1;
    g_run_start_ms = now;
  }

  g_last_save_ms = now;

  if (g_save_run < RUNAWAY_SAVES || g_dumped_runaway || !debug_coredump_ready())
    return;

  g_dumped_runaway = true;

  // config.count is the lifetime save counter and survives the rotation, so
  // count/ENTRIES_COUNT is roughly how many erase cycles the sector has spent
  // - the number this dump exists to make visible before it reaches the end.
  char msg[MAX_ERROR_MESSAGE_LEN];

  snprintf(msg, sizeof(msg), "Cfg runaway: %d saves/%lus entry %d cnt %d",
      g_save_run, (unsigned long)((now - g_run_start_ms) / 1000),
      g_entry_offset / ENTRY_SIZE, config.count);

  COREDUMP_CAPTURE(ERROR_TYPE_GENERAL_ERROR, msg);
}

//-----------------------------------------------------------------------------
static void config_save(bool lazy)
{
  save_rate_check(lazy);

  config.count++;

  // The calibration seal first: it lies inside CRC_LENGTH, so the entry's own
  // checksum has to be taken over the sealed value, not the stale one
  config.calib_crc = calib_crc_calc(&config);
  config.crc = crc32_calc((uint32_t *)&config, CRC_LENGTH);

  g_config_copy = config;
  g_have_copy = true;
  g_save_attempt = 1;

  // Whatever was pending is now on its way, lazily or not, so the next lazy
  // change starts its wait from here rather than inheriting the old deadline
  g_lazy_timer = LAZY_INTERVAL;

  store_entry();
}

//-----------------------------------------------------------------------------
void config_init(void)
{
  int index = find_last_entry();

  flash_unlock();

  if (index == -1)
  {
    // Before the erase, and before config_reset() fills the struct: the only
    // copy of this unit's calibration is in the entries that are about to be
    // wiped, and it may still be readable even though none of them validates
    // as a whole.
    bool salvaged = calib_recover();

    flash_erase(SECTOR_0_INDEX);

    config_reset();

    // config_reset() leaves the calibration alone by design, so anything
    // calib_recover() found survives it. If it found nothing there is
    // genuinely none to keep, and the struct is still zeroed at this point.
    if (!salvaged)
      config_reset_calibration();

    g_calib_source = salvaged ? CONFIG_CALIB_RECOVERED : CONFIG_CALIB_DEFAULTS;

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

    // What is in flash, which is what the tier comparison measures against.
    // Not set in the branch above: on a fresh store there is nothing to have
    // diverged from, and the first save should not wait out LAZY_INTERVAL.
    g_config_copy = config;
    g_have_copy = true;

    g_calib_source = CONFIG_CALIB_LOADED;
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
  timer_add(&g_lazy_timer);

  // Start the lazy tier on its full interval rather than at zero. The boot
  // itself only bumps power_cycles, which is lazy, so an unarmed timer would
  // spend a write on that alone one second into every power-on - and a device
  // switched on and off again inside the minute would not have needed it.
  g_lazy_timer = LAZY_INTERVAL;
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
    // Not lazy whatever it holds: the caller is ending the session on
    // purpose, so there is no later save for a deferred field to ride
    config_save(false);
    config_drain();
  }
}

//-----------------------------------------------------------------------------
// One line of store health for the System Information page: whether the last
// save read back, where the rotation is, and how much has been written
void config_get_state(char *buf, int size)
{
  const char *name = status_name(g_status);

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
// Where the calibration in use came from. Worth its own line because the two
// bad answers do not announce themselves any other way: a scope running on
// another unit's defaults looks exactly like a working one, it just reports
// the wrong volts.
void config_get_calib_state(char *buf, int size)
{
  switch (g_calib_source)
  {
    case CONFIG_CALIB_LOADED:
      snprintf(buf, size, "Cal: from store  zero %d  delta %d",
          config.calib_dac_zero, config.calib_channel_delta);
      break;

    case CONFIG_CALIB_RECOVERED:
      snprintf(buf, size, "Cal: SALVAGED from a bad entry  zero %d  delta %d",
          config.calib_dac_zero, config.calib_channel_delta);
      break;

    default:
      snprintf(buf, size, "Cal: DEFAULTS - not this unit, measurements are a guess");
      break;
  }
}

//-----------------------------------------------------------------------------
ConfigCalibSource config_calib_source(void)
{
  return g_calib_source;
}

//-----------------------------------------------------------------------------
// Store inspection, for the flash viewer. Read-only throughout: this exists to
// answer "what is on the flash" without going through the copy in RAM, because
// the two disagreeing is exactly the failure that keeps happening here.
//-----------------------------------------------------------------------------
int config_store_entries(void)
{
  return ENTRIES_COUNT;
}

//-----------------------------------------------------------------------------
const Config *config_store_entry(int index)
{
  if (index < 0 || index >= ENTRIES_COUNT)
    return NULL;

  return get_entry(index);
}

//-----------------------------------------------------------------------------
int config_store_live_index(void)
{
  return g_entry_offset / ENTRY_SIZE;
}

//-----------------------------------------------------------------------------
bool config_store_entry_calib_ok(int index)
{
  const Config *entry = config_store_entry(index);

  return entry ? calib_is_valid(entry) : false;
}

//-----------------------------------------------------------------------------
// The same checks is_entry_valid() makes, reported one at a time. Which one
// failed is the useful part: BAD_MAGIC is a slot nothing ever wrote, BAD_CRC
// is one that was written and did not survive, and BAD_SIZE/BAD_VERSION mean
// the entry belongs to a different firmware.
ConfigEntryState config_store_entry_state(int index)
{
  const Config *entry = config_store_entry(index);

  if (!entry)
    return CONFIG_ENTRY_BLANK;

  if (entry_is_blank(index))
    return CONFIG_ENTRY_BLANK;

  if (entry->magic != MAGIC)
    return CONFIG_ENTRY_BAD_MAGIC;

  if (entry->version != VERSION)
    return CONFIG_ENTRY_BAD_VERSION;

  if (entry->size != (int)sizeof(Config))
    return CONFIG_ENTRY_BAD_SIZE;

  if (crc32_calc((uint32_t *)entry, CRC_LENGTH) != entry->crc)
    return CONFIG_ENTRY_BAD_CRC;

  return CONFIG_ENTRY_VALID;
}

//-----------------------------------------------------------------------------
const char *config_entry_state_name(ConfigEntryState state)
{
  static const char *const names[] =
  {
    "blank", "ok", "BAD MAGIC", "BAD VERSION", "BAD SIZE", "BAD CRC",
  };

  return ((unsigned)state < ARRAY_SIZE(names)) ? names[state] : "?";
}

/*
 * Field descriptions, for rendering any Config - the live one or an entry
 * still on the flash - as readable lines.
 *
 * offsetof again rather than a switch over field names, for the same reason
 * the lazy table uses it: this list sits next to a struct whose layout is
 * pinned and edited often, and a description that has quietly drifted off its
 * field is worse than no description at all. Renaming a field breaks the
 * build here; moving one is followed automatically.
 */
typedef enum
{
  FT_INT,      // signed decimal
  FT_BOOL,
  FT_HEX,      // uint32_t, as hex - magic and the checksums
  FT_I64,
  FT_IARR,     // int[count], one line per element
  FT_MAP,      // int[count] of which only the non-zero entries are interesting
} FieldType;

#define FIELD(f, t)     { #f, offsetof(Config, f), t, 1 }
#define FIELD_N(f, t, n) { #f, offsetof(Config, f), t, n }

static const struct
{
  const char *name;
  uint16_t offset;
  uint8_t type;
  uint8_t count;
} g_fields[] =
{
  FIELD(magic, FT_HEX),
  FIELD(size, FT_INT),
  FIELD(version, FT_INT),
  FIELD(count, FT_INT),
  FIELD(power_cycles, FT_INT),
  FIELD(charge_cycles, FT_INT),

  FIELD(lcd_bl_level, FT_INT),
  FIELD(screen_brightness, FT_INT),

  FIELD(ac_coupling, FT_BOOL),
  FIELD(x10, FT_BOOL),

  FIELD(trigger_mode, FT_INT),
  FIELD(trigger_edge, FT_INT),
  FIELD(trigger_level, FT_INT),
  FIELD(trigger_level_mv, FT_INT),

  FIELD(horizontal_scale, FT_INT),
  FIELD(horizontal_position, FT_I64),
  FIELD(horizontal_position_px, FT_INT),
  FIELD(horizontal_period, FT_INT),

  FIELD(vertical_scale, FT_INT),
  FIELD(vertical_position, FT_INT),
  FIELD(vertical_position_mv, FT_INT),
  FIELD(vertical_mult, FT_INT),

  FIELD(sample_rate_limit, FT_INT),

  FIELD(measure_display, FT_BOOL),
  FIELD(measure_panel_mode, FT_INT),
  FIELD(show_vpp, FT_BOOL),
  FIELD(show_freq, FT_BOOL),
  FIELD(show_duty, FT_BOOL),
  FIELD(show_vrms, FT_BOOL),
  FIELD(show_vavg, FT_BOOL),
  FIELD(show_type, FT_BOOL),
  FIELD(show_thd, FT_BOOL),
  FIELD(show_jitter, FT_BOOL),
  FIELD(measure_line_set, FT_BOOL),
  FIELD_N(measure_line, FT_IARR, MEASURE_LINE_SLOTS),

  FIELD(shift_mode_enabled, FT_BOOL),
  FIELD(shift_hold_lock, FT_BOOL),
  FIELD(key_remapping_enabled, FT_BOOL),
  FIELD_N(key_mapping, FT_MAP, 32),

  FIELD(decoder_proto, FT_INT),
  FIELD(decoder_stop, FT_BOOL),
  FIELD(decoder_stop_start, FT_BOOL),
  FIELD(decoder_baud, FT_INT),
  FIELD(decoder_fit_mode, FT_INT),
  FIELD(decoder_bits_mode, FT_INT),
  FIELD(spi_clock_hz, FT_INT),
  FIELD(spi_order, FT_INT),
  FIELD(man_rate, FT_INT),
  FIELD(man_polarity, FT_INT),
  FIELD(swd_clock_hz, FT_INT),

  FIELD(persist_mode, FT_INT),
  FIELD(average_mode, FT_INT),
  FIELD(roll_from, FT_INT),

  FIELD(snake_high_score, FT_INT),
  FIELD(flappy_high_score, FT_INT),
  FIELD(g2048_high_score, FT_INT),
  FIELD(tetris_high_score, FT_INT),

  FIELD(calib_ref_mv, FT_INT),
  FIELD(calib_crc, FT_HEX),
  FIELD(calib_channel_delta, FT_INT),
  FIELD(calib_dac_zero, FT_INT),
  FIELD_N(calib_dac_mult, FT_IARR, VS_COUNT),
  FIELD_N(calib_vs_mult, FT_IARR, VS_COUNT),

  FIELD(crc, FT_HEX),
};

//-----------------------------------------------------------------------------
// How many lines a descriptor renders: arrays get one per element, everything
// else gets one
static int field_lines(int i)
{
  return (FT_IARR == g_fields[i].type) ? g_fields[i].count : 1;
}

//-----------------------------------------------------------------------------
bool config_describe(const Config *cfg, int index, char *buf, int size)
{
  const uint8_t *base = (const uint8_t *)cfg;

  if (!cfg || index < 0)
    return false;

  for (int i = 0; i < ARRAY_SIZE(g_fields); i++)
  {
    int lines = field_lines(i);

    if (index >= lines)
    {
      index -= lines;
      continue;
    }

    const void *p = base + g_fields[i].offset;
    const char *name = g_fields[i].name;

    switch (g_fields[i].type)
    {
      case FT_BOOL:
        snprintf(buf, size, "%-24s%s", name, *(const bool *)p ? "true" : "false");
        break;

      case FT_HEX:
        snprintf(buf, size, "%-24s%08lX", name,
            (unsigned long)*(const uint32_t *)p);
        break;

      case FT_I64:
      {
        // No %lld in this newlib build's nano printf, and the value is a time
        // in ns that fits a long in every reachable case
        int64_t v = *(const int64_t *)p;
        snprintf(buf, size, "%-24s%ld", name, (long)v);
        break;
      }

      case FT_IARR:
      {
        char label[28];
        snprintf(label, sizeof(label), "%s[%d]", name, index);
        snprintf(buf, size, "%-24s%d", label, ((const int *)p)[index]);
        break;
      }

      case FT_MAP:
      {
        // 32 entries of which all but a few are zero: the count is the
        // information, and the whole array would bury the rest of the dump
        const uint32_t *map = (const uint32_t *)p;
        int used = 0;

        for (int k = 0; k < g_fields[i].count; k++)
        {
          if (0 != map[k])
            used++;
        }

        snprintf(buf, size, "%-24s%d of %d set", name, used,
            g_fields[i].count);
        break;
      }

      default:
        snprintf(buf, size, "%-24s%d", name, *(const int *)p);
        break;
    }

    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
// The calibration as text, a line at a time, for the page whose whole purpose
// is that these numbers can be copied onto paper. Flash is a cache for them:
// the sector they live in is the one the store wears out, it is erased by a
// firmware that changes sizeof(Config), and nothing in here survives the
// silicon actually failing. Eighteen numbers written down do.
bool config_get_calib_line(int index, char *buf, int size)
{
  static const char *const scale_names[VS_COUNT] =
  {
    "50mV", "100mV", "200mV", "500mV", "1V", "2V", "5V", "10V",
  };

  switch (index)
  {
    case 0:
      snprintf(buf, size, "zero %d   delta %d   ref %d mV",
          config.calib_dac_zero, config.calib_channel_delta,
          config.calib_ref_mv);
      return true;

    case 1:
      snprintf(buf, size, "%-16s%-10s%s", "range", "offset(O)", "gain(S)");
      return true;

    default:
      break;
  }

  index -= 2;

  if (index < 0 || index >= VS_COUNT)
    return false;

  // A range added to VS_COUNT without a name here leaves a NULL in the array,
  // and the number still matters more than the label it is missing
  snprintf(buf, size, "%-16s%-10d%d",
      scale_names[index] ? scale_names[index] : "?",
      config.calib_dac_mult[index], config.calib_vs_mult[index]);

  return true;
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

  // Keep the calibration block, which is NOT a preference. This is reached
  // from the boot-time button combo in main.c, and that combo means "I have
  // made a mess of the settings" - not "throw away the measurement of this
  // unit's analog chain and substitute another unit's numbers", which is
  // what it silently did, permanently, one second later when the store
  // saved. config_reset_calibration() is the way to ask for that on purpose.

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

  config.persist_mode           = 0; // 0 = off, 1 = infinite, 2 = decay
  config.average_mode           = 0;
  config.decoder_baud           = 0; // auto-detect
  config.decoder_fit_mode       = 0; // fit the timebase to the rate
  config.decoder_bits_mode      = 0; // bit grid over the trace
  config.spi_clock_hz           = 0; // work the SPI bit time out of the data
  config.spi_order              = 0; // ...and score both bit orders
  config.man_rate               = 0; // Manchester rate off the record
  config.man_polarity           = 0; // ...rising mid-bit edge is a one
  config.swd_clock_hz           = 0; // work the SWD bit time out of the record
  config.roll_from              = 0; // roll only above 500 ms/div

  // General settings defaults
  config.screen_brightness      = 80;  // 80% default brightness
  // Both ways of getting shift without holding it are on out of the box. They
  // cost nothing until SHIFT is tapped twice or held, and a user who finds one
  // of them in the way can switch that one off on its own.
  config.shift_mode_enabled     = true;
  config.shift_hold_lock        = true;
  config.shift_mode_active      = false;
  config.key_remapping_enabled  = false;
  
  // Initialize key mapping with default values
  for (int i = 0; i < 32; i++)
    config.key_mapping[i] = 0;

  for (int i = 0; i < ARRAY_SIZE(config.padding); i++)
    config.padding[i] = 0;
}

//-----------------------------------------------------------------------------
// The calibration defaults, which belong to whichever unit they were measured
// on and not to this one. Split out of config_reset() so that resetting the
// settings cannot reach them by accident: getting here should always be
// somebody's decision, or config_init() finding nothing left to recover.
//
// A scope running on these is not broken, it is inaccurate - which is worse,
// because it looks fine. config_calib_source() is what says so on screen.
void config_reset_calibration(void)
{
  config.calib_ref_mv           = 0;

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

  if (g_config_timer != 0)
    return;

  g_config_timer = TIMER_INTERVAL;

  if (!config_changed())
    return;

  // Something worth a write of its own moved: goes now, and carries whatever
  // lazy changes were waiting out with it for free - the entry is written
  // whole either way
  if (important_changed())
  {
    config_save(false);
    return;
  }

  // Only lazy fields differ. They keep accumulating in RAM until either an
  // important change comes along to carry them, or the deadline runs out.
  if (g_lazy_timer == 0)
    config_save(true);
}
