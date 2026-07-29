/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Files on the SPI part. See spifs.h for why there is no directory.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "gd32f4xx.h"
#include "flash.h"
#include "spifs.h"

/*- Definitions -------------------------------------------------------------*/
/*
 * The on-chip header, 64 bytes at a sector boundary, little endian like
 * everything else here.
 *
 *   0   magic0   "5012"
 *   4   magic1   "HFS\0"
 *   8   version
 *   12  size     payload length
 *   16  crc      crc32 of the payload
 *   20  flags    all ones while the file is live
 *   24  name     32 bytes, NUL padded
 *   56  reserved
 *   60  hcrc     crc32 over the preceding 60 bytes
 *
 * The header has its own checksum so that a torn write is a header that does
 * not exist, rather than a header naming a length nobody wrote.
 */
typedef struct
{
  uint32_t magic0;
  uint32_t magic1;
  uint32_t version;
  uint32_t size;
  uint32_t crc;
  uint32_t flags;
  char name[SPIFS_NAME_MAX];
  uint32_t reserved;
  uint32_t hcrc;
} spifs_header_t;

_Static_assert(sizeof(spifs_header_t) == SPIFS_HEADER_SIZE, "header layout");

#define SCAN_CHUNK      256

/*- Variables ---------------------------------------------------------------*/
static spifs_file_t g_files[SPIFS_MAX_FILES];
static int g_count;
static bool g_ready;
static uint32_t g_free_base;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The zlib polynomial, bitwise. Same one the loader application and the host
// tools use, so a crc written by any of them means the same thing.
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
static bool header_valid(const spifs_header_t *h)
{
  if (SPIFS_MAGIC0 != h->magic0 || SPIFS_MAGIC1 != h->magic1)
    return false;

  if (SPIFS_VERSION != h->version)
    return false;

  if (crc32_bytes(0, (const uint8_t *)h, SPIFS_HEADER_SIZE - 4) != h->hcrc)
    return false;

  // A length that runs off the end of the writable area is a header that was
  // never finished, whatever its checksum says
  return h->size <= FLASH_RESERVED_BASE;
}

//-----------------------------------------------------------------------------
// How many sectors a file of this size occupies, header included.
static uint32_t file_sectors(uint32_t size)
{
  uint32_t total = size + SPIFS_HEADER_SIZE;

  return (total + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
}

//-----------------------------------------------------------------------------
void spifs_scan(void)
{
  uint32_t limit = flash_size();
  uint32_t addr = 0;

  g_count = 0;
  g_ready = false;
  g_free_base = 0;

  if (0 == limit || !flash_present())
    return;

  // Never past the reserved top: the settings store and the stock firmware's
  // records live there, and a header-shaped run of bytes inside either would
  // otherwise be read as a file
  if (limit > FLASH_RESERVED_BASE)
    limit = FLASH_RESERVED_BASE;

  while (addr < limit)
  {
    spifs_header_t h;

    if (!flash_read(addr, (uint8_t *)&h, sizeof(h)))
      return;                           // bus gave up; leave the table empty

    if (!header_valid(&h))
    {
      addr += FLASH_SECTOR_SIZE;
      continue;
    }

    // A live file also fixes where the next one can start, so the scan steps
    // over its whole extent rather than a sector at a time.
    //
    // Past SPIFS_MAX_FILES the file is stepped over but not recorded: the
    // table is full, and the alternative - stopping the scan - would leave
    // g_free_base short of the real end and put the next write on top of a
    // file that is still there.
    if (SPIFS_FLAGS_LIVE == h.flags && g_count < SPIFS_MAX_FILES)
    {
      spifs_file_t *f = &g_files[g_count++];

      f->header = addr;
      f->data = addr + SPIFS_HEADER_SIZE;
      f->size = h.size;
      f->crc = h.crc;

      memcpy(f->name, h.name, SPIFS_NAME_MAX);
      f->name[SPIFS_NAME_MAX - 1] = 0;
    }

    addr += file_sectors(h.size) * FLASH_SECTOR_SIZE;
    g_free_base = addr;
  }

  g_ready = true;
}

//-----------------------------------------------------------------------------
bool spifs_ready(void)
{
  return g_ready;
}

//-----------------------------------------------------------------------------
int spifs_count(void)
{
  return g_count;
}

//-----------------------------------------------------------------------------
const spifs_file_t *spifs_at(int index)
{
  if (index < 0 || index >= g_count)
    return NULL;

  return &g_files[index];
}

//-----------------------------------------------------------------------------
const spifs_file_t *spifs_find(const char *name)
{
  for (int i = 0; i < g_count; i++)
  {
    if (0 == strncmp(g_files[i].name, name, SPIFS_NAME_MAX))
      return &g_files[i];
  }

  return NULL;
}

//-----------------------------------------------------------------------------
bool spifs_read(const spifs_file_t *file, uint32_t offset, uint8_t *data,
    uint32_t size)
{
  if (NULL == file || offset > file->size || size > file->size - offset)
    return false;

  return flash_read(file->data + offset, data, size);
}

//-----------------------------------------------------------------------------
bool spifs_verify(const spifs_file_t *file)
{
  uint8_t chunk[SCAN_CHUNK];
  uint32_t crc = 0;
  uint32_t left;

  if (NULL == file)
    return false;

  left = file->size;

  for (uint32_t off = 0; left > 0; )
  {
    uint32_t n = (left > sizeof(chunk)) ? sizeof(chunk) : left;

    if (!spifs_read(file, off, chunk, n))
      return false;

    crc = crc32_bytes(crc, chunk, n);
    off += n;
    left -= n;
  }

  return crc == file->crc;
}

//-----------------------------------------------------------------------------
uint32_t spifs_free_base(void)
{
  return g_free_base;
}

//-----------------------------------------------------------------------------
uint32_t spifs_used(void)
{
  uint32_t total = 0;

  for (int i = 0; i < g_count; i++)
    total += g_files[i].size;

  return total;
}
