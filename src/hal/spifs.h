/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Files on the SPI part, without a directory to lose.
 */

#ifndef _SPIFS_H_
#define _SPIFS_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
/*
 * The layout question was "an index, or mark the sectors?", and this is the
 * second answer.
 *
 * A central directory is one sector whose loss costs everything: it has to be
 * rewritten every time a file is added, and a power cut inside that rewrite
 * orphans data that is still perfectly good. Keeping two copies and choosing
 * the newer is the usual fix, and it is machinery.
 *
 * Marking the sectors needs none of it. Every file carries its own header at a
 * sector boundary, so a file IS its own directory entry: adding one touches
 * only its own sectors, order on the chip means nothing, a half-written file
 * fails its own checksum and nothing else notices, and there is no structure
 * whose corruption takes the rest with it. The cost is a scan - 1952 sector
 * heads, about 20 ms over the bus - which is paid once and cached.
 *
 * Deleting clears the flags word to zero. NOR programming can only clear bits,
 * so that needs no erase and cannot disturb a neighbour; the space comes back
 * when something erases those sectors to write over them.
 */
#define SPIFS_MAGIC0            0x32313035u     // "5012"
#define SPIFS_MAGIC1            0x00534648u     // "HFS\0"
#define SPIFS_VERSION           1
#define SPIFS_HEADER_SIZE       64
#define SPIFS_NAME_MAX          32
#define SPIFS_MAX_FILES         16

// Live until something clears bits in it. Deleted is any other value, because
// a partial clear is still a deletion somebody meant.
#define SPIFS_FLAGS_LIVE        0xFFFFFFFFu

typedef struct
{
  uint32_t header;              // address of the header on the part
  uint32_t data;                // address of the payload, header + 64
  uint32_t size;                // payload length in bytes
  uint32_t crc;                 // crc32 of the payload, as the writer left it
  char name[SPIFS_NAME_MAX];
} spifs_file_t;

/*- Prototypes --------------------------------------------------------------*/
// Walks the part and rebuilds the table. Safe to call again - it is how a
// caller picks up files written while the device was running.
void spifs_scan(void);

// True once a scan has run and the part answered at all.
bool spifs_ready(void);

int spifs_count(void);
const spifs_file_t *spifs_at(int index);
const spifs_file_t *spifs_find(const char *name);

// Reads from inside a file. Offsets are relative to the payload, so a caller
// never has to know where the file sits.
bool spifs_read(const spifs_file_t *file, uint32_t offset, uint8_t *data,
    uint32_t size);

// Re-reads the whole payload and checks it against the stored crc. Slow by
// construction - it is a bus read of the entire file - and worth it before
// trusting a pack that something is about to run.
bool spifs_verify(const spifs_file_t *file);

// The first sector no file occupies, for a writer choosing where to put one.
uint32_t spifs_free_base(void);

// Total bytes held by live files, for a one-line report.
uint32_t spifs_used(void);

#endif // _SPIFS_H_
