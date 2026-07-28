/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Placement of the full-rate record window inside the capture ring.
 *
 * Pure C, no hardware dependencies: the index arithmetic here decides what
 * every consumer of a record is allowed to read, so it is swept exhaustively
 * on the host (see tests/host_test.c).
 */

#ifndef _RECORD_WINDOW_H_
#define _RECORD_WINDOW_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int start; // trigger-relative index of the window's first sample, always
             // negative: the trigger has to sit strictly inside
  int src;   // ring index of that same sample, always a multiple of 4
} RecordWindow;

/*- Prototypes --------------------------------------------------------------*/
// Place a window_size window inside a ring_size ring so that it holds the
// trigger (trigger-relative index 0) and the whole range [lo, hi], spreading
// whatever samples are left over evenly around it.
//
// ring_offset is the ring index of the oldest sample, ring_trigger that of
// the trigger; lo/hi are trigger-relative sample indices and must already
// straddle the trigger (lo <= -1, hi >= 1).
//
// Returns false when no placement satisfies all of that - the caller then has
// to fall back to a decimated whole-ring record.
bool record_window_place(int ring_size, int ring_offset, int ring_trigger,
    int window_size, int lo, int hi, RecordWindow *out);

#endif // _RECORD_WINDOW_H_
