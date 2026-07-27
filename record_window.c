/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * See record_window.h.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "record_window.h"

/*- Implementations ---------------------------------------------------------*/
bool record_window_place(int ring_size, int ring_offset, int ring_trigger,
    int window_size, int lo, int hi, RecordWindow *out)
{
  int ring_lo, ring_hi, start, src;

  if (window_size > ring_size || lo > -1 || hi < 1)
    return false;

  if ((int64_t)hi - lo + 1 > window_size)
    return false;

  // Trigger-relative bounds of the ring itself, normalized exactly like
  // capture_get_data() derives min_index/max_index from the same two fields:
  // a trigger at or below the oldest sample means the record runs entirely
  // before it
  ring_lo = ring_offset - ring_trigger;

  if (ring_trigger <= ring_offset)
    ring_lo -= ring_size;

  ring_hi = ring_lo + ring_size - 1;

  // Spread the samples the screen did not ask for evenly around it: they are
  // what the measurements and the spectrum get to work with
  start = lo - (window_size - (hi - lo + 1)) / 2;

  if (start < ring_lo)
    start = ring_lo + 4;

  if (start + window_size - 1 > ring_hi)
    start = ring_hi - window_size + 1;

  // Word-align the source in the ring: the copy moves words, and in
  // dual-channel mode a ring byte's parity IS the ADC it came from, so
  // shifting it would swap the two channels' wiring and calibration.
  // ring_size is a multiple of 4, so the low bits survive the wrap below.
  start -= (ring_trigger + start) & 3;

  if (start < ring_lo || start + window_size - 1 > ring_hi ||
      start > lo || start + window_size - 1 < hi || start > -1)
    return false;

  src = (ring_trigger + start) % ring_size;

  if (src < 0)
    src += ring_size;

  out->start = start;
  out->src   = src;

  return true;
}
