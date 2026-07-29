/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Borrowing the oscilloscope's memory while it is not running.
 */

#ifndef _TCM_BORROW_H_
#define _TCM_BORROW_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*
 * Applications already get all 120 KB of main SRAM, because acquisition is
 * stopped before one is entered and the capture ring is then dead space. This
 * is the same trade for the other pool: the oscilloscope's own state - its
 * frame buffers, the FFT's tables and scratch, the trend history, the
 * measurement accumulators - is about 28 KB of tightly coupled memory that no
 * application can reach and the instrument is not using.
 *
 * What this is NOT: a faster tier. TCM avoids the bus matrix, which matters
 * when the capture DMA is hammering main SRAM - and that is exactly when no
 * application is running. With the DMA stopped both pools are zero wait state
 * for the core, so this buys capacity, not speed. 120 KB becomes ~148 KB.
 *
 * What makes it safe is not a list of variables that may be clobbered but the
 * lifecycle: leaving the oscilloscope tears it down, coming back runs
 * scope_init() from nothing, and the range is zeroed on the way in - which is
 * the state .bss is in at boot. Anything the instrument needs, it rebuilds
 * either way.
 *
 * The rule for a borrower: valid only while your application is the one on
 * screen. Do not hold a pointer into it across a launcher switch, and do not
 * expect to find your own bytes there next time - the range is poisoned when
 * the oscilloscope hands it over, precisely so that a stale reader shows up.
 */

/*- Prototypes --------------------------------------------------------------*/
// The borrowed range. Non-NULL always; the size is what the linker gathered.
void *tcm_borrow_base(void);
uint32_t tcm_borrow_size(void);

// Called by the launcher around the oscilloscope, not by applications.
void tcm_borrow_poison(void);    // leaving the scope: fill with a pattern
void tcm_borrow_return(void);    // entering it: back to the state of a boot

#endif // _TCM_BORROW_H_
