/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Borrowing the oscilloscope's memory. See tcm_borrow.h for why it is safe.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "tcm_borrow.h"

/*- Definitions -------------------------------------------------------------*/
// Not zero, not 0xff, and not a plausible pointer or sample value: a byte that
// only comes from here. If the instrument ever draws with this in it, the
// pattern is what says where it came from.
#define POISON      0x5A

/*- External symbols from the linker script ---------------------------------*/
extern uint8_t __borrowed_start;
extern uint8_t __borrowed_end;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void *tcm_borrow_base(void)
{
  return &__borrowed_start;
}

//-----------------------------------------------------------------------------
uint32_t tcm_borrow_size(void)
{
  return (uint32_t)(&__borrowed_end - &__borrowed_start);
}

//-----------------------------------------------------------------------------
// On the way out. Costs one pass over 28 KB, once per application switch, and
// buys the guarantee that nothing quietly kept working on stale instrument
// state: anything still reading here gets a pattern rather than the plausible
// numbers it left behind.
void tcm_borrow_poison(void)
{
  memset(tcm_borrow_base(), POISON, tcm_borrow_size());
}

//-----------------------------------------------------------------------------
// On the way in, before scope_init(). Zeroed, because that is what .bss holds
// at boot and the instrument is known to come up correctly from there.
void tcm_borrow_return(void)
{
  memset(tcm_borrow_base(), 0, tcm_borrow_size());
}
