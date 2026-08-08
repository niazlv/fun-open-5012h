/*
 * Copyright (c) 2019, Alex Taradov <alex@taradov.com>
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

/* The part has 512K of flash. Whether the firmware may use all of it depends
   on where the settings store lives, which is a build-time choice (config.c):

     make                        store on the SPI part, all 512K is code
     make CONFIG_STORE=internal  store in the last 128K sector, code stops at 384K

   The Makefile passes the reserved tail in as __store_reserved, and the assert
   at the bottom is what enforces it. It has to be enforced somewhere: a
   firmware that overruns into the store runs perfectly until the first
   settings save erases its own tail, and that is not a failure anyone should
   have to debug on the device. */
MEMORY
{
  flash (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  ram (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
  tcm (rwx)  : ORIGIN = 0x10000000, LENGTH = 64K
}

__top_flash = ORIGIN(flash) + LENGTH(flash);
__top_tcm = ORIGIN(tcm) + LENGTH(tcm);

ENTRY(irq_handler_reset)

SECTIONS
{
  .text : ALIGN(4)
  {
    FILL(0xff)
    KEEP(*(.vectors))
    *(.text*)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
  } > flash

  . = ALIGN(4);
  _etext = .;

  .uninit_RESERVED : ALIGN(4)
  {
    KEEP(*(.bss.$RESERVED*))
  } > tcm

  .data : ALIGN(4)
  {
    FILL(0xff)
    _data = .;
    *(vtable)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > tcm AT > flash

  /* End of flash data (including .data section stored in flash) */
  __etext = LOADADDR(.data) + SIZEOF(.data);

  /* The instrument's own state, gathered into one range so an application can
     borrow it.

     Nothing overlaps at link time - this is still just .bss - but the four
     modules named here belong to the oscilloscope, and the oscilloscope is an
     application of the launcher like any other: leaving it runs
     scope_cleanup_wrapper, coming back runs scope_init() from scratch. While
     some other application is on screen, every byte in this range is dead.

     That makes it lendable, and safely: .bss starts zeroed at boot, so zeroing
     this range before scope_init() reproduces the boot state exactly. There is
     no list of "which variables survive" to get wrong - the answer is none of
     them, enforced by a memset.

     Order matters. ld assigns an input section to the first output section
     that matches it, so this has to come before the general .bss rule below or
     the objects land there instead. */
  .tcm_borrowed (NOLOAD) : ALIGN(8)
  {
    __borrowed_start = .;
    /* The glob is deliberately wide: the scope is several scope_*.o objects,
       and every one of them must land here or its state escapes the memset.
       Sorted by alignment so the pool packs tight across objects - the order
       is free to change, the whole range is zeroed as one. */
    *scope*.o(SORT_BY_ALIGNMENT(.bss*))
    *fft.o(.bss*)
    *trend.o(.bss*)
    *measure.o(.bss*)
    . = ALIGN(8);

    /* A floor, because how much the instrument happens to declare is not a
       promise to anyone borrowing it. DOOM needs 8K of column cache and all
       of the level's flats resident at once - 21K for a --scale 2 pack - and
       the four modules above came to 88 bytes short of that, so DOOM stopped
       starting and said "flats too big". Nothing in the scope changed on
       purpose; its .bss simply drifted below what the borrower needed, and
       nothing was watching. The ASSERT below still guards the stack. */
    . = MAX(., __borrowed_start + 29K + 512);
    __borrowed_end = .;
  } > tcm

  .bss : ALIGN(4)
  {
    _bss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    PROVIDE(_end = .);
  } > tcm

  PROVIDE(_stack_top = __top_tcm);

  /* The 128K ram region is used as raw pointers by capture.c: 96K capture
     ring at 0x20000000 + 24K storage buffer at 0x20018000. Nothing may be
     linked there. Guard the TCM stack/heap headroom at build time. */
  ASSERT(_stack_top - _end >= 16K, "TCM overflow: less than 16K left for stack+heap")

  /* Everything the image occupies, including the .data initialisers, has to
     end before the reserved tail. __store_reserved is 128K when the settings
     store is in the internal sector and 0 when it is on the SPI part. */
  ASSERT(__etext <= __top_flash - __store_reserved,
      "image runs into the settings store: build with CONFIG_STORE=spi or make it smaller")
}
