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
