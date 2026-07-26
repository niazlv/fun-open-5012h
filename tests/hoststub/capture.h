/* Host test stub: the coredump ring is placed in a plain array instead of
   the retained SRAM block, so the ring logic can be exercised on the host. */
#ifndef _HOSTSTUB_CAPTURE_H_
#define _HOSTSTUB_CAPTURE_H_

#define COREDUMP_RAM_SIZE  (3 * 1024)

extern unsigned char g_coredump_fake_ram[COREDUMP_RAM_SIZE];

#define COREDUMP_RAM_BASE  (g_coredump_fake_ram)

#endif
