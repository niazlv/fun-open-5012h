/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * The asset pack, linked into flash.
 *
 * tools/wadpack.py writes doom_assets.bin from a real IWAD; it is not in the
 * repository because it carries the WAD's own artwork. The Makefile creates an
 * empty one if it is missing so the firmware still builds - the application
 * then detects the missing pack and says how to make it.
 *
 * .incbin rather than a generated C array: 200 KB of initialisers would be
 * pointless work for the compiler, and this way the data lands in .rodata with
 * the alignment the engine's 32-bit tables need.
 */

    .section .rodata.doom_assets, "a"
    .align 4

    .global doom_assets
doom_assets:
    .incbin "../doom/doom_assets.bin"
doom_assets_end:

    /* With an empty pack the symbol would otherwise point at whatever .rodata
       follows, and the header check would be reading someone else's data */
    .word 0, 0, 0, 0

    .global doom_assets_size
doom_assets_size:
    .word doom_assets_end - doom_assets
