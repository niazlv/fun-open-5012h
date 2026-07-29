#!/usr/bin/env python3
#
# Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
"""
bkpack - gather BK-0010 tape images into one flash-resident pack.

The alternative was one spifs file per program, and it does not survive contact
with a real collection. spifs holds sixteen files, and it holds sixteen because
its table is 48 bytes a record in a 64 KB TCM with about a hundred bytes to
spare; every file also rounds up to a 4 KB sector, which over eighty tape images
averaging eleven kilobytes throws away a hundred kilobytes; and writing them
means two OpenOCD sessions each, which is a hundred and sixty sessions.

One pack costs one spifs entry, packs the images four-byte aligned, and pushes
in a single session. The device reads its index straight off the chip - see
bk_load_pack_*() - so a pack of eighty programs costs the target no RAM at all.

    python3 tools/bkpack.py games/*.bin -o bkgames.pak
    ./tools/spiflash.py add bkgames.pak

The images themselves are not ours and are not in this repository. They are the
BK's software, and the archives that carry it - archive.pdp-11.org.ru, among
others - are where they come from.
"""

import argparse
import os
import struct
import sys
import zlib

# The on-chip layout, little endian like everything else here. See the same
# description in src/bk/bk_load.h, which is the side that reads it.
#
#   0   magic    "BKPK"
#   4   version
#   8   count
#   12  reserved
#   16  entries[count], 32 bytes each:
#         0   name    24 bytes, NUL padded
#         24  offset  from the start of the pack
#         28  size    payload bytes, the four byte tape header included
#   ...  the payloads, each aligned to 4
MAGIC = b"BKPK"
VERSION = 1
HEADER = 16
ENTRY = 32
NAME = 24

BK_ROM_BASE = 0o100000          # where RAM stops being RAM, see bk_mem.h


def tape_header(blob):
    """The load address and length a BK tape image starts with, or None.

    Same three checks bk_load_bin() makes on the device: four bytes with no
    magic and no checksum read as a valid header whatever they are, so the only
    thing standing between a mistyped filename and sixteen kilobytes of
    somebody's photograph is whether the numbers describe something that fits.
    """
    if len(blob) < 8:
        return None

    addr = blob[0] | blob[1] << 8
    size = blob[2] | blob[3] << 8

    if size == 0 or size + 4 > len(blob):
        return None

    if addr + size > BK_ROM_BASE:
        return None

    return addr, size


def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("images", nargs="+", help="tape images, one program each")
    p.add_argument("-o", "--out", required=True, help="the pack to write")
    p.add_argument("--keep-going", action="store_true",
        help="skip images that are not tape images instead of stopping")
    args = p.parse_args()

    entries = []
    payloads = []
    offset = 0
    seen = {}

    # Two passes, because the index has to be complete before the first payload
    # offset is known: read and check everything, then lay it out.
    for path in sorted(args.images, key=lambda s: os.path.basename(s).lower()):
        with open(path, "rb") as f:
            blob = f.read()

        head = tape_header(blob)

        if head is None:
            msg = f"{path}: not a BK tape image"

            if not args.keep_going:
                raise SystemExit(msg)

            print(f"skip  {msg}", file=sys.stderr)
            continue

        name = os.path.basename(path)

        if name.lower().endswith(".bin"):
            name = name[:-4]

        raw = name.encode("ascii", "replace")

        if len(raw) >= NAME:
            raise SystemExit(f"{path}: name longer than {NAME - 1} bytes")

        if raw in seen:
            raise SystemExit(f"{path}: {name} is already in the pack "
                             f"(from {seen[raw]})")

        seen[raw] = path
        entries.append((raw, len(blob)))
        payloads.append(blob)

    if not entries:
        raise SystemExit("nothing to pack")

    offset = HEADER + ENTRY * len(entries)
    index = bytearray()

    for raw, size in entries:
        rec = bytearray(ENTRY)
        rec[0:len(raw)] = raw
        struct.pack_into("<II", rec, NAME, offset, size)
        index += rec
        offset += (size + 3) & ~3

    out = bytearray()
    out += MAGIC
    out += struct.pack("<III", VERSION, len(entries), 0)
    out += index

    for blob in payloads:
        out += blob
        out += b"\0" * ((-len(blob)) & 3)

    with open(args.out, "wb") as f:
        f.write(out)

    payload = sum(len(b) for b in payloads)

    print(f"{args.out}: {len(entries)} programs, {len(out)} bytes "
          f"({len(out) // 1024} KB), CRC {zlib.crc32(bytes(out)) & 0xFFFFFFFF:08X}")
    print(f"index {HEADER + ENTRY * len(entries)} bytes, "
          f"payload {payload} bytes, {len(out) - payload - HEADER - ENTRY * len(entries)} "
          f"bytes of alignment")


if __name__ == "__main__":
    main()
