#!/usr/bin/env python3
#
# Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
#
"""Read and write the board's SPI NOR part from a PC, over SWD.

Nothing on this device exposes that chip to the outside world: the USB socket
carries SWDIO/SWCLK and not USB, and the only thing wired to the flash is the
MCU. So the path is PC -> debugger -> MCU's SPI0 -> chip, and this drives it by
halting the core and using SPI0 as a shift register (tools/spiflash.tcl does
the poking; this side only converts between files and hex, because OpenOCD's
Jim Tcl has no binary I/O).

That costs three SWD transactions per byte - about a kilobyte a second. Right
for a ROM, a directory or a rescue read; for megabytes the firmware has to do
the shifting itself.

  ./tools/spiflash.py probe
  ./tools/spiflash.py dump 0x7F0000 65536 factory.bin
  ./tools/spiflash.py write 0x1000 bk/monitor.rom
  ./tools/spiflash.py verify 0x1000 bk/monitor.rom

--interface and --target follow the same names the README uses for flashing.
"""

import argparse
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TCL = os.path.join(HERE, "spiflash.tcl")

CHUNK = 4096            # bytes per OpenOCD command, keeps arg lists sane
MAILBOX_CHUNK = 32768   # the loader application's buffer, see spi_tool.c
RESERVED_BASE = 0x7A0000  # settings store, then the stock firmware's data


def openocd(args, commands):
    """Runs one OpenOCD session with `commands` and returns its output."""
    argv = ["openocd",
            "-f", args.interface,
            "-c", "transport select swd",
            "-c", f"adapter speed {args.speed}",
            "-f", args.target,
            "-f", TCL,
            "-c", "spi_init"]

    for c in commands:
        argv += ["-c", c]

    argv += ["-c", "resume", "-c", "shutdown"]

    p = subprocess.run(argv, capture_output=True, text=True)
    out = p.stdout + p.stderr

    if p.returncode != 0:
        sys.stderr.write(out)
        raise SystemExit(f"openocd failed ({p.returncode})")

    return out


def cmd_probe(args):
    out = openocd(args, ["spi_id", "spi_dump 0 64"])
    print("\n".join(l for l in out.splitlines()
                    if l.startswith(("JEDEC", "0000"))))


def cmd_dump(args):
    addr, count = int(args.addr, 0), int(args.count, 0)
    commands = [f"spi_read_hex {addr + off} {min(CHUNK, count - off)}"
                for off in range(0, count, CHUNK)]

    out = openocd(args, commands)
    blob = bytearray()

    # Chunks come back in the order they were asked for, each tagged with the
    # address it was read from - checked rather than assumed, because a
    # silently dropped chunk would produce a plausible short file.
    expect = addr

    for line in out.splitlines():
        m = re.match(r"HEX (\d+) ([0-9A-F]*)$", line.strip())

        if not m:
            continue

        if int(m.group(1)) != expect:
            raise SystemExit(f"chunk out of order at {expect:#x}")

        piece = bytes.fromhex(m.group(2))
        blob += piece
        expect += len(piece)

    if len(blob) != count:
        raise SystemExit(f"read {len(blob)} of {count} bytes")

    with open(args.path, "wb") as f:
        f.write(blob)

    print(f"wrote {args.path} ({len(blob)} bytes from {addr:#08x})")


def cmd_write(args):
    addr = int(args.addr, 0)

    with open(args.path, "rb") as f:
        data = f.read()

    # Erase covers whole sectors, so a write that starts or ends mid-sector
    # destroys what shares those sectors. Say so rather than discovering it
    # afterwards.
    first = addr & ~0xFFF
    last = (addr + len(data) - 1) & ~0xFFF

    if not args.yes:
        print(f"{len(data)} bytes at {addr:#08x}")
        print(f"erases sectors {first:#08x}..{last:#08x} "
              f"({(last - first) // 4096 + 1} x 4 KB)")

        if input("proceed? [y/N] ").strip().lower() not in ("y", "yes"):
            raise SystemExit("nothing written")

    commands = [f"spi_erase_sector {s}" for s in range(first, last + 1, 4096)]
    commands += [f"spi_write_hex {addr + off} {data[off:off + CHUNK].hex()}"
                 for off in range(0, len(data), CHUNK)]

    out = openocd(args, commands)
    print("\n".join(l for l in out.splitlines() if l.startswith("wrote")))


def openocd_mailbox(args, commands):
    """Same, but through the loader application: no halt, and much faster."""
    argv = ["openocd",
            "-f", args.interface,
            "-c", "transport select swd",
            "-c", f"adapter speed {args.speed}",
            "-f", args.target,
            "-f", TCL,
            "-c", "mb_init"]

    for c in commands:
        argv += ["-c", c]

    argv += ["-c", "shutdown"]

    p = subprocess.run(argv, capture_output=True, text=True)
    out = p.stdout + p.stderr

    if p.returncode != 0:
        sys.stderr.write(out)
        raise SystemExit(f"openocd failed ({p.returncode})")

    return out


def cmd_push(args):
    """Writes a file through the device's own driver. Needs the SPI Flash
    Loader application open on the screen - that is the safety interlock, not
    an oversight."""
    addr = int(args.addr, 0)

    with open(args.path, "rb") as f:
        data = f.read()

    if addr % 4096:
        raise SystemExit("start at a 4 KB sector boundary")

    # One temporary file per mailbox-sized chunk: OpenOCD's load_image takes a
    # whole file and an address, and there is no way to name a window inside
    # one that lands anywhere but its own address.
    import tempfile
    import zlib

    tmp = tempfile.mkdtemp(prefix="spiflash-")
    commands = [f"mb_run 1 {addr} {len(data)}"]     # erase what we will fill

    # The reserved top of the part holds the settings store and the factory
    # page. The device refuses to write there unless the override is armed,
    # and arming it automatically because the range happens to reach that far
    # would make the interlock worthless - the whole point is that the only
    # way in is somebody typing --unsafe.
    if addr + len(data) > RESERVED_BASE:
        if not args.unsafe:
            raise SystemExit(
                f"{addr:#08x}..{addr + len(data):#08x} runs into the reserved "
                f"top of the part ({RESERVED_BASE:#08x}: settings store, then "
                f"the stock firmware's saved records). Pass --unsafe if that "
                f"is really the intent.")

        commands.insert(0, "mb_unsafe")
        print("warning: writing into the reserved top of the part")

    for off in range(0, len(data), MAILBOX_CHUNK):
        piece = data[off:off + MAILBOX_CHUNK]
        path = os.path.join(tmp, f"c{off:08x}.bin")

        with open(path, "wb") as f:
            f.write(piece)

        commands.append(f"load_image {path} 0x20000020 bin")
        commands.append(f"mb_run 2 {addr + off} {len(piece)}")

    commands.append(f"mb_crc {addr} {len(data)}")

    out = openocd_mailbox(args, commands)

    want = zlib.crc32(data) & 0xFFFFFFFF
    got = None

    for line in out.splitlines():
        m = re.match(r"CRC ([0-9A-F]{8})$", line.strip())

        if m:
            got = int(m.group(1), 16)

    for f in os.listdir(tmp):
        os.unlink(os.path.join(tmp, f))

    os.rmdir(tmp)

    if got is None:
        raise SystemExit("no CRC came back")

    if got != want:
        raise SystemExit(f"CRC differs: chip {got:08X}, file {want:08X}")

    print(f"pushed {len(data)} bytes to {addr:#08x}, CRC {want:08X} verified")


def cmd_pull(args):
    """Reads through the device's own driver - the fast counterpart of push.
    Needs the SPI Flash Loader application open."""
    addr, count = int(args.addr, 0), int(args.count, 0)

    import tempfile

    tmp = tempfile.mkdtemp(prefix="spiflash-")
    parts = []
    commands = []

    for off in range(0, count, MAILBOX_CHUNK):
        n = min(MAILBOX_CHUNK, count - off)
        path = os.path.join(tmp, f"r{off:08x}.bin")
        parts.append((path, n))
        commands.append(f"mb_read {addr + off} {n} {path}")

    openocd_mailbox(args, commands)

    blob = bytearray()

    for path, n in parts:
        with open(path, "rb") as f:
            piece = f.read()

        if len(piece) != n:
            raise SystemExit(f"{path}: {len(piece)} bytes, expected {n}")

        blob += piece
        os.unlink(path)

    os.rmdir(tmp)

    with open(args.path, "wb") as f:
        f.write(blob)

    print(f"pulled {len(blob)} bytes from {addr:#08x} into {args.path}")


# The on-chip file header, see src/hal/spifs.h. Everything is little endian and
# the payload starts right after it, at the next 64 byte boundary of the sector
# the header opens.
SPIFS_MAGIC = b"5012HFS\0"
SPIFS_HEADER = 64
SPIFS_NAME = 32
SPIFS_SECTOR = 4096
SPIFS_VERSION = 1


def spifs_header(name, payload):
    import struct
    import zlib

    raw = name.encode()

    if len(raw) >= SPIFS_NAME:
        raise SystemExit(f"name longer than {SPIFS_NAME - 1} bytes: {name}")

    h = bytearray(SPIFS_HEADER)
    h[0:8] = SPIFS_MAGIC
    struct.pack_into("<III", h, 8, SPIFS_VERSION, len(payload),
                     zlib.crc32(payload) & 0xFFFFFFFF)
    struct.pack_into("<I", h, 20, 0xFFFFFFFF)           # flags: live
    h[24:24 + len(raw)] = raw
    struct.pack_into("<I", h, 60, zlib.crc32(bytes(h[:60])) & 0xFFFFFFFF)

    return bytes(h)


def spifs_list(args):
    """Asks the device for its table. Returns (files, free_base)."""
    import struct
    import tempfile

    tmp = tempfile.mkdtemp(prefix="spifs-")
    path = os.path.join(tmp, "table.bin")

    out = openocd_mailbox(args, [f"mb_scan {path}"])

    m = re.search(r"FILES (\d+) FREE (\d+)", out)

    if not m:
        raise SystemExit("the device did not answer the scan")

    count, free_base = int(m.group(1)), int(m.group(2))
    files = []

    if count:
        with open(path, "rb") as f:
            table = f.read()

        os.unlink(path)

        for i in range(count):
            rec = table[i * 48:(i + 1) * 48]
            header, data, size, crc = struct.unpack_from("<IIII", rec, 0)
            name = rec[16:48].split(b"\0")[0].decode("ascii", "replace")
            files.append(dict(name=name, header=header, data=data,
                              size=size, crc=crc))

    os.rmdir(tmp)

    return files, free_base


def cmd_ls(args):
    files, free_base = spifs_list(args)

    if not files:
        print("no files")
    else:
        print(f"{'name':24} {'size':>9}  {'at':>8}  {'crc32':>8}")

        for f in files:
            print(f"{f['name']:24} {f['size']:>9}  {f['header']:08X}  "
                  f"{f['crc']:08X}")

    print(f"\nnext free sector {free_base:#08x}, "
          f"{(RESERVED_BASE - free_base) // 1024} KB left")


def cmd_add(args):
    """Writes a file wherever there is room. The caller never names an address:
    that is the whole point of the layout - a file carries its own header, so
    the order things were written in means nothing."""
    with open(args.path, "rb") as f:
        payload = f.read()

    name = args.name or os.path.basename(args.path)
    files, free_base = spifs_list(args)

    if any(f["name"] == name for f in files):
        raise SystemExit(f"{name} is already there - rm it first")

    blob = spifs_header(name, payload) + payload
    need = -(-len(blob) // SPIFS_SECTOR) * SPIFS_SECTOR

    if free_base + need > RESERVED_BASE:
        raise SystemExit(f"{need // 1024} KB does not fit in the "
                         f"{(RESERVED_BASE - free_base) // 1024} KB left")

    tmp = args.path + ".spifs"

    with open(tmp, "wb") as f:
        f.write(blob)

    try:
        args.addr = str(free_base)
        args.path = tmp
        args.unsafe = False
        args.yes = True     # the free sectors are what it was told to fill
        cmd_push(args)
    finally:
        os.unlink(tmp)

    print(f"{name}: {len(payload)} bytes at {free_base:#08x}")


def cmd_rm(args):
    """Clears the flags word. NOR programming can only clear bits, so this
    needs no erase - the sectors come back when something writes over them."""
    files, _ = spifs_list(args)
    target = next((f for f in files if f["name"] == args.name), None)

    if target is None:
        raise SystemExit(f"no file called {args.name}")

    import tempfile

    tmp = tempfile.mkdtemp(prefix="spifs-")
    path = os.path.join(tmp, "flags.bin")

    with open(path, "wb") as f:
        f.write(b"\0\0\0\0")

    openocd_mailbox(args, [
        f"load_image {path} 0x20000020 bin",
        f"mb_run 2 {target['header'] + 20} 4",
    ])

    os.unlink(path)
    os.rmdir(tmp)

    print(f"{args.name} removed")


def cmd_crc(args):
    out = openocd_mailbox(args,
        [f"mb_crc {int(args.addr, 0)} {int(args.count, 0)}"])

    print("\n".join(l for l in out.splitlines() if l.startswith("CRC")))


def cmd_verify(args):
    addr = int(args.addr, 0)

    with open(args.path, "rb") as f:
        data = f.read()

    commands = [f"spi_read_hex {addr + off} {min(CHUNK, len(data) - off)}"
                for off in range(0, len(data), CHUNK)]

    out = openocd(args, commands)
    blob = bytearray()

    for line in out.splitlines():
        m = re.match(r"HEX (\d+) ([0-9A-F]*)$", line.strip())

        if m:
            blob += bytes.fromhex(m.group(2))

    if bytes(blob) == data:
        print(f"verified {len(data)} bytes at {addr:#08x}")
        return

    for i, (a, b) in enumerate(zip(blob, data)):
        if a != b:
            raise SystemExit(f"differs at {addr + i:#08x}: "
                             f"chip {a:02X}, file {b:02X}")

    raise SystemExit(f"short read: {len(blob)} of {len(data)} bytes")


def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--interface", default="interface/cmsis-dap.cfg")
    p.add_argument("--target", default="target/stm32f4x.cfg")
    p.add_argument("--speed", default="2000")

    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("probe").set_defaults(func=cmd_probe)

    d = sub.add_parser("dump")
    d.add_argument("addr")
    d.add_argument("count")
    d.add_argument("path")
    d.set_defaults(func=cmd_dump)

    w = sub.add_parser("write")
    w.add_argument("addr")
    w.add_argument("path")
    w.add_argument("-y", "--yes", action="store_true",
        help="do not ask before erasing")
    w.set_defaults(func=cmd_write)

    v = sub.add_parser("verify")
    v.add_argument("addr")
    v.add_argument("path")
    v.set_defaults(func=cmd_verify)

    u = sub.add_parser("push",
        help="fast write through the device's SPI Flash Loader application")
    u.add_argument("addr")
    u.add_argument("path")
    u.add_argument("--unsafe", action="store_true",
        help="allow writing above 0x7A0000 (settings store, stock data)")
    u.set_defaults(func=cmd_push)

    l = sub.add_parser("pull",
        help="fast read through the device's SPI Flash Loader application")
    l.add_argument("addr")
    l.add_argument("count")
    l.add_argument("path")
    l.set_defaults(func=cmd_pull)

    ls = sub.add_parser("ls", help="what is on the chip")
    ls.set_defaults(func=cmd_ls)

    a = sub.add_parser("add", help="write a file, wherever it fits")
    a.add_argument("path")
    a.add_argument("--name", help="name on the chip (default: the basename)")
    a.set_defaults(func=cmd_add)

    r = sub.add_parser("rm", help="mark a file deleted")
    r.add_argument("name")
    r.set_defaults(func=cmd_rm)

    c = sub.add_parser("crc", help="crc32 of a range, computed on the device")
    c.add_argument("addr")
    c.add_argument("count")
    c.set_defaults(func=cmd_crc)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
