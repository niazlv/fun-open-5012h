#!/usr/bin/env python3
"""
wadpack - turn a real DOOM IWAD into a flash-resident asset pack.

The device has 128 KB of SRAM and no memory-mapped external storage, so nothing
that DOOM normally builds at runtime can be built at runtime here: textures are
composited from their patches, the trig and projection tables are evaluated, and
the level is converted, all on the host. What lands on the target is a blob that
the renderer reads through const pointers - it costs zero RAM.

Level geometry that already has the layout the engine wants is copied verbatim
out of the WAD (the WAD is little-endian, so is the Cortex-M4). Only sidedefs
and sectors are rewritten, to replace 8-byte texture names with indices.

Usage:
    python3 tools/wadpack.py doom1.wad -o doom/doom_assets.bin [--map E1M1]
"""

import argparse
import math
import os
import struct
import sys

# ---------------------------------------------------------------------------
# WAD access
# ---------------------------------------------------------------------------


class Wad:
    def __init__(self, path):
        with open(path, "rb") as f:
            self.data = f.read()

        magic, count, dir_off = struct.unpack("<4sii", self.data[:12])

        if magic not in (b"IWAD", b"PWAD"):
            raise SystemExit("%s is not a WAD file" % path)

        self.entries = []
        self.index = {}

        for i in range(count):
            off, size, name = struct.unpack("<ii8s", self.data[dir_off + i * 16:dir_off + i * 16 + 16])
            name = name.rstrip(b"\0").decode("latin1").upper()
            self.entries.append((name, off, size))
            # First definition wins, which is how DOOM resolves IWAD lumps
            self.index.setdefault(name, i)

    def has(self, name):
        return name.upper() in self.index

    def get(self, name):
        return self.at(self.index[name.upper()])

    def at(self, i):
        _, off, size = self.entries[i]
        return self.data[off:off + size]

    def find(self, name):
        return self.index[name.upper()]

    def between(self, start, end):
        """Lump names between two markers, e.g. F_START/F_END."""
        a = self.index[start]
        b = self.index[end]
        return [self.entries[i][0] for i in range(a + 1, b) if self.entries[i][2] > 0]


# ---------------------------------------------------------------------------
# Graphics
# ---------------------------------------------------------------------------

TRANSPARENT = 0xFF   # index into our coverage mask, never a palette index


def parse_patch(raw):
    """DOOM patch_t -> (w, h, xoff, yoff, columns[[ (top, pixels) ]])."""
    w, h, xoff, yoff = struct.unpack("<hhhh", raw[:8])
    col_ofs = struct.unpack("<%di" % w, raw[8:8 + 4 * w])
    columns = []

    for c in range(w):
        p = col_ofs[c]
        posts = []

        # A post is: top, length, pad, pixels..., pad. 0xff ends the column.
        while p < len(raw) and raw[p] != 0xFF:
            top = raw[p]
            length = raw[p + 1]
            posts.append((top, raw[p + 3:p + 3 + length]))
            p += length + 4

        columns.append(posts)

    return w, h, xoff, yoff, columns


class TextureSet:
    """Composites TEXTUREx entries out of PNAMES patches, column-major."""

    def __init__(self, wad):
        self.wad = wad
        self.defs = {}
        self.order = []

        pn = wad.get("PNAMES")
        count = struct.unpack("<i", pn[:4])[0]
        self.pnames = [pn[4 + 8 * i:12 + 8 * i].rstrip(b"\0").decode("latin1").upper()
                       for i in range(count)]

        for lump in ("TEXTURE1", "TEXTURE2"):
            if wad.has(lump):
                self._read_texture_lump(wad.get(lump))

    def _read_texture_lump(self, raw):
        count = struct.unpack("<i", raw[:4])[0]
        offsets = struct.unpack("<%di" % count, raw[4:4 + 4 * count])

        for off in offsets:
            name = raw[off:off + 8].rstrip(b"\0").decode("latin1").upper()
            width, height = struct.unpack("<hh", raw[off + 12:off + 16])
            patch_count = struct.unpack("<h", raw[off + 20:off + 22])[0]
            patches = []

            for j in range(patch_count):
                x, y, pnum = struct.unpack("<hhh", raw[off + 22 + j * 10:off + 28 + j * 10])
                patches.append((x, y, self.pnames[pnum]))

            if name not in self.defs:
                self.defs[name] = (width, height, patches)
                self.order.append(name)

    def composite(self, name):
        """-> (width, height, pixels column-major, covered column-major)."""
        width, height, patches = self.defs[name]
        pixels = bytearray(width * height)
        covered = bytearray(width * height)

        for ox, oy, pname in patches:
            pw, ph, _, _, columns = parse_patch(self.wad.get(pname))

            for c in range(pw):
                x = ox + c

                if x < 0 or x >= width:
                    continue

                base = x * height

                for top, data in columns[c]:
                    for k, value in enumerate(data):
                        y = oy + top + k

                        if 0 <= y < height:
                            pixels[base + y] = value
                            covered[base + y] = 1

        return width, height, bytes(pixels), bytes(covered)


def shrink_column(column, scale):
    """Point-sample a texture column. Point sampling, not averaging: DOOM art is
    palette-indexed, and averaging indices mixes unrelated colors."""
    if scale == 1:
        return column

    return bytes(column[i * scale] for i in range(len(column) // scale))


def shrink_flat(flat, scale):
    if scale == 1:
        return flat

    size = 64 // scale
    out = bytearray(size * size)

    for y in range(size):
        for x in range(size):
            out[y * size + x] = flat[(y * scale) * 64 + x * scale]

    return bytes(out)


# ---------------------------------------------------------------------------
# Tables
#
# The formulas are the ones the original tables.c was generated with; the
# renderer's fixed-point math only lines up if these match exactly.
# ---------------------------------------------------------------------------

FRACBITS = 16
FRACUNIT = 1 << FRACBITS
FINEANGLES = 8192
ANG90 = 0x40000000
ANG180 = 0x80000000


def clamp_fixed(v):
    if v > 0x7FFFFFFF:
        return 0x7FFFFFFF
    if v < -0x80000000:
        return -0x80000000
    return int(v)


def gen_sine_quarter():
    """sin((i + 0.5) * 2pi / 8192) for the first quarter wave.

    The engine mirrors this into the full finesine[] range; storing the quarter
    costs 8 KB of flash instead of 40 KB and the symmetry is exact because of
    the half-step offset."""
    return [int(round(math.sin((i + 0.5) * math.pi * 2 / FINEANGLES) * FRACUNIT))
            for i in range(FINEANGLES // 4)]


def gen_tangent_half():
    """finetangent[] for the positive half; the engine negates for the other.

    Vanilla's table covers -pi/2..pi/2 over 4096 entries as
    tan((i - 2048 + 0.5) * 2pi / 8192); entries i and 4095-i are exact negatives
    of each other, so only 2048..4095 is stored."""
    out = []

    for i in range(FINEANGLES // 4, FINEANGLES // 2):
        a = (i - FINEANGLES // 4 + 0.5) * math.pi * 2 / FINEANGLES
        out.append(clamp_fixed(math.tan(a) * FRACUNIT))

    return out


def gen_tantoangle():
    """atan(i / 2048) as a binary angle, 2049 entries (SLOPERANGE + 1)."""
    return [int(math.atan(i / 2048.0) * ANG180 / math.pi) & 0xFFFFFFFF
            for i in range(2049)]


def gen_projection(width, height, fov_deg):
    """viewangletox / xtoviewangle / yslope / distscale, exactly as
    R_InitTextureMapping and R_InitTables build them, but for our screen."""
    finetangent = []

    for i in range(FINEANGLES // 2):
        a = (i - FINEANGLES // 4 + 0.5) * math.pi * 2 / FINEANGLES
        finetangent.append(clamp_fixed(math.tan(a) * FRACUNIT))

    def finesine(i):
        i &= FINEANGLES - 1
        return int(round(math.sin((i + 0.5) * math.pi * 2 / FINEANGLES) * FRACUNIT))

    def finecosine(i):
        return finesine(i + FINEANGLES // 4)

    center_x = width // 2
    center_y = height // 2

    # focallength so that the requested horizontal FOV maps to the screen edge:
    # FixedDiv(centerxfrac, finetangent[FINEANGLES/4 + FIELDOFVIEW/2])
    half_fov_fine = int(round((fov_deg / 2.0) / 360.0 * FINEANGLES))
    centerxfrac = center_x * FRACUNIT
    focal_length = (centerxfrac << FRACBITS) // finetangent[FINEANGLES // 4 + half_fov_fine]

    viewangletox = []

    for i in range(FINEANGLES // 2):
        t = finetangent[i]

        if t > 2 * FRACUNIT:
            x = -1
        elif t < -2 * FRACUNIT:
            x = width + 1
        else:
            t = (t * focal_length) >> FRACBITS
            x = (center_x * FRACUNIT - t + FRACUNIT - 1) >> FRACBITS
            x = max(-1, min(width + 1, x))

        viewangletox.append(x)

    # Scan viewangletox backwards for the angle that first maps to each column
    xtoviewangle = []

    for x in range(width + 1):
        i = 0

        while viewangletox[i] > x:
            i += 1

        xtoviewangle.append((i << 19) - ANG90 & 0xFFFFFFFF)

    # Clamp the table so no angle maps outside the screen
    for i in range(FINEANGLES // 2):
        if viewangletox[i] == -1:
            viewangletox[i] = 0
        elif viewangletox[i] == width + 1:
            viewangletox[i] = width

    yslope = []

    for i in range(height):
        dy = ((i - height // 2) << FRACBITS) + FRACUNIT // 2
        dy = abs(dy)
        yslope.append(int((width // 2 * FRACUNIT) * FRACUNIT / dy) if dy else 0)

    distscale = []

    for i in range(width):
        cosang = abs(finecosine(xtoviewangle[i] >> 19))
        distscale.append(int(FRACUNIT * FRACUNIT / cosang) if cosang else 0)

    return focal_length, center_y, viewangletox, xtoviewangle, yslope, distscale


LIGHTLEVELS = 16
LIGHTSEGSHIFT = 4
MAXLIGHTSCALE = 48
LIGHTSCALESHIFT = 12
MAXLIGHTZ = 128
LIGHTZSHIFT = 20
NUMCOLORMAPS = 32
DISTMAP = 2


def gen_light_tables(width):
    """scalelight / zlight as colormap indices rather than pointers - 2.7 KB of
    flash instead of 11 KB of pointers in RAM."""
    scalelight = bytearray(LIGHTLEVELS * MAXLIGHTSCALE)
    zlight = bytearray(LIGHTLEVELS * MAXLIGHTZ)

    for i in range(LIGHTLEVELS):
        start = ((LIGHTLEVELS - 1 - i) * 2) * NUMCOLORMAPS // LIGHTLEVELS

        for j in range(MAXLIGHTSCALE):
            level = start - j * 320 // width // DISTMAP
            level = max(0, min(NUMCOLORMAPS - 1, level))
            scalelight[i * MAXLIGHTSCALE + j] = level

        for j in range(MAXLIGHTZ):
            scale = ((160 * FRACUNIT) << FRACBITS) // ((j + 1) << LIGHTZSHIFT)
            scale >>= LIGHTSCALESHIFT
            level = start - scale // DISTMAP
            level = max(0, min(NUMCOLORMAPS - 1, level))
            zlight[i * MAXLIGHTZ + j] = level

    return bytes(scalelight), bytes(zlight)


# ---------------------------------------------------------------------------
# Blob writer
# ---------------------------------------------------------------------------


class Blob:
    """Named sections, 4-byte aligned, offsets relative to the blob base."""

    MAGIC = b"DPK1"
    NAME_LEN = 12

    def __init__(self):
        self.body = bytearray()
        self.sections = []

    def add(self, name, data):
        assert len(name) <= self.NAME_LEN, name

        while len(self.body) % 4:
            self.body.append(0)

        off = len(self.body)
        self.body += data
        self.sections.append((name, off, len(data)))

        return off

    def build(self):
        header_len = 16
        dir_len = len(self.sections) * (self.NAME_LEN + 8)
        base = header_len + dir_len
        base += (-base) % 4

        out = bytearray()
        out += struct.pack("<4sIII", self.MAGIC, 1, len(self.sections), base)

        for name, off, size in self.sections:
            out += name.encode("ascii").ljust(self.NAME_LEN, b"\0")
            out += struct.pack("<II", base + off, size)

        out += b"\0" * (base - len(out))
        out += self.body

        return bytes(out)

    def report(self):
        rows = sorted(self.sections, key=lambda s: -s[2])
        print("  %-12s %9s" % ("section", "bytes"))

        for name, _, size in rows:
            print("  %-12s %9d" % (name, size))


# ---------------------------------------------------------------------------
# Level conversion
# ---------------------------------------------------------------------------

MAP_LUMPS = ["THINGS", "LINEDEFS", "SIDEDEFS", "VERTEXES", "SEGS",
             "SSECTORS", "NODES", "SECTORS", "REJECT", "BLOCKMAP"]


def read_map(wad, mapname):
    start = wad.find(mapname)
    lumps = {}

    for i in range(start + 1, min(start + 12, len(wad.entries))):
        name = wad.entries[i][0]

        if name not in MAP_LUMPS:
            break

        lumps[name] = wad.at(i)

    missing = [n for n in MAP_LUMPS if n not in lumps and n != "REJECT"]

    if missing:
        raise SystemExit("map %s is missing %s" % (mapname, ", ".join(missing)))

    return lumps


def convert_level(lumps, tex_index, flat_index):
    """Rewrite the two lumps that carry names; everything else is already in the
    layout the engine reads."""
    raw = lumps["SIDEDEFS"]
    sides = bytearray()
    used_tex = set()

    for i in range(len(raw) // 30):
        s = raw[i * 30:(i + 1) * 30]
        xoff, yoff = struct.unpack("<hh", s[0:4])
        names = [s[4:12], s[12:20], s[20:28]]
        sector = struct.unpack("<h", s[28:30])[0]
        ids = []

        for nm in names:
            nm = nm.rstrip(b"\0").decode("latin1").upper()

            if nm == "-" or nm not in tex_index:
                ids.append(0xFFFF)
            else:
                ids.append(tex_index[nm])
                used_tex.add(nm)

        sides += struct.pack("<hhHHHH", xoff, yoff, ids[0], ids[1], ids[2], sector)

    raw = lumps["SECTORS"]
    sectors = bytearray()
    used_flats = set()

    for i in range(len(raw) // 26):
        s = raw[i * 26:(i + 1) * 26]
        floorh, ceilh = struct.unpack("<hh", s[0:4])
        floorpic = s[4:12].rstrip(b"\0").decode("latin1").upper()
        ceilpic = s[12:20].rstrip(b"\0").decode("latin1").upper()
        light, special, tag = struct.unpack("<hhh", s[20:26])

        fids = []

        for nm in (floorpic, ceilpic):
            if nm == "F_SKY1":
                fids.append(0xFFFF)     # the sky is not a flat
            elif nm in flat_index:
                fids.append(flat_index[nm])
                used_flats.add(nm)
            else:
                fids.append(0)

        sectors += struct.pack("<hhHHhhh", floorh, ceilh, fids[0], fids[1], light, special, tag)

    return bytes(sides), bytes(sectors), used_tex, used_flats


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wad", help="IWAD to read (doom1.wad, freedoom1.wad, ...)")
    ap.add_argument("-o", "--output", default="doom/doom_assets.bin")
    ap.add_argument("--map", default="E1M1", help="map to pack (default E1M1)")
    ap.add_argument("--scale", type=int, default=2, choices=(1, 2),
                    help="texture/flat downscale, 2 halves both axes (default)")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=200)
    ap.add_argument("--fov", type=float, default=90.0)
    ap.add_argument("--header", default=None, help="also write a C header with the sizes")
    args = ap.parse_args()

    wad = Wad(args.wad)
    print("%s: %d lumps" % (os.path.basename(args.wad), len(wad.entries)))

    if not wad.has(args.map):
        raise SystemExit("no map named %s in this WAD" % args.map)

    lumps = read_map(wad, args.map)
    textures = TextureSet(wad)

    # --- which textures and flats does this map actually reference ----------
    raw = lumps["SIDEDEFS"]
    wanted_tex = set()

    for i in range(len(raw) // 30):
        s = raw[i * 30:(i + 1) * 30]

        for k in (4, 12, 20):
            nm = s[k:k + 8].rstrip(b"\0").decode("latin1").upper()

            if nm != "-" and nm in textures.defs:
                wanted_tex.add(nm)

    sky = "SKY1" if wad.has("SKY1") else textures.order[0]
    wanted_tex.add(sky)

    raw = lumps["SECTORS"]
    wanted_flats = set()

    for i in range(len(raw) // 26):
        s = raw[i * 26:(i + 1) * 26]

        for k in (4, 12):
            nm = s[k:k + 8].rstrip(b"\0").decode("latin1").upper()

            if nm != "F_SKY1":
                wanted_flats.add(nm)

    tex_names = sorted(wanted_tex)
    flat_names = sorted(wanted_flats)
    tex_index = {n: i for i, n in enumerate(tex_names)}
    flat_index = {n: i for i, n in enumerate(flat_names)}

    blob = Blob()
    scale = args.scale

    # --- palette, in the display's own format --------------------------------
    playpal = wad.get("PLAYPAL")[:768]
    pal = bytearray()

    for i in range(256):
        r, g, b = playpal[i * 3], playpal[i * 3 + 1], playpal[i * 3 + 2]
        pal += struct.pack("<H", ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

    blob.add("PALETTE", bytes(pal))
    blob.add("COLORMAP", wad.get("COLORMAP")[:NUMCOLORMAPS * 256])

    # --- trig and projection -------------------------------------------------
    blob.add("SINQ", struct.pack("<%di" % (FINEANGLES // 4), *gen_sine_quarter()))
    blob.add("TANH", struct.pack("<%di" % (FINEANGLES // 4), *gen_tangent_half()))
    blob.add("TANTOANGLE", struct.pack("<2049I", *gen_tantoangle()))

    focal, centery, va2x, x2va, yslope, distscale = gen_projection(
        args.width, args.height, args.fov)

    blob.add("VIEWANGLETOX", struct.pack("<%dh" % len(va2x), *va2x))
    blob.add("XTOVIEWANGLE", struct.pack("<%dI" % len(x2va), *x2va))
    blob.add("YSLOPE", struct.pack("<%di" % len(yslope), *yslope))
    blob.add("DISTSCALE", struct.pack("<%di" % len(distscale), *distscale))

    scalelight, zlight = gen_light_tables(args.width)
    blob.add("SCALELIGHT", scalelight)
    blob.add("ZLIGHT", zlight)

    # --- textures, columns deduplicated --------------------------------------
    col_pool = bytearray()
    col_map = {}
    tex_dir = bytearray()
    tex_cols = bytearray()
    masked_count = 0

    for name in tex_names:
        width, height, pixels, covered = textures.composite(name)

        if height % scale or width % scale:
            tex_scale = 1
        else:
            tex_scale = scale

        sw = width // tex_scale
        sh = height // tex_scale
        masked = 0 in covered
        masked_count += masked

        col_start = len(tex_cols) // 4

        for c in range(width):
            if c % tex_scale:
                continue

            column = pixels[c * height:(c + 1) * height]
            cover = covered[c * height:(c + 1) * height]
            column = shrink_column(column, tex_scale)
            cover = shrink_column(cover, tex_scale)

            if masked:
                # Post format: [top, length, pixels...] ..., 0xff terminator
                enc = bytearray()
                y = 0

                while y < sh:
                    if not cover[y]:
                        y += 1
                        continue

                    top = y

                    while y < sh and cover[y] and y - top < 254:
                        y += 1

                    enc.append(top)
                    enc.append(y - top)
                    enc += column[top:y]

                enc.append(0xFF)
                key = bytes(enc)
            else:
                key = column

            if key not in col_map:
                col_map[key] = len(col_pool)
                col_pool += key

            tex_cols += struct.pack("<I", col_map[key])

        # width/height are the LOGICAL size the renderer maps with; shift says
        # how much smaller the stored copy is, and is folded into the mapping
        # constants rather than the inner loop.
        # The mask is the largest power of two that fits in the stored width,
        # minus one - the same rule R_InitTextures uses, so textures whose width
        # is not a power of two wrap exactly where vanilla wraps them.
        shift = tex_scale.bit_length() - 1
        mask = 1

        while mask * 2 <= sw:
            mask <<= 1

        tex_dir += struct.pack("<HHBBHHHI", width, height, shift, masked,
                               sw, sh, mask - 1, col_start)

    blob.add("TEXDIR", struct.pack("<I", len(tex_names)) + bytes(tex_dir))
    blob.add("TEXCOLS", bytes(tex_cols))
    blob.add("TEXDATA", bytes(col_pool))

    # --- flats ---------------------------------------------------------------
    flat_data = bytearray()

    for name in flat_names:
        raw = wad.get(name)

        if len(raw) < 4096:
            raw = raw + b"\0" * (4096 - len(raw))

        flat_data += shrink_flat(raw[:4096], scale)

    blob.add("FLATS", bytes(flat_data))

    # --- level ---------------------------------------------------------------
    sides, sectors, _, _ = convert_level(lumps, tex_index, flat_index)

    blob.add("VERTEXES", lumps["VERTEXES"])
    blob.add("SEGS", lumps["SEGS"])
    blob.add("SSECTORS", lumps["SSECTORS"])
    blob.add("NODES", lumps["NODES"])
    blob.add("SIDEDEFS", sides)
    blob.add("LINEDEFS", lumps["LINEDEFS"])
    blob.add("SECTORS", sectors)
    blob.add("BLOCKMAP", lumps["BLOCKMAP"])
    blob.add("THINGS", lumps["THINGS"])

    # Mirrored by level_info_t in doom/doom_data.h - keep the two in step
    info = struct.pack(
        "<8s"       # map name
        "HH"        # flat downscale shift, sky texture index
        "HHHH"      # vertexes, segs, subsectors, nodes
        "HHHH"      # sidedefs, linedefs, sectors, things
        "HH"        # flats, textures
        "hhhh"      # width, height, centerx, centery
        "i",        # focal length
        args.map.encode("ascii")[:8].ljust(8, b"\0"),
        scale.bit_length() - 1,
        tex_index.get(sky, 0),
        len(lumps["VERTEXES"]) // 4,
        len(lumps["SEGS"]) // 12,
        len(lumps["SSECTORS"]) // 4,
        len(lumps["NODES"]) // 28,
        len(sides) // 12,
        len(lumps["LINEDEFS"]) // 14,
        len(sectors) // 14,
        len(lumps["THINGS"]) // 10,
        len(flat_names),
        len(tex_names),
        args.width, args.height, args.width // 2, centery,
        focal)

    blob.add("LEVELINFO", info)

    data = blob.build()

    out_dir = os.path.dirname(os.path.abspath(args.output))

    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    with open(args.output, "wb") as f:
        f.write(data)

    print("\nmap %s: %d segs, %d sectors, %d sidedefs, %d linedefs" % (
        args.map, len(lumps["SEGS"]) // 12, len(sectors) // 14,
        len(sides) // 12, len(lumps["LINEDEFS"]) // 14))
    print("textures %d (%d masked), flats %d, downscale %dx" % (
        len(tex_names), masked_count, len(flat_names), scale))
    print()
    blob.report()
    print("\n%s: %d bytes (%.1f KB)" % (args.output, len(data), len(data) / 1024.0))

    if args.header:
        with open(args.header, "w") as f:
            f.write("// Generated by tools/wadpack.py - do not edit\n")
            f.write("#define DOOM_ASSETS_SIZE %d\n" % len(data))
            f.write("#define DOOM_ASSETS_MAP \"%s\"\n" % args.map)


if __name__ == "__main__":
    sys.exit(main())
