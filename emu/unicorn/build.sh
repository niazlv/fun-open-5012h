#!/bin/sh
#
# Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
#
# Builds Unicorn with no-per-block-jit-toggle.patch applied, for hosts where
# that pays: on Apple silicon it is worth about 1.25x, because the stock build
# flips JIT write protection twice per translation block. See the patch for
# what it does and why it is safe.
#
# The result lands in build/, and ../Makefile picks it up on its own. Nothing
# else changes: delete this directory's build/ and the emulator goes back to
# whatever unicorn the system provides.
#
# usage: ./build.sh [version-tag]

set -e

cd "$(dirname "$0")"

VER="${1:-2.1.4}"
SRC="src-$VER"
OUT="build"

for tool in git cmake; do
  command -v "$tool" >/dev/null 2>&1 || {
    echo "need $tool to build unicorn from source" >&2
    exit 1
  }
done

if [ ! -d "$SRC" ]; then
  echo "fetching unicorn $VER"
  git clone --depth 1 --branch "$VER" \
      https://github.com/unicorn-engine/unicorn.git "$SRC"
fi

# Re-applying onto a clean tree, so that a second run is not a second patch
( cd "$SRC" && git checkout -- qemu/accel/tcg/cpu-exec.c )
patch -p1 -d "$SRC" -i ../no-per-block-jit-toggle.patch

# ARM only: this emulator runs one Cortex-M4 and the other targets are most of
# the build time.
cmake -S "$SRC" -B "$SRC/cmake-build" \
      -DCMAKE_BUILD_TYPE=Release \
      -DUNICORN_ARCH="arm" \
      -DBUILD_SHARED_LIBS=ON \
      -DUNICORN_BUILD_TESTS=OFF
cmake --build "$SRC/cmake-build" -j "$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"

mkdir -p "$OUT/include"
cp -R "$SRC/include/unicorn" "$OUT/include/"
find "$SRC/cmake-build" -maxdepth 1 -name 'libunicorn*' -exec cp -P {} "$OUT/" \;

echo
echo "built into $(pwd)/$OUT - now: make -C .. clean && make -C .."
