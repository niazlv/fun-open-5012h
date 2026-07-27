#!/bin/sh
#
# Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
#
# Build and run the coredump ring tests on the host.
#
# debug_coredump.c is copied, unmodified, into a staging directory next to the
# stub device headers. That is what makes the stubs win: a quoted #include is
# resolved against the directory of the file doing the including, so the copy
# picks up tests/hoststub/lcd.h rather than the real one, while everything the
# stubs do not shadow (debug_coredump.h, menu_widget.h, buttons.h, ui.h) still
# comes from the repository via -I.
#
# Usage, from the repository root:  sh tests/run_coredump_test.sh

set -e

root=$(cd "$(dirname "$0")/.." && pwd)
stage=${TMPDIR:-/tmp}/coredump_test_stage

rm -rf "$stage"
mkdir -p "$stage"

cp "$root/debug_coredump.c" "$stage/"
cp "$root"/tests/hoststub/*.h "$stage/"

# The pointer/integer cast warnings are about 32-bit device addresses being
# handled on a 64-bit host; on the target they are exact.
cc -O1 -W -Wall -Wno-unused-function \
   -Wno-pointer-to-int-cast -Wno-int-to-pointer-cast \
   -I"$stage" -I"$root" \
   "$root/tests/coredump_test.c" -o "$stage/coredump_test"

"$stage/coredump_test"
