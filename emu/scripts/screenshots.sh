#!/bin/sh
#
# Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
#
# Regenerates every screenshot in emu/screenshots/ from the current firmware.
#
# Runs headless, so it needs no display and no hardware: each shot is the
# framebuffer the emulated ST7789 holds after the firmware has drawn into it.
#
# usage: ./scripts/screenshots.sh [path/to/firmware.bin]

set -e

cd "$(dirname "$0")/.."

EMU=./build/fun5012h-emu
FW="${1:-../make/build/open-5012h.bin}"

if [ ! -x "$EMU" ]; then
  echo "build the emulator first: make" >&2
  exit 1
fi

if [ ! -f "$FW" ]; then
  echo "no firmware image at $FW - build it with: make -C ../make" >&2
  exit 1
fi

mkdir -p screenshots

shot() {
  script="$1"
  shift
  echo "  $script"
  $EMU "$FW" --headless --script "scripts/$script" "$@" 2>&1 |
      grep -E 'wrote|error|stopped' || true
}

echo "rendering screenshots from $FW"

shot 01-launcher.txt
shot 02-scope-sine.txt  --signal "sine f=10k vpp=2 noise=4m"
shot 03-square-duty.txt --signal "square f=25k vpp=3 duty=30 noise=8m"
shot 04-fft.txt         --signal "square f=200k vpp=2"
shot 05-uart.txt         --signal "uart baud=115200 vpp=3.3 text=Hiмир!"
shot 06-glitch.txt      --signal "glitch f=200k vpp=2 every=32 width=80n"

# The same 2 MHz square through a perfect front end and through this one, so
# the difference bandwidth, overshoot, jitter and converter noise make is
# something you can look at side by side
shot 07-afe.txt --signal "square f=2M vpp=2 rise=0" --afe ideal
mv screenshots/07-afe.png screenshots/07-frontend-ideal.png 2>/dev/null || true
shot 07-afe.txt --signal "square f=2M vpp=2 rise=0"
mv screenshots/07-afe.png screenshots/08-frontend-real.png 2>/dev/null || true
shot 07-afe.txt --signal "square f=2M vpp=2 rise=0" --afe "bw=15M overshoot=15"
mv screenshots/07-afe.png screenshots/09-frontend-soft.png 2>/dev/null || true

echo "done:"
ls -1 screenshots/
