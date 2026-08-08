# Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
# Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
##############################################################################
BUILD = build
BIN = open-5012h

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

ifeq ($(OS), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif

CFLAGS += -W -Wall --std=gnu11 -O3
CFLAGS += -g3
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m4 -mthumb
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d
#CFLAGS += -save-temps

LDFLAGS += -mcpu=cortex-m4 -mthumb
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,-Map=$(BUILD)/$(BIN).map
LDFLAGS += -Wl,--script=$(LDSCRIPT)

# Where the settings store lives. `spi` puts it on the 8 MB serial part at
# 0x7A0000 and hands the whole 512 KB of internal flash back to code; the
# firmware migrates the existing store across by itself on the first boot.
# `internal` is the old arrangement - the last 128 KB sector - and costs a
# third of the image budget. See the comment at the top of config.c.
CONFIG_STORE ?= spi

ifeq ($(CONFIG_STORE),internal)
  CFLAGS += -DCONFIG_STORE_INTERNAL=1
  STORE_RESERVED = 131072
else ifeq ($(CONFIG_STORE),spi)
  STORE_RESERVED = 0
else
  $(error CONFIG_STORE must be spi or internal, not "$(CONFIG_STORE)")
endif

LDFLAGS += -Wl,--defsym=__store_reserved=$(STORE_RESERVED)

# How much of a BK-0010-01 to build. The defaults are in src/bk/bk_config.h and
# describe the real machine; this is where a smaller one is asked for:
#
#   make BK_CFLAGS="-DBK_DISASM=0 -DBK_DEBUGGER=0 -DBK_VIDEO_PALETTES=0"
#
# Nothing else in the image is affected - the switches only reach src/bk and
# src/apps/bk_port.c, all of which include bk_config.h before anything else.
BK_CFLAGS ?=

CFLAGS += $(BK_CFLAGS)

# One -I per subsystem, so a source keeps including a header by its bare name
# ("measure.h", not "../dsp/measure.h"). The directories are an index for the
# reader; they are deliberately not part of the include graph, so a file can be
# regrouped later without touching every #include that names it.
#
# -I.. stays for "doom/doom.h", which names its directory on purpose: that tree
# is vendored, and the path is what marks it as coming from outside.
INCLUDES += \
  -I../include \
  -I../doom \
  -I.. \
  -I../src/core \
  -I../src/hal \
  -I../src/acq \
  -I../src/dsp \
  -I../src/decode \
  -I../src/ui \
  -I../src/scope \
  -I../src/apps \
  -I../src/bk \
  -I../src/debug \

# Objects land flat in $(BUILD), so every basename here must be unique across
# all directories - two "utils.c" would quietly share one utils.o.
SRCS += \
  ../src/core/main.c \
  ../src/core/startup_gd32f4.c \
  ../src/core/syscalls.c \
  ../src/core/config.c \
  ../src/core/timer.c \
  ../src/core/utils.c \
  ../src/core/tcm_borrow.c \

# Parts that touch the hardware directly
SRCS += \
  ../src/hal/lcd.c \
  ../src/hal/flash.c \
  ../src/hal/spifs.c \
  ../src/hal/buttons.c \
  ../src/hal/battery.c \
  ../src/hal/input.c \

# Acquisition path: samples in
SRCS += \
  ../src/acq/capture.c \
  ../src/acq/trigger.c \
  ../src/acq/buffer.c \

# What the samples are turned into
SRCS += \
  ../src/dsp/measure.c \
  ../src/dsp/fft.c \
  ../src/dsp/alias.c \
  ../src/dsp/classify.c \
  ../src/dsp/trend.c \
  ../src/dsp/record_window.c \

# Protocol decoders. logic_decode.c is the dispatcher the rest hang off.
SRCS += \
  ../src/decode/logic_decode.c \
  ../src/decode/uart_decode.c \
  ../src/decode/onewire_decode.c \
  ../src/decode/ws2812_decode.c \
  ../src/decode/nec_decode.c \
  ../src/decode/servo_decode.c \
  ../src/decode/can_decode.c \
  ../src/decode/dht_decode.c \
  ../src/decode/sent_decode.c \
  ../src/decode/midi_decode.c \
  ../src/decode/lin_decode.c \
  ../src/decode/ev1527_decode.c \
  ../src/decode/dshot_decode.c \
  ../src/decode/spi_decode.c \
  ../src/decode/manchester_decode.c \
  ../src/decode/rc5_decode.c \
  ../src/decode/dali_decode.c \
  ../src/decode/knx_decode.c \
  ../src/decode/swo_decode.c \
  ../src/decode/swd_decode.c \
  ../src/decode/usb_decode.c \
  ../src/decode/pd_decode.c \
  ../src/decode/sirc_decode.c \
  ../src/decode/ppm_decode.c \

# Widgets and screen furniture, with no scope logic of their own
SRCS += \
  ../src/ui/ui.c \
  ../src/ui/menu_widget.c \
  ../src/ui/launcher.c \
  ../src/ui/system_menu.c \
  ../src/ui/key_remap.c \
  ../src/ui/flash_viewer.c \
  ../src/ui/fonts.c \
  ../src/ui/images.c \

# The instrument itself
SRCS += \
  ../src/scope/scope.c \
  ../src/scope/scope_chrome.c \
  ../src/scope/scope_menu.c \

# Everything the launcher can start that is not the scope
SRCS += \
  ../src/apps/cube3d.c \
  ../src/apps/engine3d.c \
  ../src/apps/raytrace_test.c \
  ../src/apps/flappy_bird.c \
  ../src/apps/game_gfx.c \
  ../src/apps/game2048.c \
  ../src/apps/tetris.c \
  ../src/apps/snake_game.c \
  ../src/apps/snake_assets.c \
  ../src/apps/spi_tool.c \
  ../src/apps/bk_port.c \

# Электроника БК-0010-01: a PDP-11 and the five registers around it. Every
# part of it is switchable from src/bk/bk_config.h - build a smaller one with
# BK_CFLAGS="-DBK_DISASM=0 -DBK_DEBUGGER=0", and see the table in the README
# for what each switch is worth.
SRCS += \
  ../src/bk/bk_cpu.c \
  ../src/bk/bk_mem.c \
  ../src/bk/bk_io.c \
  ../src/bk/bk_video.c \
  ../src/bk/bk_load.c \
  ../src/bk/bk_disasm.c \

# Size, not speed, for the 2D games and the toolkit they share.
#
# What those spend their time on is the panel: it is bit-banged, ~200 ns a
# pixel, and no amount of unrolling in the pixel arithmetic queued up behind it
# can shorten that. -O3 unrolls their 256-iteration tile loops into kilobytes
# and buys back time that was never on the critical path - measured, it is 10 KB
# of an image that is already 83% full.
#
# The 3D applications are on the list too, and that one IS a trade rather than
# a free win: cube3d, engine3d and raytrace_test are compute bound, and -Os
# costs them frame rate for the 7.5 KB it gives back. The flash ceiling won the
# argument; take them off the list if the headroom ever allows it.
#
# A target-specific variable, appended after the -O3 in CFLAGS, so the last -O
# on the command line is the one that counts.
# The BK emulator's own split is the same argument one level down: bk_cpu.c and
# bk_video.c are the inner loop and stay at -O3, while the disassembler, the
# loader and the application around them run once per keystroke and would
# rather be small.
#
# The interface is on the list for the same reason, and it is the biggest single
# win in the image: 12.8 KB, measured, from the eight files below. Every one of
# them runs when a KEY IS PRESSED - a menu is laid out and drawn, a settings
# entry is read or written, a fault dump is formatted - so a hundred cycles
# either way is invisible next to the LCD bus it is waiting for and next to the
# human who pressed the key. What is not on the list is anything the trace sweep
# touches: scope.c and capture.c stay at -O3, because those DO run per column.
#
# It was 5.8 KB of headroom that made this necessary; it is the mechanism the
# next feature should reach for before anything is cut.
SIZE_OPTIMIZED = \
  $(BUILD)/scope_menu.o \
  $(BUILD)/menu_widget.o \
  $(BUILD)/system_menu.o \
  $(BUILD)/flash_viewer.o \
  $(BUILD)/key_remap.o \
  $(BUILD)/launcher.o \
  $(BUILD)/config.o \
  $(BUILD)/debug_coredump.o \
  $(BUILD)/game_gfx.o \
  $(BUILD)/snake_game.o \
  $(BUILD)/flappy_bird.o \
  $(BUILD)/game2048.o \
  $(BUILD)/tetris.o \
  $(BUILD)/bk_disasm.o \
  $(BUILD)/bk_load.o \
  $(BUILD)/bk_port.o \
  $(BUILD)/pd_decode.o \
  $(BUILD)/usb_decode.o \
  $(BUILD)/can_decode.o \
  $(BUILD)/knx_decode.o \
  $(BUILD)/dali_decode.o \
  $(BUILD)/lin_decode.o \
  $(BUILD)/dht_decode.o \
  $(BUILD)/ev1527_decode.o \
  $(BUILD)/dshot_decode.o \
  $(BUILD)/sent_decode.o \
  $(BUILD)/midi_decode.o \
  $(BUILD)/manchester_decode.o \
  $(BUILD)/nec_decode.o \
  $(BUILD)/onewire_decode.o \
  $(BUILD)/ppm_decode.o \
  $(BUILD)/rc5_decode.o \
  $(BUILD)/servo_decode.o \
  $(BUILD)/sirc_decode.o \
  $(BUILD)/spi_decode.o \
  $(BUILD)/swd_decode.o \
  $(BUILD)/swo_decode.o \
  $(BUILD)/uart_decode.o \
  $(BUILD)/ws2812_decode.o \
  $(BUILD)/logic_decode.o \
  $(BUILD)/cube3d.o \
  $(BUILD)/engine3d.o \
  $(BUILD)/raytrace_test.o \

$(SIZE_OPTIMIZED): CFLAGS += -Os

SRCS += \
  ../src/debug/debug_coredump.c \

# Whether DOOM is in the image at all. DOOM (below) picks which asset pack is
# linked; this one decides whether the application, the engine and the
# launcher entry are built in the first place.
#
# It is for images that are handed to other people. The pack is made from a
# real IWAD by tools/wadpack.py and cannot be published, so a public build
# would carry an application whose only function is to say the data is
# missing - and pay flash for it. The CI build passes DOOM_APP=0; a local
# build is a build for someone who can make the pack, so the default is on.
DOOM_APP ?= 1

# Vendored: the DOOM engine, under its own COPYING
ifeq ($(DOOM_APP),1)
SRCS += \
  ../src/apps/doom_port.c \
  ../doom/w_assets.c \
  ../doom/w_stream.c \
  ../doom/r_main.c \
  ../doom/r_bsp.c \
  ../doom/r_segs.c \
  ../doom/r_plane.c \
  ../doom/r_draw.c \
  ../doom/p_move.c \
  ../doom/p_player.c \
  ../doom/p_doors.c \

endif

# Get build information
BUILD_DATE := $(shell date '+%Y-%m-%d')
BUILD_TIME := $(shell date '+%H:%M:%S')
GIT_HASH := $(shell git rev-parse --short HEAD 2>/dev/null || echo "unknown")
GIT_BRANCH := $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
# Wrapped in single quotes in DEFINES below, not backslash-escaped like the
# rest: this one is a person's name and usually has a space in it. Escaped
# quotes leave the space bare on the compiler's command line, the shell splits
# there, and the build dies on "linker input file not found: Leushkin" - which
# names no source and points at nothing. Every contributor whose git identity
# is two words hit that; the CI build, whose identity is "GitHub Actions", hit
# it first.
GIT_USER := $(shell git config user.name 2>/dev/null || echo "Unknown Developer")
BUILD_USER := $(shell whoami)
SHA256 := $(shell command -v sha256sum >/dev/null 2>&1 && echo sha256sum || echo "shasum -a 256")

DEFINES += \
  -D__GD32F407VE__ \
  -DF_CPU=250000000 \
  -DBUILD_DATE=\"$(BUILD_DATE)\" \
  -DBUILD_TIME=\"$(BUILD_TIME)\" \
  -DGIT_HASH=\"$(GIT_HASH)\" \
  -DGIT_BRANCH='"$(GIT_BRANCH)"' \
  -DGIT_USER='"$(GIT_USER)"' \
  -DBUILD_USER=\"$(BUILD_USER)\" \
  -DFIRMWARE_NAME=\"github.com/niazlv/fun-open-5012H\" \
  -DFIRMWARE_VERSION=\"2.3.0\" \
  -DDOOM_APP=$(DOOM_APP) \

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

# The DOOM asset pack is data, not code: an .S with .incbin drops it straight
# into .rodata. It is built from a real IWAD by tools/wadpack.py.
#
# It is also 208 KB of the 384 KB the firmware may use - more than half the
# image for one application, and what took the build to 99% full. So it is
# opt-in: `make DOOM=1` links the real pack, anything else links an empty
# stand-in and the application says how to build one. Nothing else about DOOM
# changes; the engine is a few KB of code and stays in either way.
# The pack used to be opt-in because it was 208 KB of the 384 the firmware
# had. It is not opt-in any more: wadpack.py --split leaves the texture columns
# and the flats in a second file that lives on the SPI part, and what is left
# to link is 110 KB - which fits, with room over.
DOOM ?= 1

ifeq ($(DOOM),1)
  DOOM_ASSETS = ../doom/doom_assets.bin
else
  DOOM_ASSETS = $(BUILD)/doom_assets_empty.bin
endif

ifeq ($(DOOM_APP),1)
OBJS += $(BUILD)/doom_assets.o
endif

# Which pack was linked is not recorded in any file's timestamp, so switching
# between them would otherwise leave the previous object in place and the image
# unchanged. The stamp remembers the setting and the stale artifacts are dropped
# when it changes. DOOM_APP rides along in the same stamp: it decides the same
# question one level up, and it also reaches launcher.o, which is the one
# object outside doom/ that compiles differently for it.
#
# At PARSE time, deliberately: make decides what is out of date before it runs
# any recipe, so a stamp that only gets rewritten during the build is a decision
# that arrives too late - make 3.81 does not reconsider, and the switch silently
# did nothing. By the time the rules below are read, the files are already gone.
DOOM_STAMP = $(BUILD)/doom_assets.stamp

# The .bin and .hex go too, not just the .elf: they are what gets flashed, and
# make 3.81 compares timestamps to the second - two builds a moment apart look
# simultaneous to it, and it would leave yesterday's image next to today's elf.
$(shell $(MKDIR) -p $(BUILD); \
    echo "$(DOOM_ASSETS) app=$(DOOM_APP)" | cmp -s - $(DOOM_STAMP) || { \
      echo "$(DOOM_ASSETS) app=$(DOOM_APP)" > $(DOOM_STAMP); \
      rm -f $(BUILD)/doom_assets.o $(BUILD)/launcher.o $(BUILD)/$(BIN).elf \
            $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).hex; })

# Same trick for the store backend, and for a worse reason than DOOM's: that
# one only decides which objects are linked, while this one changes what
# config.c compiles to AND how much flash the image may occupy. Switching
# without a rebuild leaves objects from the other backend in place - a store
# that reads one place and writes another, which would look like settings that
# save and then come back wrong.
CONFIG_STORE_STAMP = $(BUILD)/config_store.stamp

$(shell $(MKDIR) -p $(BUILD); \
    echo "$(CONFIG_STORE)" | cmp -s - $(CONFIG_STORE_STAMP) || { \
      echo "$(CONFIG_STORE)" > $(CONFIG_STORE_STAMP); \
      rm -f $(BUILD)/*.o $(BUILD)/$(BIN).elf \
            $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).hex; })

# And again for the emulator's switches. Narrower than the two above: they
# only reach src/bk and bk_port.c, so only those objects are dropped - which
# matters, because turning one off and rebuilding is how their cost gets
# measured, and a full rebuild each time makes that a minute instead of two
# seconds.
BK_STAMP = $(BUILD)/bk_config.stamp

$(shell $(MKDIR) -p $(BUILD); \
    echo "$(BK_CFLAGS)" | cmp -s - $(BK_STAMP) || { \
      echo "$(BK_CFLAGS)" > $(BK_STAMP); \
      rm -f $(BUILD)/bk_*.o $(BUILD)/$(BIN).elf \
            $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).hex; })

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

LDSCRIPT = ../linker/gd32f407ve.ld

$(DOOM_ASSETS):
	@echo "NOTE no DOOM asset pack, linking an empty one"
	@echo "     build one with: python3 tools/wadpack.py <iwad> -o doom/doom_assets.bin"
	@touch $@

$(BUILD)/doom_assets.o: ../doom/doom_assets.S $(DOOM_ASSETS)
	@echo AS $@ $(if $(filter 1,$(DOOM)),,"(no DOOM assets, build with DOOM=1)")
	@$(CC) $(CFLAGS) -DDOOM_ASSETS_FILE='"$(DOOM_ASSETS)"' -c $< -o $@

$(BUILD)/$(BIN).elf: $(OBJS) $(LDSCRIPT)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -lm -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^
	@echo
	@echo "Firmware Hash:"
	@$(SHA256) $(BUILD)/$(BIN).bin | cut -d' ' -f1 | head -c8 | xargs printf "  SHA256: %s\n"
	@python3 -c "import zlib; data=open('$(BUILD)/$(BIN).bin','rb').read(); print('  CRC32:  {:08x}'.format(zlib.crc32(data) & 0xffffffff))" 2>/dev/null || echo "  CRC32:  (python3 not available)"

clean:
	@echo clean
	@-rm -rf $(BUILD)

prog:
	@edbg -b -c 3000 -t gd32f4xx -pv -f $(BUILD)/$(BIN).bin

# Verify only. The DOOM asset pack sits in the second half of the image and no
# code does, so a short write leaves a firmware that runs and reports "NO WAD
# DATA" - this catches that without reflashing.
verify:
	@edbg -b -c 3000 -t gd32f4xx -v -f $(BUILD)/$(BIN).bin

# Where things landed, for checking how much of the image has to reach the part
layout: $(BUILD)/$(BIN).elf
	@arm-none-eabi-nm $(BUILD)/$(BIN).elf | grep -E ' (T|t) doom_assets(_end)?$$' | sort
	@echo "image size: `wc -c < $(BUILD)/$(BIN).bin` bytes"

# *.o.d, not *.d: every dependency file this build writes is "<object>.o.d",
# and the narrower pattern is what keeps an iCloud conflict copy out.
#
# iCloud renames the loser of a conflict by inserting a space and a number
# before the extension, so "foo.o.d" comes back as "foo.o 2.d". The wide
# pattern matched that, make split the name on the space, and the first half
# was "build/foo.o" - a real file, so make read the object as a makefile and
# died on the first NUL byte. It did so at parse time, which meant `make clean`
# could not parse either: the tree could only be recovered by deleting build/
# by hand. "foo.o 2.d" does not end in .o.d, so it is simply not matched.
-include $(wildcard $(BUILD)/*.o.d)
