# Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
##############################################################################
# Host tests: no hardware, no ARM toolchain, the system compiler is enough.
#
# The sources under test are the ones that go on the device, unmodified. They
# qualify because they are arithmetic over a sample buffer, or a state machine
# over key events, and touch no register - the parts that do are not built
# here, and what little they call into is stubbed at the top of host_test.c.
#
# These lists used to live in a comment at the top of host_test.c, which meant
# nothing checked them and moving a file broke them silently. They are here so
# that `make test` is the only copy.
##############################################################################
BUILD = build
ROOT  = ..

CC ?= cc

CFLAGS += -O2 -W -Wall -std=gnu11
LIBS   += -lm

# The same per-subsystem include path the firmware build uses, so a source
# keeps including its headers by bare name. The vendor device headers under
# ../include are deliberately absent: nothing built here may reach a register.
CFLAGS += \
  -I$(ROOT)/src/core \
  -I$(ROOT)/src/hal \
  -I$(ROOT)/src/acq \
  -I$(ROOT)/src/dsp \
  -I$(ROOT)/src/decode \
  -I$(ROOT)/src/ui \
  -I$(ROOT)/src/scope \
  -I$(ROOT)/src/debug \

# Every decoder, by wildcard: a new one is then covered the day it is added,
# rather than the day someone remembers this list exists.
HOST_SRCS = \
  host_test.c \
  $(ROOT)/src/hal/input.c \
  $(ROOT)/src/dsp/measure.c \
  $(ROOT)/src/dsp/fft.c \
  $(ROOT)/src/dsp/classify.c \
  $(ROOT)/src/dsp/record_window.c \
  $(ROOT)/src/dsp/trend.c \
  $(wildcard $(ROOT)/src/decode/*.c) \

.PHONY: all test host coredump snake flappy g2048 tetris bk bkhost doomhost doomcache clean

all: test

# The ones that run without an asset pack
test: host coredump snake flappy g2048 tetris bk

host: $(BUILD)/host_test
	@./$(BUILD)/host_test

coredump:
	@sh run_coredump_test.sh

$(BUILD)/host_test: $(HOST_SRCS) | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) $(HOST_SRCS) $(LIBS) -o $@

# Snake. It needs two things the list above deliberately withholds: the game's
# own directory, and src/ui for the real fonts - the artwork is drawn here for
# real, into a framebuffer, so that a blit off the edge of the panel fails a
# test instead of corrupting a display.
SNAKE_SRCS = \
  snake_test.c \
  $(ROOT)/src/apps/game_gfx.c \
  $(ROOT)/src/apps/snake_assets.c \
  $(ROOT)/src/ui/fonts.c \

snake: $(BUILD)/snake_test
	@./$(BUILD)/snake_test

$(BUILD)/snake_test: $(SNAKE_SRCS) $(ROOT)/src/apps/snake_game.c gamestub.h | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -I$(ROOT)/src/apps -I$(ROOT)/tests/hoststub $(SNAKE_SRCS) $(LIBS) -o $@

FLAPPY_SRCS = \
  flappy_test.c \
  $(ROOT)/src/apps/game_gfx.c \
  $(ROOT)/src/ui/fonts.c \

flappy: $(BUILD)/flappy_test
	@./$(BUILD)/flappy_test

$(BUILD)/flappy_test: $(FLAPPY_SRCS) $(ROOT)/src/apps/flappy_bird.c gamestub.h | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -I$(ROOT)/src/apps -I$(ROOT)/tests/hoststub $(FLAPPY_SRCS) $(LIBS) -o $@

G2048_SRCS = \
  g2048_test.c \
  $(ROOT)/src/apps/game_gfx.c \
  $(ROOT)/src/ui/fonts.c \

g2048: $(BUILD)/g2048_test
	@./$(BUILD)/g2048_test

$(BUILD)/g2048_test: $(G2048_SRCS) $(ROOT)/src/apps/game2048.c gamestub.h | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -I$(ROOT)/src/apps -I$(ROOT)/tests/hoststub $(G2048_SRCS) $(LIBS) -o $@

TETRIS_SRCS = \
  tetris_test.c \
  $(ROOT)/src/apps/game_gfx.c \
  $(ROOT)/src/ui/fonts.c \

tetris: $(BUILD)/tetris_test
	@./$(BUILD)/tetris_test

$(BUILD)/tetris_test: $(TETRIS_SRCS) $(ROOT)/src/apps/tetris.c gamestub.h | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -I$(ROOT)/src/apps -I$(ROOT)/tests/hoststub $(TETRIS_SRCS) $(LIBS) -o $@

# The БК-0010-01 emulator. Everything under src/bk is here, unmodified and
# linked rather than #included, because none of it touches a register - the
# processor and the bus are arithmetic over a 64 KB array, and the one file
# that draws goes through the same lcd stub the games use.
#
# BK_LOAD_SPIFS=0 leaves out the half of bk_load.c that reads the flash, which
# is the only part of the emulator that could not run on a host at all.
BK_SRCS = \
  bk_cpu_test.c \
  $(ROOT)/src/bk/bk_cpu.c \
  $(ROOT)/src/bk/bk_mem.c \
  $(ROOT)/src/bk/bk_io.c \
  $(ROOT)/src/bk/bk_video.c \
  $(ROOT)/src/bk/bk_load.c \
  $(ROOT)/src/bk/bk_disasm.c \

bk: $(BUILD)/bk_cpu_test
	@./$(BUILD)/bk_cpu_test

$(BUILD)/bk_cpu_test: $(BK_SRCS) $(wildcard $(ROOT)/src/bk/*.h) | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -DBK_LOAD_SPIFS=0 -I$(ROOT)/src/bk $(BK_SRCS) $(LIBS) -o $@

# Runs a real BK tape image. Not part of `test`: it needs one, and those are
# somebody's software.
bkhost: $(BUILD)/bk_host
$(BUILD)/bk_host: bk_host.c $(filter-out bk_cpu_test.c,$(BK_SRCS)) | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -DBK_LOAD_SPIFS=0 -I$(ROOT)/src/bk \
	    bk_host.c $(filter-out bk_cpu_test.c,$(BK_SRCS)) $(LIBS) -o $@

# Renders one DOOM frame to a .ppm. Not part of `test`: it needs the asset
# pack, which is built from a real IWAD and is not in the repository.
doomcache: $(BUILD)/doom_cache
$(BUILD)/doom_cache: doom_cache.c $(wildcard $(ROOT)/doom/*.c) | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -DDOOM_TRACE -I$(ROOT)/doom doom_cache.c $(ROOT)/doom/*.c $(LIBS) -o $@

doomhost: $(BUILD)/doom_host
$(BUILD)/doom_host: doom_host.c $(wildcard $(ROOT)/doom/*.c) | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -I$(ROOT)/doom doom_host.c $(ROOT)/doom/*.c $(LIBS) -o $@

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	@rm -rf $(BUILD)
