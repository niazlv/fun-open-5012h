##############################################################################
# fun-open-5012h hardware emulator
# Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
#
# Builds a host binary that runs the ARM firmware image unmodified. The
# firmware itself is NOT compiled here - `make -C ../make` does that, with
# arm-none-eabi-gcc, and this reads the .bin it produces.
##############################################################################
BUILD = build
BIN   = fun5012h-emu

CC ?= cc

# cmsis_shim must come first: it supplies the handful of qualifiers the
# vendor header wants from core_cm4.h, so that ../include/gd32f4xx.h - the
# same register description the firmware is built against - compiles here too.
CFLAGS += -std=gnu11 -O2 -g
CFLAGS += -Wall -Wextra -Wno-unused-parameter
CFLAGS += -Isrc/cmsis_shim -I../include -I..

LDFLAGS +=
LIBS += -lm -lz

##############################################################################
# Unicorn provides the Cortex-M4 core
UNICORN_CFLAGS := $(shell pkg-config --cflags unicorn 2>/dev/null)
UNICORN_LIBS   := $(shell pkg-config --libs unicorn 2>/dev/null)

ifeq ($(UNICORN_LIBS),)
  # Homebrew and the usual /usr/local layout, when pkg-config has no .pc
  ifneq ($(wildcard /opt/homebrew/lib/libunicorn.dylib),)
    UNICORN_CFLAGS := -I/opt/homebrew/include
    UNICORN_LIBS   := -L/opt/homebrew/lib -lunicorn
  else
    UNICORN_LIBS   := -lunicorn
  endif
endif

CFLAGS += $(UNICORN_CFLAGS)
LIBS   += $(UNICORN_LIBS)

##############################################################################
# SDL2 is optional: without it the emulator still runs headless and still
# writes screenshots, which is all the CI path needs.
SDL_CFLAGS := $(shell sdl2-config --cflags 2>/dev/null)
SDL_LIBS   := $(shell sdl2-config --libs 2>/dev/null)

ifeq ($(SDL_LIBS),)
  CFLAGS += -DEMU_NO_SDL
  $(info NOTE SDL2 not found - building headless only)
else
  CFLAGS += $(SDL_CFLAGS)
  LIBS   += $(SDL_LIBS)
endif

##############################################################################
SRCS = \
  src/main.c \
  src/cpu.c \
  src/board.c \
  src/st7789.c \
  src/siggen.c \
  src/video.c \

OBJS = $(addprefix $(BUILD)/, $(notdir $(SRCS:.c=.o)))

.PHONY: all clean run shots

all: $(BUILD)/$(BIN)

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.c | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(BIN): $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

# The firmware image the emulator runs, built by the ARM toolchain
FW = ../make/build/open-5012h.bin

$(FW):
	$(MAKE) -C ../make

run: all $(FW)
	./$(BUILD)/$(BIN) $(FW) --signal "sine f=1k vpp=1"

shots: all $(FW)
	./scripts/screenshots.sh

clean:
	@rm -rf $(BUILD)
