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
# Unicorn provides the Cortex-M4 core.
#
# The build this project wants is its own, carrying
# unicorn/no-per-block-jit-toggle.patch: on Apple silicon the stock library
# flips JIT write protection twice per translation block, and not doing that is
# worth about 1.25x. So `make` builds it, once, without being asked - the first
# build fetches unicorn and takes a couple of minutes, every one after that
# sees the library already there and does nothing.
#
# UNICORN = auto   build our own where the patch pays, system library elsewhere
#           local  build our own regardless of platform
#           system never build; use whatever unicorn is installed
UNICORN ?= auto

UC_DIR   := unicorn/build
UC_STAMP := $(UC_DIR)/.built

UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

# The patch only has something to remove on Darwin/aarch64: everywhere else
# unicorn's tb_exec_change() is already an empty function, so building our own
# would cost a clone and a compile to arrive at the same speed.
UC_WORTH_IT := $(if $(filter Darwin,$(UNAME_S)),$(filter arm64,$(UNAME_M)))

# ...and it can only be built if there is something to build it with
UC_CAN_BUILD := $(if $(shell command -v git 2>/dev/null),$(shell command -v cmake 2>/dev/null))

ifeq ($(UNICORN),auto)
  UC_USE_LOCAL := $(if $(UC_WORTH_IT),$(UC_CAN_BUILD))
else ifeq ($(UNICORN),local)
  UC_USE_LOCAL := $(UC_CAN_BUILD)
else
  UC_USE_LOCAL :=
endif

ifneq ($(UC_USE_LOCAL),)
  # rpath through @loader_path/$ORIGIN rather than an absolute path, so that
  # moving or copying the tree does not leave the binary looking for a library
  # where it used to be
  ifeq ($(UNAME_S),Darwin)
    UC_RPATH := -Wl,-rpath,@loader_path/../$(UC_DIR)
  else
    UC_RPATH := -Wl,-rpath,'$$ORIGIN/../$(UC_DIR)'
  endif

  UNICORN_CFLAGS := -I$(UC_DIR)/include
  UNICORN_LIBS   := -L$(UC_DIR) -lunicorn $(UC_RPATH)
else
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

  # Worth saying only when the fast path was wanted and did not happen
  ifneq ($(UC_WORTH_IT),)
    ifeq ($(UNICORN),auto)
      $(info NOTE no git/cmake - using the system unicorn, which is ~1.25x slower here)
    endif
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
  src/panel.c \

OBJS = $(addprefix $(BUILD)/, $(notdir $(SRCS:.c=.o)))

.PHONY: all clean distclean run shots unicorn

all: $(BUILD)/$(BIN)

# Our own unicorn, when this platform is one where the patch pays. Order-only,
# because the objects need its headers but must not rebuild when it is merely
# newer than they are.
ifneq ($(UC_USE_LOCAL),)
$(OBJS): | $(UC_STAMP)

$(UC_STAMP):
	@./unicorn/build.sh
endif

# Force the local build even where `make` would not bother with it
unicorn:
	@./unicorn/build.sh

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

# Leaves unicorn alone: rebuilding it takes minutes and it does not change
clean:
	@rm -rf $(BUILD)

# ...and this is how you get rid of it too
distclean: clean
	@rm -rf $(UC_DIR) unicorn/src-*
