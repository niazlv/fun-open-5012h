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

.PHONY: all test host coredump doomhost clean

all: test

# The two that run without an asset pack
test: host coredump

host: $(BUILD)/host_test
	@./$(BUILD)/host_test

coredump:
	@sh run_coredump_test.sh

$(BUILD)/host_test: $(HOST_SRCS) | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) $(HOST_SRCS) $(LIBS) -o $@

# Renders one DOOM frame to a .ppm. Not part of `test`: it needs the asset
# pack, which is built from a real IWAD and is not in the repository.
doomhost: $(BUILD)/doom_host
$(BUILD)/doom_host: doom_host.c $(wildcard $(ROOT)/doom/*.c) | $(BUILD)
	@echo CC $@
	@$(CC) $(CFLAGS) -I$(ROOT)/doom doom_host.c $(ROOT)/doom/*.c $(LIBS) -o $@

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	@rm -rf $(BUILD)
