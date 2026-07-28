# Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
##############################################################################
# A forwarder, so the usual targets work from the repository root.
#
# The real build files stay where they are - make/Makefile builds the firmware
# with arm-none-eabi-gcc, emu/Makefile builds the host emulator, tests/Makefile
# builds the host tests. Each still works when run directly from its own
# directory; this only saves the -C.
#
# Variables set on the command line reach the sub-make on their own, through
# MAKEFLAGS, so `make DOOM=1` from here does what it does from make/.
##############################################################################
.PHONY: all clean prog verify size layout emu test shots help

all size layout prog verify:
	@$(MAKE) -C make $@

# Build the emulator, and the firmware image it runs
emu:
	@$(MAKE) -C emu

# Host tests: no hardware, no ARM toolchain
test:
	@$(MAKE) -C tests

# Emulator screenshots
shots:
	@$(MAKE) -C emu shots

# clean is the one target that has to reach every build directory
clean:
	@$(MAKE) -C make clean
	@$(MAKE) -C emu clean
	@$(MAKE) -C tests clean

help:
	@echo "make            firmware (make DOOM=1 to link the DOOM asset pack)"
	@echo "make test       host tests for the DSP and the decoders"
	@echo "make emu        host emulator"
	@echo "make shots      emulator screenshots"
	@echo "make prog       flash the board with edbg"
	@echo "make clean      all of the above"
