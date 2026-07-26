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

INCLUDES += \
  -I../include \
  -I.. \

SRCS += \
  ../main.c \
  ../lcd.c \
  ../flash.c \
  ../timer.c \
  ../config.c \
  ../buttons.c \
  ../battery.c \
  ../capture.c \
  ../trigger.c \
  ../buffer.c \
  ../measure.c \
  ../fft.c \
  ../classify.c \
  ../logic_decode.c \
  ../uart_decode.c \
  ../onewire_decode.c \
  ../ws2812_decode.c \
  ../nec_decode.c \
  ../cube3d.c \
  ../engine3d.c \
  ../ui.c \
  ../menu_widget.c \
  ../input.c \
  ../launcher.c \
  ../system_menu.c \
  ../raytrace_test.c \
  ../flappy_bird.c \
  ../snake_game.c \
  ../doom_port.c \
  ../scope.c \
  ../syscalls.c \
  ../utils.c \
  ../fonts.c \
  ../images.c \
  ../debug_coredump.c \
  ../flash_viewer.c \
  ../startup_gd32f4.c \

# Get build information
BUILD_DATE := $(shell date '+%Y-%m-%d')
BUILD_TIME := $(shell date '+%H:%M:%S')
GIT_HASH := $(shell git rev-parse --short HEAD 2>/dev/null || echo "unknown")
GIT_BRANCH := $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
GIT_USER := $(shell git config user.name 2>/dev/null || echo "Unknown Developer")
BUILD_USER := $(shell whoami)
SHA256 := $(shell command -v sha256sum >/dev/null 2>&1 && echo sha256sum || echo "shasum -a 256")

DEFINES += \
  -D__GD32F407VE__ \
  -DF_CPU=250000000 \
  -DBUILD_DATE=\"$(BUILD_DATE)\" \
  -DBUILD_TIME=\"$(BUILD_TIME)\" \
  -DGIT_HASH=\"$(GIT_HASH)\" \
  -DGIT_BRANCH=\"$(GIT_BRANCH)\" \
  -DGIT_USER=\"$(GIT_USER)\" \
  -DBUILD_USER=\"$(BUILD_USER)\" \
  -DFIRMWARE_NAME=\"Open-5012H\" \
  -DFIRMWARE_VERSION=\"1.0.0\" \

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

LDSCRIPT = ../linker/gd32f407ve.ld

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

-include $(wildcard $(BUILD)/*.d)
