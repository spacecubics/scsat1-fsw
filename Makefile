# Conditional Compilation for features
CONFIGS := -DCONFIG_ENABLE_WDT_RESET
#CONFIGS += -DCONFIG_FPGA_PROGRAM_MODE
CONFIGS += -DCONFIG_FPGA_WATCHDOG_TIMEOUT=3

# Design Parameter
MODULE := trch-firmware
DEVICE := 16LF877
TARGET := PPK4

# Command variables
CC     := xc8-cc -mc90lib -mcpu=$(DEVICE)
AR     := xc8-ar r
IPECMD := ipecmd.sh
RM     := rm -rf

# File Path variables
HEXDIR := hex
PRGDAT := $(HEXDIR)/$(MODULE)

# Include dir and other C related flags
INCDIR       := -I src
PARSER_FLAGS := -Xclang -Wall -Xclang -Wextra -Xclang -Wno-unused-parameter

# Source and object files
SRCS := src/main.c src/fpga.c src/interrupt.c src/timer.c

-include src/ioboard/ioboard.mk
SRCS += $(addprefix src/ioboard/,$(IOBOARD_SRCS))
OBJS := $(SRCS:.c=.p1)

LIB_SRCS := src/ioboard.c \
            src/i2c-gpio.c \
            src/ina3221.c \
            src/spi.c \
	    src/tmp175.c \
	    src/usart.c
LIB_OBJS := $(LIB_SRCS:.c=.p1)

LIBDEVICE := src/libdevice.a

# Clean File
CF      = $(HEXDIR) src/*.p1 src/*.a src/*.d MPLABXLog.* log.*

# Quiets
ifndef V
QUIET_CC  = @echo '   ' CC $@;
QUIET_AR  = @echo '   ' AR $@;
QUIET_HEX = @echo '   ' HEX $@;
endif

.PHONY: all
all: build

.PHONY: build
build: $(PRGDAT).hex

# make sure $(LIBDEVICE) is the last
$(PRGDAT).hex: $(OBJS) $(LIBDEVICE)
	@mkdir -p $(HEXDIR)
	@echo '*' > $(HEXDIR)/.gitignore
	$(QUIET_HEX)$(CC) -o $(HEXDIR)/$(MODULE) $^

%.p1: %.c Makefile
	$(QUIET_CC)$(CC) $(PARSER_FLAGS) $(INCDIR) $(CONFIGS) -c -o $@ $<

$(LIBDEVICE): $(LIB_OBJS)
	$(QUIET_AR)$(AR) $@ $^

flash: program
.PHONY: program
program: $(PRGDAT).hex
	$(IPECMD) -P$(DEVICE) -T$(TARGET) -F$< -M -OL

.PHONY: erase
erase:
	$(IPECMD) -P$(DEVICE) -T$(TARGET) -E

.PHONY: reset
reset:
	$(IPECMD) -P$(DEVICE) -T$(TARGET) -OK -OL

.PHONY: halt
halt:
	$(IPECMD) -P$(DEVICE) -T$(TARGET) -OK

.PHONY: clean
clean:
	$(RM) $(CF)
