# Conditional Compilation for features
CONFIG_WDT_RESET = -DCONFIG_ENABLE_WDT_RESET
CONFIG_FPGA_DO_CONFIGURE = -DCONFIG_FPGA_DO_CONFIGURE=1
CONFIG_FPGA_PROGRAM_MODE = -DCONFIG_FPGA_PROGRAM_MODE=0

DEFINES = ${CONFIG_WDT_RESET} ${CONFIG_FPGA_DO_CONFIGURE} ${CONFIG_FPGA_PROGRAM_MODE}

# Design Parameter
MODULE := trch-firmware
DEVICE := 16LF877
TARGET := PPK4

# Command variables
CC     := xc8-cc -mc90lib -mcpu=$(DEVICE)
IPECMD := ipecmd.sh
RM     := rm -rf

# File Path variables
HEXDIR := hex
PRGDAT := $(HEXDIR)/$(MODULE)

# Source and object files
SRCS := $(wildcard src/*.c)
OBJS := $(SRCS:.c=.p1)

# Clean File
CF      = $(HEXDIR) src/*.p1 src/*.d MPLABXLog.* log.*

.PHONY: all
all: build

.PHONY: build
build: $(PRGDAT).hex

$(PRGDAT).hex: $(OBJS)
	mkdir -p $(HEXDIR)
	echo '*' > $(HEXDIR)/.gitignore
	$(CC) -o $(HEXDIR)/$(MODULE) $^

%.p1: %.c Makefile
	$(CC) $(DEFINES) -c -o $@ $<

.PHONY: program
program: $(PRGDAT).hex
	$(IPECMD) -P$(DEVICE) -T$(TARGET) -F$< -M -OL

.PHONY: erase
erase:
	$(IPECMD) -P$(DEVICE) -T$(TARGET) -E

.PHONY: clean
clean:
	$(RM) $(CF)
