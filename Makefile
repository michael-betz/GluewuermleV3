TARGET      = glueh
USB_SERIAL  = /dev/ttyUSB0
MCU         = atmega328p
F_CPU       = 8000000UL
SRCS        = $(wildcard *.c) $(wildcard myUtil/*.c)
OBJS        = $(subst .c,.o,$(SRCS)) lfsr32.o pulseCode.o
INC         = -I. -ImyUtil
GIT_VERSION = $(shell git describe --abbrev=4 --dirty --always --tags)
TOOLS_PREFIX = avr-
CC          = $(TOOLS_PREFIX)gcc
#--------------------------------
# Compiler flags
#--------------------------------
## Warnings, standards
CFLAGS      = -O3 -Wall -std=gnu11
## Debug prints on UART: 0 = None (fastest!) 1 = Some, 2 = A lot
CFLAGS     += -DDEBUG_LEVEL=0
## Hardware definitions
CFLAGS     += -DF_CPU=$(F_CPU) -mmcu=$(MCU) -DGIT_VERSION=\"$(GIT_VERSION)\"
## Use short (8-bit) data types
CFLAGS     += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
## Splits up object files per function
CFLAGS     += -ffunction-sections -fdata-sections
#--------------------------------
# Linker flags
#--------------------------------
LDFLAGS     = -Wl,-Map=$(TARGET).map,--cref
# Optional, but often ends up with smaller code
LDFLAGS    += -Wl,--gc-sections
## Relax shrinks code even more, but makes disassembly messy
LDFLAGS += -Wl,--relax
## for floating-point printf
# LDFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm
## for smaller printf
# LDFLAGS += -Wl,-u,vfprintf -lprintf_min

all: $(TARGET).hex $(TARGET).lst

flash: $(TARGET).hex
	avrdude -P $(USB_SERIAL) -b 38400 -c arduino -p atmega328p -U flash:w:$(TARGET).hex

%.o: %.S
	@echo -----------------------------------------------
	@echo  Assembling $@
	@echo -----------------------------------------------
	$(CC) $(INC) $(CFLAGS) -o $@ -c $<

%.o: %.c
	@echo -----------------------------------------------
	@echo  Compiling $@
	@echo -----------------------------------------------
	$(CC) $(INC) $(CFLAGS) -o $@ -c $<

%.lst: %.elf
	@echo -----------------------------------------------
	@echo  Generating assembly listing
	@echo -----------------------------------------------
	$(TOOLS_PREFIX)objdump -h -S -z $< > $@

%.hex: %.elf
	@echo -----------------------------------------------
	@echo  Generating intel hex file
	@echo -----------------------------------------------
	$(TOOLS_PREFIX)objcopy -O ihex -R .eeprom $< $@

$(TARGET).elf: $(OBJS)
	@echo -----------------------------------------------
	@echo  Linking together an application .elf file
	@echo -----------------------------------------------
	$(CC) -mmcu=$(MCU) $(LDFLAGS) -o $@ $^
	chmod -x $@
	$(TOOLS_PREFIX)size $@

clean:
	rm -f $(TARGET).hex $(TARGET).lst $(TARGET).elf $(TARGET).map $(OBJS)

.PHONY: clean all flash
