# ------------------------------------------------------------------
#  PulseOx firmware -- ATmega32A @ 16 MHz external crystal
# ------------------------------------------------------------------
MCU      = atmega32
F_CPU    = 16000000UL
TARGET   = pulseox

# Programmer (override on the command line, e.g. make flash PROG=usbasp)
PROG     = usbasp
PORT     = usb
BAUD     = 19200

SRC      = $(wildcard src/*.c)
OBJ      = $(SRC:.c=.o)
DEP      = $(SRC:.c=.d)

CC       = avr-gcc
OBJCOPY  = avr-objcopy
SIZE     = avr-size
AVRDUDE  = avrdude

# Kept in step with [common] build_flags in platformio.ini, which is the
# canonical set.  If you change one, change the other.
CFLAGS   = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu99 \
           -Wall -Wextra -Wundef -Wshadow -Wpointer-arith -Wcast-align \
           -Wstrict-prototypes -Wmissing-prototypes -Wredundant-decls \
           -Werror=implicit-function-declaration \
           -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
           -ffunction-sections -fdata-sections -flto -MMD -MP
LDFLAGS  = -mmcu=$(MCU) -Wl,--gc-sections -Wl,-Map,$(TARGET).map -flto -Os

all: $(TARGET).hex $(TARGET).eep size

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock $< $@

$(TARGET).eep: $(TARGET).elf
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	   --change-section-lma .eeprom=0 -O ihex $< $@

size: $(TARGET).elf
	@$(SIZE) --format=avr --mcu=$(MCU) $<
	@echo
	@echo "NOTE: avr-size measures against the bare 32 KB / 2 KB, so it does"
	@echo "NOT warn about growing over the urboot bootloader at the top of"
	@echo "flash, nor about leaving too little SRAM for the stack.  The"
	@echo "PlatformIO build enforces both -- see scripts/check_size.py."
	@echo "This Makefile builds the equivalent of env:probe (diagnostics and"
	@echo "the channel probe both on), which is the largest configuration."

# Host-side checks.  math_check.py needs only Python.  tests/host/test_ppg.c
# needs a host C compiler; see tests/host/run.ps1.
test:
	python tests/math_check.py

flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROG) -p m32 -P $(PORT) -b $(BAUD) -U flash:w:$<:i

# ---- urboot bootloader upload (MightyCore "Bootloader: Yes (UART0)") -------
# Flash only, so settings already saved in EEPROM survive the upload.
# Override SPORT to pick a different COM port:  make upload SPORT=COM7
SPORT      ?= COM3
MC_TOOLS   ?= $(LOCALAPPDATA)/Arduino15/packages/MightyCore/tools/avrdude/8.0-arduino.1
MC_AVRDUDE ?= $(MC_TOOLS)/bin/avrdude
MC_CONF    ?= $(MC_TOOLS)/etc/avrdude.conf

upload: $(TARGET).hex
	"$(MC_AVRDUDE)" "-C$(MC_CONF)" -patmega32 -curclock -P$(SPORT) -b115200 -D -xnometadata -U flash:w:$<:i

verify: $(TARGET).hex
	"$(MC_AVRDUDE)" "-C$(MC_CONF)" -patmega32 -curclock -P$(SPORT) -b115200 -xnometadata -U flash:v:$<:i

# 16 MHz crystal, CKOPT programmed (full swing), JTAG disabled,
# brown-out at 4.0 V.  Verify against your board before writing.
fuses:
	$(AVRDUDE) -c $(PROG) -p m32 -P $(PORT) -b $(BAUD) \
	   -U lfuse:w:0xFF:m -U hfuse:w:0xC9:m

clean:
	rm -f $(OBJ) $(DEP) $(TARGET).elf $(TARGET).hex $(TARGET).eep $(TARGET).map

.PHONY: all size test flash upload verify fuses clean
-include $(DEP)
