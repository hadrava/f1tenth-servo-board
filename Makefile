PROJECT = main

OBJECTS = main.o servo.o uart.o input_capture.o input_capture_asm.o speed_controller.o

CFLAGS  = -MMD -Wall -Os -finline-functions -std=gnu11
CFLAGS += -DF_CPU=16000000 -mmcu=atmega328p
AVRDUDEFLAGS  = -P /dev/ttyUSB0 -c arduino -p m328p -v


CC = avr-gcc
OBJCOPY = avr-objcopy
SIZE = avr-size
AVRDUDE = avrdude

ASM = $(CC)
ASFLAGS = -mmcu=atmega328p -I /usr/lib/avr/include/


HEX = $(PROJECT).hex
ELF = $(PROJECT).elf

DEPENDENCIES=$(OBJECTS:.o=.d)

all: $(HEX)

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
	$(SIZE) $@

%.o: %.S
	$(ASM) $(ASFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJECTS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

flash: $(HEX)
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:$<

restart:
	$(AVRDUDE) $(AVRDUDEFLAGS)

clean::
	rm -f $(HEX) $(ELF) $(OBJECTS) $(DEPENDENCIES)

-include $(DEPENDENCIES)

.PHONY: all flash restart clean
