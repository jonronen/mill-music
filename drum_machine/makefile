MCU ?= atmega168pb

ARDUINO_FILES ?= ~/Downloads/arduino-1.8.16/hardware
ARDUINO_SRCS = $(ARDUINO_FILES)/tools/avr/avr/include

ARDUINO_PORT ?= /dev/ttyACM0


GPP_FLAGS := -c -g -O3 -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=$(MCU) -DF_CPU=16000000L -DARDUINO=10807 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -I $(ARDUINO_SRCS) -I $(ARDUINO_FILES)/arduino/avr/cores/arduino -I $(ARDUINO_FILES)/arduino/avr/variants/standard


drum_machine.hex: drum_machine.elf
	$(ARDUINO_FILES)/tools/avr/bin/avr-objcopy -O ihex -R .eeprom drum_machine.elf drum_machine.hex

drum_machine.elf: drum_machine.o main.o wiring.o wiring_analog.o wiring_digital.o
	$(ARDUINO_FILES)/tools/avr/bin/avr-gcc -w -O3 -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=$(MCU) -o drum_machine.elf drum_machine.o main.o wiring.o wiring_analog.o wiring_digital.o -lm

main.o:
	$(ARDUINO_FILES)/tools/avr/bin/avr-gcc $(GPP_FLAGS) $(ARDUINO_FILES)/arduino/avr/cores/arduino/main.cpp

wiring.o:
	$(ARDUINO_FILES)/tools/avr/bin/avr-gcc $(GPP_FLAGS) $(ARDUINO_FILES)/arduino/avr/cores/arduino/wiring.c

wiring_analog.o:
	$(ARDUINO_FILES)/tools/avr/bin/avr-gcc $(GPP_FLAGS) $(ARDUINO_FILES)/arduino/avr/cores/arduino/wiring_analog.c

wiring_digital.o:
	$(ARDUINO_FILES)/tools/avr/bin/avr-gcc $(GPP_FLAGS) $(ARDUINO_FILES)/arduino/avr/cores/arduino/wiring_digital.c

drum_machine.o: drum_machine.c
	$(ARDUINO_FILES)/tools/avr/bin/avr-gcc $(GPP_FLAGS) drum_machine.c

upload: drum_machine.hex
	$(ARDUINO_FILES)/tools/avr/bin/avrdude -C $(ARDUINO_FILES)/tools/avr/etc/avrdude.conf -v -p$(MCU) -carduino -P$(ARDUINO_PORT) -b115200 -D -Uflash:w:drum_machine.hex:i

upload_no_bootloader: drum_machine.hex
	$(ARDUINO_FILES)/tools/avr/bin/avrdude -C $(ARDUINO_FILES)/tools/avr/etc/avrdude.conf -v -p$(MCU) -cstk500v1 -P$(ARDUINO_PORT) -b19200 -Uflash:w:drum_machine.hex:i

fuses:
	$(ARDUINO_FILES)/tools/avr/bin/avrdude -C $(ARDUINO_FILES)/tools/avr/etc/avrdude.conf -v -v -v -p$(MCU) -cstk500v1 -P$(ARDUINO_PORT) -b19200 -U lfuse:w:0xe7:m -U hfuse:w:0xdf:m -U efuse:w:0xfd:m


clean:
	rm -f *.o *.hex *.elf

