#!/bin/bash

echo 'Uploading...'

#objcopy -O ihex -R .eeprom ./build-avr/bin/mincopter ./mincopter.ihex
objcopy -O ihex -R .eeprom $1 $2

# -D flag stops the flash from going through the erase step

avrdude -p atmega2560 -c wiring -P/dev/ttyACM0 -b115200 -U flash:w:$2:i


