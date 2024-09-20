#!/bin/bash

echo 'Uploading...'

objcopy -O ihex -R .eeprom ./build/ArduCopter ./ArduCopter.ihex

# -D flag stops the flash from going through the erase step

avrdude -p atmega2560 -c wiring -P/dev/ttyACM0 -b115200 -U flash:w:ArduCopter.ihex:i

