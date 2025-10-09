#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
  echo "Usage: $0 {arm|avr} {executable}"
  exit 1
fi

# Read the first argument
case "$1" in
  avr)
    echo "Uploading AVR..."
	objcopy -O ihex -R .eeprom $1 ./mincopter.ihex
	# -D flag stops the flash from going through the erase step
	avrdude -p atmega2560 -c wiring -P/dev/ttyACM0 -b115200 -U flash:w:./mincopter.ihex:i
    ;;
  arm)
    echo "Uploading ARM..."
	arm-none-eabi-objcopy -O binary $2 ./mincopter.bin
	st-flash --reset write ./mincopter.bin 0x08000000
    ;;
  *)
    echo "Invalid option: $1"
    echo "Usage: $0 {arm|avr} {executable}"
    exit 1
    ;;
esac

