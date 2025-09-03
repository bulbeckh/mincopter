#!/bin/bash


## Runs both the executable with simavr and then the gdb instance

set -m

simavr -m atmega2560 -f 16000000 -v ../build-avr/bin/mincopter -g &
p1 = $!

avr-gdb ../build-avr/bin/mincopter -x loadconfig.gdb

kill $p1 2>/dev/null
wait $p1 2>/dev/null


