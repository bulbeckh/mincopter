#!/bin/bash

# Run gz
gz sim -v4 -r iris_runway.sdf > /dev/null 2>&1 &

# Run mincopter
#gdb ./build/mincopter -x ./testing/breaks.gdb

if [ -f "mincopter" ]; then
	./mincopter
else
	./build/mincopter
fi



