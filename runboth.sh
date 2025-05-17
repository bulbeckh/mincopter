#!/bin/bash

# Run gz
gz sim -v4 -r iris_runway.sdf > /dev/null 2>&1 &

# Run mincopter
#gdb ./build/mincopter -x ./testing/breaks.gdb

if [ "$1" == "gdb" ]; then

	if [ -f "mincopter" ]; then
		gdb ./mincopter  -x ../testing/breaks.gdb
	else
		./build/mincopter -x ./testing/breaks.gdb
	fi

else

	if [ -f "mincopter" ]; then
		./mincopter
	else
		./build/mincopter
	fi
fi


