#!/bin/bash

## TODO Add check if ap-gz/models is in the current GZ_RESOURCE_PATH env variables

# Run gz
gz sim -v4 -r iris_runway.sdf > /dev/null 2>&1 &

# Run mincopter
#gdb ./build/mincopter -x ./docs/breaks.gdb

if [ "$1" == "gdb" ]; then

	if [ -f "bin/mincopter" ]; then
		gdb ./bin/mincopter  -x ../docs/breaks.gdb
	else
		./build-generic/bin/mincopter -x ./docs/breaks.gdb
	fi

else

	if [ -f "bin/mincopter" ]; then
		./bin/mincopter
	else
		./build-generic/bin/mincopter
	fi
fi


