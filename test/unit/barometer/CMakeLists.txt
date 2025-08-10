
## This will be included from the source CMakeLists.txt file

## Add the hardware devices
file(GLOB_RECURSE barometer_src "${CMAKE_SOURCE_DIR}/dev/AP_Baro/*.cpp")

## Add the barometer library (with simulation)
add_library(mc-test-barometer STATIC
	${barometer_src}
	${CMAKE_SOURCE_DIR}/sim/sim_barometer.cpp
)

## Add gtest here if used

## Create test executable
add_executable(test_barometer
	test_barometer_core.cpp
)

## Link
target_link_libraries(test_barometer PUBLIC
	mc-test-barometer

	## gtest libs
)


