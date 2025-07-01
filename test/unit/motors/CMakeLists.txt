
## This will be included from the source CMakeLists.txt file

## Add the (simulated) motors library
add_library(mc-test-motors-sim STATIC
	${CMAKE_SOURCE_DIR}/gz_interface.cpp
)

## Add gtest here if used

## Create test executable
add_executable(test_motors_sim
	test_motor_sim_core.cpp
)


## Link
target_link_libraries(test_motors_sim PUBLIC
	mc-test-motors-sim

	## gtest libs
)


