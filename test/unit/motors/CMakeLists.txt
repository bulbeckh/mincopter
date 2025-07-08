
## This will be included from the source CMakeLists.txt file

message(STATUS "Building motors test file")

## Add gtest here if used

## Create test executable
add_executable(test-motors-sim
	test_motor_sim_core.cpp
	${CMAKE_SOURCE_DIR}/mincopter/mcinstance.cpp
	${CMAKE_SOURCE_DIR}/mincopter/init.cpp

)

target_include_directories(test-motors-sim PUBLIC
	${MC_INCLUDE_DIRS}
)

target_link_libraries(test-motors-sim PUBLIC
	mc-control
	libs_state
	libs_sim
	libs_dev
	libs_common
	libs_arch

	## gtest libs
)


