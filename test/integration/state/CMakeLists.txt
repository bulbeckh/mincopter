
## This will be included from the source CMakeLists.txt file

if (MC_STATE_TYPE STREQUAL "COMPLEMENTARY")

file(GLOB math_srcs ${CMAKE_SOURCE_DIR}/lib/AP_Math/*.cpp)

#[[
## Create test executable
add_executable(test-state-core
	test_state_core.cpp

	## Again, referencing AP_Math to avoid linking with mc-common
	${math_srcs}
)

## Link
target_link_libraries(test-state-core PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test-state-core PUBLIC
	${MC_INCLUDE_DIRS}
)


add_executable(test-compensation
	test_sensor_compensation.cpp

	## Again, referencing AP_Math to avoid linking with mc-common
	${math_srcs}
)

## Link
target_link_libraries(test-compensation PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test-compensation PUBLIC
	${MC_INCLUDE_DIRS}
)
]]

## Complementary Integration Testing
add_executable(test-state-complementary
	test_state_complementary.cpp

	## Again, referencing AP_Math to avoid linking with mc-common
	${math_srcs}
)

## Link
target_link_libraries(test-state-complementary PUBLIC
	mc-dev
	mc-arch
	mc-state
	mc-common
	## gtest libs
)

target_include_directories(test-state-complementary PUBLIC
	${MC_INCLUDE_DIRS}
)

endif()
