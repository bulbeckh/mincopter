
## This will be included from the source CMakeLists.txt file

file(GLOB math_srcs ${CMAKE_SOURCE_DIR}/lib/AP_Math/*.cpp)

## Create test executable
add_executable(test-imu-core
	test_imu_core.cpp

	## Again, referencing AP_Math to avoid linking with mc-common
	${math_srcs}
)

## Link
target_link_libraries(test-imu-core PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test-imu-core PUBLIC
	${MC_INCLUDE_DIRS}
)

if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
	target_link_libraries(test-imu-core PUBLIC
		-Wl,--whole-archive
		stm32_static
		-Wl,--no-whole-archive
	)
endif()

## Create test executable
add_executable(test-imu-read
	test_imu_read.cpp

	## Again, referencing AP_Math to avoid linking with mc-common
	${math_srcs}
)

## Link
target_link_libraries(test-imu-read PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test-imu-read PUBLIC
	${MC_INCLUDE_DIRS}
)

if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
	target_link_libraries(test-imu-read PUBLIC
		-Wl,--whole-archive
		stm32_static
		-Wl,--no-whole-archive
	)
endif()



