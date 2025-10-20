
## This will be included from the source CMakeLists.txt file

file(GLOB math_srcs ${CMAKE_SOURCE_DIR}/lib/AP_Math/*.cpp)

## Create test executable
add_executable(test-compass
	test_compass_core.cpp

	## Again, referencing AP_Math to avoid linking with mc-common
	${math_srcs}
)

## Link
target_link_libraries(test-compass PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test-compass PUBLIC
	${MC_INCLUDE_DIRS}
)

if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
	target_link_libraries(test-compass PUBLIC
		-Wl,--whole-archive
		stm32_static
		-Wl,--no-whole-archive
	)
endif()



