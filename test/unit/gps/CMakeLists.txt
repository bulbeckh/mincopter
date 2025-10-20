
## This will be included from the source CMakeLists.txt file

## Create test executable
add_executable(test-gps
	test_gps_core.cpp
	${CMAKE_SOURCE_DIR}/lib/AP_Math/AP_Math.cpp
)

## Link
target_link_libraries(test-gps PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test-gps PUBLIC
	${MC_INCLUDE_DIRS}
)

if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
	target_link_libraries(test-gps PUBLIC
		-Wl,--whole-archive
		stm32_static
		-Wl,--no-whole-archive
	)
endif()



