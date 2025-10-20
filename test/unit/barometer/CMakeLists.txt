
## This will be included from the source CMakeLists.txt file

## Create test executable
add_executable(test-barometer
	test_barometer_core.cpp

	## NOTE This is workaround to avoid include mc-common which incorrectly depends on mincopter and other control/planning layer libs
	${CMAKE_SOURCE_DIR}/lib/Filter/DerivativeFilter.cpp
)

## Link
target_link_libraries(test-barometer PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test-barometer PUBLIC
	${MC_INCLUDE_DIRS}
)

if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
	target_link_libraries(test-barometer PUBLIC
		-Wl,--whole-archive
		stm32_static
		-Wl,--no-whole-archive
	)
endif()


