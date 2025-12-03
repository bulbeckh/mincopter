
## This will be included from the source CMakeLists.txt file

message(STATUS "Building HAL test file")

## Add gtest here if used

## Create test executable
add_executable(test-hal
	test_hal_core.cpp
)

target_link_libraries(test-hal PUBLIC
	mc-arch
	## gtest libs
)

if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
	target_link_libraries(test-hal PUBLIC
		-Wl,--whole-archive
		stm32_static
		-Wl,--no-whole-archive
	)
endif()


## Create test executable
add_executable(test-hal-cli
	test_hal_cli.cpp
)

target_link_libraries(test-hal-cli PUBLIC
	mc-arch
	mc-common
	## gtest libs
)

if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
	target_link_libraries(test-hal-cli PUBLIC
		-Wl,--whole-archive
		stm32_static
		-Wl,--no-whole-archive
	)
endif()


