
## This will be included from the source CMakeLists.txt file

## Create test executable
add_executable(test-df-core
	test_dataflash_core.cpp
)

## Link
target_link_libraries(test-df-core PUBLIC
	mc-dev
	mc-arch
	#mc-common
	## gtest libs
)

target_include_directories(test-df-core PUBLIC
	${MC_INCLUDE_DIRS}
)

if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
	target_link_libraries(test-df-core PUBLIC
		-Wl,--whole-archive
		stm32_static
		-Wl,--no-whole-archive
	)
endif()

