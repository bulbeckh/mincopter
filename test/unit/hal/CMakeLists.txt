## Testing of HAL

## HAL Initialisation Testing
add_executable(test-hal-init
	test_hal_init.cpp
)

target_compile_definitions(test-hal-init PRIVATE
	PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_include_directories(test-hal-init PUBLIC
	${MC_INCLUDE_DIRS}
)

target_link_libraries(test-hal-init PRIVATE
	mc-arch
	GTest::gtest_main
)

## HAL Message Send Initialisation
add_executable(test-hal-sendmessage
	test_hal_sendmessage.cpp
)

target_compile_definitions(test-hal-sendmessage PRIVATE
	PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_include_directories(test-hal-sendmessage PUBLIC
	${MC_INCLUDE_DIRS}
)

target_link_libraries(test-hal-sendmessage PRIVATE
	mc-arch
	GTest::gtest_main
)

gtest_discover_tests(test-hal-init)
gtest_discover_tests(test-hal-sendmessage)


