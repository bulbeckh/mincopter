## Testing of Gazebo Simulation Interface

## TODO We should be moving all this configuration into a parent test cmake directory
include(FetchContent)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

FetchContent_MakeAvailable(googletest)
enable_testing()

find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-common5 REQUIRED)

### Test Specifics ################

add_executable(test-imu
	test_imu.cpp
)

target_compile_definitions(test-imu PRIVATE
	PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_include_directories(test-imu PUBLIC
	${MC_INCLUDE_DIRS}
	${MC_TEST_SIMULATION_INCLUDE}
)

target_link_libraries(test-imu PRIVATE
	mc-arch
	gz-sim8::gz-sim8
	gz-common5::gz-common5
	GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test-imu)

