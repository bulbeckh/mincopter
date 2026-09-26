## Testing of Gazebo Simulation Interface

add_executable(test-gzinterface-connect
	test_gzinterface_connect.cpp
)

target_compile_definitions(test-gzinterface-connect PRIVATE
	PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_include_directories(test-gzinterface-connect PUBLIC
	${MC_INCLUDE_DIRS}
	${MC_TEST_SIMULATION_INCLUDE}
)

target_link_libraries(test-gzinterface-connect PRIVATE
	## TODO Update these
	#mc-experimental-arch
	mc-arch
	#mc-experimental-dev
	gz-sim8::gz-sim8
	gz-common5::gz-common5
	GTest::gtest_main
)

## Run
add_executable(test-gzinterface-run
	test_gzinterface_run.cpp
)

target_compile_definitions(test-gzinterface-run PRIVATE
	PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_include_directories(test-gzinterface-run PUBLIC
	${MC_INCLUDE_DIRS}
	${MC_TEST_SIMULATION_INCLUDE}
)

target_link_libraries(test-gzinterface-run PRIVATE
	## TODO Update these
	#mc-experimental-arch
	mc-arch
	#mc-experimental-dev
	gz-sim8::gz-sim8
	gz-common5::gz-common5
	GTest::gtest_main
)

gtest_discover_tests(test-gzinterface-connect)
gtest_discover_tests(test-gzinterface-run)


