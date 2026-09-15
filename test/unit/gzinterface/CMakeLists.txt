## Testing of Gazebo Simulation Interface

add_executable(test-gzinterface
	test_gzinterface.cpp
)

target_compile_definitions(test-gzinterface PRIVATE
	PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_include_directories(test-gzinterface PUBLIC
	${MC_INCLUDE_DIRS}
	${MC_TEST_SIMULATION_INCLUDE}
)

target_link_libraries(test-gzinterface PRIVATE
	## TODO Update these
	#mc-experimental-arch
	mc-arch
	#mc-experimental-dev
	gz-sim8::gz-sim8
	gz-common5::gz-common5
	GTest::gtest_main
)

gtest_discover_tests(test-gzinterface)


