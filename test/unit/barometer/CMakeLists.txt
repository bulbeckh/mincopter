
## This will be included from the source CMakeLists.txt file

## Create test executable
add_executable(test_barometer
	test_barometer_core.cpp

	## NOTE This is workaround to avoid include mc-common which incorrectly depends on mincopter and other control/planning layer libs
	${CMAKE_SOURCE_DIR}/lib/Filter/DerivativeFilter.cpp
)

## Link
target_link_libraries(test_barometer PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test_barometer PUBLIC
	${MC_INCLUDE_DIRS}
)



