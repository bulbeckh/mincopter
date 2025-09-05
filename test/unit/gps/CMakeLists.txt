
## This will be included from the source CMakeLists.txt file

## Create test executable
add_executable(test_gps
	test_gps_core.cpp
	${CMAKE_SOURCE_DIR}/lib/AP_Math/AP_Math.cpp
)

## Link
target_link_libraries(test_gps PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test_gps PUBLIC
	${MC_INCLUDE_DIRS}
)



