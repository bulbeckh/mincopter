
## This will be included from the source CMakeLists.txt file

file(GLOB math_srcs ${CMAKE_SOURCE_DIR}/lib/AP_Math/*.cpp)

## Create test executable
add_executable(test_state
	test_state_core.cpp

	## Again, referencing AP_Math to avoid linking with mc-common
	${math_srcs}
)

## Link
target_link_libraries(test_state PUBLIC
	mc-dev
	mc-arch
	## gtest libs
)

target_include_directories(test_state PUBLIC
	${MC_INCLUDE_DIRS}
)



