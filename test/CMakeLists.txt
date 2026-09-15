
set(MC_TEST_SIMULATION_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/sim/include)

include(FetchContent)

## GoogleTest
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

FetchContent_MakeAvailable(googletest)
enable_testing()

include(GoogleTest)

## Gazebo
find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-common5 REQUIRED)


## pugixml (XML Parser for SDF configuration)
FetchContent_Declare(
	pugixml
	GIT_REPOSITORY https://github.com/zeux/pugixml.git
	GIT_TAG v1.16
)
FetchContent_MakeAvailable(pugixml)

add_subdirectory(unit)


