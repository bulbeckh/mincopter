

#### Build - dev/

file(GLOB_RECURSE MC_LIBS_DEV "${CMAKE_SOURCE_DIR}/dev/*.cpp")

foreach(src ${MC_LIBS_DEV})
	message(VERBOSE "${src}")
endforeach()

add_library(mc-dev STATIC ${MC_LIBS_DEV})

target_include_directories(mc-dev PUBLIC
	${MC_INCLUDE_DIRS}
)
