
#### Build - state/ ####

file(GLOB_RECURSE MC_LIBS_STATE "${CMAKE_SOURCE_DIR}/state/*.cpp")
foreach(src ${MC_LIBS_STATE})
	message(VERBOSE "${src}")
endforeach()

add_library(libs_state STATIC ${MC_LIBS_STATE})

target_include_directories(libs_state PUBLIC
	${MC_INCLUDE_DIRS}
)

