
#### Build - common libraries under lib/ ####

file(GLOB_RECURSE MC_LIBS_COMMON "${CMAKE_SOURCE_DIR}/lib/*.cpp")

foreach(src ${MC_LIBS_COMMON})
	message(VERBOSE "${src}")
endforeach()

add_library(libs_common STATIC ${MC_LIBS_COMMON})

target_include_directories(libs_common PUBLIC
	${MC_INCLUDE_DIRS}
)

target_link_libraries(libs_common PUBLIC mc-control)




