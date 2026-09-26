
#### Build - common libraries under common/ ####

file(GLOB_RECURSE MC_LIBS_COMMON "${CMAKE_SOURCE_DIR}/common/*.cpp")

foreach(src ${MC_LIBS_COMMON})
	message(VERBOSE "${src}")
endforeach()

add_library(mc-common STATIC ${MC_LIBS_COMMON})

## mc-common is used in ArduPilotPlugin which is a shared library hence it needs to be PIC
set_target_properties(mc-common PROPERTIES
	POSITION_INDEPENDENT_CODE ON
)

target_include_directories(mc-common PUBLIC
	${MC_INCLUDE_DIRS}
)



