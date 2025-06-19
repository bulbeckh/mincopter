
## Build - Control

file(GLOB MC_LIBS_CONTROL "${CMAKE_SOURCE_DIR}/control/*.cpp")

foreach(src ${MC_LIBS_CONTROL})
	message(VERBOSE "${src}")
endforeach()

add_library(libs_control STATIC ${MC_LIBS_CONTROL}
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/workspace.c
)

target_link_libraries(libs_control
	## OSQP Libraries
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/build/libosqpstatic.a
)

target_include_directories(libs_control PUBLIC
	${MC_INCLUDE_DIRS}

	## OSQP Headers
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/inc/private/
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/inc/public/
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/
)

get_target_property(dirs libs_control INCLUDE_DIRECTORIES)
message(VERBOSE "include for libs_control: ${dirs}")



