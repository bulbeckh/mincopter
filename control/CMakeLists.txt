
## Build - Control & Planner

list(APPEND mc-control-common
	${CMAKE_SOURCE_DIR}/control/AC_Fence.cpp
	${CMAKE_SOURCE_DIR}/control/AC_PID.cpp
	${CMAKE_SOURCE_DIR}/control/AC_WPNav.cpp
	${CMAKE_SOURCE_DIR}/control/radio.cpp
)

list(APPEND mc-control-mpc
	${CMAKE_SOURCE_DIR}/control/controller_mpc.cpp
	${CMAKE_SOURCE_DIR}/control/controller_mpc_constructor.cpp
)

list(APPEND mc-control-pid
	${CMAKE_SOURCE_DIR}/control/controller_pid.cpp
)

list(APPEND mc-control-none
	${CMAKE_SOURCE_DIR}/control/controller_none.cpp
)

list(APPEND mc-planner-waypoint
	${CMAKE_SOURCE_DIR}/planner/planner_waypoint.cpp
	${CMAKE_SOURCE_DIR}/planner/arming.cpp
)

list(APPEND mc-planner-none
	${CMAKE_SOURCE_DIR}/planner/planner_none.cpp
)

add_library(mc-control STATIC
	${mc-control-common}

if()

target_sources(mc-control INTERFACE
	${mc-control-common}
)

target_link_libraries(mc-control PUBLIC
	## OSQP Libraries
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/build/libosqpstatic.a
)

target_include_directories(mc-control PUBLIC
	${MC_INCLUDE_DIRS}

	## OSQP Headers
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/inc/private/
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/inc/public/
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/
)

get_target_property(dirs mc-control INCLUDE_DIRECTORIES)
message(VERBOSE "include for libs_control: ${dirs}")



