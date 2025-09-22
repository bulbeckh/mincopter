
## Build - Control & Planner

list(APPEND mc-control-common
	${CMAKE_SOURCE_DIR}/control/AC_Fence.cpp
	${CMAKE_SOURCE_DIR}/control/AC_PID.cpp
	${CMAKE_SOURCE_DIR}/control/AC_WPNav.cpp
	${CMAKE_SOURCE_DIR}/control/radio.cpp
	${CMAKE_SOURCE_DIR}/control/mixer.cpp
)

list(APPEND mc-control-mpc
	${CMAKE_SOURCE_DIR}/control/controller_mpc.cpp
	${CMAKE_SOURCE_DIR}/control/controller_mpc_constructor.cpp
)

list(APPEND mc-control-pid
	${CMAKE_SOURCE_DIR}/control/controller_pid.cpp
)

list(APPEND mc-control-lqr
	${CMAKE_SOURCE_DIR}/control/controller_lqr.cpp
)

list(APPEND mc-control-csc
	${CMAKE_SOURCE_DIR}/control/controller_csc.cpp
)

list(APPEND mc-control-none
	${CMAKE_SOURCE_DIR}/control/controller_none.cpp
)

list(APPEND mc-planner-waypoint
	${CMAKE_SOURCE_DIR}/control/planner_waypoint.cpp
	${CMAKE_SOURCE_DIR}/control/arming.cpp
	${CMAKE_SOURCE_DIR}/control/failsafe.cpp
)

list(APPEND mc-planner-none
	${CMAKE_SOURCE_DIR}/control/planner_none.cpp
)

add_library(mc-control STATIC
	${mc-control-common}
)

if(MC_CONTROLLER_TYPE STREQUAL "MPC")
	target_sources(mc-control PUBLIC
		${mc-control-mpc}
	)
	target_link_libraries(mc-control PUBLIC
		## OSQP Libraries
		${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/build/libosqpstatic.a
	)
	target_include_directories(mc-control PUBLIC
		## OSQP Headers
		${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/inc/private/
		${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/inc/public/
		${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/
	)
elseif(MC_CONTROLLER_TYPE STREQUAL "PID")
	target_sources(mc-control PUBLIC
		${mc-control-pid}
	)
elseif(MC_CONTROLLER_TYPE STREQUAL "LQR")
	target_sources(mc-control PUBLIC
		${mc-control-lqr}
	)
elseif(MC_CONTROLLER_TYPE STREQUAL "CSC")
	target_sources(mc-control PUBLIC
		${mc-control-csc}
	)
elseif(MC_CONTROLLER_TYPE STREQUAL "NONE")
	target_sources(mc-control PUBLIC
		${mc-control-none}
	)
else()
	message(FATAL_ERROR "MC_CONTROLLER_TYPE is not valid") 
endif()

if(MC_PLANNER_TYPE STREQUAL "WAYPOINT")
	target_sources(mc-control PUBLIC
		${mc-planner-waypoint}
	)
elseif(MC_PLANNER_TYPE STREQUAL "NONE")
	target_sources(mc-control PUBLIC
		${mc-planner-none}
	)
else()
	message(FATAL_ERROR "MC_PLANNER_TYPE is not valid") 
endif()


target_include_directories(mc-control PUBLIC
	${MC_INCLUDE_DIRS}
)

get_target_property(dirs mc-control INCLUDE_DIRECTORIES)
message(VERBOSE "include for mc-control: ${dirs}")



