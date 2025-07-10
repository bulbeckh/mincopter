
#### Build - state/ ####

## We have two options for state - for non-sim, we need to use an actual state estimation library like DCM
## For sim, depending on the level we will either use state estimation libs or a simulated state where we get
## state (position, velocity, etc.) directly from the gz_interface

list(APPEND MC_LIBS_STATE
	${CMAKE_SOURCE_DIR}/state/mcstate.cpp
	${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_interface.cpp
)

if(MC_SIM_TYPE STREQUAL "SIM")
	if(MC_SIM_LEVEL STREQUAL "2")
		list(APPEND MC_LIBS_STATE
			${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_sim.cpp
			${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_sim.cpp
		)
	else()
		if(MC_AHRS_TYPE STREQUAL "DCM")
			list(APPEND MC_LIBS_STATE
				${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_dcm.cpp
			)
		endif()

		## TODO Change name of this
		if(MC_INAV_TYPE STREQUAL "DEFAULT")
			list(APPEND MC_LIBS_STATE
				${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_default.cpp
			)
		endif()
	endif()
else()
	## Non-simulation
	if(MC_AHRS_TYPE STREQUAL "DCM")
		list(APPEND MC_LIBS_STATE
			${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_dcm.cpp
		)
	endif()

	## TODO Change name of this
	if(MC_INAV_TYPE STREQUAL "DEFAULT")
		list(APPEND MC_LIBS_STATE
			${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_default.cpp
		)
	endif()
endif()

## 
add_library(mc-state STATIC ${MC_LIBS_STATE})

target_include_directories(mc-state PUBLIC
	${MC_INCLUDE_DIRS}
)

