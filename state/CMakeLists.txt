
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
		## REMOVED DCM
		if(MC_AHRS_TYPE STREQUAL "DCM")
			list(APPEND MC_LIBS_STATE
				#${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_dcm.cpp
			)
		endif()

		# INAV DEFAULT Removed
		if(MC_INAV_TYPE STREQUAL "DEFAULT")
			list(APPEND MC_LIBS_STATE
				#${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_default.cpp
			)
		endif()

		## The EKF class implements both the AHRS and the INAV so we check the flag for either of them
		if(MC_INAV_TYPE STREQUAL "EKF" OR MC_AHRS_TYPE STREQUAL "EKF")
			list(APPEND MC_LIBS_STATE
				${CMAKE_SOURCE_DIR}/state/ekf/ekf.cpp
				${CMAKE_SOURCE_DIR}/state/ekf/ekf_predict.c
				${CMAKE_SOURCE_DIR}/state/ekf/ekf_correct.c
			)
		endif()
	endif()
else()
	## Non-simulation
	if(MC_AHRS_TYPE STREQUAL "DCM")
		list(APPEND MC_LIBS_STATE
			#${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_dcm.cpp
		)
	endif()

	## TODO Change name of this
	if(MC_INAV_TYPE STREQUAL "DEFAULT")
		## INAV DEFAULT Removed
		list(APPEND MC_LIBS_STATE
			#${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_default.cpp
		)
	endif()

	if(MC_INAV_TYPE STREQUAL "EKF" OR MC_AHRS_TYPE STREQUAL "EKF")
		list(APPEND MC_LIBS_STATE
			${CMAKE_SOURCE_DIR}/state/ekf/ekf.cpp
			${CMAKE_SOURCE_DIR}/state/ekf/ekf_predict.c
			${CMAKE_SOURCE_DIR}/state/ekf/ekf_correct.c
		)
	endif()
endif()

## 
add_library(mc-state STATIC ${MC_LIBS_STATE})

target_include_directories(mc-state PUBLIC
	${MC_INCLUDE_DIRS}
)

