
#### Build - state/ ####

## We have two options for state - for non-sim, we need to use an actual state estimation library like DCM
## For sim, depending on the level we will either use state estimation libs or a simulated state where we get
## state (position, velocity, etc.) directly from the gz_interface

list(APPEND MC_LIBS_STATE
	${CMAKE_SOURCE_DIR}/state/mcstate.cpp
)

## TODO This is so dumb having two like this 

if(MC_SIM_TYPE STREQUAL "SIM" AND MC_SIM_LEVEL STREQUAL "2")
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

	if(MC_AHRS_TYPE STREQUAL "NONE")
		list(APPEND MC_LIBS_STATE
			${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_none.cpp
		)
	endif()

	if(MC_AHRS_TYPE STREQUAL "COMPLEMENTARY")
		list(APPEND MC_LIBS_STATE
			${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_complementary.cpp
		)
	endif()

	# INAV DEFAULT Removed
	if(MC_INAV_TYPE STREQUAL "DEFAULT")
		list(APPEND MC_LIBS_STATE
			#${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_default.cpp
		)
	endif()

	if(MC_INAV_TYPE STREQUAL "NONE")
		list(APPEND MC_LIBS_STATE
			${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_none.cpp
			)
	endif()

	## The EKF class implements both the AHRS and the INAV so we check the flag for either of them
	if(MC_INAV_TYPE STREQUAL "EKF" OR MC_AHRS_TYPE STREQUAL "EKF")
		list(APPEND MC_LIBS_STATE
			${CMAKE_SOURCE_DIR}/state/ekf/ekf.cpp
			#${CMAKE_SOURCE_DIR}/state/ekf/ekf_predict.c
			#${CMAKE_SOURCE_DIR}/state/ekf/ekf_correct.c
			${CMAKE_SOURCE_DIR}/state/ekf/ekf_predict_mx.c
			${CMAKE_SOURCE_DIR}/state/ekf/ekf_correct_mx.c
		)
	endif()
endif()

## 
add_library(mc-state STATIC ${MC_LIBS_STATE})

target_include_directories(mc-state PUBLIC
	${MC_INCLUDE_DIRS}
)

