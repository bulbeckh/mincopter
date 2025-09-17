
#### Build - state/ ####

## We have two options for state - for non-sim, we need to use an actual state estimation library like DCM
## For sim, depending on the level we will either use state estimation libs or a simulated state where we get
## state (position, velocity, etc.) directly from the gz_interface

list(APPEND MC_LIBS_STATE
	${CMAKE_SOURCE_DIR}/state/mcstate.cpp
)

## TODO This is so dumb having two like this 

## REMOVED DCM
if(MC_AHRS_TYPE STREQUAL "DCM")
	list(APPEND MC_LIBS_STATE
		#${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_dcm.cpp
	)
elseif(MC_AHRS_TYPE STREQUAL "NONE")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_none.cpp
	)
elseif(MC_AHRS_TYPE STREQUAL "COMPLEMENTARY")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_complementary.cpp
	)
elseif(MC_AHRS_TYPE STREQUAL "SIM")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/AP_AHRS/ahrs_sim.cpp
	)
endif()


## Inertial Navigation
if(MC_INAV_TYPE STREQUAL "DEFAULT")
	list(APPEND MC_LIBS_STATE
		#${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_default.cpp
	)
elseif(MC_INAV_TYPE STREQUAL "NONE")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_none.cpp
	)
elseif(MC_INAV_TYPE STREQUAL "SIM")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_sim.cpp
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

## 
add_library(mc-state STATIC ${MC_LIBS_STATE})

target_include_directories(mc-state PUBLIC
	${MC_INCLUDE_DIRS}
)

