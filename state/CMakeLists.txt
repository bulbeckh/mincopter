
#### Build - state/ ####


if(TARGET_ARCH STREQUAL "avr")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/mcstate.cpp
		${CMAKE_SOURCE_DIR}/state/AP_AHRS/AP_AHRS.cpp
		${CMAKE_SOURCE_DIR}/state/AP_AHRS/AP_AHRS_DCM.cpp
		${CMAKE_SOURCE_DIR}/state/AP_InertialNav/AP_InertialNav.cpp
	)
elseif(TARGET_ARCH STREQUAL "x86_64")
	## NOTE This is currently equivalent to a 'sim' flag but we need to add a compiler/cmake option for sim instead
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/mcstate.cpp
		${CMAKE_SOURCE_DIR}/state/AP_AHRS/AP_AHRS.cpp
		${CMAKE_SOURCE_DIR}/state/AP_AHRS/AHRS_sim.cpp
		${CMAKE_SOURCE_DIR}/state/AP_InertialNav/inav_interface.cpp
	)
endif()

foreach(src ${MC_LIBS_STATE})
	message(VERBOSE "${src}")
endforeach()

add_library(libs_state STATIC ${MC_LIBS_STATE})

target_include_directories(libs_state PUBLIC
	${MC_INCLUDE_DIRS}
)

