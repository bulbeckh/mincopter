cmake_minimum_required(VERSION 3.22)

#[[

MinCopter CMakeLists
- Need to implement cross-compilation for multiple targets

]]


### Sets the correct toolchain based on the target architecture
function (set_toolchain TARGET_ARCH)
	message(STATUS "Using target architecture ${TARGET_ARCH}")

	if(TARGET_ARCH STREQUAL "avr")
		set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/avr.cmake" PARENT_SCOPE)
		message("Set TOOLCHAIN_FILE to: ${TOOLCHAIN_FILE}")
  	elseif(TARGET_ARCH STREQUAL "x86_64")
		set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/x86_64.cmake" PARENT_SCOPE)
		message("Set TOOLCHAIN_FILE to: ${TOOLCHAIN_FILE}")
	elseif(TARGET_ARCH STREQUAL "rpi")
		## Just use x86_64 for rpi but really should be native (windows, mac, etc.)
		set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/rpi.cmake" PARENT_SCOPE)
		message("Set TOOLCHAIN_FILE to: ${TOOLCHAIN_FILE}")
  	else()
      	message(FATAL_ERROR "Unsupported target architecture: ${TARGET_ARCH}")
  	endif()
endfunction()

if (DEFINED TARGET_ARCH)
	set_toolchain(${TARGET_ARCH})
else()
	message("Target Architecture not supplied, use -DTARGET_ARCH=<arch>")
	set_toolchain("avr")
endif()


include(${TOOLCHAIN_FILE})

project(mincopter)

####### CONFIGURATION

include(mc-config.cmake)

## TODO change the way simlog and GZ_Interface are done here - both should be dependent on existing flags
## and simlog should be part of storage/logger anyway
if(MC_SIM_TYPE STREQUAL "SIM")
	add_compile_definitions( MC_SIM )
endif()

if(MC_SIMLOG STREQUAL "TRUE")
	add_compile_definitions( MC_SIMLOG ) 
endif()

add_compile_definitions( MC_GZ_INTERFACE )

## TODO We shouldn't have to know the SIM_LEVEL anywhere other than CMake. Change this
if(MC_SIM_LEVEL STREQUAL "2")
	add_compile_definitions( MC_SIM_LEVEL=2 )
elseif(MC_SIM_LEVEL STREQUAL "1")
	add_compile_definitions( MC_SIM_LEVEL=1 )
elseif(MC_SIM_LEVEL STREQUAL "0")
	add_compile_definitions( MC_SIM_LEVEL=0 )
endif()

if(MC_SIM_TYPE STREQUAL "SIM" AND MC_SIM_LEVEL STREQUAL "2")
	add_compile_definitions( MC_AHRS_SIM )
elseif(MC_AHRS_TYPE STREQUAL "DCM")
	add_compile_definitions( MC_AHRS_DCM )
endif()

if(MC_SIM_TYPE STREQUAL "SIM" AND MC_SIM_LEVEL STREQUAL "2")
	add_compile_definitions( MC_INAV_SIM )
elseif(MC_INAV_TYPE STREQUAL "DEFAULT")
	add_compile_definitions( MC_INAV_DEFAULT )
endif()


if(MC_CONTROLLER_TYPE STREQUAL "MPC")
	add_compile_definitions( CONTROLLER_MPC )
elseif(MC_CONTROLLER_TYPE STREQUAL "PID")
	add_compile_definitions( CONTROLLER_PID )
elseif(MC_CONTROLLER_TYPE STREQUAL "NONE")
	add_compile_definitions( CONTROLLER_NONE )
endif()

if(MC_PLANNER_TYPE STREQUAL "WAYPOINT")
	add_compile_definitions( PLANNER_WAYPOINT )
elseif(MC_PLANNER_TYPE STREQUAL "NONE")
	add_compile_definitions( PLANNER_NONE )
endif()


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

message(STATUS "TOOLCHAIN FILE: ${TOOLCHAIN_FILE}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")

#[[ Main Directories

- state/
- control/
- arch/
- dev/
- lib/
- arducopter/ (soon to be renamed to mincopter)

]]

list(APPEND MC_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}/mincopter/
	${CMAKE_SOURCE_DIR}/arch/AP_HAL
	${CMAKE_SOURCE_DIR}/arch/AP_HAL_AVR
	${CMAKE_SOURCE_DIR}/arch/AP_HAL_Linux
	${CMAKE_SOURCE_DIR}/arch/AP_HAL_RPI
	${CMAKE_SOURCE_DIR}/arch/AP_Progmem
	${CMAKE_SOURCE_DIR}/arch/AP_Scheduler
	${CMAKE_SOURCE_DIR}/lib/AP_Common
	${CMAKE_SOURCE_DIR}/lib/AP_Curve
	${CMAKE_SOURCE_DIR}/lib/AP_Menu
	${CMAKE_SOURCE_DIR}/lib/AP_Math
	${CMAKE_SOURCE_DIR}/lib/Filter
	${CMAKE_SOURCE_DIR}/lib/RC_Channel
	${CMAKE_SOURCE_DIR}/lib/
	${CMAKE_SOURCE_DIR}/state/AP_AHRS
	${CMAKE_SOURCE_DIR}/state/AP_InertialNav
	${CMAKE_SOURCE_DIR}/state/
	${CMAKE_SOURCE_DIR}/dev/AP_ADC
	${CMAKE_SOURCE_DIR}/dev/AP_ADC_AnalogSource
	${CMAKE_SOURCE_DIR}/dev/AP_Baro
	${CMAKE_SOURCE_DIR}/dev/AP_BattMonitor
	${CMAKE_SOURCE_DIR}/dev/AP_Compass
	${CMAKE_SOURCE_DIR}/dev/AP_Declination
	${CMAKE_SOURCE_DIR}/dev/AP_GPS
	${CMAKE_SOURCE_DIR}/dev/AP_InertialSensor
	${CMAKE_SOURCE_DIR}/dev/AP_Motors
	${CMAKE_SOURCE_DIR}/dev/AP_Notify
	${CMAKE_SOURCE_DIR}/dev/DataFlash
	${CMAKE_SOURCE_DIR}/control/include/controllers/AC_PID
	${CMAKE_SOURCE_DIR}/control/include/controllers/
	${CMAKE_SOURCE_DIR}/control/include/planners/AC_Fence
	${CMAKE_SOURCE_DIR}/control/include/planners/AC_WPNav
	${CMAKE_SOURCE_DIR}/control/include/planners/
	${CMAKE_SOURCE_DIR}/sim/
)

## Control
# OSQP Lib
#add_subdirectory( ${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen )
add_subdirectory( ${CMAKE_SOURCE_DIR}/control )

## State
add_subdirectory( ${CMAKE_SOURCE_DIR}/state )

## Dev
add_subdirectory( ${CMAKE_SOURCE_DIR}/dev )

## Sim
if(TARGET_ARCH STREQUAL "x86_64")
	add_subdirectory( ${CMAKE_SOURCE_DIR}/sim )
endif()

#### Build - arch/ ####
## This is dependent on the target architecture.
##file(GLOB_RECURSE MC_LIBS_ARCH "arch/*.cpp" "arch/*.c")

if (${TARGET_ARCH} STREQUAL "avr")
	file(GLOB_RECURSE MC_LIBS_ARCH
		"arch/AP_HAL/*.cpp"
		"arch/AP_HAL/*.c"
		"arch/AP_HAL_AVR/*.cpp"
		"arch/AP_HAL_AVR/*.c"
		"arch/AP_Progmem/*cpp"
		"arch/AP_Scheduler/*.cpp"
	)
elseif(${TARGET_ARCH} STREQUAL "x86_64")
	file(GLOB_RECURSE MC_LIBS_ARCH
		"arch/AP_HAL/*.cpp"
		"arch/AP_HAL/*.c"
		"arch/AP_HAL_Linux/*.cpp"
		"arch/AP_Scheduler/*.cpp"
	)
elseif(${TARGET_ARCH} STREQUAL "rpi")
	file(GLOB_RECURSE MC_LIBS_ARCH
		"arch/AP_HAL/*.cpp"
		"arch/AP_HAL/*.c"
		"arch/AP_HAL_RPI/*.cpp"
		"arch/AP_Scheduler/*.cpp"
	)
endif()

foreach(src ${MC_LIBS_ARCH})
	message(VERBOSE "${src}")
endforeach()

add_library(mc-arch STATIC ${MC_LIBS_ARCH})

target_include_directories(mc-arch PUBLIC
	${MC_INCLUDE_DIRS}
)

## Common (lib/)
add_subdirectory( ${CMAKE_SOURCE_DIR}/lib )

#### Build - mincopter/ ####
add_executable(mincopter
	# TODO Change the name
	mincopter/mincopter.cpp
	mincopter/init.cpp
	mincopter/mcinstance.cpp
)

target_link_libraries(mincopter
	mc-arch
	#libs_sim
	mc-dev
	mc-control
	mc-state
	mc-common
)

if(TARGET_ARCH STREQUAL "x86_64")
target_link_libraries(mincopter
	mc-sim
)
endif()



target_include_directories(mincopter PUBLIC
	## TODO Need to separate private and public include directories (e.g. backend and interface)
	${MC_INCLUDE_DIRS}
)


## OSQP Testing
##find_package(osqp REQUIRED)

# For sim, can link with shared version but need static for embedded if this lib is used
## target_link_libraries(mytarget PRIVATE osqp::osqp)
## target_link_libraries(mytarget PRIVATE osqp::osqpstatic)

## MinCopter Testing
add_subdirectory(test)


