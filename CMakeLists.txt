cmake_minimum_required(VERSION 3.22)

#[[

MinCopter CMakeLists
- Need to implement cross-compilation for multiple targets

]]

project(mincopter)

function (set_toolchain TARGET_ARCH)
	message(STATUS "Using target architecture ${TARGET_ARCH}")

	if(TARGET_ARCH STREQUAL "avr")
		set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/avr.cmake" PARENT_SCOPE)
  elseif(TARGET_ARCH STREQUAL "x86_64")
		set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/x86_64.cmake" PARENT_SCOPE)
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

message("TOOLCHAIN FILE: ${TOOLCHAIN_FILE}")
include(${TOOLCHAIN_FILE})

# Apply the common flags to both C and C++ files
add_compile_options(${COMMON_FLAGS})
add_link_options(${LINKER_FLAGS} ${COMMON_FLAGS})

## Rewrite
list(APPEND MC_INCLUDE_DIRS
	AC_Fence
	AC_PID
	AC_WPNav
	AP_ADC
	AP_ADC_AnalogSource
	AP_AHRS
	AP_Airspeed
	AP_Baro
	AP_BattMonitor
	AP_Buffer
	AP_Common
	AP_Compass
	AP_Curve
	AP_Declination
	AP_GPS
	AP_HAL
	AP_HAL/utility
	AP_HAL_AVR
	AP_HAL_AVR/utility
	#AP_HAL_Linux
	AP_InertialNav
	AP_InertialSensor
	AP_Math
	AP_Menu
	AP_Motors
	APM_PI
	AP_Notify
	AP_Param
	AP_PerfMon
	AP_Progmem
	AP_RCMapper
	AP_Scheduler
	AP_Vehicle
	DataFlash
	Filter
	GCS_MAVLink
	## NOTE Mavlink only included due to having a dependency with other libs - remove this dependency
	PID
	RC_Channel
)

list(TRANSFORM MC_INCLUDE_DIRS PREPEND "libraries/")


## Build Libraries
foreach(lib IN LISTS MC_INCLUDE_DIRS)
	file(GLOB ${lib}_SRCS ${lib}/*.cpp ${lib}/*.c)
	list(LENGTH ${lib}_SRCS ${lib}_SRCS_LEN)
	if(NOT (${lib}_SRCS_LEN EQUAL 0))
		## Generate library
		string(REPLACE "/" "_" libname ${lib})
		message(STATUS "Generating library ${libname}")
		add_library(${libname} STATIC
			${${lib}_SRCS}
		)
		target_include_directories(${libname} PUBLIC
			${MC_INCLUDE_DIRS}
			/usr/lib/avr/include
		)
	else()
		message(STATUS "Skipped ${lib} due to no *.cpp src files")
	endif()
endforeach()

## Add manual links
target_link_libraries(libraries_AP_Motors PRIVATE libraries_AP_Curve)
target_link_libraries(libraries_AP_HAL_AVR PRIVATE libraries_AP_HAL)
target_link_libraries(libraries_AP_HAL PRIVATE libraries_AP_HAL_utility)
target_link_libraries(libraries_AP_HAL_AVR PRIVATE libraries_AP_HAL_AVR_utility)

## Build executable
add_executable(ArduCopter
	## sources
	ArduCopter/arducopter.cpp
	ArduCopter/attitude.cpp
	ArduCopter/util.cpp
	ArduCopter/control_modes.cpp
	ArduCopter/failsafe.cpp
	ArduCopter/motors.cpp
	ArduCopter/navigation.cpp
	ArduCopter/radio.cpp
	ArduCopter/system.cpp
	ArduCopter/log.cpp
	ArduCopter/parameters.cpp
	## Additional
	ArduCopter/serial.cpp
	ArduCopter/mcinstance.cpp
	ArduCopter/mcstate.cpp
	## NOTE Temporarily removing btree
	##ArduCopter/btree.cpp
	ArduCopter/menu.cpp
)

target_include_directories(ArduCopter PUBLIC
	${MC_INCLUDE_DIRS}
	ArduCopter
)

foreach(lib IN LISTS MC_INCLUDE_DIRS)
	string(REPLACE "/" "_" libname ${lib})
	message(STATUS ${libname})
	if(${libname} STREQUAL "libraries_GCS_Console")
		message(STATUS "Skipping link of GCS Console")
	elseif(${libname} STREQUAL "libraries_AP_Buffer")
		message(STATUS "Skipping link of AP_Buffer")
	elseif(${libname} STREQUAL "libraries_AP_SpdHgtControl")
		message(STATUS "Skipping link of AP_SpdHgtControl")
	elseif(${libname} STREQUAL "libraries_AP_Navigation")
		message(STATUS "Skipping link of AP_Navigation")
	elseif(${libname} STREQUAL "libraries_AP_Vehicle")
		message(STATUS "Skipping link of AP_Vehicle")
	else()
		message(STATUS "linking ${libname}")
		target_link_libraries(ArduCopter ${libname})
	endif()
endforeach()

## manuall add libgcc
target_link_libraries(ArduCopter /usr/lib/gcc/avr/7.3.0/avr6/libgcc.a)

## DEPENDENCIES

# BTree
## NOTE Temporarily removing btree
#add_subdirectory(./btree)
#target_link_libraries(ArduCopter btree)
#target_include_directories(ArduCopter PUBLIC ./btree/src)


