cmake_minimum_required(VERSION 3.22)

project(mincopter)

message(STATUS Using ${CMAKE_C_COMPILER})

list(APPEND LIBS
	AC_Fence
	AC_PID
	AC_Sprayer
	AC_WPNav
	AP_ADC
	AP_ADC_AnalogSource
	AP_AHRS
	AP_Airspeed
	AP_Arming
	AP_Baro
	AP_BattMonitor
	AP_BoardConfig
	AP_Buffer
	AP_Camera
	AP_Common
	AP_Compass
	AP_Curve
	AP_Declination
	AP_EPM
	AP_GPS
	AP_HAL
	AP_HAL_AVR
	AP_HAL_AVR_SITL
	AP_HAL_Empty
	AP_HAL_FLYMAPLE
	AP_HAL_Linux
	AP_HAL_PX4
	AP_InertialNav
	AP_InertialSensor
	AP_L1_Control
	AP_Limits
	AP_Math
	APM_Control
	AP_Menu
	APM_OBC
	AP_Motors
	AP_Mount
	APM_PI
	AP_Navigation
	AP_Notify
	AP_OpticalFlow
	AP_Param
	AP_PerfMon
	AP_Progmem
	AP_RangeFinder
	AP_RCMapper
	AP_Relay
	AP_Scheduler
	AP_ServoRelayEvents
	AP_SpdHgtControl
	AP_TECS
	AP_Vehicle
	DataFlash
	doc
	Filter
	GCS_Console
	GCS_MAVLink
	PID
	RC_Channel
	SITL
)

add_compile_definitions(
	BOARD=mega2560
	CONFIG_HAL_BOARD=HAL_BOARD_APM2
	PORT=/dev/ttyACM0
	F_CPU=16000000L
	_GNU_SOURCE
)

add_compile_options(
	-mmcu=atmega2560 -mcall-prologues
	-Wformat=2 -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings
	-Wno-reorder
	-ffunction-sections -fdata-sections -fno-exceptions -fsigned-char
	-g -O0
)

#file(GLOB AC_PID_SRCS libraries/AC_PID/*.cpp)
#message(STATUS ${AC_PID_SRCS})

#add_library(AC_PID STATIC
#)

## Setup include_dirs
foreach(lib IN LISTS LIBS)
	list(APPEND MINCOPTER_INCLUDE_DIRS libraries/${lib})
endforeach()
message(STATUS ${MINCOPTER_INCLUDE_DIRS})

foreach(lib IN LISTS LIBS)
	file(GLOB ${lib}_SRCS libraries/${lib}/*.cpp)
	#message(STATUS "Library ${lib} has following srcs ${${lib}_SRCS}")
	list(LENGTH ${lib}_SRCS ${lib}_SRCS_LEN)
	if(NOT (${lib}_SRCS_LEN EQUAL 0))
		add_library(LIB_${lib} STATIC
			${${lib}_SRCS}
		)
		target_include_directories(LIB_${lib} PUBLIC
			${MINCOPTER_INCLUDE_DIRS}
			/usr/lib/avr/include
		)
	else()
		message(STATUS "Skipped ${lib} due to no *.cpp src files")
	endif()
	
endforeach()




