cmake_minimum_required(VERSION 3.22)

#[[

MinCopter CMakeLists
- Need to implement cross-compilation for multiple targets

]]

project(mincopter)

### Sets the correct toolchain based on the target architecture
function (set_toolchain TARGET_ARCH)
	message(STATUS "Using target architecture ${TARGET_ARCH}")

	if(TARGET_ARCH STREQUAL "avr")
		set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/avr.cmake" PARENT_SCOPE)
		message("Set TOOLCHAIN_FILE to: ${TOOLCHAIN_FILE}")
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

include(${TOOLCHAIN_FILE})

message("TOOLCHAIN FILE: ${TOOLCHAIN_FILE}")
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

#### Build - control/ ####

list(APPEND MC_INCLUDE_DIRS
	control/include
)

file(GLOB MC_LIBS_CONTROL "control/*.cpp")
foreach(src ${MC_LIBS_CONTROL})
	message(STATUS "${src}")
endforeach()

add_library(libs_control STATIC ${MC_LIBS_CONTROL})

## Temporary fix
target_include_directories(libs_control PUBLIC
	ArduCopter/
	arch/AP_HAL
	arch/AP_HAL_AVR
	arch/AP_HAL_Linux
	arch/AP_Progmem
	arch/AP_Scheduler
	lib/AP_Common
	lib/AP_Curve
	lib/AP_Menu
	lib/AP_Math
	lib/Filter
	lib/RC_Channel
	lib/
	state/AP_AHRS
	state/AP_InertialNav
	state/
	dev/AP_ADC
	dev/AP_ADC_AnalogSource
	dev/AP_Baro
	dev/AP_BattMonitor
	dev/AP_Compass
	dev/AP_Declination
	dev/AP_GPS
	dev/AP_InertialSensor
	dev/AP_Motors
	dev/AP_Notify
	dev/DataFlash
	control/include/controllers/AC_PID
	control/include/controllers/
	control/include/planners/AC_Fence
	control/include/planners/AC_WPNav
	control/include/planners/
	sim/
)

get_target_property(dirs libs_control INCLUDE_DIRECTORIES)
message(STATUS "include for libs_control: ${dirs}")

#### Build - state/ ####

file(GLOB_RECURSE MC_LIBS_STATE "state/*.cpp")
foreach(src ${MC_LIBS_STATE})
	message(STATUS "${src}")
endforeach()

add_library(libs_state STATIC ${MC_LIBS_STATE})
target_include_directories(libs_state PUBLIC
	ArduCopter/
	arch/AP_HAL
	arch/AP_HAL_AVR
	arch/AP_HAL_Linux
	arch/AP_Progmem
	arch/AP_Scheduler
	lib/AP_Common
	lib/AP_Curve
	lib/AP_Menu
	lib/AP_Math
	lib/Filter
	lib/RC_Channel
	lib/
	state/AP_AHRS
	state/AP_InertialNav
	state/
	dev/AP_ADC
	dev/AP_ADC_AnalogSource
	dev/AP_Baro
	dev/AP_BattMonitor
	dev/AP_Compass
	dev/AP_Declination
	dev/AP_GPS
	dev/AP_InertialSensor
	dev/AP_Motors
	dev/AP_Notify
	dev/DataFlash
	sim/
)

#### Build - dev/ and sim/ ####
file(GLOB_RECURSE MC_LIBS_DEV "dev/*.cpp")
foreach(src ${MC_LIBS_DEV})
	message(STATUS "${src}")
endforeach()

add_library(libs_dev STATIC ${MC_LIBS_DEV})
target_include_directories(libs_dev PUBLIC
	arch/AP_HAL
	arch/AP_HAL_AVR
	arch/AP_HAL_Linux
	arch/AP_Progmem
	arch/AP_Scheduler
	lib/AP_Common
	lib/AP_Curve
	lib/AP_Menu
	lib/AP_Math
	lib/Filter
	lib/RC_Channel
	lib/
	dev/AP_ADC
	dev/AP_ADC_AnalogSource
	dev/AP_Baro
	dev/AP_BattMonitor
	dev/AP_Compass
	dev/AP_Declination
	dev/AP_GPS
	dev/AP_InertialSensor
	dev/AP_Motors
	dev/AP_Notify
	dev/DataFlash
	sim/
)

file(GLOB MC_LIBS_SIM "sim/*.cpp")
foreach(src ${MC_LIBS_SIM})
	message(STATUS "${src}")
endforeach()

add_library(libs_sim STATIC ${MC_LIBS_SIM})
target_include_directories(libs_sim PUBLIC
	arch/AP_HAL
	arch/AP_HAL_AVR
	arch/AP_HAL_Linux
	arch/AP_Progmem
	arch/AP_Scheduler
	lib/AP_Common
	lib/AP_Curve
	lib/AP_Menu
	lib/AP_Math
	lib/Filter
	lib/RC_Channel
	lib/
	dev/AP_ADC
	dev/AP_ADC_AnalogSource
	dev/AP_Baro
	dev/AP_BattMonitor
	dev/AP_Compass
	dev/AP_Declination
	dev/AP_GPS
	dev/AP_InertialSensor
	dev/AP_Motors
	dev/AP_Notify
	dev/DataFlash
	sim/
)

#### Build - arch/ ####
file(GLOB_RECURSE MC_LIBS_ARCH "arch/*.cpp" "arch/*.c")
foreach(src ${MC_LIBS_ARCH})
	message(STATUS "${src}")
endforeach()

add_library(libs_arch STATIC ${MC_LIBS_ARCH})
target_include_directories(libs_arch PUBLIC
	lib/AP_Common
	lib/AP_Curve
	lib/AP_Menu
	lib/AP_Math
	lib/Filter
	lib/RC_Channel
	lib/
	arch/AP_HAL
	arch/AP_HAL_AVR
	arch/AP_HAL_Linux
	arch/AP_Progmem
	arch/AP_Scheduler
	dev/AP_ADC
	dev/AP_ADC_AnalogSource
	dev/AP_Baro
	dev/AP_BattMonitor
	dev/AP_Compass
	dev/AP_Declination
	dev/AP_GPS
	dev/AP_InertialSensor
	dev/AP_Motors
	dev/AP_Notify
	dev/DataFlash
	sim/
)

#### Build - common libraries under lib/ ####
file(GLOB_RECURSE MC_LIBS_COMMON "lib/*.cpp")
foreach(src ${MC_LIBS_COMMON})
	message(STATUS "${src}")
endforeach()

add_library(libs_common STATIC ${MC_LIBS_COMMON})
target_include_directories(libs_common PUBLIC
	ArduCopter/
	lib/AP_Common
	lib/AP_Curve
	lib/AP_Menu
	lib/AP_Math
	lib/Filter
	lib/RC_Channel
	lib/
	state/AP_AHRS
	state/AP_InertialNav
	state/
	arch/AP_HAL
	arch/AP_HAL_AVR
	arch/AP_HAL_Linux
	arch/AP_Progmem
	arch/AP_Scheduler
	dev/AP_ADC
	dev/AP_ADC_AnalogSource
	dev/AP_Baro
	dev/AP_BattMonitor
	dev/AP_Compass
	dev/AP_Declination
	dev/AP_GPS
	dev/AP_InertialSensor
	dev/AP_Motors
	dev/AP_Notify
	dev/DataFlash
	control/include/controllers/AC_PID
	control/include/controllers/
	control/include/planners/AC_Fence
	control/include/planners/AC_WPNav
	control/include/planners/
	sim/
)

#### Build - mincopter/ ####
add_executable(mincopter
	# TODO Change the name
	ArduCopter/arducopter.cpp
	ArduCopter/init.cpp
	ArduCopter/mcinstance.cpp
)

target_link_libraries(mincopter
	libs_arch
	libs_sim
	libs_dev
	libs_control
	libs_state
	libs_common
)

target_include_directories(mincopter PUBLIC
	ArduCopter/
	arch/AP_HAL
	arch/AP_HAL_AVR
	arch/AP_HAL_Linux
	arch/AP_Progmem
	arch/AP_Scheduler
	lib/AP_Common
	lib/AP_Curve
	lib/AP_Menu
	lib/AP_Math
	lib/Filter
	lib/RC_Channel
	lib/
	state/AP_AHRS
	state/AP_InertialNav
	state/
	dev/AP_ADC
	dev/AP_ADC_AnalogSource
	dev/AP_Baro
	dev/AP_BattMonitor
	dev/AP_Compass
	dev/AP_Declination
	dev/AP_GPS
	dev/AP_InertialSensor
	dev/AP_Motors
	dev/AP_Notify
	dev/DataFlash
	control/include/controllers/AC_PID
	control/include/controllers/
	control/include/planners/AC_Fence
	control/include/planners/AC_WPNav
	control/include/planners/
	sim/
)

#[[

## Get list of libraries to include headers
file(GLOB MC_INCLUDE_DIRS LIST_DIRECTORIES true RELATIVE ${CMAKE_SOURCE_DIR} "${CMAKE_SOURCE_DIR}/libraries/*")
message(STATUS "Found the following for MC_INCLUDE_DIRS: ${MC_INCLUDE_DIRS}")

## TODO Remove the AP_Buffer and AP_Vehicle from this list of core libraries. Find a better way to get list
## Architecture-independent libs
list(APPEND MC_LIBS_CORE
		AC_Fence
		AC_PID
		AC_WPNav
		APM_PI
		AP_ADC
		AP_ADC_AnalogSource
		AP_AHRS
		AP_Baro
		AP_BattMonitor
		#AP_Buffer
		AP_Common
		AP_Compass
		AP_Curve
		AP_Declination
		AP_GPS
		AP_HAL
		AP_HAL/utility
		AP_InertialNav
		AP_InertialSensor
		AP_Math
		AP_Menu
		AP_Motors
		AP_Notify
		##AP_Param
		AP_Scheduler
		DataFlash
		Filter
		##PID
		RC_Channel
)
## TODO Remove this way of doing it - there should be a better way
list(TRANSFORM MC_LIBS_CORE PREPEND "libraries/")

## TODO Remove AP_HAL_AVR_SITL from this list

## Architecture-specific libs
if (${TARGET_ARCH} STREQUAL "avr")
	list(APPEND MC_LIBS_AVR
		AP_Progmem
		AP_HAL_AVR
		AP_HAL_AVR/utility
	)
	list(TRANSFORM MC_LIBS_AVR PREPEND "libraries/")
	message(STATUS "Found the following for MC_LIBS_AVR: ${MC_LIBS_AVR}")
elseif(${TARGET_ARCH} STREQUAL "x86_64")
	list(APPEND MC_LIBS_X86_64
		AP_HAL_Linux
	)
	list(TRANSFORM MC_LIBS_X86_64 PREPEND "libraries/")
	message(STATUS "Found the following for MC_LIBS_X86_64: ${MC_LIBS_X86_64}")
endif()


## TODO The MC_INCLUDE_DIRS needs to be different fromt he below list. There needs
## to be a different way of generating libraries as non-avr targets may still include 
## headers like "AP_Progmem" but there is a compiler guard in this header checking that
## the target is actually an avr-target.

function (mc_add_lib)
	## Build libraries
	foreach(lib IN LISTS ARGN)
		file(GLOB ${lib}_SRCS ${lib}/*.cpp ${lib}/*.c)
		list(LENGTH ${lib}_SRCS ${lib}_SRCS_LEN)
		if(NOT (${lib}_SRCS_LEN EQUAL 0))
			## Generate library
			string(REPLACE "/" "_" libname ${lib})
			message(STATUS "Generating library ${libname}")
			add_library(${libname} STATIC
				${${lib}_SRCS}
			)
			## Include all MC_INCLUDE_DIRS by default
			target_include_directories(${libname} PUBLIC
				${MC_INCLUDE_DIRS}
			)
		else()
			message(STATUS "Skipped ${lib} due to no *.cpp src files")
		endif()
	endforeach()
endfunction()

## Build executable
add_executable(ArduCopter
	## sources
	ArduCopter/arducopter.cpp
	ArduCopter/attitude.cpp
	ArduCopter/util.cpp
	ArduCopter/control_modes.cpp
	ArduCopter/motors.cpp
	ArduCopter/navigation.cpp
	ArduCopter/radio.cpp
	ArduCopter/system.cpp
	ArduCopter/log.cpp
	## NOTE Removing this in remove-params branch : ArduCopter/parameters.cpp
	## Additional
	ArduCopter/serial.cpp
	ArduCopter/mcinstance.cpp
	ArduCopter/mcstate.cpp
	## NOTE Temporarily removing btree
	##ArduCopter/btree.cpp
	ArduCopter/menu.cpp
)

## Build all architecture independent libraries
mc_add_lib(${MC_LIBS_CORE})

## TODO Add check for avr/x86-64 to add libraries to
if (${TARGET_ARCH} STREQUAL "avr")
	message("MC_LIBS_AVR: ${MC_LIBS_AVR}")
	mc_add_lib(${MC_LIBS_AVR})
elseif(${TARGET_ARCH} STREQUAL "x86_64")
	message("MC_LIBS_X86_64 ${MC_LIBS_X86_64}")
	mc_add_lib(${MC_LIBS_X86_64})
endif()


## Add Sim libraries if Linux backend (sim) is being used
if(TARGET_ARCH STREQUAL "x86_64")
	file(GLOB SIM_SRCS sim/*.cpp sim/*.c)
	list(LENGTH SIM_SRCS SIM_SRCS_LEN)
	if(NOT SIM_SRCS_LEN EQUAL 0)
		add_library(libSimSensors STATIC
			${SIM_SRCS}
		)
		target_include_directories(libSimSensors PUBLIC
			${MC_INCLUDE_DIRS}
			sim
		)
	endif()
endif()

target_include_directories(ArduCopter PUBLIC
	${MC_INCLUDE_DIRS}
	ArduCopter
	sim
)

## Add manual links
target_link_libraries(libraries_AP_Motors PRIVATE libraries_AP_Curve)
target_link_libraries(libraries_AP_HAL PRIVATE libraries_AP_HAL_utility)

if (${TARGET_ARCH} STREQUAL "avr")
	target_link_libraries(libraries_AP_HAL_AVR PRIVATE libraries_AP_HAL_AVR_utility)
	target_link_libraries(libraries_AP_HAL_AVR PRIVATE libraries_AP_HAL)
endif()

if (${TARGET_ARCH} STREQUAL "x86_64")
	target_link_libraries(libraries_AP_HAL_Linux PRIVATE libraries_AP_HAL)

	target_link_libraries(libSimSensors PRIVATE
		libraries_AP_Compass
		libraries_AP_Baro
		libraries_AP_GPS
		libraries_AP_InertialSensor
		libraries_AP_ADC
	)
endif()

if(TARGET_ARCH STREQUAL "avr")
	foreach(lib IN LISTS MC_LIBS_AVR)
		string(REPLACE "/" "_" libname ${lib})
		message(STATUS "LIBNAME: ${libname}")
		target_link_libraries(ArduCopter PRIVATE ${libname})
	endforeach()
elseif(TARGET_ARCH STREQUAL "x86_64")
	## Link sim libraries
	target_link_libraries(ArduCopter PRIVATE libSimSensors)

	foreach(lib IN LISTS MC_LIBS_X86_64)
		string(REPLACE "/" "_" libname ${lib})
		message(STATUS "LIBNAME: ${libname}")
		target_link_libraries(ArduCopter PRIVATE ${libname})
	endforeach()
endif()

foreach(lib IN LISTS MC_LIBS_CORE)
	string(REPLACE "/" "_" libname ${lib})
	message(STATUS "LIBNAME: ${libname}")
	target_link_libraries(ArduCopter PRIVATE ${libname})
endforeach()

## manually add libgcc


if(TARGET_ARCH STREQUAL "avr")
	target_link_libraries(ArduCopter PRIVATE /usr/lib/gcc/avr/7.3.0/avr6/libgcc.a)
endif()

## DEPENDENCIES

# BTree
## NOTE Temporarily removing btree
#add_subdirectory(./btree)
#target_link_libraries(ArduCopter btree)
#target_include_directories(ArduCopter PUBLIC ./btree/src)


]]

