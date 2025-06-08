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

add_library(libs_control STATIC ${MC_LIBS_CONTROL}
	control/design/src/RapidQuadrocopterTrajectories/cgen/workspace.c
)

target_link_libraries(libs_control
	## OSQP Libraries
	${CMAKE_SOURCE_DIR}/control/design/src/RapidQuadrocopterTrajectories/cgen/build/libosqpstatic.a
)

## Temporary fix
target_include_directories(libs_control PUBLIC
	mincopter/
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

	## OSQP Headers
	control/design/src/RapidQuadrocopterTrajectories/cgen/inc/private
	control/design/src/RapidQuadrocopterTrajectories/cgen/inc/public
	control/design/src/RapidQuadrocopterTrajectories/cgen/
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
	mincopter/
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
	state/AP_AHRS
	state/AP_InertialNav
	state/
	control/include/controllers/AC_PID
	control/include/controllers/
	control/include/planners/AC_Fence
	control/include/planners/AC_WPNav
	control/include/planners/
	sim/
	mincopter/
)

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
endif()

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
	mincopter/
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
	mincopter/mincopter.cpp
	mincopter/init.cpp
	mincopter/mcinstance.cpp
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
	mincopter/
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


## OSQP Testing
find_package(osqp REQUIRED)

# For sim, can link with shared version but need static for embedded if this lib is used
## target_link_libraries(mytarget PRIVATE osqp::osqp)
## target_link_libraries(mytarget PRIVATE osqp::osqpstatic)



