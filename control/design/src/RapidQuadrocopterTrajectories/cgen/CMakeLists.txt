cmake_minimum_required(VERSION 3.15)

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

project(osqp_ext_emosqp)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)  # -fPIC
set(OSQP_EMBEDDED_MODE "1")

add_compile_definitions(
	F_CPU=16000000L
)
set(MCU atmega2560)
set(ARCHITECTURE avr6)
set(COMMON_FLAGS
	-mmcu=${MCU}
	-mcall-prologues
	-Os
	-Wall
	-Wshadow
	-Wpointer-arith
	-Wcast-align
	-Wwrite-strings
	-Wformat=2
	-ffunction-sections
	-fdata-sections
	-fsigned-char
)

set(LINKER_FLAGS
	-Wl,--gc-sections
	#-Wl,-Map
	#-Wl,${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map
	## NOTE I don't think this is needed
	-Wl,-m,${ARCHITECTURE}
	-Wl,--relax
)
message("AVR Architecture - linker flags: ${LINKER_FLAGS}")

# Apply the common flags to both C and C++ files
add_compile_options(${COMMON_FLAGS})
add_link_options(${LINKER_FLAGS} ${COMMON_FLAGS})

file(
  GLOB
  OSQP_SOURCES
  src/*.c)

add_library(osqpstatic
  STATIC
    ${OSQP_SOURCES}
  )

target_include_directories(osqpstatic PUBLIC inc/public inc/private .)
target_include_directories(osqpstatic PRIVATE inc/private .)

