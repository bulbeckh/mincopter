
## CMake Toolchain file for newer series AVR boards i.e. AVR64xx and AVR128xx

## TODO Update these definitions

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

## TODO This is the standard path for the Microchip XC8 install but should make it configurable
#set(CMAKE_C_COMPILER /opt/microchip/xc8/v3.10/bin/xc8-cc)
#set(CMAKE_CXX_COMPILER /opt/microchip/xc8/v3.10/bin/xc8-cc)

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

#set(CMAKE_C_COMPILER_WORKS 1)
#set(CMAKE_CXX_COMPILER_WORKS 1)

set(DFP_LOCATION ${CMAKE_SOURCE_DIR}/arch/avr/dx-dfp )

add_compile_definitions(
	#BOARD=atmega2560
	HAL_BOARD_AVRDX
	#PORT=/dev/ttyACM0
	F_CPU=16000000L
	#_GNU_SOURCE
	## TODO Do we really need to use this anymore - is used in a few places but can probably replace
	TARGET_ARCH_AVR
)

#set(MCU atmega2560)
#set(ARCHITECTURE avr6)


#[[ COMMON FLAGS
mmcu  AVR specific flag for specifying target architecture
mcall-prologues  function prologues/epilogues are expanded as calls to subroutines
Os  optimize for size (i.e. optimize where possible except when it increase bin size)
Wall  display all warnings
Wshadow  warn when a local variable shadows another
Wpointer-arith  warn when something depends on the 'size' of a function type or void*
Wcast-align  warn when casting of variables changes alignment requirements
Wwrite-strings  warn about string constness
Wformat=2  check printf and scanf functions for correct formats
ffunction-sections  places each function into its own section in the output file. Used by linker to optimise locality of reference
fdata-sections  as above but for data
fsigned-char  allows char to be signed

]]


message("Using: -specs=${CMAKE_SOURCE_DIR}/arch/avr/dx-dfp/xc8/avr/device-specs/specs-${TARGET_ARCH}")

## NOTE TARGET_ARCH should be the same as the mmcu option for avr targets
set(COMMON_FLAGS
	#-v
	-specs=${CMAKE_SOURCE_DIR}/arch/avr/dx-dfp/xc8/avr/device-specs/specs-${TARGET_ARCH}
	#-std=gnu++11
	#-mcpu=${TARGET_ARCH}
	#-mdfp=${DFP_LOCATION}/xc8
	-mmcu=${TARGET_ARCH}
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
	-fstack-usage
	-I${DFP_LOCATION}/xc8/avr/include
	#-I/opt/microchip/xc8/v3.10/avr/avr/include
	#-I/opt/microchip/xc8/v3.10/avr/lib/gcc/avr/include
	#-I/opt/microchip/xc8/v3.10/avr/lib/gcc/avr/include-fixed
)
message("AVR Architecture - common flags: ${COMMON_FLAGS}")

#[[ LINKER FLAGS

gc-sections : allow removal (garbage collection) of unused parts of input files (.o)
relax : allows some link optimizations

]]

set(LINKER_FLAGS
	-Wl,--gc-sections
	-Wl,-Map,${CMAKE_BINARY_DIR}/output.map
	## NOTE I don't think this is needed
	#-Wl,-m,${MARCHITECTURE}
	-Wl,--relax
)
message("AVR Architecture - linker flags: ${LINKER_FLAGS}")

# Apply the common flags to both C and C++ files
add_compile_options(${COMMON_FLAGS})
add_link_options(${LINKER_FLAGS} ${COMMON_FLAGS})

message(STATUS "Loaded AVR toolchain file")


