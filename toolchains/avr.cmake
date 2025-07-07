## CMake AVR Toolchain file


## TODO Update these definitions

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

add_compile_definitions(
	#BOARD=atmega2560
	CONFIG_HAL_BOARD=HAL_BOARD_APM2
	#PORT=/dev/ttyACM0
	F_CPU=16000000L
	#_GNU_SOURCE
	TARGET_ARCH_AVR

	## NOTE TODO This should really be defined elsewhere
	CONTROLLER_MPC
	#PLANNER_WAYPOINT
	PLANNER_NONE
)

set(MCU atmega2560)
set(ARCHITECTURE avr6)


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
message("AVR Architecture - common flags: ${COMMON_FLAGS}")

#[[ LINKER FLAGS

gc-sections : allow removal (garbage collection) of unused parts of input files (.o)
relax : allows some link optimizations

]]

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

message(STATUS "Loaded AVR toolchain file")


