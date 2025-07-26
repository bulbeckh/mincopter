## x86-64 (Linux) Toolchain File

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(CMAKE_C_COMPILER x86_64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER x86_64-linux-gnu-g++)

#set(CMAKE_C_FLAGS "-march=x86-64 -O2 -Wall -Wextra")
#set(CMAKE_CXX_FLAGS "-march=x86-64 -O2 -Wall -Wextra")

add_compile_definitions(
	#BOARD=atmega2560
	CONFIG_HAL_BOARD=HAL_BOARD_LINUX
	#PORT=/dev/ttyACM0
	F_CPU=16000000L
	#_GNU_SOURCE
	TARGET_ARCH_LINUX

	## NOTE TODO This should really be defined elsewhere
	#CONTROLLER_MPC
	#PLANNER_WAYPOINT
)

set(ARCHITECTURE elf_x86_64)

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
	## TODO We have turned off optimisation here for simulation
	-O0
	-Wall
	-Wshadow
	-Wpointer-arith
	-Wcast-align
	-Wwrite-strings
	-Wformat=2
	
	## RPI specifics
	-pthread
	-lpigpiod-if2
	-lrt

#[[
	-ffunction-sections
	-fdata-sections
#]]
	-fsigned-char
	-march=x86-64
	## Add debug symbols
	-g
	## Add profiling information  - REMOVED
	-pg
	-no-pie
	-fno-builtin
	
)
message("x86-64 Architecture - common flags: ${COMMON_FLAGS}")

#[[ LINKER FLAGS

gc-sections : allow removal (garbage collection) of unused parts of input files (.o)
relax : allows some link optimizations

]]

set(LINKER_FLAGS
	-Wl,-m,${ARCHITECTURE}
	-Wl,--relax
)
message("x86-64 Architecture - linker flags: ${LINKER_FLAGS}")

# Apply the common flags to both C and C++ files
add_compile_options(${COMMON_FLAGS})
add_link_options(${LINKER_FLAGS} ${COMMON_FLAGS})


message(STATUS "Loaded x86_64 toolchain file")


