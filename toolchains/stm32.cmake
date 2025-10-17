
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

add_compile_definitions(
	CONFIG_HAL_BOARD=HAL_BOARD_STM32
	TARGET_ARCH_STM32
)


## TODO Should -g3 be present in final executable?
set(COMMON_FLAGS
	-mcpu=${MCPU}
	-mfpu=fpv4-sp-d16
	-mfloat-abi=hard
	-g3
	-D${BOARD_DEFINE}
	-Wall
	-O0
	-MMD
	-MP
	-ffunction-sections
	-fdata-sections
	-std=gnu11
)

message("COMMON FLAGS ${COMMON_FLAGS}")

## NOTE TODO This linker script should be dependent on the model we are using

## Add linker script path here
set(LINKER_FLAGS
	-T ${CMAKE_SOURCE_DIR}/arch/arm/STM32F407VGTX_FLASH.ld
	#-Wl,--gc-sections
	--specs=nosys.specs
	-Wl,-Map=mincopter.map
	-static
	--specs=nano.specs
	-u _printf_float
	-mcpu=${MCPU}
	-mfpu=fpv4-sp-d16
	-mfloat-abi=hard
	-mthumb
	-Wl,--start-group -lc -lm -Wl,--end-group
)

message("stm32 Architecture - linker flags: ${LINKER_FLAGS}")

# Apply the common flags to both C and C++ files
add_compile_options(${COMMON_FLAGS})
add_link_options(${LINKER_FLAGS} ${COMMON_FLAGS})


message(STATUS "Loaded stm32 toolchain file")
