
#pragma once

/*
// HASH define HAL_BOARD_APM1     1
// HASH define HAL_BOARD_APM2     2
// HASH define HAL_BOARD_AVR_SITL 3
// HASH define HAL_BOARD_SMACCM   4 // unused
// HASH define HAL_BOARD_PX4      5
// HASH define HAL_BOARD_FLYMAPLE 6
// HASH define HAL_BOARD_LINUX    7
// HASH define HAL_BOARD_EMPTY    99
*/

#define HAL_BOARD_GENERIC 1
#define HAL_BOARD_RPI     2
#define HAL_BOARD_STM32   3
#define HAL_BOARD_AVR     4

#if CONFIG_HAL_BOARD == HAL_BOARD_GENERIC
	#define AP_HAL_BOARD_DRIVER AP_HAL_Generic
	#define HAL_BOARD_NAME "generic"

#elif CONFIG_HAL_BOARD == HAL_BOARD_RPI
	#define AP_HAL_BOARD_DRIVER AP_HAL_RPI
	#define HAL_BOARD_NAME "rpi"

#elif CONFIG_HAL_BOARD == HAL_BOARD_STM32
	#define AP_HAL_BOARD_DRIVER AP_HAL_STM32
	#define HAL_BOARD_NAME "stm32"

#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR
	#define AP_HAL_BOARD_DRIVER AP_HAL_AVR
	#define HAL_BOARD_NAME "avr"

#else
	#error "Unknown CONFIG_HAL_BOARD type"
#endif


