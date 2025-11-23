
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

// TODO
/* The following statement contains the object name for each of the HAL instances. We only
 * really use this in mcinstance.h where we create a reference to the underlying hal object
 * so we can probably move this entire statement there. */

// TODO Can we just replace the equality with whether each definition is present
#ifdef HAL_BOARD_GENERIC
	#define AP_HAL_BOARD_DRIVER AP_HAL_Generic
#elif HAL_BOARD_RPI
	#define AP_HAL_BOARD_DRIVER AP_HAL_RPI
#elif HAL_BOARD_STM32
	#define AP_HAL_BOARD_DRIVER AP_HAL_STM32
#elif HAL_BOARD_AVRMEGA
	#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM2
#elif HAL_BOARD_AVRDX
	#define AP_HAL_BOARD_DRIVER AP_HAL_AVRDxInstance
#else
	#error "Unknown CONFIG_HAL_BOARD type"
#endif


