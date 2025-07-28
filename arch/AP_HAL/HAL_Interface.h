
#if CONFIG_HAL_BOARD == HAL_BOARD_GENERIC
	#include <arch/linux/generic/AP_HAL_Generic.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_RPI
	#include <arch/linux/rpi/AP_HAL_RPI.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_STM32
	#include <arch/linux/rpi/AP_HAL_STM32.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR
	#include <arch/avr/AP_HAL_AVR/AP_HAL_AVR.h>
#else
	#error "HAL_Interface: CONFIG_HAL_BOARD not found"
#endif


