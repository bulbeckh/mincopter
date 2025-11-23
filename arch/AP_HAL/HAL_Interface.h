
#ifdef HAL_BOARD_GENERIC
	#include <arch/linux/generic/AP_HAL_Generic.h>
#elif HAL_BOARD_RPI
	#include <arch/linux/rpi/AP_HAL_RPI.h>
#elif HAL_BOARD_STM32
	#include <arch/arm/stm32/AP_HAL_STM32.h>
#elif HAL_BOARD_AVRMEGA
	#include <arch/avr/mega/AP_HAL_AVR.h>
#elif HAL_BOARD_AVRDX
	#include <arch/avr/avrdx/AP_HAL_AVRDx.h>
#else
	#error "HAL_Interface: CONFIG_HAL_BOARD not found"
#endif


