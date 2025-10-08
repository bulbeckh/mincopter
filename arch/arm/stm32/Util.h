
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

class stm32::STM32Util : public AP_HAL::Util {
	public:
		bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
};


