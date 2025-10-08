
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

class stm32::STM32RCInput : public AP_HAL::RCInput {
	public:
		STM32RCInput();

		void init(void* machtnichts);

		uint8_t  valid_channels();

		uint16_t read(uint8_t ch);

		uint8_t read(uint16_t* periods, uint8_t len);

		bool set_overrides(int16_t *overrides, uint8_t len);

		bool set_override(uint8_t channel, int16_t override);

		void clear_overrides();
};

