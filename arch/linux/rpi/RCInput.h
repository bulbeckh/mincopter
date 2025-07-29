
#pragma once

#include <arch/linux/rpi/AP_HAL_RPI.h>

class RPI::RPIRCInput : public AP_HAL::RCInput {
	public:
		RPIRCInput();

		void init(void* machtnichts);

		uint8_t  valid_channels();

		uint16_t read(uint8_t ch);

		uint8_t read(uint16_t* periods, uint8_t len);

		bool set_overrides(int16_t *overrides, uint8_t len);

		bool set_override(uint8_t channel, int16_t override);

		void clear_overrides();
};


