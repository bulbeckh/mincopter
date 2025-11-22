
#pragma once

#include <AP_HAL.h>
#include "avrdx/AP_HAL_AVRDx_Namespace.h"

#define AVR_RC_INPUT_NUM_CHANNELS 8
#define AVR_RC_INPUT_MIN_CHANNELS 5     // for ppm sum we allow less than 8 channels to make up a valid packet

class AP_HAL_AVRDx::AVRDxRCInput : public AP_HAL::RCInput {
	public:
		/* Pass in a AP_HAL_AVR::ISRRegistry* as void*. */
		void     init(void* isrregistry);
		uint8_t  valid_channels();
		uint16_t read(uint8_t ch);
		uint8_t  read(uint16_t* periods, uint8_t len);
		bool set_overrides(int16_t *overrides, uint8_t len);
		bool set_override(uint8_t channel, int16_t override);
		void clear_overrides();
};


