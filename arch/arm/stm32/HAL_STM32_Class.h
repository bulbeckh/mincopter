
#pragma once

#include <AP_HAL/AP_HAL.h>

class HAL_STM32 : public AP_HAL::HAL {
	public:
		HAL_STM32();

		void init(int argc, char * const * argv) const;
};

extern const HAL_STM32 AP_HAL_STM32;



