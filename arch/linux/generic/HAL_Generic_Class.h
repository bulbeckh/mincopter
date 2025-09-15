
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/linux/AP_HAL_Linux_Namespace.h>
#include <arch/linux/generic/AP_HAL_Generic_Namespace.h>

class HAL_Generic : public AP_HAL::HAL {
	public:
		HAL_Generic();

		void init(int argc, char * const * argv) const;
};

extern const HAL_Generic AP_HAL_Generic;



