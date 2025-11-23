
#pragma once

#include <AP_HAL.h>

#include <avrdx/AP_HAL_AVRDx.h>
#include "avrdx/AP_HAL_AVRDx_Namespace.h"

/*
using namespace AP_HAL;
using namespace AP_HAL_AVRDx;
*/

class HAL_AVRDx : public AP_HAL::HAL {

	public:
		HAL_AVRDx(void);

		void init(int argc, char * const argv[]) const;
};

// NOTE We have added the word 'instance' to the end here to prevent naming
// conflicts with the namespace (also named AP_HAL_AVRDx). There is probably
// a cleaner solution somewhere
extern const HAL_AVRDx AP_HAL_AVRDxInstance;


