
#pragma once

#include <AP_HAL.h>
#include "avrdx/AP_HAL_AVRDx_Namespace.h"

#include "avrdx/memcheck.h"

class AP_HAL_AVRDx::AVRDxUtil : public AP_HAL::Util {
	public:
		bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
		uint16_t available_memory(void) { return memcheck_available_memory(); }
};

