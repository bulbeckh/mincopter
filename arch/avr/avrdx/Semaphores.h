
#pragma once

#include <AP_HAL.h>
#include <avrdx/AP_HAL_AVRDx_Namespace.h>

class AP_HAL_AVRDx::AVRDxSemaphore : public AP_HAL::Semaphore {
	public:
		AVRDxSemaphore();

		bool give();
		bool take(uint32_t timeout_ms);
		bool take_nonblocking();

	protected:
		bool _take_from_mainloop(uint32_t timeout_ms);
		bool _take_nonblocking();

		volatile bool _taken;
};

