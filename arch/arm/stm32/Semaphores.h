
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

class stm32::STM32Semaphore : public AP_HAL::Semaphore {
	public:
		STM32Semaphore();

		bool give(void) override;

		bool take(uint32_t timeout_ms) override;

		bool take_nonblocking(void) override;

	private:
		// pthread_mutex_t _lock;
};

