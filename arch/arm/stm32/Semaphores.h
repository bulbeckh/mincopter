
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

class stm32::STM32Semaphore : public AP_HAL::Semaphore {

	public:
		STM32Semaphore(void);

		/* @brief Unlock the semaphore
		 * @returns True if we successfully unlocked the semaphore and false otherwise */
		bool give(void) override;

		/* @brief Take the resource and block until we are able to obtain it
		 * @returns True if we locked the resource and false if we exceeded the timeout */
		bool take(uint32_t timeout_ms) override;

		/* @brief Attempt to lock the semaphore once
		 * @returns True if we successfully locked and false otherwise */
		bool take_nonblocking(void) override;

	private:

		/* @brief Semaphore that determines if we can use the resource */
		volatile bool _taken;
};

