
#include <arch/arm/stm32/Semaphores.h>

extern const AP_HAL::HAL& hal;

using namespace stm32;

// TODO The AVR Semaphore implementation has checks to see if we call semaphore functions from the timer process
// as ::take will block

STM32Semaphore::STM32Semaphore(void)
	: _taken{false}
{ }

bool STM32Semaphore::give(void)
{
	if (!_taken) {
		return false;
	} else {
		_taken = false;
		return true;
	}
}

bool STM32Semaphore::take(uint32_t timeout_ms) 
{
	uint32_t start = hal.scheduler->millis();

	while (hal.scheduler->millis()-start < timeout_ms) {
		if (!_taken) {
			_taken = true;
			return true;
		}
	}

	// If we exceed timeout, return false
    return false;
}

bool STM32Semaphore::take_nonblocking(void)
{
	if (!_taken) {
		_taken = true;
		return true;
	} else {
		return false;
	}
}

