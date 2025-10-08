
#include <arch/arm/stm32/Semaphores.h>

extern const AP_HAL::HAL& hal;

using namespace stm32;

// TODO
STM32Semaphore::STM32Semaphore(void)
{
}

bool STM32Semaphore::give() 
{
	// TODO
	return false;
}

bool STM32Semaphore::take(uint32_t timeout_ms) 
{
	// TODO
    return false;
}

bool STM32Semaphore::take_nonblocking() 
{
	// TODO
	return false;
}

