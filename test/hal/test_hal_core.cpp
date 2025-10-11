
// Motor (simulation) test

#include <iostream>

#include <AP_HAL/AP_HAL.h>

extern const HAL_STM32& AP_HAL_STM32;

int main(void)
{

	AP_HAL_STM32.init(0, NULL);

	volatile uint32_t counter=0;

	while(true) {
		counter +=1;
	}

	return 0;

}


