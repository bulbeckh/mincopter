
// Motor (simulation) test

#include <iostream>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

int main(void)
{

	AP_HAL_STM32.init(0, NULL);

	volatile uint32_t counter=0;

	while(true) {
		counter +=1;
	}

	return 0;

}


