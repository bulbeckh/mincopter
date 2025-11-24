
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

/* This is a script for a test of the motors/ESCs **via the RCOutput** interface and **not** via the AP_Motors abstraction.
 *
 * We run a simple test that pulses each motor up and down over a certain period of time.
 *
 */


// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(void)
{

	// Init

	// Run this loop at 100Hz
	for (int i=0;i<1e6;i++) {
		uint32_t start = hal.scheduler->micros();




		// Delay
		uint32_t elapsed = hal.scheduler->micros() - start;
		//if (elapsed < 10000) hal.scheduler->delay(10000-elapsed);
		// TODO Confirm that in simulation, the delay is actually working correctly
	}

	return 0;
}

int main(void)
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	run_unit_tests();

}

