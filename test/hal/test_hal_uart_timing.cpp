
#include <AP_InertialSensor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(void)
{

	// Run x iterations of hal prints and measure timing
	
	uint32_t avg_runtime = 0;

	// Average printing time over 100 iterations
	for (uint16_t j=0;j<100;j++) {
		uint32_t start = hal.scheduler->micros();
		hal.console->printf("xxxx");
		avg_runtime += hal.scheduler->micros() - start;
	}

	avg_runtime /= 100;
	hal.console->printf("Average Runtime:%f\r\n", avg_runtime);
	
	// TODO Perhaps instead change to log the test results

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	// TODO NOTE We would never have an instance where a hardware target has all types of IMUs should maybe
	// remove the MC_TEST_IMU_ALL flag/option

	run_unit_tests();

	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

