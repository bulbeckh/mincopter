
#include <AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(Compass& _compass)
{
	// Initialise Compass
	_compass.init();

	/* Test 1. Read value */
	for (int i=0;i<8;i++) {
		bool status = _compass.read();

		if (!status) {
			hal.console->printf("Error in compass read\n");
			return 1;
		}

		Vector3f field = _compass.get_field();

		hal.console->printf("X: %f, Y: %f, Z: %f\n", field.x, field.y, field.z);

		// Delay 10ms
		hal.scheduler->delay(10);
	}

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
#if defined(MC_COMP_ICM20948) || defined(MC_TEST_COMPASS_ALL)
	AP_Compass_ICM20948 compass;
	run_unit_tests(compass);
#elif defined(MC_COMP_HMC5843) || defined(MC_TEST_COMPASS_ALL)
	AP_Compass_HMC5843 compass;
	run_unit_tests(compass);
#endif
	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

