
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include "DataFlash.h"

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(DataFlash_Class& df)
{
	// TODO


	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	// TODO NOTE We would never have an instance where a hardware target has all types of IMUs should maybe
	// remove the MC_TEST_IMU_ALL flag/option

#ifdef MC_STORAGE_FILE
	// No default path
	DataFlash_File df("/tmp/mincopter/test-log-out.txt");
#elif MC_STORAGE_APM2
	DataFlash_APM2 df;
#elif MC_STORAGE_EMPTY
	DataFlash_Empty df;
#endif

	// Run tests
	run_unit_tests(df);

	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

