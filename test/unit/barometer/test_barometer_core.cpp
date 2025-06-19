
#include <AP_Baro.h>

int main()
{
	// Core setup before actual testing
	
#if defined(MC_TEST_BARO_xxclass) || defined(MC_TEST_BARO_ALL)
	BARO_CLASS barometer_instance;
	run_unit_tests(barometer_instance);
#elif defined(MC_TEST_BARO_xxclass) || defined(MC_TEST_BARO_ALL)
	BARO_CLASS barometer_instance;

	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

void run_unit_tests(
