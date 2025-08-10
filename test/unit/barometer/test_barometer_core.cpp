
#include <AP_Baro.h>
#include <AP_HAL/AP_HAL.h>

#include <stdio.h>

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
#if defined(MC_BARO_MS5611) || defined(MC_TEST_BARO_ALL)
	AP_Baro_MS5611 barometer_instance;
	run_unit_tests(barometer_instance);
#elif defined(MC_BARO_BME280) || defined(MC_TEST_BARO_ALL)
	AP_Baro_BME280 barometer_instance;
	run_unit_tests(barometer_instance);

	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(AP_Baro& _barometer)
{
	// Initialise barometer
	barometer_instance.init();

	/* Test 1. Read value */
	uint8_t status = barometer_instance.read();

	float _temp = barometer_instance.get_temperature();
	float _pressure = barometer_instance.get_pressure();

	printf("T %f P %f\n", _temp, _pressure);

	return 0;
}

