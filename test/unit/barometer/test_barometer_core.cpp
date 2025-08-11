
#include <AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <stdio.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(AP_Baro& _barometer)
{
	// Initialise barometer
	_barometer.init();

	/* Test 1. Read value */
	uint8_t status = _barometer.read();

	float _temp = _barometer.get_temperature();
	float _pressure = _barometer.get_pressure();

	printf("T %f P %f\n", _temp, _pressure);

	return 0;
}

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

