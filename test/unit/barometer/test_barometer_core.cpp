
#include <AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(AP_Baro& _barometer)
{
	// Initialise barometer
	bool init_status = _barometer.init();

	if (!init_status) {
		hal.console->printf("Error in baro initialisation\n");
		return 1;
	}

	/* Test 1. Read value */
	for (int i=0;i<8;i++) {
		uint8_t status = _barometer.read();

		if (status!=0) {
			hal.console->printf("Error in barometer read\n");
			return 1;
		}

		float _temp = _barometer.get_temperature();
		float _pressure = _barometer.get_pressure();

		hal.console->printf("T %f P %f\n", _temp, _pressure);

		// Delay 10ms
		hal.scheduler->delay(10);
	}

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
#if defined(MC_BARO_MS5611) || defined(MC_TEST_BARO_ALL)
	AP_Baro_MS5611 barometer_instance(&AP_Baro_MS5611::spi);
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

