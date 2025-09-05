
#include <AP_GPS.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#ifdef TARGET_ARCH_AVR
	#define TEST_GPS_UART hal.uartB
#else
	#define TEST_GPS_UART hal.uartA
#endif

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(GPS& _gps)
{
	// TODO
	
	// Initialise GPS
	
	_gps.init(TEST_GPS_UART, GPS::GPS_ENGINE_AIRBORNE_1G);

	/* Run GPS read */
	while(true) {
		uint8_t status = _gps.status();

		hal.console->printf("[GPS Update @ 1Hz, status=%u, ram=%u]\n", status, hal.util->available_memory());

		// Update GPS
		_gps.update();


		// Delay 1s
		hal.scheduler->delay(1000);
	}

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	// Initialise correct UART for GPS
	
	if (TEST_GPS_UART != NULL) TEST_GPS_UART->begin(115200, 256, 16);
	
#if defined(MC_GPS_UBLOX) || defined(MC_TEST_BARO_ALL)
	AP_GPS_UBLOX gps_instance;
	run_unit_tests(gps_instance);
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

