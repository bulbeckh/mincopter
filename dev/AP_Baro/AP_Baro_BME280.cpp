
#include <inttypes.h>

#include <AP_Common.h>
#include <AP_Math.h>

#include <AP_HAL.h>
#include "AP_Baro_BME280.h"

extern const AP_HAL::HAL& hal;

bool AP_Baro_BME280::init()
{
	/* Implement */

	healthy = true;

    return true;
}

void AP_Baro_BME280::accumulate(void)
{
	/* Implement */
}

uint8_t AP_Baro_BME280::read()
{
	/* Implement */

    return 0;
}

float AP_Baro_BME280::get_pressure() {
    //return Press;
	return 1000.0f;
}

float AP_Baro_BME280::get_temperature() {
    //return Temp;
	return 20.0f;
}

