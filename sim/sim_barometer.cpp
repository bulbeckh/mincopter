
#include <AP_HAL.h>

#include "sim_barometer.h"

extern const AP_HAL::HAL& hal;

bool AP_Baro_Sim::init()
{
    healthy = true;
    return true;
}

uint8_t AP_Baro_Sim::read()
{
		// Update the baro

    return 1;
}

float AP_Baro_Sim::get_pressure()
{
    return Press;
}

float AP_Baro_Sim::get_temperature()
{
    // temperature in degrees C units
    return Temp;
}

