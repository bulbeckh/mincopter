
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
	Press=1.0;

	_last_update = hal.scheduler->millis();

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

void AP_Baro_Sim::set_pressure(double gz_pressure)
{
	Press = (float)gz_pressure;
}

