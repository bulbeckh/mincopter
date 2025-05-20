
#include <AP_HAL.h>

#include "sim_barometer.h"

#include "gz_interface.h"
extern GZ_Interface gz_interface;

extern const AP_HAL::HAL& hal;

bool AP_Baro_Sim::init()
{
    healthy = true;
	
	/* Non-zero starting pressure so that the calibrate routine doesn't trigger an error */
	pressure_pa = 1.0;

    return true;
}

uint8_t AP_Baro_Sim::read()
{

	_last_update = hal.scheduler->millis();

	gz_interface.get_barometer_pressure(pressure_pa);

	temperature_degc = 26;

    return 1;
}

float AP_Baro_Sim::get_pressure()
{
    return pressure_pa;
}

float AP_Baro_Sim::get_temperature()
{
    return temperature_degc;
}

