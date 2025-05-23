
#include <AP_HAL.h>

#include "sim_barometer.h"

#include "gz_interface.h"
extern GZ_Interface gz_interface;

#include "simulation_logger.h"
extern SimulationLogger simlog;

#include "mcinstance.h"
extern MCInstance mincopter;

extern const AP_HAL::HAL& hal;

bool AP_Baro_Sim::init()
{
    healthy = true;
	
	/* Non-zero starting pressure so that the calibrate routine doesn't trigger an error.
	 * This is the simulated pressure as reported by the GZ sensor barometer. */
	pressure_pa = 101322.6;

    return true;
}

uint8_t AP_Baro_Sim::read()
{

	_last_update = hal.scheduler->millis();

	gz_interface.get_barometer_pressure(pressure_pa);

	temperature_degc = 26;

	// Log read value
	float baro_alt = mincopter.barometer.get_altitude();
	simlog.write_barometer_state(temperature_degc, pressure_pa, baro_alt);

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

