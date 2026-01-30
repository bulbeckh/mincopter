
#include <AP_HAL.h>

#include "sim_barometer.h"

#include "mcinstance.h"
extern MCInstance mincopter;

extern const AP_HAL::HAL& hal;

bool AP_Baro_Sim::init()
{
	/* Non-zero starting pressure so that the calibrate routine doesn't trigger an error.
	 * This is the simulated pressure as reported by the GZ sensor barometer. */
	pressure_pa = 101322.6;

    return true;
}

void AP_Baro_Sim::read(void)
{
	if (hal.sim->valid) {

		_last_update = hal.scheduler->millis();

		// NOTE Narrowing conversion from double to float
		// Get pressure reading from sim
		pressure_pa = hal.sim->last_sensor_state.pressure;

		temperature_degc = 26;

		healthy = true;
	}

    return;
}

float AP_Baro_Sim::get_pressure(void)
{
    return pressure_pa;
}

float AP_Baro_Sim::get_temperature(void)
{
    return temperature_degc;
}

