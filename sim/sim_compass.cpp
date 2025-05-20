

#include <AP_Math.h>
#include <AP_HAL.h>

#include "sim_compass.h"

extern const AP_HAL::HAL& hal;

void AP_Compass_Sim::accumulate(void)
{
	// TODO 
	return;
}

bool AP_Compass_Sim::init()
{
	// perform an initial read
	_healthy[0] = true;

	read();

  	return true;
}

bool AP_Compass_Sim::read()
{
	last_update = hal.scheduler->micros();

	// TODO Change this to a call to the gz_interface to retrieve magnetometer readings

    return true;
}

void AP_Compass_Sim::set_field(double field_x, double field_y, double field_z)
{
	/* The simulated field readings are in Tesla (T) */

	// NOTE do we need to explicitly cast to float here?
	_field[0].x = field_x;
	_field[0].y = field_y;
	_field[0].z = field_z;

	return;
}



