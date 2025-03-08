

#include <AP_Math.h>
#include <AP_HAL.h>

#include "sim_compass.h"

extern const AP_HAL::HAL& hal;

// accumulate a reading from the magnetometer
void AP_Compass_Sim::accumulate(void)
{
	// Do nothing
	return;
}

// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_Sim::init()
{
    _initialised = true;

		// perform an initial read
		_healthy[0] = true;

		read();

  	return true;
}

// Read Sensor data
bool AP_Compass_Sim::read()
{
		// Update field values in a deterministic manner + error

		/*
		_field[0].x = _mag_x_accum * calibration[0] / _accum_count;
		_field[0].y = _mag_y_accum * calibration[1] / _accum_count;
		_field[0].z = _mag_z_accum * calibration[2] / _accum_count;
		*/

		_field[0].x = 0.0;
		_field[0].y = 0.0;
		_field[0].z = 0.0;

		_healthy[0] = true;

    return true;
}
