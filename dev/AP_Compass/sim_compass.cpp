
#include <AP_Math.h>
#include <AP_HAL.h>

#include "sim_compass.h"

extern const AP_HAL::HAL& hal;

void AP_Compass_Sim::accumulate(void)
{
	// Called at 50z
	// TODO This should be calling the gz_interface to retrieve magnetometer values
	
	// Read latest field into temporary vector
	// TODO Downcast from double to float
	Vector3f temp_field(
			hal.sim->last_sensor_state.field_x,
			hal.sim->last_sensor_state.field_y,
			hal.sim->last_sensor_state.field_z
			);

	/* Currently, our Gazebo sim is setup so that our compass and imu are rigidly attached to body frame 
	 * which at the simulation start has x point north, y point left (west) and z up. We negate y and z
	 * to rotate into NED frame */

	// Rotate into NED frame
	temp_field.y *= -1;
	temp_field.z *= -1;

	// Add to the accumulated field
	acc_field += temp_field;
	acc_samples += 1;

	return;
}

bool AP_Compass_Sim::init(void)
{
	// perform an initial read
	_healthy = true;

	read();

  	return true;
}

bool AP_Compass_Sim::read(void)
{
	// Called at 10Hz
	// This should be calculating an average of the accumulate values
	
	if (acc_samples==0) {
		accumulate();
	}

	// TODO Change to simulation timing
	last_update = hal.scheduler->micros();

	if (acc_samples>0) {
		// Calculate an average field reading
		acc_field.x /= acc_samples;
		acc_field.y /= acc_samples;
		acc_field.z /= acc_samples;

		// Correct for magnetic field here - _field should be in NED frame now
		_field.x = acc_field.x;
		_field.y = acc_field.y;
		_field.z = acc_field.z;

		// Reset accumulated variables
		acc_field.zero();
		acc_samples = 0;
	} else {
		// TODO Report error with compass read/accumulate
		return false;
	}

    return true;
}


