
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
		_field.x = acc_field.x / acc_samples;
		_field.y = acc_field.y / acc_samples;
		_field.z = acc_field.z / acc_samples;

		// Reset accumulated variables
		acc_field.zero();
		acc_samples = 0;
	} else {
		// TODO Report error with compass read/accumulate
		return false;
	}

    return true;
}


