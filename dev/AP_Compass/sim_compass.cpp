

#include <AP_Math.h>
#include <AP_HAL.h>

#include "sim_compass.h"

#include "simulation_logger.h"
extern SimulationLogger simlog; 

#include "gz_interface.h"
extern GZ_Interface gz_interface;

extern const AP_HAL::HAL& hal;

void AP_Compass_Sim::accumulate(void)
{
	// Called at 50z
	// TODO This should be calling the gz_interface to retrieve magnetometer values
	
	/* Read latest field into temporary vector */
	Vector3f temp_field;
	gz_interface.get_compass_field(temp_field);

	// Add to the accumulated field
	acc_field += temp_field;
	acc_samples += 1;

	return;
}

bool AP_Compass_Sim::init()
{
	// perform an initial read
	_healthy = true;

	read();

  	return true;
}

bool AP_Compass_Sim::read()
{
	// Called at 10Hz
	// This should be calculating an average of the accumulate values
	
	if (acc_samples==0) {
		accumulate();
	}

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
		return false;
	}

	simlog.write_compass_state(
			_field.x,
			_field.y,
			_field.z);

    return true;
}


