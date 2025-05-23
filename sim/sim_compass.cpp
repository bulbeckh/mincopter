

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

	gz_interface.get_compass_field(_field[0]);

	simlog.write_compass_state(_field[0].x, _field[0].y, _field[0].z);

    return true;
}


