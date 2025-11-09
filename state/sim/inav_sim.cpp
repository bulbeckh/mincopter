
#include <AP_HAL.h>

#include "inav_sim.h"

extern const AP_HAL::HAL& hal;

void MC_InertialNav_Sim::_inav_init_internal(void)
{
	return;
}

void MC_InertialNav_Sim::inav_update(void)
{
	// TODO Need to update to match new interface (i.e update _state._position and _state._velocity)
	
	// TODO Possible downcast here from double to float
	_inav_state->_position[0] = hal.sim->last_sensor_state.pos_x;
	_inav_state->_position[1] = hal.sim->last_sensor_state.pos_y;
	_inav_state->_position[2] = hal.sim->last_sensor_state.pos_z;

	_inav_state->_velocity[0] = hal.sim->last_sensor_state.vel_x;
	_inav_state->_velocity[1] = hal.sim->last_sensor_state.vel_y;
	_inav_state->_velocity[2] = hal.sim->last_sensor_state.vel_z;

	/* Normally this would use integrations of the accelerometer readings and 
	 * then correct with the GPS and baro but in simulation, we have these
	 * measurements directly */

	/*
	inav_lat = (int32_t)(gz_interface.last_sensor_state.lat_deg*1e7);
	inav_lng = (int32_t)(gz_interface.last_sensor_state.lng_deg*1e7);

	inav_alt = (int32_t)(100*gz_interface.last_sensor_state.alt_met);

	// Update position in cm
	inav_pos.x = 100.0f*gz_interface.last_sensor_state.pos_x;
	inav_pos.y = 100.0f*gz_interface.last_sensor_state.pos_y;
	inav_pos.z = 100.0f*gz_interface.last_sensor_state.pos_z;

	// Update velocity in cm/s
	inav_vel.x = 100.0f*gz_interface.last_sensor_state.vel_x;
	inav_vel.y = 100.0f*gz_interface.last_sensor_state.vel_y;
	inav_vel.z = 100.0f*gz_interface.last_sensor_state.vel_z;
	*/

	return;
}

