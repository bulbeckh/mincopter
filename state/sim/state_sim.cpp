
#include <AP_Math.h>

#include "state_sim.h"

#include "mcinstance.h"
extern MCInstance mincopter;

extern AP_HAL::HAL& hal;

void StateSim::init_derived(void)
{
	// NOTE Nothing to initialise for simulated state
	return;
}

void StateSim::update(void)
{
	// Retrieve attitude quaternion from simulation euler angles
	data.attitude.from_euler(
			mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_x,
			mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_y,
			mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_z
		);

	// State struct maintains both quaternion and euler angle representations
	data.euler.x = mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_x;
	data.euler.y = mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_y;
	data.euler.z = mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_z;

	// Euler rates taken directly from sim
	data.euler_rates.x = mincopter.hal.sim->last_sensor_state.euler_rate_x;
	data.euler_rates.y = mincopter.hal.sim->last_sensor_state.euler_rate_y;
	data.euler_rates.z = mincopter.hal.sim->last_sensor_state.euler_rate_z;

	// Position taken directly from sim (already converted to NED in ap-gz plugin)
	data.position[0] = hal.sim->last_sensor_state.pos_x;
	data.position[1] = hal.sim->last_sensor_state.pos_y;
	data.position[2] = hal.sim->last_sensor_state.pos_z;

	// Velocity taken directly from sim (already converted to NED frame in ap-gz plugin)
	data.velocity[0] = hal.sim->last_sensor_state.vel_x;
	data.velocity[1] = hal.sim->last_sensor_state.vel_y;
	data.velocity[2] = hal.sim->last_sensor_state.vel_z;

	return;
}



