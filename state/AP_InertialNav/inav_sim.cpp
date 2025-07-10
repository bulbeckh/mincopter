
#include "inav_sim.h"


#ifdef MC_GZ_INTERFACE
#include "gz_interface.h"
extern GZ_Interface gz_interface;
#endif

#ifdef MC_SIMLOG
#include "simulation_logger.h"
extern SimulationLogger simlog;
#endif

void MC_InertialNav_Sim::init()
{
	return;
}

void MC_InertialNav_Sim::update(float dt)
{
	/* Normally this would use integrations of the accelerometer readings and 
	 * then correct with the GPS and baro but in simulation, we have these
	 * measurements directly */

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

#ifdef MC_SIMLOG
	simlog.write_inav_state(inav_pos, inav_vel);
#endif

	return;
}

void MC_InertialNav_Sim::set_altitude(float new_alt)
{
	inav_alt = new_alt;
	return;
}

void MC_InertialNav_Sim::set_home_position(int32_t lat, int32_t lng)
{
	inav_lat = lat;
	inav_lng = lng;
	return;
}

