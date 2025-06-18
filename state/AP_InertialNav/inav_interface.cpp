
#include "inav_interface.h"

#include "gz_interface.h"
extern GZ_Interface gz_interface;

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

	inav_pos = Vector3f(gz_interface.last_sensor_state.pos_x,
			gz_interface.last_sensor_state.pos_y,
			gz_interface.last_sensor_state.pos_z);

	inav_vel = Vector3f(gz_interface.last_sensor_state.vel_x,
			gz_interface.last_sensor_state.vel_y,
			gz_interface.last_sensor_state.vel_z);

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

