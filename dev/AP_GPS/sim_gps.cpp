
#include <stdint.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

#include "sim_gps.h"

void AP_GPS_Sim::init(AP_HAL::UARTDriver* /* unused */, enum GPS_Engine_Setting nav_setting)
{
	_nav_setting = nav_setting;
	return;
}

bool AP_GPS_Sim::read(void)
{
	// Update our GPS data if we have a valid read
	if (hal.sim->valid) {

		// Get latitude and longitude readings
		double sim_lat = hal.sim->last_sensor_state.lat_deg;
		double sim_lon = hal.sim->last_sensor_state.lng_deg;

		latitude = (int32_t)(sim_lat*1e7);
		longitude = (int32_t)(sim_lon*1e7);
		altitude_cm = (int32_t)(hal.sim->last_sensor_state.alt_met*1e2);

		// Velocities in cm/s from GPS
		_vel_north = (int32_t)(hal.sim->last_sensor_state.vel_north*1e2);
		_vel_east = (int32_t)(hal.sim->last_sensor_state.vel_east*1e2);
		_vel_down = (int32_t)(-1e2*hal.sim->last_sensor_state.vel_up);
		
		// Update intermediate variables
		num_sats = 10;
		hdop = 200;
		
		_last_gps_time = hal.scheduler->millis();

		/* NOTE Wrong place to to do this but will fix other issues */
		fix = Fix_Status::FIX_3D; 

		_have_raw_velocity = true;
	}

	return true;
}


