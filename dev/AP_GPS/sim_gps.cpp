
#include <stdint.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

#include "sim_gps.h"

void AP_GPS_Sim::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_nav_setting = nav_setting;
	return;
}

bool AP_GPS_Sim::read(void)
{
	// Retrieve sensor readings from GZ interface
	// TODO Retrieve directly
	//hal.sim->update_gps_position(latitude, longitude, altitude_cm);
	//hal.sim->update_gps_velocities(_vel_north, _vel_east, _vel_down);

	// Get latitude and longitude readings
	double sim_lat = hal.sim->last_sensor_state.lat_deg;
	double sim_lon = hal.sim->last_sensor_state.lng_deg;

	latitude = (int32_t)(sim_lat*1e7);
	longitude = (int32_t)(sim_lon*1e7);

	altitude_cm = (int32_t)(hal.sim->last_sensor_state.alt_met*1e2);
	
	// Update intermediate variables
	num_sats = 10;
	hdop = 200;
	
	_last_gps_time = hal.scheduler->millis();

	/* NOTE Wrong place to to do this but will fix other issues */
	fix = Fix_Status::FIX_3D; 

    _have_raw_velocity = true;

	return true;
}


