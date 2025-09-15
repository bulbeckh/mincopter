
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
	
	num_sats = 10;
	hdop = 200;
	
	_last_gps_time = hal.scheduler->millis();

	/* NOTE Wrong place to to do this but will fix other issues */
	fix = Fix_Status::FIX_3D; 

    _have_raw_velocity = true;

	return true;
}


