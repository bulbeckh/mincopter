
#include <stdint.h>
#include <AP_HAL.h>

// NOTE normally these includes should be guarded by HASH ifdef TARGET_ARCH_LINUX but this file should
// only be included from simulated backend (TARGET_ARCH_LINUX)
#include <iostream>

extern const AP_HAL::HAL& hal;

#include "sim_gps.h"

extern const AP_HAL::HAL& hal;


void
AP_GPS_Sim::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_nav_setting = nav_setting;
	return;
}

bool
AP_GPS_Sim::read(void)
{
	num_sats = 10;

	hdop = 200;
	
	_last_gps_time = hal.scheduler->millis();

    _have_raw_velocity = true;

	return true;
}

void AP_GPS_Sim::set_gps_vel3d(double vel_east, double vel_north, double vel_up)
{
	/* Velocity arguments from gz sim are in m/s but _vel_* are in cm/s and stored as integers */
	_vel_east = (int32_t)(vel_east*100.0f);
	_vel_north = (int32_t)(vel_north*100.0f);
	_vel_down = (int32_t)(-100*vel_up);

	/* NOTE Wrong place to to do this but will fix other issues */
	fix = Fix_Status::FIX_3D; 

	return;
}

void AP_GPS_Sim::set_gps_attitude(double lat, double lng, double alt)
{
	/* Simulation latitude and longitude are in degrees but we stored lat/lng in degrees*10,000,000 */
	latitude = (int32_t)(1e7*lat);
	longitude = (int32_t)(1e7*lng);

	/* Simulation altitude is in m but we store in cm */
	altitude_cm = (int32_t)(100*alt);

	return;
}



