
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
	_step = 0;
	_new_position = false;
	_new_speed = false;
	return;
}

bool
AP_GPS_Sim::read(void)
{
    //static int gps_read_counter=0;
    //if (gps_read_counter%100==0) std::cout << "GPS read counter: " << gps_read_counter << "\n";

    uint8_t data;
    int16_t numc;
    bool parsed = true;

		// TODO Just move the function body here
		return _parse_gps();
}

// Private Methods /////////////////////////////////////////////////////////////

bool
AP_GPS_Sim::_parse_gps(void)
{
	/* This updates various members based on the message type. We can just update all directly each call to read */


	/*
  	longitude       = 0;
	latitude        = 0;
    altitude_cm     = 0;
    fix             = GPS::FIX_3D;
    _new_position = true;

    next_fix = GPS::FIX_3D;


    num_sats        = 0;
    hdop            = 0;
    _last_gps_time  = 0;
    time_week_ms    = 0;
    time_week       = 0;

  	speed_3d_cm     = 0;
    ground_speed_cm = 0;
    ground_course_cd = 0;

    _vel_north  = 0;
    _vel_east   = 0;
    _vel_down   = 0;
    _new_speed = true;
	*/

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



