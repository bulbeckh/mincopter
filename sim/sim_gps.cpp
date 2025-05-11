
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
    _have_raw_velocity = true;
    _vel_north  = 0;
    _vel_east   = 0;
    _vel_down   = 0;
    _new_speed = true;

    return true;
}


