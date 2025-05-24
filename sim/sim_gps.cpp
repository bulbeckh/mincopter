
#include <stdint.h>
#include <AP_HAL.h>

// NOTE normally these includes should be guarded by HASH ifdef TARGET_ARCH_LINUX but this file should
// only be included from simulated backend (TARGET_ARCH_LINUX)
#include <iostream>

#include "gz_interface.h"
extern GZ_Interface gz_interface;

extern const AP_HAL::HAL& hal;

#include "sim_gps.h"


extern const AP_HAL::HAL& hal;

void AP_GPS_Sim::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_nav_setting = nav_setting;
	return;
}

bool AP_GPS_Sim::read(void)
{
	// Retrieve sensor readings from GZ interface
	gz_interface.update_gps_position(latitude, longitude, altitude_cm);
	gz_interface.update_gps_velocities(_vel_north, _vel_east, _vel_down);
	
	num_sats = 10;
	hdop = 200;
	
	_last_gps_time = hal.scheduler->millis();

	/* NOTE Wrong place to to do this but will fix other issues */
	fix = Fix_Status::FIX_3D; 

    _have_raw_velocity = true;

	return true;
}


