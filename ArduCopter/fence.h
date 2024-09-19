#pragma once

#include "parameters.h"
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <AP_InertialNav.h>
#include <GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Compass.h>
#include <AC_Fence.h>
#include <AC_WPNav.h>

#include <stdint.h>

#include "ap_union.h"
#include "config.h"
#include "defines.h"
#include "motors.h"
#include "log.h"
#include "system.h"

bool set_mode(uint8_t mode);

/* @brief Check for brief of fence. 
*/
void fence_check();

extern int32_t home_distance;
extern int8_t control_mode;

// Remove mavlink funcs
// void fence_send_mavlink_status(mavlink_channel_t chan);


extern Parameters g;
extern Vector3f omega;
extern AP_AHRS_DCM ahrs;
extern float G_Dt;
extern AP_MotorsQuad motors;
extern AP_InertialNav inertial_nav;
extern GPS         *g_gps;
extern GPS_Glitch   gps_glitch;
extern AP_Compass_HMC5843 compass;
extern AC_WPNav wp_nav;

extern AP_UNION_T ap;
extern AP_FAILSAFE_T failsafe;
extern AC_Fence fence;
