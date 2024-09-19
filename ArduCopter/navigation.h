#pragma once

#include <AP_Common.h>
#include <stdint.h>

#include <AP_Math.h>

#include "config.h"
#include "defines.h"

#include "parameters.h"
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <AP_InertialNav.h>
#include <GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Compass.h>
#include <AC_Fence.h>
#include <AC_WPNav.h>

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"

extern uint8_t prev_nav_index;
extern uint8_t command_cond_index;

extern float lon_error;
extern float lat_error;

extern int32_t wp_bearing;
extern int32_t original_wp_bearing;
extern int32_t home_bearing;
extern int32_t home_distance;
extern uint32_t wp_distance;
extern uint8_t nav_mode;

// system.cpp
bool set_mode(uint8_t mode);

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

extern uint16_t loiter_time_max;
extern uint32_t loiter_time;

// navigation.h

/* @brief Prepare to run autopilot which makes decisions about which navigation control mode to use.
*/
void run_nav_updates(void);

// NOTE This function now doesn't do anything?
/* @brief Uses the current control_mode to set the parameters required for autopilot. Actual autopilot controller is in update_nav_mode
*/
void run_autopilot();

/* @brief Gets latitude and longitude from inertial nav
*/
void calc_position();

/* @brief Calculates distance and bearing to waypoint. Sets wp_distance and wp_bearing
*/
void calc_distance_and_bearing();

/* @brief Sets the navigation mode. Sub-function of set_mode
* @param new_nav_mode The new navigation mode. This is NAV_NONE for the manual modes.
*/
bool set_nav_mode(uint8_t new_nav_mode);

/* @brief Runs navigation controller. Called by scheduler
*/
void update_nav_mode();

/* @brief Zeroes-out wp_bearing, wp_location, lat, and long
*/
void reset_nav_params(void);

// NOTE why are these here
/* @brief Reduces rate-of-change of yaw to a maximum value
* @param current_yaw The current yaw value. Usually control_yaw
* @param desired_yaw The target yaw value.
* @param deg_per_sec The maximum rate-of-change of yaw value. For example AUTO_YAW_SLEW_RATE
*/
int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec);

