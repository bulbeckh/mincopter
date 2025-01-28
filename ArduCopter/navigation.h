#pragma once

#include <stdint.h>

/*
HASH include <AP_Common.h>
HASH include <AP_Math.h>
HASH include "config.h"
HASH include "defines.h"
HASH include "parameters.h"
HASH include <AP_AHRS.h>
HASH include <AP_Motors.h>
HASH include <AP_InertialNav.h>
HASH include <GPS.h>
HASH include <AP_GPS_Glitch.h>
HASH include <AP_Compass.h>
HASH include <AC_Fence.h>
HASH include <AC_WPNav.h>
HASH include "navigation.h"
HASH include "util.h"
HASH include "motors.h"
HASH include "failsafe.h"
*/

// system.cpp
bool set_mode(uint8_t mode);

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

