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

// navigation.h
void run_nav_updates(void);
void calc_position();
void calc_distance_and_bearing();
void run_autopilot();
bool set_nav_mode(uint8_t new_nav_mode);
void update_nav_mode();
void reset_nav_params(void);
int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec);
void circle_set_center(const Vector3f current_position, float heading_in_radians);
void update_circle();

extern Vector3f circle_center;
extern float circle_angle;
extern float circle_angle_total;
extern uint8_t circle_desired_rotations;
extern float circle_angular_acceleration;       // circle mode's angular acceleration
extern float circle_angular_velocity;           // circle mode's angular velocity
extern float circle_angular_velocity_max;       // circle mode's max angular velocity
extern uint16_t loiter_time_max;
extern uint32_t loiter_time;
