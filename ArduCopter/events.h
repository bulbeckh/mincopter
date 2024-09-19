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

#include <AP_ServoRelayEvents.h>

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"
#include "system.h"

// events.h

/* @brief Called when radio loses connection, triggering the failsafe to kick-in
*/
void failsafe_radio_on_event();

/* @brief Called when returning from a failsafe mode
*/
void failsafe_radio_off_event();

/* @brief Called when a low battery occurs, triggering failsafe
*/
void failsafe_battery_event(void);

/* @brief Called when losing GPS signal
*/
void failsafe_gps_check();

/* @brief Called when GPS returns signal
*/
void failsafe_gps_off_event(void);

/* @brief Calls the update_events method of AP_ServoRelayEvents
*/
void update_events();

extern AP_ServoRelayEvents ServoRelayEvents;
extern int32_t home_distance;
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

// from arducopter.cpp
extern float cos_roll_x;
extern float cos_pitch_x;
extern float cos_yaw;
extern float sin_yaw;
extern float sin_roll;
extern float sin_pitch;
extern uint8_t rate_targets_frame;    // indicates whether rate targets provided in earth or body frame
extern int32_t roll_rate_target_ef;
extern int32_t pitch_rate_target_ef;
extern int32_t yaw_rate_target_ef;
extern int32_t roll_rate_target_bf;     // body frame roll rate target
extern int32_t pitch_rate_target_bf;    // body frame pitch rate target
extern int32_t yaw_rate_target_bf;      // body frame yaw rate target
extern int16_t throttle_accel_target_ef;    // earth frame throttle acceleration target
extern bool throttle_accel_controller_active;   // true when accel based throttle controller is act
extern float throttle_avg;                  // g.throttle_cruise as a float
extern int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
extern float target_alt_for_reporting;      // target altitude in cm for reporting (logs and ground station)

extern struct   Location home;
extern struct   Location current_loc;
extern struct   Location command_nav_queue;
extern struct   Location command_cond_queue;

extern int32_t control_yaw;
extern Vector3f yaw_look_at_WP;
extern int32_t yaw_look_at_WP_bearing;
extern int32_t yaw_look_at_heading;
extern int16_t yaw_look_at_heading_slew;

extern float controller_desired_alt;
extern int32_t altitude_error;
extern int16_t climb_rate;
extern int16_t sonar_alt;
extern uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
extern float target_sonar_alt;      // desired altitude in cm above the ground
extern int32_t baro_alt;

extern int16_t angle_boost;
extern uint16_t land_detector;

extern Vector3f circle_center;
extern float circle_angle;
extern float circle_angle_total;
extern uint8_t circle_desired_rotations;
extern float circle_angular_acceleration;       // circle mode's angular acceleration
extern float circle_angular_velocity;           // circle mode's angular velocity
extern float circle_angular_velocity_max;       // circle mode's max angular velocity
extern uint16_t loiter_time_max;
extern uint32_t loiter_time;
