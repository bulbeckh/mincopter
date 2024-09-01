/* GLOBALS */

// float roll_in_filtered;     // roll-in in filtered with RC_FEEL_RP parameter
// float pitch_in_filtered;    // pitch-in filtered with RC_FEEL_RP parameter
#pragma once

#include <stdint.h>

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

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"

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

void set_target_alt_for_reporting(float alt_cm);

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

void reset_roll_pitch_in_filters(int16_t roll_in, int16_t pitch_in);
void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out);

void get_stabilize_roll(int32_t target_angle);
void get_stabilize_pitch(int32_t target_angle);
void get_stabilize_yaw(int32_t target_angle);

void get_acro_level_rates();

void get_roll_rate_stabilized_bf(int32_t stick_angle);
void get_pitch_rate_stabilized_bf(int32_t stick_angle);
void get_yaw_rate_stabilized_bf(int32_t stick_angle);

void get_roll_rate_stabilized_ef(int32_t stick_angle);
void get_pitch_rate_stabilized_ef(int32_t stick_angle);
void get_yaw_rate_stabilized_ef(int32_t stick_angle);

int16_t get_rate_roll(int32_t target_rate);
int16_t get_rate_pitch(int32_t target_rate);
int16_t get_rate_yaw(int32_t target_rate);

// optflow - remove
int32_t get_of_roll(int32_t input_roll);
int32_t get_of_pitch(int32_t input_pitch);

void get_circle_yaw();
void get_look_at_yaw();

void get_look_ahead_yaw(int16_t pilot_yaw);
void update_throttle_cruise(int16_t throttle);

// heli version also
int16_t get_angle_boost(int16_t throttle);

// throttle ctrl
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost );
void set_throttle_accel_target( int16_t desired_acceleration );
void throttle_accel_deactivate();

void set_throttle_takeoff();
int16_t get_throttle_accel(int16_t z_target_accel);
int16_t get_pilot_desired_throttle(int16_t throttle_control);
int16_t get_pilot_desired_climb_rate(int16_t throttle_control);

int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms);

void get_throttle_rate(float z_target_speed);
void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
void get_throttle_rate_stabilized(int16_t target_rate);
void get_throttle_land();

// land ctrl
void reset_land_detector();
bool update_land_detector();
void get_throttle_surface_tracking(int16_t target_rate);

// integrator resets
void reset_I_all(void);
void reset_rate_I();
void reset_optflow_I(void);
void reset_throttle_I(void);
void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle);

void update_rate_controller_targets();
void run_rate_controllers();

void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );


