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

/*********************************
 MANUAL mode - cascaded PID approach
**********************************

Roll/Pitch
-> Stick value
	-> Calculate desired lean angle from radio in
		-> Calculate error in lean angle between desired and current
			-> Convert to a target rate using P controller (e.g. g.pi_stabilize_roll)
				-> Run rate PID controllers to drive rate to target
					-> Pass control signal to g.rc<1,2,4>

Yaw
-> Stick value
	->

Throttle
-> Throttle stick value
	-> Calculate desired throttle from radio in
		-> Pass desired throttle value to radio out (after potentially applying angle boost

Each rate controller is run at 100hz
The update of the lean angle to target rate is also called at 100hz??

**********************************
RTL/AUTO mode - xx
**********************************
Use WP_NAV here.

set_roll_pitch_mode(RTL_RP);
set_throttle_mode(RTL_THR);
set_yaw_mode(YAW_HOLD);



*********************************/

/* @brief Transform pilot's roll or pitch input into a desired lean angle
* @param roll_in input roll fed in from xx
* @param pitch_in input pitch fed in from xx
* @param roll_out roll angle in centi-degrees
* @param pitch_out pitch angle in centi-degrees
*/
void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out);

/* @brief Convert input throttle stick value into a throttle_control_signal to be fed into throttle controller
* @param throttle_control The throttle stick value from RC
* @returns The control signal to be fed into throttle controller
*/
int16_t get_pilot_desired_throttle(int16_t throttle_control);

/* @brief Mode-specific functions which convert the error between current and desired lean angle into a rate target and pass to set_<roll/pitch/yaw>_rate_target. Uses g.pi_stabilize_roll PID controller.
* @param target_angle The target angles obtained from get_pilot_desired_lean_angles
*/
void get_stabilize_roll(int32_t target_angle);
void get_stabilize_pitch(int32_t target_angle);
void get_stabilize_yaw(int32_t target_angle);

/* @brief Run PID controllers during run_rate_controllers
* @param a target rate for the controller
* @param a control signal which is send directly to g.rc_<1,2,4>.servo_out. This is then used by motors
*/
int16_t get_rate_roll(int32_t target_rate);
int16_t get_rate_pitch(int32_t target_rate);
int16_t get_rate_yaw(int32_t target_rate);


void get_throttle_rate(float z_target_speed);

/* @brief Sets the g.rc_3 (throttle channel) output based on the (angle boosted) throttle control signal during run_rate_controllers
* @param throttle_out The output of the throttle controller from get_throttle_accel
* @param apply_angle_boost Whether or not to compensate throttle value for roll/pitch
*/
void set_throttle_out(int16_t throttle_out, bool apply_angle_boost);

/* @brief Throttle PID acceleration controller
* @param z_target_accel The target acceleration value, computed by throttle controller (get_throttle_rate)
* @returns The throttle control signal to be passed to g.rc_3
*/
int16_t get_throttle_accel(int16_t z_target_accel);



void get_look_at_yaw();
void get_look_ahead_yaw(int16_t pilot_yaw);

void update_throttle_cruise(int16_t throttle);


/* @brief Used to compensate throttle value for roll/pitch
* @param throttle a throttle control signal from throttle controller
* @returns the compensated throttle control signal
*/
int16_t get_angle_boost(int16_t throttle);



// NOTE This should be removed
void set_throttle_accel_target( int16_t desired_acceleration );

// NOTE Probably remove this or inline it
/* @brief Turns off acceleration-based throttle control
*/
void throttle_accel_deactivate();



// used by AUTO mode
int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms);
void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);

// used during LAND/AUTO modes
void get_throttle_rate_stabilized(int16_t target_rate);
void get_throttle_land();

// NOTE why does this return bool? Return val not used during throttle loop
/* @brief Runs land detection during throttle loop
* @returns true if landed 
*/
bool update_land_detector();


/* @brief Resets parameters
*/
void reset_land_detector();
void reset_I_all(void);
void reset_rate_I();
void reset_throttle_I(void);
void reset_roll_pitch_in_filters(int16_t roll_in, int16_t pitch_in);

/* @brief Converts earth frame targets to body frame targets using DCM matrix. Called during fast loop
*/
void update_rate_controller_targets();

/* @brief Runs the roll/pitch/yaw/throttle controllers. Called during fast loop
*/
void run_rate_controllers();

// NOTE used during auto mode
void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle);

// NOTE These can almost definitely be removed
/* @brief Set Roll/Pitch/Yaw rate targets in the desired frame
*/
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );

// From arducopter.cpp
extern float roll_in_filtered;     // roll-in in filtered with RC_FEEL_RP parameter
extern float pitch_in_filtered;    // pitch-in filtered with RC_FEEL_RP parameter

