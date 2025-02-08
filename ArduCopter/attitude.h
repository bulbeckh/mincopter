/* GLOBALS */

#pragma once

#include <stdint.h>



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

/* @brief Converts earth frame targets to body frame targets using DCM matrix. Called during fast loop
*/
void update_rate_controller_targets();

/* Part 1. Setting targets */

/* @brief Mode-specific functions which convert the error between current and desired lean angle into a rate target and pass to set_<roll/pitch/yaw>_rate_target. Uses g.pi_stabilize_roll PID controller.
* @param target_angle The target angles obtained from get_pilot_desired_lean_angles
*/
void get_stabilize_roll(int32_t target_angle);
void get_stabilize_pitch(int32_t target_angle);
void get_stabilize_yaw(int32_t target_angle);

// NOTE These can almost definitely be removed
/* @brief Set Roll/Pitch/Yaw rate targets in the desired frame
*/
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );


void get_throttle_rate(float z_target_speed);


void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);

// NOTE This should be removed
void set_throttle_accel_target( int16_t desired_acceleration );


/* Part 2. Running rate/accel controller */


/* @brief Runs the roll/pitch/yaw/throttle controllers. Called during fast loop
*/
void run_rate_controllers();

/* @brief Run PID controllers during run_rate_controllers
* @param a target rate for the controller
* @param a control signal which is send directly to g.rc_<1,2,4>.servo_out. This is then used by motors
*/
int16_t get_rate_roll(int32_t target_rate);
int16_t get_rate_pitch(int32_t target_rate);
int16_t get_rate_yaw(int32_t target_rate);

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

/* @brief Used to compensate throttle value for roll/pitch
* @param throttle a throttle control signal from throttle controller
* @returns the compensated throttle control signal
*/
int16_t get_angle_boost(int16_t throttle);

/* NOTE not part of primary rate controllers but used during throttle-land mode */
void get_throttle_rate_stabilized(int16_t target_rate);
void get_throttle_land();


/* OTHER FUNCTIONS
*  The yaw functions can likely be removed
*/


/* @brief Reduces rate-of-change of yaw to a maximum value
* @param current_yaw The current yaw value. Usually control_yaw
* @param desired_yaw The target yaw value.
* @param deg_per_sec The maximum rate-of-change of yaw value. For example AUTO_YAW_SLEW_RATE
*/
int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec);
void get_look_at_yaw();
void get_look_ahead_yaw(int16_t pilot_yaw);


/* @brief Updates the throttle cruise parameter during the call to get_throttle_rate
* @param throttle The throttle value to update to
*/
void update_throttle_cruise(int16_t throttle);

// NOTE Probably remove this or inline it
/* @brief Turns off acceleration-based throttle control. Used if motors are not armed.
*/
void throttle_accel_deactivate();

/* This is called during set_throttle_mode to get the initial alt hold desired altitude */
int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms);


// NOTE why does this return bool? Return val not used during throttle loop
/* @brief Runs land detection during throttle loop
* @returns true if landed 
*/
bool update_land_detector();


/* @brief Unsets the land detector variable
*/
void reset_land_detector();

/* @brief Calls both `reset_rate_I` and `reset_throttle_I`
*/
void reset_I_all(void);

/* @brief Resets The I term of the three PID rate controllers
*/
void reset_rate_I(void);

/* @brief Resets the I term of both the throttle alt hold controller and throttle accel controllers
*/
void reset_throttle_I(void);

// TODO Change the name of this?
/* @brief Constrains the roll and pitch between +-ROLL_PITCH_INPUT_MAX
*/
void reset_roll_pitch_in_filters(int16_t roll_in, int16_t pitch_in);




