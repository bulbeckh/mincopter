
#pragma once

#include <stdint.h>

#include "control.h"
#include "mcinstance.h"
#include "mcstate.h"

class PID_Controller : MC_Controller
{
	public:

		// TODO Remove this call to parent constructor as nothing is actually initialised in control.h yet
		PID_Controller(
		) : MC_Controller(),

        // PID controller	      initial P	              initial I		            initial D               initial imax
        //-----------------------------------------------------------------------------------------------------
        pid_rate_roll           (RATE_ROLL_P,           RATE_ROLL_I,            RATE_ROLL_D,            RATE_ROLL_IMAX),
        pid_rate_pitch          (RATE_PITCH_P,          RATE_PITCH_I,           RATE_PITCH_D,           RATE_PITCH_IMAX),
        pid_rate_yaw            (RATE_YAW_P,            RATE_YAW_I,             RATE_YAW_D,             RATE_YAW_IMAX),
        pid_throttle_accel      (THROTTLE_ACCEL_P,      THROTTLE_ACCEL_I,       THROTTLE_ACCEL_D,       THROTTLE_ACCEL_IMAX),

        // PI controller	initial P			initial I			initial imax
        //----------------------------------------------------------------------
        pi_stabilize_roll       (STABILIZE_ROLL_P,      STABILIZE_ROLL_I,       STABILIZE_ROLL_IMAX),
        pi_stabilize_pitch      (STABILIZE_PITCH_P,     STABILIZE_PITCH_I,      STABILIZE_PITCH_IMAX),
        pi_stabilize_yaw        (STABILIZE_YAW_P,       STABILIZE_YAW_I,        STABILIZE_YAW_IMAX),
        pid_throttle_rate       (THROTTLE_RATE_P,       THROTTLE_RATE_I,        THROTTLE_RATE_D,        THROTTLE_RATE_IMAX),
        pi_alt_hold             (ALT_HOLD_P,            ALT_HOLD_I,             ALT_HOLD_IMAX)
	{
	}

	public:

		void run() override;

		/* Controller runs in the following way, using inputs
		 *
		 * - update_roll_pitch_mode(), using control_roll and control_pitch as inputs
		 *   - get_stabilize_roll
		 *   - get_stabilize_pitch
		 * - update_yaw_mode(), using control_yaw as input (either directly or after passing through get_yaw_slew)
		 *   - get_look_at_yaw, if mode is YAW_LOOK_AT_LOCATION
		 *     - get_stabilize_yaw
		 *   - get_look_ahead_yaw, if mode is YAW_LOOK_AHEAD
		 *     - get_stabilize_yaw
		 *   - get_stabilize_yaw, if mode is any other mode
		 * - update_throttle_mode()
		 *   - get_throttle_althold_with_slew
		 *     - get_throttle_althold
		 *   - get_throttle_althold
		 *     - get_throttle_rate
		 *   - get_throttle_land
		 *   	 - get_throttle_rate_stabilized
		 *   	 	 - get_throttle_althold
		 *
		 * - update_rate_controller_targets
		 *
		 * - run_rate_controllers
		 *   - get_rate_roll
		 *   - get_rate_pitch
		 *   - get_rate_yaw
		 *   - get_throttle_accel
		 *
		 */
	
	private:
		/* @brief Rate controller targets updated by update_rate_controller_targets and feed into PID rate controllers */
		int32_t roll_rate_target_ef;
		int32_t pitch_rate_target_ef;
		int32_t yaw_rate_target_ef;

		int32_t roll_rate_target_bf;
		int32_t pitch_rate_target_bf;
		int32_t yaw_rate_target_bf;

		int16_t throttle_accel_target_ef;

	public:
		// Public for now - needs to be accessible from planner and getter/setter is bloat
		float throttle_avg;                  // throttle_cruise as a float
		bool throttle_accel_controller_active;   // true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
																		
    int16_t        throttle_min;
    int16_t        throttle_max;
    int16_t        throttle_cruise;

    int32_t        angle_rate_max;             // maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
	
	private:
		// An additional throttle added to keep the copter at the same altitude when banking
		int16_t angle_boost;

		// The (throttle) controller desired altitude in cm
		float controller_desired_alt;

    int16_t        pilot_velocity_z_max;        // maximum vertical velocity the pilot may request
																								//
    int16_t        land_speed;
		// counter to verify landings
		uint16_t land_detector;


	private:
	
		// High-level Controllers
    AC_PID                  pi_stabilize_roll;
    AC_PID                  pi_stabilize_pitch;
    AC_PID                  pi_stabilize_yaw;
    AC_PID                  pid_throttle_rate;
    AC_PID                  pi_alt_hold;

		// Rate Controllers
    AC_PID                  pid_rate_roll;
    AC_PID                  pid_rate_pitch;
    AC_PID                  pid_rate_yaw;
    AC_PID                  pid_throttle_accel;

	private:

		/****** Update functions ******/

		/* @brief Triggers run of the throttle controller based on the current throttle mode
		*/
		void update_throttle_mode(void);

		/* @brief Triggers run of the roll and pitch controllers based on the current
		* roll/pitch modes set by `set_roll_pitch_mode`
		*/
		void update_roll_pitch_mode(void);

		/* @brief Triggers update of the yaw controller based on current yaw mode
		*/
		void update_yaw_mode(void);

		/* @brief Converts earth frame targets to body frame targets using DCM matrix. Called during fast loop
		*/
		void update_rate_controller_targets();

		/****** Higher-level controllers ******/

		/* @brief Mode-specific functions which convert the error between current and desired lean angle into a rate target and pass to set_<roll/pitch/yaw>_rate_target. Uses g.pi_stabilize_roll PID controller.
		* @param target_angle The target angles obtained from get_pilot_desired_lean_angles
		*/
		void get_stabilize_roll(int32_t target_angle);
		void get_stabilize_pitch(int32_t target_angle);
		void get_stabilize_yaw(int32_t target_angle);
		void get_throttle_rate(float z_target_speed);

		/* @brief Wrappers around get_throttle_rate. All eventually call get_throttle_rate
		 */
		void get_throttle_rate_stabilized(int16_t target_rate);
		void get_throttle_land();
		void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
		void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);

		/* @brief Wrappers around get_stabilize_yaw, ultimately modifying the control_yaw variable
		 */

		/* @brief Reduces rate-of-change of yaw to a maximum value
		* @param current_yaw The current yaw value. Usually control_yaw
		* @param desired_yaw The target yaw value.
		* @param deg_per_sec The maximum rate-of-change of yaw value. For example AUTO_YAW_SLEW_RATE
		*/
		int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec);

		/****** Lower-level controllers ******/

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

		/* @brief Throttle PID acceleration controller
		* @param z_target_accel The target acceleration value, computed by throttle controller (get_throttle_rate)
		* @returns The throttle control signal to be passed to g.rc_3
		*/
		int16_t get_throttle_accel(int16_t z_target_accel);

		/* @brief Sets the g.rc_3 (throttle channel) output based on the (angle boosted) throttle control signal during run_rate_controllers
		* @param throttle_out The output of the throttle controller from get_throttle_accel
		* @param apply_angle_boost Whether or not to compensate throttle value for roll/pitch
		*/
		void set_throttle_out(int16_t throttle_out, bool apply_angle_boost);

		/* @brief Used to compensate throttle value for roll/pitch
		* @param throttle a throttle control signal from throttle controller
		* @returns the compensated throttle control signal
		*/
		int16_t get_angle_boost(int16_t throttle);


		/************** ?? What does this do **************/
		/* @brief Updates the throttle cruise parameter during the call to get_throttle_rate
		* @param throttle The throttle value to update to
		*/
		void update_throttle_cruise(int16_t throttle);

	public:

		/****** Reset functions ******/

		/* @brief Calls both `reset_rate_I` and `reset_throttle_I`
		*/
		void reset_I_all(void);

		/* @brief Resets The I term of the three PID rate controllers
		*/
		void reset_rate_I(void);

		/* @brief Resets the I term of both the throttle alt hold controller and throttle accel controllers
		*/
		void reset_throttle_I(void);

};

