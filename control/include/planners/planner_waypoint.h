
#pragma once

// Waypoint Implementation of Planner

/* The waypoint planner should start in a land state and then fly to each waypoint and the end in a land state.
 *
 *
 * ## Flight States
 * FS_TAKEOFF  - Planner begins in this state. Carefully tuned motor gains.
 * FS_WAYPOINT - Navigation between waypoints. Standard flight mode.
 * FS_LOITER   - Stationary mode for being stationary in air.
 * FS_LAND     - Prepare for landing. Slower descent speed and checks for ground.
 *
 * ## Motor States
 * ARMED    - Can send outputs to motors.
 * DISARMED - Cannot send outputs to motors.
 *
 * ## Sensor States
 * CLEAN    - All sensors have valid readings and can be trusted.
 * DIRTY    - At least one sensor is not working and has been broken for a minimum threshold time.
 *
 * planner_waypoint is responsible for generating the following control inputs to the controller
 *
 * - control_pitch. 
 * - control_roll
 * - control_yaw
 * - a desired altitude (from wp_nav.get_desired_alt)
 * - a descent velocity (from wp_nav.get_descent_velocity)
 * - a climb velocity (from wp_nav.get_climb_velocity)
 *
 * TODO I don't actually think yaw is even used in the autonomous modes
 *
 * Planner follows this loop:
 *
 * - update_nav_mode (called by scheduler)
 *   - update_wpnav
 *   	 - advance_target_along_track
 *   	   - (sets wp_nav._target variable which is used in get_desired_alt)
 *   	 - get_loiter_acceleration_to_lean_angles
 *   - update_loiter
 *   	 - (four steps)
 *   	 - get_loiter_acceleration_to_lean_angles
 * 
 * TODO A lot of the functionality specified in this base class needs to be hoisted to the parent class - planner.h
 *
 */

#include "planner_interface.h"
#include "config.h"

#include <AC_WPNav.h>

class WP_Planner : public MC_Planner
{
	public:
		WP_Planner() : MC_Planner() { }

	public:
		/* @brief Run planner */
		void run(void) override;

	public:

		// TODO Will bring back in when land detection is added 
		//uint16_t land_detector;

		/* Navigation Parameters */

	public:
		// NOTE why does this return bool? Return val not used during throttle loop
		/* @brief Runs land detection during throttle loop
		* @returns true if landed 
		*/
		//bool update_land_detector();
		//
		/* @brief Unsets the land detector variable
		*/
		//void reset_land_detector();

	private:

		/* @brief Arms motors. Starts logging, enables output to motors, and a few other functions
		*/
		void init_arm_motors();

		/* @brief Disarms motors.
		*/
		void init_disarm_motors();
		
	private:

		bool arm_checks(void);

		// TODO Kept here only for reference when we actually need to implement failsafe checks in the scheduled
		// failsafe function.

		// TODO We should explore having a planner interface for any custom planner-specific failsafe checks that need to run
		// at the failsafe frequency (10Hz)
		/****** Failsafe Functions ******/

		/* @brief Called when radio loses connection, triggering the failsafe to kick-in */
		//void failsafe_radio_on_event();

		/* @brief Called when returning from a failsafe mode */
		//void failsafe_radio_off_event();

		/* @brief Called when a low battery occurs, triggering failsafe */
		//void failsafe_battery_event(void);

		/* @brief Called when GPS returns signal */
		//void failsafe_gps_check(void);

		void fence_check(void);

};


