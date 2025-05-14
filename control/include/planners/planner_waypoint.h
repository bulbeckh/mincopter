
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

#include "planner.h"
#include "config.h"

enum class WP_FLIGHT_STATE
{
		FS_TAKEOFF,
		FS_WAYPOINT,
		FS_LOITER,
		FS_LAND
};

class WP_Planner : public MC_Planner
{

	public:
		WP_Planner() :
			MC_Planner()
		{
		    /* Initialise planner states */
		    nav_mode = WP_FLIGHT_STATE::FS_TAKEOFF;

		}

	public:

		void run(void) override;

		/* @brief entry point into the planner. Called by scheduler */
		void run_nav_updates(void);

		/* @brief Runs navigation controller. Called by scheduler */
		void update_nav_mode();

	public:
		float lon_error;
		float lat_error; 

		bool throttle_initialised;

		// counter to verify landings
		uint16_t land_detector;

		AP_UNION_T ap;

		/* Failsafe Parameters */
    int8_t         failsafe_battery_enabled;   // battery failsafe enabled
    float          fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    float          fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered
    int8_t         failsafe_gps_enabled;       // gps failsafe enabled
    int8_t         failsafe_gcs;               // ground station failsafe behavior
    int8_t         failsafe_throttle;
    int16_t        failsafe_throttle_value;

		/* Navigation Parameters */

		// The original bearing to the next waypoint.  used to point the nose of the copter at the next waypoint
		// Used in the YAW_LOOK_AHEAD_NEXT_WP yaw mode
		int32_t original_wp_bearing;

		// distance between plane and home in cm
		int32_t home_distance;

		// The altitude as reported by Baro in cm – Values can be quite high
		int32_t baro_alt;
																							 
		//TODO Remove wp_distance and wp_bearing - I think they are updated but not used
		// Distance between copter and next wp in cm
		uint32_t wp_distance;

		// Angle from copter to next wp in centi-degrees
		int32_t wp_bearing;

		/* Yaw Variables */

		// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
		Vector3f yaw_look_at_WP;
		// bearing from current location to the yaw_look_at_WP
		int32_t yaw_look_at_WP_bearing;

		/* Planner State Variables */

		/* @brief Control mode variable - NOTE will be replaced */
		int8_t control_mode = STABILIZE;

		/* @brief Flight mode variables that get updated during call to set_mode */
		//uint8_t yaw_mode = STABILIZE_YAW;
		//uint8_t roll_pitch_mode = STABILIZE_RP;
		//uint8_t throttle_mode = STABILIZE_THR;
		//
		WP_FLIGHT_STATE nav_mode;

		// Throttle variables
		int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
		float target_alt_for_reporting;      // target altitude in cm for reporting (logs and ground station)

		// The Commanded Throttle from the autopilot.
		int16_t nav_throttle;    // 0-1000 for throttle control
														 //
														 //
		// We use atan2 and other trig techniques to calaculate angles
		// We need to scale the longitude up to make these calcs work
		// to account for decreasing distance between lines of longitude away from the equator
		float scaleLongUp = 1;
		// Sometimes we need to remove the scaling for distance calcs
		float scaleLongDown = 1;

		// Max lean-angle of the copter in centi-degrees
		int16_t angle_max;

	private:

		/* @brief Sets the control_mode (e.g. ALT_HOLD, STABILIZE, etc.)
		* @param mode The control mode to be set.
		*/
		bool set_mode(uint8_t mode); /* @brief Gets latitude and longitude from inertial nav

		*/
		void calc_position();

		/* @brief Calculates distance and bearing to waypoint. Sets wp_distance and wp_bearing
		*/
		void calc_distance_and_bearing();

		/* @brief Zeroes-out wp_bearing, wp_location, lat, and long
		*/
		void reset_nav_params(void);

	public:
		// NOTE why does this return bool? Return val not used during throttle loop
		/* @brief Runs land detection during throttle loop
		* @returns true if landed 
		*/
		bool update_land_detector();


	private:
		/* @brief Unsets the land detector variable
		*/
		void reset_land_detector();

		/* @brief Sets throttle mode
		*/
		bool init_throttle( uint8_t new_throttle_mode );

		/* This is called during set_throttle_mode to get the initial alt hold desired altitude */
		int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms);

		/****** Arming Functions ******/

		/* @brief Arms motors. Starts logging, enables output to motors, and a few other functions
		*/
		void init_arm_motors();

		/* @brief Disarms motors.
		*/
		void init_disarm_motors();

		/* @brief Disarms motors if copter has been stationary on ground for 15sec. Called at 1hz by 1hz_loop
		*/
		void auto_disarm_check(); 

		/* @brief Checks run before motors are armed
		*/
		void pre_arm_checks(bool display_failure);

		/* @brief Reduces rate-of-change of yaw to a maximum value
		* @param current_yaw The current yaw value. Usually control_yaw
		* @param desired_yaw The target yaw value.
		* @param deg_per_sec The maximum rate-of-change of yaw value. For example AUTO_YAW_SLEW_RATE
		*/
		int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec);

		void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);

	private:
		bool pre_arm_gps_checks(bool display_failure);
		bool arm_checks(bool display_failure);

		/****** Failsafe Functions ******/

		/* @brief Called when radio loses connection, triggering the failsafe to kick-in
		*/
		void failsafe_radio_on_event();

		/* @brief Called when returning from a failsafe mode
		*/
		void failsafe_radio_off_event();

		/* @brief Called when a low battery occurs, triggering failsafe
		*/
		void failsafe_battery_event(void);

		/* @brief Called when GPS returns signal
		*/
		void failsafe_gps_off_event(void);

		void failsafe_gps_check(void);

		void fence_check();

};


