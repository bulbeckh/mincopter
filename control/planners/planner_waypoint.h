
#pragma once

// Waypoint Implementation of Planner

/* The waypoint planner should start in a land state and then fly to each waypoint and the end in a land state.
 *
 *
 * METHOD
 * 
 * 1. Check failsafe and fence.
 *
 * 2. Check if criteria is met to move to next state (i.e. from LAND to first WP or from 4th WP to 5th WP).
 *
 * 3. Use planner to determine control inputs/targets.
 *
 *
 *
 * planner_waypoint is responsible for generating the following control inputs to the controller
 *
 *
 * - control_pitch
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
 *
 */


class WP_Planner : MC_Planner
{

	public:
		WP_Planner() :
		{
		}

	public:

		/* @brief entry point into the planner. Called by scheduler */
		void run_nav_updates(void);

		/* @brief Runs navigation controller. Called by scheduler */
		void update_nav_mode();

	public:
		float lon_error;
		float lat_error; 

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
		int32_t original_wp_bearing;
		// The location of home in relation to the copter in centi-degrees
		int32_t home_bearing;
		// distance between plane and home in cm
		int32_t home_distance;
		int32_t initial_armed_bearing;
		// The cm/s we are moving up or down based on filtered data - Positive = UP
		int16_t climb_rate;
		// The altitude as reported by Baro in cm – Values can be quite high
		int32_t baro_alt;
																							 
		// Distance between copter and next wp in cm
		uint32_t wp_distance;

		// Angle from copter to next wp in centi-degrees
		int32_t wp_bearing;

		/* Yaw Variables */

		// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
		Vector3f yaw_look_at_WP;
		// bearing from current location to the yaw_look_at_WP
		int32_t yaw_look_at_WP_bearing;
		// yaw used for YAW_LOOK_AT_HEADING yaw_mode
		int32_t yaw_look_at_heading;
		// Deg/s we should turn
		int16_t yaw_look_at_heading_slew;

		/* Planner State Variables */

		/* @brief Control mode variable - NOTE will be replaced */
		int8_t control_mode = STABILIZE;

		/* @brief Flight mode variables that get updated during call to set_mode */
		uint8_t yaw_mode = STABILIZE_YAW;
		uint8_t roll_pitch_mode = STABILIZE_RP;
		uint8_t throttle_mode = STABILIZE_THR;
		uint8_t nav_mode;

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

	private:
		/* @brief Gets latitude and longitude from inertial nav
		*/
		void calc_position();

		/* @brief Calculates distance and bearing to waypoint. Sets wp_distance and wp_bearing
		*/
		void calc_distance_and_bearing();

		/* @brief Zeroes-out wp_bearing, wp_location, lat, and long
		*/
		void reset_nav_params(void);

		// NOTE why does this return bool? Return val not used during throttle loop
		/* @brief Runs land detection during throttle loop
		* @returns true if landed 
		*/
		bool update_land_detector();

		/* @brief Unsets the land detector variable
		*/
		void reset_land_detector();
}

