
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
		
		// Distance between copter and next wp in cm
		uint32_t wp_distance;

		// Angle from copter to next wp in centi-degrees
		int32_t wp_bearing;


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

