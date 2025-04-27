#pragma once

#include <AP_AHRS.h>
#include <AP_InertialNav.h>


class MCState
{

	public:
		/* @brief MCState is a shared representation of the copter at a given time including kinematics (position, velocity, accel)
		 * and angular rotation/velocity.
		 *
		 * Things like flight states/modes and armed/disarmed states should be part of the planner and **not** part of state.
		 *
		 * The controllers should rely on this copter state object as inputs for determining control outputs.
		 *
		 * State estimation should reside here which takes input from sensors (in dev/) and determine state (i.e. via EKF3, DCM).
		 *
		 * @param mci (MCInstance*) Pointer to an MCInstance object containing interfaces to sensor inputs and control outputs
		 */
		MCState();

	public:
	
		/* @brief ahrs tracks the copter orientation */
		AP_AHRS_DCM ahrs;

		/* @brief inertial_nav fuses AHRS and GPS to determine position and velocity */
		AP_InertialNav inertial_nav;

	public:
		/* @brief IMU roll rates that get updated during read_AHRS */
		// TODO does this really need to be here - can we use ins readings directly instead?
		Vector3f omega;

		/* @brief Orientation values from DCM. Updated during call to update_trig in fast_loop */
		float cos_roll_x         = 1.0;
		float cos_pitch_x        = 1.0;
		float cos_yaw            = 1.0;
		float sin_yaw;
		float sin_roll;
		float sin_pitch;

	public:
		/* Navigation Variables */

		// home location is stored when we have a good GPS lock and arm the copter
		struct   Location home;
		// Current location of the copter
		struct   Location current_loc;

	public:
		// TODO Do these really need to be class methods
		void update_trig(void);

		/* @brief Entry point into state update
		 */
		void read_AHRS(void);


};


