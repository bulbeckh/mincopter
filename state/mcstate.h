#pragma once

#include <ahrs.h>
#include <inav.h>

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

		/* MCState should contain the entire state vector:
		 * 
		 * - position (float[3])
		 * - velocity (float[3)
		 * - orientation (quaternion[4])
		 * 
		 * Additionally, it should also contain:
		 *
		 * - angular velocity (float[3])
		 * - acceleration (float[3])
		 *
		 * These are typically just read straight from the gyrometer/accelerometer and rotated to inertial frame except
		 * in the case when we are estimating a sensor bias state in which case we also applied the bias correction.
		 *
		 */

	public:
	
		// NOTE These two classes are macros to the desired AHRS and INAV classes
		//
		// TODO If the EKF is used for inertial navigation, then we simply run the entire state estimation library off
		// the inertial_nav and ignore the ahrs object.
		
		/* @brief ahrs Used to estimate the copter orientation */
		MC_AHRS_CLASS* ahrs;

		/* @brief inertial_nav Used to estimate position and velocity */
		MC_INAV_CLASS* inertial_nav;

		void update(void);

	public:

		/* @brief State struct containing full system state */
		struct {
			double _position[3];
			double _velocity[3];
			// TODO We should be able to select the internal data type used by Quaternion class (i.e. float, double)
			Quaternion _attitude;

			// Angular velocity and inertial frame accelerations
			Vector3f _omega;
			Vector3f _accel;

			// TODO Add bias states
		} _state;

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

	private:
		/* @brief Euler Angles <R,P,Y> representation of orientation. Retrieved through get_euler_angles in order to ensure on-demand computation */
		Vector3f _euler_angles;

		/* @brief 

	public:
		/* Navigation Variables */

		/* @brief home Location (lat/lng/alt) where we first have arm the copter (after achieving GPS lock) */
		struct   Location home;

		/* @brief current_loc Current location (lat/lng/alt) of the copter */
		struct   Location current_loc;

	public:
		// TODO Do these really need to be class methods
		void update_trig(void);

};


