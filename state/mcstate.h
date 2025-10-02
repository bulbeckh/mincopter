#pragma once

#include <ahrs.h>
#include <inav.h>

#include "mcstate_state.h"

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
		MCState(void);

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
		MC_AHRS_CLASS ahrs;

		/* @brief inertial_nav Used to estimate position and velocity */
		MC_INAV_CLASS inertial_nav;

		/* @brief Initialise the MCState */
		void init(void);

		/* @brief Run an update of the state estimation libraries to produce an accurate state vector */
		void update(void);

	public:

		/* @brief State struct which is passed between ahrs and inertial_nav */
		MCStateData _state;

		// TODO REMOVE
		/* @brief IMU roll rates that get updated during read_AHRS */
		// TODO does this really need to be here - can we use ins readings directly instead?
		//Vector3f omega;

		// TODO Do we remove or keep these - they are helpful values to avoid calling get_dcm or get_euler from controller
		/* @brief Orientation values from DCM. Updated during call to update_trig in fast_loop */
		float cos_roll_x         = 1.0;
		float cos_pitch_x        = 1.0;
		float cos_yaw            = 1.0;
		float sin_yaw;
		float sin_roll;
		float sin_pitch;
		
		//
		int32_t roll_sensor;
		int32_t pitch_sensor;
		int32_t yaw_sensor;

	private:
		/* @brief Euler angle <R,P,Y> representation of orientation. Retrieved through get_euler_angles in order to ensure on-demand computation */
		//Vector3f _euler;

		/* @brief DCM Matrix representation of orientation. Computed on demand */
		Matrix3f _dcm;

	public:
		// NOTE These two functions are computed on demand based off the current attitude quaternion

		/* @brief Get euler angle representation of orientation */
		const Vector3f &get_euler_angles(void);

		/* @brief Get euler rates which are body-frame gyro readings converted to euler rates */ 
		const Vector3f &get_euler_rates(void);

		/* @brief Get DCM matrix representation of orientation */
		const Matrix3f &get_dcm(void);

	public:
		// TODO These should be private and retrieved via get_pos or get_home
		
		bool home_set{false};
		
		/* @brief home Location (lat/lng/alt) where we first have arm the copter (after achieving GPS lock) */
		struct   Location home;

		/* @brief current_loc Current location (lat/lng/alt) of the copter */
		struct   Location current_loc;

		/* @brief Set altitude of inertial nav. */
		void set_altitude(float new_alt);

		/* @brief Set home position via latitude and longitude (in deg*1e7) */
		void set_home_position(int32_t lat, int32_t lng);

		/* @brief Checks if position reading is valid. Always true for simulation. */
		bool position_ok(void) const;

		/* @brief Returns latitude in deg*1e7  (*10,000,000) */
		int32_t get_latitude() const;

		/* @brief Returns longitude in deg*1e7  (*10,000,000) */
		int32_t get_longitude() const;

		/* @brief Returns altitude in cm. NOTE Even though the earth frame is NED, this will return a positive altitude */
		float get_altitude() const;

		/* @brief Get position in earth frame (NED) */
		const Vector3f get_position() const;

		// TODO Should this return a ref? Same with get_position?
		/* @brief Get velocity in earth frame (NED) */
		const Vector3f get_velocity() const;

		// TODO Add a get_heading method (formerly in Compass.h)

	public:
		// TODO REMOVE
		void update_trig(void);

};


