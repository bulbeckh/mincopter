#pragma once

#include <ahrs.h>
#include <inav.h>


// NOTE Forward declaration of the MCStatePrivate instance
class MCStatePrivate;

class MCState
{

	public:
		/* @brief MCState is a shared representation of the copter at a given time including kinematics (position, velocity, accel)
		 * and angular rotation/velocity. Things like flight states/modes and armed/disarmed states should be part of the planner and
		 * **not** part of state. The controllers should rely on this copter state object as inputs for determining control outputs.
		 *
		 * State estimation should reside here which takes input from sensors (in dev/) and determine state. The actual implementations
		 * are derived from this base MCState class and need to implement both attitude and inertial navigation.
		 *
		 * This is done by overriding the **update** method and the implementation-specific **init_derived** methods */
		MCState(void);

		/* MCStatePrivate contains the entire state vector:
		 * 
		 * - position (float[3])
		 * - velocity (float[3)
		 * - attitude (as quaternion - float[4])
		 * - attitude (as euler angles - float[3])
		 * - angular velocity (float[3]) */

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

		// @brief Implementation-specific initialisation */
		virtual void init_derived(void) = 0;

		// TODO We can either choose to update the whole method, or if there is some implementation-independent update then we can
		// use a pattern similar to the **init** / **init_derived** functions
		/* @brief Run an update of the state estimation libraries to produce an accurate state vector */
		virtual void update(void) = 0;

	public:

		// TODO This should be private and retrieved only via the interfaces

		/* @brief State struct which is passed between ahrs and inertial_nav */
		MCStatePrivate _state;

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
		int32_t get_latitude(void) const;

		/* @brief Returns longitude in deg*1e7  (*10,000,000) */
		int32_t get_longitude(void) const;

		/* @brief Returns altitude in cm. NOTE Even though the earth frame is NED, this will return a positive altitude */
		float get_altitude(void) const;

		/* @brief Get position in earth frame (NED) */
		const Vector3f get_position(void) const;

		// TODO Should this return a ref? Same with get_position?
		/* @brief Get velocity in earth frame (NED) */
		const Vector3f get_velocity(void) const;

		// TODO Add a get_heading method (formerly in Compass.h)

	public:
		// TODO REMOVE
		void update_trig(void);

};


