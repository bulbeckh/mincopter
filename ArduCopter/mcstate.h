#pragma once

#include <AP_AHRS.h>
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>     		// ArduCopter waypoint navigation library
#include <AC_Fence.h>           // Arducopter Fence library

#include "failsafe.h"

class MCState
{

	public:
		/* @brief MCState
		* @param mci (MCInstance*) Pointer to an MCInstance object containing interfaces to sensor inputs and control outputs
		*/
		MCState();

	public:
	
		/* @brief ahrs tracks the copter orientation and heading */
		AP_AHRS_DCM ahrs;

		// Failsafe
		AP_FAILSAFE_T failsafe;

		AP_InertialNav inertial_nav;
		AC_Fence fence;

		// TODO Move this to BTree as this is a control function not a state function
		AC_WPNav wp_nav;

	public:
		/* @brief IMU roll rates that get updated during read_AHRS */
		// TODO does this really need to be here - can we use ins readings directly instead?
		Vector3f omega;

		/* @brief Desired Roll/Pitch angles in (centi-degrees) and desired yaw */
		// TODO Should the control values be part of the btree? Should there be a separate class holding control variables and PID controllers
		int16_t control_roll;
		int16_t control_pitch;
		int32_t control_yaw;

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

		void read_AHRS(void);

		void failsafe_gps_check(void);

};


