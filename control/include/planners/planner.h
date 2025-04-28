#pragma once

// Planner Class
//

/* The planner class is responsible for determining target state given the current state. It provides the
 * controllers with desired roll/pitch/yaw/throttle targets that the controller actualises.
 *
 * The planner is also responsible for managing copter states including failsafe checks and fence checks.
 *
 * Other than completing failsafe and fence checks, the planners ultimate job is to generate a reference
 * trajectory for the controller to follow. This includes reference states for the entire state vector.
 *
 * We will use a 12-parameter state vector which is to be used by the controller to determine the control
 * inputs.
 *
 * State Vector:
 * - Position (in inertial frame) (3) : x, y, z 
 * - Linear Velocity (in inertial frame) (3) : vx, vy, vz
 * - Angular Rotation (in inertial frame) (3) : roll, pitch, yaw
 * - Angular Velocity (in body frame) (3) : v_roll, v_pitch, v_yaw
 *
 * A reference trajectory with all 12 states need to be calculated by the planner. This is then fed to the
 * controller. Some controllers may ignore certain states (i.e. the PID_Controller only uses desired_throttle
 * (which is equivalent to vz), and a control_roll, control_pitch, and control_yaw (which is the angular
 * rotation in the above state vector).
 *
 * Here are some examples of reference trajectories, 
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

#include <AC_Fence.h>
#include <AC_WPNav.h>

#include "failsafe.h"

#include "mcinstance.h"
#include "mcstate.h"
extern MCInstance mincopter;
extern MCState mcstate;

class MC_Planner
{

	public:

		MC_Planner() :
			fence(&mcstate.inertial_nav),
			wp_nav(&mcstate.inertial_nav, &mcstate.ahrs)
		{
		}


		/* @brief Entry point for planner function
		 */
		virtual void run() = 0;



	public:
		AP_FAILSAFE_T failsafe;

		AC_Fence fence;

		AC_WPNav wp_nav;


		/* @brief Desired roll/pitch/yaw values for determination by planner and usage by controller
		 */
		int16_t control_roll;
		int16_t control_pitch;
		int32_t control_yaw;

};

