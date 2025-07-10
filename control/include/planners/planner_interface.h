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

enum class PlannerArmState
{
    ARMED,
    DISARMED
};

class MC_Planner
{
	public:

		MC_Planner() :
			fence(&mcstate.inertial_nav),
			wp_nav(&mcstate.inertial_nav, &mcstate.ahrs)
		{
			// Initialised planner states
			planner_arm_state = PlannerArmState::DISARMED;
		}


		/* @brief Entry point for planner function
		 */
		virtual void run() = 0;

		PlannerArmState planner_arm_state;

	public:
		AP_FAILSAFE_T failsafe;

		AP_UNION_T ap;

		AC_Fence fence;

		AC_WPNav wp_nav;

		// The altitude as reported by Baro in cm – Values can be quite high
		int32_t baro_alt;
		int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
		// Max lean-angle of the copter in centi-degrees
		int16_t angle_max;
		// We use atan2 and other trig techniques to calaculate angles
		// We need to scale the longitude up to make these calcs work
		// to account for decreasing distance between lines of longitude away from the equator
		float scaleLongUp = 1;
		// Sometimes we need to remove the scaling for distance calcs
		float scaleLongDown = 1;

		/* Failsafe Parameters */
    int8_t         failsafe_battery_enabled;   // battery failsafe enabled
    float          fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    float          fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered
    int8_t         failsafe_gps_enabled;       // gps failsafe enabled
    int8_t         failsafe_gcs;               // ground station failsafe behavior
    int8_t         failsafe_throttle;
    int16_t        failsafe_throttle_value;
};

