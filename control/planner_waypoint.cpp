
#include "planner_waypoint.h"

#include "failsafe.h"
#include "util.h"
#include "log.h"

#include "mcinstance.h"
extern MCInstance mincopter;

#include "mcstate.h"

//TODO Change all these references to either state or mcstate.

/* TODO There should be a clear one-way relationship between planner and controller. At the moment it is being mixed
 * Either the planner should set controller control variables directly like 'controller_desired_altitude' or the controller should request
 * the variables from the planner.
 */

#include "control.h"

/* Instantiate WP_Planner here */
WP_Planner planner;

void WP_Planner::run(void)
{
    /* When the planner is first run, it needs to wait until the sensor signals are all correct (i.e. GPS
     * lock, correct compass, baro, and IMU readings). We should then add the waypoints to the waypoint navigation
     * library. Then we can begin the arm process and start the planner.
     */

	/* TODO Fix this NOTE In the simulated sensors, we need to wait for a at least 20ms (2 iterations) for the
	 * simulated GPS to be called and a signal to be read. Since this planner runs at 100Hz, we should wait 5
	 * iterations before actually arming successfully (and hence calling the *init_home* method which sets
	 * the home reference location) */
#ifdef TARGET_ARCH_LINUX
	static int arm_delay_counter=0;
#endif

	// Commence arming if we are disarmed and have been requested to arm by the telemetry
	if (!planner.ap.arm_active && planner.ap.arm_requested_telem) { 

		// Run arm checks
		if (!arm_checks()) {
			// TODO Change this to a reason
			// Log to telemetry that we have failed to arm
			mincopter.hal.console->printf("Failed arm check\r\n");

			// Clear arm request flag
			planner.ap.arm_requested_telem = 0;

			return;
		}

#ifdef TARGET_ARCH_LINUX
		// Wait 1sec until we arm
		if (arm_delay_counter<100) {
			arm_delay_counter++;
			return;
		}
#endif

		// Begin to arm motors if we pass all arming checks
		init_arm_motors();

		/*
		// If the motor flag is actually set to armed then update the planner state to ARMED
		if (mincopter.motors.armed()) {
		} else {
			return;
		}
		*/

    }

	// TODO Replace the rest of this function with the nav/waypoint controller from Controller_CSC. We need the planner to
	// determine the throttle and desired lean/yaw angles which are passed to the controller
	return;

	// On the first iteration of the planner, we initialise the controller with a reference trajectory
	/*
	static uint8_t state_ref_call=0;
	if (!state_ref_call) {
		float ref[12] = {0, 0, -10, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		controller.update_constant_state_reference(ref);
		state_ref_call=1;
	}
	*/

	/*
	 * 1. Check failsafe and fence.
	 *
	 * 2. Check if criteria is met to move to next state (i.e. from LAND to first WP or from 4th WP to 5th WP).
	 *
	 * 3. Use planner to determine control inputs/targets.
	 */

  	// 1. Failsafe and fence checks
	// TODO What is the behaviour if fence_check fails?
	fence_check();

	return;
}

// TODO Remove
// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// returns true if we have landed
/*
bool WP_Planner::update_land_detector()
{
    // detect whether we have landed by watching for low climb rate and minimum throttle
    if (abs(controller.climb_rate) < 20 && mincopter.motors.limit.throttle_lower) {
        if (!ap.land_complete) {
            // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                //set_land_complete(true);
                land_detector = 0;
            }
        }
    }else if (mincopter.rc_3.control_in != 0 || failsafe.radio){    // zero throttle locks land_complete as true
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        if(ap.land_complete) {
            set_land_complete(false);
        }
    }

    // return current state of landing
    return ap.land_complete;
}
*/

// reset_land_detector - initialises land detector
/*
void WP_Planner::reset_land_detector()
{
    set_land_complete(false);
    land_detector = 0;
}
*/

// TODO Fence checks moved to failsafe check
/*
void WP_Planner::fence_check()
{
		uint8_t new_breaches; // the type of fence that has been breached
		uint8_t orig_breaches = fence.get_breaches();

		// return immediately if motors are not armed
		if(!mincopter.motors.armed()) {
				return;
		}

		// give fence library our current distance from home in meters
		fence.set_home_distance(home_distance*0.01f);

		// check for a breach
		new_breaches = fence.check_fence();

		// if there is a new breach take action
		if( new_breaches != AC_FENCE_TYPE_NONE ) {
#ifdef TARGET_ARCH_LINUX
			std::cout << "Fence breached\n";
#endif

				// if the user wants some kind of response and motors are armed
				if(fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY ) {

						// disarm immediately if we think we are on the ground
						// don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
						if(// manual_flight_mode(control_mode) &&
mincopter.rc_3.control_in == 0 && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0)){
								init_disarm_motors();
						}else{
								// if we are within 100m of the fence, RTL
								if (fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
										nav_mode = WP_FLIGHT_STATE::FS_LAND;
								}
						}
				}

				// log an error in the dataflash
				Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);
		}

		// record clearing of breach
		if(orig_breaches != AC_FENCE_TYPE_NONE && fence.get_breaches() == AC_FENCE_TYPE_NONE) {
				Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
		}
}
*/

