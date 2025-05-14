
#include "planner_waypoint.h"

#include "failsafe.h"
#include "util.h"
#include "log.h"

#include "mcinstance.h"
#include "mcstate.h"
extern MCInstance mincopter;
extern MCState mcstate;
//TODO Change all these references to either state or mcstate.

/* TODO There should be a clear one-way relationship between planner and controller. At the moment it is being mixed
 * Either the planner should set controller control variables directly like 'controller_desired_altitude' or the controller should request
 * the variables from the planner.
 */
#include "control.h"
#include "controller_pid.h"
extern PID_Controller controller;

#ifdef TARGET_ARCH_LINUX
    #include <iostream>
#endif

void WP_Planner::run(void)
{
    /* When the planner is first run, it needs to wait until the sensor signals are all correct (i.e. GPS
     * lock, correct compass, baro, and IMU readings). We should then add the waypoints to the waypoint navigation
     * library. Then we can begin the arm process and start the planner.
     */

	if (planner_arm_state==PlannerArmState::DISARMED) { 
		// Run pre_arm_checks
		pre_arm_checks(false);
		if (!ap.pre_arm_check) {
			std::cout << "pre arm check failed\n";
			return;
		} else {
			std::cout << "pre arm check passed\n";
		}

		// Run final checks
		if (arm_checks(false))
		{
			// Attempt Arm
			init_arm_motors();

			// If the motor flag is actually set to armed then update the planner state to ARMED
			if (mincopter.motors.armed()) {
			planner_arm_state=PlannerArmState::ARMED;
			} else {
			return;
			}
		} else {
			return;
		}
    }

	/*
	 * 1. Check failsafe and fence.
	 *
	 * 2. Check if criteria is met to move to next state (i.e. from LAND to first WP or from 4th WP to 5th WP).
	 *
	 * 3. Use planner to determine control inputs/targets.
	 */

  	// 1. Failsafe and fence checks
	fence_check();

	/* Failsafe Check */
	
	// 2. State changes

	// 3. Control determination
	update_nav_mode();

	// Roll/Pitch determination
	controller.control_roll = wp_nav.get_desired_roll();
	controller.control_pitch = wp_nav.get_desired_pitch();

	/* TODO At some point during update_nav_mode, the control_yaw will be updated. It is passed through a slew filter
	 * to ensure it stays between a specified rate. This is moved from controller to planner and now needs to be called
	 */

	// Yaw determination
	//controller.control_yaw = get_yaw_slew(controller.control_yaw, original_wp_bearing, AUTO_YAW_SLEW_RATE);
	
	// Throttle determination
  get_throttle_althold_with_slew(wp_nav.get_desired_alt(), -wp_nav.get_descent_velocity(), wp_nav.get_climb_velocity());
 	
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
void WP_Planner::get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    float alt_change = target_alt-controller.controller_desired_alt;
    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !mincopter.motors.limit.throttle_lower) || (alt_change>0 && !mincopter.motors.limit.throttle_upper)) {
        controller.controller_desired_alt += constrain_float(alt_change, min_climb_rate*0.02f, max_climb_rate*0.02f);
    }

    // do not let target altitude get too far from current altitude
    controller.controller_desired_alt = constrain_float(controller.controller_desired_alt,mcstate.current_loc.alt-750,mcstate.current_loc.alt+750);

		controller.max_climb_rate = max_climb_rate;
		controller.min_climb_rate = min_climb_rate;
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// returns true if we have landed
bool WP_Planner::update_land_detector()
{
    // detect whether we have landed by watching for low climb rate and minimum throttle
    if (abs(controller.climb_rate) < 20 && mincopter.motors.limit.throttle_lower) {
        if (!ap.land_complete) {
            // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                set_land_complete(true);
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


void WP_Planner::run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

}

void WP_Planner::calc_position(){
    if( mcstate.inertial_nav.position_ok() ) {
        // pull position from interial nav library
        mcstate.current_loc.lng = mcstate.inertial_nav.get_longitude();
        mcstate.current_loc.lat = mcstate.inertial_nav.get_latitude();
    }
}

void WP_Planner::calc_distance_and_bearing()
{
    Vector3f curr = mcstate.inertial_nav.get_position();

    // get target from loiter or wpinav controller
  	if( nav_mode == WP_FLIGHT_STATE::FS_WAYPOINT ) {
        wp_distance = wp_nav.get_distance_to_destination();
        wp_bearing = wp_nav.get_bearing_to_destination();
    }else{
        wp_distance = 0;
        wp_bearing = 0;
    }

    // calculate home distance and bearing
    if(GPS_ok()) {
        home_distance = pythagorous2(curr.x, curr.y);
        //home_bearing = pv_get_bearing_cd(curr,Vector3f(0,0,0));
    }
}

//update_nav_mode - run navigation controller based on nav_mode
// called at 100hz
void WP_Planner::update_nav_mode()
{
    // exit immediately if not auto_armed or inertial nav position bad
	/*
    if (!ap.auto_armed || !mcstate.inertial_nav.position_ok()) {
        return;
	}
	*/

	static bool firstcall=true;

    switch( nav_mode ) {

		case WP_FLIGHT_STATE::FS_LOITER:
			// reset target if we are still on the ground
			if (ap.land_complete) {
				wp_nav.init_loiter_target(mcstate.inertial_nav.get_position(),mcstate.inertial_nav.get_velocity());
			}else{
				// call loiter controller
				wp_nav.update_loiter();
			}
			break;

		case WP_FLIGHT_STATE::FS_WAYPOINT:
			// call waypoint controller
			wp_nav.update_wpnav();
			break;

		case WP_FLIGHT_STATE::FS_TAKEOFF:
			if (firstcall) { 
				Vector3f nav_current_position = mcstate.inertial_nav.get_position();
				nav_current_position.z += 10;
				nav_current_position.x -= 5;
				nav_current_position.y -= 5;

				// TODO Why does this take two separate calls
				wp_nav.init_loiter_target(nav_current_position,mcstate.inertial_nav.get_velocity());
				wp_nav.set_desired_alt(nav_current_position.z);

				// TODO Fix the name (its a target, and not a current position) and fix why this is happening here
				controller.controller_desired_alt = nav_current_position.z;

				firstcall=false;
			} else {
				wp_nav.update_loiter();
			}
			break;
    }

}

void WP_Planner::reset_nav_params(void)
{
    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by nav or loiter controllers
    lon_error                       = 0;
    lat_error                       = 0;
}

// reset_land_detector - initialises land detector
void WP_Planner::reset_land_detector()
{
    set_land_complete(false);
    land_detector = 0;
}

// get_initial_alt_hold - get new target altitude based on current altitude and climb rate
int32_t WP_Planner::get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms)
{
    int32_t target_alt;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
    int32_t linear_velocity;      // the velocity we swap between linear and sqrt.

    linear_velocity = ALT_HOLD_ACCEL_MAX/controller.pi_alt_hold.kP();

    if (abs(climb_rate_cms) < linear_velocity) {
        target_alt = alt_cm + climb_rate_cms/controller.pi_alt_hold.kP();
    } else {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*controller.pi_alt_hold.kP()*controller.pi_alt_hold.kP());
        if (climb_rate_cms > 0){
            target_alt = alt_cm + linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX);
        } else {
            target_alt = alt_cm - ( linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX) );
        }
    }
    return constrain_int32(target_alt, alt_cm - ALT_HOLD_INIT_MAX_OVERSHOOT, alt_cm + ALT_HOLD_INIT_MAX_OVERSHOOT);
}

bool WP_Planner::init_throttle( uint8_t new_throttle_mode )
{
		controller.controller_desired_alt = get_initial_alt_hold(mcstate.current_loc.alt, controller.climb_rate);     // reset controller desired altitude to current altitude
    wp_nav.set_desired_alt(controller.controller_desired_alt);                                 // same as above but for loiter controller
    throttle_initialised = true;

		/* LANDING case
            reset_land_detector();  // initialise land detector
            mincopter.controller_desired_alt = get_initial_alt_hold(mcstate.current_loc.alt, mincopter.climb_rate);   // reset controller desired altitude to current altitude
            throttle_initialised = true;
            break;
		*/

    return throttle_initialised;
}


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
						if(/* manual_flight_mode(control_mode) && */ mincopter.rc_3.control_in == 0 && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0)){
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

void WP_Planner::failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !mincopter.motors.armed() ) {
        return;
    }

		// TODO This should work but it is heavily unoptimized
		//
    // This is how to handle a failsafe.
		//
		// TODO Remove the switch statement in favor of a single AUTO mode
    switch(control_mode) {
        case STABILIZE:
            // if throttle is zero disarm motors
            if (mincopter.rc_3.control_in == 0) {
                init_disarm_motors();
            }else if(failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
						// TODO Change home_distance to mcstate
						// TODO wp_nav will be moved to the btree
            } else {
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }
            break;
        case AUTO:
            // failsafe_throttle is 1 do RTL, 2 means continue with the mission
            if (failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }else if(failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;
        case LOITER:
        case ALT_HOLD:
            // if landed with throttle at zero disarm, otherwise do the regular thing
            if (mincopter.rc_3.control_in == 0 && ap.land_complete) {
                init_disarm_motors();
            }else if(failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }else if(home_distance > wp_nav.get_waypoint_radius()) {
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }else{
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }
            break;
        case LAND:
            // continue to land if battery failsafe is also active otherwise fall through to default handling
            if (failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
                break;
            }
        default:
            if(failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }else if(home_distance > wp_nav.get_waypoint_radius()) {
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }else{
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
            }
            break;
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

void WP_Planner::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

void WP_Planner::failsafe_battery_event(void)
{
    // return immediately if low battery event has already been triggered
    if (failsafe.battery) {
        return;
    }

    // failsafe check
    if (failsafe_battery_enabled != FS_BATT_DISABLED && mincopter.motors.armed()) {
        switch(control_mode) {
            case STABILIZE:
            case ACRO:
            case SPORT:
                // if throttle is zero disarm motors
                if (mincopter.rc_3.control_in == 0) {
                    init_disarm_motors();
                }else{
									nav_mode = WP_FLIGHT_STATE::FS_LAND;
                }
                break;
            case AUTO:
                // set mode to RTL or LAND
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
                break;
            case LOITER:
            case ALT_HOLD:
                // if landed with throttle at zero disarm, otherwise fall through to default handling
                if (mincopter.rc_3.control_in == 0 && ap.land_complete) {
                    init_disarm_motors();
                    break;
                }
            default:
								nav_mode = WP_FLIGHT_STATE::FS_LAND;
                break;
        }
    }

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    //gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}

void WP_Planner::failsafe_gps_check()
{
    uint32_t last_gps_update_ms;

    // return immediately if gps failsafe is disabled or we have never had GPS lock
    if (failsafe_gps_enabled == FS_GPS_DISABLED || !ap.home_is_set) {
        // if we have just disabled the gps failsafe, ensure the gps failsafe event is cleared
        if (failsafe.gps) {
            set_failsafe_gps(false);
        }
        return;
    }

    // calc time since last gps update
    last_gps_update_ms = millis() - mincopter.gps_glitch.last_good_update();

    // check if all is well
    if( last_gps_update_ms < FAILSAFE_GPS_TIMEOUT_MS) {
        // check for recovery from gps failsafe
        if( failsafe.gps ) {
            set_failsafe_gps(false);
        }
        return;
    }

    // do nothing if gps failsafe already triggered or motors disarmed
    if( failsafe.gps || !mincopter.motors.armed()) {
        return;
    }

    // GPS failsafe event has occured
    // update state, warn the ground station and log to dataflash
    set_failsafe_gps(true);
    //gcs_send_text_P(SEVERITY_LOW,PSTR("Lost GPS!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on flight mode and FS_GPS_ENABLED parameter
    if (failsafe_gps_enabled == FS_GPS_ALTHOLD && !failsafe.radio) {
			nav_mode = WP_FLIGHT_STATE::FS_LOITER;
    } else {
			nav_mode = WP_FLIGHT_STATE::FS_LAND;
    }
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
int32_t WP_Planner::get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
		// An example of how this function meant to be called
		//control_yaw = get_yaw_slew(control_yaw, planner.original_wp_bearing, AUTO_YAW_SLEW_RATE);
    return wrap_360_cd(current_yaw + constrain_int16(wrap_180_cd(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}


