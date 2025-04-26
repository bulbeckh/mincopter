
#include "planner_waypoint.h"

#include "failsafe.h"

extern MCInstance mincopter;
extern MCState mcstate;

void WP_Planner::run(void)
{

	/*
	 * 1. Check failsafe and fence.
	 *
	 * 2. Check if criteria is met to move to next state (i.e. from LAND to first WP or from 4th WP to 5th WP).
	 *
	 * 3. Use planner to determine control inputs/targets.
	 */


	/* Fence Check */
	fence_check();

	/* Failsafe Check */
	//failsafe_check()

	update_nav_mode();

}

bool WP_Planner::set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !mincopter.motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
		// #TODO control_mode is part of MCInstance but will be moved either as a state variable or the btree
    if (mode == control_mode) {
        return true;
    }

    switch(mode) {

        case STABILIZE:
            success = true;
            set_yaw_mode(STABILIZE_YAW);
            set_roll_pitch_mode(STABILIZE_RP);
            set_throttle_mode(STABILIZE_THR);
            set_nav_mode(NAV_NONE);
            break;

        case AUTO:
            // check we have a GPS and at least one mission command (note the home position is always command 0)
            if (GPS_ok() || ignore_checks) {
                success = true;
                // roll-pitch, throttle and yaw modes will all be set by the first nav command
                //init_commands();            // clear the command queues. will be reloaded when "run_autopilot" calls "update_commands" function
								// NOTE removed commands - need to reconfigure how autpilot starts and runs
            }
            break;

        case LAND:
            success = true;
						// NOTE As with above, need to reconfigure how autopilot works here
            //do_land(NULL);  // land at current location
            break;

        default:
            success = false;
            break;
    }

    // update flight mode
    if (success) {
        control_mode = mode;
        //Log_Write_Mode(control_mode);
    }else{
        // Log error that we failed to enter desired flight mode
        //Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
    }

    // return success or failure
    return success;
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// returns true if we have landed
bool WP_Planner::update_land_detector()
{
    // detect whether we have landed by watching for low climb rate and minimum throttle
    if (abs(climb_rate) < 20 && mincopter->motors.limit.throttle_lower) {
        if (!ap.land_complete) {
            // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = 0;
            }
        }
    }else if (mincopter->rc_3.control_in != 0 || failsafe.radio){    // zero throttle locks land_complete as true
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
    if( nav_mode == NAV_LOITER || nav_mode == NAV_CIRCLE ) {
        wp_distance = wp_nav.get_distance_to_target();
        wp_bearing = wp_nav.get_bearing_to_target();
    }else if( nav_mode == NAV_WP ) {
        wp_distance = wp_nav.get_distance_to_destination();
        wp_bearing = wp_nav.get_bearing_to_destination();
    }else{
        wp_distance = 0;
        wp_bearing = 0;
    }

    // calculate home distance and bearing
    if(GPS_ok()) {
        home_distance = pythagorous2(curr.x, curr.y);
        home_bearing = pv_get_bearing_cd(curr,Vector3f(0,0,0));

        // update super simple bearing (if required) because it relies on home_bearing
        //update_super_simple_bearing(false);
    }
}

//update_nav_mode - run navigation controller based on nav_mode
// called at 100hz
void WP_Planner::update_nav_mode()
{
    // exit immediately if not auto_armed or inertial nav position bad
    if (!ap.auto_armed || !mcstate.inertial_nav.position_ok()) {
        return;
    }

    switch( nav_mode ) {

        case NAV_LOITER:
            // reset target if we are still on the ground
            if (ap.land_complete) {
                wp_nav.init_loiter_target(mcstate.inertial_nav.get_position(),mcstate.inertial_nav.get_velocity());
            }else{
                // call loiter controller
                wp_nav.update_loiter();
            }
            break;

        case NAV_WP:
            // call waypoint controller
            wp_nav.update_wpnav();
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

    linear_velocity = ALT_HOLD_ACCEL_MAX/pi_alt_hold.kP();

    if (abs(climb_rate_cms) < linear_velocity) {
        target_alt = alt_cm + climb_rate_cms/pi_alt_hold.kP();
    } else {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*pi_alt_hold.kP()*pi_alt_hold.kP());
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
		mincopter.controller_desired_alt = get_initial_alt_hold(mcstate.current_loc.alt, mincopter.climb_rate);     // reset controller desired altitude to current altitude
    mcstate.wp_nav.set_desired_alt(mincopter.controller_desired_alt);                                 // same as above but for loiter controller
    throttle_initialised = true;

		/* LANDING case
            reset_land_detector();  // initialise land detector
            mincopter.controller_desired_alt = get_initial_alt_hold(mcstate.current_loc.alt, mincopter.climb_rate);   // reset controller desired altitude to current altitude
            throttle_initialised = true;
            break;
		*/

    return throttle_initialised;
}

// set_yaw_mode - update yaw mode and initialise any variables required
bool WP_Planner::init_yaw(uint8_t new_yaw_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool yaw_initialised = false;

    // return immediately if no change
    if( new_yaw_mode == mincopter.yaw_mode ) {
        return true;
    }

    switch( new_yaw_mode ) {
        case YAW_HOLD:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_NEXT_WP:
            if( mincopter.ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_LOCATION:
            if( mincopter.ap.home_is_set ) {
                // update bearing - assumes yaw_look_at_WP has been intialised before set_yaw_mode was called
                mincopter.yaw_look_at_WP_bearing = pv_get_bearing_cd(mcstate.inertial_nav.get_position(), mincopter.yaw_look_at_WP);
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_HEADING:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_HOME:
            if( mincopter.ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AHEAD:
            if( mincopter.ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_RESETTOARMEDYAW:
            mcstate.control_yaw = mcstate.ahrs.yaw_sensor; // store current yaw so we can start rotating back to correct one
            yaw_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( yaw_initialised ) {
        mincopter.yaw_mode = new_yaw_mode;
    }

    // return success or failure
    return yaw_initialised;
}


