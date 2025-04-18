

#include "planner_waypoint.h"



void WP_Planner::run(void)
{


}

bool WP_Planner::set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !mincopter.motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
		// #TODO control_mode is part of MCInstance but will be moved either as a state variable or the btree
    if (mode == mincopter.control_mode) {
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
        mincopter.control_mode = mode;
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
    if (abs(mincopter->climb_rate) < 20 && mincopter->motors.limit.throttle_lower) {
        if (!mincopter->ap.land_complete) {
            // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = 0;
            }
        }
    }else if (mincopter->rc_3.control_in != 0 || state->failsafe.radio){    // zero throttle locks land_complete as true
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        if(mincopter->ap.land_complete) {
            set_land_complete(false);
        }
    }

    // return current state of landing
    return mincopter->ap.land_complete;
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
    if( mincopter.nav_mode == NAV_LOITER || mincopter.nav_mode == NAV_CIRCLE ) {
        mincopter.wp_distance = mcstate.wp_nav.get_distance_to_target();
        mincopter.wp_bearing = mcstate.wp_nav.get_bearing_to_target();
    }else if( mincopter.nav_mode == NAV_WP ) {
        mincopter.wp_distance = mcstate.wp_nav.get_distance_to_destination();
        mincopter.wp_bearing = mcstate.wp_nav.get_bearing_to_destination();
    }else{
        mincopter.wp_distance = 0;
        mincopter.wp_bearing = 0;
    }

    // calculate home distance and bearing
    if(GPS_ok()) {
        mincopter.home_distance = pythagorous2(curr.x, curr.y);
        mincopter.home_bearing = pv_get_bearing_cd(curr,Vector3f(0,0,0));

        // update super simple bearing (if required) because it relies on home_bearing
        //update_super_simple_bearing(false);
    }
}

//update_nav_mode - run navigation controller based on nav_mode
// called at 100hz
void WP_Planner::update_nav_mode()
{
    static uint8_t log_counter;     // used to slow NTUN logging

    // exit immediately if not auto_armed or inertial nav position bad
    if (!mincopter.ap.auto_armed || !mcstate.inertial_nav.position_ok()) {
        return;
    }

    switch( mincopter.nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

				// REMOVED CIRCLE

        case NAV_LOITER:
            // reset target if we are still on the ground
            if (mincopter.ap.land_complete) {
                mcstate.wp_nav.init_loiter_target(mcstate.inertial_nav.get_position(),mcstate.inertial_nav.get_velocity());
            }else{
                // call loiter controller
                mcstate.wp_nav.update_loiter();
            }
            break;

        case NAV_WP:
            // call waypoint controller
            mcstate.wp_nav.update_wpnav();
            break;
    }

    // log to dataflash at 10hz
    log_counter++;
    if (log_counter >= 10 && (mincopter.log_bitmask & MASK_LOG_NTUN) && mincopter.nav_mode != NAV_NONE) {
        log_counter = 0;
        Log_Write_Nav_Tuning();
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
