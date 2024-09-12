// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "navigation.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_position - get lat and lon positions from inertial nav library
void calc_position(){
    if( inertial_nav.position_ok() ) {
        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }
}

// calc_distance_and_bearing - calculate distance and direction to waypoints for reporting and autopilot decisions
void calc_distance_and_bearing()
{
    Vector3f curr = inertial_nav.get_position();

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
        update_super_simple_bearing(false);
    }
}

// run_autopilot - highest level call to process mission commands
void run_autopilot()
{
    switch( control_mode ) {
        case AUTO:
            // load the next command if the command queues are empty
            //update_commands();
						// NOTE Have removed update_commands - auto mode should get waypoints from somewhere else or based on behaviour tree

            // process the active navigation and conditional commands
            //verify_commands();
            break;
        case GUIDED:
            // no need to do anything - wp_nav should take care of getting us to the desired location
            break;
        case RTL:
						// REMOVED support for RTL
            //verify_RTL();
            break;
    }
}

// set_nav_mode - update nav mode and initialise any variables as required
bool set_nav_mode(uint8_t new_nav_mode)
{
    bool nav_initialised = false;       // boolean to ensure proper initialisation of nav modes
    Vector3f stopping_point;            // stopping point for circle mode

    // return immediately if no change
    if( new_nav_mode == nav_mode ) {
        return true;
    }

    switch( new_nav_mode ) {

        case NAV_NONE:
            nav_initialised = true;
            // initialise global navigation variables including wp_distance
            reset_nav_params();
            break;

				// REMOVED CIRCLE

        case NAV_LOITER:
            // set target to current position
            wp_nav.init_loiter_target(inertial_nav.get_position(), inertial_nav.get_velocity());
            nav_initialised = true;
            break;

        case NAV_WP:
            nav_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( nav_initialised ) {
        nav_mode = new_nav_mode;
    }

    // return success or failure
    return nav_initialised;
}

// update_nav_mode - run navigation controller based on nav_mode
// called at 100hz
void update_nav_mode()
{
    static uint8_t log_counter;     // used to slow NTUN logging

    // exit immediately if not auto_armed or inertial nav position bad
    if (!ap.auto_armed || !inertial_nav.position_ok()) {
        return;
    }

    switch( nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

				// REMOVED CIRCLE

        case NAV_LOITER:
            // reset target if we are still on the ground
            if (ap.land_complete) {
                wp_nav.init_loiter_target(inertial_nav.get_position(),inertial_nav.get_velocity());
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

    // log to dataflash at 10hz
    log_counter++;
    if (log_counter >= 10 && (g.log_bitmask & MASK_LOG_NTUN) && nav_mode != NAV_NONE) {
        log_counter = 0;
        Log_Write_Nav_Tuning();
    }
}

// Keeps old data out of our calculation / logs
void reset_nav_params(void)
{
    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by nav or loiter controllers
    lon_error                       = 0;
    lat_error                       = 0;
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
    return wrap_360_cd(current_yaw + constrain_int16(wrap_180_cd(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}

