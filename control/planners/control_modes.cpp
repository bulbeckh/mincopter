
#include "control_modes.h"

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "log.h"
#include "attitude.h"
#include "navigation.h"
#include "util.h"

#include "config.h"


// get_initial_alt_hold - get new target altitude based on current altitude and climb rate
int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms)
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

// set_throttle_mode - sets the throttle mode and initialises any variables as required
bool set_throttle_mode( uint8_t new_throttle_mode )
{
    // boolean to ensure proper initialisation of throttle modes
    bool throttle_initialised = false;

    // return immediately if no change
    if( new_throttle_mode == mincopter.throttle_mode ) {
        return true;
    }

    // initialise any variables required for the new throttle mode
    switch(new_throttle_mode) {

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
            mincopter.controller_desired_alt = get_initial_alt_hold(mcstate.current_loc.alt, mincopter.climb_rate);     // reset controller desired altitude to current altitude
            mcstate.wp_nav.set_desired_alt(mincopter.controller_desired_alt);                                 // same as above but for loiter controller
            throttle_initialised = true;
            break;

        case THROTTLE_LAND:
            reset_land_detector();  // initialise land detector
            mincopter.controller_desired_alt = get_initial_alt_hold(mcstate.current_loc.alt, mincopter.climb_rate);   // reset controller desired altitude to current altitude
            throttle_initialised = true;
            break;
    }

    // update the throttle mode
    if( throttle_initialised ) {
        mincopter.throttle_mode = new_throttle_mode;

        // reset some variables used for logging
        mincopter.desired_climb_rate = 0;
        mincopter.nav_throttle = 0;
    }

    // return success or failure
    return throttle_initialised;
}


// set_roll_pitch_mode - update roll/pitch mode and initialise any variables as required
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool roll_pitch_initialised = false;

    // return immediately if no change
    if( new_roll_pitch_mode == mincopter.roll_pitch_mode ) {
        return true;
    }

    switch( new_roll_pitch_mode ) {
        case ROLL_PITCH_STABLE:
            reset_roll_pitch_in_filters(mincopter.rc_1.control_in, mincopter.rc_2.control_in);
            roll_pitch_initialised = true;
            break;
        case ROLL_PITCH_AUTO:
            roll_pitch_initialised = true;
            break;

    }

    // if initialisation has been successful update the yaw mode
    if( roll_pitch_initialised ) {
        mincopter.roll_pitch_mode = new_roll_pitch_mode;
    }

    // return success or failure
    return roll_pitch_initialised;
}

// set_yaw_mode - update yaw mode and initialise any variables required
bool set_yaw_mode(uint8_t new_yaw_mode)
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


