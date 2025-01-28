
#include "control_modes.h"

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "log.h"
#include "attitude.h"
#include "navigation.h"
#include "util.h"


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
						/* REMOVED 
            if (throttle_mode_manual(throttle_mode)) {  // reset the alt hold I terms if previous throttle mode was manual
                reset_throttle_I();
                set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
            }
						*/
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

// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
void update_throttle_mode(void)
{
    int16_t pilot_climb_rate;
    int16_t pilot_throttle_scaled;

    // do not run throttle controllers if motors disarmed
    if( !mincopter.motors.armed() ) {
        set_throttle_out(0, false);
        throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        return;
    }

    switch(mincopter.throttle_mode) {

    case THROTTLE_AUTO:
        // auto pilot altitude controller with target altitude held in wp_nav.get_desired_alt()
        if(mincopter.ap.auto_armed) {
            // special handling if we are just taking off
            if (mincopter.ap.land_complete) {
                // tell motors to do a slow start.
                mincopter.motors.slow_start(true);
            }
            get_throttle_althold_with_slew(mcstate.wp_nav.get_desired_alt(), -mcstate.wp_nav.get_descent_velocity(), mcstate.wp_nav.get_climb_velocity());
        }else{
            // pilot's throttle must be at zero so keep motors off
            set_throttle_out(0, false);
            // deactivate accel based throttle controller
            throttle_accel_deactivate();
        }
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
        break;
    }
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
            reset_roll_pitch_in_filters(mincopter.g.rc_1.control_in, mincopter.g.rc_2.control_in);
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

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void update_yaw_mode(void)
{
		// TODO Remove for autonomous flight
    int16_t pilot_yaw = mincopter.g.rc_4.control_in;

    // do not process pilot's yaw input during radio failsafe
    if (mcstate.failsafe.radio) {
        pilot_yaw = 0;
    }

    switch(mincopter.yaw_mode) {

		/* NOTE REMOVE DUE TO ACRO VARIABLE IN get_yaw_rate_stabilized_ef
    case YAW_HOLD:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // heading hold at heading held in control_yaw but allow input from pilot
        get_yaw_rate_stabilized_ef(pilot_yaw);
        break;
		*/

		/* REMOVE ACRO
    case YAW_ACRO:
        // pilot controlled yaw using rate controller
        get_yaw_rate_stabilized_bf(pilot_yaw);
        break;
		*/

    case YAW_LOOK_AT_NEXT_WP:
        // if we are landed reset yaw target to current heading
        if (mincopter.ap.land_complete) {
            mcstate.control_yaw = mcstate.ahrs.yaw_sensor;
        }else{
            // point towards next waypoint (no pilot input accepted)
            // we don't use wp_bearing because we don't want the copter to turn too much during flight
            mcstate.control_yaw = get_yaw_slew(mcstate.control_yaw, mincopter.original_wp_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(mcstate.control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_LOCATION:
        // if we are landed reset yaw target to current heading
        if (mincopter.ap.land_complete) {
            mcstate.control_yaw = mcstate.ahrs.yaw_sensor;
        }
        // point towards a location held in yaw_look_at_WP
        get_look_at_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

		// REMOVED get_circle_yaw

    case YAW_LOOK_AT_HOME:
        // if we are landed reset yaw target to current heading
        if (mincopter.ap.land_complete) {
            mcstate.control_yaw = mcstate.ahrs.yaw_sensor;
        }else{
            // keep heading always pointing at home with no pilot input allowed
            mcstate.control_yaw = get_yaw_slew(mcstate.control_yaw, mincopter.home_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(mcstate.control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HEADING:
        // if we are landed reset yaw target to current heading
        if (mincopter.ap.land_complete) {
            mcstate.control_yaw = mcstate.ahrs.yaw_sensor;
        }else{
            // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
            mcstate.control_yaw = get_yaw_slew(mcstate.control_yaw, mincopter.yaw_look_at_heading, mincopter.yaw_look_at_heading_slew);
        }
        get_stabilize_yaw(mcstate.control_yaw);
        break;

	case YAW_LOOK_AHEAD:
        // if we are landed reset yaw target to current heading
        if (mincopter.ap.land_complete) {
            mcstate.control_yaw = mcstate.ahrs.yaw_sensor;
        }
		// Commanded Yaw to automatically look ahead.
        get_look_ahead_yaw(pilot_yaw);
        break;

    case YAW_RESETTOARMEDYAW:
        // if we are landed reset yaw target to current heading
        if (mincopter.ap.land_complete) {
            mcstate.control_yaw = mcstate.ahrs.yaw_sensor;
        }else{
            // changes yaw to be same as when quad was armed
            mcstate.control_yaw = get_yaw_slew(mcstate.control_yaw, mincopter.initial_armed_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(mcstate.control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }

        break;
    }
}
// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
void update_roll_pitch_mode(void)
{
    switch(mincopter.roll_pitch_mode) {
		// NO ACRO MODE

		// NO manual modes
    case ROLL_PITCH_AUTO:
				// Get control roll/pitch from the waypoint controller
        mcstate.control_roll = mcstate.wp_nav.get_desired_roll();
        mcstate.control_pitch = mcstate.wp_nav.get_desired_pitch();

        get_stabilize_roll(mcstate.control_roll);
        get_stabilize_pitch(mcstate.control_pitch);
        break;
    }

    if(mincopter.g.rc_3.control_in == 0 && mincopter.control_mode <= ACRO) {
        reset_rate_I();
    }

    if(mincopter.ap.new_radio_frame) {
        // clear new radio frame info
        mincopter.ap.new_radio_frame = false;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
void save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)mincopter.g.rc_1.control_in/100.0f);
    float pitch_trim = ToRad((float)mincopter.g.rc_2.control_in/100.0f);
    mcstate.ahrs.add_trim(roll_trim, pitch_trim);
    Log_Write_Event(DATA_SAVE_TRIM);
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
void auto_trim()
{
    if(mincopter.auto_trim_counter > 0) {
        mincopter.auto_trim_counter--;

        // flash the leds
        AP_Notify::flags.save_trim = true;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)mincopter.g.rc_1.control_in / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)mincopter.g.rc_2.control_in / 4000.0f);

        // make sure accelerometer values impact attitude quickly
        mcstate.ahrs.set_fast_gains(true);

        // add trim to ahrs object
        // save to eeprom on last iteration
        mcstate.ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (mincopter.auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if(mincopter.auto_trim_counter == 0) {
            mcstate.ahrs.set_fast_gains(false);
            AP_Notify::flags.save_trim = false;
        }
    }
}

