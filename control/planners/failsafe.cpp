// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */

#include "failsafe.h"

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "log.h"
#include "motors.h"
#include "system.h"
#include "util.h"

void failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !mincopter.motors.armed() ) {
        return;
    }

    // This is how to handle a failsafe.
    switch(mincopter.control_mode) {
        case STABILIZE:
        case ACRO:
        case SPORT:
            // if throttle is zero disarm motors
            if (mincopter.rc_3.control_in == 0) {
                init_disarm_motors();
            }else if(mincopter.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
                set_mode(LAND);
						// TODO Change home_distance to mcstate
						// TODO wp_nav will be moved to the btree
            }else if(mincopter.home_distance > mcstate.wp_nav.get_waypoint_radius()) {
                if (!set_mode(RTL)) {
                    set_mode(LAND);
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
        case AUTO:
            // failsafe_throttle is 1 do RTL, 2 means continue with the mission
            if (mincopter.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                if(mincopter.home_distance > mcstate.wp_nav.get_waypoint_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    // We are very close to home so we will land
                    set_mode(LAND);
                }
            }else if(mincopter.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
            	set_mode(LAND);
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;
        case LOITER:
        case ALT_HOLD:
            // if landed with throttle at zero disarm, otherwise do the regular thing
            if (mincopter.rc_3.control_in == 0 && mincopter.ap.land_complete) {
                init_disarm_motors();
            }else if(mincopter.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
                set_mode(LAND);
            }else if(mincopter.home_distance > mcstate.wp_nav.get_waypoint_radius()) {
                if (!set_mode(RTL)) {
                    set_mode(LAND);
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
        case LAND:
            // continue to land if battery failsafe is also active otherwise fall through to default handling
            if (mincopter.failsafe_battery_enabled == FS_BATT_LAND && mcstate.failsafe.battery) {
                break;
            }
        default:
            if(mincopter.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
                set_mode(LAND);
            }else if(mincopter.home_distance > mcstate.wp_nav.get_waypoint_radius()) {
                if (!set_mode(RTL)){
                    set_mode(LAND);
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
void failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

void failsafe_battery_event(void)
{
    // return immediately if low battery event has already been triggered
    if (mcstate.failsafe.battery) {
        return;
    }

    // failsafe check
    if (mincopter.failsafe_battery_enabled != FS_BATT_DISABLED && mincopter.motors.armed()) {
        switch(mincopter.control_mode) {
            case STABILIZE:
            case ACRO:
            case SPORT:
                // if throttle is zero disarm motors
                if (mincopter.rc_3.control_in == 0) {
                    init_disarm_motors();
                }else{
                    // set mode to RTL or LAND
                    if (mincopter.failsafe_battery_enabled == FS_BATT_RTL && mincopter.home_distance > mcstate.wp_nav.get_waypoint_radius()) {
                        if (!set_mode(RTL)) {
                            set_mode(LAND);
                        }
                    }else{
                        set_mode(LAND);
                    }
                }
                break;
            case AUTO:
                // set mode to RTL or LAND
                if (mincopter.home_distance > mcstate.wp_nav.get_waypoint_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    set_mode(LAND);
                }
                break;
            case LOITER:
            case ALT_HOLD:
                // if landed with throttle at zero disarm, otherwise fall through to default handling
                if (mincopter.rc_3.control_in == 0 && mincopter.ap.land_complete) {
                    init_disarm_motors();
                    break;
                }
            default:
                // set mode to RTL or LAND
                if (mincopter.failsafe_battery_enabled == FS_BATT_RTL && mincopter.home_distance > mcstate.wp_nav.get_waypoint_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    set_mode(LAND);
                }
                break;
        }
    }

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    //gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}


// Why are these here??
static bool failsafe_enabled = true;
static uint16_t failsafe_last_mainLoop_count;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.

// in arducopter.cpp
extern uint16_t mainLoop_count;

void failsafe_check()
{
    uint32_t tnow = mincopter.hal.scheduler->micros();

    if (mainLoop_count != failsafe_last_mainLoop_count) {
        // the main loop is running, all is OK
        failsafe_last_mainLoop_count = mainLoop_count;
        failsafe_last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(mincopter.motors.armed()) {
            mincopter.motors.armed(false);
            mincopter.motors.output();
        }
    }
}

uint8_t lim_state = 0, lim_old_state = 0;

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void fence_check()
{
    uint8_t new_breaches; // the type of fence that has been breached
    uint8_t orig_breaches = mcstate.fence.get_breaches();

    // return immediately if motors are not armed
    if(!mincopter.motors.armed()) {
        return;
    }

    // give fence library our current distance from home in meters
    mcstate.fence.set_home_distance(mincopter.home_distance*0.01f);

    // check for a breach
    new_breaches = mcstate.fence.check_fence();

    // if there is a new breach take action
    if( new_breaches != AC_FENCE_TYPE_NONE ) {

        // if the user wants some kind of response and motors are armed
        if(mcstate.fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if(/* manual_flight_mode(control_mode) && */ mincopter.rc_3.control_in == 0 && !mcstate.failsafe.radio && ((mcstate.fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0)){
                init_disarm_motors();
            }else{
                // if we are within 100m of the fence, RTL
                if (mcstate.fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    // if more than 100m outside the fence just force a land
                    set_mode(LAND);
                }
            }
        }

        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);
    }

    // record clearing of breach
    if(orig_breaches != AC_FENCE_TYPE_NONE && mcstate.fence.get_breaches() == AC_FENCE_TYPE_NONE) {
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
    }
}

// failsafe_gps_check - check for gps failsafe
void failsafe_gps_check()
{
    uint32_t last_gps_update_ms;

    // return immediately if gps failsafe is disabled or we have never had GPS lock
    if (mincopter.failsafe_gps_enabled == FS_GPS_DISABLED || !mincopter.ap.home_is_set) {
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
    if (mincopter.failsafe_gps_enabled == FS_GPS_ALTHOLD && !failsafe.radio) {
    	set_mode(ALT_HOLD);
    } else {
      set_mode(LAND);
    }
}
