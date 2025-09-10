

/* Failsafe functions from Waypoint Planner WP_Planner */

#include "planner_waypoint.h"

#include "util.h"
#include "log.h"

#include "mcinstance.h"
extern MCInstance mincopter;

#include "mcstate.h"
extern MCState mcstate;

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
