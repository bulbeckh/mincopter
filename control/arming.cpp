/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds

#include "planner_waypoint.h"

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "log.h"
#include "init.h"
#include "radio.h"
#include "util.h"


// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz

/* REMOVE
void WP_Planner::arm_motors_check()
{
    static int16_t arming_counter;
    bool allow_arming = false;

    // ensure throttle is down
    if (mincopter.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    // allow arming/disarming in Loiter and AltHold if landed
    if (mincopter.ap.land_complete && (mincopter.control_mode == LOITER || mincopter.control_mode == ALT_HOLD)) {
        allow_arming = true;
    }

    // kick out other flight modes
    if (!allow_arming) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = mincopter.rc_4.control_in;

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !mincopter.motors.armed()) {
            // run pre-arm-checks and display failures
            pre_arm_checks(true);
            if(mincopter.ap.pre_arm_check && arm_checks(true)) {
                init_arm_motors();
            }else{
                // reset arming counter if pre-arm checks fail
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && mincopter.motors.armed() && mincopter.control_mode == STABILIZE) {
            mincopter.auto_trim_counter = 250;
        }

    // full left
    }else if (tmp < -4000) {

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && mincopter.motors.armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}
*/

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
// called at 1hz
void WP_Planner::auto_disarm_check()
{
    static uint8_t auto_disarming_counter;

    // exit immediately if we are already disarmed or throttle is not zero
    if (!mincopter.motors.armed() || mincopter.rc_3.control_in > 0) {
        auto_disarming_counter = 0;
        return;
    }

    // allow auto disarm in manual flight modes or Loiter/AltHold if we're landed
    if(ap.land_complete && (control_mode == LOITER || control_mode == ALT_HOLD)) {
        auto_disarming_counter++;

        if(auto_disarming_counter >= AUTO_DISARMING_DELAY) {
            init_disarm_motors();
            auto_disarming_counter = 0;
        }
    }else{
        auto_disarming_counter = 0;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
void WP_Planner::init_arm_motors()
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // disable inertial nav errors temporarily
    mcstate.inertial_nav.ignore_next_error();

#if LOGGING_ENABLED == ENABLED
    // start dataflash
    start_logging();
#endif

/*
#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif
*/

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
    mincopter.hal.uartA->set_blocking_writes(false);
    mincopter.hal.uartC->set_blocking_writes(false);
    if (mincopter.hal.uartD != NULL) {
        mincopter.hal.uartD->set_blocking_writes(false);
    }

    // Remember Orientation
    // --------------------
    //init_simple_bearing();

    initial_armed_bearing = mcstate.ahrs.yaw_sensor;

    // Reset home position
    // -------------------
    if (ap.home_is_set) {
        init_home();
        calc_distance_and_bearing();
    }

    // all I terms are invalid
    // -----------------------
    reset_I_all();

    if(did_ground_start == false) {
        did_ground_start = true;
        startup_ground(true);
    }

#if HIL_MODE != HIL_MODE_ATTITUDE
    // fast baro calibration to reset ground pressure
    init_barometer(false);
#endif

    // go back to normal AHRS gains
    mcstate.ahrs.set_fast_gains(false);

    // enable gps velocity based centrefugal force compensation
    mcstate.ahrs.set_correct_centrifugal(true);

    // set hover throttle
    //mincopter.motors.set_mid_throttle(mincopter.throttle_mid);

    // Cancel arming if throttle is raised too high so that copter does not suddenly take off
    //read_radio();
    if (mincopter.rc_3.control_in > throttle_cruise && throttle_cruise > 100) {
        mincopter.motors.output_min();
        failsafe_enable();
        return;
    }

    // enable output to motors
    output_min();

    // finally actually arm the motors
    mincopter.motors.armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // reenable failsafe
    failsafe_enable();
}

// perform pre-arm checks and set ap.pre_arm_check flag
void WP_Planner::pre_arm_checks(bool display_failure)
{
    // exit immediately if we've already successfully performed the pre-arm check
    if (ap.pre_arm_check) {
        return;
    }

    // succeed if pre arm checks are disabled
    if(mincopter.arming_check == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        set_pre_arm_rc_check(true);
        return;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        if (display_failure) {
            //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: RC not calibrated"));
        }
        return;
    }

    // check Baro
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_BARO)) {
        // barometer health check
        if(!mincopter.barometer.healthy) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Baro not healthy"));
            }
            return;
        }
        // check Baro & inav alt are within 1m
        if(fabs(mcstate.inertial_nav.get_altitude() - baro_alt) > 100) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Alt disparity"));
            }
            return;
        }
    }

    // check Compass
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_COMPASS)) {
        // check the compass is healthy
        if(!mincopter.compass.healthy()) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy"));
            }
            return;
        }

        // check compass learning is on or offsets have been set
        Vector3f offsets = mincopter.compass.get_offsets();
        if(!mincopter.compass._learn && offsets.length() == 0) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
            }
            return;
        }

        // check for unreasonable compass offsets
        if(offsets.length() > 500) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass offsets too high"));
            }
            return;
        }

        // check for unreasonable mag field length
        float mag_field = mincopter.compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65 || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check mag field"));
            }
            return;
        }
    }

    // check GPS
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_GPS)) {
        // check gps is ok if required - note this same check is repeated again in arm_checks
        if (!pre_arm_gps_checks(display_failure)) {
            return;
        }

#if AC_FENCE == ENABLED
        // check fence is initialised
        if(!fence.pre_arm_check() || (((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) && !pre_arm_gps_checks(display_failure))) {
            return;
        }
#endif
    }

    // check INS
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if(!mincopter.ins.calibrated()) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not calibrated"));
            }
            return;
        }

        // check accels and gyros are healthy
        if(!mincopter.ins.healthy()) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not healthy"));
            }
            return;
        }
    }

    // check various parameter values
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_PARAMETERS)) {

        // failsafe parameter checks
        if (failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (mincopter.rc_3.radio_min <= failsafe_throttle_value+10 || failsafe_throttle_value < 910) {
                if (display_failure) {
                    //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check FS_THR_VALUE"));
                }
                return;
            }
        }

        // lean angle parameter check
        if (angle_max < 1000 || angle_max > 8000) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check ANGLE_MAX"));
            }
            return;
        }
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(true);
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
void WP_Planner::pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.pre_arm_rc_check ) {
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((mincopter.arming_check != ARMING_CHECK_ALL) && !(mincopter.arming_check & ARMING_CHECK_RC)) {
        set_pre_arm_rc_check(true);
        return;
    }

    // check if radio has been calibrated
		/*
    if(!mincopter.rc_3.radio_min.load() && !mincopter.rc_3.radio_max.load()) {
        return;
    }
		*/

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (mincopter.rc_1.radio_min > 1300 || mincopter.rc_1.radio_max < 1700 || mincopter.rc_2.radio_min > 1300 || mincopter.rc_2.radio_max < 1700) {
        return;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (mincopter.rc_3.radio_min > 1300 || mincopter.rc_3.radio_max < 1700 || mincopter.rc_4.radio_min > 1300 || mincopter.rc_4.radio_max < 1700) {
        return;
    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(true);
}

// performs pre_arm gps related checks and returns true if passed
bool WP_Planner::pre_arm_gps_checks(bool display_failure)
{
    float speed_cms = mcstate.inertial_nav.get_velocity().length();     // speed according to inertial nav in cm/s

    // ensure GPS is ok and our speed is below 50cm/s
    if (!GPS_ok() || mincopter.gps_glitch.glitching() || speed_cms == 0 || speed_cms > PREARM_MAX_VELOCITY_CMS) {
        if (display_failure) {
            //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Bad GPS Pos"));
        }
        return false;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (mincopter.g_gps->hdop > mincopter.gps_hdop_good) {
        if (display_failure) {
            //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: High GPS HDOP"));
        }
        return false;
    }

    // if we got here all must be ok
    return true;
}

// arm_checks - perform final checks before arming
// always called just before arming.  Return true if ok to arm
bool WP_Planner::arm_checks(bool display_failure)
{
    // succeed if arming checks are disabled
    if (mincopter.arming_check == ARMING_CHECK_NONE) {
        return true;
    }

    // check Baro & inav alt are within 1m
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_BARO)) {
        if(fabs(mcstate.inertial_nav.get_altitude() - baro_alt) > 100) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Alt disparity"));
            }
            return false;
        }
    }

    // check gps is ok if required - note this same check is also done in pre-arm checks
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_GPS)) {
        if (!pre_arm_gps_checks(display_failure)) {
            return false;
        }
    }

    // check parameters
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_PARAMETERS)) {
        // check throttle is above failsafe throttle
        if (failsafe_throttle != FS_THR_DISABLED && mincopter.rc_3.radio_in < failsafe_throttle_value) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Thr below FS"));
            }
            return false;
        }
    }

    // check lean angle
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_INS)) {
        if (labs(mcstate.ahrs.roll_sensor) > angle_max || labs(mcstate.ahrs.pitch_sensor) > angle_max) {
            if (display_failure) {
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Leaning"));
            }
            return false;
        }
    }

    // check if safety switch has been pushed
    if (mincopter.hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (display_failure) {
            //gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Safety Switch"));
        }
        return false;
    }

    // if we've gotten this far all is ok
    return true;
}

// init_disarm_motors - disarm motors
void WP_Planner::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!mincopter.motors.armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    //gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    mincopter.motors.armed(false);

    // disable inertial nav errors temporarily
    mcstate.inertial_nav.ignore_next_error();

    // we are not in the air
    set_takeoff_complete(false);
    
    // setup fast AHRS gains to get right attitude
    mcstate.ahrs.set_fast_gains(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // suspend logging
		// TODO why is this camelcase - change all sensor objects to lowercase
    mincopter.DataFlash.EnableWrites(false);

    // disable gps velocity based centrefugal force compensation
    mcstate.ahrs.set_correct_centrifugal(false);
}

