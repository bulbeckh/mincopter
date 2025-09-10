/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds

#include "planner.h"

#include "mcinstance.h"
#include "mcstate.h"

#include "control.h"

extern MCInstance mincopter;
extern MCState mcstate;


#include "log.h"
#include "radio.h"
#include "util.h"

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
// called at 1hz
// TODO Remove this whole function - no need for auto-disarm in autonomous modes
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

    // disable inertial nav errors temporarily
    //mcstate.inertial_nav.ignore_next_error();

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
    if (mincopter.hal.uartC != NULL) mincopter.hal.uartC->set_blocking_writes(false);
    if (mincopter.hal.uartD != NULL) mincopter.hal.uartD->set_blocking_writes(false);

    // Remember Orientation
    // --------------------
    //init_simple_bearing();

    // Set home position
    // -------------------
    init_home();
    calc_distance_and_bearing();

	mincopter.compass.set_initial_location(mincopter.g_gps->latitude, mincopter.g_gps->longitude);

    // all I terms are invalid
    // -----------------------
#if CONTROLLER_PID
    controller.reset_I_all();
#endif

		// TODO Removed because startup_ground function missing/removed. Investigate further
		/*
    if(did_ground_start == false) {
        did_ground_start = true;
        startup_ground(true);
    }
		*/

#if HIL_MODE != HIL_MODE_ATTITUDE
    // fast baro calibration to reset ground pressure
    init_barometer(false);
#endif

	// TODO Remove these two mcstate function calls
	
    // go back to normal AHRS gains
    //mcstate.ahrs.set_fast_gains(false);

    // enable gps velocity based centrefugal force compensation
    //mcstate.ahrs.set_correct_centrifugal(true);

    // set hover throttle
    //mincopter.motors.set_mid_throttle(mincopter.throttle_mid);

    // Cancel arming if throttle is raised too high so that copter does not suddenly take off
    //read_radio();
	
	// TODO This breaks the controller abstraction - need to fix. For now just adding a directive as workaround
#if CONTROLLER_PID
    if (mincopter.rc_3.control_in > controller.throttle_cruise && controller.throttle_cruise > 100) {
        mincopter.motors.output_min();
        return;
    }
#endif

    // enable output to motors
	init_rc_out();
	init_esc();

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

}

// perform pre-arm checks and set ap.pre_arm_check flag
void WP_Planner::pre_arm_checks(void)
{
    // exit immediately if we've already successfully performed the pre-arm check
    if (ap.pre_arm_check) {
        return;
    }

    // succeed if pre arm checks are disabled
    if(mincopter.arming_check == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        return;
    }

    // check Baro
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_BARO)) {
        // barometer health check
        if(!mincopter.barometer.healthy) {
            return;
        }
        // check Baro & inav alt are within 1m
        if(fabs(mcstate.get_altitude() - baro_alt) > 100) {
            return;
        }
    }

    // check Compass
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_COMPASS)) {
        // check the compass is healthy
        if(!mincopter.compass.healthy()) {
            return;
        }

        // check compass learning is on or offsets have been set
        Vector3f offsets = mincopter.compass.get_offsets();
		/*
        if(offsets.length() == 0) {
            return;
        }
		*/

        // check for unreasonable compass offsets
        if(offsets.length() > 500) {
            return;
        }

        // check for unreasonable mag field length
        float mag_field = mincopter.compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65 || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35) {
            return;
        }
    }

    // check GPS
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_GPS)) {
        // check gps is ok if required - note this same check is repeated again in arm_checks

#if AC_FENCE == ENABLED
        // check fence is initialised
        if(!fence.pre_arm_check() || (((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) && !pre_arm_gps_checks())) {
            return;
        }
#endif
    }

    // check INS
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if(!mincopter.ins.calibrated()) {
            return;
        }

        // check accels and gyros are healthy
        if(!mincopter.ins.get_gyro_health() || !mincopter.ins.get_accel_health()) {
            return;
        }
    }

    // check various parameter values
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_PARAMETERS)) {

        // failsafe parameter checks
        if (failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (mincopter.rc_3.radio_min <= failsafe_throttle_value+10 || failsafe_throttle_value < 910) {
                return;
            }
        }

        // lean angle parameter check
        if (angle_max < 1000 || angle_max > 8000) {
            return;
        }
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(true);
}

// performs pre_arm gps related checks and returns true if passed
bool WP_Planner::pre_arm_gps_checks(void)
{
    float speed_cms = mcstate.get_velocity().length();     // speed according to inertial nav in cm/s

    // ensure GPS is ok and our speed is below 50cm/s
    if (!GPS_ok() || mincopter.gps_glitch.glitching() || speed_cms == 0 || speed_cms > PREARM_MAX_VELOCITY_CMS) {
        return false;
    }

    // if we got here all must be ok
    return true;
}

// arm_checks - perform final checks before arming
// always called just before arming.  Return true if ok to arm
bool WP_Planner::arm_checks(void)
{
    // succeed if arming checks are disabled
    if (mincopter.arming_check == ARMING_CHECK_NONE) {
        return true;
    }

    // check Baro & inav alt are within 1m
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_BARO)) {
        if(fabs(mcstate.get_altitude() - baro_alt) > 100) {
            return false;
        }
    }

    // check gps is ok if required - note this same check is also done in pre-arm checks
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_GPS)) {
    }

    // check parameters
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_PARAMETERS)) {
        // check throttle is above failsafe throttle
        if (failsafe_throttle != FS_THR_DISABLED && mincopter.rc_3.radio_in < failsafe_throttle_value) {
            return false;
        }
    }

    // check lean angle
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_INS)) {
        if (labs(mcstate.roll_sensor) > angle_max || labs(mcstate.pitch_sensor) > angle_max) {
            return false;
        }
    }

    // check if safety switch has been pushed
    if (mincopter.hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
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
    //mcstate.inertial_nav.ignore_next_error();

    // we are not in the air
    set_takeoff_complete(false);
    
    // setup fast AHRS gains to get right attitude
    //mcstate.ahrs.set_fast_gains(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // suspend logging
		// TODO why is this camelcase - change all sensor objects to lowercase
    mincopter.DataFlash.EnableWrites(false);

    // disable gps velocity based centrefugal force compensation
    //mcstate.ahrs.set_correct_centrifugal(false);
}

