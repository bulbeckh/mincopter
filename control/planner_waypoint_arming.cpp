
#include "planner.h"

#include "mcinstance.h"
#include "mcstate.h"

#include "control.h"

extern MCInstance mincopter;

#include "log.h"
#include "radio.h"
#include "util.h"

void WP_Planner::init_arm_motors(void)
{

#if defined(LOGGING_ENABLED)
    start_logging();
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
	// TODO NOTE UART writes should never block anyway - REMOVE
    mincopter.hal.uartA->set_blocking_writes(false);
    if (mincopter.hal.uartC != NULL) mincopter.hal.uartC->set_blocking_writes(false);
    if (mincopter.hal.uartD != NULL) mincopter.hal.uartD->set_blocking_writes(false);

	// TODO NOTE have moved home initialisation to state
    // Set home position
    //init_home();

	// TODO This should be part of a different interface - the one that pushes the 
    //calc_distance_and_bearing();

	// Give location to compass so it can set magnetic declination
	mincopter.compass.set_initial_location(mincopter.g_gps->latitude, mincopter.g_gps->longitude);

	// TODO Add reset of PID/CSC controller I terms (and any other persistent state) here

	// TODO Maybe we should move this to the **init_home** function
	// Set the current pressure/temperature as the ground pressure/ground temperature
	mincopter.barometer.update_calibration();

	// TODO We don't even use the motor or rc_* interfaces anymore. The mixer sends the correct control signal directly to the
	// appropriate motors based on the configuration. We will eventually need some sort of custom motor/mixer configuration
	// but for now this should suffice
	
    // enable output to motors
	//init_rc_out();
	//init_esc();

	// Update arm state to armed
	planner.ap.arm_active = 1;
	
    // log arming to dataflash
    //Log_Write_Event(DATA_ARMED);

	mincopter.hal.console->printf("ARMED\r\n");
	return;
}

void WP_Planner::init_disarm_motors(void)
{
	// Disarm
	planner.ap.arm_active = 0;

    // Return immediately if we are already disarmed
    //if (!mincopter.motors.armed()) return;

	// Set motor arm to false
    //mincopter.motors.armed(false);

	// TODO We should change the land/takeoff to a flight state [landed, takeoff, flying]
    // we are not in the air
    //planner.ap.takeoff_complete = false;
    
    // Log disarm to the dataflash
	//Log_Write_Event(DATA_DISARMED);

    // suspend logging
		// TODO why is this camelcase - change all sensor objects to lowercase
    //mincopter.DataFlash.EnableWrites(false);

    // disable gps velocity based centrefugal force compensation
    //mcstate.ahrs.set_correct_centrifugal(false);
	
	// Log disarmed status to console
	mincopter.hal.console->printf("DISARMED\r\n");

	return;
}

bool WP_Planner::arm_checks(void)
{
	// TODO Add console logging when we fail an arm check
	
	// 1. Barometer Check
	//
	//
	
	// barometer health check
	if(!mincopter.barometer.healthy) {
		mincopter.hal.console->printf("arm check - bad barometer\r\n");
		return false;
	}

	// TODO Even though we should have a similar test to ensure our state estimation hasn't diverged too far from ground
	// truths like the barometer altitude, that test should probably be done in mcstate rather than as an arm check/failsafe check
	
	/*
	// check Baro & inav alt are within 1m
	if(fabs(mcstate.get_altitude() - baro_alt) > 100) {
		mincopter.hal.console->printf("arm check -  baro altitude check\r\n");
		return false;
	}
	*/

	// 2. Compass Check
	//
	//

	// check the compass is healthy
	if(!mincopter.compass.healthy()) {
		mincopter.hal.console->printf("arm check - bad compass\r\n");
		return false;
	}

	// check compass learning is on or offsets have been set
	Vector3f offsets = mincopter.compass.get_offsets();
	/*
	if(offsets.length() == 0) {
		return false;
	}
	*/

	// TODO Re-enable this check - giving offsets of all zero - meaning compass is uncalibrated
	// check for unreasonable compass offsets
	/*
	if(offsets.length() > 500 || offsets.length() == 0) {
		mincopter.hal.console->printf("arm check - bad compass offsets (%f,%f,%f)\r\n", offsets.x, offsets.y, offsets.z);
		return false;
	}
	*/

	// TODO Re-enable
	// check for unreasonable mag field length
	/*
	float mag_field = mincopter.compass.get_field().length();
	if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65 || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35) {
		mincopter.hal.console->printf("arm check - unreasonable mag field\r\n");
		return false;
	}
	*/

	// 3. GPS Check
	//
	//
	
	float speed_cms = mcstate.get_velocity().length();     // speed according to inertial nav in cm/s
	
	// TODO Re-enable - currently not using GPS
	// ensure GPS is ok and our speed is below 50cm/s
	/*
	if (!GPS_ok() || mincopter.gps_glitch.glitching() || speed_cms == 0 || speed_cms > PREARM_MAX_VELOCITY_CMS) {
		mincopter.hal.console->printf("arm check - unhealthy GPS\r\n");
		return false;
	}
	*/

	// TODO We need to perform fence checks here but unsure where AP_Fence class should reside
	// 4. Fence Check
	//
	//
	
#if AC_FENCE == ENABLED
	// check fence is initialised
	/*
	if(!fence.pre_arm_check() || (((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0))) {
		return false;
	}
	*/
#endif

    // 5. INS Check
	//
	//
	
	// TODO This is a bad test for calibration
#if !defined(TARGET_ARCH_LINUX)
	// check accelerometers have been calibrated
	if(!mincopter.ins.calibrated()) {
		mincopter.hal.console->printf("arm check - uncalibrated accel\r\n");
		return false;
	}
#endif

	// check accels and gyros are healthy
	if(!mincopter.ins.get_gyro_health() || !mincopter.ins.get_accel_health()) {
		mincopter.hal.console->printf("arm check - bad ins\r\n");
		return false;
	}

	// TODO As above - move to mcstate
	/*
    // check Baro & inav alt are within 1m
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_BARO)) {
        if(fabs(mcstate.get_altitude() - baro_alt) > 100) {
			mincopter.hal.console->printf("arm check - inav and baro mismatch\r\n");
            return false;
        }
    }
	*/

	// Flag that arm checks have been completed
    ap.arm_check = true;

    // if we've gotten this far all is ok
    return true;
}
