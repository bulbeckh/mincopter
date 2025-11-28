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


#include "log.h"
#include "radio.h"
#include "util.h"

void WP_Planner::init_arm_motors(void)
{

#if LOGGING_ENABLED == ENABLED
    start_logging();
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
	// TODO NOTE UART writes should never block anyway - REMOVE
    mincopter.hal.uartA->set_blocking_writes(false);
    if (mincopter.hal.uartC != NULL) mincopter.hal.uartC->set_blocking_writes(false);
    if (mincopter.hal.uartD != NULL) mincopter.hal.uartD->set_blocking_writes(false);

    // Set home position
    init_home();

	// TODO This should be part of a different interface - the one that pushes the 
    calc_distance_and_bearing();

	// Give location to compass so it can set magnetic declination
	mincopter.compass.set_initial_location(mincopter.g_gps->latitude, mincopter.g_gps->longitude);

    // all I terms are invalid
    // -----------------------
#if CONTROLLER_PID
    controller.reset_I_all();
#endif

	// Set the current pressure/temperature as the ground pressure/ground temperature
	mincopter.barometer.update_calibration();

	// TODO We don't event use the motor or rc_* interfaces anymore. The mixer sends the correct control signal directly to the
	// appropriate motors based on the configuration. We will eventually need some sort of custom motor/mixer configuration
	// but for now this should suffice
	
    // enable output to motors
	//init_rc_out();
	//init_esc();

	// Update arm state to armed
	planner.ap.arm_active = 1;
	
    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

	return;
}

void WP_Planner::init_disarm_motors(void)
{
    // Return immediately if we are already disarmed
    if (!mincopter.motors.armed()) return;

	// Set motor arm to false
    mincopter.motors.armed(false);

	// TODO We should change the land/takeoff to a flight state [landed, takeoff, flying]
    // we are not in the air
    planner.ap.takeoff_complete = false;
    
    // Log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // suspend logging
		// TODO why is this camelcase - change all sensor objects to lowercase
    mincopter.DataFlash.EnableWrites(false);

    // disable gps velocity based centrefugal force compensation
    //mcstate.ahrs.set_correct_centrifugal(false);
}

bool WP_Planner::arm_checks(void)
{
	// TODO Add console logging when we fail an arm check
	
	// 1. Barometer Check
	//
	//
	
	// barometer health check
	if(!mincopter.barometer.healthy) {
		return;
	}

	// check Baro & inav alt are within 1m
	if(fabs(mcstate.get_altitude() - baro_alt) > 100) {
		return;
	}

	// 2. Compass Check
	//
	//

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

	// 3. GPS Check
	//
	//
	
	float speed_cms = mcstate.get_velocity().length();     // speed according to inertial nav in cm/s
	
	// ensure GPS is ok and our speed is below 50cm/s
	if (!GPS_ok() || mincopter.gps_glitch.glitching() || speed_cms == 0 || speed_cms > PREARM_MAX_VELOCITY_CMS) {
		return;
	}

	// 4. Fence Check
	//
	//
	
#if AC_FENCE == ENABLED
	// check fence is initialised
	if(!fence.pre_arm_check() || (((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0))) {
		return;
	}
#endif

    // 5. INS Check
	//
	//
	
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

    // check Baro & inav alt are within 1m
    if ((mincopter.arming_check == ARMING_CHECK_ALL) || (mincopter.arming_check & ARMING_CHECK_BARO)) {
        if(fabs(mcstate.get_altitude() - baro_alt) > 100) {
            return;
        }
    }

	// Flag that arm checks have been completed
    ap.arm_check = true;

    // if we've gotten this far all is ok
    return true;
}
