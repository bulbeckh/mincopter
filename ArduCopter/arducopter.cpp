// mincopter - henry



#include <AP_Scheduler.h>       // main loop scheduler

#include "compat.h"

// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// Local modules
#include "parameters.h"
#include "GCS.h"

// New headers
#include "attitude.h"
#include "compat.h"
#include "control_modes.h"
#include "events.h"
#include "failsafe.h"
#include "fence.h"
#include "log.h"
#include "motors.h"
#include "navigation.h"
#include "radio.h"
#include "system.h"
#include "util.h"
#include "serial.h"
#include "ap_union.h"


// Forward Declarations
void loop(void);
void fast_loop(void);
void throttle_loop(void);
void read_AHRS();
void update_trig();
void run_rate_controllers();
void read_control_switch();
void update_roll_pitch_mode();
void update_rate_controller_targets();
void update_throttle_mode();
void read_inertial_altitude();
void update_auto_armed();
void set_target_alt_for_reporting(float alt_cm);
float get_target_alt_for_reporting();
bool set_yaw_mode(uint8_t new_yaw_mode);
void update_yaw_mode();
void update_roll_pitch_mode();

// func globals



void loop()
{
    // wait for an INS sample
    if (!ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
        return;
    }
    uint32_t timer = micros();

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - micros();
    scheduler.run(time_available - 300);
}


/* fast_loop Performance Monitoring */
#define MC_PROFILE(name, fcall) uint32_t name##_pre = micros();\
							{ fcall }\
							uint32_t name##_diff = micros() - name##_pre;\
							(name).t_sum += name##_diff;\
							(name).n_measure +=1;

#define MC_RESET(name) (name).n_measure=0; (name).t_sum=0;

typedef struct {
	uint32_t n_measure=0;
	uint32_t t_sum=0;
} FLFunctionProfile;

/* List of functions to profile */
FLFunctionProfile updatemodes;
FLFunctionProfile rrcontrollers;
FLFunctionProfile readahrs;
FLFunctionProfile updatetrig;
FLFunctionProfile updatemotors;
FLFunctionProfile readinertia;

// Main loop - 100hz
void fast_loop()
{
		static uint32_t n_measure=0;

    // IMU DCM Algorithm
    // --------------------
    MC_PROFILE(readahrs,{read_AHRS();})

    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    MC_PROFILE(updatetrig,{update_trig();})

		// Run controllers that take body frame rate targets and convert to motor values using PID rate controllers (get_rate_{roll,pitch,yaw})
		MC_PROFILE(rrcontrollers,{run_rate_controllers();})

    // write out the servo PWM values to motors
    // ------------------------------
    MC_PROFILE(updatemotors,{motors.output();})

    // Inertial Nav
    // --------------------
    MC_PROFILE(readinertia,{read_inertia();})

		// Calls flight P controller to convert desired angle into desired rate
		MC_PROFILE(updatemodes,{update_yaw_mode(); update_roll_pitch_mode();})

		// convert rate targets to body frame using DCM values (stored in variables like cos_roll_x and cos_pitch_x)
    update_rate_controller_targets();

		n_measure+=1;
		// Performance profiling
		if (n_measure>100) {
			cliSerial->printf_P(PSTR("T_UMOD: %fus\n"), (float)updatemodes.t_sum/(1.0f* updatemodes.n_measure));
			cliSerial->printf_P(PSTR("T_RR: %fus\n"), (float)rrcontrollers.t_sum/(1.0f* rrcontrollers.n_measure));
			cliSerial->printf_P(PSTR("T_RA: %fus\n"), (float)readahrs.t_sum/(1.0f* readahrs.n_measure));
			cliSerial->printf_P(PSTR("T_UT: %fus\n"), (float)updatetrig.t_sum/(1.0f* updatetrig.n_measure));
			cliSerial->printf_P(PSTR("T_UMOT: %fus\n"), (float)updatemotors.t_sum/(1.0f* updatemotors.n_measure));
			cliSerial->printf_P(PSTR("T_RI: %fus\n"), (float)readinertia.t_sum/(1.0f* readinertia.n_measure));

			// Rest global measure variable
			n_measure=0;
			MC_RESET(updatemodes)
			MC_RESET(rrcontrollers)
			MC_RESET(readahrs)
			MC_RESET(updatetrig)
			MC_RESET(updatemotors)
			MC_RESET(readinertia)
		}
}

// TODO move this to btree
// throttle_loop - should be run at 50 hz
// ---------------------------
void throttle_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

    // check if we've landed
    update_land_detector();

    // check auto_armed status
    update_auto_armed();
}

// set_target_alt_for_reporting - set target altitude in cm for reporting purposes (logs and gcs)
void set_target_alt_for_reporting(float alt_cm)
{
    target_alt_for_reporting = alt_cm;
}

// get_target_alt_for_reporting - returns target altitude in cm for reporting purposes (logs and gcs)
float get_target_alt_for_reporting()
{
    return target_alt_for_reporting;
}




/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */


/* TODO The following should be moved to the behaviour tree
- throttle_loop
- certain fast_loop functions
- run_nav_updates
- crash_check
- the three Hz loop only contains a fence_check and that should be moved to the behaviour tree

The only thing that should be scheduled like this is sensor updates

// TODO Move the ins update from the AHRS into the below scheduled function

*/
const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { throttle_loop,         2,     450 },
    { update_GPS,            2,     900 },
    { update_nav_mode,       1,     400 },
    { read_batt_compass  ,  10,     720 },
    { arm_motors_check,     10,      10 },
    { update_altitude,      10,    1000 },
    { run_nav_updates,      10,     800 },
    { fence_check	 ,        33,      90 },
    { read_compass	    ,    2,     420 },
    { read_baro      ,  2,     250 },
    { update_notify,         2,     100 },
    { one_hz_loop,         100,     420 },
    { crash_check,          10,      20 },
    { read_receiver_rssi,   10,      50 }
};


/* GLOBAL Objects */

static MCInstance mincopter;

AP_Scheduler scheduler;

/* The scheduler should schedule functions that execute sensor and state updates.
* It should then 'tick' the behaviour tree which runs control libraries.
*/

// Called by HAL
void setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

AP_HAL_MAIN();


