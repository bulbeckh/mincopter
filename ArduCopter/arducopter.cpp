// mincopter - henry

/** MinCopter - A modular end-to-end flight controller supporting multiple backend architectures and copter configurations
*
* The runtime defines the MCInstance object and the AP_Scheduler object at the global level.
* 
* The scheduler will run the sensor update methods at 100Hz and will give the remaining time
* to run the behaviour tree. The behaviour tree is the modular part and will handle updating
* of the copter state and executing the control libraries.
*
* **Sensor Updates**
* The following sensors are updated at 100Hz via the scheduler.
* - Compass (Magnetometer)
* - Barometer
* - IMU (currently indirectly via call to `update_altitude`
* - GPS
* - LEDs (via AP_Notify)
* - Battery Monitor (via AP_BattMonitor)
*
* **State Updates**
* The state estimation library is responsible for updating state variables so that that navigation and control libraries
* can generate control outputs based on current state. This library is modular meaning multiple state estimation libraries
* can be used (i.e. EKF3, DCM)
*
* **Control Updates**
* The controller is responsible for generating control outputs (and sending to motors) and planner is responsible for
* higher level waypoint/trajectory planning as well as managing fences and failsafes.
*
*/

#include <AP_Scheduler.h>       // main loop scheduler

#include "compat.h"

#include "defines.h"
#include "config.h"
#include "config_channels.h"

// Local modules
#include "attitude.h"
#include "compat.h"
#include "control_modes.h"
#include "failsafe.h"
#include "motors.h"
#include "log.h"
#include "navigation.h"
#include "radio.h"
#include "system.h"
#include "util.h"
#include "serial.h"
#include "profiler.h"

#include "mcinstance.h"
#include "mcstate.h"


#ifdef TARGET_ARCH_LINUX
	#include <iostream>
#endif

// TODO Check that accessing mincopter directly without dereferencing via pointer will not mess up any virtual methods/inheritance
MCInstance mincopter;

AP_Scheduler scheduler;

MCState mcstate;

// TODO Define controllers and planners here (and pass mincopter/mcstate as args)
//
//

// NOTE Bad hack to resolve linking errors as AP_Scheduler library uses an extern hal reference as original HAL was defined globally
const AP_HAL::HAL& hal = mincopter.hal;

uint32_t fast_loopTimer;
uint16_t mainLoop_count;

// Forward Declaration
void sensor_update_loop();

/* Core Loop - Meant to run every 10ms (10,000 microseconds) */
void loop()
{
    // wait for an INS sample
    if (!mincopter.ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
        return;
    }
    uint32_t timer = micros();

    // used by PI Loops
    mincopter.G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------

    sensor_update_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - micros();

#ifdef TARGET_ARCH_LINUX
		//std::cout << "loop: time_available " << time_available << "\n";
#endif
    scheduler.run(time_available - 300);

		uint32_t time_remaining = micros() - timer;
#ifdef TARGET_ARCH_LINUX
		std::cout << "loop: time used during sensor update and scheduler call " << time_remaining << "\n";
#endif
		// Delay if we have time remaining (i.e. time took less than 10000us)
		if (time_remaining<=10e4) mincopter.hal.scheduler->delay_microseconds(10e4-time_remaining);

}


/* There should be strictly three components to the flight loop
 *
 * 1. Sensor updates
 * 2. State updates
 * 3. Control determination
 *
 * + things like logging/comms
 *
 */

void state_update()
{
	mcstate.read_AHRS();
}

void control_determination()
{
		/* At lower frequency than controller */
		planner.run();

		controller.run();
}

// Main loop - 100hz
void sensor_update_loop()
{
		static uint32_t n_measure=0;

		// (Part of state update)
    // IMU DCM Algorithm
    // --------------------
    MC_PROFILE(readahrs,{mcstate.read_AHRS();})

		// (Part of state update for now - long term remove this)
    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    MC_PROFILE(updatetrig,{mcstate.update_trig();})


		// (Part of state update)
    MC_PROFILE(readinertia,{read_inertia();})
}

// TODO move this to btree
// throttle_loop - should be run at 50 hz
// ---------------------------
//void throttle_loop()


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

// TODO Add a new (scheduled) function that checks for input (commands) from the console and asynchronously runs them
// ensuring that enough time is provided to run them.

/* `scheduler_tasks` has the following structure
 * { function_name, interval_ticks (multiples of 10ms), max time in us }
 *
 * I believe these are executed in the order they are specified below.
 * There is no mechanism to stop a function overrunning - AP_Scheduler will only report that
 * it overran.
 */
const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
/*												 TOTAL 4210 */
    { update_GPS, 				 2,     900 }, /* Sensor Update - GPS */
    { read_batt_compass,  10,     720 }, /* Sensor Update - Battery */
// control_modes.cpp
    { update_altitude,    10,    1000 }, /* Sensor Update - IMU */
    { read_compass,        2,     420 }, /* Sensor Update - Compass */
    { read_baro,  				 2,     250 }, /* Sensor Update - Barometer */
    { one_hz_loop,       100,     420 },
		{ dump_serial, 				20,     500 },
		{ run_cli,            10,     500 },
    //{ throttle_loop,         2,     450 },
    //{ crash_check,          10,      20 },
    //{ read_receiver_rssi,   10,      50 }
    //{ update_notify,         2,     100 },
    //{ run_nav_updates,      10,     800 }, 	/* Planner Update - moved to planner */
    //{ fence_check	 ,        33,      90 },
    //{ arm_motors_check,     10,      10 },
    //{ update_nav_mode,       1,     400 }   /* Planner Update - moved to planner */
};


/* The scheduler should schedule functions that execute sensor and state updates.
* It should then 'tick' the behaviour tree which runs control libraries.
*/

// Called by HAL
void setup() 
{
		// NOTE cliSerial is an alias for mincopter.hal.console
    mincopter.cliSerial = mincopter.hal.console;

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

// NOTE Replaced the macro expansion with the main entrypoint to avoid modifying AP_HAL_AVR library
// AP_HAL_MAIN();

extern "C" {
  int main (void) {
		mincopter.hal.init(0, NULL);
    setup();
    mincopter.hal.scheduler->system_initialized();
    for(;;) loop();
    return 0;
	}
}


