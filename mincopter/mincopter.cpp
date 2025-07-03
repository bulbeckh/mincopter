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

/* There should be strictly three components to the flight loop
 *
 * 1. Sensor updates
 * 2. State updates
 * 3. Control determination
 *
 * + things like logging/comms
 *
 */


#include <AP_Scheduler.h>       // main loop scheduler

#include "defines.h"
#include "config.h"

// Local modules
// HASHinclude "compat.h"
#include "log.h"
// HASH include "init.h" // Replaced with forward declaration
#include "util.h"

#include "profiler.h"

#include "mcinstance.h"
#include "mcstate.h"


#ifdef TARGET_ARCH_LINUX
    #include <iostream>
    #include "gz_interface.h"
    GZ_Interface gz_interface;
#endif

/* @brief Interface to the object storing each sensor and other hardware abstraction (DataFlash, Battery, ..) */
MCInstance mincopter;

/* @brief Interface to the scheduler which runs sensor updates and other non-HAL, non-interrupt functions */
AP_Scheduler scheduler;

/* @brief Interface to the state estimation module */
MCState mcstate;

/* ### CONTROLLER & PLANNER ###
 * We instantiate our chosen controller here so that it can be referenced in other translation units with
 * extern. The interface is a 'soft interface' as there a no compile time checks that we are not breaking
 * the abstraction by using a derived class method (for example a method exposed by PID_Controller but
 * not by MC_Controller). This is the trade-off we make to avoid using a virtual table and extra cycle/cycles
 * for dereferencing the pointer.
 */
#include "control.h"
#include "planner.h"

/* ### SIMULATION LOGGER ###
 * In the simulation, we often want to log certain states of function variables and class variables.
 * Like the planner and the controller, we instantiate an object here an call it's methods when we want
 * to log.
 */
#ifdef TARGET_ARCH_LINUX
    #include "simulation_logger.h"
    SimulationLogger simlog(true);
#endif

// NOTE Bad hack to resolve linking errors as AP_Scheduler library uses an extern hal reference as original HAL was defined globally
const AP_HAL::HAL& hal = mincopter.hal;

uint32_t fast_loopTimer;

/* @brief Initialises the ardupilot. Defined in init.cpp */
void init_ardupilot();

/* @brief The state update routine. Will update the AHRS, the Inertial Navigation, and some sensors
 */
void state_update()
{
    mcstate.read_AHRS();

    read_inertia();

    mcstate.update_trig();
}

/* @brief The control update routine. Runs the planner and then the controller.
 */
void control_determination()
{
	// The planner should run at every iteration but the controller should only run when armed
    planner.run();

	// Run controller only if ARMED
	if (planner.planner_arm_state==PlannerArmState::ARMED) controller.run();

#ifdef TARGET_ARCH_LINUX
    simlog.write_controller_state();
    simlog.write_planner_state();
#endif

}

#ifdef TARGET_ARCH_LINUX
uint32_t loop_iterations=0;
#endif

/* Core Loop - Meant to run every 10ms (10,000 microseconds) */
#ifdef TARGET_ARCH_LINUX
bool loop()
#else
void loop()
#endif
{
    uint32_t timer = micros();

#ifdef TARGET_ARCH_LINUX
    simlog.increase_iteration();
#endif

    // wait for an INS sample
    if (!mincopter.ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
        return false;
    }


#ifdef TARGET_ARCH_LINUX
    /* NOTE This is where the simulation is progressed. This loop is meant to run at 10ms
     * but the gazebo simulation uses a step size of 1ms. The workaround is to send/receive
     * over UDP with simulation 10 times and then execute this loop but that is not a long
     * term solution.
     *
     * TODO Also, we are checking for the TARGET_ARCH_LINUX to be defined but this should really
     * be it's own simulation architecture like TARGET_ARCH_SIM so as not to confuse simulations
     * with linux based boards like Raspberry PI.
     */

	// NOTE TODO I have updated the gz_interface to only call at 100Hz so no need to bucket the calls together like this
	
    // Repeat 10x times
    // 1. Setup and send control output packet (x4 motor vel)
    // 2. Receive and parse packet (update simulated sensor readings, incl. noise if needed)

	// NOTE This is taking ~10ms to send/receive 10 times
	uint32_t st = micros();
    //for (int ii=0;ii<10;ii++) {
		gz_interface.send_control_output();
		gz_interface.recv_state_input();
    //}
	uint32_t gz_elapsed = micros()-st;

#endif

#ifdef TARGET_ARCH_LINUX
    if (loop_iterations%100==0) {
	/* Should output every 1 second */
		std::cout << "TIMING (should be 10ms): " << (1.0e-3)*(micros()-fast_loopTimer) << "ms \n";
		//std::cout << "GZ (should be <<10ms): " << (1.0e-3)*(gz_elapsed) << "ms \n";
    	//std::cout << loop_iterations << " loop: time used during sensor update and scheduler call " << time_elapsed << "\n";
    	//std::cout << "load avg " << scheduler.load_average((uint32_t)10000) << "\n";

	//std::cout << "delay" << 10000-(uint16_t)time_elapsed << "\n";
    }
    loop_iterations++;
#endif


    // used by PI Loops
    mincopter.G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer          = timer;

    // Update state
    state_update();

    // Control Determination
    control_determination();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - micros();

#ifdef TARGET_ARCH_LINUX
	/* NOTE In the simulated environment, the round of 10 GZ sensor updates takes about 10ms
	 * so we run the scheduled run to account for this */
	uint32_t runtime = gz_elapsed>(uint32_t)10000 ? 300 : (uint32_t)(10000-gz_elapsed);
	// Run whatever has more time available. Will likely be the runtime because gz_time normally takes >10ms
	scheduler.run(runtime);
	if (loop_iterations%100==0) std::cout << "RUNTIME: " << runtime << "\n";
#else
    scheduler.run(time_available - 300);
#endif

    uint32_t time_elapsed = micros() - timer;
    // Delay if we have time remaining (i.e. time took less than 10000us)
	
#ifdef TARGET_ARCH_LINUX
	if (loop_iterations%100==0) std::cout << "TIME LEFT AFTER SCHEDULER RUN us: " << time_elapsed << "\n";
#endif

#ifdef TARGET_ARCH_LINUX
	static uint32_t exit_count=0;
	if (exit_count>=500) {
		return true;
	}
	// NOTE Uncomment to ensure exit for gprof
	//exit_count++;
	
	if (loop_iterations%100==0) std::cout << "FINAL TIME us : " << micros()-timer << "\n";
#endif

	return false;
}




/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
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
//												 TOTAL 4210
#ifdef TARGET_ARCH_LINUX
	/* For simulation, we reduce the maximum runtime for each to 1us in order to ensure they all run
	 * within a single scheduler call */
    { update_GPS, 	   2,     1 }, /* Sensor Update - GPS */
    { read_batt_compass,  10,  1 }, /* Sensor Update - Battery */
    { update_altitude,    10,  1 }, /* Sensor Update - Barometer (read) */
    { read_compass,        2,  1 }, /* Sensor Update - Compass */
    { read_baro,  	   2,     1 }, /* Sensor Update - Barometer (accumulate) */
    { one_hz_loop,       100,  1 },
#else
    { update_GPS, 	   2,     900 }, /* Sensor Update - GPS */
    { read_batt_compass,  10,     720 }, /* Sensor Update - Battery */
    { update_altitude,    10,    1000 }, /* Sensor Update - Barometer (read) */
    { read_compass,        2,     420 }, /* Sensor Update - Compass */
    { read_baro,  	   2,     250 }, /* Sensor Update - Barometer (accumulate) */
    { one_hz_loop,       100,     420 },
#endif

    //{ dump_serial, 	  20,     500 },
    //{ run_cli,          10,     500 },
    //{ throttle_loop,     2,     450 },
    //{ crash_check,      10,      20 },
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

/* TODO Remove this - this is an artefact from the old Arduino setup/loop format of code. Can reducing bloat
 * here by moving to a single main function */
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
#ifdef TARGET_ARCH_LINUX
	// For simulation purposes, have ability to exit early
	bool early_return=false;
	while (!early_return) {
		early_return = loop();
	}
#else
    for(;;) loop();
#endif
    return 0;
	}
}


