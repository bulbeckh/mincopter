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


#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler.h>

#include "defines.h"
#include "config.h"
#include "log.h"
#include "util.h"
#include "profiler.h"
#include "mcinstance.h"
#include "mcstate.h"

// TODO Remove - not board specific

#ifdef TARGET_ARCH_RPI
	#include <stdio.h>
#endif

#include <AP_Math.h>
#include <AP_GPS.h>

/* @brief Interface to the object storing each sensor and other hardware abstraction (DataFlash, Battery, ..) */
MCInstance mincopter;

/* @brief Interface to the scheduler which runs sensor updates and other non-HAL, non-interrupt functions */
AP_Scheduler scheduler;

// TODO Change this to the same model that we use for planner and control (i.e. generic header file and interface
/* @brief Interface to the state estimation module */
#ifdef MC_STATE_NONE
	StateNone mcstate;
#elif MC_STATE_COMPLEMENTARY
	StateComplementary mcstate;
#elif MC_STATE_MADGWICK
	StateMadgwick mcstate;
#elif MC_STATE_EKF
	// TODO Add implementation
	StateEKF mcstate;
#elif MC_STATE_SIM
	// TODO Add implementation
	StateSim mcstate;
#endif

/* ### CONTROLLER & PLANNER ###
 * We instantiate our chosen controller here so that it can be referenced in other translation units with
 * extern. The interface is a 'soft interface' as there a no compile time checks that we are not breaking
 * the abstraction by using a derived class method (for example a method exposed by PID_Controller but
 * not by MC_Controller). This is the trade-off we make to avoid using a virtual table and extra cycle/cycles
 * for dereferencing the pointer.
 */
#include "control.h"
#include "planner.h"

// TODO Removed simulation logger - use another logging class

// NOTE Bad hack to resolve linking errors as AP_Scheduler library uses an extern hal reference as original HAL was defined globally
// TODO Remove all direct references to hal and just keep mincopter.hal
const AP_HAL::HAL& hal = mincopter.hal;

/* @brief Forward declaration of ardupilot initialisation. Defined in init.cpp */
void init_ardupilot(void);

uint32_t _counter=0;

/* Core Loop - Meant to run every 10ms (10,000 microseconds) */
void loop(void)
{
	// Record loop start time
    uint32_t timer = hal.scheduler->micros();

    // wait for an INS sample
    if (!mincopter.ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
		return;
    }

	// We accumulate the INS readings with a timer process (@ 1kHz) but we actually update the
	// sensor at 100Hz here
	mincopter.ins.update();


	// TODO This is the wrong compiler flag - need to check if we are using Generic (simulation) rather than a Linux
	// distribution as our HAL because RPI/Beaglebone do not use simulation
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

	
    // Repeat 10x times
    // 1. Setup and send control output packet (x4 motor vel)
    // 2. Receive and parse packet (update simulated sensor readings, incl. noise if needed)

	// TODO Check for reset flag here and reset simulation
	// A MinCopter reset should trigger:
	// - Resets of all controllers/planners/state/devices
	// - Reset of simulation logger
	// - Reset of timing variables (and iteration counters)
	
	// TODO Check for call to a pose update
	
	uint32_t st = hal.scheduler->micros();
	
	// Step the simulation by the desired microseconds (us)
	hal.sim->tick(10000);

	uint32_t gz_elapsed = hal.scheduler->micros()-st;
#endif

	// Run our core flight loop only if we have connected to our telemetry

	// TODO This call to update the state at 100Hz does not yet consider the frequency at which
	// we update each sensor. While the gyro/accel updates at 100Hz, the compass updates at 50Hz and the GPS
	// at 20Hz. We need to consider this during the sensor fusion
	
	// 1. Update state, regardless of whether we are connected to telemetry
	mcstate.update();

	// Print some basic state information to console
	if (_counter%100==0) {
		hal.console->printf("State (r,p,y): (% 6.2fr,% 6.2fr,% 6.2fr), (% 8.2fd, % 8.2fd, % 8.2fd) height (% 6.3f) %s, [%u,%u,%u,%u]\r\n",
				mcstate.data.euler.x,
				mcstate.data.euler.y,
				mcstate.data.euler.z,
				mcstate.data.euler.x * 180.0f / M_PI_F,
				mcstate.data.euler.y * 180.0f / M_PI_F,
				mcstate.data.euler.z * 180.0f / M_PI_F,
				mcstate.data.position[2],
				planner.ap.arm_active ? "armed" : "disarmed",
				controller.mixer.get_motor_pwm(0),
				controller.mixer.get_motor_pwm(1),
				controller.mixer.get_motor_pwm(2),
				controller.mixer.get_motor_pwm(3));
	}


	// 2. Run controller & planner
	if (planner.failsafe.telemetry_active) {

		/* Our planner algorithm updates the desired roll and pitch based on our position from desired
		 * waypoint as well as our velocity.
		 *
		 * The control flow is as follows:
		 *
		 * - planner.run
		 *   - update_nav_mode (planner)
		 *   	- update_wpnav
		 *   		- advance_target_along_track
		 *   		- get_loiter_position_to_velocity
		 *   		- get_loiter_velocity_to_acceleration
		 *   		- get_loiter_acceleration_to_lean_angles
		 *
		 *   	OR 
		 *   	- update_loiter
		 *   - wp_nav.get_desired_roll
		 *   - wp_nav.get_desired_pitch
		 *   - get_yaw_slew
		 *   - get_throttle_althold_with_slew
		 * 
		 * ## update_wpnav control flow
		 *
		 * **get_loiter_position_to_velocity**
		 * Calculates _desired_vel (x and y) by K controller w error as (_target - _curr). Uses
		 * the lat and lon PID controllers. Also feeds-forward _target_vel (x and y) into _desired_vel.
		 *
		 * **get_loiter_velocity_to_acceleration**
		 * Calculates _desired_accel (x and y) by PID controller w error as (_desired_vel - vel_curr).
		 * Feeds-forward an accel estimate based on the difference between the previous iterations
		 * desired velocity and this iterations desired velocity (multiplied by dt).
		 * 
		 * **get_loiter_acceleration_to_lean_angles**
		 * Calculates desired_roll and desired_pitch from the (yaw-corrected) desired accelerations.
		 * These are inputs into the controller.
		 *
		 * ## update_loiter control flow
		 *
		 */

		// The planner should run at every iteration but the controller should only run when armed
		planner.run();

		// Run controller only if ARMED
		if (planner.ap.arm_active) {
			controller.run();
		}
	}

	// Set motor PWM to minimum each iteration if we are not armed
	if (planner.ap.arm_active) {
		// TODO We should use an rcoutput interface function like '::zero' instead
		// Otherwise, make sure to zero all PWM output
		hal.rcout->write(0,1000);
		hal.rcout->write(1,1000);
		hal.rcout->write(2,1000);
		hal.rcout->write(3,1000);
	}

    // Tell the scheduler one tick has passed
    scheduler.tick();

	// Read telemetry for incoming commands
	read_telemetry();

	// Update state LEDs
	if (planner.ap.arm_active) {
		hal.gpio->write(27, 0);
	}

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - hal.scheduler->micros();

#ifdef TARGET_ARCH_LINUX
	/* NOTE In the simulated environment, the round of 10 GZ sensor updates takes about 10ms
	 * so we run the scheduled run to account for this */
	uint32_t runtime = gz_elapsed>(uint32_t)10000 ? 300 : (uint32_t)(10000-gz_elapsed);
	// Run whatever has more time available. Will likely be the runtime because gz_time normally takes >10ms
	scheduler.run(runtime);
#else
    scheduler.run(time_available - 300);
#endif

    uint32_t time_elapsed = hal.scheduler->micros() - timer;

    // Delay if we have time remaining (i.e. time took less than 10000us). NOTE delay_microseconds will use the
	// remaining time to run 'delay' functions.
	if (time_elapsed < 10000) {
		hal.scheduler->delay_microseconds(10000lu-time_elapsed);
	}

	// Increment loop counter;
	_counter++;

	return;
}


/* **Scheduled Functions**
 *
 * The `scheduler_tasks` object has the following structure:
 *
 * 		{ function_name, interval_ticks (multiples of 10ms), max time in us }
 *
 * We schedule the following functions to be run at certain intervals. Note, there is no mechanism to stop a scheduled 
 * function from overrunning - AP_Scheduler will only report that it overran.
 *
 * | Compass::accumulate | 50Hz (20ms)  | Accumulates a raw 3x magnetometer reading 									 |
 * | Compass::read       | 10Hz (100ms) | Converts the average of raw compass readings into an actual uT field reading   |
 * | Barometer::read     | 10Hz (100ms) | Calculates a pressure and temperature measurement from the barometer   	 	 |
 * | GPS::update		 | 50Hz (20ms)  | Reads a GPS message over UART and updates GPS state							 |
 * 
 * Additionally, for some sensors like the MS5611, we register a timer process to do a read of the internal state at 1kHz.
 *
 * In simulation, we also reduce the maximum runtime for each function to 1us in order to ensure that they all run within
 * a single call to scheduler.run . */

const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {

#ifdef TARGET_ARCH_LINUX
    { update_GPS, 	       2,   1 }, /* Sensor Update - GPS */
    { read_batt_compass,  10,   1 }, /* Sensor Update - Battery */
	// NOTE TODO Why are barometer reads even scheduled at all??
    //{ /* update_altitude */ Delegate<void(void)>::Create<AP_Baro, &AP_Baro::read>((AP_Baro*)&mincopter.barometer),    10,   1 }, /* Sensor Update - Barometer (read) */
    { read_barometer, 2, 1},
    { accumulate_compass, 2, 1},
	//{ Delegate<void(void)>::Create<Compass, &Compass::accumulate>(&mincopter.compass),        2,   1 }, /* Sensor Update - Compass */
    //{ /* read_baro */ Delegate<void(void)>::Create<AP_Baro, &AP_Baro::accumulate>(&mincopter.barometer),  	       2,   1 }, /* Sensor Update - Barometer (accumulate) */
    { accumulate_barometer, 2,   1 },
#else
    { update_GPS, 	      	     2,  900 }, /* Sensor Update - GPS */
    { read_batt_compass,  	    10,  720 }, /* Sensor Update - Battery */
    { read_barometer,		    10, 1000 }, /* Sensor Update - Barometer (read) */
    { accumulate_compass,    	 2,  420 }, /* Sensor Update - Compass */
    { accumulate_barometer,  	 2,  250 }, /* Sensor Update - Barometer (accumulate) */
// TODO The run-times for these two functions need to be tested - 500us and 300us are arbitrarary
	{ send_telemetry_heartbeat, 10,  500 }, /* Telemetry 	 - heartbeat message */
	{ failsafe_checks,			10,  300 }, /* Failsafe		 - run all required failsafe checks */
	{ crash_checks, 			10,  300 }  /* Crash		 - run checks to see if we have likely crashed */
#endif

	/* NOTE These functions have been removed from the codebase. Kept here for reference only.
	 *
	 * { dump_serial, 	      20,     500 },
	 * { run_cli,            10,     500 },
	 * { throttle_loop,       2,     450 },
	 * { crash_check,        10,      20 },
	 * { read_receiver_rssi, 10,      50 }
	 * { update_notify,       2,     100 },
	 * { run_nav_updates,    10,     800 },
	 * { fence_check	 ,    33,      90 },
	 * { arm_motors_check,   10,      10 },
	 * { update_nav_mode,     1,     400 }
	 */

};

extern "C" {
	int main (void) {

		mincopter.hal.init(0, NULL);

		/* Print initial RAM available after HAL initialisation */
		uint16_t _mem_left = mincopter.hal.util->available_memory();
		mincopter.hal.console->printf_P(PSTR("[INIT] Pre-init RAM:%u\n"), _mem_left);

		// Initialise MinCopter
		init_ardupilot();

		// Initialise & start the main loop scheduler
		scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));

		mincopter.hal.scheduler->system_initialized();

		for(;;) loop();

		return 0;
	}
}


