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

// TODO Removed simulation logger - use another logging class

// NOTE Bad hack to resolve linking errors as AP_Scheduler library uses an extern hal reference as original HAL was defined globally
// TODO Remove all direct references to hal and just keep mincopter.hal
const AP_HAL::HAL& hal = mincopter.hal;

uint32_t fast_loopTimer;

/* @brief Forward declaration of ardupilot initialisation. Defined in init.cpp */
void init_ardupilot();

/* @brief The state update routine. Will update the AHRS, the Inertial Navigation, and some sensors */
void state_update()
{
	mcstate.ahrs.ahrs_update();

	//mcstate.omega = mincopter.ins.get_gyro();

	mcstate.inertial_nav.inav_update();

    mcstate.update_trig();

	return;
}

/* @brief The control update routine. Runs the planner and then the controller. */
void control_determination()
{
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
	//if (planner.planner_arm_state==PlannerArmState::ARMED) controller.run();
	controller.run();


	return;
}

uint32_t _counter=0;

/* Core Loop - Meant to run every 10ms (10,000 microseconds) */
void loop()
{
	// Loop heartbeat
	if (_counter%100==0) {
		hal.console->printf("Looping..\n");
	}

	_counter++;

	if (_counter%100==0) {
		Vector3f _gyr_meas = mincopter.ins.get_gyro();
		Vector3f _acc_meas = mincopter.ins.get_accel();
		Vector3f _mag_meas = mincopter.compass.get_field();
		GPS::GPS_Status _status = mincopter.g_gps->status();

		Vector3f _temp_pos = mcstate.get_position();
		Vector3f _temp_vel = mcstate.get_velocity();

		float _pres = mincopter.barometer.get_pressure();
		float _temperature = mincopter.barometer.get_temperature();

		Quaternion& _temp_att = mcstate._state._attitude;

		float roll,pitch,yaw;
		_temp_att.to_euler(&roll, &pitch, &yaw);

		Matrix3f _temp_rot;
		_temp_att.rotation_matrix(_temp_rot);

		mincopter.hal.console->printf("[loop %lu] remaining_ram=%u\n", _counter, mincopter.hal.util->available_memory());
		mincopter.hal.console->printf_P(PSTR("gyr: % 6.2f, % 6.2f, % 6.2f\n"), _gyr_meas.x, _gyr_meas.y, _gyr_meas.z);
		mincopter.hal.console->printf_P(PSTR("acc: % 6.2f, % 6.2f, % 6.2f\n"), _acc_meas.x, _acc_meas.y, _acc_meas.z);
		mincopter.hal.console->printf_P(PSTR("mag: % 6.2f, % 6.2f, % 6.2f\n"), _mag_meas.x, _mag_meas.y, _mag_meas.z);
		mincopter.hal.console->printf_P(PSTR("baro: % 6.2f, % 6.2f\n"), _pres, _temperature);
		mincopter.hal.console->printf_P(PSTR("gps: %d\n"), _status);
		mincopter.hal.console->printf_P(PSTR("lat/lng: %d, %d\n"), mincopter.g_gps->latitude, mincopter.g_gps->longitude);
		mincopter.hal.console->printf_P(PSTR("pos x,y,z: %f, %f, %f\n"), _temp_pos.x, _temp_pos.y, _temp_pos.z);
		mincopter.hal.console->printf_P(PSTR("vel x,y,z: %f, %f, %f\n"), _temp_vel.x, _temp_vel.y, _temp_vel.z);
		mincopter.hal.console->printf_P(PSTR("att q1,q2,q3,q4: %f, %f, %f, %f\n"), _temp_att[0], _temp_att[1], _temp_att[2], _temp_att[3]);
		mincopter.hal.console->printf_P(PSTR("eul r,p,y: %f, %f, %f\n"), roll, pitch, yaw);
		mincopter.hal.console->printf_P(PSTR("DCM: -----------\n[%f, %f, %f\n %f, %f, %f,\n%f, %f, %f]\n"),
				_temp_rot[0][0], _temp_rot[0][1], _temp_rot[0][2],
				_temp_rot[1][0], _temp_rot[1][1], _temp_rot[1][2],
				_temp_rot[2][0], _temp_rot[2][1], _temp_rot[2][2]);
	}

    uint32_t timer = micros();

    // wait for an INS sample
    if (!mincopter.ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
		return;
    }

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
	
	uint32_t st = micros();
	
	// Step the simulation by the desired microseconds (us)
	hal.sim->tick(10000);

	//gz_interface.send_control_output();
	//gz_interface.recv_state_input();
	uint32_t gz_elapsed = micros()-st;
#endif

    // mincopter.G_Dt is used by PI loops
    mincopter.G_Dt = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer = timer;

    // Update state (wait 10 iterations to gather compass and IMU data
	if (_counter>10) {
		state_update();
		// Control Determination
		control_determination();
	}

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - micros();

	if (_counter<3) mincopter.hal.console->printf("STACK pre-loop:%u\n", mincopter.hal.util->available_memory());

#ifdef TARGET_ARCH_LINUX
	/* NOTE In the simulated environment, the round of 10 GZ sensor updates takes about 10ms
	 * so we run the scheduled run to account for this */
	uint32_t runtime = gz_elapsed>(uint32_t)10000 ? 300 : (uint32_t)(10000-gz_elapsed);
	// Run whatever has more time available. Will likely be the runtime because gz_time normally takes >10ms
	scheduler.run(runtime);
#else
    scheduler.run(time_available - 300);
#endif

	if (_counter<3) mincopter.hal.console->printf("STACK post-loop:%u\n", mincopter.hal.util->available_memory());

    uint32_t time_elapsed = micros() - timer;
    // Delay if we have time remaining (i.e. time took less than 10000us)
	
	return;
}


/* TODO Add a new (scheduled) function that checks for input (commands) from
 * the console and asynchronously runs them ensuring that enough time is
 * provided to run them. */

/* `scheduler_tasks` has the following structure:
* 		{ function_name, interval_ticks (multiples of 10ms), max time in us }
 *
 * NOTE I believe these are executed in the order they are specified below.
 * 
 * NOTE There is no mechanism to stop a function overrunning - AP_Scheduler
 * will only report that it overran. */

const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
#ifdef TARGET_ARCH_LINUX
	/* For simulation, we reduce the maximum runtime for each function
	 * to 1us in order to ensure they all run within a single scheduler call */
    { update_GPS, 	       2,   1 }, /* Sensor Update - GPS */
    { read_batt_compass,  10,   1 }, /* Sensor Update - Battery */
    { update_altitude,    10,   1 }, /* Sensor Update - Barometer (read) */
    { read_compass,        2,   1 }, /* Sensor Update - Compass */
    { read_baro,  	       2,   1 }, /* Sensor Update - Barometer (accumulate) */
    { one_hz_loop,       100,   1 },
#else
    { update_GPS, 	       2, 900 }, /* Sensor Update - GPS */
    { read_batt_compass,  10, 720 }, /* Sensor Update - Battery */
    { update_altitude,    10,1000 }, /* Sensor Update - Barometer (read) */
    { read_compass,        2, 420 }, /* Sensor Update - Compass */
    { read_baro,  	       2, 250 }, /* Sensor Update - Barometer (accumulate) */
    { one_hz_loop,       100, 420 },
#endif

	/* NOTE These functions have been removed from the codebase.
	 * Kept here for reference only. */
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
* It should then 'tick' the behaviour tree which runs control libraries. */

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

// AP_HAL_MAIN() body
extern "C" {
  int main (void) {
	mincopter.hal.init(0, NULL);

	/* Print initial RAM available */
	uint16_t _mem_left = mincopter.hal.util->available_memory();
	mincopter.hal.console->printf_P(PSTR("Pre-init RAM:%u\n"), _mem_left);

    setup();
    mincopter.hal.scheduler->system_initialized();
	for(;;) loop();
    return 0;
	}
}


