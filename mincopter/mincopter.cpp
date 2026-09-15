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
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_BattMonitor.h>

/* @brief Interface to the object storing each sensor and other hardware abstraction (DataFlash, Battery, ..) */
//MCInstance mincopter;



// TODO Unfortunately, the planner and controller follow the same pattern as the HAL, but not thes
// same pattern as the state or drivers

/* ### CONTROLLER & PLANNER ###
 * We instantiate our chosen controller here so that it can be referenced in other translation units with
 * extern. The interface is a 'soft interface' as there a no compile time checks that we are not breaking
 * the abstraction by using a derived class method (for example a method exposed by PID_Controller but
 * not by MC_Controller). This is the trade-off we make to avoid using a virtual table and extra cycle/cycles
 * for dereferencing the pointer.
 */

#include "control.h"
//#include "planner.h"


// TODO Removed simulation logger - use another logging class

// NOTE Bad hack to resolve linking errors as AP_Scheduler library uses an extern hal reference as original HAL was defined globally
// TODO Remove all direct references to hal and just keep mincopter.hal
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

uint32_t _counter=0;

/* Core Loop - Meant to run every 10ms (10,000 microseconds) */
bool loop(AP_Scheduler& scheduler, MCInstance& mincopter, MCState& mcstate /*, MC_Planner& planner, MC_Controller& controller */ )
{
	// Record loop start time
    uint32_t timer = hal.scheduler->micros();

    // wait for an INS sample
    if (!mincopter.ins.wait_for_sample(1000)) {
        //Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
		return true;
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
	
	// If we lose connection to the simulation, then we should exit the simulation loop
	if (!hal.sim->connected()) return false;

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
	/*
	if (_counter%100==0) {
		hal.console->printf("[%u, armed=%d]State (r,p,y): (% 6.2fr,% 6.2fr,% 6.2fr), (% 8.2fd, % 8.2fd, % 8.2fd) height (% 6.3f) %s, [%u,%u,%u,%u]\r\n",
				hal.scheduler->millis(),
				planner.ap.arm_active,
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
	*/


	// 2. Run controller & planner
	//if (planner.failsafe.telemetry_active) {

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

		// TODO Stopped until we discuss planner loop
		// Run the planner. The controller is called from within the planner
		//planner.run();
	//}

	// Set motor PWM to minimum each iteration if we are not armed
	/*
	if (!planner.ap.arm_active) {
		// TODO We should use an rcoutput interface function like '::zero' instead
		// Otherwise, make sure to zero all PWM output
		hal.rcout->write(0,1000);
		hal.rcout->write(1,1000);
		hal.rcout->write(2,1000);
		hal.rcout->write(3,1000);
	}
	*/

    // Tell the scheduler one tick has passed
    scheduler.tick();

    // TODO This is such a strange design pattern to pass mincopter object like this
	// Read telemetry for incoming commands
	read_telemetry(mincopter);

	// Update state LEDs
	/*
	if (planner.ap.arm_active) {
		hal.gpio->write(27, 0);
	}
	*/

#ifdef TARGET_ARCH_LINUX
	// Log state to the simulation debug file
	dump_state(_counter);
#endif

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - hal.scheduler->micros();

#ifdef TARGET_ARCH_LINUX
	//uint32_t runtime = gz_elapsed>(uint32_t)10000 ? 300 : (uint32_t)(10000-gz_elapsed);
	// Run whatever has more time available. Will likely be the runtime because gz_time normally takes >10ms
	//scheduler.run(runtime);
	scheduler.run(10000);
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

	return true;
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

/* Short discussion of where each func is located
 *
 *
 * mcinstance.cpp:
 * 	read_telemetry 			(called during tick loop)  	Simple - depends on mincopter
 * 	send_telemetry_heartbeat 	(scheduled)			Depends on planner and mincopter
 * 	failsafe_checks 		(scheduled)			Depends on planner and mincopter
 * 	accumulate_compass 		(scheduled)			Simple - depends on mincopter
 * 	accumulate_barometer 		(scheduled)			Simple - depends on mincopter
 * 	read_barometer 			(scheduled)			Simple - depends on mincopter
 * 	read_batt_compass 		(scheduled)			Complex - depends on mincopter
 * 	update_GPS 			(scheduled)			Complex - depends on mincopter
 *	read_receiver_rssi 		(scheduled)			Simple - depends on mincopter
 *
 * lib/util.cpp:
 *	crash_checks (scheduled)		Depends mincopter, planner, and state
 *	init_home (??)				Depends on mincopter and state
 *	GPS_ok (??)				Depends on planner and mincopter
 *	dump_state (called during tick loop)	Depends on mincopter
 * 	
 *
 * Important - IMU accumulations happen as part of timer process but their
 * read is done once per loop tick.
 *
 *
 */

const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {

#ifdef TARGET_ARCH_LINUX
    { update_GPS, 	       2,   1 }, /* Sensor Update - GPS */
    { read_batt_compass,  10,   1 }, /* Sensor Update - Battery */
    { read_barometer, 2, 1},
    { accumulate_compass, 2, 1},
    { accumulate_barometer, 2,   1 },
    //{ send_telemetry_heartbeat, 10, 1 },
    //{ failsafe_checks, 10, 1 },
    //{ crash_checks, 10, 1}
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

	// TODO Was previously exploring providing sensor class methods directly as callbacks rather than wrapper functions like 'accumulate_compass' that just call the underlying sensor method anyway
    //{ /* update_altitude */ Delegate<void(void)>::Create<AP_Baro, &AP_Baro::read>((AP_Baro*)&mincopter.barometer),    10,   1 }, /* Sensor Update - Barometer (read) */
	//{ Delegate<void(void)>::Create<Compass, &Compass::accumulate>(&mincopter.compass),        2,   1 }, /* Sensor Update - Compass */
    //{ /* read_baro */ Delegate<void(void)>::Create<AP_Baro, &AP_Baro::accumulate>(&mincopter.barometer),  	       2,   1 }, /* Sensor Update - Barometer (accumulate) */

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

//extern "C" {
int main (void) {

	// TODO We won't be able to use mincopter here - need access to a separate HAL object somewhere else
	// HAL global already created at this point
	hal.init(0, NULL);

	// Create objects
	AP_BattMonitor battery;

	Telemetry telemetry;

#ifdef MC_STORAGE_FILE
	// TODO Remove hardcoded filepath
	DataFlash_File DataFlash("/home/henry/Documents/mc-dev/logs");
#elif  MC_STORAGE_DATAFLASH
	DataFlash_APM2 DataFlash;
#elif  MC_STORAGE_EMPTY
	DataFlash_Empty DataFlash;
#endif

#ifdef MC_ADC_ADS7844
	AP_ADC_ADS7844 adc;
#elif  MC_ADC_NONE
	AP_ADC_None adc;
#elif  MC_ADC_SIM
	AP_ADC_Sim adc;
#endif

#ifdef MC_IMU_MPU6000
	AP_InertialSensor_MPU6000 ins;
#elif  MC_IMU_MPU6050
	AP_InertialSensor_MPU6050 ins;
#elif  MC_IMU_ICM20948
	AP_InertialSensor_ICM20948 ins;
#elif  MC_IMU_SIM
	AP_InertialSensor_Sim ins;
#elif  MC_IMU_NONE
	AP_InertialSensor_None ins;
#endif

#ifdef MC_BARO_MS5611
	// TODO Whether to use I2C or SPI should be a separate configuration, where the wiring is also specified
	// HASH if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
	AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
	// HASH elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
	// TODO Remove this - I2C is not used for Baro
	// Confirmed this is the baro (the I2C version)
	// AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
	// HASH endif
#elif  MC_BARO_BME280
	AP_Baro_BME280 barometer;
#elif  MC_BARO_SIM
	AP_Baro_Sim barometer;
#elif  MC_BARO_NONE
	AP_Baro_None barometer;
#endif

#ifdef MC_COMP_HMC5843
	AP_Compass_HMC5843 compass;
#elif  MC_COMP_ICM20948
	AP_Compass_ICM20948 compass;
#elif  MC_COMP_SIM
	AP_Compass_Sim compass;
#elif  MC_COMP_NONE
	AP_Compass_None compass;
#endif

	// TODO How is this GPS organised/managed - very confusing
	GPS* g_gps;

	GPS_Glitch gps_glitch(g_gps);

#ifdef MC_GPS_AUTO
	// NOTE Almost certain ours is ublox
	// TODO I'm pretty sure AP_GPS_Auto will include code for
	// all GPS backends into final executable and determine at
	// runtime. This clogs executable. Change this to a specific
	// GPS backend. I think ublox is correct for APM2.5
	#if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
	AP_GPS_Auto     g_gps_driver(&g_gps);
	// TODO Remove the remaining GPS objects
	 #elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
	AP_GPS_NMEA     g_gps_driver(&g_gps);
	 #elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
	AP_GPS_SIRF     g_gps_driver(&g_gps);
	 #elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
	AP_GPS_UBLOX    g_gps_driver(&g_gps);
	 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
	AP_GPS_MTK      g_gps_driver(&g_gps);
	 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
	AP_GPS_MTK19    g_gps_driver(&g_gps);
	 #elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
	AP_GPS_None     g_gps_driver(&g_gps);
	 #else
		#error Unrecognised GPS_PROTOCOL setting.
	#endif // GPS PROTOCOL
#elif MC_GPS_SIM
	AP_GPS_Sim   g_gps_driver;
#elif MC_GPS_UBLOX
	AP_GPS_UBLOX g_gps_driver;
#elif MC_GPS_NONE
	AP_GPS_None  g_gps_driver;
#endif

    	g_gps = &g_gps_driver;

	// TODO As above, change this to a hal reference, not a mincopter reference
	/* Print initial RAM available after HAL initialisation */
	uint16_t _mem_left = hal.util->available_memory();
	hal.console->printf_P(PSTR("[INIT] Pre-init RAM:%u\n"), _mem_left);

	// Create MinCopter object
	MCInstance mincopter(
			DataFlash,
			barometer,
			compass,
			g_gps,
			gps_glitch,
			battery,
			telemetry,
			ins,
			adc);
	
	// Initialise MinCopter
	mincopter.init_ardupilot();

	// State estimation library
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

	// TODO Add creation/initialisation of state, planner, and controller
	// Initialise our mcstate algorithms. This will call the class-specific **init_derived** method
    	mcstate.init();
	hal.console->printf_P(PSTR("[INIT] MCState initialised\n"));

#ifdef CONTROLLER_MPC
	MPC_Controller controller(mincopter, mcstate);
#elif CONTROLLER_PID
	// TODO We don't have a PID controller implementation any more. Moved to CSC controller
#elif CONTROLLER_LQR
	LQR_Controller controller(mincopter, mcstate);
#elif CONTROLLER_CSC
	CSC_Controller controller(mincopter, mcstate);
#elif CONTROLLER_NONE
	None_Controller controller(mincopter, mcstate);
#endif

	/* @brief Interface to the scheduler which runs sensor updates and other non-HAL, non-interrupt functions */
	AP_Scheduler scheduler(mincopter);
	
	// Initialise & start the main loop scheduler
	scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));

	hal.scheduler->system_initialized();

	// TODO The loop should typically not return true, but we need to include here for when we lose
	// connection to the simulation plugin and need to exit early
	for(;;) {
		if (!loop(scheduler, mincopter, mcstate/*, planner, controller */)) break;
	}

	return 0;
}
// } // EXTERN C

