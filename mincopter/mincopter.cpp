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

// To enable logging over named pipe
#ifdef TARGET_ARCH_LINUX
	#include <cstring>
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
void init_ardupilot(void);

/* @brief The state update routine. Will update the AHRS, the Inertial Navigation, and some sensors */
void state_update(void)
{
	mcstate.ahrs.ahrs_update();

	//mcstate.omega = mincopter.ins.get_gyro();

	mcstate.inertial_nav.inav_update();

	// TODO What is this actually doing? None of the values are used
    //mcstate.update_trig();

	return;
}

/* @brief The control update routine. Runs the planner and then the controller. */
void control_determination(void)
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
void loop(void)
{
	// Loop heartbeat
	_counter++;

#ifdef TARGET_ARCH_LINUX
	// Sensor readings
	Vector3f _gyr_meas = mincopter.ins.get_gyro();
	Vector3f _acc_meas = mincopter.ins.get_accel();
	Vector3f _mag_meas = mincopter.compass.get_field();
	GPS::GPS_Status _status = mincopter.g_gps->status();

	// Estimated state
	Vector3f _temp_pos = mcstate.get_position();
	Vector3f _temp_vel = mcstate.get_velocity();
	Vector3f _temp_eul = mcstate.get_euler_angles();
	Vector3f _temp_e_rates = mcstate.get_euler_rates();

	// Actual state

	float _pres = mincopter.barometer.get_pressure();
	float _temperature = mincopter.barometer.get_temperature();

	Quaternion& _temp_att = mcstate._state._attitude;

	float roll,pitch,yaw;
	_temp_att.to_euler(&roll, &pitch, &yaw);

	Matrix3f _temp_rot;
	_temp_att.rotation_matrix(_temp_rot);

	// Dump to pipe @100Hz
	uint8_t log_packet[52];

	uint32_t iterations = (uint32_t)hal.sim->last_sensor_state.iterations;

	/* In place of an enum, we use the following type IDs for log messages
	 * 0x01 RPY (euler)
	 * 0x02 Position
	 * 0x03 Velocity
	 * 0x04 Euler Rates
	 * 0x05 Control Input
	 * 0x06 Motor velocities
	 *
	 * 0x07 Full sensor state (3x imu accel, 3x imu gyro, 3x compass)
	 * 0x08 Actual state (from gazebo)
	 *
	 * 0x09 GPS position and velocity
	 */

	// RPY
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &roll, 4);
	std::memcpy(log_packet+8, &pitch, 4);
	std::memcpy(log_packet+12, &yaw, 4);
	mincopter.hal.sim->log_state(log_packet, 16, 0x01);

	// Position
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &_temp_pos.x, 4);
	std::memcpy(log_packet+8, &_temp_pos.y, 4);
	std::memcpy(log_packet+12, &_temp_pos.z, 4);
	mincopter.hal.sim->log_state(log_packet, 16, 0x02);

	// Velocity
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &_temp_vel.x, 4);
	std::memcpy(log_packet+8, &_temp_vel.y, 4);
	std::memcpy(log_packet+12, &_temp_vel.z, 4);
	mincopter.hal.sim->log_state(log_packet, 16, 0x03);

	// Euler rates
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &_temp_e_rates.x, 4);
	std::memcpy(log_packet+8, &_temp_e_rates.y, 4);
	std::memcpy(log_packet+12, &_temp_e_rates.z, 4);
	mincopter.hal.sim->log_state(log_packet, 16, 0x04);

	// Control Input (Force,Torque)
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &mincopter.hal.sim->control_input[0], 4);
	std::memcpy(log_packet+8, &mincopter.hal.sim->control_input[1], 4);
	std::memcpy(log_packet+12, &mincopter.hal.sim->control_input[2], 4);
	std::memcpy(log_packet+16, &mincopter.hal.sim->control_input[3], 4);
	mincopter.hal.sim->log_state(log_packet, 20, 0x05);

	// Motor Velocities PWM
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &mincopter.hal.sim->motor_out[0], 2);
	std::memcpy(log_packet+6, &mincopter.hal.sim->motor_out[1], 2);
	std::memcpy(log_packet+8, &mincopter.hal.sim->motor_out[2], 2);
	std::memcpy(log_packet+10, &mincopter.hal.sim->motor_out[3], 2);
	mincopter.hal.sim->log_state(log_packet, 12, 0x06);

	// NOTE Downcast to float
	float imu_a_x = (float)mincopter.hal.sim->last_sensor_state.imu_accel_x;
	float imu_a_y = (float)mincopter.hal.sim->last_sensor_state.imu_accel_y;
	float imu_a_z = (float)mincopter.hal.sim->last_sensor_state.imu_accel_z;

	float imu_g_x = (float)mincopter.hal.sim->last_sensor_state.imu_gyro_x;
	float imu_g_y = (float)mincopter.hal.sim->last_sensor_state.imu_gyro_y;
	float imu_g_z = (float)mincopter.hal.sim->last_sensor_state.imu_gyro_z;

	float comp_x = (float)mincopter.hal.sim->last_sensor_state.field_x;
	float comp_y = (float)mincopter.hal.sim->last_sensor_state.field_y;
	float comp_z = (float)mincopter.hal.sim->last_sensor_state.field_z;

	// TODO Change this to the sensor variables and not the simulation variables
	// Sensor state ( (3+3+3)*4)
	std::memcpy(log_packet, &iterations, 4);

	std::memcpy(log_packet+4, &imu_a_x, 4);
	std::memcpy(log_packet+8, &imu_a_y, 4);
	std::memcpy(log_packet+12, &imu_a_z, 4);

	std::memcpy(log_packet+16, &imu_g_x, 4);
	std::memcpy(log_packet+20, &imu_g_y, 4);
	std::memcpy(log_packet+24, &imu_g_z, 4);

	std::memcpy(log_packet+28, &comp_x, 4);
	std::memcpy(log_packet+32, &comp_y, 4);
	std::memcpy(log_packet+36, &comp_z, 4);

	mincopter.hal.sim->log_state(log_packet, 40, 0x07);

	float pos_x_actual = (float)mincopter.hal.sim->last_sensor_state.pos_x;
	float pos_y_actual = (float)mincopter.hal.sim->last_sensor_state.pos_y;
	float pos_z_actual = (float)mincopter.hal.sim->last_sensor_state.pos_z;

	float vel_x_actual = (float)mincopter.hal.sim->last_sensor_state.vel_x;
	float vel_y_actual = (float)mincopter.hal.sim->last_sensor_state.vel_y;
	float vel_z_actual = (float)mincopter.hal.sim->last_sensor_state.vel_z;

	float euler_x_actual = (float)mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_x;
	float euler_y_actual = (float)mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_y;
	float euler_z_actual = (float)mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_z;

	float euler_rate_x = (float)mincopter.hal.sim->last_sensor_state.euler_rate_x;
	float euler_rate_y = (float)mincopter.hal.sim->last_sensor_state.euler_rate_y;
	float euler_rate_z = (float)mincopter.hal.sim->last_sensor_state.euler_rate_z;

	// TODO Repeat for actual angular rates
	
	std::memcpy(log_packet, &iterations, 4);

	std::memcpy(log_packet+4, &pos_x_actual, 4);
	std::memcpy(log_packet+8, &pos_y_actual, 4);
	std::memcpy(log_packet+12, &pos_z_actual, 4);

	std::memcpy(log_packet+16, &vel_x_actual, 4);
	std::memcpy(log_packet+20, &vel_y_actual, 4);
	std::memcpy(log_packet+24, &vel_z_actual, 4);

	std::memcpy(log_packet+28, &euler_x_actual, 4);
	std::memcpy(log_packet+32, &euler_y_actual, 4);
	std::memcpy(log_packet+36, &euler_z_actual, 4);

	std::memcpy(log_packet+40, &euler_rate_x, 4);
	std::memcpy(log_packet+44, &euler_rate_y, 4);
	std::memcpy(log_packet+48, &euler_rate_z, 4);

	mincopter.hal.sim->log_state(log_packet, 52, 0x08);

	// GPS
	std::memcpy(log_packet, &iterations, 4);

	std::memcpy(log_packet+4, &mincopter.g_gps->latitude, 4);
	std::memcpy(log_packet+8, &mincopter.g_gps->longitude, 4);
	std::memcpy(log_packet+12, &mincopter.g_gps->altitude_cm, 4);

	Vector3f gps_vel_vector = mincopter.g_gps->velocity_vector();

	std::memcpy(log_packet+16, &gps_vel_vector.x, 4);
	std::memcpy(log_packet+20, &gps_vel_vector.y, 4);
	std::memcpy(log_packet+24, &gps_vel_vector.z, 4);

	mincopter.hal.sim->log_state(log_packet, 28, 0x09);

	// Update position directly as a test every second
	/*
	if (_counter%100==0 && _counter<500) {
		mincopter.hal.sim->set_mincopter_position(0,0,5);
		mincopter.hal.sim->set_mincopter_linvelocity(0,0,0);
		mincopter.hal.sim->set_mincopter_attitude(0,0,0);
		mincopter.hal.sim->set_mincopter_angvelocity(0,0,0);
	}

	if (_counter%500==0) {
		// Call for simulation reset after 5 seconds
		mincopter.hal.sim->reset();
	}
	*/

	// In linux/generic (simulation) targets we dump all relevant information at 1Hz
	
	// Dump to console @1Hz
	if (_counter%100==0) {
		mincopter.hal.console->printf("---[LOOP, simtime=%f, iterations=%u]----------------------\r\n", mincopter.hal.sim->last_sensor_state.timestamp, mincopter.hal.sim->last_sensor_state.iterations);

		mincopter.hal.console->printf("AHRS sensor readings | X      | Y      | Z      |\r\n");
		mincopter.hal.console->printf("    gyro (rad/s)     | %+6.2f | %+6.2f | %+6.2f |\r\n", _gyr_meas.x, _gyr_meas.y, _gyr_meas.z);
		mincopter.hal.console->printf("    acc  (m/2  )     | %+6.2f | %+6.2f | %+6.2f |\r\n", _acc_meas.x, _acc_meas.y, _acc_meas.z);
		mincopter.hal.console->printf("    mag  (ut   )     | %+6.2f | %+6.2f | %+6.2f |\r\n\n", _mag_meas.x, _mag_meas.y, _mag_meas.z);

		/*
		mincopter.hal.console->printf("mag : % 6.2f, % 6.2f, % 6.2f\n"), _mag_meas.x, _mag_meas.y, _mag_meas.z);
		mincopter.hal.console->printf("baro: % 6.2f, % 6.2f\n"), _pres, _temperature);
		mincopter.hal.console->printf("gpss: %d\n"), _status);
		mincopter.hal.console->printf("gll : %d, %d\n"), mincopter.g_gps->latitude, mincopter.g_gps->longitude);
		mincopter.hal.console->printf("galt: %d\n"), mincopter.g_gps->altitude_cm);
		*/
		
		mincopter.hal.console->printf("state estimation (position/velocity)\r\n");
		mincopter.hal.console->printf("| x (m)  | y (m)  | z (m)  | dx (m/s) | dy (m/s) | dz (m/s) |\r\n");
		mincopter.hal.console->printf("| %+6.2f | %+6.2f | %+6.2f | %+8.2f | %+8.2f | %+8.2f |\r\n",
				_temp_pos.x, _temp_pos.y, _temp_pos.z,
				_temp_vel.x, _temp_vel.y, _temp_vel.z);

		mincopter.hal.console->printf("state estimation (attitude)\r\n");
		mincopter.hal.console->printf("| roll   | pitch  | yaw    | droll  | dpitch | dyaw  |\r\n");
		mincopter.hal.console->printf("| %+6.2f | %+6.2f | %+6.2f |    - |      - |     - |\r\n",
				roll, pitch, yaw);

		/*
		mincopter.hal.console->printf("att q1,q2,q3,q4: %f, %f, %f, %f\n"), _temp_att[0], _temp_att[1], _temp_att[2], _temp_att[3]);
		mincopter.hal.console->printf("eul r,p,y      : %f, %f, %f\n"), roll, pitch, yaw);
		mincopter.hal.console->printf("homelng/lat/alt: %d, %d, %d\n"), mcstate.home.lat, mcstate.home.lng, mcstate.home.alt);
		*/

		mincopter.hal.console->printf("control\r\n");
		mincopter.hal.console->printf("| F       | RollT   | PitchT  | YawT    |\r\n");
		mincopter.hal.console->printf("| %+5.2fN | %+5.2fNm | %+5.2fNm | %+5.2fNm |\r\n\n",
				mincopter.hal.sim->control_input[0],
				mincopter.hal.sim->control_input[1],
				mincopter.hal.sim->control_input[2],
				mincopter.hal.sim->control_input[3]
				);

		mincopter.hal.console->printf("pwm output\r\n");
		mincopter.hal.console->printf("| m0   | m1   | m2   | m3   |\r\n");
		mincopter.hal.console->printf("| %4d | %4d | %4d | %4d |\r\n\n",
				mincopter.hal.sim->motor_out[0],
				mincopter.hal.sim->motor_out[1],
				mincopter.hal.sim->motor_out[2],
				mincopter.hal.sim->motor_out[3]
				);

	}
#endif

/* Logging output is as follows:

---[LOOP, simtime=150000.00]--------------------------------------------

AHRS sensor readings | X    | Y    | Z    |
	gyro (rad/s)    | 1.02 | 0.23 | 4.23 |
	acc  (m/s2 )    | 9.03 | 1.23 | 0.23 |
	mag  (ut   )    | 34.53 | 20.02 | -19.23 |

state estimation (position/velocity)
| x (m)  | y (m)  | z (m)  | dx (m/s) | dy (m/s) | dz (m/s) |
| +40.02 | -20.02 | +30.50 |    +2.01 |     -3.02

state estimation (attitude/angvel)
| roll   | pitch  | yaw    | droll  | dpitch | dyaw   |
| -0.153 | +1.594 | -0.123 | +2.234 | -0.012 | -0.024 |

control
| F      | RollT   | PitchT   | YawT    |
| 20.42N | +1.23Nm | -0.02 Nm | +3.21Nm |

pwm output
| m0   | m1   | m2   | m3   |
| 1000 | 1000 | 1000 | 1000 |



sdas
sads



*/

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

	//if (_counter<3) mincopter.hal.console->printf("STACK pre-loop:%u\n", mincopter.hal.util->available_memory());

#ifdef TARGET_ARCH_LINUX
	/* NOTE In the simulated environment, the round of 10 GZ sensor updates takes about 10ms
	 * so we run the scheduled run to account for this */
	uint32_t runtime = gz_elapsed>(uint32_t)10000 ? 300 : (uint32_t)(10000-gz_elapsed);
	// Run whatever has more time available. Will likely be the runtime because gz_time normally takes >10ms
	scheduler.run(runtime);
#else
    scheduler.run(time_available - 300);
#endif

	//if (_counter<3) mincopter.hal.console->printf("STACK post-loop:%u\n", mincopter.hal.util->available_memory());

    uint32_t time_elapsed = micros() - timer;
    // Delay if we have time remaining (i.e. time took less than 10000us)

	if (time_elapsed < 10000) {
		hal.scheduler->delay_microseconds(10000lu-time_elapsed);
	}

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
    { update_altitude, 2, 1},
    { read_compass, 2, 1},
	//{ Delegate<void(void)>::Create<Compass, &Compass::accumulate>(&mincopter.compass),        2,   1 }, /* Sensor Update - Compass */
    //{ /* read_baro */ Delegate<void(void)>::Create<AP_Baro, &AP_Baro::accumulate>(&mincopter.barometer),  	       2,   1 }, /* Sensor Update - Barometer (accumulate) */
    { read_baro, 2,   1 },
#else
    { update_GPS, 	       2, 900 }, /* Sensor Update - GPS */
    { read_batt_compass,  10, 720 }, /* Sensor Update - Battery */
    { update_altitude,    10,1000 }, /* Sensor Update - Barometer (read) */
    { read_compass,        2, 420 }, /* Sensor Update - Compass */
    { read_baro,  	       2, 250 }, /* Sensor Update - Barometer (accumulate) */
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

/* The scheduler should schedule functions that execute sensor and state updates.
* It should then 'tick' the behaviour tree which runs control libraries. */

/* TODO Remove this - this is an artefact from the old Arduino setup/loop format of code. Can reducing bloat
 * here by moving to a single main function */
void setup(void)
{
    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

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


