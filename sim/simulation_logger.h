#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include <stdint.h>

#include <AP_Math.h>

/* TODO Rewrite this whole class to have a more compact representation of the simulation file
 * TODO Add the flags check to each of the functions
 */

class SimulationLogger
{
    private:
	std::ofstream simulation_out;

    public:
	/* @brief Number of lines we have written to the log file. Currently used to limit the file to 100k lines
	 * so that we don't leave it running accidentally. */
	uint32_t lines_written;

	/* @brief Maximum number of lines we want to log */
	uint32_t max_lines;

	/* @brief Structure holding the number of iterations to log for each type of log
	 *
	 * For example, to log every n'th planner, we set the log_planner entry to n
	 *
	 */
	struct {
		uint16_t log_iteration = 0;
		uint16_t log_planner = 0;
		uint16_t log_controller = 0;
		uint16_t log_ahrs = 0;
		uint16_t log_motors = 0;
		uint16_t log_pids = 0;
		uint16_t log_inav = 0;
		uint16_t log_inavc = 0;
		uint16_t log_barometer = 0;
		uint16_t log_compass = 0;
		uint16_t log_imu = 0;
		uint16_t log_gps = 0;
		uint16_t log_mpc = 0;
	} simlog_flags;

	// TODO Add a setter method for these flags

	/* @brief Simulation logging object to write state of controller, planner, state estimation, and other relevant parameters"
	 * @param bool overwrite : Whether to use a generic name for the log file that gets overwritten or use a name based on date and time of simulation start
	 */
	SimulationLogger(bool overwrite);

	~SimulationLogger();

    public:
	/* @brief Should be called at start of mincopter 100Hz (10ms) loop to signal an iteration */
	void write_iteration(uint32_t iter);

	/* @brief Write the variables related to the current planner library */
	void write_planner_state();

	/* @brief Write the variables related to the current controller library */
	void write_controller_state();


	/* @brief Write the four motor output variables */
	void write_motor_outputs();

	/* @brief Write a PID controllers target, error, output */
	void write_pid_state(const char* pid_name, int32_t target, int32_t error, int32_t out, int32_t out_max, int32_t out_min);

	/* **State Estimation**
	 *
	 *
	 */

	/* @brief Write the ahrs state including main public variables */
	void write_ahrs_state();

	/* @brief Write an inertial navigation state */
	void write_inav_state(Vector3f position, Vector3f velocity);
	
	/* @brief Write an inertial navigation correction state */
	void write_inav_correction(Vector3f pos_correction, Vector3f pos_error, Vector3f accel_correction);

	/* **Sensors**
	 *
	 * These are normally called during each sensors **read** or **update** method, meaning that they will log 
	 * at a frequency equivalent to the sensor update frequency.
	 */

	/* @brief Write a barometer reading */
	void write_barometer_state(float temperature, float pressure, float altitude_calculated);

	/* @brief Write a compass reading */
	void write_compass_state(float field_x, float field_y, float field_z);

	/* @brief Write an IMU reading */
	void write_imu_state(Vector3f gyro, Vector3f accel);
	
	/* @brief Write a GPS state reading
	 * @param lat Latitude
	 * @param lng Longitude
	 * @param alt_cm Altitude in cm
	 * @param vel_north, vel_east, vel_down GPS velocities in m/s
	 *
	 * */
	void write_gps_state(int32_t lat, int32_t lng, int32_t alt_cm, float vel_north, float vel_east, float vel_down);
	
	/* @brief Write an MPC control output
	 * @param total_thrust Output thurst
	 * @param roll_torque Output roll torque
	 * @param pitch_torque Output pitch torque
	 * @param yaw_torque Output yaw torque
	 */
	void write_mpc_control_output(float total_thrust, float roll_torque, float pitch_torque, float yaw_torque);



};


