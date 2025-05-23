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

	/* @brief Whether we choose to log every n'th call to each of the log functions. NOTE separate value for each
	 * log type */
	bool reduce_log_factor;

	/* @brief Structuring holding flags of whether to log each module */
	struct {
		bool log_iteration;
		bool log_planner;
		bool log_controller;
		bool log_ahrs;
		bool log_motors;
		bool log_pids;
		bool log_barometer;
		bool log_compass;
		bool log_imu;
		bool log_gps;
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

	/* @brief Write the ahrs state including main public variables */
	void write_ahrs_state();

	/* @brief Write the four motor output variables */
	void write_motor_outputs();

	/* @brief Write a PID controllers target, error, output */
	void write_pid_state(const char* pid_name, int32_t target, int32_t error, int32_t out, int32_t out_max, int32_t out_min);

	/* **Inertial Navigation**
	 *
	 *
	 */

	/* @brief Write an inertial navigation state */
	void write_inav_state(Vector3f position, Vector3f velocity);
	
	/* @brief Write an inertial navigation correction state */
	void write_inav_correction(Vector3f pos_correction, Vector3f pos_error);

	/* **Sensor Logging Methods**
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
	
	// TODO GPS State
	




};


