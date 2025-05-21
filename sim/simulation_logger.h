#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include <stdint.h>

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

	/* @brief Simulation logging object to write state of controller, planner, state estimation, and other relevant parameters"
	 * @param bool overwrite : Whether to use a generic name for the log file that gets overwritten or use a name based on date and time of simulation start
	 */
	SimulationLogger(bool overwrite);

	~SimulationLogger();

    public:
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

	/* **Sensor Logging Methods** */

	/* @brief Write a barometer reading */
	void write_barometer_state(float temperature, float pressure, float altitude_calculated);



	/* @brief Should be called at start of mincopter 100Hz (10ms) loop to signal an iteration */
	void write_iteration(uint32_t iter);


};


