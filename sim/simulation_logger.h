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

	/* @brief Write the variables related to the current state estimation library */
	void write_stest_state();

	/* @brief Write the four motor output variables */
	void write_motor_outputs();



	/* @brief Should be called at start of mincopter 100Hz (10ms) loop to signal an iteration */
	void write_iteration(uint32_t iter);


};


