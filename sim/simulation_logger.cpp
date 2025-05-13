
#include "simulation_logger.h"

#include "mcinstance.h"
extern MCInstance mincopter;

#include "mcstate.h"
extern MCState state;

#include "control.h"
#include "controller_pid.h"
extern PID_Controller controller;

#include "planner.h"
#include "planner_waypoint.h"
extern WP_Planner planner;


SimulationLogger::~SimulationLogger() {
    simulation_out.close();
}

SimulationLogger::SimulationLogger(bool overwrite)
    : simulation_out()
{
    /* Get current time */
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[1024];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%d_%m_%Y_%H_%M_%S", timeinfo);

    /* Create file name */
    std::string fileName;
    if (overwrite) {
	fileName = "./standard.mcsimlog";
    } else {
	fileName = "./" + std::string(buffer) + ".mcsimlog";

    }

    /* Open simulation logfile */
    if (overwrite) {
	simulation_out.open(fileName, std::ios::out );
    } else {
	simulation_out.open(fileName, std::ios::out | std::ios::app);
    }

    if (!simulation_out.good()) std::cout << "ERROR OPENING SIMULATION FILE FOR WRITING\n";

}

void SimulationLogger::write_iteration(uint32_t iter)
{
    /* Every 100 iterations (or roughly 1 second) we flush the log buffers */
    if (iter%100==0) {
	simulation_out.flush();
    }

    simulation_out << "i" << iter << "\n";

}

void SimulationLogger::write_planner_state()
{

    simulation_out << "p\n";
}

void SimulationLogger::write_controller_state()
{
    /* The controller states that we currently want to log are the three control variables:
     *
     * control_roll
     * control_pitch
     * control_yaw
     *
     */

    simulation_out << "c"
	<< controller.control_roll << ","
	<< controller.control_pitch << ","
	<< controller.control_yaw << "\n";
}

void SimulationLogger::write_stest_state()
{
    simulation_out << "s\n";
}


