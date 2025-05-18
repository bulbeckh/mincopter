
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

	/* Write loiter step */
	Vector3f nav_target = planner.wp_nav.get_wp_nav_target();

	int32_t d_roll = planner.wp_nav.get_desired_roll();
	int32_t d_pitch = planner.wp_nav.get_desired_pitch();

    simulation_out << "p"
		<< (int)planner.wp_nav.get_loiter_step() << ","
		<< nav_target.x << ","
		<< nav_target.y << ","
		<< nav_target.z << ","
		<< d_roll << ","
		<< d_pitch << "\n";

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
	<< controller.control_yaw << ","
	<< controller.controller_desired_alt << "\n";
}

void SimulationLogger::write_stest_state()
{
    simulation_out << "s\n";
}

void SimulationLogger::write_motor_outputs()
{
    simulation_out << "m"
	<< mincopter.motors.get_raw_motor_out(0) << ","
	<< mincopter.motors.get_raw_motor_out(1) << ","
	<< mincopter.motors.get_raw_motor_out(2) << ","
	<< mincopter.motors.get_raw_motor_out(3) << "\n";

}

void SimulationLogger::write_pid_state(const char* pid_name, int32_t target, int32_t error, int32_t out, int32_t out_max, int32_t out_min)
{
	simulation_out << "pid,"
		<< pid_name << ","
		<< target << ","
		<< error << ","
		<< out << ","
		<< out_max << ","
		<< out_min << "\n";
}

