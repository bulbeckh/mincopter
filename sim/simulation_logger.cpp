
#include "simulation_logger.h"

#include <AP_Math.h>

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
    : simulation_out(),
	lines_written(0),
	max_lines(100000)
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

	// Log all sensors by default
	simlog_flags.log_iteration = true;
	simlog_flags.log_planner = true;
	simlog_flags.log_controller = true;
	simlog_flags.log_ahrs = true;
	simlog_flags.log_motors = true;
	simlog_flags.log_pids = true;
	simlog_flags.log_barometer = true;
	simlog_flags.log_compass = true;
	simlog_flags.log_imu = true;
	simlog_flags.log_gps = true;

}

void SimulationLogger::write_iteration(uint32_t iter)
{
	if (lines_written>max_lines) return;

    /* Every 100 iterations (or roughly 1 second) we flush the log buffers */
    if (iter%100==0) {
	simulation_out.flush();
    }

    simulation_out << "i" << iter << "\n";

	lines_written++;
}

void SimulationLogger::write_planner_state()
{
	if (lines_written>max_lines) return;

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

	lines_written++;
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

	if (lines_written>max_lines) return;

    simulation_out << "c"
	<< controller.control_roll << ","
	<< controller.control_pitch << ","
	<< controller.control_yaw << ","
	<< controller.controller_desired_alt << "\n";

	lines_written++;
}

void SimulationLogger::write_ahrs_state()
{
	if (lines_written>max_lines) return;

	float error_rp = mcstate.ahrs.get_error_rp();
	float error_yaw = mcstate.ahrs.get_error_yaw();

	Vector3f accel_ef = mcstate.ahrs.get_accel_ef();
		
	simulation_out << "ah,"
		<< mcstate.ahrs.roll << ","
		<< mcstate.ahrs.pitch << ","
		<< mcstate.ahrs.yaw << ","
		<< accel_ef.x << ","
		<< accel_ef.y << ","
		<< accel_ef.z << ","
		<< error_rp << ","
		<< error_yaw << "\n";

	/*
	Matrix3f dcm = mcstate.ahrs.get_dcm_matrix();

	simulation_out << "ah-dcm"
	*/

	lines_written++;
}

void SimulationLogger::write_motor_outputs()
{
	if (lines_written>max_lines) return;

    simulation_out << "m"
	<< mincopter.motors.get_raw_motor_out(0) << ","
	<< mincopter.motors.get_raw_motor_out(1) << ","
	<< mincopter.motors.get_raw_motor_out(2) << ","
	<< mincopter.motors.get_raw_motor_out(3) << "\n";

	lines_written++;
}

void SimulationLogger::write_pid_state(const char* pid_name, int32_t target, int32_t error, int32_t out, int32_t out_max, int32_t out_min)
{
	if (lines_written>max_lines) return;

	simulation_out << "pid,"
		<< pid_name << ","
		<< target << ","
		<< error << ","
		<< out << ","
		<< out_max << ","
		<< out_min << "\n";

	lines_written++;
}

void SimulationLogger::write_barometer_state(float temperature, float pressure, float altitude_calculated)
{
	if (lines_written>max_lines) return;

	simulation_out << "baro,"
		<< temperature << ","
		<< pressure << ","
		<< altitude_calculated <<"\n";

	lines_written++;
}

void SimulationLogger::write_compass_state(float field_x, float field_y, float field_z)
{
	if (lines_written>max_lines) return;

	simulation_out << "comp,"
		<< field_x << ","
		<< field_y << ","
		<< field_z << "\n"; 

}

void SimulationLogger::write_imu_state(Vector3f gyro, Vector3f accel)
{
	if (lines_written>max_lines) return;

	simulation_out << "imu,"
		<< gyro.x << ","
		<< gyro.y << ","
		<< gyro.z << ","
		<< accel.x << ","
		<< accel.y << ","
		<< accel.z << "\n";

}

void SimulationLogger::write_inav_state(Vector3f position, Vector3f velocity)
{
	if (lines_written>max_lines) return;

	simulation_out << "inav,"
		<< position.x << ","
		<< position.y << ","
		<< position.z << ","
		<< velocity.x << ","
		<< velocity.y << ","
		<< velocity.z << "\n";

}

void SimulationLogger::write_inav_correction(Vector3f pos_correction, Vector3f pos_error, Vector3f accel_correction)
{
	if (lines_written>max_lines) return;

	simulation_out << "inavc,"
		<< pos_correction.x << ","
		<< pos_correction.y << ","
		<< pos_correction.z << ","
		<< pos_error.x << ","
		<< pos_error.y << ","
		<< pos_error.z << ","
		<< accel_correction.x << ","
		<< accel_correction.y << ","
		<< accel_correction.z << "\n";

}




