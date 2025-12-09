
#include "controller_csc.h"

#include <AP_Math.h>

#define CSC_PID_IMAX 100

#include "mcinstance.h"
extern MCInstance mincopter;

#include "mcstate.h"

// Instance
CSC_Controller controller;

CSC_Controller::CSC_Controller()
	: MC_Controller(),

	csc_counter(0),

	rate_roll(0.5, 0, 0, CSC_PID_IMAX),
	rate_pitch(0.5, 0, 0, CSC_PID_IMAX),
	rate_yaw(0.5, 0, 0, CSC_PID_IMAX),

	error_roll(5, 0, 0, CSC_PID_IMAX),
	error_pitch(5, 0, 0, CSC_PID_IMAX),
	error_yaw(5, 0, 0, CSC_PID_IMAX),

	pos_throttle(1.0,0,0, CSC_PID_IMAX),
	vel_throttle(1.0,0,0, CSC_PID_IMAX),

	// Navigation Controllers - TODO May be moved
	// TODO Modify gains - significant overshoot in x,y waypoint response
	nav_x_pos(0.6, 0, 0, CSC_PID_IMAX),
	nav_y_pos(0.6, 0, 0, CSC_PID_IMAX),
	nav_x_vel(5, 0, 0, CSC_PID_IMAX),
	nav_y_vel(5, 0, 0, CSC_PID_IMAX)

	/*
	rate_roll(0.5, 0.1, 0, CSC_PID_IMAX),
	rate_pitch(0.5, 0.1, 0, CSC_PID_IMAX),
	rate_yaw(0.5, 0.1, 0, CSC_PID_IMAX),
	error_roll(1, 0.1, 0, CSC_PID_IMAX),
	error_pitch(1, 0.1, 0, CSC_PID_IMAX),
	error_yaw(1, 0.1, 0, CSC_PID_IMAX),
	*/

{
}

/* Cascaded PID Controller (CSC)
 *
 * This implementation a cascaded PID controller supports three calling interfaces from the planner.
 *
 * ## Control pipeline
 * We have three separate control pipelines that we each provide some sort of target, dependent on the run method we call. For example, in the run_position method,
 * the planner sets an (x,y,z) position as well as a yaw target. In the run_xy_position_z_velocity method though, the planner provides an (x,y) position but a 
 * target velocity for the z-axis (in addition to a yaw target).
 *
 * x and y position reference -> [csc_run_xy_position] -> x and y velocity reference -> [csc_run_xy_velocity] -> roll/pitch reference -> [csc_run_roll_pitch] -> roll/pitch rate targets -> [csc_run_rp_velocity] -> u_rt, u_pt (roll and pitch torque control actions)
 * 
 * z position reference -> [csc_run_z_position] -> z velocity reference -> [csc_run_z_velocity] -> u_force (control action)
 *
 * yaw reference -> [csc_run_yaw] -> yaw rate target -> [csc_run_yaw_velocity] -> u_yt (yaw torque control action)
 *
 */

void CSC_Controller::reset(void)
{
	// TODO Reset PID controller I-terms

	// Reset run counter so that our cascaded functions run at the correct frequency
	csc_counter = 0;

	return;
}

void CSC_Controller::run_position(void)
{
	// Run x-y controller pipeline
	csc_run_xy_position();
	
	// Run yaw pipeline
	csc_run_yaw();
	
	// Run z-position (throttle) pipeline
	csc_run_z_position();

	// At each base controller function, we always need to increment the csc_counter and also send the control action to the mixer
	csc_counter++;

	// TODO Fix hardcoded mass
	mixer.output(2.43*GRAVITY_MSS - u_force, u_rt, u_pt, u_yt);

	return;
}

void CSC_Controller::run_xy_position_z_velocity(void)
{
	// Run x-y position controller
	csc_run_xy_position();

	// Run z-velocity controller
	csc_run_z_velocity();

	// Run yaw controller
	csc_run_yaw();

	// At each base controller function, we always need to increment the csc_counter and also send the control action to the mixer
	csc_counter++;

	// TODO Fix hardcoded mass
	mixer.output(2.43*GRAVITY_MSS - u_force, u_rt, u_pt, u_yt);

	return;
}

void CSC_Controller::run_roll_pitch_z_velocity(void)
{
	// Run roll-pitch controller
	csc_run_roll_pitch();

	// Run z-velocity controller
	csc_run_z_velocity();

	// Run yaw controller
	csc_run_yaw();

	// At each base controller function, we always need to increment the csc_counter and also send the control action to the mixer
	csc_counter++;

	// TODO Fix hardcoded mass
	mixer.output(2.43*GRAVITY_MSS - u_force, u_rt, u_pt, u_yt);

	return;
}

void CSC_Controller::csc_run_roll_pitch(void)
{
	if (csc_counter%5==0) {
		Vector3f orientation = mcstate.get_euler_angles();
		reference[0].droll = error_roll.get_pi(reference[0].roll - orientation.x, 0.05);
		reference[0].dpitch = error_pitch.get_pi(reference[0].pitch - orientation.y, 0.05);
	}

	// TODO Do we need to limit the roll/pitch angular velocities?
	
	csc_run_rp_velocity();

	return;
}

void CSC_Controller::csc_run_yaw(void)
{
	if (csc_counter%5==0) {
		Vector3f orientation = mcstate.get_euler_angles();
		reference[0].dyaw = error_yaw.get_pi(reference[0].yaw - orientation.z, 0.05);
	}

	csc_run_yaw_velocity();

	return;
}

void CSC_Controller::csc_run_rp_velocity(void)
{
	Vector3f gyros = mincopter.ins.get_gyro();

	// Calculate roll/pitch torques
	u_rt = rate_roll.get_pi(reference[0].droll - gyros.x, 0.01);
	u_pt = rate_pitch.get_pi(reference[0].dpitch - gyros.y, 0.01);

	return;
}

void CSC_Controller::csc_run_yaw_velocity(void)
{
	Vector3f gyros = mincopter.ins.get_gyro();

	// Calculate yaw torque
	u_yt = rate_yaw.get_pi(reference[0].dyaw - gyros.z, 0.01);

	return;
}



void CSC_Controller::csc_run_xy_position(void)
{
	if (csc_counter%25==0) {
		Vector3f pos = mcstate.get_position();

		// Run xy position controllers
		reference[0].dx = nav_x_pos.get_pi(reference[0].x - pos.x, 0.25);
		reference[0].dy = nav_y_pos.get_pi(reference[0].y - pos.y, 0.25);

		// Constrain velocity to be between [-2,2]
		reference[0].dx = ap_max(-2.0f, ap_min(2.0f, reference[0].dx));
		reference[0].dy = ap_max(-2.0f, ap_min(2.0f, reference[0].dy));
	}

	csc_run_xy_velocity();

	return;
}

void CSC_Controller::csc_run_xy_velocity(void)
{
	if (csc_counter%5==0) {
		Vector3f vel = mcstate.get_velocity();
		float x_accel_target = nav_x_vel.get_pi(reference[0].dx - vel.x, 0.05);
		float y_accel_target = nav_y_vel.get_pi(reference[0].dy - vel.y, 0.05);

		// TODO This should really be the net z-axis body frame force and not just m*g (hover force)

		// TODO Hardcoded mass here needs to be configurable
		reference[0].roll = y_accel_target / (2.43*GRAVITY_MSS);
		reference[0].pitch = -1*x_accel_target / (2.43*GRAVITY_MSS);

		// Constrain 'pre-sin' roll,pitch to be between [-0.7,0.7] so that our actual desired roll,pitch is between [-pi/4, pi/4]
		reference[0].roll = ap_min(ap_max(reference[0].roll, -0.7), 0.7);
		reference[0].pitch = ap_min(ap_max(reference[0].pitch, -0.7), 0.7);

		reference[0].roll = safe_asin(reference[0].roll);
		reference[0].pitch = safe_asin(reference[0].pitch);
	}

	// TODO The above linear velocity controller runs at 20Hz but so does the roll/pitch controller? We should be slowing on of them
	csc_run_roll_pitch();

	return;
}

void CSC_Controller::csc_run_z_position(void)
{
	if (csc_counter%5==0) {
		Vector3f pos = mcstate.get_position();
		reference[0].dz = pos_throttle.get_pi(reference[0].z - pos.z, 0.05);
	}

	csc_run_z_velocity();

	return;
}

void CSC_Controller::csc_run_z_velocity(void)
{
	Vector3f vel = mcstate.get_velocity();
	u_force = vel_throttle.get_pi(reference[0].dz - vel.z, 0.01);

	return;
}




