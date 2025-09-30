
#include "controller_csc.h"

#include <AP_Math.h>

#define CSC_PID_IMAX 100

#include "mcinstance.h"
extern MCInstance mincopter;

#include "mcstate.h"
extern MCState mcstate;

// Instance
CSC_Controller controller;

CSC_Controller::CSC_Controller()
	: MC_Controller(),
	rate_roll(0.5, 0.1, 0, CSC_PID_IMAX),
	rate_pitch(0.5, 0.1, 0, CSC_PID_IMAX),
	rate_yaw(0.5, 0.1, 0, CSC_PID_IMAX),
	error_roll(20, 0.1, 0, CSC_PID_IMAX),
	error_pitch(20, 0.1, 0, CSC_PID_IMAX),
	error_yaw(20, 0.1, 0, CSC_PID_IMAX),

	vel_throttle(1.0,0,0, CSC_PID_IMAX),
	pos_throttle(1.0,0,0, CSC_PID_IMAX),

	// Navigation Controllers - TODO May be moved
	nav_x_pos(1.0, 0, 0, CSC_PID_IMAX),
	nav_y_pos(1.0, 0, 0, CSC_PID_IMAX),
	nav_x_vel(1, 0, 0, CSC_PID_IMAX),
	nav_y_vel(1, 0, 0, CSC_PID_IMAX),

	/*
	rate_roll(0.5, 0.1, 0, CSC_PID_IMAX),
	rate_pitch(0.5, 0.1, 0, CSC_PID_IMAX),
	rate_yaw(0.5, 0.1, 0, CSC_PID_IMAX),
	error_roll(1, 0.1, 0, CSC_PID_IMAX),
	error_pitch(1, 0.1, 0, CSC_PID_IMAX),
	error_yaw(1, 0.1, 0, CSC_PID_IMAX),
	*/

	csc_counter(0)
{

}

void CSC_Controller::run(void)
{
	/* This is a quick hack in place of a 'take-off' function to high-throttle all motors for first half second */
	/*
	if (csc_counter<300) {
		mixer.output(2.43*GRAVITY_MSS+5, 0, 0, 0);
		csc_counter ++;
		return;
	}
	*/

	Vector3f orientation = mcstate.get_euler_angles();
	Vector3f gyros = mincopter.ins.get_gyro();

	Vector3f pos = mcstate.get_position();
	Vector3f vel = mcstate.get_velocity();

	if (csc_counter%25==0) {
		// Run outer nav loop at 4Hz
		// NOTE Target of x=10m, y=10m
		x_vel_target = nav_x_pos.get_pi(/* X-Target */ 10 - pos.x, 0.25);
		y_vel_target = nav_y_pos.get_pi(/* Y-Target */ 10 - pos.y, 0.25);
	}

	if (csc_counter%5==0) {
		// Run nav inner

		float x_accel_target = nav_x_vel.get_pi(x_vel_target - vel.x, 0.05);
		float y_accel_target = nav_y_vel.get_pi(y_vel_target - vel.y, 0.05);

		// TODO This should really be the net z-axis body frame force and not just m*g (hover force)

		// TODO Hardcoded mass here needs to be configurable
		float desired_roll = y_accel_target / (2.43*GRAVITY_MSS);
		float desired_pitch = -1*x_accel_target / (2.43*GRAVITY_MSS);

		// Constrain 'pre-sin' roll,pitch to be between [-0.7,0.7] so that our actual desired roll,pitch is between [-pi/4, pi/4]
		desired_roll = ap_min(ap_max(desired_roll, -0.7), 0.7);
		desired_pitch = ap_min(ap_max(desired_pitch, -0.7), 0.7);

		desired_roll = safe_asin(desired_roll);
		desired_pitch = safe_asin(desired_pitch);

		// Run angle error controllers (outer)
		//roll_rate_target = error_roll.get_pi(0 - orientation.x, 0.05);
		//pitch_rate_target = error_pitch.get_pi(0 - orientation.y, 0.05);

		// In the first 2secs, we just zero the roll and pitch when we 'takeoff'
		if (csc_counter<200) {
			roll_rate_target = error_roll.get_pi(0 - orientation.x, 0.05);
			pitch_rate_target = error_pitch.get_pi(0 - orientation.y, 0.05);
		} else {
			roll_rate_target = error_roll.get_pi(desired_roll - orientation.x, 0.05);
			pitch_rate_target = error_pitch.get_pi(desired_pitch - orientation.y, 0.05);
		}

		yaw_rate_target = error_yaw.get_pi(0 - orientation.z, 0.05);

		// Throttle ctrl
		// NOTE We have set a target of 20m here
		vert_vel_target = pos_throttle.get_pi(-20 - pos.z, 0.05);
	}

	float rt = rate_roll.get_pi(roll_rate_target - gyros.x, 0.01);
	float pt = rate_pitch.get_pi(pitch_rate_target - gyros.y, 0.01);
	float yt = rate_yaw.get_pi(yaw_rate_target - gyros.z, 0.01);

	float tforce = vel_throttle.get_pi(vert_vel_target - vel.z, 0.01);

	// NOTE Confusingly, we subtract tforce here because the output of our controller is negative if we desire to go up, since we have chosen a NED world frame.
	mixer.output(2.43*GRAVITY_MSS - tforce, rt, pt, yt);

	if (csc_counter%100) {
		mincopter.hal.console->printf("CSC: %f, %f, %f, %f\n", 2.43*GRAVITY_MSS - tforce, rt, pt, yt);
	}

	csc_counter++;

	return;
}


