
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

	Vector3f orientation = mcstate.get_euler_angles();
	Vector3f gyros = mincopter.ins.get_gyro();

	Vector3f pos = mcstate.get_position();
	Vector3f vel = mcstate.get_velocity();

	if (csc_counter%5==0) {
		roll_rate_target = error_roll.get_pi(0 - orientation.x, 0.05);
		pitch_rate_target = error_pitch.get_pi(0 - orientation.y, 0.05);
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


