
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
	error_roll(1, 0.1, 0, CSC_PID_IMAX),
	error_pitch(1, 0.1, 0, CSC_PID_IMAX),
	error_yaw(1, 0.1, 0, CSC_PID_IMAX),

	/*
	rate_roll(0.05, 0, 0, CSC_PID_IMAX),
	rate_pitch(0.05, 0, 0, CSC_PID_IMAX),
	rate_yaw(0.05, 0, 0, CSC_PID_IMAX),
	error_roll(0.05, 0, 0, CSC_PID_IMAX),
	error_pitch(0.05, 0, 0, CSC_PID_IMAX),
	error_yaw(0.05, 0, 0, CSC_PID_IMAX),
	*/

	csc_counter(0)
{

}

void CSC_Controller::run(void)
{

	Vector3f orientation = mcstate.get_euler_angles();

	Vector3f gyros = mincopter.ins.get_gyro();

	if (csc_counter%5==0) {
		roll_rate_target = error_roll.get_pi(0 - orientation.x, 0.05);
		pitch_rate_target = error_pitch.get_pi(0 - orientation.y, 0.05);
		yaw_rate_target = error_yaw.get_pi(0 - orientation.z, 0.05);
		
	}

	float rt = rate_roll.get_pi(roll_rate_target - gyros.x, 0.01);
	float pt = rate_pitch.get_pi(pitch_rate_target - gyros.y, 0.01);
	float yt = rate_yaw.get_pi(yaw_rate_target - gyros.z, 0.01);

	mixer.output(2.43*GRAVITY_MSS, rt, pt, yt);

	if (csc_counter%100) {
		mincopter.hal.console->printf("CSC: %f, %f, %f, %f\n", 2.43*GRAVITY_MSS, rt, pt, yt);
	}

	csc_counter++;

	return;
}


