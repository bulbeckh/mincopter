
#include "controller_lqr.h"

#include <AP_Math.h>

#include "mcstate.h"
extern MCState mcstate;

#include "mcinstance.h"
extern MCInstance mincopter;

// Instance
LQR_Controller controller;

void LQR_Controller::run(void)
{
	// Retrieve values from state matrix and then calculate gain U = -k*x
	
	// NOTE Still need to use mixer algorithm to convert from U to individual motor speeds/outputs/PWM signals
	
	float _state[12];

	Vector3f state_pos = mcstate.get_position();
	Vector3f state_orientation = mcstate.get_euler_angles();

	_state[0] = state_pos[0];
	_state[1] = state_pos[1];
	_state[2] = state_pos[2];

	_state[3] = state_orientation[0];
	_state[4] = state_orientation[1];
	_state[5] = state_orientation[2];

	Vector3f state_vel = mcstate.get_velocity();

	// TODO Are these body frame angular velocities??
	Vector3f state_angular_vel = mincopter.ins.get_gyro();

	_state[6] = state_vel[0];
	_state[7] = state_vel[1];
	_state[8] = state_vel[2];

	_state[9] = state_angular_vel[0];
	_state[10] = state_angular_vel[1];
	_state[11] = state_angular_vel[2];

	// Calculate control vector and call mixer
	
	// TODO Change _state to be a member variable to avoid the object being passed
	control_output(_state);

	return;
}

void LQR_Controller::control_output(float* _state)
{
	/* We use this control law:
	 *
	 * U = -K @ x 
	 *
	 */

	// Our LQR controller directs each state to zero so we need to offset by our targets
	
	// Offset by a target of Z=-20
	_state[2] -= -20;

	// Offset yaw by ~11degc because we haven't yet offset the yaw from magnetometer
	_state[5] -= -0.19;

	float control_out[4] = {0,0,0,0};

	for (uint8_t i=0;i<4;i++) {
		for (uint8_t j=0;j<12;j++) {
			// K @ x
			control_out[i] += lqr_k[i*12 + j]*_state[j];
		}

		control_out[i] *= -1;
	}

	// Add back our gravity vector * mass
	control_out[0] += 2.43f*GRAVITY_MSS;

	// Call mixer
	mixer.output(control_out[0], control_out[1], control_out[2], control_out[3]);

	return;
}

