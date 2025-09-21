
#include "mixer.h"

#include "mcinstance.h"
extern MCInstance mincopter;

void Mixer::output(float total_force_n, float roll_t_nm, float pitch_t_nm, float yaw_t_nm)
{
	float allocation_out;

	for (uint8_t i=0;i<4;i++) {
		allocation_out = 0;

		allocation_out += _allocation[4*i]*total_force_n;
		allocation_out += _allocation[4*i+1]*roll_t_nm;
		allocation_out += _allocation[4*i+2]*pitch_t_nm;
		allocation_out += _allocation[4*i+3]*yaw_t_nm;

		float candidate_rotor_vel = safe_sqrt(allocation_out);

		// Scale the desired rotor velocity to be between the minimum and maximum ESC PWM signal (default is [1000, 2000])
		candidate_rotor_vel = (candidate_rotor_vel/_rotor_vel_max_rads)*(_pwm_max_us - _pwm_min_us) + _pwm_min_us;

		// Constrain and convert to int16_t
		int16_t motor_out = constrain_int16((int16_t)candidate_rotor_vel, _pwm_min_us, _pwm_max_us);

		_motor_pwm_us[i] = motor_out;
	}

	// Write our calculated PWM signals to the corresponding HAL RC channel
	write_pwm_channel();

	// TODO Logging
	
	mincopter.hal.sim->control_input[0] = total_force_n;
	mincopter.hal.sim->control_input[1] = roll_t_nm;
	mincopter.hal.sim->control_input[2] = pitch_t_nm;
	mincopter.hal.sim->control_input[3] = yaw_t_nm;
	
	static uint32_t cnt=0;
	if (cnt%100==0) {
		mincopter.hal.console->printf("allocation: %d,%d,%d,%d\n", _motor_pwm_us[0], _motor_pwm_us[1], _motor_pwm_us[2], _motor_pwm_us[3]);
	}
	cnt++;
	
	return;
}

void Mixer::write_pwm_channel(void)
{
	for (uint8_t i=0;i<4;i++) {
		// Write value to motors
		// TODO We need to implement the mapping between motor and motor channel
		mincopter.hal.rcout->write(_motor_to_channel_map[i], _motor_pwm_us[i]);

		// TODO Remove this - bad hack
		mincopter.hal.sim->motor_out[i] = _motor_pwm_us[i];
	}

	return;
}
