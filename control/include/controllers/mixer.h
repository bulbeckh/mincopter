
#include <stdint.h>

/* Default implementation of QuadCopter mixer
 *
 *
 *
 */

class Mixer {

	public:
		Mixer(void) :
			_pwm_min_us(1000),
			_pwm_max_us(2000),
			// TODO For now, align the maximum rotor velocity with the PWM @ 2000rad/s
			_rotor_vel_max_rads(2000)
		{
			// Params with kt=2.05e-6
			/*
			float c1 = 121951.2195122;
			float c2 = 609756.09756097;
			float c3 = 609756.09756098;
			float c4 = 6097560.97560976;
			*/

			// kt=1.5e-6
			float c1 = 238095;
			float c2 = 1190476;
			float c3 = 1190476; 
			float c4 = 11904761;

			// Setup allocation
			
			_allocation[0] = c1;
			_allocation[1] = -c2;
			_allocation[2] = c3;
			_allocation[3] = c4;

			_allocation[4] = c1;
			_allocation[5] = c2;
			_allocation[6] = -c3;
			_allocation[7] = c1;

			_allocation[8] = c1;
			_allocation[9] = -c2;
			_allocation[10] = -c3;
			_allocation[11] = -c4;

			_allocation[12] = c1;
			_allocation[13] = c2;
			_allocation[14] = c3;
			_allocation[15] = -c4;

		}

	public:

		/* @brief Motor output method that calculates the required PWM signal for each motor ESC based
		 * on the controller output - force, <roll,pitch,yaw> torque */
		void output(float total_force_n, float roll_t_nm, float pitch_t_nm, float yaw_t_nm);

	private:

		/* @brief Write calculated PWM signal to corresponding RC channel (ESC) */
		void write_pwm_channel(void);

	private:

		/* @brief Maximum and minimum values for the PWM interval for each motor, in us (microseconds).
		 * NOTE This value is what is written to the HAL PWM timer */
		int16_t _pwm_min_us;
		int16_t _pwm_max_us;

		/* @brief Individual motor velocity maximums in rad/s. Used to constrain and scale the calculated
		 * rotor speeds to a PWM value */
		float _rotor_vel_max_rads;

		/* @brief The PWM signals for each motor/ESC in the range [1000,2000] */
		// TODO Remove the hardcoded 4 for num motors
		int16_t _motor_pwm_us[4];

		/* @brief Inverse (psuedo-) of mixer allocation to calculate indivudal motor speeds. NOTE Allocation
		 * is row-major */
		float _allocation[16];

		/* @brief Mapping from each motor to the corresponding RC channel (output pin) where the PWM signal is sent */
		uint8_t _motor_to_channel_map[4];

};

