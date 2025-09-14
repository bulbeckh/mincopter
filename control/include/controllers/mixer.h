

/* Default implementation of QuadCopter mixer
 *
 *
 *
 */

class Mixer {

	public:
		Mixer(void) :
			_pwm_min_us(1000),
			_pwm_max_us(2000)
		{ }

	public:
		/* @brief Motor output method that calculates the required PWM signal for each motor ESC based
		 * on the controller output - force, <roll,pitch,yaw> torque */
		void output(float total_force_n, float roll_t_nm, float pitch_t_nm, float yaw_t_nm);

	private:

		/* @brief Maximum and minimum values for the PWM interval for each motor, in us (microseconds).
		 * NOTE This value is what is written to the HAL PWM timer */
		int16_t _pwm_min_us;
		int16_t _pwm_max_us;

		/* @brief Individual motor velocity maximums in rad/s. Used to constrain and scale the calculated
		 * rotor speeds to a PWM value */
		float _rotor_vel_max_rads;

		/* @brief Inverse (psuedo-) of mixer allocation to calculate indivudal motor speeds. NOTE Allocation
		 * is row-major */
		float _allocation[16] = {
			// Row 0
			121951.2195122,
			-609756.09756097,
			609756.09756098,
			6097560.97560976,
			// Row 1
			121951.2195122,
			609756.09756098,
			-609756.09756097,
			6097560.97560976,
			// Row 2
			121951.2195122,
			-609756.09756098,
			-609756.09756098,
			-6097560.97560976,
			// Row 3
			121951.2195122,
			609756.09756097,
			609756.09756097,
			-6097560.97560976
		};


};

