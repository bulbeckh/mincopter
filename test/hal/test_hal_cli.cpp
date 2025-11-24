
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(void)
{
	// Init

	uint8_t buff[8] = {0};
	uint8_t _bc = 0;

	uint32_t loop_time_us = 0;

	/* Here is where we will update the PWM signals and enable them */
	
	// Set min throttle for channel 0
	hal.rcout->write(0, 1000);
	hal.rcout->enable_ch(0);

	// Run this loop at 100Hz
	for (int i=0;i<1e6;i++) {
		uint32_t start = hal.scheduler->micros();

		int16_t nb = hal.console->read();
		if (nb!=-1) _bc++;

		// Log received characters
		//hal.console->printf("%.*s\n", _bc==8 ? _bc : _bc+1, buff);

		// Clear buffer
		for (uint8_t j=0;j<8;j++) buff[j]=0;



		// Heartbeat every second
		if (i%100==0) {
			hal.console->printf("[%luus] heartbeat %d, received %u\r\n", hal.scheduler->micros()-loop_time_us, i, _bc);

			uint16_t pwm_read[4];
			hal.rcout->read(pwm_read, 4);

			hal.console->printf("\tPWM0 %u, PWM1 %u, PWM2 %u, PWM3 %u\r\n", pwm_read[0], pwm_read[1], pwm_read[2], pwm_read[3]);

			// Toggle the PWM signal every 5 seconds
			if ( (i%1000) / 500) {
				hal.rcout->write(0, 1000);
			} else {
				hal.rcout->write(0, 2000);
			}
		}

		// Record loop time
		loop_time_us = hal.scheduler->micros();
	
		// Delay
		uint32_t elapsed = hal.scheduler->micros() - start;
		if (elapsed < 10000) hal.scheduler->delay_microseconds(10000-elapsed);
	}

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	// TODO NOTE We would never have an instance where a hardware target has all types of IMUs should maybe
	// remove the MC_TEST_IMU_ALL flag/option

	// Run tests
	run_unit_tests();

}

