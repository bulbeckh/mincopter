
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


/* Design of simple console
 *
 * ## Receiver Interface
 *
 * Byte 1. Sync byte - always 0x24 ($)
 * Byte 2. Command Type (# args)
 * 	- 0x0A Receive Heartbeat (1)
 * 	- 0x0B Set PWM (2)
 * 	- 0x0C Run Motor Test (0)
 *
 * Byte 3+ Args
 * - 0x0A: [ heartbeat sequence number ]
 * - 0x0B: [ motor number, pwm val idx (0 = 1100, 1 = 1200, 2 = 1300, ... ) ]
 * - 0x0C: no args
 *
 *
 */


// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(void)
{
	// Byte counter used to track how many total bytes we have received/parsed
	uint32_t _bc = 0;

	// Tracks runtime (in us) of each loop to ensure we stay at 100Hz
	uint32_t loop_time_us = 0;

	uint8_t do_heartbeat = 0;

	// Whether we had at least one connection to GCS
	uint8_t gcs_connected = 0;

	/* Here is where we will update the PWM signals and enable them */

	// Set min throttle for all channels
	for (uint8_t i=0;i<4;i++) {
		hal.rcout->write(i, 1000);
		hal.rcout->enable_ch(i);
	}

	// Set target_pwm to default of min
	uint16_t target_pwm[4] = {1000,1000,1000,1000};

	// 0 = sync byte, 1 = command type, 3+ = args
	uint8_t cmd_state=0;
	uint8_t cmd_type=0;
	uint8_t remaining=0;

	// Run this loop at 100Hz
	for (int i=0;i<1e6;i++) {
		uint32_t start = hal.scheduler->micros();

		// We read up to 8 bytes from the console each iteration
		uint8_t read_counter = 8;
		int16_t nb;

		while (read_counter) {
			read_counter--;

			// Read a single byte
			nb = hal.console->read();

			if (nb!=-1) {
				_bc++;

				switch (cmd_state) {
					case 0:
						// Sync byte
						if (nb!=0x24) {
							// Log packet miss and reset state
							hal.console->printf("cli pkt miss! resetting\r\n");
							cmd_state = 0;
							cmd_type = 0;
							remaining = 0;
							break;
						}

						// Increment to next state
						cmd_state++;
						break;

					case 1:
						// Command Type
						switch (nb) {
							case 0x0A:
								// Receive heartbeat
								cmd_type = 0x0A;
								
								// Flag that we have connected to GCS at least once and that we should maintain connection
								gcs_connected = 0xFF;

								// Expect 1 packet (sequence identifier)
								remaining = 1;
								cmd_state++;
								break;
							case 0x0B:
								// Set PWM
								cmd_type = 0x0B;
								// Expect 2 packets (motor #, pwm index)
								remaining = 2;
								cmd_state++;
								break;
							case 0x0C:
								// Run test
								cmd_type = 0x0C;
								do_heartbeat = 5;
								remaining = 0;
								break;
							default:
								hal.console->printf("Wrong cmd type! resetting\r\n");
								cmd_state = 0;
								cmd_type = 0;
								remaining = 0;
								break;
						}

						// For some command types, we don't expect any more arguments so we should reset
						if (!remaining) {
							cmd_state=0;
							cmd_type=0;
						}
						break;

					case 2:
						// Arguments
						remaining--;

						/*
						if (cmd_type==0x0A) {
							if (nb

						TODO */


				}


				/*
				switch(nb) {
					// PWM0
					case 0x60:
						target_pwm[0] = 1100;
						break;
					case 0x61:
						target_pwm[0] = 1200;
						break;
					case 0x62:
						target_pwm[0] = 1300;
						break;

					// PWM1
					case 0x63:
						target_pwm[1] = 1100;
						break;
					case 0x64:
						target_pwm[1] = 1200;
						break;
					case 0x65:
						target_pwm[1] = 1300;
						break;

					// PWM2
					case 0x66:
						target_pwm[2] = 1100;
						break;
					case 0x67:
						target_pwm[2] = 1200;
						break;
					case 0x68:
						target_pwm[2] = 1300;
						break;

					// PWM3
					case 0x69:
						target_pwm[3] = 1100;
						break;
					case 0x6A:
						target_pwm[3] = 1200;
						break;
					case 0x6B:
						target_pwm[3] = 1300;
						break;

					// Run setup for 5sec
					case 0x9F:
						do_heartbeat = 5;

					default:
						break;
				}
				*/
			} else {
				// We have reached end of input stream so we break;
				break;
			}
		}

		// After we first connect to a GCS, every 100ms we send a heartbeat and expect the GCS to respond within 100ms
		if (i%10==0) {
			// TODO Send heartbeat to GCS

			/*
			if (gcs_connected && // last heartbeat does not match sequence then flag that we have lost connection 

			}
			*/
			

		}

		// If we are currently in a run, we send a signal to PWM0
		if (do_heartbeat) {
			hal.rcout->write(0, target_pwm[0]);
			hal.rcout->write(1, target_pwm[1]);
			hal.rcout->write(2, target_pwm[2]);
			hal.rcout->write(3, target_pwm[3]);
		} else {
			hal.rcout->write(0, 1000);
			hal.rcout->write(1, 1000);
			hal.rcout->write(2, 1000);
			hal.rcout->write(3, 1000);
		}

		// Heartbeat every second
		if (i%100==0 && do_heartbeat) {
			hal.console->printf("[%luus] heartbeat %d, received %lu\r\n", hal.scheduler->micros()-loop_time_us, i, _bc);

			uint16_t pwm_read[4];
			hal.rcout->read(pwm_read, 4);

			hal.console->printf("\tPWM0 %u, PWM1 %u, PWM2 %u, PWM3 %u\r\n", pwm_read[0], pwm_read[1], pwm_read[2], pwm_read[3]);

			// Decrement by 1 each second
			do_heartbeat--;
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

