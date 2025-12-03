
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include "telemetry.h"

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Stub functions
void mincopter_telemetry_command_heartbeatrequest(void* argptr) {}
void mincopter_telemetry_command_armrequest(void* argptr) {}
void mincopter_telemetry_command_disarmrequest(void* argptr) {}
void mincopter_telemetry_command_flightstaterequest(void* argptr) {}

// Telemetry instance
Telemetry telemetry;

// Set target_pwm to default of min
uint16_t target_pwm[4] = {1000,1000,1000,1000};

// Whether we had at least one connection to GCS
uint8_t gcs_connected = 0;

uint8_t do_heartbeat = 0;

void mincopter_telemetry_command_testrequest(void* argptr)
{
	uint8_t test_type = ((uint8_t*)argptr)[7];

	// Assume that test_type = 0xF0 motor tests
	if (test_type != 0xF0) {
		hal.console->printf("Wrong test type\r\n");
		return;
	}

	// NOTE: PWM allocation is in reverse order
	uint8_t* pwm_bytes_reversed = (uint8_t*)argptr;

	// Convert to PWM and write
	for (uint8_t i=0;i<4;i++) {
		uint16_t pwm_target = 1000 + pwm_bytes_reversed[i]*(1000/256.0f);
		// TODO Disabled temporarily
		target_pwm[3-i] = ap_max(ap_min(2000,pwm_target), 1000);
	}

	hal.console->printf("Set Target PWM: %u,%u,%u,%u\r\n",
			target_pwm[0],
			target_pwm[1],
			target_pwm[2],
			target_pwm[3]);
	hal.console->printf("argbuff: 0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X\r\n",
			pwm_bytes_reversed[0],
			pwm_bytes_reversed[1],
			pwm_bytes_reversed[2],
			pwm_bytes_reversed[3],
			pwm_bytes_reversed[4],
			pwm_bytes_reversed[5],
			pwm_bytes_reversed[6],
			pwm_bytes_reversed[7]);


	// TODO Temporarily disabled
	// Run for 5 seconds
	do_heartbeat = 5;

	return;
}


// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(void)
{

	// Tracks runtime (in us) of each loop to ensure we stay at 100Hz
	uint32_t loop_time_us = 0;


	/* Here is where we will update the PWM signals and enable them */

	// Set min throttle for all channels
	for (uint8_t i=0;i<4;i++) {
		hal.rcout->write(i, 1000);
		hal.rcout->enable_ch(i);
	}

	// Run this loop at 100Hz
	for (int i=0;i<1e6;i++) {

		uint32_t start = hal.scheduler->micros();
		
		// Do a telemetry read
		telemetry.read(8);

		// If we are currently in a run, we send a signal to PWM0
		if (do_heartbeat) {
			hal.rcout->write(0, target_pwm[0]);
			hal.rcout->write(1, target_pwm[1]);
			hal.rcout->write(2, target_pwm[2]);
			hal.rcout->write(3, target_pwm[3]);

			/*
			hal.rcout->write(0, 1000);
			hal.rcout->write(1, 1000);
			hal.rcout->write(2, 1000);
			hal.rcout->write(3, 1000);
			*/
		} else {
			hal.rcout->write(0, 1000);
			hal.rcout->write(1, 1000);
			hal.rcout->write(2, 1000);
			hal.rcout->write(3, 1000);
		}

		// Heartbeat every second
		if (i%100==0) {
			uint16_t pwm_read[4];
			hal.rcout->read(pwm_read, 4);

			hal.console->printf("\tPWM0 %u, PWM1 %u, PWM2 %u, PWM3 %u\r\n", pwm_read[0], pwm_read[1], pwm_read[2], pwm_read[3]);

		}

		if (i%100==0 && do_heartbeat) {
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

