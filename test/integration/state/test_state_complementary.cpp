
#include <AP_InertialSensor.h>
#include <AP_Compass.h>

#include <mcinstance.h>

#include <mcstate.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

/* Complementary Filter for AHRS test script */

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// GLOBALS
MCInstance mincopter;

StateComplementary mcstate;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(void)
{
	// Initialise/calibrate sensors
	
	// IMU
	mincopter.ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

	// Compass
	mincopter.compass.init();

	// Barometer
	// TODO Not yet fused in complementary sensor but need to add

	// Initialise state
	// TODO
	mcstate.init();

	hal.console->printf("State and sensors initialised\r\n");

	uint32_t loop_us;
	// This should always be ~10,000us
	uint32_t loop_fixed_us;

	/* Test 1. Read value */
	for (int i=0;i<1e7;i++) {
		uint32_t ts_start = hal.scheduler->micros();

		// Update INS sensors
		bool status = mincopter.ins.update();

		bool c_status = mincopter.compass.read();

		// Update quaternion
		mcstate.update();

		if (!status) {
			hal.console->printf("Error in IMU read\n");
			hal.scheduler->delay(100);
			continue;
		}

		if (!c_status) {
			hal.console->printf("Error in compass read\n");
			hal.scheduler->delay(100);
			continue;
		}

		/*
		Vector3f accel = mincopter.ins.get_accel();
		Vector3f gyro = mincopter.ins.get_gyro();
		Vector3f mag_field = mincopter.compass.get_field();
		*/

		// Log
		//Quaternion& _temp_att = mcstate._state._attitude;

		float roll = mcstate.data.euler.x;
		float pitch = mcstate.data.euler.y;
		float yaw = mcstate.data.euler.z;

		
		// Log to console (not telemetry) at 4Hz
		if (i%25==0) hal.uartA->printf("%fs,<RPY>% 8.3f (%8.3fdegc),% 8.3f (%8.3fdegc),% 8.3f (%8.3fdegc)\n",
				(hal.scheduler->micros()-loop_fixed_us)*1.0e-6f,
				roll,
				roll*180.0f/M_PI_F,
				pitch,
				pitch*180.0f/M_PI_F,
				yaw,
				yaw*180.0f/M_PI_F);

		loop_fixed_us = hal.scheduler->micros();
		/*
		hal.console->printf("<MAG>% 8.3f,% 8.3f,% 8.3f\n", mag_field.x, mag_field.y, mag_field.z);
		hal.console->printf("<ACC>% 8.3f,% 8.3f,% 8.3f\n", accel.x, accel.y, accel.z);
		hal.console->printf("<GYR>% 8.3f,% 8.3f,% 8.3f\n", gyro.x, gyro.y, gyro.z);
		*/

		loop_us = hal.scheduler->micros()-ts_start;


		// Delay (ms)
		hal.scheduler->delay_microseconds(ap_max(0, 10000 - loop_us));
	}

	return 0;
}


int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	run_unit_tests();

	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

