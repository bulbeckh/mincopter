
#include <AP_InertialSensor.h>
#include <AP_Compass.h>

#include <mcstate.h>
#include <mcinstance.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

/* Complementary Filter for AHRS test script */

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// GLOBALS
MCState mcstate; 
MCInstance mincopter;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(void)
{
	/* Test 1. Read value */
	for (int i=0;i<1e7;i++) {
		uint32_t ts_start = hal.scheduler->micros();

		// Update INS sensors
		bool status = mincopter.ins.update();
		bool c_status = mincopter.compass.read();

		// Update quaternion
		mcstate.ahrs.ahrs_update();

		if (!status || !c_status) {
			hal.console->printf("Error in IMU or compass read\n");
			return 1;
		}

		Vector3f accel = mincopter.ins.get_accel();
		Vector3f gyro = mincopter.ins.get_gyro();
		Vector3f mag_field = mincopter.compass.get_field();

		if (i%50==0) {
			Quaternion& _temp_att = mcstate._state._attitude;

			float roll,pitch,yaw;
			_temp_att.to_euler(&roll, &pitch, &yaw);

			// IN NED
			hal.console->printf("<RPY>% 8.3f,% 8.3f,% 8.3f\n", roll, pitch, yaw);
			hal.console->printf("<MAG>% 8.3f,% 8.3f,% 8.3f\n", mag_field.x, mag_field.y, mag_field.z);
			hal.console->printf("<ACC>% 8.3f,% 8.3f,% 8.3f\n", accel.x, accel.y, accel.z);
			hal.console->printf("<GYR>% 8.3f,% 8.3f,% 8.3f\n", gyro.x, gyro.y, gyro.z);

		}

		// 
		uint32_t loop_us = hal.scheduler->micros()-ts_start;

		// 50Hz = 20,000us
		int32_t delay_us = 10000-(int32_t)loop_us;

		if (i%50==0) {
			hal.console->printf("(loop, delay): %lu, %ld\n", loop_us, delay_us);
		}

		if (delay_us<0) {
			delay_us=0;
		}

		// Delay (ms)
		hal.scheduler->delay_microseconds(delay_us);
	}

	return 0;
}


int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	// Initialise sensors
	mincopter.ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
	mincopter.compass.init();

	// Initialise state
	mcstate.init();

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

