
#include <AP_InertialSensor.h>
#include <AP_Compass.h>

// HASH include <MCState.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(AP_InertialSensor& _imu, Compass& _compass)
{
	// Initialise IMU
	_imu.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

	// Initialise Compass
	_compass.init();

	/* Test 1. Read value */
	for (int i=0;i<250;i++) {
		// Loop for 5 seconds at 50Hz
		uint32_t ts_start = hal.scheduler->micros();

		bool status = _imu.update();
		bool c_status = _compass.read();

		if (!status || !c_status) {
			hal.console->printf("Error in IMU or compass read\n");
			return 1;
		}

		Vector3f accel = _imu.get_accel();
		Vector3f gyro = _imu.get_gyro();

		Vector3f mag_field = _compass.get_field();

		hal.console->printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%lu\n",
				accel.x, accel.y, accel.z,
				gyro.x, gyro.y, gyro.z,
				mag_field.x, mag_field.y, mag_field.z,
				ts_start);

		uint32_t loop_us = hal.scheduler->micros()-ts_start;

		// 50Hz = 20,000us
		
		int32_t delay_us = 20000-loop_us;
		if (delay_us<0) delay_us=0;

		// Delay (ms)
		hal.scheduler->delay_microseconds(delay_us);
	}

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
#if defined(MC_IMU_MPU6050)
	AP_InertialSensor_MPU6050 imu;
#elif defined(MC_IMU_MPU6000)
	AP_InertialSensor_MPU6000 imu;
#endif

#if defined(MC_COMP_HMC5843)
	AP_Compass_HMC5843 compass;
#elif defined(MC_COMP_ICM20948)
	AP_Compass_ICM20948 compass;
#endif

	run_unit_tests(imu, compass);

	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

