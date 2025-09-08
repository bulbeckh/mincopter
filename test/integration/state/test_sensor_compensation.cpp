
#include <AP_InertialSensor.h>
#include <AP_Compass.h>

// HASH include <MCState.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

/* Script to continuously measure max and min readings @ 50Hz for accel and compass and then dump every 5 sec */

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


static float m_limit[6]; // mx_min, mx_max, my_min, ....
static float a_limit[6];

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(AP_InertialSensor& _imu, Compass& _compass)
{
	// Initialise IMU
	_imu.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

	// Initialise Compass
	_compass.init();

	for (uint8_t i=0;i<3;i++) {
		a_limit[2*i] = 1e9; // MIN
		a_limit[2*i+1] = -1e9; // MAX

		m_limit[2*i] = 1e9; // MIN
		m_limit[2*i+1] = -1e9; // MAX
	}

	/* Test 1. Read value */
	for (int i=0;i<3000;i++) {
		// Loop for 60 seconds at 50Hz
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

		// MIN
		if (accel.x<a_limit[0]) a_limit[0] = accel.x;
		if (accel.y<a_limit[2]) a_limit[2] = accel.y;
		if (accel.z<a_limit[4]) a_limit[4] = accel.z;
		// MAX
		if (accel.x>a_limit[1]) a_limit[1] = accel.x;
		if (accel.y>a_limit[3]) a_limit[3] = accel.y;
		if (accel.z>a_limit[5]) a_limit[5] = accel.z;

		// MIN
		if (mag_field.x<m_limit[0]) m_limit[0] = mag_field.x;
		if (mag_field.y<m_limit[2]) m_limit[2] = mag_field.y;
		if (mag_field.z<m_limit[4]) m_limit[4] = mag_field.z;
		// MAX
		if (mag_field.x>m_limit[1]) m_limit[1] = mag_field.x;
		if (mag_field.y>m_limit[3]) m_limit[3] = mag_field.y;
		if (mag_field.z>m_limit[5]) m_limit[5] = mag_field.z;

		if (i%50==0) {
			hal.console->printf("%d\n	mx_min:%f mx_max:%f\n	my_min:%f my_max:%f\n	mz_min:%f mz_max:%f\n	ax_min:%f ax_max:%f\n	ay_min:%f ay_max:%f\n	az_min:%f az_max%f\n",
					i,
					m_limit[0], m_limit[1], m_limit[2], m_limit[3], m_limit[4], m_limit[5],
					a_limit[0], a_limit[1], a_limit[2], a_limit[3], a_limit[4], a_limit[5]);
		}

		/*
		hal.console->printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%lu\n",
				accel.x, accel.y, accel.z,
				gyro.x, gyro.y, gyro.z,
				mag_field.x, mag_field.y, mag_field.z,
				ts_start);
		*/

		uint32_t loop_us = hal.scheduler->micros()-ts_start;

		// 50Hz = 20,000us
		
		int32_t delay_us = 20000-(int32_t)loop_us;

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

