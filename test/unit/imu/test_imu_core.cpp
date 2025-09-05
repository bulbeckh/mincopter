
#include <AP_InertialSensor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(AP_InertialSensor& _imu)
{
	// Initialise IMU
	_imu.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

	/* Test 1. Read value */
	for (int i=0;i<8;i++) {
		bool status = _imu.update();

		if (!status) {
			hal.console->printf("Error in IMU read\n");
			return 1;
		}

		Vector3f accel = _imu.get_accel();

		hal.console->printf("X: %f, Y: %f, Z: %f\n", accel.x, accel.y, accel.z);

		// Delay 10ms
		hal.scheduler->delay(10);
	}

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
#if defined(MC_IMU_MPU6050) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_MPU6050 imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_MPU6000) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_MPU6000 imu;
	run_unit_tests(imu);
#endif
	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

