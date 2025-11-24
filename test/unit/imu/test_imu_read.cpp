
#include <AP_InertialSensor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(AP_InertialSensor& _imu)
{

	// Initialise IMU + gyro/accel calibrate (offsets, scaling)
	_imu.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

	// Run this loop at 100Hz
	for (int i=0;i<1e6;i++) {
		uint32_t start = hal.scheduler->micros();

#if defined(MC_IMU_SIM)
		// Tick simulation for a 100Hz interval
		hal.sim->tick(10000);
#endif

		bool status = _imu.update();

		if (!status) {
			hal.console->printf("Error in IMU read\n");
			return 1;
		}

		// Read accelerometer
		Vector3f accel = _imu.get_accel();

		// Read gyrometer
		Vector3f gyro  = _imu.get_gyro();

		hal.console->printf("%d,acc X: %f, Y: %f, Z: %f, gyro X: %f, Y: %f, Z: %f\r\n", i,
				accel.x, accel.y, accel.z,
				gyro.x, gyro.y, gyro.z);

		// Delay
		uint32_t elapsed = hal.scheduler->micros() - start;
		//if (elapsed < 10000) hal.scheduler->delay(10000-elapsed);
		// TODO Confirm that in simulation, the delay is actually working correctly
	}

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	// TODO NOTE We would never have an instance where a hardware target has all types of IMUs should maybe
	// remove the MC_TEST_IMU_ALL flag/option

#if defined(MC_IMU_MPU6050) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_MPU6050 imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_MPU6000) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_MPU6000 imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_ICM20948) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_ICM20948 imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_L3G4200D) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_L3G4200D imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_SIM) || defined(MC_TEST_IMU_ALL)
	// TODO Do we need to add/instantiate any other sim parameters here or is that handled in the hal.sim object
	AP_InertialSensor_Sim imu;
	run_unit_tests(imu);
#endif

}

