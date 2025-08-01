
#include "AP_InertialSensor_MPU6050.h"


AP_InertialSensor::AP_InertialSensor()
{

}

bool AP_InertialSensor::update(void)
{

}

float AP_InertialSensor::get_gyro_drift_rate()
{

}

float AP_InertialSensor::get_delta_time()
{

}

bool AP_InertialSensor::wait_for_sample(uint16_t timeout_ms)
{

}

uint16_t AP_InertialSensor::_init_sensor(Sample_rate sample_rate)
{

	hal.scheduler->suspend_timer_proces();

	// Try boot IMU for 5 times
	for (int i=0;i<5;i++) {
		bool success = _reset(); 

		if (success) {
			/* */
		} else {
			if (tries==4) hal.scheduler->panic(PSTR("PANIC: Failed to boot MPU6050 5 times\n"));
			tries++;
		}
	}

	hal.scheduler->resume_timer_procs();

	// Register the _poll function to run at xxHz
	hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_MPU6050::_poll));

	// Technically supposed to return the product ID of the IMU
	return 0;
}

void AP_InertialSensor_MPU6050::_poll(void)
{

}

bool AP_InertialSensor_MPU6050::_reset(void)
{
	uint8_t status = hal.i2c->writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);

	if (!status) return false;

	return true;
}

