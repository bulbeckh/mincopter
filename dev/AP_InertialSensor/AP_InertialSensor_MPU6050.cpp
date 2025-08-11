
#include "AP_InertialSensor_MPU6050.h"

#include <stdio.h>

// MPU6050 Register Definitions
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

extern const AP_HAL::HAL& hal;

AP_InertialSensor_MPU6050::AP_InertialSensor_MPU6050()
{

}

bool AP_InertialSensor_MPU6050::update(void)
{
	// NOTE TODO The majority of the 'reading' is done in _poll but it should just be accumulating in poll and actually calculated here
	
	wait_for_sample(1000);
	return true;
}

float AP_InertialSensor_MPU6050::get_gyro_drift_rate()
{

}

float AP_InertialSensor_MPU6050::get_delta_time()
{

}

bool AP_InertialSensor_MPU6050::wait_for_sample(uint16_t timeout_ms)
{
	/* TODO Implement */
	_poll();
	return true;
}

uint16_t AP_InertialSensor_MPU6050::_init_sensor(Sample_rate sample_rate)
{

	hal.scheduler->suspend_timer_procs();

	// Try boot IMU for 5 times
	for (int i=0;i<5;i++) {
		bool success = _reset(); 

		if (success) {
			/* */
			break;
		} else {
			if (i==4) hal.scheduler->panic(PSTR("PANIC: Failed to boot MPU6050 5 times\n"));
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
	// TODO Are semaphore/mutex required for I2C
	
	// Buffer to store high and low bits of x/y/z acceleration
	static uint8_t _accel_raw[6];

	// Read a contiguous block of 6 bytes
	uint8_t status = hal.i2c->readRegisters(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 6, _accel_raw);
	if (status) {
		printf("Bad I2C read - poll\n");
		return;
	}


	// Combine high and low bytes
	int16_t ax = (_accel_raw[0] << 8) | _accel_raw[1];
	int16_t ay = (_accel_raw[2] << 8) | _accel_raw[3];
	int16_t az = (_accel_raw[4] << 8) | _accel_raw[5];

	// Convert into actual acceleration values
	_accel[0][0] = ax / 16384.0f;
	_accel[0][1] = ay / 16384.0f;
	_accel[0][2] = az / 16384.0f;

	return;
}

bool AP_InertialSensor_MPU6050::_reset(void)
{
	uint8_t status = hal.i2c->writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);

	// Return on a non-zero status code
	if (status) return false;

	return true;
}


