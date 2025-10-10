
#include "AP_InertialSensor_MPU6050.h"

#include <AP_Math.h>

// MPU6050 Register Definitions
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43


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

bool AP_InertialSensor_MPU6050::get_gyro_health(void) const
{
	// TODO Implement
	return true;
}
		
bool AP_InertialSensor_MPU6050::get_accel_health(void) const
{
	// TODO Implement
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
	hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(AP_InertialSensor_MPU6050, &AP_InertialSensor_MPU6050::_poll));

	// Run a single _poll during initialisation to get a valid reading
	_poll();

	// Technically supposed to return the product ID of the IMU
	return 0;
}

void AP_InertialSensor_MPU6050::_poll(void)
{
	// TODO I think _poll can just store the int16_t reading and we can convert/scale/transform during ::update
	// TODO Are semaphore/mutex required for I2C
	
	/* 1. Accelerometer read and convert */

	// Buffer to store high and low bits of x/y/z acceleration
	static uint8_t _read_raw[6];

	// Read a contiguous block of 6 bytes
	uint8_t status = hal.i2c->readRegisters(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 6, _read_raw);
	if (status) {
		//printf("MPU6050 bad I2C read\n");
		return;
	}

	// Combine high and low bytes
	int16_t ax = (_read_raw[0] << 8) | _read_raw[1];
	int16_t ay = (_read_raw[2] << 8) | _read_raw[3];
	int16_t az = (_read_raw[4] << 8) | _read_raw[5];

	// Convert into actual acceleration values
	_accel[0] = ax / 16384.0f;
	_accel[1] = ay / 16384.0f;
	_accel[2] = az / 16384.0f;

	// Convert from 'g' into m/s2
	_accel[0] *= GRAVITY_MSS;
	_accel[1] *= GRAVITY_MSS;
	_accel[2] *= GRAVITY_MSS;

	/* 2. Gyrometer read and convert */

	status = hal.i2c->readRegisters(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 6, _read_raw);
	if (status) {
		//printf("MPU6050 bad I2C read\n");
		return;
	}

	// Combine high and low bytes
	int16_t gx = (_read_raw[0] << 8) | _read_raw[1];
	int16_t gy = (_read_raw[2] << 8) | _read_raw[3];
	int16_t gz = (_read_raw[4] << 8) | _read_raw[5];

	// Convert into actual acceleration values
	_gyro[0] = gx / 131.0f;
	_gyro[1] = gy / 131.0f;
	_gyro[2] = gz / 131.0f;

	// TODO Scale and conversion to rad/s should be combined
	_gyro[0] *= DEG_TO_RAD;
	_gyro[1] *= DEG_TO_RAD;
	_gyro[2] *= DEG_TO_RAD;

	return;
}

bool AP_InertialSensor_MPU6050::_reset(void)
{
	uint8_t status = hal.i2c->writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);

	// Return on a non-zero status code
	if (status) return false;

	return true;
}

