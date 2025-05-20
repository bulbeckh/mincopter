/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#include "sim_inertialsensor.h"

extern const AP_HAL::HAL& hal;

AP_InertialSensor_Sim::AP_InertialSensor_Sim() : 
	AP_InertialSensor()
{
}

float AP_InertialSensor_Sim::get_delta_time()
{
	// TODO This hardcoded method 
	// Flight loop runs at 100hz and call is updated
	return 0.01;
}

bool AP_InertialSensor_Sim::wait_for_sample(uint16_t timeout_ms)
{
    return true;
}

float AP_InertialSensor_Sim::get_gyro_drift_rate()
{
	// NOTE This drift rate was taken from the MPU6000 IMU. Not sure what it would be for simulation
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

bool AP_InertialSensor_Sim::update( void )
{
    return true;
}

void AP_InertialSensor_Sim::set_imu_gyros(double imu_gyro_x, double imu_gyro_y, double imu_gyro_z)
{
	_gyro[0].x = imu_gyro_x;
	_gyro[0].y = imu_gyro_y;
	_gyro[0].z = imu_gyro_z;

	return;
}


void AP_InertialSensor_Sim::set_imu_accel(double imu_accel_x, double imu_accel_y, double imu_accel_z)
{
	_accel[0].x = imu_accel_x;
	_accel[0].y = imu_accel_y;
	_accel[0].z = imu_accel_z;

	return;
}

uint16_t AP_InertialSensor_Sim::_init_sensor( Sample_rate sample_rate )
{
	// NOTE This is meant to return the product_id of the IMU sensor
	return AP_PRODUCT_ID_NONE;
}

