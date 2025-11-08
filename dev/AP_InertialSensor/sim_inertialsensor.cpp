/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#include "sim_inertialsensor.h"
#include "AP_Math.h"

extern const AP_HAL::HAL& hal;

AP_InertialSensor_Sim::AP_InertialSensor_Sim() : 
	AP_InertialSensor()
{
}

float AP_InertialSensor_Sim::get_delta_time(void)
{
	// TODO Not sure if we should change this - flight loop runs at 100hz
	
	return 0.01;
    //return _delta_time_usec * 1.0e-6;
}

bool AP_InertialSensor_Sim::wait_for_sample(uint16_t /* unused */)
{
    return true;
}

float AP_InertialSensor_Sim::get_gyro_drift_rate(void)
{
	// NOTE This drift rate was taken from the MPU6000 IMU. Not sure what it would be for simulation
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

bool AP_InertialSensor_Sim::update( void )
{
	// TODO Change this to simulation timestamp
    uint32_t now = hal.scheduler->millis();
    _delta_time_usec = (now - _last_update_ms) * 1000;
    _last_update_ms = now;

	// Retrieve gyro readings in rad/s from Gazebo simulation
	// TODO The imu_gyro_* objects are of type double so this is implicitly downcasting
	Vector3f simulation_gyro_reading(
			hal.sim->last_sensor_state.imu_gyro_x,
			hal.sim->last_sensor_state.imu_gyro_y,
			hal.sim->last_sensor_state.imu_gyro_z
			);

	// Apply rotations
	// NOTE See docs for discussion on gazebo reference frames used for Compass and IMU
	/*
	simulation_gyro_reading.y *= -1;
	simulation_gyro_reading.z *= -1;
	*/

	_gyro = simulation_gyro_reading;

	Vector3f simulation_accel_reading(
			hal.sim->last_sensor_state.imu_accel_x,
			hal.sim->last_sensor_state.imu_accel_y,
			hal.sim->last_sensor_state.imu_accel_z
			);

	// Apply rotations
	/*
	simulation_accel_reading.y *= -1;
	simulation_accel_reading.z *= -1;
	*/

	_accel = simulation_accel_reading;

    return true;
}


uint16_t AP_InertialSensor_Sim::_init_sensor( Sample_rate /* unused */ )
{
	set_board_orientation(Rotation::ROTATION_NONE);
	// NOTE This is meant to return the product_id of the IMU sensor
	
	// NOTE TODO The _init_accel and _init_gyro methods are exposed as virtual functions for the interface but are never actually called?? Only this method is called during init

	// We assume we are station for the accelerometer and gyrometer
	_accel = Vector3f(0.0f, 0.0f,-9.81f);
	_gyro = Vector3f(0.0f, 0.0f, 0.0f);

	return AP_PRODUCT_ID_NONE;
}



