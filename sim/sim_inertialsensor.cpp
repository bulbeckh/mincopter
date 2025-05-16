/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#include "sim_inertialsensor.h"

extern const AP_HAL::HAL& hal;

// Needed because _gyro_scale is defined static in class method
const float AP_InertialSensor_Sim::_gyro_scale=1.0;

AP_InertialSensor_Sim::AP_InertialSensor_Sim() : 
	AP_InertialSensor(),
    _drdy_pin(NULL),
    _initialised(false)
{
}

float AP_InertialSensor_Sim::get_delta_time()
{
	// TODO This hardcoded method 
	// Flight loop runs at 100hz and call is updated
	return 0.01;
}

uint16_t AP_InertialSensor_Sim::_init_sensor( Sample_rate sample_rate )
{
    if (_initialised) return AP_PRODUCT_ID_NONE;
    _initialised = true;

    return AP_PRODUCT_ID_NONE;
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_Sim::wait_for_sample(uint16_t timeout_ms)
{
    return true;
}

float AP_InertialSensor_Sim::get_gyro_drift_rate()
{
		return 0.0;
}

bool AP_InertialSensor_Sim::update( void )
{
    /* TODO Do nothing on call to update for simulated IMU. The updates to the _accel
	 * and _gyro vectors are done via the gz_interface.
     */
    
    // Set values for _gyro and _accel

    //_previous_accel[0] = _accel[0];

    //_gyro[0]  = Vector3f(_gyro_sum.x, _gyro_sum.y, _gyro_sum.z);
    //_accel[0] = Vector3f(_accel_sum.x, _accel_sum.y, _accel_sum.z);

    /* NOTE TODO Double to float conversion here. Is this casting done automatically? */
	/*
    _gyro[0] = Vector3f(gz_interface.sensor_states.imu_gyro_x,
			gz_interface.sensor_states.imu_gyro_y,
			gz_interface.sensor_states.imu_gyro_z);

    _accel[0] = Vector3f(gz_interface.sensor_states.imu_accel_x,
			gz_interface.sensor_states.imu_accel_y,
			gz_interface.sensor_states.imu_accel_z);
	*/

    /* NOTE I don't think this is needed for simulated environment */

    //_num_samples = _sum_count;
	/*
    _num_samples = 1;

    _accel_sum.zero();
    _gyro_sum.zero();
    _sum_count = 0;

    _gyro[0].rotate(_board_orientation);
    _gyro[0] *= _gyro_scale / _num_samples;
    _gyro[0] -= _gyro_offset[0];

    _accel[0].rotate(_board_orientation);
    _accel[0] *= 1 / _num_samples;

    Vector3f accel_scale = _accel_scale[0];
    _accel[0].x *= accel_scale.x;
    _accel[0].y *= accel_scale.y;
    _accel[0].z *= accel_scale.z;
    _accel[0] -= _accel_offset[0];
	*/

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

