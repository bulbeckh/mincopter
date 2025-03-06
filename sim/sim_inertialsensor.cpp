/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor_Sim.h"

extern const AP_HAL::HAL& hal;


AP_InertialSensor_Sim::AP_InertialSensor_Sim() : 
	AP_InertialSensor(),
    _drdy_pin(NULL),
    _initialised(false),
    _mpu6000_product_id(AP_PRODUCT_ID_NONE)
{
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

bool AP_InertialSensor_Sim::update( void )
{
		// Set values for _gyro and _accel

    _previous_accel[0] = _accel[0];

    _gyro[0]  = Vector3f(_gyro_sum.x, _gyro_sum.y, _gyro_sum.z);
    _accel[0] = Vector3f(_accel_sum.x, _accel_sum.y, _accel_sum.z);
    _num_samples = _sum_count;
    _accel_sum.zero();
    _gyro_sum.zero();
    _sum_count = 0;

    _gyro[0].rotate(_board_orientation);
    _gyro[0] *= _gyro_scale / _num_samples;
    _gyro[0] -= _gyro_offset[0];

    _accel[0].rotate(_board_orientation);
    _accel[0] *= MPU6000_ACCEL_SCALE_1G / _num_samples;

    Vector3f accel_scale = _accel_scale[0].get();
    _accel[0].x *= accel_scale.x;
    _accel[0].y *= accel_scale.y;
    _accel[0].z *= accel_scale.z;
    _accel[0] -= _accel_offset[0];

    return true;
}

