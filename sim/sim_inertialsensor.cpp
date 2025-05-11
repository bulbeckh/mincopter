/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#include "sim_inertialsensor.h"

#include "gz_interface.h"
extern GZ_Interface gz_interface;

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
		return 0.0;
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

    /* TODO For updates to the simulated sensors, we retrieve values from the
     * gazebo simulation via GZ_Interface. This contains the most recently read
     * simulation state packet.
     */
    
    // Set values for _gyro and _accel

    _previous_accel[0] = _accel[0];

    //_gyro[0]  = Vector3f(_gyro_sum.x, _gyro_sum.y, _gyro_sum.z);
    //_accel[0] = Vector3f(_accel_sum.x, _accel_sum.y, _accel_sum.z);

    /* NOTE TODO Double to float conversion here. Is this casting done automatically? */
    _gyro[0] = Vector3f(gz_interface.sensor_states.imu_gyro_x,
			gz_interface.sensor_states.imu_gyro_y,
			gz_interface.sensor_states.imu_gyro_z);

    _accel[0] = Vector3f(gz_interface.sensor_states.imu_accel_x,
			gz_interface.sensor_states.imu_accel_y,
			gz_interface.sensor_states.imu_accel_z);

    /* NOTE I don't think this is needed for simulated environment */

    //_num_samples = _sum_count;
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

    return true;
}

