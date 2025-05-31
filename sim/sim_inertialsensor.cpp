/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#include "sim_inertialsensor.h"
#include "AP_Math.h"

#include "gz_interface.h"
extern GZ_Interface gz_interface;

#include "simulation_logger.h"
extern SimulationLogger simlog;

extern const AP_HAL::HAL& hal;

AP_InertialSensor_Sim::AP_InertialSensor_Sim() : 
	AP_InertialSensor()
{
}

float AP_InertialSensor_Sim::get_delta_time()
{
	// TODO This hardcoded method 
	// Flight loop runs at 100hz and call is updated
	//return 0.01;
	
    return _delta_time_usec * 1.0e-6;
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
    uint32_t now = hal.scheduler->millis();
    _delta_time_usec = (now - _last_update_ms) * 1000;
    _last_update_ms = now;

	gz_interface.get_imu_gyro_readings(_gyro[0]);
	gz_interface.get_imu_accel_readings(_accel[0]);

	simlog.write_imu_state(_gyro[0], _accel[0]);

    return true;
}


uint16_t AP_InertialSensor_Sim::_init_sensor( Sample_rate sample_rate )
{
	set_board_orientation(Rotation::ROTATION_NONE);
	// NOTE This is meant to return the product_id of the IMU sensor
	return AP_PRODUCT_ID_NONE;
}

