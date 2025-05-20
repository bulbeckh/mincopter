/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_SIM_INERTIAL_SENSOR_H__
#define __AP_SIM_INERTIAL_SENSOR_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>

#include "AP_InertialSensor.h"

/* **AP_InertialSensor** Interface
 *
 * Accelerometers values (in m/s^2)
 * `Vector3f _accel[INS_MAX_INSTANCES]`
 *
 * Accelerometer value from previous call to ::update
 * `Vector3f _previous_accel[INS_MAX_INSTANCES]`
 *
 * Gyrometer rates (in rad/s)
 * `Vector3f _gyro[INS_MAX_INSTANCES]`
 *
 * Accelerometer scaling factor calculated during calibrate routine
 * `Vector3f _accel_scale[INS_MAX_INSTANCES]`
 * `Vector3f _accel_offset[INS_MAX_INSTANCES]`
 * `Vector3f _gyro_offset[INS_MAX_INSTANCES]`
 *
 * Not used TODO Why is this at the interface level
 * `int8_t _mpu6000_filter`
 *
 * Orientation of the IMU
 * `enum Rotation _board_orientation`
 *
 */

class AP_InertialSensor_Sim : public AP_InertialSensor
{
public:

    AP_InertialSensor_Sim();

	/* @brief IMU update method. Called at 100Hz */
    bool update() override;

	/* @brief Returns gyro drift rate. Have used the MPU6000 gyro drift rate for reference */
    float get_gyro_drift_rate() override;

	/* @brief Waits for a sample from the IMU */
    bool wait_for_sample(uint16_t timeout_ms) override;
	
	/* @brief Return time in seconds since last call to update. NOTE This has been hardcoded for simulation */
    float get_delta_time() override;

	// The following methods are also virtualised */
	
	/* @brief Retrieve number of errors in INS measurement */
    uint16_t error_count(void) const { return 0; }

	/* @brief Check if IMU readings are accurate */
    bool healthy(void) const { return true; }

	/* @brief Get Gyro health. Always true for simulation */
    bool get_gyro_health(uint8_t instance) const { return true; }

	/* @brief Get accel health. Always true for simulation */
    bool get_accel_health(uint8_t instance) const { return true; }

	/* @brief Initialise sensor. Does nothing for simulated IMU */
	uint16_t _init_sensor( Sample_rate sample_rate ) override;

};

#endif


