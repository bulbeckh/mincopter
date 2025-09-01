
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math.h>

#include "AP_InertialSensor.h"


/* Communication with MPU6050 over I2C uses the SDL and SCK pins of the HAL device.
 *
 * The following HAL interface functions are typically used.
 *
 * uint8_t read(uint8_t addr, uint8_t len, uint8_t* data)
 * uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
 * uint8_t readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
 *
 * I2C initialisation should be handled by the HAL initialisation so we should just be
 * able to read/write immediately.
 *
 * The ::update method should read and populate both the _accel and _gyro vectors with
 * correct measurements in the ENU frame. (TODO Are we using NED or ENU?)
 *
 * The gyrometer measurements should be in **rad/s** and the accelerometer measurements
 * in m/s^2
 *
 */


#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B

class AP_InertialSensor_MPU6050 : public AP_InertialSensor
{ 

	public:

		/* @brief Constructor */
		AP_InertialSensor_MPU6050(void);

		// Override Methods
		
		/* @brief Update sensor */
		bool update(void) override;

		/* @brief Returns gyrometer drift rates (in rad/s) */
		float get_gyro_drift_rate(void) override;

		/* @brief Returns time (in seconds) that the last data read represents */
		float get_delta_time(void) override;

		/* @brief Wait for a sample to be available
		 * @param timeout_ms The period of time (in ms) to wait for before timing out */
		bool wait_for_sample(uint16_t timeout_ms) override;

		/* @brief Sensor specific initialisation routine. Called during ::init */
		uint16_t _init_sensor(Sample_rate sample_rate) override;

		/* @brief Returns true if latest gyro readings are valid */
		bool get_gyro_health(void) const override;
		
		/* @brief Returns true if latest accel readings are valid */
		bool get_accel_health(void) const override;

	private:

		/* @brief Poll MPU6050 for new data */
		void _poll(void);

		/* @brief Resets the MPU6050 by writing to power mgmt register */
		bool _reset(void);


};

