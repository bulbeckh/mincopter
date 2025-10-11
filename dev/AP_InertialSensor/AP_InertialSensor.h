/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_H__
#define __AP_INERTIAL_SENSOR_H__

// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f

/**
   maximum number of INS instances available on this platform. If more
   than 1 then redundent sensors may be available
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#define INS_MAX_INSTANCES 2
#else
#define INS_MAX_INSTANCES 1
#endif

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
// HASH include "AP_InertialSensor_UserInteract.h"

/* AP_InertialSensor is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 *
 * Gauss-Newton accel calibration routines borrowed from Rolfe Schmidt
 * blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
 * original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 */
class AP_InertialSensor
{
	public:
		AP_InertialSensor();

		// TODO Change name
		enum Start_style {
			COLD_START = 0,
			WARM_START
		};

		/* @brief The rate that updates will be available to the application */
		// TODO Change name
		enum Sample_rate {
			RATE_50HZ,
			RATE_100HZ,
			RATE_200HZ
		};

		/* @brief Perform startup initialisation. For COLD_START, implementations using real sensors can
		 * assume that the airframe is stationary and nominally oriented.
		 * For WARM_START, no assumptions should be made about the orientation or motion of the airframe.
		 * Calibration should be as for the previous COLD_START call.
		 * @param style	The initialisation startup style. */
		virtual void init(Start_style style, Sample_rate sample_rate);

		/* @brief Perform cold startup initialisation for just the accelerometers. */
		virtual void init_accel(void);

		/* @brief Returns true if the accelerometers have been calibrated */
		bool calibrated(void);

		/* @brief Perform cold-start initialisation for just the gyros. */
		virtual void init_gyro(void);

		/* @brief Get the current gyro values in rad/s */
		const Vector3f &get_gyro(void) const {
			return _gyro;
		}

		/* @brief Get gyro offsets in rad/s */
		const Vector3f &get_gyro_offsets(void) const {
			return _gyro_offset;
		}

		/* @brief Get the current accelerometer values in m/s2 */
		const Vector3f &get_accel(void) const {
			return _accel;
		}

		/* @brief Returns true if latest gyro readings are valid */
		virtual bool get_gyro_health(void) const = 0;
		
		/* @brief Returns true if latest accel readings are valid */
		virtual bool get_accel_health(void) const = 0;

		/* @brief Get accel offsets in m/s2 */
		const Vector3f &get_accel_offsets(void) const {
			return _accel_offset;
		}

		/* @brief Get accel scale in units xx */
		const Vector3f &get_accel_scale(void) const {
			return _accel_scale;
		}

		/* @brief Update the sensor data. Returns true if data was updated */
		virtual bool update(void) = 0;

		/* @brief Returns the time period in seconds overwhich the sensor data was collected */
		virtual float get_delta_time(void) = 0;

		/* @brief Returns the maximum gyro drift rate in rad/s/s */
		virtual float get_gyro_drift_rate(void) = 0;

		/* @brief Wait for a sample to be available, with timeout in milliseconds */
		virtual bool wait_for_sample(uint16_t timeout_ms) = 0;

		/* @brief Set overall board orientation */
		void set_board_orientation(enum Rotation orientation) {
			_board_orientation = orientation;
		}

	protected:
		/* @brief Sensor specific init to be overwritten by descendant classes */
		virtual uint16_t _init_sensor(Sample_rate sample_rate) = 0;
		virtual void _init_accel(void);
		virtual void _init_gyro(void);

#if !defined( __AVR_ATmega1280__ )
		// Calibration routines borrowed from Rolfe Schmidt
		// blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
		// original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde

		// _calibrate_accel - perform low level accel calibration
		virtual bool _calibrate_accel(Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale);
		virtual void _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
		virtual void _calibrate_reset_matrices(float dS[6], float JS[6][6]);
		virtual void _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
		virtual void _calculate_trim(Vector3f accel_sample, float& trim_roll, float& trim_pitch);
#endif

		/* @brief Most recent accelerometer reading obtained by ::update */
		Vector3f _accel;

		/* @brief Previous accelerometer reading obtained by ::update */
		Vector3f _previous_accel;

		/* @brief Most recent gyro reading obtained by ::update */
		Vector3f _gyro;

		// TODO Remove
		// product id
		int16_t _product_id;

		/* @brief Accelerometer scaling and offsets */
		Vector3f _accel_scale;
		Vector3f _accel_offset;
		Vector3f _gyro_offset;

		/* @brief Board orientation from AHRS */
		enum Rotation _board_orientation;
};

// HASH include "AP_InertialSensor_Oilpan.h"
#include "AP_InertialSensor_MPU6000.h"
#include "AP_InertialSensor_MPU6050.h"
// HASH include "AP_InertialSensor_HIL.h"
// HASH include "AP_InertialSensor_PX4.h"
// HASH include "AP_InertialSensor_UserInteract_Stream.h"
// HASH include "AP_InertialSensor_Flymaple.h"
#include "AP_InertialSensor_L3G4200D.h"
#include "AP_InertialSensor_None.h"

#endif // __AP_INERTIAL_SENSOR_H__
