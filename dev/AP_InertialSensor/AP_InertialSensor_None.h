
#pragma once

class AP_InertialSensor_None : public AP_InertialSensor
{
	public:
	AP_InertialSensor_None(void) { }

		/* @brief Returns true if latest gyro readings are valid */
		bool get_gyro_health(void) const override { return true; }
		
		/* @brief Returns true if latest accel readings are valid */
		bool get_accel_health(void) const override { return true; }

		/* @brief Update the sensor data. Returns true if data was updated */
		bool update(void) override { return true; }

		/* @brief Returns the time period in seconds overwhich the sensor data was collected */
		float get_delta_time(void) override { return 0.01f; }

		/* @brief Returns the maximum gyro drift rate in rad/s/s */
		float get_gyro_drift_rate(void) override { return 0.0f; }

		/* @brief Wait for a sample to be available, with timeout in milliseconds */
		bool wait_for_sample(uint16_t timeout_ms) override { return true; }

		/* @brief Sensor specific init to be overwritten by descendant classes */
		uint16_t _init_sensor(Sample_rate sample_rate) override { return 0; }
};

