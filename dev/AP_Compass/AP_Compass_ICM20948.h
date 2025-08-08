
#pragma once

#include <Compass.h>
#include <AP_HAL.h>


class AP_Compass_ICM20948 : public Compass
{
	public:
		AP_Compass_ICM20948();

		/* Override Methods */
		bool init() override;

    	bool read(void) override;

    	void accumulate(void) override;

	private:
		/* @brief Reset device */
		uint8_t _icm_reset(void);

		/* @brief Initialise ICM for gyrometer and accelerometer measurements */
		uint8_t _icm_init_gyro_accel(void);

		/* @brief Initialise ICM for magnetometer measurements */
		uint8_t _icm_init_magnetometer(void);

		/* @brief Prepare the ICM for I2C Master mode and magnetometer as I2C slave */
		uint8_t _icm_prep_magnetometer_measure(void);

		/* @brief Retrieve and assert correct ICM device ID */
		uint8_t _icm_whoami(void);

		/* @brief Retrieve and assert correct magnetometer device ID */
		uint8_t _mgt_whoami(void);

		/* @brief Block until data is ready */
		uint8_t _poll_data_ready(void);

		// SPI Interface
		
		/* @brief Reference to SPIDriver */
		AP_HAL::SPIDeviceDriver* _spi;



};


