/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_H__
#define __AP_BARO_H__

#include <stdint.h>

class AP_Baro
{
	public:

		AP_Baro() :
			_ground_pressure(0),
			_ground_temperature(29) // Initialise ground temperature at 29 degrees celsius
		{ }

		/* @brief Validity of current barometer readings */
		bool healthy;

		/* @brief Initialisation method of barometer */
		virtual bool init(void) = 0;

		/* @brief Execute a barometer read and update fo temperature and pressure */
		virtual uint8_t read(void) = 0;

		/* @brief Get pressure in Pascal (Pa). Divide by 100 for millibars or hectopascals */
		virtual float get_pressure(void) = 0;

		/* @brief Get temperature in degrees celsius */
		virtual float get_temperature(void) = 0;

		/* @brief Accumulate a reading - overridden in some drivers */
		virtual void accumulate(void);

		/* @brief Calibrate the barometer. This must be called on startup if the
		 * altitude/climb_rate/acceleration interfaces are ever used the callback
		 * is a delay() like routine */
		void calibrate(void);

		/* Update the barometer calibration to the current pressure. Can be used for incremental
		 * preflight update of baro */
		void update_calibration(void);

		/* @brief Returns ground temperature in degrees C */
		float get_ground_temperature(void) {
			return _ground_temperature;
		}

		/* @brief Returns ground pressure in Pascal (Pa) */
		float get_ground_pressure(void) {
			return _ground_pressure;
		}

		/* @brief Returns last time sample was taken (in ms) */
		uint32_t get_last_update() {
			return _last_update;
		}

	protected:
		/* @brief Time of last barometer read (in ms) */
		uint32_t _last_update;

	private:

		/* @brief Temperature reading at ground (set during arming) */
		float _ground_temperature;

		/* @brief Pressure reading at ground (set during arming) */
		float _ground_pressure;
};

#include "AP_Baro_MS5611.h"
#include "AP_Baro_BMP085.h"
#include "AP_Baro_BME280.h"
#include "AP_Baro_None.h"

#endif // __AP_BARO_H__
