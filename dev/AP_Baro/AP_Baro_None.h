
#pragma once

class AP_Baro_None : public AP_Baro
{
	public:

		AP_Baro_None(void) { }

		/* @brief Initialisation method of barometer */
		bool init(void) override { return true; }

		/* @brief Execute a barometer read and update fo temperature and pressure */
		uint8_t read(void) override { return 0; }

		/* @brief Get pressure in Pascal (Pa). Divide by 100 for millibars or hectopascals */
		float get_pressure(void) override { return 100000.0f; }

		/* @brief Get temperature in degrees celsius */
		float get_temperature(void) override { return 30.0f; }

};

