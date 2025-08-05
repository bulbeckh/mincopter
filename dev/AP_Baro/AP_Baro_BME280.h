

#pragma once

#include "AP_Baro.h"
#include <AverageFilter.h>

class AP_Baro_BME280 : public AP_Baro
{
public:
    AP_Baro_BME280() {
    };

    /* AP_Baro public interface: */
    bool            init();

    uint8_t         read();

    void 			accumulate(void);

    float           get_pressure();

    float           get_temperature();

private:

	/* @brief Used by pigpiod */
	int16_t _pi_ref;
	int16_t _handle;

	//float Press;
	//float Temp;


	/* @brief Compensation variables */
	struct {
		int16_t dig_T1;
		int16_t dig_T2;
		int16_t dig_T3;

		int16_t dig_P1;
		int16_t dig_P2;
		int16_t dig_P3;
		int16_t dig_P4;
		int16_t dig_P5;
		int16_t dig_P6;
		int16_t dig_P7;
		int16_t dig_P8;
		int16_t dig_P9;
	} _dig;

	/* @brief Temperature compensation function for BME280 */
	int32_t BME280_compensate_T_int32(int32_t adc_T);

	/* @brief Pressure compensation function for BME280 */
	uint32_t BME280_compensate_P_int32(int32_t adc_P);

	/* @brief Placeholder parameter */
	int32_t t_fine;

	/* @brief Set the oversample values for pressure and temperature
	 * @param pressure_oversample The oversample reading for pressure measurement
	 * @param temperature_oversample The oversample reading for temperature measurement */
	uint8_t set_bme280_oversample(uint8_t pressure_oversample, uint8_t temperature_oversample);

	/* @brief Change the mode from sleep to another mode
	 * @param mode The mode to transition to */
	uint8_t set_bme280_mode(uint8_t mode);

	/* @brief Retrieve the compensation parameters from the BME280 EEPROM */
	uint8_t init_compensation_parameters(void);
};


