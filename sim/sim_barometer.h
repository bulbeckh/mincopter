/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_SIM_H__
#define __AP_BARO_SIM_H__

#include <AP_HAL.h>
#include "AP_Baro.h"

/* Barometer measurements are received from the simulation in units of Pascals (Pa).
 *
 * **AP_Baro Interface**
 *
 * Used to track is barometer measurements can be trusted
 * `bool healthy`
 *
 * How many samples have been taken during calls to barometer.accumulate
 * `uint8_t _pressure_samples`
 *
 * Time (in ms) since last call to ::read
 * `uint32_t last_update`
 *
 * Set during calibration routines to determine a reference pressure and temperature reading
 * `float _ground_temperature`
 * `float _ground_pressure`
 *
 * Not used TODO remove
 * `int8_t _alt_offset`
 *
 * The current altitude in **meters**, calculated using a conversion from atmospheric pressure
 * `float _altitude`
 *
 * The time (in ms) that the pressure and temperature readings were used to calculate altitude
 * `uint32_t _last_altitude_t`
 *
 * Not used TODO remove
 * `float _last_altitude_EAS2TAS`
 * `float _EAS2TAS`
 *
 * Used to calculate the climb rate (by filtering _altitude) in m/s
 * `DerivativeFilterFloat_Size7 _climb_rate_filter`
 *
 */

class AP_Baro_Sim : public AP_Baro
{
public:
    AP_Baro_Sim() : AP_Baro()
    {
    }

	/* @brief Initialise the simulated barometer. Called during setup/initialisation */
    bool init() override;

	/* @brief Read barometer sensors. Called at 10Hz */
    uint8_t read() override;

	/* @brief Return pressure in pascals */
    float get_pressure() override;
								   
	/* @brief Return temperature in degrees celsius */
    float get_temperature() override;

private:

	/* @brief temperature reading in degrees celsius */
    float temperature_degc;

	/* @brief pressure reading in pascals (101,325 Pa is approximately sea-level) */
    float pressure_pa;

};

#endif
