
#ifndef AP_COMPASS_SIM_H
#define AP_COMPASS_SIM_H

#include <AP_HAL.h>

#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"

/* **Compass Interface**
 *
 * Time (in us) since last call to ::read
 * `uint32_t last_update`
 *
 * Enables calibration learning
 * `int8_t _learn`
 *
 * Identifies if compass reading can be trusted
 * `bool _healthy[COMPASS_MAX_INSTANCES]`
 *
 * 3-axis compass readings TODO Confirm that units here are Tesla or Gauss
 * `Vector3f _field[COMPASS_MAX_INSTANCES]
 *
 * Orientation of the compass relative to the board. Default is NED
 * `int8_t _orientation`
 *
 * Orientation from AHRS
 * `enum Rotation _board_orientation
 *
 * Compass offsets to compensate for metal in frame
 * `Vector3f _offset[COMPASS_MAX_INSTANCES]`
 *
 * Compass declination (in radians)
 * `float _declination`
 *
 * Flag for whether to use compass readings for yaw calculations
 * `int8_t _use_for_yaw`
 *
 * Enables automatic declination code
 * `int8_t _auto_declination`
 *
 * Flag for whether an external compass is used
 * `int8_t _external`
 *
 * Sentinel for first call to null_offsets
 * `bool _null_init_done
 *
 * Used in offset correction and calculation
 * `static const uint8_t _mag_history_size = 20;
 * `uint8_t _mag_history_index[COMPASS_MAX_INSTANCES];
 * `Vector3i _mag_history[COMPASS_MAX_INSTANCES][_mag_history_size];
 *
 * Compensates field readings for magnetic field change during large throttles (or high motor currents)
 * `int8_t _motor_comp_type;               // 0 = disabled, 1 = enabled for throttle, 2 = enabled for current
 * `Vector3f _motor_compensation[COMPASS_MAX_INSTANCES]; // factors multiplied by throttle and added to compass outputs
 * `Vector3f _motor_offset[COMPASS_MAX_INSTANCES]; // latest compensation added to compass
 * `float _thr_or_curr;                   // throttle expressed as a percentage from 0 ~ 1.0 or current expressed in amps
 *
 */

class AP_Compass_Sim : public Compass
{

public:
    AP_Compass_Sim() :
		Compass(),
		acc_samples(0)
	{
    }
	
	/* @brief Compass Initialisation */
    bool init(void) override;

	/* @brief Update field measurements. Called at 10hz */
    bool read(void) override;

	/* @brief Accumulation readings from compass. Called at 50Hz */
    void accumulate(void) override;

private:
	
	Vector3f acc_field;
	uint16_t acc_samples;

};
#endif
