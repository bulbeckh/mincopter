/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef Compass_h
#define Compass_h

#include <inttypes.h>
#include <AP_Common.h>

#include <AP_Math.h>
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library

// compass product id
#define AP_COMPASS_TYPE_UNKNOWN  0x00
#define AP_COMPASS_TYPE_HMC5843  0x02
#define AP_COMPASS_TYPE_HMC5883L 0x03

// motor compensation types (for use with motor_comp_enabled)
#define AP_COMPASS_MOT_COMP_DISABLED    0x00
#define AP_COMPASS_MOT_COMP_THROTTLE    0x01
#define AP_COMPASS_MOT_COMP_CURRENT     0x02

// TODO Make this a configurable parameter somewhere
// setup default mag orientation for each board type
# define MAG_BOARD_ORIENTATION ROTATION_NONE

class Compass
{
	public:
		Compass();

		/* @brief Time since last valid reading of _field (in microseconds, us) */
		uint32_t last_update;

		/* @brief Initialize the compass device */
		virtual bool init();

		/* @brief Read the compass and update the mag_ variables */
		virtual bool read(void) = 0;

		/* @brief Use spare CPU cycles to accumulate values from the compass */
		virtual void accumulate(void) = 0;

		/* @brief Get magnetic field in microteslas (uT) in the ENU frame */
		const Vector3f &get_field(void) const {
			return _field;
		}

		/* @brief Return the health of a compass */
		bool healthy(void) const {
			return _healthy;
		}

		/* @brief Returns the current offset values in microteslas (uT) */
		const Vector3f &get_offsets(void) const {
			return _offset;
		}

		/* @brief Sets the initial location used to get declination
		 * @param latitude GPS Latitude
		 * @param longitude GPS Longitude */
		void set_initial_location(int32_t latitude, int32_t longitude);

		/* @brief Return true if the compass should be used for yaw calculations */
		bool use_for_yaw(void) const {
			return _healthy && _use_for_yaw;
		}

		/* @brief Sets the local magnetic field declination
		 * @param radians Local field declination */
		void set_declination(float radians);

		/* @brief Get current magnetic field declination we are using */
		float get_declination() const;

		/* @brief Set overall board orientation */
		void set_board_orientation(enum Rotation orientation) {
			_board_orientation = orientation;
		}

		/* @brief Set the motor compensation type
		 * @param comp_type 0 = disabled, 1 = enabled use throttle, 2 = enabled use current */
		// TODO Change the function signature to set_motor_compensation_type
		void motor_compensation_type(const uint8_t comp_type) {
			if (_motor_comp_type <= AP_COMPASS_MOT_COMP_CURRENT && _motor_comp_type != (int8_t)comp_type) {
				_motor_comp_type = (int8_t)comp_type;
				_thr_or_curr = 0;                               // set current current or throttle to zero
				set_motor_compensation(Vector3f(0,0,0));        // clear out invalid compensation vector
			}
		}

		// TODO Change to enum class
		// TODO Change function signature to get_motor_compensation_type
		/* @brief Get the motor compensation value */
		uint8_t motor_compensation_type() const {
			return _motor_comp_type;
		}

		/* @brief Set the motor compensation factor x/y/z values.
		 * @param offsets Offsets multiplied by the throttle value and added to the raw mag_ values */
		void set_motor_compensation(const Vector3f &motor_comp_factor);

		/* @brief Get motor compensation factors as a vector */
		const Vector3f& get_motor_compensation(void) const {
			return _motor_compensation;
		}

		/* @brief Returns the current motor compensation offset values */
		const Vector3f &get_motor_offsets(void) const {
			return _motor_offset;
		}

		/* @brief Set the throttle as a percentage from 0.0 to 1.0
		 * @param thr_pct throttle expressed as a percentage from 0 to 1.0 */
		void set_throttle(float thr_pct) {
			if(_motor_comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
				_thr_or_curr = thr_pct;
			}
		}

		/* @brief Set the current used by system in amps
		 * @param amps current flowing to the motors expressed in amps */
		void set_current(float amps) {
			if(_motor_comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
				_thr_or_curr = amps;
			}
		}

	protected:
		/* @brief Whether current compass reading is valid */
		bool _healthy;

		/* @brief Compass reading in microteslas (uT) in the ENU frame */
		Vector3f _field;

		/* @brief Current orientation of the magnetometer (used for rotating to ENU frame */
		int8_t _orientation;

		/* @brief Compass offset in microteslas (uT) in the ENU frame */
		Vector3f _offset;

		// TODO Do we need inclination too?
		/* @brief Current magnetic declination value in degrees */
		float _declination;

		/* @brief Whether we are using this compass for yaw calculations */
		int8_t _use_for_yaw;

		///< used by offset correction
		static const uint8_t _mag_history_size = 20;
		uint8_t _mag_history_index;
		Vector3i _mag_history[_mag_history_size];

		// motor compensation
	
		// TODO Change to enum class
		/* @brief Motor compensation type (0 = disabled, 1 = enabled for throttle, 2 = enabled for current) */
		int8_t _motor_comp_type;

		/* @brief Factors multiplied by throttle and added to compass outputs */
		Vector3f _motor_compensation;

		/* @brief Latest compensation added to compass */
		Vector3f _motor_offset;

		/* @brief Either throttle expressed as a percentage from 0 ~ 1.0 or current expressed in amps */
		float _thr_or_curr;

		/* @brief board orientation from AHRS */
		enum Rotation _board_orientation;
};
#endif
