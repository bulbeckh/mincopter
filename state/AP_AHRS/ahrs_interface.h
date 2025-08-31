/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_AHRS_H__
#define __AP_AHRS_H__
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  AHRS (Attitude Heading Reference System) interface for ArduPilot
 *
 */

#include <AP_Math.h>
#include <inttypes.h>

#include "mcstate_state.h"

/* This class is the base class for any AHRS implementation. The AHRS should typically
 * only be used by the MCState class */

// TODO Remove AP_ from here - this interface has been significantly modified
class AP_AHRS
{
	public:
		// Constructor
		AP_AHRS(void) { }

		/* @brief Initialisation of the AHRS */
		void ahrs_init(MCStateData* state) {
			_state = state;
			_ahrs_init_internal();
			return;
		}

		/* @brief Update method for AHRS. Called by MCState during MCState::update */
		virtual void ahrs_update(void) = 0;

		/* @brief Reset the current attitude representation to zero */
		virtual void ahrs_reset(void) {
			_state->_attitude(0.0f, 0.0f, 0.0f, 0.0f);
			return;
		}

		/* @brief Reset the current attitude representation to the provided roll, pitch, and yaw */
		virtual void ahrs_reset_attitude(const float &roll, const float &pitch, const float &yaw) {
			_state->_attitude.from_euler(roll, pitch, yaw);
			return;
		}

	private:
		/* @brief Pointer to the state object to be updated on each call to ahrs_update */
		MCStateData* _state;

		/* @brief Class specific initialisation method */
		virtual void _ahrs_init_internal(void) = 0;

		// TODO Are these actually used? Remove if not used or not sufficiently general for a state class
		// how often our attitude representation has gone out of range
		uint8_t renorm_range_count;

		// how often our attitude representation has blown up completely
		uint8_t renorm_blowup_count;

};


#endif // __AP_AHRS_H__
