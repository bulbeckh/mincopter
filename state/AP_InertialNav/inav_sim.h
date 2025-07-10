
#pragma once

/* Interface for Inertial Nav */


#include <AP_Baro.h>
#include <AP_GPS_Glitch.h>              // GPS Glitch detection library
#include <AP_InertialSensor.h>          // ArduPilot Mega IMU Library
#include <AP_Math.h>

#include "inav_interface.h"

#include "ahrs.h"

class MC_InertialNav_Sim : public MC_InertialNav
{

	public:
    	MC_InertialNav_Sim(const MC_AHRS_CLASS *ahrs, AP_Baro* baro, GPS*& gps, GPS_Glitch& gps_glitch ) :
			MC_InertialNav()
		{
		}

		/* @brief Initialise the inertial navigation */
		void init();

		/* @brief Update the inertial navigation. Called via xx TODO */
		void update(float dt);

		/* @brief Checks if position reading is valid. Always true for simulation. */
		bool position_ok() const {
			return true;
		}

		/* @brief Returns latitude in deg*1e7  (*10,000,000) */
		int32_t get_latitude() const {
			return inav_lat;
		}

		/* @brief Returns longitude in deg*1e7  (*10,000,000) */
		int32_t get_longitude() const {
			return inav_lng;
		}

		/* @brief Returns altitude in cm. NOTE Even though the earth frame is NED, this will return a positive altitude */
		float get_altitude() const {
			return inav_alt;
		}

		/* @brief Set altitude of inertial nav.
		 * @param new_alt float New altitude in cm */
		void set_altitude(float new_alt);

		/* @brief Get position in earth frame (NED) */
		const Vector3f get_position() const {
			return inav_pos;
		}

		/* @brief Get velocity in earth frame (NED) */
		const Vector3f get_velocity() const {
			return inav_vel;
		}

		/* @brief Get z-component of velocity in earth frame (NED) which is the vertical climb rate */
		float get_velocity_z() const {
			return inav_vel.z;
		}

		/* @brief Set home position via latitude and longitude (in deg*1e7)
		 * @param lat int32t Latitude in degrees*1e7
		 * @param lng int32t Longitude in degrees*1e7 */
		void set_home_position(int32_t lat, int32_t lng);

};


