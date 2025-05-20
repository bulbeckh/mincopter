
#ifndef __AP_GPS_SIM_H__
#define __AP_GPS_SIM_H__

#include <AP_HAL.h>
#include <AP_Common.h>
#include "GPS.h"

/* **GPS Interface **
 *
 * GPS Time (milliseconds since start of GPS week)
 * `uint32_t time_week_ms`
 *
 * GPS Week Number
 * `uint16_t time_week`
 *
 * Latitude/Longitude in deg*10,000,000
 * `int32_t latitude`
 * `int32_t longitude`
 *
 * Altitude in cm
 * `int32_t altitude_cm`
 *
 * Ground speed in cm/sec
 * `uint32_t ground_speed_cm;           ///< ground speed in cm/sec
 *
 * Ground course in 100ths of a degree
 * `int32_t ground_course_cd`
 *
 * 3D speed in cm/s
 * `int32_t speed_3d_cm`
 *
 * HDOP (Horizontal Dilution of Precision in cm)
 * `int16_t hdop`
 *
 * Number of Satellites
 * `uint8_t num_sats`
 *
 * Flag for new data. Set to true during ::update
 * `bool new_data`
 *
 * GPS Fix Status. 0=No Fix, 2=2D Fix, 3=3D Fix
 * `Fix_Status fix`
 *
 * Set to true if last read is valid
 * `bool valid_read`
 *
 * TODO Remove this
 * `bool print_errors`
 *
 * Last GPS Fix time in ms. Will be updated during each call to ::update.
 * `uint32_t last_fix_time`
 *
 * TODO Remove this. GPS Engine backend setting
 * enum GPS_Engine_Setting _nav_setting;
 *
 * GPS Velocities in cm/s. These are set by the GPS backend and _velocity_<direction> is set during call to ::update
 * `int32_t _vel_north`
 * `int32_t _vel_east`
 * `int32_t _vel_down`
 *
 * Flag for whether the GPS provide velocity readings in 3D. Used in ::update to determine whether to use this or groundspeed
 * `bool _have_raw_velocity`
 *
 * GPS communication baudrate
 * uint16_t _baudrate;
 *
 * Time that the last GPS timestamp message was received
 * `uint32_t _last_gps_time`
 *
 * Last time that the GPS driver got a good packet from the GPS
 * `uint32_t _idleTimer`
 *
 * Current GPS status
 * `GPS_Status _status`
 *
 * Previous ground speed in cm/s
 * `uint32_t _last_ground_speed_cm`
 *
 * Calculated velocities (in m/s) from the ::update method. Calculated based on either groundspeed or raw 3D velocity readings
 * `float _velocity_north`
 * `float _velocity_east`
 * `float _velocity_down`
 *
 */

class AP_GPS_Sim: public GPS
{
public:
	AP_GPS_Sim() : GPS()
	{}

	/* @brief Initialise the simulated GPS */
    virtual void init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE) override;

	/* @brief Read a measurement from the simulated sensor */
    virtual bool read() override;

#ifdef TARGET_ARCH_LINUX
	void set_gps_vel3d(double vel_east, double vel_north, double vel_up);
	void set_gps_attitude(double lat, double lng, double alt);
#endif

	/* @brief Get GPS lag for simulated GPS. No lag for simulated GPS */
    float get_lag() override { return 0.0f; }

};

#endif
