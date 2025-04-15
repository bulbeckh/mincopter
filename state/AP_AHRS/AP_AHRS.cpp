/*
  APM_AHRS.cpp

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
#include <AP_AHRS.h>
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

void AP_AHRS::set_trim(Vector3f new_trim)
{
    Vector3f trim;
    trim.x = constrain_float(new_trim.x, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(new_trim.y, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    _trim = trim;
}

void AP_AHRS::add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom)
{
    Vector3f trim = _trim;

    // add new trim
    trim.x = constrain_float(trim.x + roll_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(trim.y + pitch_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));

    // set new trim values
    _trim = trim;
}

Vector2f AP_AHRS::groundspeed_vector(void)
{
    // Generate estimate of ground speed vector using air data system
    Vector2f gndVelADS;
    Vector2f gndVelGPS;

		bool gotGPS = (_gps && _gps->status() >= GPS::GPS_OK_FIX_2D);
    
    // Generate estimate of ground speed vector using GPS
    if (gotGPS) {
	    float cog = radians(_gps->ground_course_cd*0.01f);
	    gndVelGPS = Vector2f(cosf(cog), sinf(cog)) * _gps->ground_speed_cm * 0.01f;
    }
    // Only GPS data is available so return GPS estimate
    if (gotGPS) {
	    return gndVelGPS;
    }
    return Vector2f(0.0f, 0.0f);
}

/*
  get position projected by groundspeed and heading
 */
bool AP_AHRS::get_projected_position(struct Location &loc)
{
        if (!get_position(loc)) {
		return false;
        }
        location_update(loc, degrees(yaw), _gps->ground_speed_cm * 0.01 * _gps->get_lag());
        return true;
}

/*
  get the GPS lag in seconds
 */
float AP_AHRS::get_position_lag(void) const
{
    return _gps->get_lag();
}
