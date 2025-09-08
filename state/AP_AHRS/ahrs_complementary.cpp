
#include <AP_Math.h>

#include "ahrs_complementary.h"

#include "mcinstance.h"
extern MCInstance mincopter;

void AHRS_Complementary::ahrs_update(void)
{
	/* updating _ahrs_state quaternion */

	// TODO Can these return a const reference instead?
	Vector3f accel_reading = mincopter.ins.get_accel();
	Vector3f gyro_reading = mincopter.ins.get_gyro();

	float ins_time_s= mincopter.ins.get_delta_time();

	// Get magnetometer reading
	Vector3f mag_reading = mincopter.compass.get_field();

	// Convert mag into NED
	float temp_x = mag_reading.x;
	mag_reading.x = -mag_reading.z;
	mag_reading.z = -mag_reading.y;
	mag_reading.y = temp_x;

	// TODO This should definitely be done in the compass class instead
	// Correct for declination/inclination (using -68deg inc and +11deg dec)
	Vector3f mag_ned(
			mag_reading.x*0.36772402 -mag_reading.y*0.07147831 + mag_reading.z*0.92718385,
			mag_reading.x*0.190809 - mag_reading.y*0.98162718,
			-mag_reading.x*0.91014888 + mag_reading.y*0.17691502 + mag_reading.z*0.37460659);

	float theta_magx = atan2f(accel_reading.y, accel_reading.z); 
	float theta_magy = atan2f(-accel_reading.x,
			safe_sqrt(accel_reading.y*accel_reading.y + accel_reading.z*accel_reading.z));

	// Separate calculation for yaw using mag
	float theta_magz = atan2f(
			mag_reading.z*sin(theta_magy) - mag_reading.y*cos(theta_magy),
			mag_reading.x*sin(theta_magx) + sin(theta_magx)*(mag_reading.y*sin(theta_magy)+mag_reading.z*cos(theta_magy)));

	if (_first_update) {
		// Don't fuse on first update
		euler_internal.x = theta_magx;
		euler_internal.y = theta_magy;
		euler_internal.z = theta_magz;
		_first_update = 0;
	} else {
		// Fuse with gyro
		float theta_gyrox = euler_internal.x + gyro_reading.x*ins_time_s;
		float theta_gyroy = euler_internal.y + gyro_reading.y*ins_time_s;
		float theta_gyroz = euler_internal.z + gyro_reading.z*ins_time_s;

		euler_internal.x = alpha*theta_gyrox + (1-alpha)*theta_magx;
		euler_internal.y = alpha*theta_gyroy + (1-alpha)*theta_magy;
		euler_internal.z = alpha*theta_gyroz + (1-alpha)*theta_magz;
	}

	// Compute and update quaternion
	
	_ahrs_state->_attitude.from_euler(
			euler_internal.x,
			euler_internal.y,
			euler_internal.z);



	return;
}

void AHRS_Complementary::_ahrs_init_internal(void)
{
	// Set internal euler representation to zero
	euler_internal(0,0,0);

	// We are in first update
	_first_update = 1;

	return;
}


