
#include <AP_Math.h>

#include "ahrs_complementary.h"

#include "mcinstance.h"
extern MCInstance mincopter;

void AHRS_Complementary::ahrs_update(void)
{
	/* **Complementary Filter Update method
	 *
	 * These equations are derived assuming both the ENU frame and **extrinsic** euler
	 * angles. Since our sensors and measurements are in NED frame, we start by converting
	 * sensor measurements to ENU, computing orientation and then converting roll, pitch, yaw
	 * back to NED frame
	 *
	 */

	// TODO Can these return a const reference instead?
	// Accelerometer and Gyrometer readings in NED frame
	Vector3f accel_reading = mincopter.ins.get_accel();
	Vector3f gyro_reading = mincopter.ins.get_gyro();

	// Add offset constant for z-axis
#ifdef TARGET_ARCH_AVR
	accel_reading.z += 3.57;
#endif
	accel_reading.normalize();

	// Get elapsed gyrometer time for use in integration of gyros
	float ins_time_s= mincopter.ins.get_delta_time();

	// TODO Soon the 'native' sensor frame for all magnetometer/compasses will be defined as NED
	// and we won't need this step
	
	// Get magnetometer reading (in native sensor frame)
	Vector3f mag_reading = mincopter.compass.get_field();

	// Add mag offsets
#ifdef TARGET_ARCH_AVR
	mag_reading.x += 49.5;
	mag_reading.y += 157;
	mag_reading.z -= 17;
#endif
	mag_reading.normalize();

	// Convert mag into NED
#ifdef TARGET_ARCH_AVR
	float temp_x = mag_reading.x;
	mag_reading.x = -mag_reading.y;
	mag_reading.y = -temp_x;
	mag_reading.z = -mag_reading.z;
#endif

	// TODO This should definitely be done in the compass class instead
	// Correct for declination/inclination (using -68deg inc and +11deg dec)
#ifdef TARGET_ARCH_AVR
	Vector3f mag_ned(
			mag_reading.x*0.36772402 -mag_reading.y*0.190809 + mag_reading.z*0.9101488,
			mag_reading.x*0.071478 + mag_reading.y*0.981627 + mag_reading.z*0.176915,
			-mag_reading.x*0.927183 + mag_reading.z*0.374606);
#else
	// Copy mag_reading
	Vector3f mag_ned = mag_reading;
#endif


	// TODO UPDATED
	// Estimate **roll** and **pitch** from accelerometer gravity vector by also multiplying by sign of y
	float theta_magx = atan2f(-accel_reading.y, -accel_reading.z);

	// TODO UPDATED
	// NOTE This is a valid pitch reading in the NED frame, between [-pi/2, pi/2]
	float theta_magy = atan2f(accel_reading.x,
			safe_sqrt(accel_reading.y*accel_reading.y + accel_reading.z*accel_reading.z));

	// TODO Update this calculation as our roll, pitch, yaw euler angles are now EXTRINSIC and hence this rotation may no longer be valid
	// Our yaw calculation uses magnetometer readings
	float theta_magz = atan2f(mag_reading.z*sin(theta_magy) - mag_reading.y*cos(theta_magy),
			mag_reading.x*cos(theta_magx) + sin(theta_magx)*(mag_reading.y*sin(theta_magy) + mag_reading.z*cos(theta_magy)));

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

	// Compute and update quaternion (in NED frame)
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


