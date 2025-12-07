
#include <AP_Math.h>

#include "state_complementary.h"

#include "mcinstance.h"
extern MCInstance mincopter;

void StateComplementary::update(void)
{
	/* **Complementary Filter Update method
	 *
	 * These equations are derived assuming both the ENU frame and **extrinsic** euler
	 * angles. Since our sensors and measurements are in NED frame, we start by converting
	 * sensor measurements to ENU, computing orientation and then converting roll, pitch, yaw
	 * back to NED frame
	 *
	 */

	static uint16_t _state_counter=0;
	if (_state_counter<20) {

		// TODO NOTE In the first few iterations of the simulation, our packet contains no valid
		// readings and will send 'nans' or all zeroes as measurement for a few of the sensors. This
		// corrupts our complementary filter so we basically ignore the first 20 iterations and assume no movement
	
		// No roll rates
		data.euler_rates.x = 0.0f;
		data.euler_rates.y = 0.0f;
		data.euler_rates.z = 0.0f;

		// No roll/pitch/yaw
		data.attitude.from_euler(0.0f, 0.0f, 0.0f);
		data.euler = Vector3f(0.0f, 0.0f, 0.0f);

		_state_counter++;

		return;
	}

	// TODO Can these return a const reference instead?
	// Accelerometer and Gyrometer readings in NED frame
	Vector3f accel_reading = mincopter.ins.get_accel();

	// TODO NOTE This is fixed by enabling accelerometer offsets
	/*
	// Add offset constant for z-axis
#ifdef TARGET_ARCH_AVR
	accel_reading.z += 3.57;
#endif
	*/

	// NOTE No need to normalise accel
	//accel_reading.normalize();

	// Get elapsed gyrometer time for use in integration of gyros
	float ins_time_s = mincopter.ins.get_delta_time();
	// NOTE TODO temporarily fixed this to 100Hz update rate
	ins_time_s = 0.015;

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

	// TODO Remove this whole thing - we do magnetic declination correction AFTER we calculate the yaw
	/*
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
	*/

	// TODO UPDATED
	// Estimate **roll** and **pitch** from accelerometer gravity vector by also multiplying by sign of y
	float theta_magx = atan2f(-accel_reading.y, -accel_reading.z);

	// TODO UPDATED
	// NOTE This is a valid pitch reading in the NED frame, between [-pi/2, pi/2]
	float theta_magy = atan2f(accel_reading.x,
			safe_sqrt(accel_reading.y*accel_reading.y + accel_reading.z*accel_reading.z));

	// TODO Update this calculation as our roll, pitch, yaw euler angles are now EXTRINSIC and hence this rotation may no longer be valid
	// Our yaw calculation uses magnetometer readings
	/*
	 * float theta_magz = atan2f(mag_reading.z*sin(theta_magy) - mag_reading.y*cos(theta_magy),
			mag_reading.x*cos(theta_magx) + sin(theta_magx)*(mag_reading.y*sin(theta_magy) + mag_reading.z*cos(theta_magy)));
	*/

	float theta_magz = atan2f(
			-1*mag_reading.y*cos(theta_magx) + mag_reading.z*sin(theta_magx),
			mag_reading.x*cos(theta_magy) + mag_reading.y*sin(theta_magy)*sin(theta_magx) + mag_reading.z*sin(theta_magy)*cos(theta_magx)
			);

	// TODO Now is where we correct for magnetic declination (yaw)
	theta_magz += 0.19;
	
	Vector3f gyro_reading = mincopter.ins.get_gyro();
	
	// Update euler rates ahead of gyro integration below
	/*
	_ahrs_state->_euler_rates.x = gyro_reading.x + gyro_reading.y*sin(_ahrs_state->_euler.x)*tan(_ahrs_state->_euler.y) + gyro_reading.z*cos(_ahrs_state->_euler.x)*tan(_ahrs_state->_euler.y);
	_ahrs_state->_euler_rates.y = gyro_reading.y*cos(_ahrs_state->_euler.x) - gyro_reading.z*sin(_ahrs_state->_euler.x);
	_ahrs_state->_euler_rates.z = gyro_reading.y*sin(_ahrs_state->_euler.x) / cos(_ahrs_state->_euler.y) + gyro_reading.z*cos(_ahrs_state->_euler.x) / cos(_ahrs_state->_euler.y);
	*/

	data.euler_rates.x = gyro_reading.x;
	data.euler_rates.y = gyro_reading.y;
	data.euler_rates.z = gyro_reading.z;

	if (_first_update) {
		// Don't fuse on first update
		euler_internal.x = theta_magx;
		euler_internal.y = theta_magy;
		euler_internal.z = theta_magz;
		_first_update = 0;
	} else {
		// Fuse with gyro
		// TODO Change gyro_reading to the euler rate for integration
		float theta_gyrox = euler_internal.x + data.euler_rates.x*ins_time_s;
		float theta_gyroy = euler_internal.y + data.euler_rates.y*ins_time_s;
		float theta_gyroz = euler_internal.z + data.euler_rates.z*ins_time_s;

		// We fuse with the accelerometer tilt only when we are not accelerating (norm is close to 9.8)
		
		float a_norm = safe_sqrt(accel_reading.x*accel_reading.x + accel_reading.y*accel_reading.y + accel_reading.z*accel_reading.z);

		if (fabs(a_norm - 9.8) >= 1.0f) {
			// If we are accelerating too much then just integrate gyro
			euler_internal.x = theta_gyrox;
			euler_internal.y = theta_gyroy;
			euler_internal.z = theta_gyroz;
		} else {
			euler_internal.x = alpha*theta_gyrox + (1-alpha)*theta_magx;
			euler_internal.y = alpha*theta_gyroy + (1-alpha)*theta_magy;
			euler_internal.z = alpha_yaw*theta_gyroz + (1-alpha_yaw)*theta_magz;
		}
	}


	// Compute and update quaternion (in NED frame)
	data.attitude.from_euler(
			euler_internal.x,
			euler_internal.y,
			euler_internal.z);

	// Update the euler angles
	data.euler = euler_internal;

	// TODO Add fusion of position and velocity measurements here - see complementary-derivation.ipynb in state/design for implementation

	// We only update/fuse position and velocity measurements if we have set our initial latitude/longitude/altitude (from GPS) as well as our
	// ground pressure and temperature
	if (home_set) {

		// Integrate accelerometer for position/velocityS

		// TODO This transformation should be a standard transformation with one of the math libraries (matrix3f?) and one of the existing state representations (DCM)
		// Convert IMU reading to world frame
		float accel_x_world = cos(data.euler.y)*cos(data.euler.z)*accel_reading.x
			+ accel_reading.y*(sin(data.euler.x)*sin(data.euler.y)*cos(data.euler.z) - cos(data.euler.x)*sin(data.euler.z))
			+ accel_reading.z*(cos(data.euler.x)*sin(data.euler.y)*cos(data.euler.z) + sin(data.euler.x)*sin(data.euler.z));

		float accel_y_world = cos(data.euler.y)*sin(data.euler.z)*accel_reading.x
			+ accel_reading.y*(sin(data.euler.x)*sin(data.euler.y)*sin(data.euler.z) + cos(data.euler.x)*cos(data.euler.z))
			+ accel_reading.z*(cos(data.euler.x)*sin(data.euler.y)*sin(data.euler.z) - sin(data.euler.x)*cos(data.euler.z));

		// Transform accelerometer reading from body frame to world frame
		float accel_z_world = -1.0*sin(data.euler.y)*accel_reading.x + sin(data.euler.x)*cos(data.euler.y)*accel_reading.y
			+ cos(data.euler.x)*cos(data.euler.y)*accel_reading.z;

		// Correct world frame z-accel for gravity
		accel_z_world += GRAVITY_MSS;

		// TODO Fuse GPS velocity
		Vector3f gps_ned_velocities = mincopter.g_gps->velocity_vector();

		// TODO We update state at 100Hz but we receive new barometer/gps readings at a lower frequency. We need to add a flag
		// for checks for new sensor data and only fuse when it is received.
		
		// TODO Should we be updating position with the previous timesteps velocity measurement or the most recent velocity measurement
		// Integrate velocities
		data.velocity[0] = (1-x_axis_gpsvel_fuse_alpha)*(data.velocity[0] + accel_x_world*ins_time_s) + x_axis_gpsvel_fuse_alpha*gps_ned_velocities.x;
		data.velocity[1] = (1-y_axis_gpsvel_fuse_alpha)*(data.velocity[1] + accel_y_world*ins_time_s) + y_axis_gpsvel_fuse_alpha*gps_ned_velocities.y;
		data.velocity[2] = (1-z_axis_gpsvel_fuse_alpha)*(data.velocity[2] + accel_z_world*ins_time_s) + z_axis_gpsvel_fuse_alpha*gps_ned_velocities.z;

		// TODO Fuse GPS position
		// We have the GPS latitude/longitude and altitude (as well as the three velocity measurements). We fuse our state estimation with
		// these to get an accurate position reading.
		//
		// int32_t g_gps->latitude : current latitude in degrees*1e7
		// int32_t g_gps->longitude : current longitude in degrees*1e7
		// int32_t g_gps->altitude_cm : current altitude in cm
		//
		// int32_t home.lat : latitude (deg*1e7) at time of arm
		// int32_t home.lng : longitude (deg*1e7) at time of arm
		// int32_t home.alt : altitude (cm) at time of arm
		//
		// Vector3f g_gps->velocity_vector() : GPS NED velocities in m/s
		
		int32_t lat_offset = mincopter.g_gps->latitude - home.lat;
		int32_t lng_offset = mincopter.g_gps->longitude - home.lng;

		x_position_est = lat_offset*0.0111320f;
		y_position_est = 4.0075000f*lng_offset*cos(M_PI_F*mincopter.g_gps->latitude/(180*1e7f)) / 360.0f;

		z_position_est = -1.0f*mincopter.g_gps->altitude_cm / 1e2f;

		// Integrate position (with z-axis fuse weighting)
		data.position[0] = (1-x_axis_gps_fuse_alpha)*(data.position[0] + data.velocity[0]*ins_time_s) + x_axis_gps_fuse_alpha*x_position_est;
		data.position[1] = (1-y_axis_gps_fuse_alpha)*(data.position[1] + data.velocity[1]*ins_time_s) + y_axis_gps_fuse_alpha*y_position_est;

		// For z-axis, we fuse both GPS position as well as barometer with a lot more weighting to barometer
		data.position[2] = (1-z_axis_baro_fuse_alpha-z_axis_gps_fuse_alpha)*(data.position[2] + data.velocity[2]*ins_time_s)
			+ z_axis_baro_fuse_alpha*mincopter.barometer.get_altitude()*(-1.0f)
			+ z_axis_gps_fuse_alpha*z_position_est;

	}

	if (_state_counter%100==0) {
		mincopter.hal.console->printf("t: %f, pos(%f,%f,%f) lat/lng offset (%d,%d)\r\n", //eul:%f,%f,%f | %f,%f,%f | %f,%f,%f | %f,%f,%f | %f,%f,%f\r\n",
				ins_time_s,
				/*
				euler_internal.x, euler_internal.y, euler_internal.z,
				accel_reading.x,
				accel_reading.y,
				accel_reading.z,
				gyro_reading.x,
				gyro_reading.y,
				gyro_reading.z,
				mag_reading.x,
				mag_reading.y,
				mag_reading.z,
				*/
				x_position_est,
				y_position_est,
				z_position_est,
				mincopter.g_gps->latitude - home.lat,
				mincopter.g_gps->longitude - home.lng);
	}

	_state_counter++;

	return;
}

void StateComplementary::init_derived(void)
{
	// Set internal euler representation to zero
	euler_internal(0,0,0);

	// We are in first update
	_first_update = 1;

	return;
}


