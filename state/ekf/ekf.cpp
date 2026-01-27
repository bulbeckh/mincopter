

#include "state_ekf.h"

#include <AP_Math.h>

#include "mcinstance.h"
extern MCInstance mincopter;

// NOTE We forward declare the generated ekf casadi methods here rather than using a header file

// Casadi generated c functions for ekf prediction and correction steps
extern "C" {
	int ekf_predict(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem);
	int ekf_fuse_acc(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem);
	int ekf_fuse_mag(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem);
	int ekf_fuse_gps(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem);
	int ekf_fuse_baro(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem);
}

/* NOTE TODO The following is a placeholder for the predict, correct functions while we reduce size */
int _none_ekf_function(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem) {
	return 0;
}

void StateEKF::ekf_predict(void) {

	// When we run ekf_predict, our last interaction should be either a fusion (in which case the cov_out and cov_est should
	// be equal) or a previous call to ekf_predict. If that latter, we need to ensure that our covariance matrix that we are
	// feeding in (which is confusingly re-using the cov_out variable) is correct
	
	// Run prediction step (last three args are real/int workspace sizes and memory index, which are all 0)
	int result = ::ekf_predict((const EKF_DATA_TYPE**)ekf_predict_arg, ekf_predict_res, 0, 0, 0);
	//int _result = _none_ekf_function((const EKF_DATA_TYPE**)ekf_predict_arg, ekf_predict_res, 0, 0, 0);\
	
	// After the prediction step, we have the state and covariance in state_est and cov_est. These are used as inputs
	// to the fuse functions.
	//
	// If we are planning to fuse on different time steps, we should also update the mcstate representation (data) here.
	if (!result) {
		for (uint16_t i=0;i<EKF_STATE_SIZE;i++) state_est[i] = state_out[i];
		for (uint16_t i=0;i<EKF_COVARIANCE_SIZE * EKF_COVARIANCE_SIZE;i++) cov_est[i] = cov_out[i];
	} else {
		mincopter.hal.console->printf("[EKF] Error in ekf predict\r\n");
		return;
	}
	
	return;
}

#if EKF_FUSE_ACC
void StateEKF::ekf_fuse_acc(void) {
	int result = ::ekf_fuse_acc((const EKF_DATA_TYPE**)ekf_fuse_acc_arg, ekf_fuse_acc_res, 0, 0, 0);

	if (!result) {
		// Update the state_est and cov_est matrices so that if we call another fuse function, they will be using the latest data
		for (uint16_t i=0;i<EKF_STATE_SIZE;i++) state_est[i] = state_out[i];
		for (uint16_t i=0;i<EKF_COVARIANCE_SIZE * EKF_COVARIANCE_SIZE;i++) cov_est[i] = cov_out[i];

	} else {
		mincopter.hal.console->printf("[EKF] Error in accelerometer fusion\r\n");
		return;
	}

	return;
}
#endif

#if EKF_FUSE_MAG
void StateEKF::ekf_fuse_mag(void) {
	int result = ::ekf_fuse_mag((const EKF_DATA_TYPE**)ekf_fuse_mag_arg, ekf_fuse_mag_res, 0, 0, 0);

	if (!result) {
		// Update the state_est and cov_est matrices so that if we call another fuse function, they will be using the latest data
		for (uint16_t i=0;i<EKF_STATE_SIZE;i++) state_est[i] = state_out[i];
		for (uint16_t i=0;i<EKF_COVARIANCE_SIZE * EKF_COVARIANCE_SIZE;i++) cov_est[i] = cov_out[i];

	} else {
		mincopter.hal.console->printf("[EKF] Error in mag fusion\r\n");
		return;
	}

	return;
}
#endif

#if EKF_FUSE_GPS
void StateEKF::ekf_fuse_gps(void) {
	int result = ::ekf_fuse_gps((const EKF_DATA_TYPE**)ekf_fuse_gps_arg, ekf_fuse_gps_res, 0, 0, 0);

	if (!result) {
		// Update the state_est and cov_est matrices so that if we call another fuse function, they will be using the latest data
		for (uint16_t i=0;i<EKF_STATE_SIZE;i++) state_est[i] = state_out[i];
		for (uint16_t i=0;i<EKF_COVARIANCE_SIZE * EKF_COVARIANCE_SIZE;i++) cov_est[i] = cov_out[i];

	} else {
		mincopter.hal.console->printf("[EKF] Error in gps fusion\r\n");
		return;
	}

	return;
}
#endif

#if EKF_FUSE_BARO
void StateEKF::ekf_fuse_baro(void) {
	int result = ::ekf_fuse_baro((const EKF_DATA_TYPE**)ekf_fuse_baro_arg, ekf_fuse_baro_res, 0, 0, 0);

	if (!result) {
		// Update the state_est and cov_est matrices so that if we call another fuse function, they will be using the latest data
		for (uint16_t i=0;i<EKF_STATE_SIZE;i++) state_est[i] = state_out[i];
		for (uint16_t i=0;i<EKF_COVARIANCE_SIZE * EKF_COVARIANCE_SIZE;i++) cov_est[i] = cov_out[i];

	} else {
		mincopter.hal.console->printf("[EKF] Error in baro fusion\r\n");
		return;
	}

	return;
}
#endif

void StateEKF::update(void)
{

	// TODO Remove this for something else
	static uint16_t _state_counter=0;
	if (_state_counter<20) {

		data.euler_rates.x = 0.0f;
		data.euler_rates.y = 0.0f;
		data.euler_rates.z = 0.0f;

		// No roll/pitch/yaw
		data.attitude.from_euler(0.0f, 0.0f, 0.0f);
		data.euler = Vector3f(0.0f, 0.0f, 0.0f);

		_state_counter++;
		return;
	}

	// This is called from the main loop at ~100Hz
	
	// Prepare the predict and correct variable arrays with sensor readings and previous estimated state from _state
	setup_ekf_args();

	ekf_predict();

#if EKF_FUSE_ACC
	ekf_fuse_acc();
#endif

#if EKF_FUSE_MAG
	ekf_fuse_mag();
#endif

#if EKF_FUSE_GPS
	ekf_fuse_gps();
#endif

#if EKF_FUSE_BARO
	ekf_fuse_baro();
#endif

	// After zero or more fusion steps, the latest state/covariance will be available in state_est and cov_est and we
	// should update our internal state using these two matrices
	
	// Update internal state variables
	
	// Check that the norm is non-zero
	float q_norm = safe_sqrt(sq(state_est[6]) + sq(state_est[7]) + sq(state_est[8]) + sq(state_est[9]));

	if (q_norm != 0.0f) {
		state_est[6] /= q_norm;
		state_est[7] /= q_norm;
		state_est[8] /= q_norm;
		state_est[9] /= q_norm;
	} else {
		// TODO - Log that we have a zero quaternion (which likely indicates and error w EKF functions)
		mincopter.hal.console->printf("[EKF] Zero-quaternion after update\r\n");
		return;
	}

	// MCState maintains the following state variables that we need to update after every EKF run
	//
	// - x, position (3x float)
	// - v, velocity (3x float)
	// - q, quaternion (4x float)
	// - euler angles (3x float)
	
	// Update MCState via _state variable - _result[0] is the state_out as (x,v,q)
	data.attitude(
			state_est[6],
			state_est[7],
			state_est[8],
			state_est[9]);

	// Also update euler angles
	data.attitude.to_euler(
			&data.euler.x,
			&data.euler.y,
			&data.euler.z
			);

	data.position[0] = state_est[0];
	data.position[1] = state_est[1];
	data.position[2] = state_est[2];

	data.velocity[0] = state_est[3];
	data.velocity[1] = state_est[4];
	data.velocity[2] = state_est[5];

	return;
}

void StateEKF::setup_ekf_args(void)
{
	// This function is run at each iteration of the EKF and reads the latest measurements from each of the sensors
	// that are being fused on this timestep
	//
	// We need to retrieve the following readings ahead of our prediction and fusion/correction steps
	//
	// State: x
	// State: v
	// State: q
	// Accelerometer Measurement
	// Accelerometer Variance
	// Gyrometer Measurement
	// Gyrometer Variance
	// Magnetometer Measurement
	// Magnetometer Variance
	// GPS Position
	// GPS Velocity
	// GPS Variances
	// Barometer altitude estimate
	// Barometer variance
	//
	//
	// Some of the variances will be constant and other (like the GPS) will change between each update

	// TODO Does our dt represent a gyrometer time or an accelerometer time?
	// Read dt from gyrometer
	dt = 0.01; // 100Hz approx.
			   
	// Get state (q,x,v) from _state
	// TODO FIx this - should not really be using mcstate directly for state here - should be passed in from somewhere else
	x[0] = data.position[0];
	x[1] = data.position[1];
	x[2] = data.position[2];

	v[0] = data.velocity[0];
	v[1] = data.velocity[1];
	v[2] = data.velocity[2];

	q[0] = data.attitude[0];
	q[1] = data.attitude[1];
	q[2] = data.attitude[2];
	q[3] = data.attitude[3];
	
	// Get latest gyrometer reading in rad/s
	Vector3f gyro = mincopter.ins.get_gyro();
	
	// Get latest accel reading in m/s2
	Vector3f accel = mincopter.ins.get_accel();

	a[0] = accel.x;
	a[1] = accel.y;
	a[2] = accel.z;

	w[0] = gyro.x;
	w[1] = gyro.y;
	w[2] = gyro.z;
	
	// Get m - latest magnetometer reading
	Vector3f field = mincopter.compass.get_field();

	// normalize the magnetic field as we only need it for rotational reference
	field.normalize();

	m[0] = field.x;
	m[1] = field.y;
	m[2] = field.z;

	var_gyro = 0.01*0.01;
	//var_accel = 1000*1000;
	var_accel = 0.01*0.01;
	var_mag = 0.01*0.01;

	// TODO Check this calculation
	// GPS delta from home location in centi-degrees (deg*1e7)
	int32_t lat_offset = mincopter.g_gps->latitude - home.lat;
	int32_t lon_offset = mincopter.g_gps->latitude - home.lng;

	gps_pos[0] = (lat_offset*111320) / (1e7); // North (x)
	gps_pos[1] = (lon_offset*40075000) * cos(mincopter.g_gps->latitude/1e7) / (1e7); // East (y)
																					 
	// Update altitude reading NOTE we need this in NED frame so we subtract the altitude_cm from the home alt as these are in ENU frame
	gps_pos[2] = (home.alt - mincopter.g_gps->altitude_cm) / 100.0f;

	Vector3f velocity_reading_gps = mincopter.g_gps->velocity_vector();

	// Update velocity readings
	gps_vel[0] = velocity_reading_gps.x;
	gps_vel[1] = velocity_reading_gps.y;
	gps_vel[2] = velocity_reading_gps.z;

	var_gps_pos = 100;
	var_gps_vel = 100;

	// Update altitude (barometer) reading
	barometer_altitude = mincopter.barometer.get_altitude();

	var_barometer = 0.1*0.1;

	return;
}


void StateEKF::reset(void)
{
	// TODO We need to figure out the behaviour when a reset is called. It is 
	// complicated by the fact that we have two reset methods - one to zero everything
	// and then another to initialise to an euler angles (RPY) value
	
	// This reset method should reset both the internal EKF state variables
	// but also the state from the _state struct
	
	// acc2q method from github.com/mayitzin/ahrs
	
	// Reset covariance matrix to identity matrix
	for (uint8_t i=0;i<10;i++) {
		for (uint8_t j=0;j<10;j++) {
			if (i==j) {
				cov_out[i*10+j] = 1.0f;
				cov_est[i*10+j] = 1.0f;
			} else {
				cov_out[i*10+j] = 0.0f;
				cov_est[i*10+j] = 0.0f;
			}
		}
	}

	// TODO Should we be resetting position/velocity to most recent GPS measurement?
	// Reset position and velocity
	for (uint8_t i=0;i<3;i++) x[i] = 0.0f;
	for (uint8_t i=0;i<3;i++) v[i] = 0.0f;

	// Reset quaternion to unit quaternion
	q[0] = 1.0f;
	q[1] = 0.0f;
	q[2] = 0.0f;
	q[3] = 0.0f;

	/*
	// Reset quaternion to latest orientation
	Vector3f accel_bf = mincopter.ins.get_accel().normalized();

	double ex = atan2(accel_bf.y, accel_bf.z);
	double ey = atan2(-accel_bf.x, safe_sqrt(accel_bf.y*accel_bf.y + accel_bf.z*accel_bf.z));
	double ez = 0.0f;

	double cx2 = cos(ex/2.0f);
	double sx2 = sin(ex/2.0f);
	double cy2 = cos(ey/2.0f);
	double sy2 = sin(ey/2.0f);

	// Scalar-first quaternion rep
	q[0] = cx2*cy2;
	q[1] = sx2*cy2;
	q[2] = cx2*sy2;
	q[3] = -sx2*sy2;
	*/

	// TODO normalize the _altitude quaternion


	// Update _altitude quaternion
	data.attitude(q[0], q[1], q[2], q[3]);
	
	return;
}

void StateEKF::init_derived(void)
{
	reset();
	return;
}
