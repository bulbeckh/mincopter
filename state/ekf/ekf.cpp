

#include "ekf.h"
#include <AP_Math.h>

#include "mcinstance.h"
extern MCInstance mincopter;

// TODO Bad hack - fix this
#include "mcstate.h"
extern MCState mcstate;

// NOTE We forward declare the generated ekf casadi methods here rather than using a header file

// TODO This is bad - need to formalise the way be define MCState and pass AHRS and INAV objects
EKF ekf;

// Casadi generated c functions for ekf prediction and correction steps
extern "C" {
	int ekf_predict(const double** arg, double** res, long long int* iw, double* w, int mem);
	int ekf_correct(const double** arg, double** res, long long int* iw, double* w, int mem);
}

void EKF::inav_update(void)
{
	// This is called from the main loop at ~100Hz
	
	// Prepare the predict and correct variable arrays with sensor readings and previous estimated state from _state
	setup_ekf_args();
	
	// Run prediction step (last three args are real/int workspace sizes and memory index, which are all 0)
	// NOTE Not sure why the ekf_predict function requires const double** arg as the arg changes between prediction runs.
	int _result = ekf_predict((const double**)ekf_predict_arg, ekf_predict_res, 0, 0, 0);

	// Run correction step
	_result = ekf_correct((const double**)ekf_correct_arg, ekf_correct_res, 0, 0, 0);

	// Normalise quaternion before re-assigning
	float q_norm = safe_sqrt(sq(ekf_correct_res[0][6]) + sq(ekf_correct_res[0][7]) + sq(ekf_correct_res[0][8]) + sq(ekf_correct_res[0][9]));

	// Check that the norm is non-zero
	if (q_norm > 1e-6) { 
		ekf_correct_res[0][6] /= q_norm;
		ekf_correct_res[0][7] /= q_norm;
		ekf_correct_res[0][8] /= q_norm;
		ekf_correct_res[0][9] /= q_norm;
	}

	// Update MCState via _state variable - _result[0] is the state_out as (x,v,q)
	_ahrs_state->_attitude(
			ekf_correct_res[0][6],
			ekf_correct_res[0][7],
			ekf_correct_res[0][8],
			ekf_correct_res[0][9]);

	_inav_state->_position[0] = ekf_correct_res[0][0];
	_inav_state->_position[1] = ekf_correct_res[0][1];
	_inav_state->_position[2] = ekf_correct_res[0][2];

	_inav_state->_velocity[0] = ekf_correct_res[0][3];
	_inav_state->_velocity[1] = ekf_correct_res[0][4];
	_inav_state->_velocity[2] = ekf_correct_res[0][5];

	return;
}

void EKF::setup_ekf_args(void)
{
	/* Predict setup */


	// TODO Does our dt represent a gyrometer time or an accelerometer time?
	// Read dt from gyrometer
	dt = 0.01; // 100Hz approx.
	
	// Get w - latest gyrometer reading in rad/s
	Vector3f gyro = mincopter.ins.get_gyro();
	// NOTE TODO We are deliberately switching (rotating) the axis here because the MPU6050 on my breadboard is backwards
	w[0] = -gyro.x;
	w[1] = -gyro.y;
	w[2] = gyro.z;
	
	// Get a - latest accel reading in m/s2
	Vector3f accel = mincopter.ins.get_accel();
	Vector3f accel_normalized = accel;
	
	accel_normalized.normalize();

	/* NOTE Our EKF is wrong as the functions expect a normalized acceleration vector but we actually need to integrate the
	 * real acceleration (in m/s2). For now, to get the orientation working, we pass in the normalized vector to use as a
	 * gravitational measurement, knowing that the pos/vel will not work */

	/* 
	a[0] = accel.x;
	a[1] = accel.y;
	a[2] = accel.z;
	*/

	// NOTE TODO See above comment about rotations for gyro/accel for MPU6050
	a[0] = -accel_normalized.x;
	a[1] = -accel_normalized.y;
	a[2] = accel_normalized.z;
	
	// Get accel and gyro variances TODO These should not change and be retrieved during
	// init from the sensor drivers
	
	var_gyro = 0.3*0.3;
	var_accel = 0.5*0.5;
	var_mag = 0.8*0.8;
	
	// Get state (q,x,v) from _state
	// TODO FIx this - should not really be using mcstate directly for state here - should be passed in from somewhere else
	x[0] = mcstate._state._position[0];
	x[1] = mcstate._state._position[1];
	x[2] = mcstate._state._position[2];

	v[0] = mcstate._state._velocity[0];
	v[1] = mcstate._state._velocity[1];
	v[2] = mcstate._state._velocity[2];
	
	/* Correct step */
	
	// Get m - latest magnetometer reading
	Vector3f field = mincopter.compass.get_field();

	// normalize the magnetic field as we only need it for rotational reference
	field.normalize();

	m[0] = field.x;
	m[1] = field.y;
	m[2] = field.z;
	
	// Get gps pos/vel/variances
	// TODO I don't even think GPS is fused in the function call - need to check
	gps_pos[0] = 0.0f;
	gps_pos[1] = 0.0f;
	gps_pos[2] = 0.0f;

	gps_vel[0] = 0.0f;
	gps_vel[1] = 0.0f;
	gps_vel[2] = 0.0f;

	var_gps_pos = 0.1;
	var_gps_vel = 0.1;
	
	return;
}


void EKF::reset(void)
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
				cov[i*10+j] = 1.0f;
			} else {
				cov[i*10+j] = 0.0f;
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
	_ahrs_state->_attitude(q[0], q[1], q[2], q[3]);
	
	
	return;
}


void EKF::_ahrs_init_internal(void)
{
	// Initialise quaternion state
	reset();

	return;
}

void EKF::ahrs_update(void)
{
	/* For the EKF, the entire state update (predict and correct) is handled by
	 * inav_update. As such, we don't actually do anything in this method. */
	return;
}

void EKF::_inav_init_internal(void)
{
	reset();
	return;
}
