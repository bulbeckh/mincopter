

#include "ekf.h"

#include "ekf_predict.h"
#include "ekf_correct.h"

#include <math.h>

#include "mcinstance.h"
extern MCInstance mincopter;

EKF::EKF() :
	AP_AHRS(&mincopter.ins, mincopter.g_gps),
	MC_InertialNav()
{

}

// TODO REMOVE THESE 
float EKF::get_error_rp(void) {}
float get_error_yaw(void) {}


void EKF::update(void)
{
	// This is called from the main loop at ~100Hz
	
	// Run prediction step (last three args are real/int workspace sizes and memory index, which are all 0)
	int _result = ekf_predict(&ekf_predict_arg, &ekf_predict_res, 0, 0, 0);

	// Run correction step
	int _result = ekf_correct(&ekf_correct_arg, &ekf_correct_res, 0, 0, 0);

	return;
}


/* IMPLEMENT REMAINING FUNCTIONS HERE */

void EKF::predict(float dt)
{
	/* For now this is a placeholder until we change mcstate update interface */
}

void EKF::correct_pos_vel(Vector3f gps_position, Vector3f gps_velocity)
{
	/* For now this is a placeholder until we change mcstate update interface */
}

void EKF::correct_attitude(Vector3f mag_reading, Vector3f accelerometer_reading)
{
	/* For now this is a placeholder until we change mcstate update interface */
}

const Vector3f EKF::get_gyro(void) const
{
	// TODO Remove this whole interface - controllers/planners should just get
	// gyrometer readings from devices directly rather than going through state
	// estimation. Angular velocity is not really state unless it has biases
	// removed during state estimation
	return mincopter.ins.get_gyro();
}

const Vector3f& EKF::get_gyro_drift(void) const
{
	return Vector3f(0.0f,0.0f,0.0f);
}

void reset(bool recover_eulers)
{
	// acc2q method from github.com/mayitzin/ahrs
	
	// TODO Ignored the recover_eulers argument
	
	// Reset covariance matrix
	for (uint8_t i=0;i<10;i++) {
		for (uint8_t j=0;j<10;j++) cov[i][j] = 0.0f;
	}

	// TODO Should we be resetting position/velocity to most recent GPS measurement?
	// Reset position and velocity
	for (uint8_t i=0;i<3;i++) x[i] = 0.0f;
	for (uint8_t i=0;i<3;i++) v[i] = 0.0f;

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

	// Update _altitude quaternion
	_altitude(q[0], q[1], q[2], q[3]);
	
	// TODO normalize the _altitude quaternion

	return;
}

void reset_attitude(const float &roll, const float &pitch, const float &yaw)
{
	_altitude.from_euler(roll, pitch, yaw);

	q[0] = _altitude.q1;
	q[1] = _altitude.q2;
	q[2] = _altitude.q3;
	q[3] = _altitude.q4;

	return;
}

// TODO Remove these from MCState representation
float EKF::get_error_rp(void) {}
float EKF::get_error_yaw(void) {}

const Matrix3f& EKF::get_dcm_matrix(void) const
{
	_altitude.rotation_matrix(_dcm);

	return _dcm;
}

// Implementation of AP_InertialNav methods


// TODO Change name
void EKF::init(void)
{
	/* No implementation yet*/
	return;
}

void EKF::update(float dt)
{
	/* No implementation yet*/
	return;
}

bool EKF::position_ok() const
{
	/* TODO No implementation yet*/
	return;
}

void EKF::set_altitude(float new_alt)
{
	// x is a vector with position in ENU frame (z-axis is positive-up and last index)
	x[2] = new_alt;
	return;
}

void EKF::set_home_position(int32_t lat, int32_t lng)
{
	/* TODO No implementation yet*/
	return;
}

