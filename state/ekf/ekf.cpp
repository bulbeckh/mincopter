

#include "ekf.h"
#include <math.h>

#include "mcinstance.h"
extern MCInstance mincopter;

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
	// TODO Not sure why the ekf_predict function requires const double** arg as the arg changes between prediction runs.
	int _result = ekf_predict((const double**)ekf_predict_arg, ekf_predict_res, 0, 0, 0);

	// Run correction step
	_result = ekf_correct((const double**)ekf_correct_arg, ekf_correct_res, 0, 0, 0);

	return;
}

void EKF::setup_ekf_args(void)
{
	/* Predict setup */

	// TODO Does our dt represent a gyrometer time or an accelerometer time?
	// Read dt from gyrometer
	
	// Get w - latest gyrometer reading
	
	// Get a - latest accel reading
	
	// Get accel and gyro variances TODO These should not change and be retrieved during
	// init from the sensor drivers
	
	// Get state (q,x,v) from _state
	

	/* Correct step */

	// Get a - latest accel reading
	
	// Get m - latest magnetometer reading
	
	// Get var_mag - mag variance
	
	// Get gps pos/vel/variances
	
	return;
}


void EKF::reset(void)
{
	// TODO We need to figure out the behaviour when a reset is called. It is 
	// complicated by the fact that we have two reset methods - one to zero everything
	// and then another to initialise to an euler angles (RPY) value
	
	/*
	
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
	_state._attitude(q[0], q[1], q[2], q[3]);
	
	// TODO normalize the _altitude quaternion
	
	*/

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
	/* TODO */
	return;
}
