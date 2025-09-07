
/** Manual implementations of **ekf_correct** and **ekf_predict** as opposed to casadi implementations using rolled
 * and unrolled loops. Both casadi implementations use too much RAM and potentially exceed remaining stack.
 *
 * The EKF prediction arguments is as follows:
 *
 * EKF_DATA_TYPE* ekf_predict_arg[9] = {
 * 		&dt,
 * 		w,
 * 		a,
 * 		&var_gyro,
 * 		&var_accel,
 * 		cov, 
 * 		q,
 * 		x,
 * 		v};
 *
 * EKF_DATA_TYPE* ekf_predict_res[2] = {state_est, cov_est};
 *
 */

#include <AP_Math.h>

#define EKF_DATA_TYPE float

extern "C" {
	int ekf_predict(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem);
	int ekf_correct(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem);
}






