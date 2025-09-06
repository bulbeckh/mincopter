
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

extern "C" { 
	int ekf_predict(const EKF_DATA_TYPE** arg, EKF_DATA_TYPE** res, long long int* iw, EKF_DATA_TYPE* w, int mem)
	{
		// TODO Pass in Quaternion directly rather than creating local copy to compute DCM. Even better, compute DCM elsewhere in mcstate and use that
		Quaternion _q(arg->q[0], arg->q[1], arg->q[2], arg->q[3]);

		// Compute DCM
		Matrix3f c_rot;
		_q.rotation_matrix(c_rot);

		// State Prediction (state_est)
		for (uint8_t i=0;i<3;i++) res->state_est[i] = arg->x[i] + arg->v[i]*dt;
		for (uint8_t i=0;i<3;i++) {
			// Update v state by multiplying TRANSPOSE of c_rot with a_reading and subtract 1 (g_ref=(0,0,1) ) if we are on last index
			res->state_est[i+3] = dt*(c_rot[0][i]*a[0] + c_rot[1][i]*a[1] + c_rot[2][i]*a[2] - (i==2 ? 1 : 0 ));
		}
		res->state_est[6] = arg->q[0] + *arg->dt/2.0f * -1 * (arg->w[0]*arg->q[1] + arg->w[1]*arg->q[2] + arg->w[2]*arg->q[3]);
		res->state_est[7] = arg->q[1] + *arg->dt/2.0f * (arg->w[0]*arg->q[0] + arg->w[2]*q[2] - arg->w[1]*arg->q[3]);
		res->state_est[8] = arg->q[2] + *arg->dt/2.0f * (arg->w[1]*arg->q[0] - arg->w[2]*q[1] + arg->w[0]*arg->q[3]);
		res->state_est[9] = arg->q[3] + *arg->dt/2.0f * (arg->w[2]*arg->q[0] + arg->w[1]*q[1] - arg->w[0]*arg->q[2]);

		// Covariance prediction
		float gyro_noise[4];
		float gyro_sd = safe_sqrt(*arg->var_gyro);

		gyro_noise[0] = (-gyro_sd* *arg->dt/2.0f)*(-arg->q[1] -arg->q[2] -arg->q[3]);
		gyro_noise[1] = (-gyro_sd* *arg->dt/2.0f)*( arg->q[0] -arg->q[3] -arg->q[2]);
		gyro_noise[2] = (-gyro_sd* *arg->dt/2.0f)*( arg->q[3] +arg->q[0] -arg->q[1]);
		gyro_noise[3] = (-gyro_sd* *arg->dt/2.0f)*(-arg->q[2] +arg->q[1] +arg->q[0]);

		// TODO NOTE Do we need to manually zero this matrix?
		for (uint8_t i=0;i<100;i++) res->cov_est[i] = 0;

		float dt_pow_4 = arg->var_accel*(*arg->dt * *arg->dt * *arg->dt * *arg->dt);
		float dt_pow_2 = arg->var_accel*(*arg->dt * *arg->dt);

		res->cov_est[0] = dt_pow_4;
		res->cov_est[11] = dt_pow_4;
		res->cov_est[22] = dt_pow_4;
		res->cov_est[33] = dt_pow_2;
		res->cov_est[44] = dt_pow_2;
		res->cov_est[55] = dt_pow_2;

		// Setup q-noise section of matrix
		for (uint8_t i=0;i<4;i++) {
			for (uint8_t j=0;j<4;j++) {
				res->cov_est[66 + i*10 + j] = gyro_noise[i]*gyro_noise[j];
			}
		}

		// Compute state_jac 
		// TODO Find alternative to this - will consume ~0.4kb of extra RAM, a lot of which is redundant
		float state_jac[100];
		for (uint8_t i=0;i<10;i++) {
			for (uint8_t j=0;j<10;j++) {
				if (i==j) {
					state_jac[i*10+j] = 1;
				} else {
					state_jac[i*10+j] = 0;
				}
			}
		}

		state_jac[3] =  *arg->dt;
		state_jac[14] = *arg->dt;
		state_jac[25] = *arg->dt;

		state_jac[36] = *arg->dt*(((arg->a[0]*(2*arg->q[0]))+(arg->a[1]*(2*arg->q[3])))-(arg->a[2]*(2*arg->q[2])));
		state_jac[37] = *arg->dt*(((arg->a[0]*(2*arg->q[1]))+(arg->a[1]*(2*arg->q[2])))+(arg->a[2]*(2*arg->q[3])));
		state_jac[38] = *arg->dt*(((arg->a[1]*(2*arg->q[1]))-(arg->a[0]*(2*arg->q[2])))-(arg->a[2]*(2*arg->q[0])));
		state_jac[39] = *arg->dt*(((arg->a[1]*(2*arg->q[0]))-(arg->a[0]*(2*arg->q[3])))+(arg->a[2]*(2*arg->q[1])));

		// TODO Is this the correct index
 		state_jac[46] = *arg->dt*(((arg->a[1]*(2*arg->q[0]))-(arg->a[0]*(2*arg->q[3])))+(arg->a[2]*(2*arg->q[1])));
		state_jac[47] = *arg->dt*(((arg->a[0]*(2*arg->q[2]))-(arg->a[1]*(2*arg->q[1])))+(arg->a[2]*(2*arg->q[0])));
		state_jac[48] = *arg->dt*(((arg->a[0]*(2*arg->q[1]))+(arg->a[1]*(2*arg->q[2])))+(arg->a[2]*(2*arg->q[3])));
		state_jac[49] = *arg->dt*((arg->a[2]*(2*arg->q[2]))-((arg->a[0]*(2*arg->q[0]))+(arg->a[1]*(2*arg->q[3]))));

		state_jac[56] = *arg->dt*(((arg->a[0]*(2*arg->q[2]))-(arg->a[1]*(2*arg->q[1])))+(arg->a[2]*(2*arg->q[0])));
		state_jac[57] = *arg->dt*(((arg->a[0]*(2*arg->q[3]))-(arg->a[1]*(2*arg->q[0])))-(arg->a[2]*(2*arg->q[1])));
		state_jac[58] = *arg->dt*(((arg->a[0]*(2*arg->q[0]))+(arg->a[1]*(2*arg->q[3])))-(arg->a[2]*(2*arg->q[2])));
		state_jac[59] = *arg->dt*(((arg->a[0]*(2*arg->q[1]))+(arg->a[1]*(2*arg->q[2])))+(arg->a[2]*(2*arg->q[3])));

		state_jac[67] = -(*arg->dt/2.0f)*arg->w[0];
		state_jac[68] = -(*arg->dt/2.0f)*arg->w[1];
		state_jac[69] = -(*arg->dt/2.0f)*arg->w[2];

		state_jac[76] = (*arg->dt/2.0f)*arg->w[0];
		state_jac[78] = (*arg->dt/2.0f)*arg->w[2];
		state_jac[79] = -(*arg->dt/2.0f)*arg->w[1];

		state_jac[86] = (*arg->dt/2.0f)*arg->w[1];
		state_jac[87] = -(*arg->dt/2.0f)*arg->w[2];
		state_jac[89] = (*arg->dt/2.0f)*arg->w[0];

		state_jac[96] = (*arg->dt/2.0f)*arg->w[2];
		state_jac[97] = (*arg->dt/2.0f)*arg->w[1];
		state_jac[98] = -(*arg->dt/2.0f)*arg->w[0];

		// TODO Add state_jac @ p @ state_jac.T to cov_est
		
	




	}

}




