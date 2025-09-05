
#pragma once

/* Full 10-state EKF implementation of state estimation
 *
 * Position (3-state)
 * Velocity (3-state)
 * Attitude (4-state quaternion)
 *
 * We integrate accelerometer readings for position/velocity estimation and gyrometer
 * readings for attitude estimation.
 *
 * Correction is done using GPS for position/velocity and magnetometer & accelerometer
 * reference vectors for the attitude correction.
 *
 * All EKF inputs should be expressed in an ENU frame. Similarly, the output state
 * vector is expressed in an ENU frame.
 *
 *
 * Part 1. Predict
 * This function returns a predicted state (float[10]) and predicted covariance matrix (float[10][10])
 *
 * Part 2. Correct
 * This function returns a measurement corrected state matrix and covariance matrix
 *
 */

#include <AP_Math.h>

#include <ahrs_interface.h>
#include <inav_interface.h>

#define EKF_DATA_TYPE float

// TODO Why do we have 'AP' for AHRS but 'MC' for InertialNav

class EKF : public AP_AHRS, public MC_InertialNav {
	public:
		EKF() : AP_AHRS(), MC_InertialNav() { }

	private:
		/* @brief Prepare correct and predict arguments vectors */
		void setup_ekf_args(void);

		/* @brief Internal reset method */
		void reset(void);

	private:
		/* Our casadi functions use the following interface:
		 *
		 *
		 *
		 */

		// ekf arg/res
		EKF_DATA_TYPE dt;
		EKF_DATA_TYPE w[3];
		EKF_DATA_TYPE a[3];
		EKF_DATA_TYPE m[3];
		EKF_DATA_TYPE var_gyro;
		EKF_DATA_TYPE var_accel;
		EKF_DATA_TYPE var_mag;

		// NOTE Covariance matrix (10,10)
		EKF_DATA_TYPE cov[100];
		EKF_DATA_TYPE q[4];
		EKF_DATA_TYPE x[3];
		EKF_DATA_TYPE v[3];

		// NOTE TODO These are duplicates with the above state - change into a single struct/union so that one can be re-used
		EKF_DATA_TYPE state_out[10];
		EKF_DATA_TYPE cov_out[100];

		// NOTE In very constrained system, we could point this directly to the memory location of current GPS position and velocity
		EKF_DATA_TYPE gps_pos[3];
		EKF_DATA_TYPE gps_vel[3];
		EKF_DATA_TYPE var_gps_pos;
		EKF_DATA_TYPE var_gps_vel;

		EKF_DATA_TYPE state_est[10];
		EKF_DATA_TYPE cov_est[100];

		EKF_DATA_TYPE* ekf_predict_arg[9] = {
			&dt,
			w,
			a,
			&var_gyro,
			&var_accel,
			cov, 
			q,
			x,
			v};

		EKF_DATA_TYPE* ekf_predict_res[2] = {state_est, cov_est};

		EKF_DATA_TYPE* ekf_correct_arg[10] = {
			state_est, // Re-use state_est and cov_est from ekf_predict
			cov_est,
			a,
			&var_accel,
			m,
			&var_mag,
			gps_pos,
			gps_vel,
			&var_gps_pos,
			&var_gps_vel};

		// vt is (12,1) and kgain is (10,12)
		EKF_DATA_TYPE vt[12];
		EKF_DATA_TYPE kgain[120];

		EKF_DATA_TYPE* ekf_correct_res[4] = {state_out, cov_out, vt, kgain};

	public:
		// Implementation of AP_AHRS methods
		
		/* @brief Initialise the EKF orientation portion */
		void _ahrs_init_internal(void) override;

		/* @brief Typically this calls the ahrs portion of the update but all updates for EKF are handled by the call to inav_update */
		void ahrs_update(void) override;

		// Implementation of AP_InertialNav methods

		/* @brief Initialise the INS (position/velocity) portion of the EKF, ideally using the latest accelerometer reading */
		void _inav_init_internal(void) override;

		/* @brief Update the INS using latest accelerometer readings */
		void inav_update(void) override;

		/* @brief Overrides of the reset methods */
		// TODO


};


