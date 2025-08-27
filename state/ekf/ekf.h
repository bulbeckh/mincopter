
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

// TODO Why do we have 'AP' for AHRS but 'MC' for InertialNav

class EKF : public AP_AHRS, public MC_InertialNav {
	public:
		EKF() {}

	private:
		/* @brief Run estimation part of EKF algorithm
		 * @param dt The elapsed time (in seconds) since last the last EKF prediction */
		void predict(float dt);

		/* @brief Correct state position and velocity estimates with GPS reading
		 * @param gps_position Vector3f with latest gps position measurement
		 * @param gps_velocity Vector3f with latest gps velocity measurement */
		void correct_pos_vel(Vector3f gps_position, Vector3f gps_velocity);

		/* @brief Correct state attitude with magnetometer and accelerometer (gravity) readings
		 * @param mag_reading Latest magnetometer measurement
		 * @param accelerometer_reading Latest accelerometer measurement */
		void correct_attitude(Vector3f mag_reading, Vector3f accelerometer_reading);

	private:
		/* Our casadi functions use the following interface:
		 *
		 *
		 *
		 */

		// ekf arg/res
		double dt;
		double w[3];
		double a[3];
		double m[3];
		double var_gyro;
		double var_accel;
		double var_mag;

		// NOTE Is this type of indexing correct here or should we create a truly continugous array rather than pointer->pointer?
		double cov[10][10];
		double q[4];
		double x[3];
		double v[3];

		// NOTE TODO These are duplicates with the above state - change into a single struct/union so that one can be re-used
		double state_out[10];
		double cov_out[10][10];

		// NOTE In very constrained system, we could point this directly to the memory location of current GPS position and velocity
		double gps_pos[3];
		double gps_vel[3];
		double var_gps_pos;
		double var_gps_vel;

		double state_est[10];
		double cov_est[10][10];

		double ekf_predict_arg[] = {
			&dt,
			w,
			a,
			&var_gyro,
			&var_accel,
			cov, 
			q,
			x,
			v};

		double ekf_predict_res[] = {state_est, cov_est};

		double ekf_correct_arg[] = {
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

		double vt[10][12];
		double kgain[10][10];

		double ekf_correct_res[] = {state_out, cov_out, vt, kgain};

		// TODO What units are each of these??
		// TODO Change to struct

		/* @brief Vector holding position estimate in ENU frame */
		Vector3f _position;

		/* @brief Vector holding velocity estimate in ENU frame */
		Vector3f _velocity;

		/* @brief Vector holding attitude quaternion in ENU frame */
		Quaternion _attitude;

	public:
		
		// TODO Find a better way of implementing INAV and AHRS in the same class - there are methods like 'update' 
		// which is confusing to know which method is being called. Additionaly, the EKF runs on separate predict
		// and correct functions which is different from how the INAV and AHRS are separated. Need to reconcile
		// them both somehow

		// Implementation of AP_AHRS methods
	
		/* @brief Update the EKF state estimate TODO How and when will the measurements be fused? */
		void update(void) override;

		/* @brief Returns the latest gyrometer reading TODO Why should this be in mcstate and not got directly from sensors? */
    	const Vector3f get_gyro(void) override;

		/* @brief Returns the estimate gyrometer drift TODO Not yet implemented in EKF */
    	const Vector3f &get_gyro_drift(void) const override;

		// TODO Should default kwargs be allowed?
		/* @brief Reset the entire state estimate */
    	void reset(bool recover_eulers=false) override;

		/* @brief Reset only the attitude estimate to the provided roll, pitch, and yaw euler angles */
    	void reset_attitude(const float &roll, const float &pitch, const float &yaw) override;

		// TODO Remove these from MCState representation
    	virtual float get_error_rp(void) = 0;
    	virtual float get_error_yaw(void) = 0;

		/* @brief Returns a DCM matrix representation of the current attitude in the ENU frame */
    	const Matrix3f &get_dcm_matrix(void) const override;


		// Implementation of AP_InertialNav methods

		/* @brief Initialise the INS (position/velocity) portion of the EKF, ideally using the latest accelerometer reading */
		void init(void) override;

		/* @brief Update the INS using latest accelerometer readings */
		void update(float dt) override;

		/* @brief Check if position reading is valid. Always true for simulation. */
		bool position_ok() const override;

		/* @brief Directly set altitude of the position state of EKF */
		void set_altitude(float new_alt) override;

		// TODO Should origin be tracked as part of MCState or in planner? Remove if needed
		virtual void set_home_position(int32_t lat, int32_t lng) = 0;



};


