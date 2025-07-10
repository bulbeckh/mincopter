
#pragma once

#include "ahrs_interface.h"

/* Simulated AHRS with state obtained directly from the simulation */

class AHRS_sim : public AP_AHRS
{
public:
    // Constructors
    AHRS_sim(AP_InertialSensor &ins, GPS *&gps) :
        AP_AHRS(ins, gps)
    {
    }

    // return the smoothed gyro vector corrected for drift
    const Vector3f get_gyro(void) const;

    // return rotation matrix representing rotaton from body to earth axes
    const Matrix3f &get_dcm_matrix(void) const {
        return _body_dcm_matrix;
    }

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const {
        return Vector3f(0,0,0);
    }

    // Methods
    void            update(void);
    void            reset(bool recover_eulers = false);

    // reset the current attitude, used on new IMU calibration
    void reset_attitude(const float &roll, const float &pitch, const float &yaw);

    // status reporting
    float get_error_rp(void) {
		return 0.0f;
	}

    float get_error_yaw(void) {
		return 0.0f;
	}

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate(void) {
        return Vector3f(0,0,0);
    }

	// TODO What does this do? Not used anywhere so REMOVE
    bool use_compass(void) {
		return true;
	}

	private:
		Matrix3f _body_dcm_matrix;

};


