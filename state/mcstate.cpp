

/* Implementation of MinCopter State Algorithms */

#include "mcstate.h"
#include "mcinstance.h"

extern MCInstance mincopter;

#include "util.h"

// TODO This is now incorrect as it is potentially created and initialising two EKF instances. Change to pointers instead

MCState::MCState() : 
#if MC_AHRS_DCM
		ahrs(mincopter.ins, mincopter.g_gps),
#elif MC_AHRS_EKF
		ahrs(),
#endif

#if MC_INAV_DEFAULT
		inertial_nav(&this->ahrs, &mincopter.barometer, mincopter.g_gps, mincopter.gps_glitch)
#elif MC_INAV_EKF
		inertial_nav()
#endif
		//wp_nav(&this->inertial_nav, &this->ahrs, &mincopter.pi_loiter_lat, &mincopter.pi_loiter_lon, &mincopter.pid_loiter_rate_lat, &mincopter.pid_loiter_rate_lon)
		//fence(&this->inertial_nav),
{

}


void MCState::update(void)
{

	/* If we are using the EKF, then we run the full update using a call to the inertial_nav and ignore the ahrs update method */
#ifndef MC_AHRS_EKF
	ahrs->update();
#endif

	// TODO Change the function prototype to contain NO dt parameter - this should be taken from accelerometer/gyrometer elasped time
	inertial_nav->update(0.0f);

	// TODO Check if we are using a drift/bias compensation state in our model and then compensate direct sensor readings
	_omega = mincopter.ins.get_gyro();
	_accel = mincopter.ins.get_accel();

	// TODO I don't think this needs to be called
	update_trig();

	// TODO Update current_loc variable

	// TODO Should this return something to indicate successful update
	return;
}

void MCState::update_trig(void){
		Vector2f yawvector;
		// TODO add this-> infront of class members. more verbose is better
		const Matrix3f &temp   = ahrs.get_dcm_matrix();

		yawvector.x     = temp.a.x;     // sin
		yawvector.y     = temp.b.x;         // cos
		yawvector.normalize();

		cos_pitch_x     = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
		cos_roll_x      = temp.c.z / cos_pitch_x;                       // level = 1

		cos_pitch_x     = constrain_float(cos_pitch_x, 0, 1.0);
		// this relies on constrain_float() of infinity doing the right thing,
		// which it does do in avr-libc
		cos_roll_x      = constrain_float(cos_roll_x, -1.0, 1.0);

		sin_yaw         = constrain_float(yawvector.y, -1.0, 1.0);
		cos_yaw         = constrain_float(yawvector.x, -1.0, 1.0);

		// added to convert earth frame to body frame for rate controllers
		sin_pitch       = -temp.c.x;
		sin_roll        = temp.c.y / cos_pitch_x;

		// TODO If this is used by wp_nav then it needs to pull from here
		// update wp_nav controller with trig values
		//wp_nav.set_cos_sin_yaw(cos_yaw, sin_yaw, cos_pitch_x);

		//flat:
		// 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
		// 90° = cos_yaw:  0.00, sin_yaw:  1.00,
		// 180 = cos_yaw: -1.00, sin_yaw:  0.00,
		// 270 = cos_yaw:  0.00, sin_yaw: -1.00,
}


