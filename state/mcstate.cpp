

/* Implementation of MinCopter State Algorithms */

#include "mcstate.h"
#include "mcinstance.h"

extern MCInstance mincopter;

#include "util.h"
#include "log.h"
#include "system.h"

MCState::MCState() : 
		ahrs(mincopter.ins, mincopter.g_gps),
		//fence(&this->inertial_nav),
		inertial_nav(&this->ahrs, &mincopter.barometer, mincopter.g_gps, mincopter.gps_glitch),
		//wp_nav(&this->inertial_nav, &this->ahrs, &mincopter.pi_loiter_lat, &mincopter.pi_loiter_lon, &mincopter.pid_loiter_rate_lat, &mincopter.pid_loiter_rate_lon)
{

}

void MCState::read_AHRS(void)
{
		// Perform IMU calculations and get attitude info
		this->ahrs.update();
		this->omega = mincopter.ins.get_gyro();
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

		// update wp_nav controller with trig values
		wp_nav.set_cos_sin_yaw(cos_yaw, sin_yaw, cos_pitch_x);

		//flat:
		// 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
		// 90° = cos_yaw:  0.00, sin_yaw:  1.00,
		// 180 = cos_yaw: -1.00, sin_yaw:  0.00,
		// 270 = cos_yaw:  0.00, sin_yaw: -1.00,
}


