

/* Implementation of MinCopter State Algorithms */

#include "mcstate.h"
#include "mcinstance.h"

extern MCInstance mincopter;

#include "util.h"

// TODO This is now incorrect as it is potentially created and initialising two EKF instances. Change to pointers instead

MCState::MCState(void) { }

void MCState::init(void)
{
	// Pass the _state variable into the ahrs and inertial_nav
	
	// TODO Check return value
	ahrs.ahrs_init(&_state);
	inertial_nav.inav_init(&_state);

	return;
}

void MCState::update(void)
{

	/* If we are using the EKF, then we run the full update using a call to the inertial_nav and ignore the ahrs update method */
#ifndef MC_AHRS_EKF
	ahrs.ahrs_update();
#endif

	// TODO Change the function prototype to contain NO dt parameter - this should be taken from accelerometer/gyrometer elasped time
	inertial_nav.inav_update();

	// TODO Check if we are using a drift/bias compensation state in our model and then compensate direct sensor readings
	//_omega = mincopter.ins.get_gyro();
	//_accel = mincopter.ins.get_accel();

	// TODO I don't think this needs to be called
	update_trig();

	// TODO Update current_loc variable

	// TODO Should this return something to indicate successful update
	return;
}

const Vector3f& MCState::get_euler_angles(void)
{
	_state._attitude.to_euler(
			&_euler.x,
			&_euler.y,
			&_euler.z
	);

	return _euler;
}

const Matrix3f& MCState::get_dcm(void)
{
	/* TODO Compute DCM matrix from quaternion */
	return _dcm;
}

// TODO REMOVE
void MCState::update_trig(void){
		Vector2f yawvector;
		// TODO add this-> infront of class members. more verbose is better
		const Matrix3f &temp   = get_dcm();

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
	
	// Update the roll,pitch,yaw sensor values
	_euler = get_euler_angles();

	// TODO Remove the use of <roll,pitch,yaw>_sensor in controller PID - replace with 
	// TODO Does get_euler_angles express in degrees or radians?
	// Euler angles as int32 (degc*100)
	roll_sensor  = (int32_t)(_euler.x*100);
	pitch_sensor = (int32_t)(_euler.y*100);
	yaw_sensor   = (int32_t)(_euler.z*100);
	
	return;
}

void MCState::set_altitude(float new_alt)
{
	// TODO Implement
	return;

}

void MCState::set_home_position(int32_t lat, int32_t lng)
{
	// TODO Implement
	return;
}

bool MCState::position_ok(void) const
{
	// TODO Implement
	return false;
}

int32_t MCState::get_latitude() const
{
	// TODO Implement
	return 0;
}

int32_t MCState::get_longitude() const
{
	// TODO Implement
	return 0;
}

float MCState::get_altitude() const
{
	// TODO Implement
	return 0.0f;
}

const Vector3f MCState::get_position() const
{
	// TODO Change this to a reference
	return Vector3f(
			_state._position[0],
			_state._position[1],
			_state._position[2]);
}

const Vector3f MCState::get_velocity() const
{
	return Vector3f(
			_state._velocity[0],
			_state._velocity[1],
			_state._velocity[2]);
}
