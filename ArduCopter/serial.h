// For printing serial information to console

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_InertialSensor.h>
#include <AP_Math.h>

#include "system.h"

#include <AP_Common.h>

extern AP_HAL::BetterStream* cliSerial;

extern AP_InertialSensor_MPU6000 ins;

extern struct Location current_loc;
extern float cos_roll_x;
extern float cos_pitch_x;
extern float cos_yaw;


/* @brief Prints information about current location
*/
void print_GPS();

/* @brief Prints the current roll/pitch/yaw values
*/
void print_RPY();

/* @brief Prints the current imu roll and accel rates
*/
void print_roll_rates_and_accel();


/*

We want to be able to log the following sensor measurements and state variables

Raw measurements
1. Compass (Magnetomer)
2. IMU (note: technically the roll rates are state variables and not raw measurements)
3. GPS (skip)
4. Barometer
5. Battery Voltage/Current (skip)
6. RC In and Out

State Variables
Location and attitude (from AHRS)
Flight states and modes
PID Values
EKF Gains and states

Performance and Timing

************************************************

1. Compass
  Mag offsets 			: Vector3f off = compass.get_offsets(0)
  Mag motor offsets : Vector3f m_off = compass.get_motor_offsets(0)
	Mag 							: Vector3f mg = compass.get_field(0);
	Declination 			: float dec = compass.get_declination();

2. IMU
	Roll rates (rad/s)    : Vector3f omega = ins.get_gyros()
	Gyro offsets (rad/s)  : Vector3f off = ins.get_gyro_offsets()
	Acceleration (m/s/s)  : Vector3f acc = ins.get_accel()
	Accel offsets (m/s/s) : Vector3f a_off = ins.get_accel_offsets()
	Accel scale 					: Vector3f a_scale = ins.get_accel_scale()

3. GPS
4. Barometer
5. Current/Voltage
  battery_voltage     : (int16_t) (battery.voltage() * 100.0f),
  current_amps        : (int16_t) (battery.current_amps() * 100.0f),
  board_voltage       : board_voltage(),
  current_total       : battery.current_total_mah()

6. RC In and Out
	Roll Input: g.rc_1.control_in
	Pitch Input: g.rc_2.control_in
	Throttle Input: g.rc_3.control_in
	Yaw Input: g.rc_4.control_in 

	(add auxiliary inputs)



*/

