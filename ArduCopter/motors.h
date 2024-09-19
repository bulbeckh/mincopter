#pragma once

// motors.h

#include <stdint.h>

#include "parameters.h"
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <AP_InertialNav.h>
#include <AP_InertialSensor.h>
#include <GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Compass.h>
#include <AC_Fence.h>
#include <AC_WPNav.h>
#include <AP_Baro.h>
#include <DataFlash.h>

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"
#include "failsafe.h"
#include "log.h"
#include "navigation.h"

#include "system.h"
#include "radio.h"


// system.cpp
bool set_mode(uint8_t mode);

extern AP_Baro_MS5611 barometer;


void read_radio(void);
void start_logging(void);
void init_simple_bearing(void);

extern int32_t initial_armed_bearing;

void reset_I_all(void);

// util.h
void init_home();

uint16_t board_voltage(void);

extern uint8_t auto_trim_counter;

extern DataFlash_APM2 DataFlash;
extern AP_InertialSensor_MPU6000 ins;

extern Parameters g;
extern Vector3f omega;
extern AP_AHRS_DCM ahrs;
extern float G_Dt;
extern AP_MotorsQuad motors;
extern AP_InertialNav inertial_nav;
extern GPS         *g_gps;
extern GPS_Glitch   gps_glitch;
extern AP_Compass_HMC5843 compass;
extern AC_WPNav wp_nav;

extern AP_UNION_T ap;
extern AP_FAILSAFE_T failsafe;
extern AC_Fence fence;


/* @brief Checks for sustained pilot input required to arm/disarm motor (throttle down and stick to right). Function call is
* scheduled at 10hz via scheduler
*/
void arm_motors_check();

/* @brief Arms motors. Starts logging, enables output to motors, and a few other functions
*/
void init_arm_motors();

/* @brief Disarms motors.
*/
void init_disarm_motors();

/* @brief Disarms motors if copter has been stationary on ground for 15sec. Called at 1hz by 1hz_loop
*/
void auto_disarm_check(); 

/* @brief Checks run before motors are armed
*/
void pre_arm_checks(bool display_failure);
void pre_arm_rc_checks();
bool pre_arm_gps_checks(bool display_failure);
bool arm_checks(bool display_failure);

/* @brief Send output to motors via motors.output(). Called during fast_loop.
*/
void set_servos_4();

// NOTE not used - can remove
/* @brief Writes value to a channel
*/
void servo_write(uint8_t ch, uint16_t pwm);

