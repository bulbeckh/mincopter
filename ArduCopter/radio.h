#pragma once

// radio.h

#include <stdint.h>

#include "parameters.h"
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <AP_InertialNav.h>
#include <GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Compass.h>
#include <AC_Fence.h>
#include <AC_WPNav.h>
#include <AP_RCMapper.h>
#include <AP_HAL.h>

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"

// system.cpp
bool set_mode(uint8_t mode);

extern AP_HAL::BetterStream* cliSerial;

extern RCMapper rcmap;

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

/* @brief Setup default dead zones for each radio channel
*/
void default_dead_zones();

/* @brief Called during init_rc_out
*/
void init_esc();

/* @brief Initialise radio input channels, and outputs (motors, esc)
*/
void init_rc_in();
void init_rc_out();

/* @brief Enable and output lowest possible value to motors
*/
void output_min();

/* @brief Reads radio PWM and assigns values to each g.rc_<x>. Run at 100hz during fast_loop
*/
void read_radio();

/* @brief Sets throttle PWM signal during call to read_radio
* @param throttle_pwm The throttle PWM signal during 
*/
void set_throttle_and_failsafe(uint16_t throttle_pwm);

// ?
void trim_radio();
void aux_servos_update_fn();


