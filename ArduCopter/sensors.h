#pragma once

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
#include <AP_BattMonitor.h>
#include <AP_HAL.h>

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"

extern uint8_t receiver_rssi;
extern AP_HAL::AnalogSource* rssi_analog_source;

// system.cpp
bool set_mode(uint8_t mode);

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

extern AP_BattMonitor battery;

// sensors.h
void init_sonar(void);
void init_barometer(bool full_calibration);
int32_t read_barometer(void);
int16_t read_sonar(void);
void init_compass();
void init_optflow();
void read_battery(void);
void read_receiver_rssi(void);




