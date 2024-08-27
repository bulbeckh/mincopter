#pragma once

// setup.h
#include <stdint.h>
#include <AP_Menu.h>
#include <AP_HAL.h>
#include <AP_Math.h>

#include "config.h"

#include "parameters.h"
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <AP_InertialNav.h>
#include <AP_InertialSensor.h>
#include <GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Compass.h>
#include <AC_Fence.h>
#include <AC_WPNav.h>

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"
#include "control_modes.h"

// system.cpp
bool set_mode(uint8_t mode);

extern AP_Int8 *flight_modes;
extern const AP_InertialSensor::Sample_rate ins_sample_rate;

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

extern AP_HAL::BetterStream* cliSerial;
extern const AP_HAL::HAL& hal;

int8_t   setup_accel_scale       (uint8_t argc, const Menu::arg *argv);
int8_t   setup_compass           (uint8_t argc, const Menu::arg *argv);
int8_t   setup_compassmot        (uint8_t argc, const Menu::arg *argv);
int8_t   setup_erase             (uint8_t argc, const Menu::arg *argv);
int8_t   setup_flightmodes       (uint8_t argc, const Menu::arg *argv);
int8_t   setup_optflow           (uint8_t argc, const Menu::arg *argv);
int8_t   setup_radio             (uint8_t argc, const Menu::arg *argv);
int8_t   setup_range             (uint8_t argc, const Menu::arg *argv);
int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
int8_t   setup_set               (uint8_t argc, const Menu::arg *argv);
int8_t   setup_show              (uint8_t argc, const Menu::arg *argv);
int8_t   setup_sonar             (uint8_t argc, const Menu::arg *argv);


int8_t setup_mode(uint8_t argc, const Menu::arg *argv);

void report_batt_monitor();
void report_sonar();
void report_frame();
void report_radio();
void report_ins();
void report_compass();
void report_flight_modes();
void report_optflow();
void print_radio_values();
void print_switch(uint8_t p, uint8_t m, bool b);
void print_done();
void zero_eeprom();
void print_accel_offsets_and_scaling(void);
void print_gyro_offsets(void);
void print_blanks(int16_t num);
void print_divider(void);
void print_enabled(bool b);
void init_esc();
void report_version();
void report_tuning();

void display_compassmot_info(Vector3f& motor_impact, Vector3f& motor_compensation);
