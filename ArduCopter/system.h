#pragma once

#include <stdint.h>
#include <AP_Menu.h>
#include <AP_HAL.h>

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
#include <AP_BoardConfig.h>
#include <AP_Relay.h>
#include <AP_HAL.h>
#include <DataFlash.h>
#include <AP_GPS.h>
#include <AP_Progmem.h>

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"
#include "log.h"
#include "config.h"
#include "control_modes.h"


extern AP_HAL::AnalogSource* board_vcc_analog_source;
uint8_t get_wp_yaw_mode(bool rtl);
bool set_throttle_mode( uint8_t new_throttle_mode );

extern const struct LogStructure log_structure[] PROGMEM;

extern AP_GPS_Auto    g_gps_driver;

extern uint8_t receiver_rssi;
extern AP_HAL::AnalogSource* rssi_analog_source;

void load_parameters(void);

extern AP_Relay relay;
extern AP_BoardConfig BoardConfig;

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
int8_t   process_logs(uint8_t argc, const Menu::arg *argv);      // in Log.pde
int8_t   setup_mode(uint8_t argc, const Menu::arg *argv);        // in setup.pde
int8_t   test_mode(uint8_t argc, const Menu::arg *argv);         // in test.cpp

int8_t   main_menu_help(uint8_t argc, const Menu::arg *argv);

uint16_t board_voltage(void);

int8_t reboot_board(uint8_t argc, const Menu::arg *argv);
void run_cli(AP_HAL::UARTDriver *port);
void init_ardupilot();
void startup_ground(bool force_gyro_cal);
bool GPS_ok();
bool mode_requires_GPS(uint8_t mode);
bool manual_flight_mode(uint8_t mode);
bool set_mode(uint8_t mode);
void update_auto_armed();
uint32_t map_baudrate(int8_t rate, uint32_t default_baud);
void check_usb_mux(void);
void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);

extern const AP_InertialSensor::Sample_rate ins_sample_rate;

