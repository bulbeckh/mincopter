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
bool set_throttle_mode( uint8_t new_throttle_mode );

extern const struct LogStructure log_structure[] PROGMEM;
extern DataFlash_APM2 DataFlash;

extern AP_GPS_Auto    g_gps_driver;

extern uint8_t receiver_rssi;
extern AP_HAL::AnalogSource* rssi_analog_source;

void load_parameters(void);

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
extern const AP_InertialSensor::Sample_rate ins_sample_rate;

/* @brief Gets board voltage
* @returns Board voltage
*/
uint16_t board_voltage(void);

/* @brief Initialises ardupilot on startup. Called during setup function (HAL function) right before scheduler is started.
*/
void init_ardupilot();

/* @brief Sets the control_mode (e.g. ALT_HOLD, STABILIZE, etc.)
* @param mode The control mode to be set.
*/
bool set_mode(uint8_t mode);

/* @brief Initialises ahrs and gyros (ins)
* @param force_gyro_cal Whether or not to use COLD or WARM start for INS
*/
void startup_ground(bool force_gyro_cal);


/* @brief Checks if GPS is ok
* @returns True if GPS is ok otherwise false
*/
bool GPS_ok();

void update_auto_armed();
uint32_t map_baudrate(int8_t rate, uint32_t default_baud);
void check_usb_mux(void);
void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);


