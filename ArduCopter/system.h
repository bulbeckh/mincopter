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
#include <AP_HAL.h>
#include <DataFlash.h>
#include <AP_GPS.h>
#include <AP_Progmem.h>

// TODO Remove all of these libs
#include "navigation.h"
#include "util.h"
#include "motors.h"
#include "log.h"
#include "config.h"
#include "control_modes.h"
#include "failsafe.h"


// 
//void load_parameters(void);

// system.cpp
bool set_mode(uint8_t mode);

// TODO what is this?
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


