#pragma once

#include <stdint.h>

#include <AP_HAL.h>

/*
HASH include <AP_Menu.h>
HASH include "parameters.h"
HASH include <AP_Math.h>
HASH include <AP_AHRS.h>
HASH include <AP_Motors.h>
HASH include <AP_InertialNav.h>
HASH include <GPS.h>
HASH include <AP_GPS_Glitch.h>
HASH include <AP_Compass.h>
HASH include <AC_Fence.h>
HASH include <AC_WPNav.h>
HASH include <AP_BattMonitor.h>
HASH include <AP_HAL.h>
HASH include <DataFlash.h>
HASH include <AP_GPS.h>
HASH include <AP_Progmem.h>
HASH include "navigation.h"
HASH include "util.h"
HASH include "motors.h"
HASH include "log.h"
HASH include "config.h"
HASH include "control_modes.h"
HASH include "failsafe.h"
*/

// 
//void load_parameters(void);

// system.cpp
bool set_mode(uint8_t mode);

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


