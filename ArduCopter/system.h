#pragma once

#include <stdint.h>

#include <AP_HAL.h>

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



