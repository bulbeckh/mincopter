#pragma once

#include <stdint.h>
#include "ap_union.h"
#include <AP_Common.h>
#include <AP_Math.h>
#include <AC_Fence.h>

#include "parameters.h"
#include "system.h"

#include "config.h"
#include "util.h"

extern Parameters g;
extern AP_Int8 *flight_modes;

extern int8_t control_mode;
extern uint8_t oldSwitchPosition;

extern int8_t aux_switch_wp_index;

extern AP_UNION_T ap;
extern AP_FAILSAFE_T failsafe;
extern AC_Fence fence;

// from arducopter.cpp
bool set_yaw_mode(uint8_t new_yaw_mode);
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode);

extern uint8_t yaw_mode;
extern uint8_t roll_pitch_mode;
extern uint8_t throttle_mode;

extern uint8_t auto_trim_counter;

/* @brief Reads value of control mode switch. Called during fast_loop. Calls set_mode with new switch value.
*/
void read_control_switch();

/* @brief Helper function to return index based on rc PWM value
*/
uint8_t readSwitch(void);

/* @brief Re-runs read_control_switch but forcing an update
*/
void reset_control_switch();

/* @brief Converts auxiliary switch PWM to index
* @param radio_in The radio PWM signal
*/
uint8_t read_3pos_switch(int16_t radio_in);

/* @brief Read auxiliary switches. Channels 7 and 8 (g.rc_<7,8>)
* called at 10hz by scheduler
*/
void read_aux_switches();

/* @brief Initializes auxiliary functions. Called during init_autopilot
*/
void init_aux_switches();

/* @brief Run the correct auxiliary function. Executed during read_aux_switches
* @param ch_function Which auxiliary code to execute
* @param ch_flag The index identified by read_3pos_switch
*/
void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);

// NOTE are these used?
void save_trim();
void auto_trim();




