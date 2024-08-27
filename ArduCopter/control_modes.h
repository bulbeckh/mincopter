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

// control_modes.h
void read_control_switch();
uint8_t readSwitch(void);
void reset_control_switch();
uint8_t read_3pos_switch(int16_t radio_in);
void read_aux_switches();
void init_aux_switches();
void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);
void save_trim();
void auto_trim();

