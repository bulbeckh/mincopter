#pragma once

#include <stdint.h>

/*
HASH include <AP_Common.h>
HASH include <AP_Math.h>
HASH include <AC_Fence.h>
HASH include "parameters.h"
HASH include "system.h"
HASH include "failsafe.h"
HASH include "config.h"
HASH include "util.h"
HASH include "attitude.h"
*/

// from arducopter.cpp
bool set_yaw_mode(uint8_t new_yaw_mode);
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode);

/* --- CONTROL MODES ------------------------------------------------------------------
*
*
* ---------------------------------------------------------------------------------- */

bool set_throttle_mode( uint8_t new_throttle_mode );
void update_throttle_mode(void);

bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode);
void update_roll_pitch_mode(void);

bool set_yaw_mode(uint8_t new_yaw_mode);
void update_yaw_mode(void);

// NOTE are these used?
void save_trim();
void auto_trim();




