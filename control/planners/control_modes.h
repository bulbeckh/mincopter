#pragma once

#include <stdint.h>

/* --- CONTROL MODES ------------------------------------------------------------------
* The `set<throttle,roll_pitch,yaw>_mode functions are called by the `set_mode`
* function in `arducopter.cpp`.
*
* The `update_<throttle,roll_pitch,yaw>_mode functions are called in the fast_loop to
* trigger the generation of the control outputs.
* ---------------------------------------------------------------------------------- */

/* @brief Sets throttle mode
*/
bool set_throttle_mode( uint8_t new_throttle_mode );

/* @brief Sets the roll and pitch modes
*/
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode);

/* @brief Sets the yaw mode
*/
bool set_yaw_mode(uint8_t new_yaw_mode);




