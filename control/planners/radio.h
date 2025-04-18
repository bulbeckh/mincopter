#pragma once

// radio.h

#include <stdint.h>

/* @brief Setup default dead zones for each radio channel
*/
void default_dead_zones();

/* @brief Called during init_rc_out
*/
void init_esc();

/* @brief Initialise radio input channels, and outputs (motors, esc)
*/
void init_rc_in();
void init_rc_out();

/* @brief Enable and output lowest possible value to motors
*/
void output_min();

/* @brief Sets throttle PWM signal during call to read_radio
* @param throttle_pwm The throttle PWM signal during 
*/
void set_throttle_and_failsafe(uint16_t throttle_pwm);

void aux_servos_update_fn();


