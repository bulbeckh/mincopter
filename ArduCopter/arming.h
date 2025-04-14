#pragma once

#include <stdint.h>

/* @brief Checks for sustained pilot input required to arm/disarm motor (throttle down and stick to right). Function call is
* scheduled at 10hz via scheduler
*/
void arm_motors_check();

/* @brief Arms motors. Starts logging, enables output to motors, and a few other functions
*/
void init_arm_motors();

/* @brief Disarms motors.
*/
void init_disarm_motors();

/* @brief Disarms motors if copter has been stationary on ground for 15sec. Called at 1hz by 1hz_loop
*/
void auto_disarm_check(); 

/* @brief Checks run before motors are armed
*/
void pre_arm_checks(bool display_failure);
void pre_arm_rc_checks();
bool pre_arm_gps_checks(bool display_failure);
bool arm_checks(bool display_failure);

// NOTE not used - can remove
/* @brief Writes value to a channel
*/
void servo_write(uint8_t ch, uint16_t pwm);

