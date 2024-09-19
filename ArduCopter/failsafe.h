#pragma once

#include <stdint.h>

#include "util.h"
#include "motors.h"

extern uint16_t mainLoop_count;

/* @brief Helper functions for enabling and disabling failsafe
*/
void failsafe_enable();
void failsafe_disable();

/* @brief Tracks failsafe status by modifying in_failsafe parameter. Registered to run by scheduler during init_autopilot
*/
void failsafe_check();



