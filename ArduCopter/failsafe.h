#pragma once

#include <stdint.h>

#include "util.h"
#include "motors.h"

extern uint16_t mainLoop_count;

// failsafe.h
void failsafe_enable();
void failsafe_disable();
void failsafe_check();


