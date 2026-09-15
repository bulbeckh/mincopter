
#pragma once

/*
 * AP_HAL_Generic is the target HAL for use during SITL (simulation) at any level (level 0,1,3).
 *
 * The peripheral drivers are skeleton drivers and do not have any actual functionality.
 */

#ifdef HAL_BOARD_GENERIC

#include <arch/linux/generic/HAL_Generic_Class.h>
#include <arch/linux/generic/AP_HAL_Generic_Main.h>

#endif // CONFIG_HAL_BOARD



