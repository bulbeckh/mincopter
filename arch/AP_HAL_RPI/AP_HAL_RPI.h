
#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL.h>


#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "HAL_RPI_Class.h"
#include "AP_HAL_RPI_Main.h"

#endif // CONFIG_HAL_BOARD

