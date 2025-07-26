
#pragma once

#include <AP_HAL.h>
#include "AP_HAL_RPI_Namespace.h"

class RPI::LinuxUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
};

