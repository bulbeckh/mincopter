
#pragma once

#include <AP_HAL/AP_HAL.h>

#include <arch/linux/rpi/AP_HAL_RPI_Namespace.h>

class HAL_RPI : public AP_HAL::HAL {
public:
    HAL_RPI();
    void init(int argc, char * const * argv) const;
};

extern const HAL_RPI AP_HAL_RPI;



