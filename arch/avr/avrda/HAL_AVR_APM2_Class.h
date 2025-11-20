
#pragma once

#include <AP_HAL.h>

#include <AP_HAL_AVR.h>
#include "AP_HAL_AVR_Namespace.h"

/**
 * HAL_AVR_APM2 class derives from HAL but provides an AVR-specific
 * init method.
 */

class HAL_AVR_APM2 : public AP_HAL::HAL {
public:
    HAL_AVR_APM2();
    void init(int argc, char * const argv[]) const;
};

/**
 * Static instance exported here, defined in the Class.cpp file
 */
extern const HAL_AVR_APM2 AP_HAL_AVR_APM2;


