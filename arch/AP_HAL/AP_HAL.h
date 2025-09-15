
#ifndef __AP_HAL_H__
#define __AP_HAL_H__

#include <stdint.h>
#include <stdbool.h>

#include <AP_HAL/AP_HAL_Namespace.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>

/* HAL Module Classes (all pure virtual) */
#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/I2CDriver.h>
#include <AP_HAL/SPIDriver.h>
#include <AP_HAL/AnalogIn.h>
#include <AP_HAL/Storage.h>
#include <AP_HAL/GPIO.h>
#include <AP_HAL/RCInput.h>
#include <AP_HAL/RCOutput.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Util.h>

#include <AP_HAL/Sim.h>

#include <AP_HAL/utility/Print.h>
#include <AP_HAL/utility/Stream.h>
#include <AP_HAL/utility/BetterStream.h>

/* HAL Class definition */
#include <AP_HAL/HAL.h>

#endif // __AP_HAL_H__

