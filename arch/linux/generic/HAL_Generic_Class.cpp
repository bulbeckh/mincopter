
#include <AP_HAL/AP_HAL.h>

#include <arch/linux/generic/HAL_Generic_Class.h>
#include <arch/linux/generic/AP_HAL_Generic_Private.h>

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

//using namespace Linux;

// TODO For generic linux (and maybe embedded linux too), one UART should redirect to stdout or some other
// device file and the rest should be NULL, or we have separate device files for each of the 4 UARTs

// 3 serial ports on Linux for now
namespace generic {
	static GenericUARTDriver uartADriver(true);
	static GenericUARTDriver uartBDriver(false);
	static GenericUARTDriver uartCDriver(false);
	static GenericI2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
	static GenericSPIDeviceManager spiDeviceManager;
	static GenericAnalogIn analogIn;
	static GenericGPIO gpioDriver;
	static GenericRCInput rcinDriver;
	static GenericRCOutput rcoutDriver;
}

namespace linux {
	static LinuxSemaphore  i2cSemaphore;
	static LinuxStorage storageDriver;
	static LinuxScheduler schedulerInstance;
	static LinuxUtil utilInstance;
}

HAL_Generic::HAL_Generic() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        NULL,            /* no uartD */
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}

void HAL_Generic::init(int argc,char* const argv[]) const 
{
	// No CLAs for now
	
    scheduler->init(NULL);
    uartA->begin(115200);
    i2c->begin();
    spi->init(NULL);
}

const HAL_Generic AP_HAL_Generic;



