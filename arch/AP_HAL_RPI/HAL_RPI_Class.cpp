#include <AP_HAL.h>

/* TODO Update this to RPI */
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "HAL_RPI_Class.h"
#include "AP_HAL_RPI_Private.h"

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

using namespace RPI;

// Single UART for the Raspberry PI 2 Model B
static RPIUARTDriver uartADriver(true);
//static LinuxUARTDriver uartBDriver(false);
//static LinuxUARTDriver uartCDriver(false);

static LinuxSemaphore  i2cSemaphore;
static RPII2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static RPISPIDeviceManager spiDeviceManager;
static RPIAnalogIn analogIn;
static LinuxStorage storageDriver;
static RPIGPIO gpioDriver;
static RPIRCInput rcinDriver;
static RPIRCOutput rcoutDriver;
static LinuxScheduler schedulerInstance;
static LinuxUtil utilInstance;

HAL_RPI::HAL_RPI() :
    AP_HAL::HAL(
        &uartADriver,
        NULL, /*&uartBDriver,*/
        NULL, /*&uartCDriver,*/
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
{
}

/* TODO Method to setup pigpio interface and object */

void HAL_RPI::init(int argc,char* const argv[]) const 
{
	// Ignore CLAs

    scheduler->init(NULL);
    uartA->begin(115200);
    i2c->begin();
    spi->init(NULL);
}

const HAL_RPI AP_HAL_RPI;

#endif
