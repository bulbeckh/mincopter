#include <AP_HAL/AP_HAL.h>

#include <arch/linux/rpi/HAL_RPI_Class.h>
#include <arch/linux/rpi/AP_HAL_RPI_Private.h>
#include <arch/linux/AP_HAL_Linux_Namespace.h>

#include <arch/linux/Scheduler.h>
#include <arch/linux/Util.h>
#include <arch/linux/Storage.h>
#include <arch/linux/Semaphores.h>

// HASH include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

//using namespace RPI;

// Single UART for the Raspberry PI 2 Model B
static RPI::RPIUARTDriver uartADriver(true);
//static LinuxUARTDriver uartBDriver(false);
//static LinuxUARTDriver uartCDriver(false);

static Linux::LinuxSemaphore  i2cSemaphore;
static RPI::RPII2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static RPI::RPISPIDeviceManager spiDeviceManager;
static RPI::RPIAnalogIn analogIn;
static Linux::LinuxStorage storageDriver;
static RPI::RPIGPIO gpioDriver;
static RPI::RPIRCInput rcinDriver;
static RPI::RPIRCOutput rcoutDriver;
static Linux::LinuxScheduler schedulerInstance;
static Linux::LinuxUtil utilInstance;

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

