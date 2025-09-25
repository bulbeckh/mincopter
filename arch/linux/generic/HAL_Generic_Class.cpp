
#include <AP_HAL/AP_HAL.h>

#include <arch/linux/generic/HAL_Generic_Class.h>
#include <arch/linux/generic/AP_HAL_Generic_Private.h>

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

//using namespace Linux;

#define USE_LOG_FILE

// TODO For generic linux (and maybe embedded linux too), one UART should redirect to stdout or some other
// device file and the rest should be NULL, or we have separate device files for each of the 4 UARTs

static Linux::LinuxSemaphore  i2cSemaphore;
static Linux::LinuxStorage storageDriver;
static Linux::LinuxScheduler schedulerInstance;
static Linux::LinuxUtil utilInstance;

// 3 serial ports on Linux for now
static generic::GenericUARTDriver uartADriver(true);
static generic::GenericUARTDriver uartBDriver(false);
static generic::GenericUARTDriver uartCDriver(false);
static generic::GenericI2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static generic::GenericSPIDeviceManager spiDeviceManager;
static generic::GenericAnalogIn analogIn;
static generic::GenericGPIO gpioDriver;
static generic::GenericRCInput rcinDriver;
static generic::GenericRCOutput rcoutDriver;

static generic::GenericGZInterface simDriver;

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
        &utilInstance,
		&simDriver)
{}

void HAL_Generic::init(int argc,char* const argv[]) const 
{
	// No CLAs for now
	
    scheduler->init(NULL);
	
	// TODO Really, we should initialise the UART which was specified as the console/default UART rather than uartA explicitly
	// Start the console UART so we can log HAL initialisation messages from all other classes
    uartA->begin(115200);

    i2c->begin();

    spi->init(NULL);

	// Initialise simulation
	sim->setup_sim_socket();

	// Initialise logging over pipe
#if defined(USE_LOG_PIPE)
	sim->setup_log_source("/tmp/mincopter_log", AP_HAL::Sim::LogSource::PIPE);
#elif defined(USE_LOG_FILE)
	sim->setup_log_source("./mincopter_log.txt", AP_HAL::Sim::LogSource::LOGFILE);
#endif

}

const HAL_Generic AP_HAL_Generic;



