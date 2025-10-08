
#include <AP_HAL/AP_HAL.h>

#include <arch/arm/stm32/HAL_STM32_Class.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>
#include <arch/arm/stm32/AP_HAL_STM32_Private.h>

static stm32::STM32Semaphore  i2cSemaphore;
static stm32::STM32Scheduler schedulerInstance;
static stm32::STM32Util utilInstance;

// TODO NOTE What is the bool arg here?
static stm32::STM32UARTDriver uartADriver(true);
static stm32::STM32UARTDriver uartBDriver(false);
static stm32::STM32UARTDriver uartCDriver(false);
static stm32::STM32UARTDriver uartDDriver(false);

static stm32::STM32I2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static stm32::STM32SPIDeviceManager spiDeviceManager;
static stm32::STM32AnalogIn analogIn;
static stm32::STM32GPIO gpioDriver;
static stm32::STM32RCInput rcinDriver;
static stm32::STM32RCOutput rcoutDriver;


HAL_STM32::HAL_STM32() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        NULL,   /* No Storage Driver for STM32 */
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
		NULL)  /* No Sim Driver for STM32 */
{}

void HAL_STM32::init(int argc,char* const argv[]) const 
{
	// No CLAs for now
	
    scheduler->init(NULL);
	
	// TODO Really, we should initialise the UART which was specified as the console/default UART rather than uartA explicitly
	// Start the console UART so we can log HAL initialisation messages from all other classes
    uartA->begin(115200);

    i2c->begin();

    spi->init(NULL);


}

// STM32 HAL Instance;
const HAL_STM32 AP_HAL_STM32;


