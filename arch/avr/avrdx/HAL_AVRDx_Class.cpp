
#include <AP_HAL.h>

#include <avrdx/AP_HAL_AVRDx.h>
#include "avrdx/AP_HAL_AVRDx_private.h"

#include "avrdx/HAL_AVRDx_Class.h"

using namespace AP_HAL;
using namespace AP_HAL_AVRDx;


// NOTE These are macros to create the UART drivers and ISRs for each UART
// TODO Temporarily removed these
/*
AVRUARTDriverISRs(0);
AVRUARTDriverISRs(1);
AVRUARTDriverISRs(2);

AVRUARTDriverInstance(avrUart0Driver, 0);
AVRUARTDriverInstance(avrUart1Driver, 1);
AVRUARTDriverInstance(avrUart2Driver, 2);
*/

// Instances of each HAL object
static AVRDxSemaphore     i2cSemaphore;
static AVRDxI2CDriver     avrI2CDriver(&i2cSemaphore);
static AVRDxSPIDeviceManager apm2SPIDriver;
static AVRDxAnalogIn      avrAnalogIn;
static AVRDxEEPROMStorage avrEEPROMStorage;
static AVRDxGPIO          avrGPIO;
static AVRDxRCInput      apm2RCInput;
static AVRDxRCOutput     apm2RCOutput;
static AVRDxScheduler     avrScheduler;
static AVRDxUtil          avrUtil;

static ISRRegistry isrRegistry;

HAL_AVRDx::HAL_AVRDx(void) :
    AP_HAL::HAL(
        NULL, /* &avrUart0Driver */ /* phys UART0 -> uartA */
        NULL, /* &avrUart1Driver */ /* phys UART1 -> uartB */
        NULL, /* &avrUart2Driver */ /* phys UART2 -> uartC */
        NULL,            /* no uartD */
        &avrI2CDriver,
        &apm2SPIDriver,
        &avrAnalogIn,
        &avrEEPROMStorage,
        &avrUart0Driver,
        &avrGPIO,
        &apm2RCInput,
        &apm2RCOutput,
        &avrScheduler,
        &avrUtil,
		NULL /* AP_HAL::Sim */ )
{}

void HAL_AVRDx::init(int argc, char * const argv[]) const {

    scheduler->init((void*)&isrRegistry);
   
    /* uartA is the serial port used for the console, so lets make sure
     * it is initialized at boot */
    uartA->begin(115200, 128, 128);
    /* The AVR RCInput drivers take an AP_HAL_AVR::ISRRegistry*
     * as the init argument */
    rcin->init((void*)&isrRegistry);
    rcout->init(NULL);
    spi->init(NULL);
    i2c->begin();
    i2c->setTimeout(100);
    analogin->init(NULL);

    /* Enable the pullups on the RX pins of the 3 UARTs This is important when
     * the RX line is high-Z: capacitive coupling between input and output pins
     * can cause bytes written to show up as an input. Occasionally this causes
     * us to detect a phantom GPS by seeing our own outgoing config message.
     * PE0 : RX0 (uartA)
     * PD2 : RX1 (uartB)
     * PH0 : RX2 (uartC)
     */

	// TODO Update these
	/*
    PORTE |= _BV(0);
    PORTD |= _BV(2);
    PORTH |= _BV(0);
	*/
};

// AVR-Dx HAL Class instance
const HAL_AVRDx AP_HAL_AVRDxInstance;

