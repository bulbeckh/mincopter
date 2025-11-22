#include <AP_HAL.h>

#include <avr/io.h>

#include "avrdx/SPIDriver.h"
#include "avrdx/SPIDevices.h"
#include "avrdx/utility/pins_arduino_mega.h"

using namespace AP_HAL_AVRDx;

extern const AP_HAL::HAL& hal;

#define SPI0_SPCR_8MHz   0
#define SPI0_SPSR_8MHz   _BV(SPI2X)
#define SPI0_SPCR_500kHz _BV(SPR1)
#define SPI0_SPSR_500kHz _BV(SPI2X)

AVRDxSPIDeviceManager::AVRDxSPIDeviceManager(void)
	: AP_HAL::SPIDeviceManager(),
	mpu6k_cs(0,0),
    _mpu6k(NULL,0,0,0),

    ms5611_cs(0,0),
    _ms5611(NULL,0,0,0),

    df_cs(0,0)
    //_dataflash(&NULL, 0, 0)

	  // TODO
	  /*
	mpu6k_cs(_BV(0), PB),
    _mpu6k(&mpu6k_cs, SPI0_SPCR_500kHz, SPI0_SPCR_8MHz, SPI0_SPSR_8MHz),

    ms5611_cs(_BV(1), PG),
    _ms5611(&ms5611_cs, SPI0_SPCR_500kHz, SPI0_SPCR_8MHz, SPI0_SPSR_8MHz),

    df_cs(_BV(6), PA),
    _dataflash(&df_cs, 0, 0)
	*/
{

}


void AVRDxSPIDeviceManager::init(void* machtnichts) {

    /* Note that the order of the init() of the MS5611 and MPU6k is
     * critical for the APM2. If you initialise in the wrong order
     * then the MS5611 doesn't initialise itself correctly. This
     * indicates an electrical fault in the APM2 which needs to be
     * investigated. Meanwhile, initialising the MPU6k CS pin before
     * the MS5611 CS pin works around the problem
     */


    /* mpu6k cs is on Arduino pin 53, PORTB0 */
    //AVRDigitalSource* mpu6k_cs = new AVRDigitalSource(_BV(0), PB);
    /* mpu6k: run clock at 8MHz in high speed mode and 512kHz for low
     * speed */
    //_mpu6k = new AVRSPI0DeviceDriver(mpu6k_cs, SPI0_SPCR_500kHz, SPI0_SPCR_8MHz, SPI0_SPSR_8MHz);
	// TODO
    //_mpu6k.init();

    /* ms5611 cs is on Arduino pin 40, PORTG1 */
    //AVRDigitalSource* ms5611_cs = new AVRDigitalSource(_BV(1), PG);
    /* ms5611: run clock at 8MHz */
    //_ms5611 = new AVRSPI0DeviceDriver(ms5611_cs, SPI0_SPCR_500kHz, SPI0_SPCR_8MHz, SPI0_SPSR_8MHz);
	// TODO
    // _ms5611.init();
   
	// NOTE All optflow removed
    /* optflow cs is on Arduino pin A3, PORTF3 */
    //AVRDigitalSource* optflow_cs = new AVRDigitalSource(_BV(3), PF);
    /* optflow: divide clock by 8 to 2Mhz
     * spcr gets bit SPR0, spsr gets bit SPI2X */
    //_optflow_spi0 = new AVRSPI0DeviceDriver(optflow_cs, _BV(SPR0)|_BV(CPOL)|_BV(CPHA), _BV(SPR0)|_BV(CPOL)|_BV(CPHA), _BV(SPI2X));
    //_optflow_spi0->init();

    /* Dataflash CS is on Arduino pin 28, PORTA6 */
    //AVRDigitalSource* df_cs = new AVRDigitalSource(_BV(6), PA);
    /* dataflash uses mode 0 and a clock of 8mhz
     * ucsr3c = 0 
     * ubrr3 = 0 */
    //_dataflash = new AVRSPI3DeviceDriver(df_cs, 0, 0);
	// TODO
    //_dataflash.init();

    /* optflow uses mode 3 and a clock of 2mhz
     * ucsr3c = _BV(UCPHA3N)|_BV(UCPOL3) = 3
     * ubrr3 = 3 */
    //_optflow_spi3 = new AVRSPI3DeviceDriver(optflow_cs, 3, 3);
    //_optflow_spi3->init();
}

AP_HAL::SPIDeviceDriver* AVRDxSPIDeviceManager::device(enum AP_HAL::SPIDevice d) 
{
	// TODO
	return NULL;
	/*
    switch (d) {
        case AP_HAL::SPIDevice_Dataflash:
            return &_dataflash;
        case AP_HAL::SPIDevice_MS5611:
            return &_ms5611;
        case AP_HAL::SPIDevice_MPU6000:
            return &_mpu6k;
        default:
            return NULL;
    };
	*/
}

