
#ifndef __AP_HAL_AVR_SPI_DRIVER_H__
#define __AP_HAL_AVR_SPI_DRIVER_H__

#include <AP_HAL.h>
#include "mega/AP_HAL_AVR_Namespace.h"
#include "mega/GPIO.h"
#include "mega/SPIDevices.h"
#include "mega/Semaphores.h"

class AP_HAL_AVR::APM2SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:

	// Constructor
	APM2SPIDeviceManager(void);

    void init(void* machtnichts);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice d);

private:
	AVRDigitalSource mpu6k_cs;
	AVRDigitalSource ms5611_cs;
	AVRDigitalSource df_cs;


    AVRSPI0DeviceDriver _mpu6k;
    AVRSPI0DeviceDriver _ms5611;
    //AVRSPI0DeviceDriver* _optflow_spi0;


#if defined(__AVR_ATmega2560__) || defined(__AVR_AT_mega1280__)
    AVRSPI3DeviceDriver _dataflash;
#else
	// For mega1281/mega2561, we move the dataflash to SPI0 which is the actual SPI (rather than
	// the USART in SPI mode)
    AVRSPI0DeviceDriver _dataflash;
#endif
	
    //AVRSPI3DeviceDriver* _optflow_spi3;
};

#endif // __AP_HAL_AVR_SPI_DRIVER_H__

