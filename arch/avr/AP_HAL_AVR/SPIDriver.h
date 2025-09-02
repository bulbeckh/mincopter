
#ifndef __AP_HAL_AVR_SPI_DRIVER_H__
#define __AP_HAL_AVR_SPI_DRIVER_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"
#include "GPIO.h"
#include "SPIDevices.h"
#include "Semaphores.h"

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

    AVRSPI3DeviceDriver _dataflash;
    //AVRSPI3DeviceDriver* _optflow_spi3;
};

#endif // __AP_HAL_AVR_SPI_DRIVER_H__

