
#pragma once

#include <AP_HAL.h>

#include "avrdx/AP_HAL_AVRDx_Namespace.h"
#include "avrdx/GPIO.h"
#include "avrdx/SPIDevices.h"
#include "avrdx/Semaphores.h"

class AP_HAL_AVRDx::AVRDxSPIDeviceManager : public AP_HAL::SPIDeviceManager {
	public:

		// Constructor
		AVRDxSPIDeviceManager(void);

		void init(void* machtnichts);
		AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice d);

	private:
		AVRDxDigitalSource mpu6k_cs;
		AVRDxDigitalSource ms5611_cs;
		AVRDxDigitalSource df_cs;

		AVRDxSPI0DeviceDriver _mpu6k;
		AVRDxSPI0DeviceDriver _ms5611;
		//AVRSPI0DeviceDriver* _optflow_spi0;

		// TODO Removed SPI3 so need to place dataflash (or other external storage chip to another SPI
		//AVRSPI3DeviceDriver _dataflash;
		//AVRSPI3DeviceDriver* _optflow_spi3;
};


