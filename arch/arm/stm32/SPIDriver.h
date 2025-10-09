
#pragma once

#include <AP_HAL/AP_HAL.h>

#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>
#include <arch/arm/stm32/Semaphores.h>

#include "stm32f4xx_hal.h"

class stm32::STM32SPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
	public:
		STM32SPIDeviceDriver(uint32_t speed);

		void init(void);

		AP_HAL::Semaphore* get_semaphore(void);

		void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

		void cs_assert();

		void cs_release();

		uint8_t transfer (uint8_t data);

		void transfer (const uint8_t *data, uint16_t len);

	private:
		// TODO Are these needed
		stm32::STM32Semaphore _semaphore;

		/*
		const char *_spipath;
		int _fd;
		uint8_t _mode;
		uint8_t _bitsPerWord;
		*/

		uint32_t _speed;
};

class stm32::STM32SPIDeviceManager : public AP_HAL::SPIDeviceManager {
	public:
		STM32SPIDeviceManager(void);

		void init(void *);

		AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);

	private:
		/* @brief SPI HAL Configuration instance */
		SPI_HandleTypeDef mc_spi;

		// TODO Add configuration for more than 2 SPI devices
		// TODO Change the names of these - leftover from RPI names (i.e /dev/cs0)
		STM32SPIDeviceDriver _device_cs0;
		STM32SPIDeviceDriver _device_cs1;
};

