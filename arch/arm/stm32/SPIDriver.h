
#pragma once

#include <AP_HAL/AP_HAL.h>

#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>
#include <arch/arm/stm32/Semaphores.h>

class stm32::STM32SPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
	public:
		STM32SPIDeviceDriver(const char *spipath, uint8_t mode, uint8_t bitsPerWord, uint32_t speed);

		void init();

		AP_HAL::Semaphore* get_semaphore();

		void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

		void cs_assert();

		void cs_release();

		uint8_t transfer (uint8_t data);

		void transfer (const uint8_t *data, uint16_t len);

	private:
		// TODO Are these needed
		stm32::STM32Semaphore _semaphore;
		const char *_spipath;
		int _fd;
		uint8_t _mode;
		uint8_t _bitsPerWord;
		uint32_t _speed;
};

class stm32::STM32SPIDeviceManager : public AP_HAL::SPIDeviceManager {
	public:
		STM32SPIDeviceManager();

		void init(void *);

		AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);

	private:
		STM32SPIDeviceDriver _device_cs0;
		STM32SPIDeviceDriver _device_cs1;
};

