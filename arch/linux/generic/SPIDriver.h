
#pragma once

#include <arch/linux/generic/AP_HAL_Generic.h>

#include <arch/linux/Semaphores.h>

class generic::GenericSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
	public:
		GenericSPIDeviceDriver(const char *spipath, uint8_t mode, uint8_t bitsPerWord, uint32_t speed);

		void init();

		AP_HAL::Semaphore* get_semaphore();

		void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

		void cs_assert();

		void cs_release();

		uint8_t transfer (uint8_t data);

		void transfer (const uint8_t *data, uint16_t len);

	private:
		Linux::LinuxSemaphore _semaphore;
		const char *_spipath;
		int _fd;
		uint8_t _mode;
		uint8_t _bitsPerWord;
		uint32_t _speed;
};

class generic::GenericSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    GenericSPIDeviceManager();

    void init(void *);

    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);

private:
    GenericSPIDeviceDriver _device_cs0;
    GenericSPIDeviceDriver _device_cs1;
};

