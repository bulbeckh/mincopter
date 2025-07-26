#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "SPIDriver.h"

using namespace RPI;

RPISPIDeviceDriver::RPISPIDeviceDriver(const char *spipath, uint8_t mode, uint8_t bitsPerWord, uint32_t speed) :
    _spipath(spipath),
    _fd(-1),
    _mode(mode),
    _bitsPerWord(bitsPerWord),
    _speed(speed)
{}

void RPISPIDeviceDriver::init()
{
	/* Implement */
}

AP_HAL::Semaphore* RPISPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void RPISPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	/* Implement */
}


void RPISPIDeviceDriver::cs_assert()
{
	/* Implement */
}

void RPISPIDeviceDriver::cs_release()
{
	/* Implement */
}

uint8_t RPISPIDeviceDriver::transfer(uint8_t data)
{
	/* Implement */
	return 0;
}

void RPISPIDeviceDriver::transfer(const uint8_t *data, uint16_t len)
{
	/* Implement */
}

RPISPIDeviceManager::RPISPIDeviceManager() :
    _device_cs0("/dev/spidev0.0", 0 /* SPI_MODE_0 */, 8, 2600000),
    _device_cs1("/dev/spidev0.1", 0 /* SPI_MODE_0 */, 8, 1000000)
{
	/* Implement */
}

void RPISPIDeviceManager::init(void *)
{
    _device_cs0.init();
    _device_cs1.init();
}

AP_HAL::SPIDeviceDriver* RPISPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
    switch (dev) {
		/* Implement Case Statement */
		/*
        case AP_HAL::SPIDevice_ADS7844:
            return &_device_cs0;
		*/
    }
    return NULL;
}

#endif // CONFIG_HAL_BOARD
