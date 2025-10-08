
#include <arch/arm/stm32/SPIDriver.h>

using namespace stm32;

// TODO Remove data members like _fd
STM32SPIDeviceDriver::STM32SPIDeviceDriver(const char *spipath, uint8_t mode, uint8_t bitsPerWord, uint32_t speed) :
    _spipath(spipath),
    _fd(-1),
    _mode(mode),
    _bitsPerWord(bitsPerWord),
    _speed(speed)
{}

void STM32SPIDeviceDriver::init()
{
	// TODO
}

AP_HAL::Semaphore* STM32SPIDeviceDriver::get_semaphore()
{
	// TODO
	return NULL;
}

void STM32SPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	// TODO
}


void STM32SPIDeviceDriver::cs_assert()
{
	// TODO
}

void STM32SPIDeviceDriver::cs_release()
{
	// TODO
}

uint8_t STM32SPIDeviceDriver::transfer(uint8_t data)
{
	// TODO
    return 0;
}

void STM32SPIDeviceDriver::transfer(const uint8_t *data, uint16_t len)
{
    transaction(data, NULL, len);
}

// TODO Update args for these constructors
STM32SPIDeviceManager::STM32SPIDeviceManager() :
    _device_cs0("/dev/spidev0.0", 0 /* SPI_MODE_0 */, 8, 2600000),
    _device_cs1("/dev/spidev0.1", 0 /* SPI_MODE_0 */, 8, 1000000)
{}

void STM32SPIDeviceManager::init(void *)
{
	// TODO
    _device_cs0.init();
    _device_cs1.init();

	return;
}

AP_HAL::SPIDeviceDriver* STM32SPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
	// TODO
	/*
    switch (dev) {
        case AP_HAL::SPIDevice_ADS7844:
            return &_device_cs0;
    }
	*/
    return NULL;
}

