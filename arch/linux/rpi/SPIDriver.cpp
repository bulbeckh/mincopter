#include <AP_HAL/AP_HAL.h>

#include <arch/linux/rpi/SPIDriver.h>

#include <pigpiod_if2.h>

#define RPI_SPI_

using namespace RPI;

extern const AP_HAL::HAL& hal;

// TODO Fix this bad hack by modifying the SPIDeviceDriver::init function prototype
static int32_t _pi_ref;

RPISPIDeviceDriver::RPISPIDeviceDriver(const char *spipath, uint8_t mode, uint8_t bitsPerWord, uint32_t speed) :
    _spipath(spipath),
    _fd(-1),
    _mode(mode),
    _bitsPerWord(bitsPerWord),
    _speed(speed)
{}

void RPISPIDeviceDriver::init()
{
	//_pi_ref = _ref;

	// TODO Change the '0' second argument to be configurable
	_handle = spi_open(_pi_ref, 0, _speed, 0x00);

	if(_handle<0) {
		// TODO Can we call a hal.scheduler->panic here? Has the scheduler been initialised yet by now
	}

	return;
}

AP_HAL::Semaphore* RPISPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void RPISPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	if (spi_xfer(_pi_ref, _handle, (char*)tx, (char*)rx, len) < len) hal.scheduler->panic("SPI Transaction failed\n");
	return;
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
	if (spi_write(_pi_ref, _handle, (char*)&data, 1) < 1) hal.scheduler->panic("SPI Transfer failed\n");
	return 0;
}

void RPISPIDeviceDriver::transfer(const uint8_t *data, uint16_t len)
{
	if (spi_write(_pi_ref, _handle, (char*)data, len) < len) hal.scheduler->panic("SPI Transfer failed\n");
	return;
}

RPISPIDeviceManager::RPISPIDeviceManager() :
	// TODO Remove the file path, mode, and bitsPerWord. Add the chip select pin (CE0 or CE1) as an argument
    _device_cs0("/dev/spidev0.0", 0 /* SPI_MODE_0 */, 8, 1000000),
    _device_cs1("/dev/spidev0.1", 0 /* SPI_MODE_0 */, 8, 1000000)
{
	/* Implement */
}

void RPISPIDeviceManager::init(void *)
{
	// Setup connection w GPIO daemon for SPI
	_pi_ref = pigpio_start(NULL, NULL);

    _device_cs0.init();
    _device_cs1.init();

	return;
}

AP_HAL::SPIDeviceDriver* RPISPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
    switch (dev) {
		/* Implement Case Statement */
        case AP_HAL::SPIDevice_ADS7844:
            return &_device_cs1;
		case AP_HAL::SPIDevice_ICM20948:
			// TODO Make this all configurable as per board wiring
			return &_device_cs0;
		/*
		case AP_HAL::SPIDevice_MPU6000:
			return &_device_cs0;
		*/
		default:
			return NULL;
    }
    return NULL;
}

