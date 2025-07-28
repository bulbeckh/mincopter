#include <AP_HAL/AP_HAL.h>

#include <arch/linux/generic/SPIDriver.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

using namespace generic;

GenericSPIDeviceDriver::GenericSPIDeviceDriver(const char *spipath, uint8_t mode, uint8_t bitsPerWord, uint32_t speed) :
    _spipath(spipath),
    _fd(-1),
    _mode(mode),
    _bitsPerWord(bitsPerWord),
    _speed(speed)
{}

void GenericSPIDeviceDriver::init()
{
    _fd = open(_spipath, O_RDWR);
    if (_fd == -1) {
        return;
    }
    int ret;
    ret = ioctl(_fd, SPI_IOC_WR_MODE, &_mode);
    if (ret == -1) {
        goto failed;
    }
 
    ret = ioctl(_fd, SPI_IOC_RD_MODE, &_mode);
    if (ret == -1) {
        goto failed;
    }
 
    ret = ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &_bitsPerWord);
    if (ret == -1) {
        goto failed;
    }
    
    ret = ioctl(_fd, SPI_IOC_RD_BITS_PER_WORD, &_bitsPerWord);
    if (ret == -1) {
        goto failed;
    }
 
    ret = ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &_speed);    
    if (ret == -1) {
        goto failed;
    }
 
    ret = ioctl(_fd, SPI_IOC_RD_MAX_SPEED_HZ, &_speed);    
    if (ret == -1) {
        goto failed;
    }

    return;

failed:
    close(_fd);
    _fd = -1;
}

AP_HAL::Semaphore* GenericSPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void GenericSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    struct spi_ioc_transfer spi[1];
    spi[0].tx_buf        = (uint64_t)tx;
    spi[0].rx_buf        = (uint64_t)rx;
    spi[0].len           = len;
    spi[0].delay_usecs   = 0;
    spi[0].speed_hz      = _speed;
    spi[0].bits_per_word = _bitsPerWord;
    spi[0].cs_change     = 0;
 
    ioctl(_fd, SPI_IOC_MESSAGE(1), &spi);
}


void GenericSPIDeviceDriver::cs_assert()
{
}

void GenericSPIDeviceDriver::cs_release()
{
}

uint8_t GenericSPIDeviceDriver::transfer(uint8_t data)
{
    uint8_t v = 0;
    transaction(&data, &v, 1);
    return v;
}

void GenericSPIDeviceDriver::transfer(const uint8_t *data, uint16_t len)
{
    transaction(data, NULL, len);
}

GenericSPIDeviceManager::GenericSPIDeviceManager() :
    _device_cs0("/dev/spidev0.0", SPI_MODE_0, 8, 2600000),
    _device_cs1("/dev/spidev0.1", SPI_MODE_0, 8, 1000000)
{}

void GenericSPIDeviceManager::init(void *)
{
    _device_cs0.init();
    _device_cs1.init();
}

/*
  return a SPIDeviceDriver for a particular device
 */
AP_HAL::SPIDeviceDriver* GenericSPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
    switch (dev) {
        case AP_HAL::SPIDevice_ADS7844:
            return &_device_cs0;
    }
    return NULL;
}

