
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

#include <arch/linux/rpi/I2CDriver.h>

#include <pigpiod_if2.h>

using namespace RPI;

RPII2CDriver::RPII2CDriver(AP_HAL::Semaphore* semaphore, const char *device) : 
    _semaphore(semaphore),
    _fd(-1),
    _device(device),
	_i2c_device_count(0)
{
	/* Initialise array of device handles to -1 */
	for (int i=0;i<16;i++) {
		i2c_device[i] = 0;
	}
}

/* called from HAL class init() */
void RPII2CDriver::begin() 
{
}

void RPII2CDriver::end() 
{
	/* TODO Close each i2c_device */
}

bool RPII2CDriver::set_address(uint8_t addr)
{
}

void RPII2CDriver::setTimeout(uint16_t ms) 
{
    // unimplemented
}

void RPII2CDriver::setHighSpeed(bool active) 
{
    // unimplemented
}

int RPII2CDriver::check_device(uint8_t dev)
{
	for (int i=0;i<I2C_MAX_DEVICES;i++) {
		if (i2c_device[0]==dev) return handles[i];
	}

	/* Device not found, need to initialise */
	int handle = i2c_open(1, dev, 0);
	if (handle<0) {
		fprintf(stderr, "Failed to open I2C device: %x\n", dev);
		return -1;
	} else {
		if (_i2c_device_count+1==I2C_MAX_DEVICES) fprintf(stderr, "Cannot add I2C device. Limit reached\n");

		i2c_device[_i2c_device_count] = dev;
		handles[_i2c_device_count] = handle;
		_i2c_device_count++;
		return handle;
	}
}

uint8_t RPII2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
	int handle = check_device(addr);
	if(handle>0) {
		/* Write data to whatever device register is in _addr */
		if (i2c_write_byte_data(handle, _addr, *data) != 0 ) {
			fprintf(stderr, "I2C Error: Writing bytes [device: %x, data: %x]\n",addr, *data);
			// Return error
			return 1;
		}
	}
	return 0;
}

uint8_t RPII2CDriver::writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
{
	int handle = check_device(addr);
	if(handle>0) {
		/* Write data to device */
		if (i2c_write_block_data(handle, reg, data, len) != len ) {
			fprintf(stderr, "I2C Error: Writing bytes [device: %x]\n",addr);
			// Return error
			return 1;
		}
	}
	return 0;
}

uint8_t RPII2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
	int handle = check_device(addr);
	if(handle>0) {
		/* Write data to register */
		if (i2c_write_byte_data(handle, reg, val) != 0 ) {
			fprintf(stderr, "I2C Error: Writing bytes [device: %x, data: %x]\n",addr, val);
			// Return error
			return 1;
		}
	}
	return 0;
}

uint8_t RPII2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
}

uint8_t RPII2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
{
}

uint8_t RPII2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
}

uint8_t RPII2CDriver::lockup_count() 
{
    return 0;
}

