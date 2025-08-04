
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

#include <arch/linux/rpi/I2CDriver.h>

#include <pigpiod_if2.h>

using namespace RPI;

extern const AP_HAL::HAL& hal;

RPII2CDriver::RPII2CDriver(AP_HAL::Semaphore* semaphore, const char *device) : 
    _semaphore(semaphore),
    _fd(-1),
    _device(device),
	_i2c_device_head(0),
	_daemon(-1)
{
	/* Initialise array of device handles to -1 */
	for (int i=0;i<I2C_MAX_DEVICES;i++) {
		_i2c_device_address[i] = 0;
	}
}

/* called from HAL class init() */
void RPII2CDriver::begin() 
{
	// Connect to pigpiod daemon with default (localhost, 8888)
	int daemon = pigpio_start(NULL, NULL);
	if (daemon<0) {
		fprintf(stderr, "I2C Error: Connecting to pigpiod.\n");
	} else {
		_daemon = daemon;
	}
	return;
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
	// TODO Either change this to something faster like a binary tree or change the way we retrieve
	// device handles because this is inefficient to do this on every call to check_device.
	
	/* Search the array for the corresponding handle */
	for (int i=0;i<I2C_MAX_DEVICES;i++) {
		if (_i2c_device_address[i]==dev) return _i2c_device_handle[i];
	}

	// Check if we have too many devices
	if (_i2c_device_head==I2C_MAX_DEVICES) {
		hal.scheduler->panic(PSTR("Cannot add I2C device. Limit reached\n"));
	}

	// Device not found, need to initialise
	int handle = i2c_open(_daemon, 1, dev, 0x00);
	if (handle<0) {
		fprintf(stderr, "Failed to open I2C device: %x\n", dev);
		return -1;
	} else {
		// TODO Does this need to be atomic
		_i2c_device_address[_i2c_device_head] = dev;
		_i2c_device_handle[_i2c_device_head]  = handle;
		_i2c_device_head += 1;
		return handle;
	}
}

uint8_t RPII2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
	int handle = check_device(addr);
	if(handle>0) {
		/* Write data to whatever device register is in _addr */
		if (i2c_write_byte_data(_daemon, handle, _addr, *data) != 0 ) {
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
		if (i2c_write_block_data(_daemon, handle, reg, (char*)data, len) != len ) {
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
		if (i2c_write_byte_data(_daemon, handle, reg, val) != 0 ) {
			fprintf(stderr, "I2C Error: Writing bytes [device: %x, data: %x]\n",addr, val);
			// Return error
			return 1;
		}
	}
	return 0;
}

uint8_t RPII2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
	/* TODO */
}

uint8_t RPII2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
{
	int handle = check_device(addr);

	if (handle>0) {
		int32_t status = i2c_read_i2c_block_data(_daemon, handle, reg, (char*)data, len);

		if(status<0) {
			fprintf(stderr, "I2C Error: Reading bytes [device: %x]\n",addr);
			return 1;
		}
	} else {
		fprintf(stderr, "I2C Error: read - bad handle\n");
		return 1;
	}
	return 0;
}

uint8_t RPII2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	return readRegisters(addr, reg, 1, data);
}

uint8_t RPII2CDriver::lockup_count() 
{
    return 0;
}

