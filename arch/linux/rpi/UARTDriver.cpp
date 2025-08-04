#include <AP_HAL/AP_HAL.h>

#include <arch/linux/rpi/UARTDriver.h>

#include <stdio.h>

#include <pigpiod_if2.h>

// HASH include <errno.h>
// HASH include <termios.h>
// HASH include <stdlib.h>
// HASH include <sys/types.h>
// HASH include <sys/stat.h>
// HASH include <fcntl.h>
// HASH include <unistd.h>
// HASH include <poll.h>
// HASH include <assert.h>
// HASH include <sys/ioctl.h>

extern const AP_HAL::HAL& hal;

using namespace RPI;

RPIUARTDriver::RPIUARTDriver(const char* device_path) :
    _device(device_path),
	_handle(-1)
{

}

void RPIUARTDriver::begin(uint32_t b) 
{
	begin(b,0,0);
}

void RPIUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
	// If we are not using this UART, then ignore the init
	if (_device==NULL) return;

	// If we are not changing the baudrate, then ignore
	if (b==_baudrate) return;

	// Connect to pigpio daemon, if not already initialised
	if (!_initialised) {
		_pi_ref = pigpio_start(NULL, NULL);
		if (_pi_ref<0) hal.scheduler->panic(PSTR("UARTDriver Init: failed to connect to pigpiod daemon\n"));
	}

	// Open serial
	if (_handle!=-1) {
		// If we already have a handle, then we are re-initialising at a different baudrate so we close existing handle
		serial_close(_pi_ref, _handle);
	}
	_handle = serial_open(_pi_ref, (char*)_device, b, 0x00);
	if (_handle<0) hal.scheduler->panic(PSTR("UARTDriver Init: failed to open connection to /dev/ttyAMA0\n"));

	_initialised = true;
}

void RPIUARTDriver::end() 
{
	if (_device==NULL) return;

	int32_t status = serial_close(_pi_ref, _handle);
	if(status) hal.scheduler->panic(PSTR("UARTDriver Close: failed to close\n"));
}

void RPIUARTDriver::flush() 
{
	// Do nothing - the tx/rx buffers for pigpiod are fixed size and not configurable.
}

bool RPIUARTDriver::is_initialized() 
{
    return _initialised;
}

void RPIUARTDriver::set_blocking_writes(bool blocking) 
{
    _nonblocking_writes = !blocking;
}

bool RPIUARTDriver::tx_pending() 
{ 
	// TODO pigpiod handles writes asynchronously but there is no way to monitor the buffer for writes
	return false;
}

int16_t RPIUARTDriver::available() 
{ 
	if (_device==NULL) return 0;

	int16_t bytes_rx = serial_data_available(_pi_ref, _handle);
	if (bytes_rx<0) hal.scheduler->panic(PSTR("UARTDriver: could not read available bytes\n"));
	return bytes_rx;
}

int16_t RPIUARTDriver::txspace() 
{ 
	// Return max buffer size since we don't implement buffers directly
	return 4096;
}

int16_t RPIUARTDriver::read() 
{ 
	if (_device==NULL) return 0;

	int16_t data = serial_read_byte(_pi_ref, _handle);
	if (data==PI_SER_READ_NO_DATA) {
		hal.console->printf(PSTR("RPI-UART: No data available to read\n"));
		return 0x00;
	}
	return data;
}

size_t RPIUARTDriver::write(uint8_t c) 
{ 
	return write((uint8_t*)&c, 1);
}

size_t RPIUARTDriver::write(const uint8_t *buffer, size_t size)
{
	if (_device==NULL) return 0;

	int16_t status = serial_write(_pi_ref, _handle, (char*)buffer, size);
	if (status) {
		hal.console->printf(PSTR("RPI-UART: Failed to write\n"));
		return 0;
	}
	return size;
}

