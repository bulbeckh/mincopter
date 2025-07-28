#include <AP_HAL/AP_HAL.h>

#include <arch/linux/generic/UARTDriver.h>

#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <assert.h>
#include <sys/ioctl.h>

extern const AP_HAL::HAL& hal;

using namespace generic;

GenericUARTDriver::GenericUARTDriver(bool default_console) :
    device_path(NULL),
    _rd_fd(-1),
    _wr_fd(-1)
{
    if (default_console) {
        _rd_fd = 0;
        _wr_fd = 1;
        _console = true;
    }
}

void GenericUARTDriver::set_device_path(const char *path)
{
    device_path = path;
}

void GenericUARTDriver::begin(uint32_t b) 
{
    begin(b, 0, 0);
}

void GenericUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
	/* Implement */
}

void GenericUARTDriver::end() 
{
	/* Implement */
}


void GenericUARTDriver::flush() 
{
    // we are not doing any buffering, so flush is a no-op
}

bool GenericUARTDriver::is_initialized() 
{
    return _initialised;
}

void GenericUARTDriver::set_blocking_writes(bool blocking) 
{
    _nonblocking_writes = !blocking;
}

bool GenericUARTDriver::tx_pending() 
{ 
	return false;
}

int16_t GenericUARTDriver::available() 
{ 
	return 0;
}

int16_t GenericUARTDriver::txspace() 
{ 
	return 0;
}

int16_t GenericUARTDriver::read() 
{ 
	/* Implement */
	return 0;
}

size_t GenericUARTDriver::write(uint8_t c) 
{ 
    return 0;
}

size_t GenericUARTDriver::write(const uint8_t *buffer, size_t size)
{
    return 0;
}

