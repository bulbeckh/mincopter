#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "UARTDriver.h"

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

using namespace RPI;

RPIUARTDriver::RPIUARTDriver(bool default_console) :
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

void RPIUARTDriver::begin(uint32_t b) 
{
	/* Implement */
}

void RPIUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
	/* Implement */
}

void RPIUARTDriver::end() 
{
	/* Implement */
}

void RPIUARTDriver::flush() 
{
	/* Implement */
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
	/* Implement */
	return false
}

int16_t RPIUARTDriver::available() 
{ 
	/* Implement */
}

int16_t RPIUARTDriver::txspace() 
{ 
	/* Implement */
}

int16_t RPIUARTDriver::read() 
{ 
	/* Implement */
}

size_t RPIUARTDriver::write(uint8_t c) 
{ 
	/* Implement */
}

size_t LinuxUARTDriver::write(const uint8_t *buffer, size_t size)
{
	/* Implement */
}

#endif // CONFIG_HAL_BOARD
