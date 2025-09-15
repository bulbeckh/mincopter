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

#include <iostream>

/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size

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
	if (_rd_fd==0 && _wr_fd==1) {
		// We are in the console UART and hence write directly to stdout
		::printf("%c", (char)c);
	}

    return 1;
}

size_t GenericUARTDriver::write(const uint8_t *buffer, size_t size)
{
	if (_rd_fd==0 && _wr_fd==1) {
		// We are in the console UART and hence write directly to stdout
		::printf("%.*s", size, (const char*)buffer);
	}
    return 0;
}

int GenericUARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    int ret = 0;

    struct pollfd fds;
    fds.fd = _wr_fd;
    fds.events = POLLOUT;
    fds.revents = 0;

    if (poll(&fds, 1, 0) == 1) {
        ret = ::write(_wr_fd, buf, n);
    }

    if (ret > 0) {
        BUF_ADVANCEHEAD(_writebuf, ret);
        return ret;
    }

    return ret;
}

int GenericUARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    int ret;
    ret = ::read(_rd_fd, buf, n);
    if (ret > 0) {
        BUF_ADVANCETAIL(_readbuf, ret);
    }
    return ret;
}

void GenericUARTDriver::_timer_tick(void)
{
    uint16_t n;

    if (!_initialised) return;

    _in_timer = true;

    // write any pending bytes
    uint16_t _tail;
    n = BUF_AVAILABLE(_writebuf);
    if (n > 0) {
        if (_tail > _writebuf_head) {
            // do as a single write
            _write_fd(&_writebuf[_writebuf_head], n);
        } else {
            // split into two writes
            uint16_t n1 = _writebuf_size - _writebuf_head;
            int ret = _write_fd(&_writebuf[_writebuf_head], n1);
            if (ret == n1 && n != n1) {
                _write_fd(&_writebuf[_writebuf_head], n - n1);                
            }
        }
    }

    // try to fill the read buffer
    uint16_t _head;
    n = BUF_SPACE(_readbuf);
    if (n > 0) {
        if (_readbuf_tail < _head) {
            // one read will do
            assert(_readbuf_tail+n <= _readbuf_size);
            _read_fd(&_readbuf[_readbuf_tail], n);
        } else {
            uint16_t n1 = _readbuf_size - _readbuf_tail;
            assert(_readbuf_tail+n1 <= _readbuf_size);
            int ret = _read_fd(&_readbuf[_readbuf_tail], n1);
            if (ret == n1 && n != n1) {
                assert(_readbuf_tail+(n-n1) <= _readbuf_size);
                _read_fd(&_readbuf[_readbuf_tail], n - n1);                
            }
        }
    }

    _in_timer = false;
}
