#include <AP_HAL/AP_HAL.h>

#include <arch/linux/generic/UARTDriver.h>

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

/*
  buffer handling macros
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size
*/

extern const AP_HAL::HAL& hal;

using namespace generic;

GenericUARTDriver::GenericUARTDriver(bool default_console) :
	_console{false}
{
    if (default_console) {
        _console = true;
    }
}

void GenericUARTDriver::begin(uint32_t b) 
{
    begin(b, 0, 0);
}

void GenericUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
	// If this is the console terminal instance then we can just use printf
	if (_console) {
		initialised = true;
		::printf("Initialised console terminal with stdin, stdout\r\n");
		return;
	}

	// TODO
	master_fd = posix_openpt(O_RDWR | O_NOCTTY);
	grantpt(master_fd);
	unlockpt(master_fd);

	generic_fp = fdopen(master_fd, "r+");

	if (!generic_fp) hal.scheduler->panic("Unable to start uart terminal\r\n");

	char* pseudo_name = ptsname(master_fd);

	::printf("Initialised uart with pseudo terminal %s\r\n", pseudo_name);

	return;
}

void GenericUARTDriver::end(void)
{
	// TODO
}


void GenericUARTDriver::flush(void)
{
	// Flush any text immediately. We generally won't need to use this for generic as we flush our writes immediately anyway
	fflush(generic_fp);

	return;
}

void GenericUARTDriver::set_blocking_writes(bool blocking) 
{
	// Our writes always block so ignore this call
	return;
}

bool GenericUARTDriver::tx_pending(void)
{ 
	return false;
}

int16_t GenericUARTDriver::available(void)
{ 
	// TODO I think we should be able to see query this
	return 0;
}

int16_t GenericUARTDriver::txspace(void)
{ 
	return 0;
}

int16_t GenericUARTDriver::read(void)
{ 
	char uart_next_char[1];

	// If we are in the console uart, then we read a single character and return it
	if (_console) {
		int n = ::scanf("%c", &uart_next_char);

		// If we didn't read anything then return -1
		if (!n) return -1;

		return (int16_t)(*uart_next_char);
	}

	char* char_ptr = fgets(uart_next_char, sizeof(uart_next_char), generic_fp);

	// Return -1 if we did not read anything
	if (!char_ptr) return -1;

	// Otherwise return the read character
	return (int16_t)(*uart_next_char);
}

size_t GenericUARTDriver::write(uint8_t c) 
{ 
	if (_console) {
		// We are in the console UART and hence write directly to stdout
		::printf("%c", (char)c);
		return 1;
	}

	// Write to pseudo terminal for this UART
	::fprintf(generic_fp, "%c", (char)c);
	
	// TODO Add check for error in write

    return 1;
}

size_t GenericUARTDriver::write(const uint8_t *buffer, size_t size)
{
	if (_console) {
		// We are in the console UART and hence write directly to stdout
		::printf("%.*s", size, (const char*)buffer);
	}
	
	::fprintf(generic_fp, "%.*s", size, (const char*)buffer);

	// TODO Should this really return 0??
	
    return 0;
}

/*
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
*/

