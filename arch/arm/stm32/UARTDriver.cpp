
#include <arch/arm/stm32/UARTDriver.h>

/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size

extern const AP_HAL::HAL& hal;

using namespace stm32;

STM32UARTDriver::STM32UARTDriver(bool default_console) :
    device_path(NULL),
    _rd_fd(-1),
    _wr_fd(-1)
{
	// TODO
}

void STM32UARTDriver::set_device_path(const char *path)
{
    device_path = path;
}

void STM32UARTDriver::begin(uint32_t b) 
{
    begin(b, 0, 0);
}

void STM32UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
	// TODO
}

void STM32UARTDriver::end() 
{
	// TODO
}


void STM32UARTDriver::flush() 
{
	// TODO
}

bool STM32UARTDriver::is_initialized() 
{
    return _initialised;
}

void STM32UARTDriver::set_blocking_writes(bool blocking) 
{
    _nonblocking_writes = !blocking;
}

bool STM32UARTDriver::tx_pending() 
{ 
	// TODO
	return false;
}

int16_t STM32UARTDriver::available() 
{ 
	// TODO
	return 0;
}

int16_t STM32UARTDriver::txspace() 
{ 
	// TODO
	return 0;
}

int16_t STM32UARTDriver::read() 
{ 
	// TODO
	return 0;
}

size_t STM32UARTDriver::write(uint8_t c) 
{ 
	// TODO
    return 1;
}

size_t STM32UARTDriver::write(const uint8_t *buffer, size_t size)
{
	// TODO
    return 0;
}

int STM32UARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
	// TODO
    return 0;
}

int STM32UARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
	// TODO
    return 0;
}

void STM32UARTDriver::_timer_tick(void)
{
	// TODO
}
