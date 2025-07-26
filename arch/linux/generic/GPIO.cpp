#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "GPIO.h"

using namespace generic;

GenericGPIO::GenericGPIO()
{}

void GenericGPIO::init()
{}

void GenericGPIO::pinMode(uint8_t pin, uint8_t output)
{}

int8_t GenericGPIO::analogPinToDigitalPin(uint8_t pin)
{
	return -1;
}


uint8_t GenericGPIO::read(uint8_t pin) {
    return 0;
}

void GenericGPIO::write(uint8_t pin, uint8_t value)
{}

void GenericGPIO::toggle(uint8_t pin)
{}

/* Alternative interface: */
AP_HAL::DigitalSource* GenericGPIO::channel(uint16_t n) {
    return new GenericDigitalSource(0);
}

/* Interrupt interface: */
bool GenericGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool GenericGPIO::usb_connected(void)
{
    return false;
}

GenericDigitalSource::GenericDigitalSource(uint8_t v) :
    _v(v)
{}

void GenericDigitalSource::mode(uint8_t output)
{}

uint8_t GenericDigitalSource::read() {
    return _v;
}

void GenericDigitalSource::write(uint8_t value) {
    _v = value;
}

void GenericDigitalSource::toggle() {
    _v = !_v;
}

#endif // CONFIG_HAL_BOARD
