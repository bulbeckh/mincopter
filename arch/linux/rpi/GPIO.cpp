#include <AP_HAL/AP_HAL.h>

#include <arch/linux/rpi/GPIO.h>

using namespace RPI;

RPIGPIO::RPIGPIO()
{
	/* Implement */
}

void RPIGPIO::init()
{
	/* Implement */
}

void RPIGPIO::pinMode(uint8_t pin, uint8_t output)
{
	/* Implement */
}

int8_t RPIGPIO::analogPinToDigitalPin(uint8_t pin)
{
	return -1;
}


uint8_t RPIGPIO::read(uint8_t pin) {
    return 0;
}

void RPIGPIO::write(uint8_t pin, uint8_t value)
{
	/* Implement */
}

void RPIGPIO::toggle(uint8_t pin)
{
	/* Implement */
}

/* Alternative interface: */
AP_HAL::DigitalSource* RPIGPIO::channel(uint16_t n) {
    return new RPIDigitalSource(0);
}

/* Interrupt interface: */
bool RPIGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool RPIGPIO::usb_connected(void)
{
    return false;
}

RPIDigitalSource::RPIDigitalSource(uint8_t v) :
    _v(v)
{
	/* Implement */
}

void RPIDigitalSource::mode(uint8_t output)
{
	/* Implement */
}

uint8_t RPIDigitalSource::read() {
    return _v;
}

void RPIDigitalSource::write(uint8_t value) {
    _v = value;
}

void RPIDigitalSource::toggle() {
    _v = !_v;
}

