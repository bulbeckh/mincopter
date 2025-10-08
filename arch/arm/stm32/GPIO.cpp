
#include <arch/arm/stm32/GPIO.h>

using namespace stm32;


STM32GPIO::STM32GPIO()
{
	// TODO
}

void STM32GPIO::init()
{
	// TODO
}

void STM32GPIO::pinMode(uint8_t pin, uint8_t output)
{
	// TODO
}

int8_t STM32GPIO::analogPinToDigitalPin(uint8_t pin)
{
	// TODO
	return -1;
}


uint8_t STM32GPIO::read(uint8_t pin) {
	// TODO
    return 0;
}

void STM32GPIO::write(uint8_t pin, uint8_t value)
{
	// TODO
}

void STM32GPIO::toggle(uint8_t pin)
{
	// TODO
}

AP_HAL::DigitalSource* STM32GPIO::channel(uint16_t n) {
	// TODO
    return new STM32DigitalSource(0);
}

/* Interrupt interface: */
bool STM32GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) {
	// TODO
    return true;
}

bool STM32GPIO::usb_connected(void)
{
	// TODO
    return false;
}

STM32DigitalSource::STM32DigitalSource(uint8_t v) :
    _v(v)
{
	// TODO
}

void STM32DigitalSource::mode(uint8_t output)
{
	// TODO
}

uint8_t STM32DigitalSource::read() {
	// TODO
    return _v;
}

void STM32DigitalSource::write(uint8_t value) {
	// TODO
    _v = value;
}

void STM32DigitalSource::toggle() {
	// TODO
    _v = !_v;
}


