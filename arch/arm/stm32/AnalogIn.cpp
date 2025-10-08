

#include <arch/arm/stm32/AnalogIn.h>

using namespace stm32;

STM32AnalogSource::STM32AnalogSource(float v) :
    _v(v)
{
	// TODO
}

float STM32AnalogSource::read_average() {
	// TODO
    return _v;
}

float STM32AnalogSource::voltage_average() {
	// TODO
    return 5.0 * _v / 1024.0;
}

float STM32AnalogSource::voltage_latest() {
	// TODO
    return 5.0 * _v / 1024.0;
}

float STM32AnalogSource::read_latest() {
	// TODO
    return _v;
}

void STM32AnalogSource::set_pin(uint8_t p)
{
	// TODO
}

void STM32AnalogSource::set_stop_pin(uint8_t p)
{
	// TODO
}

void STM32AnalogSource::set_settle_time(uint16_t settle_time_ms)
{
	// TODO
}

STM32AnalogIn::STM32AnalogIn()
{
	// TODO
}

// TODO Remove machtnichts arg from this init
void STM32AnalogIn::init(void* machtnichts)
{
	// TODO
}

AP_HAL::AnalogSource* STM32AnalogIn::channel(int16_t n) {
	// TODO
    return new STM32AnalogSource(1.11);
}


