#include <AP_HAL/AP_HAL.h>

#include <arch/linux/generic/AnalogIn.h>

using namespace generic;

GenericAnalogSource::GenericAnalogSource(float v) :
    _v(v)
{}

float GenericAnalogSource::read_average() {
    return _v;
}

float GenericAnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float GenericAnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float GenericAnalogSource::read_latest() {
    return _v;
}

void GenericAnalogSource::set_pin(uint8_t p)
{}

void GenericAnalogSource::set_stop_pin(uint8_t p)
{}

void GenericAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

GenericAnalogIn::GenericAnalogIn()
{}

void GenericAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* GenericAnalogIn::channel(int16_t n) {
    return new GenericAnalogSource(1.11);
}

