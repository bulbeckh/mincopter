#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn.h"

using namespace RPI;

RPIAnalogSource::RPIAnalogSource(float v) :
    _v(v)
{
}

float RPIAnalogSource::read_average() {
    return _v;
}

float RPIAnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float RPIAnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float RPIAnalogSource::read_latest() {
    return _v;
}

void RPIAnalogSource::set_pin(uint8_t p)
{}

void RPIAnalogSource::set_stop_pin(uint8_t p)
{}

void RPIAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

RPIAnalogIn::RPIAnalogIn()
{}

void RPIAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* RPIAnalogIn::channel(int16_t n) {
    return new RPIAnalogSource(1.11);
}

#endif // CONFIG_HAL_BOARD
