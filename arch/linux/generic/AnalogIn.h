
#pragma once

#include <AP_HAL_Generic.h>

class generic::GenericAnalogSource : public AP_HAL::AnalogSource {
public:
    GenericAnalogSource(float v);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }
private:
    float _v;
};

class generic::GenericAnalogIn : public AP_HAL::AnalogIn {
public:
    GenericAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);
};


