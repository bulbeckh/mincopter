#pragma once

#include <stdint.h>

namespace mc_rtos_hal {

class Time {
public:
    virtual ~Time() = default;
    virtual uint32_t millis() const = 0;
    virtual uint32_t micros() const = 0;
    virtual void delay_ms(uint32_t delay_ms) = 0;
    virtual void delay_until_ms(uint32_t &last_wake_ms, uint32_t period_ms) = 0;
    virtual void busy_wait_us(uint32_t delay_us) = 0;
};

}  // namespace mc_rtos_hal
