#pragma once

#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

class PwmOutput {
public:
    virtual ~PwmOutput() = default;
    virtual Status enable(uint8_t channel) = 0;
    virtual Status disable(uint8_t channel) = 0;
    virtual Status set_frequency(uint8_t channel, uint32_t frequency_hz) = 0;
    virtual Status set_pulse_width_us(uint8_t channel, uint16_t pulse_width_us) = 0;
    virtual uint16_t pulse_width_us(uint8_t channel) const = 0;
};

}  // namespace mc_rtos_hal
