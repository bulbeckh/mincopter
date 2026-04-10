#pragma once

#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

struct AdcChannelConfig {
    uint8_t channel_index;
    float reference_voltage;
    float scale;
};

class AdcChannel {
public:
    virtual ~AdcChannel() = default;
    virtual Status configure(const AdcChannelConfig &config) = 0;
    virtual Status read_raw(uint16_t &value) = 0;
    virtual Status read_voltage(float &voltage) = 0;
};

class Adc {
public:
    virtual ~Adc() = default;
    virtual AdcChannel *channel(uint8_t index) = 0;
    virtual uint8_t channel_count() const = 0;
};

}  // namespace mc_rtos_hal
