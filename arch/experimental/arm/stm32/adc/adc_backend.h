#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/adc.h>
#include <stm32f4xx_hal.h>

namespace stm32 {

struct Stm32AdcChannelHardwareConfig {
    uint8_t logical_index;
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t channel;
};

struct Stm32AdcUnitConfig {
    ADC_TypeDef *instance;
};

class Stm32AdcBackend {
public:
    virtual ~Stm32AdcBackend() = default;

    virtual mc_rtos_hal::Status init(const Stm32AdcUnitConfig &config) = 0;
    virtual mc_rtos_hal::Status init_channel(const Stm32AdcChannelHardwareConfig &config) = 0;
    virtual mc_rtos_hal::Status read_raw(const Stm32AdcChannelHardwareConfig &config, uint16_t &value) = 0;
};

}  // namespace stm32
