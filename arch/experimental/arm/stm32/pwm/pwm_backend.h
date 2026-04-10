#pragma once

#include <stdint.h>

#include <rtos_hal/pwm.h>
#include <stm32f4xx_hal.h>

namespace stm32 {

struct Stm32PwmPinConfig {
    uint8_t logical_channel;
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t alternate_function;
    uint32_t timer_channel;
};

struct Stm32PwmTimerConfig {
    TIM_TypeDef *instance;
    uint32_t timer_clock_hz;
    uint8_t channel_count;
    Stm32PwmPinConfig channels[4];
};

class Stm32PwmBackend {
public:
    virtual ~Stm32PwmBackend() = default;

    virtual mc_rtos_hal::Status init(const Stm32PwmTimerConfig &config) = 0;
    virtual mc_rtos_hal::Status enable(uint8_t logical_channel) = 0;
    virtual mc_rtos_hal::Status disable(uint8_t logical_channel) = 0;
    virtual mc_rtos_hal::Status set_frequency(uint32_t frequency_hz) = 0;
    virtual mc_rtos_hal::Status set_pulse_width_us(uint8_t logical_channel, uint16_t pulse_width_us) = 0;
    virtual uint16_t pulse_width_us(uint8_t logical_channel) const = 0;
};

}  // namespace stm32
