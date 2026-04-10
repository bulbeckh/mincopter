#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/gpio.h>
#include <stm32f4xx_hal.h>

namespace stm32 {

struct Stm32HalGpioPinConfig {
    uint16_t logical_pin;
    GPIO_TypeDef *port;
    uint16_t pin;
    bool active_low;
    bool interrupt_capable;
};

class Stm32GpioBackend {
public:
    virtual ~Stm32GpioBackend() = default;

    virtual mc_rtos_hal::Status init_pin(const Stm32HalGpioPinConfig &config, mc_rtos_hal::PinMode mode) = 0;
    virtual mc_rtos_hal::Status write_pin(const Stm32HalGpioPinConfig &config, bool level) = 0;
    virtual bool read_pin(const Stm32HalGpioPinConfig &config) const = 0;
    virtual mc_rtos_hal::Status toggle_pin(const Stm32HalGpioPinConfig &config) = 0;
    virtual mc_rtos_hal::Status attach_interrupt(const Stm32HalGpioPinConfig &config, mc_rtos_hal::Edge edge) = 0;
    virtual void enable_irq(uint16_t pin) = 0;
};

}  // namespace stm32
