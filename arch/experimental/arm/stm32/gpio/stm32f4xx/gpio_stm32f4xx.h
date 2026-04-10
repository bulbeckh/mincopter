#pragma once

#include <arm/stm32/gpio/gpio_backend.h>

namespace stm32 {

class Stm32F4xxGpioBackend final : public Stm32GpioBackend {
public:
    mc_rtos_hal::Status init_pin(const Stm32HalGpioPinConfig &config, mc_rtos_hal::PinMode mode) override;
    mc_rtos_hal::Status write_pin(const Stm32HalGpioPinConfig &config, bool level) override;
    bool read_pin(const Stm32HalGpioPinConfig &config) const override;
    mc_rtos_hal::Status toggle_pin(const Stm32HalGpioPinConfig &config) override;
    mc_rtos_hal::Status attach_interrupt(const Stm32HalGpioPinConfig &config, mc_rtos_hal::Edge edge) override;
    void enable_irq(uint16_t pin) override;

private:
    static void enable_gpio_clock(GPIO_TypeDef *port);
    static GPIO_InitTypeDef make_init(const Stm32HalGpioPinConfig &config, mc_rtos_hal::PinMode mode);
    static uint32_t to_exti_mode(mc_rtos_hal::Edge edge);
    static IRQn_Type irq_for_pin(uint16_t pin);
};

}  // namespace stm32
