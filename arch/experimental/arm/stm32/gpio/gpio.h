#pragma once

#include <stddef.h>
#include <stdint.h>

#include <arm/stm32/gpio/gpio_backend.h>
#include <rtos_hal/gpio.h>

namespace stm32 {

class Stm32GpioController;

class Stm32GpioPin final : public mc_rtos_hal::GpioPin {
public:
    Stm32GpioPin();

    void configure(Stm32GpioController *controller, Stm32GpioBackend *backend, const Stm32HalGpioPinConfig &config);
    bool is_configured() const;
    uint16_t logical_pin() const;
    uint16_t exti_line() const;
    const Stm32HalGpioPinConfig &config() const;
    void handle_interrupt() const;

    mc_rtos_hal::Status set_mode(mc_rtos_hal::PinMode mode) override;
    void write(bool level) override;
    bool read() const override;
    void toggle() override;
    mc_rtos_hal::Status attach_interrupt(const mc_rtos_hal::GpioInterruptConfig &config) override;

private:
    Stm32GpioController *controller_;
    Stm32GpioBackend *backend_;
    Stm32HalGpioPinConfig config_;
    mc_rtos_hal::PinMode mode_;
    mc_rtos_hal::GpioInterruptConfig interrupt_config_;
    bool configured_;
    bool interrupt_attached_;
};

class Stm32GpioController final : public mc_rtos_hal::GpioController {
public:
    static constexpr uint16_t kMaxLogicalPins = 26U * 16U;

    Stm32GpioController();

    mc_rtos_hal::Status configure(const Stm32HalGpioPinConfig *pins, size_t pin_count);
    mc_rtos_hal::GpioPin *pin(uint16_t logical_pin) override;
    mc_rtos_hal::Status register_interrupt(Stm32GpioPin &pin, const mc_rtos_hal::GpioInterruptConfig &config);
    void handle_exti_irq(uint16_t pin_mask);
    static uint16_t exti_line_for_mask(uint16_t pin_mask);

private:
    Stm32GpioPin pins_[kMaxLogicalPins];
    Stm32GpioPin *exti_bindings_[16];
    Stm32GpioBackend *backend_;
};

}  // namespace stm32
