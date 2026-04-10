#pragma once

#include <arm/stm32/pwm/pwm_backend.h>

namespace stm32 {

class Stm32F4xxPwmBackend final : public Stm32PwmBackend {
public:
    Stm32F4xxPwmBackend();

    mc_rtos_hal::Status init(const Stm32PwmTimerConfig &config) override;
    mc_rtos_hal::Status enable(uint8_t logical_channel) override;
    mc_rtos_hal::Status disable(uint8_t logical_channel) override;
    mc_rtos_hal::Status set_frequency(uint32_t frequency_hz) override;
    mc_rtos_hal::Status set_pulse_width_us(uint8_t logical_channel, uint16_t pulse_width_us) override;
    uint16_t pulse_width_us(uint8_t logical_channel) const override;

private:
    mc_rtos_hal::Status init_gpio(const Stm32PwmTimerConfig &config);
    mc_rtos_hal::Status init_timer(uint32_t frequency_hz);
    mc_rtos_hal::Status enable_clock(TIM_TypeDef *instance);
    static void enable_gpio_clock(GPIO_TypeDef *port);
    static uint32_t period_ticks_for(uint32_t timer_clock_hz, uint32_t frequency_hz);
    int8_t index_for_channel(uint8_t logical_channel) const;

    TIM_HandleTypeDef handle_;
    Stm32PwmTimerConfig config_;
    uint16_t pulse_widths_[8];
    bool enabled_[8];
    uint32_t frequency_hz_;
    uint32_t tick_hz_;
    uint32_t period_ticks_;
    bool initialized_;
};

}  // namespace stm32
