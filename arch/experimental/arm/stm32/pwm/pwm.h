#pragma once

#include <memory>

#include <arm/stm32/pwm/pwm_backend.h>
#include <rtos_hal/pwm.h>

namespace stm32 {

class Stm32PwmOutput final : public mc_rtos_hal::PwmOutput {
public:
    Stm32PwmOutput();

    mc_rtos_hal::Status configure(const Stm32PwmTimerConfig &config);

    mc_rtos_hal::Status enable(uint8_t channel) override;
    mc_rtos_hal::Status disable(uint8_t channel) override;
    mc_rtos_hal::Status set_frequency(uint8_t channel, uint32_t frequency_hz) override;
    mc_rtos_hal::Status set_pulse_width_us(uint8_t channel, uint16_t pulse_width_us) override;
    uint16_t pulse_width_us(uint8_t channel) const override;

private:
    std::unique_ptr<Stm32PwmBackend> backend_;
    bool configured_;
};

}  // namespace stm32
