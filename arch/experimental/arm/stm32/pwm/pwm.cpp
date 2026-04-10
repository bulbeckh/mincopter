#include <arm/stm32/pwm/pwm.h>

#include <arm/stm32/pwm/stm32f4xx/pwm_stm32f4xx.h>

namespace stm32 {

Stm32PwmOutput::Stm32PwmOutput()
    : backend_(),
      configured_(false) {}

mc_rtos_hal::Status Stm32PwmOutput::configure(const Stm32PwmTimerConfig &config) {
    backend_ = std::make_unique<Stm32F4xxPwmBackend>();
    const mc_rtos_hal::Status status = backend_->init(config);
    configured_ = (status == mc_rtos_hal::Status::Ok);
    return status;
}

mc_rtos_hal::Status Stm32PwmOutput::enable(uint8_t channel) {
    if (!configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->enable(channel);
}

mc_rtos_hal::Status Stm32PwmOutput::disable(uint8_t channel) {
    if (!configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->disable(channel);
}

mc_rtos_hal::Status Stm32PwmOutput::set_frequency(uint8_t, uint32_t frequency_hz) {
    if (!configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->set_frequency(frequency_hz);
}

mc_rtos_hal::Status Stm32PwmOutput::set_pulse_width_us(uint8_t channel, uint16_t pulse_width_us) {
    if (!configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->set_pulse_width_us(channel, pulse_width_us);
}

uint16_t Stm32PwmOutput::pulse_width_us(uint8_t channel) const {
    if (!configured_ || !backend_) {
        return 0U;
    }
    return backend_->pulse_width_us(channel);
}

}  // namespace stm32
