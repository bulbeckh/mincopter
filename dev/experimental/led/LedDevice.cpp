#include <dev/experimental/led/LedDevice.h>

namespace mc_experimental {

LedDevice::LedDevice(mc_rtos_hal::GpioController &gpio, const LedConfig &config)
    : gpio_(gpio),
      config_(config),
      pin_(nullptr),
      initialized_(false),
      state_(false) {}

bool LedDevice::init() {
    pin_ = gpio_.pin(config_.pin);
    if (pin_ == nullptr) {
        return false;
    }

    if (pin_->set_mode(mc_rtos_hal::PinMode::Output) != mc_rtos_hal::Status::Ok) {
        return false;
    }

    initialized_ = true;
    return set(config_.initially_on);
}

bool LedDevice::on() {
    return set(true);
}

bool LedDevice::off() {
    return set(false);
}

bool LedDevice::toggle() {
    return set(!state_);
}

bool LedDevice::set(bool enabled) {
    if (!initialized_ || pin_ == nullptr) {
        return false;
    }

    state_ = enabled;
    pin_->write(enabled == config_.active_high);
    return true;
}

bool LedDevice::is_available() const {
    return initialized_;
}

}  // namespace mc_experimental
