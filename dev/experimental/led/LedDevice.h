#pragma once

#include <stdint.h>

#include <rtos_hal/gpio.h>

namespace mc_experimental {

struct LedConfig {
    uint16_t pin;
    bool active_high;
    bool initially_on;
};

class LedDevice {
public:
    LedDevice(mc_rtos_hal::GpioController &gpio, const LedConfig &config);

    bool init();
    bool on();
    bool off();
    bool toggle();
    bool set(bool enabled);
    bool is_available() const;

private:
    mc_rtos_hal::GpioController &gpio_;
    LedConfig config_;
    mc_rtos_hal::GpioPin *pin_;
    bool initialized_;
    bool state_;
};

}  // namespace mc_experimental
