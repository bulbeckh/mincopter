#pragma once

#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

using GpioCallback = void (*)(void *context);

struct GpioInterruptConfig {
    Edge edge;
    GpioCallback callback;
    void *context;
};

class GpioPin {
public:
    virtual ~GpioPin() = default;
    virtual Status set_mode(PinMode mode) = 0;
    virtual void write(bool level) = 0;
    virtual bool read() const = 0;
    virtual void toggle() = 0;
    virtual Status attach_interrupt(const GpioInterruptConfig &config) = 0;
};

class GpioController {
public:
    virtual ~GpioController() = default;
    virtual GpioPin *pin(uint16_t logical_pin) = 0;
};

}  // namespace mc_rtos_hal
