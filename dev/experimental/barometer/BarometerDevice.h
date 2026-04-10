#pragma once

#include <stdint.h>

#include <rtos_hal/gpio.h>
#include <rtos_hal/i2c.h>
#include <rtos_hal/spi.h>
#include <rtos_hal/time.h>

namespace mc_experimental {

struct BarometerSample {
    uint32_t timestamp_us;
    uint32_t sequence;
    float pressure_pa;
    float temperature_c;
    bool valid;
};

class BarometerDevice {
public:
    virtual ~BarometerDevice() = default;

    virtual bool init() = 0;
    virtual bool reset() = 0;
    virtual bool configure() = 0;
    virtual bool has_data_ready_irq() const = 0;
    virtual uint16_t data_ready_pin() const = 0;
    virtual uint32_t sample_rate_hz() const = 0;
    virtual bool read_sample(BarometerSample &sample) = 0;
};

}  // namespace mc_experimental
