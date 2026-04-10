#pragma once

#include <stdint.h>

#include <rtos_hal/i2c.h>
#include <rtos_hal/spi.h>
#include <rtos_hal/time.h>
#include <rtos_hal/gpio.h>

namespace mc_experimental {

struct ImuSample {
    uint32_t timestamp_us;
    uint32_t sequence;
    float gyro_rad_s[3];
    float accel_m_s2[3];
    float temperature_c;
    bool valid;
};

class ImuDevice {
public:
    virtual ~ImuDevice() = default;

    virtual bool init() = 0;
    virtual bool reset() = 0;
    virtual bool check_identity() = 0;
    virtual bool configure() = 0;
    virtual bool has_data_ready_irq() const = 0;
    virtual uint16_t data_ready_pin() const = 0;
    virtual uint32_t sample_rate_hz() const = 0;
    virtual bool read_sample(ImuSample &sample) = 0;
};

}  // namespace mc_experimental
