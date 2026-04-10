#pragma once

#include <rtos_hal/gpio.h>
#include <rtos_hal/i2c.h>
#include <rtos_hal/time.h>

#include <dev/experimental/compass/CompassDevice.h>

namespace mc_experimental {

struct Hmc5843Config {
    uint8_t i2c_address;
    uint16_t data_ready_pin;
    bool has_data_ready_irq;
    uint32_t sample_rate_hz;
};

class Hmc5843CompassDevice final : public CompassDevice {
public:
    Hmc5843CompassDevice(mc_rtos_hal::I2cBus &i2c,
                         mc_rtos_hal::Time &time,
                         mc_rtos_hal::GpioController &gpio,
                         const Hmc5843Config &config);

    bool init() override;
    bool reset() override;
    bool configure() override;
    bool has_data_ready_irq() const override;
    uint16_t data_ready_pin() const override;
    uint32_t sample_rate_hz() const override;
    bool read_sample(CompassSample &sample) override;

private:
    bool write_register(uint8_t reg, uint8_t value);
    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);

private:
    mc_rtos_hal::I2cBus &i2c_;
    mc_rtos_hal::Time &time_;
    mc_rtos_hal::GpioController &gpio_;
    Hmc5843Config config_;
    uint32_t sequence_;
};

}  // namespace mc_experimental
