#pragma once

#include <rtos_hal/gpio.h>
#include <rtos_hal/i2c.h>
#include <rtos_hal/time.h>

#include <dev/experimental/compass/CompassDevice.h>

namespace mc_experimental {

struct Icm20948CompassConfig {
    uint8_t i2c_address;
    uint16_t data_ready_pin;
    bool has_data_ready_irq;
    uint32_t sample_rate_hz;
};

class Icm20948CompassDevice final : public CompassDevice {
public:
    Icm20948CompassDevice(mc_rtos_hal::I2cBus &i2c,
                          mc_rtos_hal::Time &time,
                          mc_rtos_hal::GpioController &gpio,
                          const Icm20948CompassConfig &config);

    bool init() override;
    bool reset() override;
    bool configure() override;
    bool has_data_ready_irq() const override;
    uint16_t data_ready_pin() const override;
    uint32_t sample_rate_hz() const override;
    bool read_sample(CompassSample &sample) override;

private:
    bool select_bank(uint8_t bank);
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t &value);
    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
    bool write_ak09916_register(uint8_t reg, uint8_t value);
    bool read_ak09916_register(uint8_t reg, uint8_t &value);
    bool configure_external_sensor_read();

private:
    mc_rtos_hal::I2cBus &i2c_;
    mc_rtos_hal::Time &time_;
    mc_rtos_hal::GpioController &gpio_;
    Icm20948CompassConfig config_;
    uint32_t sequence_;
    uint8_t selected_bank_;
};

}  // namespace mc_experimental
