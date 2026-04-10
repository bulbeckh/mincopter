#pragma once

#include <rtos_hal/gpio.h>
#include <rtos_hal/i2c.h>
#include <rtos_hal/time.h>

#include <dev/experimental/imu/ImuDevice.h>

namespace mc_experimental {

struct Mpu6050Config {
    uint8_t i2c_address;
    uint16_t data_ready_pin;
    bool has_data_ready_irq;
    uint32_t sample_rate_hz;
};

class Mpu6050ImuDevice final : public ImuDevice {
public:
    Mpu6050ImuDevice(mc_rtos_hal::I2cBus &i2c,
                     mc_rtos_hal::Time &time,
                     mc_rtos_hal::GpioController &gpio,
                     const Mpu6050Config &config);

    bool init() override;
    bool reset() override;
    bool check_identity() override;
    bool configure() override;
    bool has_data_ready_irq() const override;
    uint16_t data_ready_pin() const override;
    uint32_t sample_rate_hz() const override;
    bool read_sample(ImuSample &sample) override;

private:
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t &value);
    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);

private:
    mc_rtos_hal::I2cBus &i2c_;
    mc_rtos_hal::Time &time_;
    mc_rtos_hal::GpioController &gpio_;
    Mpu6050Config config_;
    uint32_t sequence_;
};

}  // namespace mc_experimental
