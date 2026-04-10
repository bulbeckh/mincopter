#pragma once

#include <rtos_hal/gpio.h>
#include <rtos_hal/i2c.h>
#include <rtos_hal/time.h>

#include <dev/experimental/barometer/BarometerDevice.h>

namespace mc_experimental {

struct Bme280Config {
    uint8_t i2c_address;
    uint16_t data_ready_pin;
    bool has_data_ready_irq;
    uint32_t sample_rate_hz;
};

class Bme280BarometerDevice final : public BarometerDevice {
public:
    Bme280BarometerDevice(mc_rtos_hal::I2cBus &i2c,
                          mc_rtos_hal::Time &time,
                          mc_rtos_hal::GpioController &gpio,
                          const Bme280Config &config);

    bool init() override;
    bool reset() override;
    bool configure() override;
    bool has_data_ready_irq() const override;
    uint16_t data_ready_pin() const override;
    uint32_t sample_rate_hz() const override;
    bool read_sample(BarometerSample &sample) override;

private:
    struct Compensation {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
    };

    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t &value);
    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
    bool init_compensation();
    int32_t compensate_temperature(int32_t adc_t);
    uint32_t compensate_pressure(int32_t adc_p);

private:
    mc_rtos_hal::I2cBus &i2c_;
    mc_rtos_hal::Time &time_;
    mc_rtos_hal::GpioController &gpio_;
    Bme280Config config_;
    Compensation compensation_;
    int32_t t_fine_;
    uint32_t sequence_;
};

}  // namespace mc_experimental
