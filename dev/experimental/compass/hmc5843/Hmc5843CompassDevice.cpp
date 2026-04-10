#include <dev/experimental/compass/hmc5843/Hmc5843CompassDevice.h>

namespace mc_experimental {

namespace {

constexpr uint8_t kCompassConfigA = 0x00;
constexpr uint8_t kCompassConfigB = 0x01;
constexpr uint8_t kCompassMode = 0x02;
constexpr uint8_t kCompassDataStart = 0x03;

constexpr uint8_t kNormalOperation = 0x10;
constexpr uint8_t kContinuousConversion = 0x00;
constexpr uint8_t kGain1090 = 0x20;

constexpr float kGaussPerCount = 1.0f / 1090.0f;

uint8_t data_rate_bits(uint32_t sample_rate_hz) {
    if (sample_rate_hz >= 75U) {
        return 0x06;
    }
    if (sample_rate_hz >= 30U) {
        return 0x05;
    }
    if (sample_rate_hz >= 15U) {
        return 0x04;
    }
    if (sample_rate_hz >= 8U) {
        return 0x03;
    }
    return 0x02;
}

}  // namespace

Hmc5843CompassDevice::Hmc5843CompassDevice(mc_rtos_hal::I2cBus &i2c,
                                           mc_rtos_hal::Time &time,
                                           mc_rtos_hal::GpioController &gpio,
                                           const Hmc5843Config &config)
    : i2c_(i2c),
      time_(time),
      gpio_(gpio),
      config_(config),
      sequence_(0) {}

bool Hmc5843CompassDevice::init() {
    return reset() && configure();
}

bool Hmc5843CompassDevice::reset() {
    return write_register(kCompassMode, kContinuousConversion);
}

bool Hmc5843CompassDevice::configure() {
    const uint8_t config_a = static_cast<uint8_t>((0x03U << 5) | (data_rate_bits(config_.sample_rate_hz) << 2) | kNormalOperation);
    if (!write_register(kCompassConfigA, config_a)) {
        return false;
    }
    if (!write_register(kCompassConfigB, kGain1090)) {
        return false;
    }
    if (!write_register(kCompassMode, kContinuousConversion)) {
        return false;
    }
    time_.delay_ms(10);
    return true;
}

bool Hmc5843CompassDevice::has_data_ready_irq() const {
    return config_.has_data_ready_irq;
}

uint16_t Hmc5843CompassDevice::data_ready_pin() const {
    return config_.data_ready_pin;
}

uint32_t Hmc5843CompassDevice::sample_rate_hz() const {
    return config_.sample_rate_hz;
}

bool Hmc5843CompassDevice::read_sample(CompassSample &sample) {
    uint8_t raw[6] = {};
    if (!read_registers(kCompassDataStart, raw, sizeof(raw))) {
        sample.valid = false;
        return false;
    }

    const int16_t rx = static_cast<int16_t>((raw[0] << 8) | raw[1]);
    const int16_t rz = static_cast<int16_t>((raw[2] << 8) | raw[3]);
    const int16_t ry = static_cast<int16_t>((raw[4] << 8) | raw[5]);

    sample.timestamp_us = time_.micros();
    sample.sequence = ++sequence_;
    sample.field_gauss[0] = static_cast<float>(rx) * kGaussPerCount;
    sample.field_gauss[1] = static_cast<float>(ry) * kGaussPerCount;
    sample.field_gauss[2] = static_cast<float>(rz) * kGaussPerCount;
    sample.valid = !(rx == -4096 || ry == -4096 || rz == -4096);
    return sample.valid;
}

bool Hmc5843CompassDevice::write_register(uint8_t reg, uint8_t value) {
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.write_register(config_.i2c_address, reg, value, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

bool Hmc5843CompassDevice::read_registers(uint8_t reg, uint8_t *data, uint8_t len) {
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.read_registers(config_.i2c_address, reg, data, len, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

}  // namespace mc_experimental
