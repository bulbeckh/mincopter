#include <dev/experimental/imu/mpu6050/Mpu6050ImuDevice.h>

#include <AP_Math.h>

namespace mc_experimental {

namespace {

constexpr uint8_t kMpu6050WhoAmI = 0x75;
constexpr uint8_t kMpu6050WhoAmIValue = 0x68;
constexpr uint8_t kMpu6050PwrMgmt1 = 0x6B;
constexpr uint8_t kMpu6050SmplrtDiv = 0x19;
constexpr uint8_t kMpu6050Config = 0x1A;
constexpr uint8_t kMpu6050GyroConfig = 0x1B;
constexpr uint8_t kMpu6050AccelConfig = 0x1C;
constexpr uint8_t kMpu6050IntEnable = 0x38;
constexpr uint8_t kMpu6050AccelXoutH = 0x3B;
constexpr uint8_t kMpu6050TempOutH = 0x41;
constexpr uint8_t kMpu6050GyroXoutH = 0x43;

constexpr float kAccelScaleLsbPerG = 16384.0f;   // +-2g
constexpr float kGyroScaleLsbPerDegS = 131.0f;   // +-250 deg/s
constexpr float kTempScale = 340.0f;
constexpr float kTempOffset = 36.53f;

}  // namespace

Mpu6050ImuDevice::Mpu6050ImuDevice(mc_rtos_hal::I2cBus &i2c,
                                   mc_rtos_hal::Time &time,
                                   mc_rtos_hal::GpioController &gpio,
                                   const Mpu6050Config &config)
    : i2c_(i2c),
      time_(time),
      gpio_(gpio),
      config_(config),
      sequence_(0) {}

bool Mpu6050ImuDevice::init() {
    return reset() && check_identity() && configure();
}

bool Mpu6050ImuDevice::reset() {
    if (!write_register(kMpu6050PwrMgmt1, 0x80)) {
        return false;
    }
    time_.delay_ms(100);
    if (!write_register(kMpu6050PwrMgmt1, 0x00)) {
        return false;
    }
    time_.delay_ms(10);
    return true;
}

bool Mpu6050ImuDevice::check_identity() {
    uint8_t whoami = 0;
    if (!read_register(kMpu6050WhoAmI, whoami)) {
        return false;
    }
    return whoami == kMpu6050WhoAmIValue;
}

bool Mpu6050ImuDevice::configure() {
    uint8_t smplrt_div = 0;
    if (config_.sample_rate_hz > 0 && config_.sample_rate_hz <= 1000) {
        smplrt_div = static_cast<uint8_t>((1000U / config_.sample_rate_hz) - 1U);
    }

    // DLPF config = 0x03 (~44Hz gyro / 42Hz accel), gyro FS = +-250 dps, accel FS = +-2g.
    if (!write_register(kMpu6050SmplrtDiv, smplrt_div)) {
        return false;
    }
    if (!write_register(kMpu6050Config, 0x03)) {
        return false;
    }
    if (!write_register(kMpu6050GyroConfig, 0x00)) {
        return false;
    }
    if (!write_register(kMpu6050AccelConfig, 0x00)) {
        return false;
    }
    if (config_.has_data_ready_irq && !write_register(kMpu6050IntEnable, 0x01)) {
        return false;
    }

    return true;
}

bool Mpu6050ImuDevice::has_data_ready_irq() const {
    return config_.has_data_ready_irq;
}

uint16_t Mpu6050ImuDevice::data_ready_pin() const {
    return config_.data_ready_pin;
}

uint32_t Mpu6050ImuDevice::sample_rate_hz() const {
    return config_.sample_rate_hz;
}

bool Mpu6050ImuDevice::read_sample(ImuSample &sample) {
    uint8_t raw[14] = {};
    if (!read_registers(kMpu6050AccelXoutH, raw, sizeof(raw))) {
        sample.valid = false;
        return false;
    }

    const int16_t ax = static_cast<int16_t>((raw[0] << 8) | raw[1]);
    const int16_t ay = static_cast<int16_t>((raw[2] << 8) | raw[3]);
    const int16_t az = static_cast<int16_t>((raw[4] << 8) | raw[5]);
    const int16_t temp_raw = static_cast<int16_t>((raw[6] << 8) | raw[7]);
    const int16_t gx = static_cast<int16_t>((raw[8] << 8) | raw[9]);
    const int16_t gy = static_cast<int16_t>((raw[10] << 8) | raw[11]);
    const int16_t gz = static_cast<int16_t>((raw[12] << 8) | raw[13]);

    sample.timestamp_us = time_.micros();
    sample.sequence = ++sequence_;
    sample.accel_m_s2[0] = (static_cast<float>(ax) / kAccelScaleLsbPerG) * GRAVITY_MSS;
    sample.accel_m_s2[1] = (static_cast<float>(ay) / kAccelScaleLsbPerG) * GRAVITY_MSS;
    sample.accel_m_s2[2] = (static_cast<float>(az) / kAccelScaleLsbPerG) * GRAVITY_MSS;
    sample.gyro_rad_s[0] = (static_cast<float>(gx) / kGyroScaleLsbPerDegS) * DEG_TO_RAD;
    sample.gyro_rad_s[1] = (static_cast<float>(gy) / kGyroScaleLsbPerDegS) * DEG_TO_RAD;
    sample.gyro_rad_s[2] = (static_cast<float>(gz) / kGyroScaleLsbPerDegS) * DEG_TO_RAD;
    sample.temperature_c = (static_cast<float>(temp_raw) / kTempScale) + kTempOffset;
    sample.valid = true;
    return true;
}

bool Mpu6050ImuDevice::write_register(uint8_t reg, uint8_t value) {
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.write_register(config_.i2c_address, reg, value, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

bool Mpu6050ImuDevice::read_register(uint8_t reg, uint8_t &value) {
    return read_registers(reg, &value, 1);
}

bool Mpu6050ImuDevice::read_registers(uint8_t reg, uint8_t *data, uint8_t len) {
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.read_registers(config_.i2c_address, reg, data, len, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

}  // namespace mc_experimental
