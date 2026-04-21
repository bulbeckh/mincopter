#include <dev/experimental/imu/bmi270/Bmi270ImuDevice.h>

#include <math.h>

#include <AP_Math.h>

#include <dev/experimental/imu/bmi270/Bmi270ConfigBlob.h>

namespace mc_experimental {

namespace {

constexpr uint8_t kBmi270ChipIdReg = 0x00;
constexpr uint8_t kBmi270ChipIdValue = 0x24;
constexpr uint8_t kBmi270StatusReg = 0x03;
constexpr uint8_t kBmi270AccelDataReg = 0x0C;
constexpr uint8_t kBmi270GyroDataReg = 0x12;
constexpr uint8_t kBmi270InternalStatusReg = 0x21;
constexpr uint8_t kBmi270TemperatureReg = 0x22;
constexpr uint8_t kBmi270AccelConfReg = 0x40;
constexpr uint8_t kBmi270AccelRangeReg = 0x41;
constexpr uint8_t kBmi270GyroConfReg = 0x42;
constexpr uint8_t kBmi270GyroRangeReg = 0x43;
constexpr uint8_t kBmi270Int1IoCtrlReg = 0x53;
constexpr uint8_t kBmi270IntLatchReg = 0x55;
constexpr uint8_t kBmi270IntMapDataReg = 0x58;
constexpr uint8_t kBmi270InitCtrlReg = 0x59;
constexpr uint8_t kBmi270InitAddr0Reg = 0x5B;
constexpr uint8_t kBmi270InitAddr1Reg = 0x5C;
constexpr uint8_t kBmi270InitDataReg = 0x5E;
constexpr uint8_t kBmi270PwrConfReg = 0x7C;
constexpr uint8_t kBmi270PwrCtrlReg = 0x7D;
constexpr uint8_t kBmi270CmdReg = 0x7E;

constexpr uint8_t kBmi270SoftResetCmd = 0xB6;
constexpr uint8_t kBmi270InitOk = 0x01;
constexpr uint8_t kBmi270DrdyAccMask = 0x80;
constexpr uint8_t kBmi270DrdyGyrMask = 0x40;
constexpr uint8_t kBmi270DataReadyInt = 0x04;

constexpr uint8_t kBmi270AccelNormalMode = 0x02;
constexpr uint8_t kBmi270GyroNormalMode = 0x02;
constexpr uint8_t kBmi270PerfFilterBit = 0x80;
constexpr uint8_t kBmi270GyroRange250Dps = 0x03;
constexpr uint8_t kBmi270AccelRange2G = 0x00;
constexpr uint8_t kBmi270DisableAdvancedPowerSave = 0x00;
constexpr uint8_t kBmi270NormalPowerConfig = 0x02;
constexpr uint8_t kBmi270EnableAccelGyroTemp = 0x0E;
constexpr uint8_t kBmi270Int1ActiveHighPushPull = 0x0A;
constexpr uint8_t kBmi270NonLatchedInterrupt = 0x00;

constexpr size_t kBmi270ConfigChunkSize = 32;

constexpr float kAccelScaleLsbPerG = 16384.0f;
constexpr float kGyroScaleLsbPerDegS = 131.072f;
constexpr float kTemperatureOffsetC = 23.0f;
constexpr float kTemperatureScaleLsbPerC = 512.0f;

}  // namespace

Bmi270ImuDevice::Bmi270ImuDevice(mc_rtos_hal::I2cBus &i2c,
                                 mc_rtos_hal::Time &time,
                                 mc_rtos_hal::GpioController &gpio,
                                 const Bmi270Config &config)
    : i2c_(i2c),
      time_(time),
      gpio_(gpio),
      config_(config),
      sequence_(0) {}

bool Bmi270ImuDevice::init() {
    return reset() && check_identity() && configure();
}

bool Bmi270ImuDevice::reset() {
    if (!write_register(kBmi270CmdReg, kBmi270SoftResetCmd)) {
        return false;
    }

    // Bosch specifies 450 us register access latency after reset; 1 ms is a safe RTOS-friendly delay.
    time_.delay_ms(2);
    return true;
}

bool Bmi270ImuDevice::check_identity() {
    uint8_t chip_id = 0;
    if (!read_register(kBmi270ChipIdReg, chip_id)) {
        return false;
    }

    return chip_id == kBmi270ChipIdValue;
}

bool Bmi270ImuDevice::configure() {
    uint8_t acc_odr = 0;
    uint8_t gyr_odr = 0;
    if (!encode_sample_rate(config_.sample_rate_hz, acc_odr, gyr_odr)) {
        return false;
    }

    if (!write_register(kBmi270PwrConfReg, kBmi270DisableAdvancedPowerSave)) {
        return false;
    }

    if (!write_register(kBmi270InitCtrlReg, 0x00)) {
        return false;
    }

    if (!upload_configuration_blob()) {
        return false;
    }

    if (!write_register(kBmi270InitCtrlReg, 0x01)) {
        return false;
    }

    time_.delay_ms(20);

    uint8_t internal_status = 0;
    if (!read_register(kBmi270InternalStatusReg, internal_status)) {
        return false;
    }
    if ((internal_status & 0x0F) != kBmi270InitOk) {
        return false;
    }

    const uint8_t accel_conf = static_cast<uint8_t>(kBmi270PerfFilterBit |
                                                    (kBmi270AccelNormalMode << 4) |
                                                    acc_odr);
    const uint8_t gyro_conf = static_cast<uint8_t>(kBmi270PerfFilterBit |
                                                   (kBmi270GyroNormalMode << 4) |
                                                   gyr_odr);

    if (!write_register(kBmi270AccelRangeReg, kBmi270AccelRange2G)) {
        return false;
    }
    if (!write_register(kBmi270GyroRangeReg, kBmi270GyroRange250Dps)) {
        return false;
    }
    if (!write_register(kBmi270AccelConfReg, accel_conf)) {
        return false;
    }
    if (!write_register(kBmi270GyroConfReg, gyro_conf)) {
        return false;
    }

    if (config_.has_data_ready_irq) {
        if (gpio_.pin(config_.data_ready_pin) == nullptr) {
            return false;
        }
        if (!write_register(kBmi270Int1IoCtrlReg, kBmi270Int1ActiveHighPushPull)) {
            return false;
        }
        if (!write_register(kBmi270IntLatchReg, kBmi270NonLatchedInterrupt)) {
            return false;
        }
        if (!write_register(kBmi270IntMapDataReg, kBmi270DataReadyInt)) {
            return false;
        }
    }

    if (!write_register(kBmi270PwrCtrlReg, kBmi270EnableAccelGyroTemp)) {
        return false;
    }
    if (!write_register(kBmi270PwrConfReg, kBmi270NormalPowerConfig)) {
        return false;
    }

    time_.delay_ms(5);
    return true;
}

bool Bmi270ImuDevice::has_data_ready_irq() const {
    return config_.has_data_ready_irq;
}

uint16_t Bmi270ImuDevice::data_ready_pin() const {
    return config_.data_ready_pin;
}

uint32_t Bmi270ImuDevice::sample_rate_hz() const {
    return config_.sample_rate_hz;
}

bool Bmi270ImuDevice::read_sample(ImuSample &sample) {
    uint8_t status = 0;
    if (!read_register(kBmi270StatusReg, status)) {
        sample.valid = false;
        return false;
    }

    uint8_t accel_raw[6] = {};
    uint8_t gyro_raw[6] = {};
    uint8_t temp_raw_bytes[2] = {};
    if (!read_registers(kBmi270AccelDataReg, accel_raw, sizeof(accel_raw)) ||
        !read_registers(kBmi270GyroDataReg, gyro_raw, sizeof(gyro_raw)) ||
        !read_registers(kBmi270TemperatureReg, temp_raw_bytes, sizeof(temp_raw_bytes))) {
        sample.valid = false;
        return false;
    }

    const int16_t ax = static_cast<int16_t>((static_cast<uint16_t>(accel_raw[1]) << 8) | accel_raw[0]);
    const int16_t ay = static_cast<int16_t>((static_cast<uint16_t>(accel_raw[3]) << 8) | accel_raw[2]);
    const int16_t az = static_cast<int16_t>((static_cast<uint16_t>(accel_raw[5]) << 8) | accel_raw[4]);
    const int16_t gx = static_cast<int16_t>((static_cast<uint16_t>(gyro_raw[1]) << 8) | gyro_raw[0]);
    const int16_t gy = static_cast<int16_t>((static_cast<uint16_t>(gyro_raw[3]) << 8) | gyro_raw[2]);
    const int16_t gz = static_cast<int16_t>((static_cast<uint16_t>(gyro_raw[5]) << 8) | gyro_raw[4]);
    const int16_t temp_raw = static_cast<int16_t>((static_cast<uint16_t>(temp_raw_bytes[1]) << 8) | temp_raw_bytes[0]);

    sample.timestamp_us = time_.micros();
    sample.sequence = ++sequence_;
    sample.accel_m_s2[0] = (static_cast<float>(ax) / kAccelScaleLsbPerG) * GRAVITY_MSS;
    sample.accel_m_s2[1] = (static_cast<float>(ay) / kAccelScaleLsbPerG) * GRAVITY_MSS;
    sample.accel_m_s2[2] = (static_cast<float>(az) / kAccelScaleLsbPerG) * GRAVITY_MSS;
    sample.gyro_rad_s[0] = (static_cast<float>(gx) / kGyroScaleLsbPerDegS) * DEG_TO_RAD;
    sample.gyro_rad_s[1] = (static_cast<float>(gy) / kGyroScaleLsbPerDegS) * DEG_TO_RAD;
    sample.gyro_rad_s[2] = (static_cast<float>(gz) / kGyroScaleLsbPerDegS) * DEG_TO_RAD;
    sample.temperature_c = (temp_raw == INT16_MIN) ? NAN
                                                   : (kTemperatureOffsetC +
                                                      (static_cast<float>(temp_raw) / kTemperatureScaleLsbPerC));
    sample.valid = ((status & (kBmi270DrdyAccMask | kBmi270DrdyGyrMask)) ==
                    (kBmi270DrdyAccMask | kBmi270DrdyGyrMask)) ||
                   !config_.has_data_ready_irq;
    return sample.valid;
}

bool Bmi270ImuDevice::write_register(uint8_t reg, uint8_t value) {
    return write_registers(reg, &value, 1);
}

bool Bmi270ImuDevice::write_registers(uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t buffer[kBmi270ConfigChunkSize + 1] = {};
    if (len > kBmi270ConfigChunkSize) {
        return false;
    }

    buffer[0] = reg;
    for (size_t i = 0; i < len; ++i) {
        buffer[i + 1] = data[i];
    }

    const mc_rtos_hal::Timeout timeout{20};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }

    const mc_rtos_hal::Status status = i2c_.write(config_.i2c_address, buffer, len + 1U, timeout);
    i2c_.unlock();

    if (status != mc_rtos_hal::Status::Ok) {
        return false;
    }

    delay_after_write();
    return true;
}

bool Bmi270ImuDevice::read_register(uint8_t reg, uint8_t &value) {
    return read_registers(reg, &value, 1);
}

bool Bmi270ImuDevice::read_registers(uint8_t reg, uint8_t *data, uint8_t len) {
    const mc_rtos_hal::Timeout timeout{20};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }

    const mc_rtos_hal::Status status = i2c_.read_registers(config_.i2c_address, reg, data, len, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

bool Bmi270ImuDevice::upload_configuration_blob() {
    for (size_t offset = 0; offset < kBmi270ConfigBlobSize; offset += kBmi270ConfigChunkSize) {
        const uint16_t init_addr = static_cast<uint16_t>(offset / 2U);
        const uint8_t init_addr_lsb = static_cast<uint8_t>(init_addr & 0xFFU);
        const uint8_t init_addr_msb = static_cast<uint8_t>((init_addr >> 8) & 0x0FU);
        const size_t chunk_size = ((offset + kBmi270ConfigChunkSize) <= kBmi270ConfigBlobSize)
                                      ? kBmi270ConfigChunkSize
                                      : (kBmi270ConfigBlobSize - offset);

        if (!write_register(kBmi270InitAddr0Reg, init_addr_lsb) ||
            !write_register(kBmi270InitAddr1Reg, init_addr_msb) ||
            !write_registers(kBmi270InitDataReg, &kBmi270ConfigBlob[offset], chunk_size)) {
            return false;
        }
    }

    return true;
}

bool Bmi270ImuDevice::encode_sample_rate(uint32_t sample_rate_hz, uint8_t &acc_odr, uint8_t &gyr_odr) const {
    switch (sample_rate_hz) {
    case 25:
        acc_odr = 0x06;
        gyr_odr = 0x06;
        return true;
    case 50:
        acc_odr = 0x07;
        gyr_odr = 0x07;
        return true;
    case 100:
        acc_odr = 0x08;
        gyr_odr = 0x08;
        return true;
    case 200:
        acc_odr = 0x09;
        gyr_odr = 0x09;
        return true;
    case 400:
        acc_odr = 0x0A;
        gyr_odr = 0x0A;
        return true;
    case 800:
        acc_odr = 0x0B;
        gyr_odr = 0x0B;
        return true;
    case 1600:
        acc_odr = 0x0C;
        gyr_odr = 0x0C;
        return true;
    default:
        return false;
    }
}

void Bmi270ImuDevice::delay_after_write() {
    time_.delay_ms(1);
}

}  // namespace mc_experimental
