#include <dev/experimental/compass/icm20948/Icm20948CompassDevice.h>

namespace mc_experimental {

namespace {

constexpr uint8_t kRegBankSel = 0x7F;

constexpr uint8_t kBank0 = 0;
constexpr uint8_t kBank3 = 3;

constexpr uint8_t kWhoAmI = 0x00;
constexpr uint8_t kWhoAmIValue = 0xEA;
constexpr uint8_t kUserCtrl = 0x03;
constexpr uint8_t kLpConfig = 0x05;
constexpr uint8_t kPwrMgmt1 = 0x06;
constexpr uint8_t kPwrMgmt2 = 0x07;
constexpr uint8_t kExtSlvSensData00 = 0x3B;

constexpr uint8_t kI2cMstOdrConfig = 0x00;
constexpr uint8_t kI2cMstCtrl = 0x01;
constexpr uint8_t kI2cSlv0Addr = 0x03;
constexpr uint8_t kI2cSlv0Reg = 0x04;
constexpr uint8_t kI2cSlv0Ctrl = 0x05;
constexpr uint8_t kI2cSlv0Do = 0x06;

constexpr uint8_t kPwrMgmt1DeviceReset = 0x80;
constexpr uint8_t kPwrMgmt1ClockAuto = 0x01;
constexpr uint8_t kUserCtrlI2cMstEn = 0x20;
constexpr uint8_t kUserCtrlI2cMstRst = 0x02;
constexpr uint8_t kLpConfigI2cMstCycle = 0x40;

constexpr uint8_t kAk09916Address = 0x0C;
constexpr uint8_t kAk09916Wia2 = 0x01;
constexpr uint8_t kAk09916Wia2Value = 0x09;
constexpr uint8_t kAk09916St1 = 0x10;
constexpr uint8_t kAk09916Cntl2 = 0x31;
constexpr uint8_t kAk09916Cntl3 = 0x32;
constexpr uint8_t kAk09916ModePowerDown = 0x00;
constexpr uint8_t kAk09916ModeContinuous10Hz = 0x02;
constexpr uint8_t kAk09916ModeContinuous20Hz = 0x04;
constexpr uint8_t kAk09916ModeContinuous50Hz = 0x06;
constexpr uint8_t kAk09916ModeContinuous100Hz = 0x08;
constexpr uint8_t kAk09916Reset = 0x01;
constexpr uint8_t kAk09916DataReady = 0x01;
constexpr uint8_t kAk09916DataOverrun = 0x02;

constexpr float kGaussPerCount = 0.0015f;  // AK09916 sensitivity is 0.15 uT/LSB.

uint8_t ak09916_mode_for_rate(uint32_t sample_rate_hz) {
    if (sample_rate_hz >= 100U) {
        return kAk09916ModeContinuous100Hz;
    }
    if (sample_rate_hz >= 50U) {
        return kAk09916ModeContinuous50Hz;
    }
    if (sample_rate_hz >= 20U) {
        return kAk09916ModeContinuous20Hz;
    }
    return kAk09916ModeContinuous10Hz;
}

int16_t le_i16(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>(static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
}

}  // namespace

Icm20948CompassDevice::Icm20948CompassDevice(mc_rtos_hal::I2cBus &i2c,
                                             mc_rtos_hal::Time &time,
                                             mc_rtos_hal::GpioController &gpio,
                                             const Icm20948CompassConfig &config)
    : i2c_(i2c),
      time_(time),
      gpio_(gpio),
      config_(config),
      sequence_(0),
      selected_bank_(0xFF) {}

bool Icm20948CompassDevice::init() {
    return reset() && configure();
}

bool Icm20948CompassDevice::reset() {
    if (!select_bank(kBank0)) {
        return false;
    }
    if (!write_register(kPwrMgmt1, kPwrMgmt1DeviceReset)) {
        return false;
    }
    selected_bank_ = 0xFF;
    time_.delay_ms(100);

    if (!select_bank(kBank0)) {
        return false;
    }
    if (!write_register(kPwrMgmt1, kPwrMgmt1ClockAuto)) {
        return false;
    }
    if (!write_register(kPwrMgmt2, 0x00)) {
        return false;
    }
    time_.delay_ms(10);
    return true;
}

bool Icm20948CompassDevice::configure() {
    if (!select_bank(kBank0)) {
        return false;
    }

    uint8_t whoami = 0;
    if (!read_register(kWhoAmI, whoami) || whoami != kWhoAmIValue) {
        return false;
    }

    if (!write_register(kUserCtrl, kUserCtrlI2cMstRst)) {
        return false;
    }
    time_.delay_ms(10);
    if (!write_register(kUserCtrl, kUserCtrlI2cMstEn)) {
        return false;
    }
    if (!write_register(kLpConfig, kLpConfigI2cMstCycle)) {
        return false;
    }

    if (!select_bank(kBank3)) {
        return false;
    }
    if (!write_register(kI2cMstOdrConfig, 0x03)) {
        return false;
    }
    if (!write_register(kI2cMstCtrl, 0x07)) {
        return false;
    }

    uint8_t magnetometer_id = 0;
    if (!read_ak09916_register(kAk09916Wia2, magnetometer_id) || magnetometer_id != kAk09916Wia2Value) {
        return false;
    }

    if (!write_ak09916_register(kAk09916Cntl3, kAk09916Reset)) {
        return false;
    }
    time_.delay_ms(10);
    if (!write_ak09916_register(kAk09916Cntl2, kAk09916ModePowerDown)) {
        return false;
    }
    time_.delay_ms(10);
    if (!write_ak09916_register(kAk09916Cntl2, ak09916_mode_for_rate(config_.sample_rate_hz))) {
        return false;
    }
    time_.delay_ms(10);

    return configure_external_sensor_read();
}

bool Icm20948CompassDevice::has_data_ready_irq() const {
    return config_.has_data_ready_irq;
}

uint16_t Icm20948CompassDevice::data_ready_pin() const {
    return config_.data_ready_pin;
}

uint32_t Icm20948CompassDevice::sample_rate_hz() const {
    return config_.sample_rate_hz;
}

bool Icm20948CompassDevice::read_sample(CompassSample &sample) {
    uint8_t raw[8] = {};
    if (!select_bank(kBank0) || !read_registers(kExtSlvSensData00, raw, sizeof(raw))) {
        sample.valid = false;
        return false;
    }

    const uint8_t st1 = raw[0];
    const int16_t mx = le_i16(raw[1], raw[2]);
    const int16_t my = le_i16(raw[3], raw[4]);
    const int16_t mz = le_i16(raw[5], raw[6]);
    const uint8_t st2 = raw[7];

    sample.timestamp_us = time_.micros();
    sample.sequence = ++sequence_;
    sample.field_gauss[0] = static_cast<float>(mx) * kGaussPerCount;
    sample.field_gauss[1] = static_cast<float>(my) * kGaussPerCount;
    sample.field_gauss[2] = static_cast<float>(mz) * kGaussPerCount;
    (void)st2;
    sample.valid = (st1 & (kAk09916DataReady | kAk09916DataOverrun)) != 0U;
    return sample.valid;
}

bool Icm20948CompassDevice::select_bank(uint8_t bank) {
    if (selected_bank_ == bank) {
        return true;
    }

    const uint8_t bank_value = static_cast<uint8_t>((bank & 0x03U) << 4);
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.write_register(config_.i2c_address, kRegBankSel, bank_value, timeout);
    i2c_.unlock();
    if (status == mc_rtos_hal::Status::Ok) {
        selected_bank_ = bank;
        return true;
    }
    return false;
}

bool Icm20948CompassDevice::write_register(uint8_t reg, uint8_t value) {
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.write_register(config_.i2c_address, reg, value, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

bool Icm20948CompassDevice::read_register(uint8_t reg, uint8_t &value) {
    return read_registers(reg, &value, 1);
}

bool Icm20948CompassDevice::read_registers(uint8_t reg, uint8_t *data, uint8_t len) {
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.read_registers(config_.i2c_address, reg, data, len, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

bool Icm20948CompassDevice::write_ak09916_register(uint8_t reg, uint8_t value) {
    return select_bank(kBank3) &&
           write_register(kI2cSlv0Addr, kAk09916Address) &&
           write_register(kI2cSlv0Reg, reg) &&
           write_register(kI2cSlv0Do, value) &&
           write_register(kI2cSlv0Ctrl, 0x81) &&
           (time_.delay_ms(10), true);
}

bool Icm20948CompassDevice::read_ak09916_register(uint8_t reg, uint8_t &value) {
    if (!select_bank(kBank3) ||
        !write_register(kI2cSlv0Addr, static_cast<uint8_t>(0x80U | kAk09916Address)) ||
        !write_register(kI2cSlv0Reg, reg) ||
        !write_register(kI2cSlv0Ctrl, 0x81)) {
        return false;
    }

    time_.delay_ms(10);
    return select_bank(kBank0) && read_register(kExtSlvSensData00, value);
}

bool Icm20948CompassDevice::configure_external_sensor_read() {
    return select_bank(kBank3) &&
           write_register(kI2cSlv0Addr, static_cast<uint8_t>(0x80U | kAk09916Address)) &&
           write_register(kI2cSlv0Reg, kAk09916St1) &&
           write_register(kI2cSlv0Ctrl, 0x88) &&
           (time_.delay_ms(10), true);
}

}  // namespace mc_experimental
