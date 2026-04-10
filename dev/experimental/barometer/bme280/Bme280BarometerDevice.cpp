#include <dev/experimental/barometer/bme280/Bme280BarometerDevice.h>

namespace mc_experimental {

namespace {

constexpr uint8_t kBme280Id = 0xD0;
constexpr uint8_t kBme280Reset = 0xE0;
constexpr uint8_t kBme280PressStart = 0xF7;
constexpr uint8_t kBme280CtrlMeas = 0xF4;
constexpr uint8_t kBme280DigStart = 0x88;

constexpr uint8_t kBme280DeviceId = 0x60;
constexpr uint8_t kBme280Oversample1 = 0x01;
constexpr uint8_t kBme280ModeNormal = 0x03;

}  // namespace

Bme280BarometerDevice::Bme280BarometerDevice(mc_rtos_hal::I2cBus &i2c,
                                             mc_rtos_hal::Time &time,
                                             mc_rtos_hal::GpioController &gpio,
                                             const Bme280Config &config)
    : i2c_(i2c),
      time_(time),
      gpio_(gpio),
      config_(config),
      compensation_{},
      t_fine_(0),
      sequence_(0) {}

bool Bme280BarometerDevice::init() {
    return reset() && init_compensation() && configure();
}

bool Bme280BarometerDevice::reset() {
    if (!write_register(kBme280Reset, 0xB6)) {
        return false;
    }
    time_.delay_ms(100);
    uint8_t id = 0;
    if (!read_register(kBme280Id, id)) {
        return false;
    }
    return id == kBme280DeviceId;
}

bool Bme280BarometerDevice::configure() {
    const uint8_t ctrl_meas = static_cast<uint8_t>((kBme280Oversample1 << 5) | (kBme280Oversample1 << 2) | kBme280ModeNormal);
    return write_register(kBme280CtrlMeas, ctrl_meas);
}

bool Bme280BarometerDevice::has_data_ready_irq() const {
    return config_.has_data_ready_irq;
}

uint16_t Bme280BarometerDevice::data_ready_pin() const {
    return config_.data_ready_pin;
}

uint32_t Bme280BarometerDevice::sample_rate_hz() const {
    return config_.sample_rate_hz;
}

bool Bme280BarometerDevice::read_sample(BarometerSample &sample) {
    uint8_t raw[6] = {};
    if (!read_registers(kBme280PressStart, raw, sizeof(raw))) {
        sample.valid = false;
        return false;
    }

    const int32_t pressure_raw = static_cast<int32_t>((raw[0] << 12) | (raw[1] << 4) | ((raw[2] >> 4) & 0x0F));
    const int32_t temp_raw = static_cast<int32_t>((raw[3] << 12) | (raw[4] << 4) | ((raw[5] >> 4) & 0x0F));

    const int32_t temp_centideg = compensate_temperature(temp_raw);
    const uint32_t pressure_pa = compensate_pressure(pressure_raw);

    sample.timestamp_us = time_.micros();
    sample.sequence = ++sequence_;
    sample.temperature_c = static_cast<float>(temp_centideg) / 100.0f;
    sample.pressure_pa = static_cast<float>(pressure_pa);
    sample.valid = true;
    return true;
}

bool Bme280BarometerDevice::write_register(uint8_t reg, uint8_t value) {
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.write_register(config_.i2c_address, reg, value, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

bool Bme280BarometerDevice::read_register(uint8_t reg, uint8_t &value) {
    return read_registers(reg, &value, 1);
}

bool Bme280BarometerDevice::read_registers(uint8_t reg, uint8_t *data, uint8_t len) {
    const mc_rtos_hal::Timeout timeout{10};
    if (i2c_.lock(timeout) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    const mc_rtos_hal::Status status = i2c_.read_registers(config_.i2c_address, reg, data, len, timeout);
    i2c_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

bool Bme280BarometerDevice::init_compensation() {
    uint8_t data[24] = {};
    if (!read_registers(kBme280DigStart, data, sizeof(data))) {
        return false;
    }

    compensation_.dig_T1 = static_cast<uint16_t>((data[1] << 8) | data[0]);
    compensation_.dig_T2 = static_cast<int16_t>((data[3] << 8) | data[2]);
    compensation_.dig_T3 = static_cast<int16_t>((data[5] << 8) | data[4]);
    compensation_.dig_P1 = static_cast<uint16_t>((data[7] << 8) | data[6]);
    compensation_.dig_P2 = static_cast<int16_t>((data[9] << 8) | data[8]);
    compensation_.dig_P3 = static_cast<int16_t>((data[11] << 8) | data[10]);
    compensation_.dig_P4 = static_cast<int16_t>((data[13] << 8) | data[12]);
    compensation_.dig_P5 = static_cast<int16_t>((data[15] << 8) | data[14]);
    compensation_.dig_P6 = static_cast<int16_t>((data[17] << 8) | data[16]);
    compensation_.dig_P7 = static_cast<int16_t>((data[19] << 8) | data[18]);
    compensation_.dig_P8 = static_cast<int16_t>((data[21] << 8) | data[20]);
    compensation_.dig_P9 = static_cast<int16_t>((data[23] << 8) | data[22]);
    return true;
}

int32_t Bme280BarometerDevice::compensate_temperature(int32_t adc_t) {
    int32_t var1 = ((((adc_t >> 3) - (static_cast<int32_t>(compensation_.dig_T1) << 1))) *
                    static_cast<int32_t>(compensation_.dig_T2)) >>
                   11;
    int32_t var2 = (((((adc_t >> 4) - static_cast<int32_t>(compensation_.dig_T1)) *
                      ((adc_t >> 4) - static_cast<int32_t>(compensation_.dig_T1))) >>
                     12) *
                    static_cast<int32_t>(compensation_.dig_T3)) >>
                   14;
    t_fine_ = var1 + var2;
    return (t_fine_ * 5 + 128) >> 8;
}

uint32_t Bme280BarometerDevice::compensate_pressure(int32_t adc_p) {
    int32_t var1 = (t_fine_ >> 1) - 64000;
    int32_t var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * static_cast<int32_t>(compensation_.dig_P6);
    var2 += (var1 * static_cast<int32_t>(compensation_.dig_P5)) << 1;
    var2 = (var2 >> 2) + (static_cast<int32_t>(compensation_.dig_P4) << 16);
    var1 = (((compensation_.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((static_cast<int32_t>(compensation_.dig_P2) * var1) >> 1)) >>
           18;
    var1 = ((32768 + var1) * static_cast<int32_t>(compensation_.dig_P1)) >> 15;
    if (var1 == 0) {
        return 0;
    }

    uint32_t p = static_cast<uint32_t>((1048576 - adc_p) - (var2 >> 12)) * 3125U;
    if (p < 0x80000000U) {
        p = (p << 1) / static_cast<uint32_t>(var1);
    } else {
        p = (p / static_cast<uint32_t>(var1)) * 2U;
    }

    var1 = (static_cast<int32_t>(compensation_.dig_P9) * static_cast<int32_t>(((p >> 3) * (p >> 3)) >> 13)) >> 12;
    var2 = (static_cast<int32_t>(p >> 2) * static_cast<int32_t>(compensation_.dig_P8)) >> 13;
    p = static_cast<uint32_t>(static_cast<int32_t>(p) + ((var1 + var2 + compensation_.dig_P7) >> 4));
    return p;
}

}  // namespace mc_experimental
