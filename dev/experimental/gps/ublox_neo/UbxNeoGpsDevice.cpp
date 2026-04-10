#include <dev/experimental/gps/ublox_neo/UbxNeoGpsDevice.h>

namespace mc_experimental {

namespace {

constexpr uint8_t kCfgRateClass = 0x06;
constexpr uint8_t kCfgRateId = 0x08;

void store_u16_le(uint8_t *dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFFU);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

}  // namespace

UbxNeoGpsDevice::UbxNeoGpsDevice(mc_rtos_hal::UartPort &uart,
                                 mc_rtos_hal::Time &time,
                                 const UbxNeoGpsConfig &config)
    : uart_(uart),
      time_(time),
      config_(config),
      step_(0),
      msg_class_(0),
      msg_id_(0),
      payload_length_(0),
      payload_counter_(0),
      ck_a_(0),
      ck_b_(0),
      buffer_{},
      latest_fix_{},
      next_fix_(GpsFixType::None),
      new_position_(false),
      new_speed_(false),
      sequence_(0) {}

bool UbxNeoGpsDevice::init() {
    mc_rtos_hal::UartConfig uart_cfg{};
    uart_cfg.baud_rate = config_.baud_rate;
    uart_cfg.rx_buffer_size = 256;
    uart_cfg.tx_buffer_size = 128;
    if (uart_.configure(uart_cfg) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    return configure();
}

bool UbxNeoGpsDevice::configure() {
    // Configure navigation rate to approximately expected_fix_rate_hz.
    const uint16_t rate_ms = static_cast<uint16_t>(1000U / (config_.expected_fix_rate_hz == 0 ? 1U : config_.expected_fix_rate_hz));
    uint8_t payload[6] = {};
    store_u16_le(&payload[0], rate_ms);
    store_u16_le(&payload[2], 1);
    store_u16_le(&payload[4], 0);

    uint8_t packet[6 + sizeof(payload) + 2] = {};
    packet[0] = PREAMBLE1;
    packet[1] = PREAMBLE2;
    packet[2] = kCfgRateClass;
    packet[3] = kCfgRateId;
    store_u16_le(&packet[4], sizeof(payload));
    for (size_t i = 0; i < sizeof(payload); ++i) {
        packet[6 + i] = payload[i];
    }

    uint8_t cka = 0;
    uint8_t ckb = 0;
    for (size_t i = 2; i < 6 + sizeof(payload); ++i) {
        cka = static_cast<uint8_t>(cka + packet[i]);
        ckb = static_cast<uint8_t>(ckb + cka);
    }
    packet[6 + sizeof(payload)] = cka;
    packet[7 + sizeof(payload)] = ckb;

    size_t written = 0;
    return uart_.write(packet, sizeof(packet), {50}, &written) == mc_rtos_hal::Status::Ok;
}

uint32_t UbxNeoGpsDevice::expected_fix_rate_hz() const {
    return config_.expected_fix_rate_hz;
}

bool UbxNeoGpsDevice::service(GpsFix &fix, bool &fix_updated) {
    fix_updated = false;

    uint8_t rx_buf[64] = {};
    size_t read_len = 0;
    if (!read_chunk(rx_buf, sizeof(rx_buf), read_len)) {
        return false;
    }

    for (size_t i = 0; i < read_len; ++i) {
        if (!process_byte(rx_buf[i], fix, fix_updated)) {
            return false;
        }
    }

    return true;
}

bool UbxNeoGpsDevice::read_chunk(uint8_t *buffer, size_t capacity, size_t &read_len) {
    read_len = 0;
    return uart_.read(buffer, capacity, {1}, &read_len) == mc_rtos_hal::Status::Ok ||
           read_len > 0;
}

bool UbxNeoGpsDevice::process_byte(uint8_t byte, GpsFix &fix, bool &fix_updated) {
reset:
    switch (step_) {
    case 0:
        if (byte == PREAMBLE1) {
            step_ = 1;
        }
        break;
    case 1:
        if (byte == PREAMBLE2) {
            step_ = 2;
            break;
        }
        step_ = 0;
        goto reset;
    case 2:
        msg_class_ = byte;
        ck_a_ = ck_b_ = byte;
        step_ = 3;
        break;
    case 3:
        msg_id_ = byte;
        update_checksum(byte);
        step_ = 4;
        break;
    case 4:
        payload_length_ = byte;
        update_checksum(byte);
        step_ = 5;
        break;
    case 5:
        payload_length_ |= static_cast<uint16_t>(byte << 8);
        update_checksum(byte);
        payload_counter_ = 0;
        if (payload_length_ > sizeof(buffer_.bytes)) {
            step_ = 0;
            return true;
        }
        step_ = 6;
        break;
    case 6:
        buffer_.bytes[payload_counter_++] = byte;
        update_checksum(byte);
        if (payload_counter_ == payload_length_) {
            step_ = 7;
        }
        break;
    case 7:
        if (ck_a_ != byte) {
            step_ = 0;
            goto reset;
        }
        step_ = 8;
        break;
    case 8:
        step_ = 0;
        if (ck_b_ != byte) {
            return true;
        }
        return parse_message(fix, fix_updated);
    default:
        step_ = 0;
        break;
    }

    return true;
}

bool UbxNeoGpsDevice::parse_message(GpsFix &fix, bool &fix_updated) {
    if (msg_class_ != CLASS_NAV) {
        return true;
    }

    switch (msg_id_) {
    case MSG_STATUS:
        if (buffer_.status.fix_status & NAV_STATUS_FIX_VALID) {
            if (buffer_.status.fix_type == FIX_3D) {
                next_fix_ = GpsFixType::Fix3D;
            } else if (buffer_.status.fix_type == FIX_2D) {
                next_fix_ = GpsFixType::Fix2D;
            } else {
                next_fix_ = GpsFixType::None;
            }
        } else {
            next_fix_ = GpsFixType::None;
        }
        break;

    case MSG_POSLLH:
        latest_fix_.longitude_e7 = buffer_.posllh.longitude;
        latest_fix_.latitude_e7 = buffer_.posllh.latitude;
        latest_fix_.altitude_cm = buffer_.posllh.altitude_msl / 10;
        latest_fix_.fix_type = next_fix_;
        latest_fix_.valid = (next_fix_ != GpsFixType::None);
        new_position_ = true;
        break;

    case MSG_VELNED:
        latest_fix_.vel_north_cm_s = buffer_.velned.ned_north;
        latest_fix_.vel_east_cm_s = buffer_.velned.ned_east;
        latest_fix_.vel_down_cm_s = buffer_.velned.ned_down;
        latest_fix_.ground_speed_cm_s = buffer_.velned.speed_2d;
        latest_fix_.heading_cd = static_cast<uint16_t>(buffer_.velned.heading_2d / 1000);
        new_speed_ = true;
        break;

    default:
        break;
    }

    if (new_position_ && new_speed_) {
        latest_fix_.timestamp_us = time_.micros();
        latest_fix_.sequence = ++sequence_;
        latest_fix_.satellites = 0;
        fix = latest_fix_;
        fix_updated = true;
        new_position_ = false;
        new_speed_ = false;
    }

    return true;
}

void UbxNeoGpsDevice::update_checksum(uint8_t data) {
    ck_a_ = static_cast<uint8_t>(ck_a_ + data);
    ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_);
}

}  // namespace mc_experimental
