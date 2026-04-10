#include <dev/experimental/storage/at45db321d/At45Db321dStorageDevice.h>

namespace mc_experimental {

namespace {

constexpr uint8_t kDfStatusRegisterRead = 0xD7;
constexpr uint8_t kDfReadManufacturerAndId = 0x9F;
constexpr uint8_t kDfPageRead = 0xD2;
constexpr uint8_t kDfBuffer1Write = 0x84;
constexpr uint8_t kDfBuffer1ToPageWithErase = 0x83;
constexpr uint8_t kDfBlockErase = 0x50;
constexpr uint8_t kDfChipErase0 = 0xC7;
constexpr uint8_t kDfChipErase1 = 0x94;
constexpr uint8_t kDfChipErase2 = 0x80;
constexpr uint8_t kDfChipErase3 = 0x9A;

constexpr uint32_t kDefaultCapacityBytes = 8192U * 512U;

mc_rtos_hal::SpiDeviceConfig make_spi_config(const At45Db321dConfig &config) {
    mc_rtos_hal::SpiDeviceConfig spi_cfg{};
    spi_cfg.frequency_hz = config.spi_frequency_hz;
    spi_cfg.mode = 0;
    spi_cfg.bits_per_word = 8;
    spi_cfg.chip_select_pin = config.chip_select_pin;
    return spi_cfg;
}

}  // namespace

At45Db321dStorageDevice::At45Db321dStorageDevice(mc_rtos_hal::SpiBus &spi,
                                                 mc_rtos_hal::Time &time,
                                                 mc_rtos_hal::GpioController &gpio,
                                                 const At45Db321dConfig &config)
    : spi_(spi),
      time_(time),
      gpio_(gpio),
      config_(config),
      page_size_bytes_(512),
      capacity_bytes_(kDefaultCapacityBytes) {}

bool At45Db321dStorageDevice::init() {
    if (!hardware_reset()) {
        return false;
    }
    if (spi_.configure_device(make_spi_config(config_)) != mc_rtos_hal::Status::Ok) {
        return false;
    }

    uint8_t id_response[4] = {};
    const uint8_t command[4] = {kDfReadManufacturerAndId, 0x00, 0x00, 0x00};
    if (!transfer_command(command, id_response, sizeof(command))) {
        return false;
    }

    page_size_bytes_ = page_size_bytes();
    return true;
}

bool At45Db321dStorageDevice::is_ready() {
    uint8_t status = 0;
    if (!read_status(status)) {
        return false;
    }
    return (status & 0x80U) != 0U;
}

bool At45Db321dStorageDevice::sync() {
    return wait_ready(100);
}

bool At45Db321dStorageDevice::erase_all() {
    if (spi_.lock({10}) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    if (spi_.configure_device(make_spi_config(config_)) != mc_rtos_hal::Status::Ok) {
        spi_.unlock();
        return false;
    }

    const uint8_t command[4] = {kDfChipErase0, kDfChipErase1, kDfChipErase2, kDfChipErase3};
    const mc_rtos_hal::Status status = spi_.transfer(command, nullptr, sizeof(command), {100});
    spi_.unlock();
    return status == mc_rtos_hal::Status::Ok && wait_ready(5000);
}

bool At45Db321dStorageDevice::erase_block(uint32_t block_index) {
    if (spi_.lock({10}) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    if (spi_.configure_device(make_spi_config(config_)) != mc_rtos_hal::Status::Ok) {
        spi_.unlock();
        return false;
    }

    uint8_t command[4] = {kDfBlockErase, 0x00, 0x00, 0x00};
    if (page_size_bytes_ == 512U) {
        command[1] = static_cast<uint8_t>(block_index >> 4);
        command[2] = static_cast<uint8_t>(block_index << 4);
    } else {
        command[1] = static_cast<uint8_t>(block_index >> 3);
        command[2] = static_cast<uint8_t>(block_index << 5);
    }

    const mc_rtos_hal::Status status = spi_.transfer(command, nullptr, sizeof(command), {100});
    spi_.unlock();
    return status == mc_rtos_hal::Status::Ok && wait_ready(500);
}

bool At45Db321dStorageDevice::write(uint32_t address, const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0U) {
        return true;
    }

    const uint32_t page = address / page_size_bytes_;
    const uint16_t offset = static_cast<uint16_t>(address % page_size_bytes_);
    if ((offset + len) > page_size_bytes_) {
        return false;
    }

    if (spi_.lock({10}) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    if (spi_.configure_device(make_spi_config(config_)) != mc_rtos_hal::Status::Ok) {
        spi_.unlock();
        return false;
    }

    uint8_t write_cmd[4] = {kDfBuffer1Write, 0x00, static_cast<uint8_t>(offset >> 8), static_cast<uint8_t>(offset)};
    if (spi_.transfer(write_cmd, nullptr, sizeof(write_cmd), {10}) != mc_rtos_hal::Status::Ok ||
        spi_.transfer(data, nullptr, len, {50}) != mc_rtos_hal::Status::Ok) {
        spi_.unlock();
        return false;
    }

    uint8_t program_cmd[4] = {kDfBuffer1ToPageWithErase, 0x00, 0x00, 0x00};
    if (page_size_bytes_ == 512U) {
        program_cmd[1] = static_cast<uint8_t>(page >> 7);
        program_cmd[2] = static_cast<uint8_t>(page << 1);
    } else {
        program_cmd[1] = static_cast<uint8_t>(page >> 6);
        program_cmd[2] = static_cast<uint8_t>(page << 2);
    }

    const mc_rtos_hal::Status status = spi_.transfer(program_cmd, nullptr, sizeof(program_cmd), {100});
    spi_.unlock();
    return status == mc_rtos_hal::Status::Ok && wait_ready(100);
}

bool At45Db321dStorageDevice::read(uint32_t address, uint8_t *data, size_t len) {
    if (data == nullptr || len == 0U) {
        return true;
    }

    if (spi_.lock({10}) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    if (spi_.configure_device(make_spi_config(config_)) != mc_rtos_hal::Status::Ok) {
        spi_.unlock();
        return false;
    }

    uint8_t command[8] = {kDfPageRead,
                          static_cast<uint8_t>(address >> 16),
                          static_cast<uint8_t>(address >> 8),
                          static_cast<uint8_t>(address),
                          0x00,
                          0x00,
                          0x00,
                          0x00};

    if (spi_.transfer(command, nullptr, sizeof(command), {10}) != mc_rtos_hal::Status::Ok ||
        spi_.transfer(nullptr, data, len, {100}) != mc_rtos_hal::Status::Ok) {
        spi_.unlock();
        return false;
    }

    spi_.unlock();
    return true;
}

uint32_t At45Db321dStorageDevice::capacity_bytes() const {
    return capacity_bytes_;
}

uint32_t At45Db321dStorageDevice::erase_block_size_bytes() const {
    return static_cast<uint32_t>(page_size_bytes_) * 8U;
}

bool At45Db321dStorageDevice::hardware_reset() {
    mc_rtos_hal::GpioPin *reset_pin = gpio_.pin(config_.reset_pin);
    if (reset_pin == nullptr) {
        return false;
    }
    reset_pin->set_mode(mc_rtos_hal::PinMode::Output);
    reset_pin->write(false);
    time_.delay_ms(1);
    reset_pin->write(true);
    time_.delay_ms(10);
    return true;
}

bool At45Db321dStorageDevice::read_status(uint8_t &status) {
    uint8_t tx[2] = {kDfStatusRegisterRead, 0x00};
    uint8_t rx[2] = {};
    if (!transfer_command(tx, rx, sizeof(tx))) {
        return false;
    }
    status = rx[1];
    return true;
}

uint16_t At45Db321dStorageDevice::page_size_bytes() {
    uint8_t status = 0;
    if (!read_status(status)) {
        return 512U;
    }
    return static_cast<uint16_t>(528U - ((status & 0x01U) << 4));
}

bool At45Db321dStorageDevice::wait_ready(uint32_t timeout_ms) {
    const uint32_t start = time_.millis();
    while ((time_.millis() - start) < timeout_ms) {
        if (is_ready()) {
            return true;
        }
        time_.delay_ms(1);
    }
    return false;
}

bool At45Db321dStorageDevice::transfer_command(const uint8_t *tx, uint8_t *rx, size_t len) {
    if (spi_.lock({10}) != mc_rtos_hal::Status::Ok) {
        return false;
    }
    if (spi_.configure_device(make_spi_config(config_)) != mc_rtos_hal::Status::Ok) {
        spi_.unlock();
        return false;
    }
    const mc_rtos_hal::Status status = spi_.transfer(tx, rx, len, {20});
    spi_.unlock();
    return status == mc_rtos_hal::Status::Ok;
}

}  // namespace mc_experimental
