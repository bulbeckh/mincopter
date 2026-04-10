#pragma once

#include <rtos_hal/gpio.h>
#include <rtos_hal/spi.h>
#include <rtos_hal/time.h>

#include <dev/experimental/storage/StorageDevice.h>

namespace mc_experimental {

struct At45Db321dConfig {
    uint16_t reset_pin;
    uint16_t chip_select_pin;
    uint32_t spi_frequency_hz;
};

class At45Db321dStorageDevice final : public StorageDevice {
public:
    At45Db321dStorageDevice(mc_rtos_hal::SpiBus &spi,
                            mc_rtos_hal::Time &time,
                            mc_rtos_hal::GpioController &gpio,
                            const At45Db321dConfig &config);

    bool init() override;
    bool is_ready() override;
    bool sync() override;
    bool erase_all() override;
    bool erase_block(uint32_t block_index) override;
    bool write(uint32_t address, const uint8_t *data, size_t len) override;
    bool read(uint32_t address, uint8_t *data, size_t len) override;
    uint32_t capacity_bytes() const override;
    uint32_t erase_block_size_bytes() const override;

private:
    bool hardware_reset();
    bool read_status(uint8_t &status);
    uint16_t page_size_bytes();
    bool wait_ready(uint32_t timeout_ms);
    bool transfer_command(const uint8_t *tx, uint8_t *rx, size_t len);

private:
    mc_rtos_hal::SpiBus &spi_;
    mc_rtos_hal::Time &time_;
    mc_rtos_hal::GpioController &gpio_;
    At45Db321dConfig config_;
    uint16_t page_size_bytes_;
    uint32_t capacity_bytes_;
};

}  // namespace mc_experimental
