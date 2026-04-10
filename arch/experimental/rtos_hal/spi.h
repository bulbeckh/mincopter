#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

struct SpiDeviceConfig {
    uint32_t frequency_hz;
    uint8_t mode;
    uint8_t bits_per_word;
    uint16_t chip_select_pin;
};

class SpiBus {
public:
    virtual ~SpiBus() = default;
    virtual Status lock(Timeout timeout) = 0;
    virtual void unlock() = 0;
    virtual Status configure_device(const SpiDeviceConfig &config) = 0;
    virtual Status transfer(const uint8_t *tx, uint8_t *rx, size_t len, Timeout timeout) = 0;
};

}  // namespace mc_rtos_hal
