#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/gpio.h>
#include <rtos_hal/spi.h>
#include <rtos_hal/time.h>

namespace mc_experimental {

class StorageDevice {
public:
    virtual ~StorageDevice() = default;

    virtual bool init() = 0;
    virtual bool is_ready() = 0;
    virtual bool sync() = 0;
    virtual bool erase_all() = 0;
    virtual bool erase_block(uint32_t block_index) = 0;
    virtual bool write(uint32_t address, const uint8_t *data, size_t len) = 0;
    virtual bool read(uint32_t address, uint8_t *data, size_t len) = 0;
    virtual uint32_t capacity_bytes() const = 0;
    virtual uint32_t erase_block_size_bytes() const = 0;
};

}  // namespace mc_experimental
