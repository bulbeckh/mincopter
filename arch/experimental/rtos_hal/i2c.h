#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

class I2cBus {
public:
    virtual ~I2cBus() = default;
    virtual Status lock(Timeout timeout) = 0;
    virtual void unlock() = 0;
    virtual Status write(uint8_t address, const uint8_t *data, size_t len, Timeout timeout) = 0;
    virtual Status read(uint8_t address, uint8_t *data, size_t len, Timeout timeout) = 0;
    virtual Status write_register(uint8_t address, uint8_t reg, uint8_t value, Timeout timeout) = 0;
    virtual Status read_registers(uint8_t address, uint8_t reg, uint8_t *data, size_t len, Timeout timeout) = 0;
};

}  // namespace mc_rtos_hal
