#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

class Storage {
public:
    virtual ~Storage() = default;
    virtual Status read(uint32_t offset, void *dst, size_t len) = 0;
    virtual Status write(uint32_t offset, const void *src, size_t len) = 0;
    virtual Status erase(uint32_t offset, size_t len) = 0;
    virtual Status sync() = 0;
};

}  // namespace mc_rtos_hal
