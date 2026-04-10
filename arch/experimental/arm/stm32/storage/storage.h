#pragma once

#include <rtos_hal/storage.h>

namespace stm32 {

class Stm32Storage final : public mc_rtos_hal::Storage {
public:
    mc_rtos_hal::Status read(uint32_t offset, void *dst, size_t len) override;
    mc_rtos_hal::Status write(uint32_t offset, const void *src, size_t len) override;
    mc_rtos_hal::Status erase(uint32_t offset, size_t len) override;
    mc_rtos_hal::Status sync() override;
};

}  // namespace stm32
