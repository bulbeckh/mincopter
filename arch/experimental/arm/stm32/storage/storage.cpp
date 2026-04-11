#include <arm/stm32/storage/storage.h>

namespace stm32 {

mc_rtos_hal::Status Stm32Storage::read(uint32_t, void *, size_t) {
    return mc_rtos_hal::Status::Unsupported;
}

mc_rtos_hal::Status Stm32Storage::write(uint32_t, const void *, size_t) {
    return mc_rtos_hal::Status::Unsupported;
}

mc_rtos_hal::Status Stm32Storage::erase(uint32_t, size_t) {
    return mc_rtos_hal::Status::Unsupported;
}

mc_rtos_hal::Status Stm32Storage::sync() {
    return mc_rtos_hal::Status::Ok;
}

}  // namespace stm32
