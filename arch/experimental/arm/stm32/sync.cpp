#include <arm/stm32/sync.h>

namespace stm32 {

namespace {

TickType_t timeout_to_ticks(mc_rtos_hal::Timeout timeout) {
    if (timeout.ms == 0U) {
        return 0;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout.ms);
    if (ticks == 0) {
        ticks = 1;
    }
    return ticks;
}

}  // namespace

Stm32Mutex::Stm32Mutex() : handle_(nullptr) {
    handle_ = xSemaphoreCreateMutex();
}

mc_rtos_hal::Status Stm32Mutex::lock(mc_rtos_hal::Timeout timeout) {
    if (handle_ == nullptr) {
        return mc_rtos_hal::Status::Error;
    }

    return (xSemaphoreTake(handle_, timeout_to_ticks(timeout)) == pdTRUE) ? mc_rtos_hal::Status::Ok
                                                                           : mc_rtos_hal::Status::Timeout;
}

void Stm32Mutex::unlock() {
    if (handle_ != nullptr) {
        xSemaphoreGive(handle_);
    }
}

Stm32BinarySemaphore::Stm32BinarySemaphore() : handle_(nullptr) {
    handle_ = xSemaphoreCreateBinary();
}

mc_rtos_hal::Status Stm32BinarySemaphore::take(mc_rtos_hal::Timeout timeout) {
    if (handle_ == nullptr) {
        return mc_rtos_hal::Status::Error;
    }

    return (xSemaphoreTake(handle_, timeout_to_ticks(timeout)) == pdTRUE) ? mc_rtos_hal::Status::Ok
                                                                           : mc_rtos_hal::Status::Timeout;
}

void Stm32BinarySemaphore::give() {
    if (handle_ != nullptr) {
        xSemaphoreGive(handle_);
    }
}

void Stm32BinarySemaphore::give_from_isr() {
    if (handle_ != nullptr) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xSemaphoreGiveFromISR(handle_, &higher_priority_task_woken);
    }
}

Stm32EventGroup::Stm32EventGroup() : handle_(nullptr) {
    handle_ = xEventGroupCreate();
}

uint32_t Stm32EventGroup::set_bits(uint32_t bits) {
    if (handle_ == nullptr) {
        return 0;
    }

    return static_cast<uint32_t>(xEventGroupSetBits(handle_, static_cast<EventBits_t>(bits)));
}

uint32_t Stm32EventGroup::clear_bits(uint32_t bits) {
    if (handle_ == nullptr) {
        return 0;
    }

    return static_cast<uint32_t>(xEventGroupClearBits(handle_, static_cast<EventBits_t>(bits)));
}

uint32_t Stm32EventGroup::wait_bits(uint32_t bits,
                                    bool clear_on_exit,
                                    bool wait_for_all,
                                    mc_rtos_hal::Timeout timeout) {
    if (handle_ == nullptr) {
        return 0;
    }

    return static_cast<uint32_t>(xEventGroupWaitBits(handle_,
                                                     static_cast<EventBits_t>(bits),
                                                     clear_on_exit ? pdTRUE : pdFALSE,
                                                     wait_for_all ? pdTRUE : pdFALSE,
                                                     timeout_to_ticks(timeout)));
}

}  // namespace stm32
