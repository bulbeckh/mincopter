#pragma once

#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>

#include <rtos_hal/sync.h>

namespace stm32 {

class Stm32Mutex final : public mc_rtos_hal::Mutex {
public:
    Stm32Mutex();
    mc_rtos_hal::Status lock(mc_rtos_hal::Timeout timeout) override;
    void unlock() override;

private:
    SemaphoreHandle_t handle_;
};

class Stm32BinarySemaphore final : public mc_rtos_hal::BinarySemaphore {
public:
    Stm32BinarySemaphore();
    mc_rtos_hal::Status take(mc_rtos_hal::Timeout timeout) override;
    void give() override;
    void give_from_isr() override;

private:
    SemaphoreHandle_t handle_;
};

class Stm32EventGroup final : public mc_rtos_hal::EventGroup {
public:
    Stm32EventGroup();
    uint32_t set_bits(uint32_t bits) override;
    uint32_t clear_bits(uint32_t bits) override;
    uint32_t wait_bits(uint32_t bits, bool clear_on_exit, bool wait_for_all, mc_rtos_hal::Timeout timeout) override;

private:
    EventGroupHandle_t handle_;
};

}  // namespace stm32
