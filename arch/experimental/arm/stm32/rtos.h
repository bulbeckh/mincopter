#pragma once

#include <stddef.h>

#include <FreeRTOS.h>
#include <task.h>

#include <rtos_hal/rtos.h>

namespace stm32 {

class Stm32Rtos final : public mc_rtos_hal::Rtos {
public:
    Stm32Rtos();

    mc_rtos_hal::Status create_task(const mc_rtos_hal::TaskConfig &config, void **task_handle) override;
    void start_scheduler() override;
    void notify_task(void *task_handle) override;
    void notify_task_from_isr(void *task_handle, bool &should_yield) override;
    uint32_t wait_for_notification(uint32_t timeout_ms) override;
    uint32_t stack_high_water_mark_words(void *task_handle) override;
    bool in_isr() const override;

private:
    static constexpr size_t kMaxTasks = 16;
    size_t task_count_;
};

}  // namespace stm32
