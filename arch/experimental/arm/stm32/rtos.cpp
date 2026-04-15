#include <arm/stm32/rtos.h>

#include <cmsis_gcc.h>

namespace stm32 {

namespace {

constexpr UBaseType_t kBaseTaskPriority = tskIDLE_PRIORITY + 1U;

TickType_t timeout_ms_to_ticks(uint32_t timeout_ms) {
    if (timeout_ms == 0U) {
        return 0;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (ticks == 0) {
        ticks = 1;
    }
    return ticks;
}

UBaseType_t to_freertos_priority(mc_rtos_hal::TaskPriority priority) {
    const UBaseType_t available_priorities =
        (configMAX_PRIORITIES > kBaseTaskPriority) ? (configMAX_PRIORITIES - kBaseTaskPriority) : 1U;
    const UBaseType_t max_offset = available_priorities - 1U;

    switch (priority) {
    case mc_rtos_hal::TaskPriority::Lowest:
        return kBaseTaskPriority;
    case mc_rtos_hal::TaskPriority::Low:
        return kBaseTaskPriority + ((max_offset >= 1U) ? 1U : 0U);
    case mc_rtos_hal::TaskPriority::Medium:
        return kBaseTaskPriority + ((max_offset >= 2U) ? (max_offset / 2U) : max_offset);
    case mc_rtos_hal::TaskPriority::High:
        return kBaseTaskPriority + ((max_offset >= 2U) ? (max_offset - 1U) : max_offset);
    case mc_rtos_hal::TaskPriority::Highest:
        return kBaseTaskPriority + max_offset;
    }

    return kBaseTaskPriority;
}

}  // namespace

Stm32Rtos::Stm32Rtos() : task_count_(0) {}

mc_rtos_hal::Status Stm32Rtos::create_task(const mc_rtos_hal::TaskConfig &config, void **task_handle) {
    if (task_handle == nullptr || config.entry == nullptr || config.stack_words == 0U) {
        if (task_handle != nullptr) {
            *task_handle = nullptr;
        }
        return mc_rtos_hal::Status::InvalidArgument;
    }

    if (task_count_ >= kMaxTasks) {
        *task_handle = nullptr;
        return mc_rtos_hal::Status::Busy;
    }

    TaskHandle_t handle = nullptr;
    const BaseType_t created = xTaskCreate(config.entry,
                                           config.name,
                                           config.stack_words,
                                           config.context,
                                           to_freertos_priority(config.priority),
                                           &handle);
    if (created != pdPASS || handle == nullptr) {
        *task_handle = nullptr;
        return mc_rtos_hal::Status::Error;
    }

    *task_handle = handle;
    ++task_count_;
    return mc_rtos_hal::Status::Ok;
}

void Stm32Rtos::start_scheduler() {
    vTaskStartScheduler();
}

void Stm32Rtos::notify_task(void *task_handle) {
    if (task_handle == nullptr) {
        return;
    }

    xTaskNotifyGive(static_cast<TaskHandle_t>(task_handle));
}

void Stm32Rtos::notify_task_from_isr(void *task_handle, bool &should_yield) {
    should_yield = false;
    if (task_handle == nullptr) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(task_handle), &higher_priority_task_woken);
    should_yield = (higher_priority_task_woken == pdTRUE);
}

uint32_t Stm32Rtos::wait_for_notification(uint32_t timeout_ms) {
    return static_cast<uint32_t>(ulTaskNotifyTake(pdTRUE, timeout_ms_to_ticks(timeout_ms)));
}

uint32_t Stm32Rtos::stack_high_water_mark_words(void *task_handle) {
    if (task_handle == nullptr) {
        return 0U;
    }
    return static_cast<uint32_t>(uxTaskGetStackHighWaterMark(static_cast<TaskHandle_t>(task_handle)));
}

bool Stm32Rtos::in_isr() const {
    return __get_IPSR() != 0U;
}

}  // namespace stm32
