#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

using TaskEntry = void (*)(void *);

struct TaskConfig {
    const char *name;
    TaskEntry entry;
    void *context;
    uint16_t stack_words;
    TaskPriority priority;
};

class Rtos {
public:
    virtual ~Rtos() = default;
    virtual Status create_task(const TaskConfig &config, void **task_handle) = 0;
    virtual void start_scheduler() = 0;
    virtual void notify_task(void *task_handle) = 0;
    virtual void notify_task_from_isr(void *task_handle, bool &should_yield) = 0;
    virtual uint32_t wait_for_notification(uint32_t timeout_ms) = 0;
    virtual uint32_t stack_high_water_mark_words(void *task_handle) = 0;
    virtual bool in_isr() const = 0;
};

}  // namespace mc_rtos_hal
