#pragma once

#include <stdint.h>

#include <dev/experimental/led/LedDevice.h>
#include <experimental/mincopter/RuntimeConfig.h>
#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>
#include <rtos_hal/uart.h>

namespace experimental_mincopter {

struct HeartbeatTaskStats {
    uint32_t messages_sent;
    uint32_t write_failures;
};

struct HeartbeatTaskContext {
    mc_rtos_hal::Hal &hal;
    HeartbeatTaskSchedule config;
    mc_experimental::LedDevice *led;
    void *task_handle;
    HeartbeatTaskStats stats;
};

class HeartbeatTask {
public:
    static void task_entry(void *context);
    static bool create(HeartbeatTaskContext &context);

private:
    static void run(HeartbeatTaskContext &context);
};

}  // namespace experimental_mincopter
