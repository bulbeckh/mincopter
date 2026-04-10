#pragma once

#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

#include <experimental/mincopter/SensorChannels.h>
#include <experimental/mincopter/RuntimeConfig.h>

namespace experimental_mincopter {

struct EstimatorTaskStats {
    uint32_t imu_samples_consumed;
    uint32_t compass_samples_consumed;
    uint32_t barometer_samples_consumed;
    uint32_t gps_fixes_consumed;
    uint32_t update_count;
};

struct EstimatorTaskContext {
    mc_rtos_hal::Hal &hal;
    SensorChannels &channels;
    EstimatorTaskSchedule config;
    void *task_handle;
    EstimatorTaskStats stats;
};

class EstimatorTask {
public:
    static void task_entry(void *context);
    static bool create(EstimatorTaskContext &context);

private:
    static void run(EstimatorTaskContext &context);
};

}  // namespace experimental_mincopter
