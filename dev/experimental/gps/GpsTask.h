#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

#include <dev/experimental/gps/GpsDevice.h>

namespace mc_experimental {

struct GpsTaskStats {
    uint32_t fixes_published;
    uint32_t service_failures;
    uint32_t overruns;
    uint32_t poll_count;
    uint32_t last_fix_timestamp_us;
    bool healthy;
};

template <size_t Capacity>
class GpsFixRingBuffer {
public:
    GpsFixRingBuffer() : write_index_(0), read_index_(0), dropped_(0) {}

    bool push(const GpsFix &fix) {
        const size_t next = (write_index_ + 1U) % Capacity;
        if (next == read_index_) {
            ++dropped_;
            return false;
        }
        buffer_[write_index_] = fix;
        write_index_ = next;
        return true;
    }

    bool pop(GpsFix &fix) {
        if (read_index_ == write_index_) {
            return false;
        }
        fix = buffer_[read_index_];
        read_index_ = (read_index_ + 1U) % Capacity;
        return true;
    }

private:
    GpsFix buffer_[Capacity];
    volatile size_t write_index_;
    volatile size_t read_index_;
    size_t dropped_;
};

struct GpsTaskConfig {
    uint32_t polling_period_ms;
};

template <size_t BufferCapacity>
struct GpsTaskContext {
    mc_rtos_hal::Hal &hal;
    GpsDevice &device;
    GpsTaskConfig config;
    GpsFixRingBuffer<BufferCapacity> *ring;
    void *task_handle;
    GpsTaskStats stats;
};

class GpsTask {
public:
    template <size_t BufferCapacity>
    static void task_entry(void *context) {
        run(*static_cast<GpsTaskContext<BufferCapacity> *>(context));
    }

    template <size_t BufferCapacity>
    static bool create(GpsTaskContext<BufferCapacity> &context,
                       const char *name,
                       uint16_t stack_words,
                       mc_rtos_hal::TaskPriority priority) {
        mc_rtos_hal::TaskConfig task_config{};
        task_config.name = name;
        task_config.entry = &GpsTask::task_entry<BufferCapacity>;
        task_config.context = &context;
        task_config.stack_words = stack_words;
        task_config.priority = priority;
        return context.hal.rtos().create_task(task_config, &context.task_handle) == mc_rtos_hal::Status::Ok;
    }

private:
    template <size_t BufferCapacity>
    static void run(GpsTaskContext<BufferCapacity> &context) {
        context.stats = {0, 0, 0, 0, 0, false};

        if (!context.device.init()) {
            for (;;) {
                context.hal.time().delay_ms(1000);
            }
        }

        context.stats.healthy = true;
        uint32_t last_wake_ms = context.hal.time().millis();

        for (;;) {
            context.hal.time().delay_until_ms(last_wake_ms, context.config.polling_period_ms);
            ++context.stats.poll_count;

            GpsFix fix{};
            bool fix_updated = false;
            if (!context.device.service(fix, fix_updated)) {
                ++context.stats.service_failures;
                context.stats.healthy = false;
                continue;
            }

            if (!fix_updated) {
                continue;
            }

            if (!context.ring->push(fix)) {
                ++context.stats.overruns;
            }

            ++context.stats.fixes_published;
            context.stats.last_fix_timestamp_us = fix.timestamp_us;
            context.stats.healthy = true;
        }
    }
};

}  // namespace mc_experimental
