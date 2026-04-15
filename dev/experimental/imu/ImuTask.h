#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/gpio.h>
#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

#include <dev/experimental/imu/ImuDevice.h>

namespace mc_experimental {

struct ImuTaskStats {
    uint32_t samples_published;
    uint32_t read_failures;
    uint32_t wake_timeouts;
    uint32_t overruns;
    uint32_t last_sample_timestamp_us;
    uint32_t recovery_attempts;
    uint32_t recovery_failures;
    uint32_t loop_runs;
    bool healthy;
};

template <size_t Capacity>
class ImuSampleRingBuffer {
public:
    ImuSampleRingBuffer() : write_index_(0), read_index_(0), dropped_(0) {}

    bool push(const ImuSample &sample) {
        const size_t next = (write_index_ + 1U) % Capacity;
        if (next == read_index_) {
            ++dropped_;
            return false;
        }
        buffer_[write_index_] = sample;
        write_index_ = next;
        return true;
    }

    bool pop(ImuSample &sample) {
        if (read_index_ == write_index_) {
            return false;
        }
        sample = buffer_[read_index_];
        read_index_ = (read_index_ + 1U) % Capacity;
        return true;
    }

    size_t dropped() const {
        return dropped_;
    }

private:
    ImuSample buffer_[Capacity];
    volatile size_t write_index_;
    volatile size_t read_index_;
    size_t dropped_;
};

struct ImuTaskConfig {
    uint32_t notification_timeout_ms;
    uint32_t polling_period_ms;
    uint32_t recovery_failure_threshold;
    uint32_t recovery_wake_timeout_threshold;
    uint32_t recovery_backoff_ms;
};

template <size_t BufferCapacity>
struct ImuTaskContext {
    mc_rtos_hal::Hal &hal;
    ImuDevice &device;
    ImuTaskConfig config;
    ImuSampleRingBuffer<BufferCapacity> *ring;
    void *task_handle;
    ImuTaskStats stats;
};

class ImuTask {
public:
    template <size_t BufferCapacity>
    static void task_entry(void *context) {
        run(*static_cast<ImuTaskContext<BufferCapacity> *>(context));
    }

    template <size_t BufferCapacity>
    static bool create(ImuTaskContext<BufferCapacity> &context,
                       const char *name,
                       uint16_t stack_words,
                       mc_rtos_hal::TaskPriority priority) {
        mc_rtos_hal::TaskConfig task_config{};
        task_config.name = name;
        task_config.entry = &ImuTask::task_entry<BufferCapacity>;
        task_config.context = &context;
        task_config.stack_words = stack_words;
        task_config.priority = priority;
        return context.hal.rtos().create_task(task_config, &context.task_handle) == mc_rtos_hal::Status::Ok;
    }

    template <size_t BufferCapacity>
    static void data_ready_isr(void *context) {
        auto &task_context = *static_cast<ImuTaskContext<BufferCapacity> *>(context);
        bool should_yield = false;
        task_context.hal.rtos().notify_task_from_isr(task_context.task_handle, should_yield);
        (void)should_yield;
    }

private:
    template <size_t BufferCapacity>
    static void run(ImuTaskContext<BufferCapacity> &context) {
        context.stats = {0, 0, 0, 0, 0, 0, 0, 0, false};

        // Initialise device
        if (!context.device.init()) {
            context.stats.healthy = false;

            // Retry device init every 1s
            context.hal.time().delay_ms(1000);
        }

        // If we are using DRDY then attach the pin interrupt to the above data-ready-isr (which triggers the task notification)
        if (context.device.has_data_ready_irq()) {
            mc_rtos_hal::GpioPin *pin = context.hal.gpio().pin(context.device.data_ready_pin());
            if (pin != nullptr) {
                mc_rtos_hal::GpioInterruptConfig irq_cfg{};
                irq_cfg.edge = mc_rtos_hal::Edge::Rising;
                irq_cfg.callback = &ImuTask::data_ready_isr<BufferCapacity>;
                irq_cfg.context = &context;
                pin->attach_interrupt(irq_cfg);
            }
        }

        context.stats.healthy = true;

        uint32_t last_wake_ms = context.hal.time().millis();
        uint32_t consecutive_read_failures = 0;
        uint32_t consecutive_wake_timeouts = 0;
        for (;;) {
            // Increment loop run counter
            ++context.stats.loop_runs;
            
            if (context.device.has_data_ready_irq()) {
                const uint32_t notified = context.hal.rtos().wait_for_notification(context.config.notification_timeout_ms);
                if (notified == 0U) {
                    ++context.stats.wake_timeouts;
                    ++consecutive_wake_timeouts;
                    if (context.config.recovery_wake_timeout_threshold > 0U &&
                        consecutive_wake_timeouts >= context.config.recovery_wake_timeout_threshold) {
                        ++context.stats.recovery_attempts;
                        if (context.device.init()) {
                            consecutive_read_failures = 0;
                            consecutive_wake_timeouts = 0;
                        } else {
                            ++context.stats.recovery_failures;
                        }
                        context.hal.time().delay_ms(context.config.recovery_backoff_ms);
                        continue;
                    }
                } else {
                    consecutive_wake_timeouts = 0;
                }
            } else {
                context.hal.time().delay_until_ms(last_wake_ms, context.config.polling_period_ms);
            }

            ImuSample sample{};
            if (!context.device.read_sample(sample) || !sample.valid) {
                ++context.stats.read_failures;
                ++consecutive_read_failures;
                context.stats.healthy = false;

                if (context.config.recovery_failure_threshold > 0U &&
                    consecutive_read_failures >= context.config.recovery_failure_threshold) {
                    ++context.stats.recovery_attempts;
                    if (context.device.init()) {
                        consecutive_read_failures = 0;
                        consecutive_wake_timeouts = 0;
                    } else {
                        ++context.stats.recovery_failures;
                    }
                    context.hal.time().delay_ms(context.config.recovery_backoff_ms);
                } else {
                    context.hal.time().delay_ms(20);
                }
                continue;
            }

            consecutive_read_failures = 0;
            consecutive_wake_timeouts = 0;

            if (!context.ring->push(sample)) {
                ++context.stats.overruns;
            }

            ++context.stats.samples_published;
            context.stats.last_sample_timestamp_us = sample.timestamp_us;
            context.stats.healthy = true;
        }
    }
};

}  // namespace mc_experimental
