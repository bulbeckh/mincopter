#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/gpio.h>
#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

#include <dev/experimental/barometer/BarometerDevice.h>

namespace mc_experimental {

struct BarometerTaskStats {
    uint32_t samples_published;
    uint32_t read_failures;
    uint32_t wake_timeouts;
    uint32_t overruns;
    uint32_t last_sample_timestamp_us;
    bool healthy;
};

template <size_t Capacity>
class BarometerSampleRingBuffer {
public:
    BarometerSampleRingBuffer() : write_index_(0), read_index_(0), dropped_(0) {}

    bool push(const BarometerSample &sample) {
        const size_t next = (write_index_ + 1U) % Capacity;
        if (next == read_index_) {
            ++dropped_;
            return false;
        }
        buffer_[write_index_] = sample;
        write_index_ = next;
        return true;
    }

    bool pop(BarometerSample &sample) {
        if (read_index_ == write_index_) {
            return false;
        }
        sample = buffer_[read_index_];
        read_index_ = (read_index_ + 1U) % Capacity;
        return true;
    }

private:
    BarometerSample buffer_[Capacity];
    volatile size_t write_index_;
    volatile size_t read_index_;
    size_t dropped_;
};

struct BarometerTaskConfig {
    uint32_t notification_timeout_ms;
    uint32_t polling_period_ms;
};

template <size_t BufferCapacity>
struct BarometerTaskContext {
    mc_rtos_hal::Hal &hal;
    BarometerDevice &device;
    BarometerTaskConfig config;
    BarometerSampleRingBuffer<BufferCapacity> *ring;
    void *task_handle;
    BarometerTaskStats stats;
};

class BarometerTask {
public:
    template <size_t BufferCapacity>
    static void task_entry(void *context) {
        run(*static_cast<BarometerTaskContext<BufferCapacity> *>(context));
    }

    template <size_t BufferCapacity>
    static bool create(BarometerTaskContext<BufferCapacity> &context,
                       const char *name,
                       uint16_t stack_words,
                       mc_rtos_hal::TaskPriority priority) {
        mc_rtos_hal::TaskConfig task_config{};
        task_config.name = name;
        task_config.entry = &BarometerTask::task_entry<BufferCapacity>;
        task_config.context = &context;
        task_config.stack_words = stack_words;
        task_config.priority = priority;
        return context.hal.rtos().create_task(task_config, &context.task_handle) == mc_rtos_hal::Status::Ok;
    }

    template <size_t BufferCapacity>
    static void data_ready_isr(void *context) {
        auto &task_context = *static_cast<BarometerTaskContext<BufferCapacity> *>(context);
        bool should_yield = false;
        task_context.hal.rtos().notify_task_from_isr(task_context.task_handle, should_yield);
        (void)should_yield;
    }

private:
    template <size_t BufferCapacity>
    static void run(BarometerTaskContext<BufferCapacity> &context) {
        context.stats = {0, 0, 0, 0, 0, false};

        while (!context.device.init()) {
            ++context.stats.read_failures;
            context.stats.healthy = false;
            context.stats.last_sample_timestamp_us = context.hal.time().micros();
            context.hal.time().delay_ms(1000);
        }

        if (context.device.has_data_ready_irq()) {
            mc_rtos_hal::GpioPin *pin = context.hal.gpio().pin(context.device.data_ready_pin());
            if (pin != nullptr) {
                mc_rtos_hal::GpioInterruptConfig irq_cfg{};
                irq_cfg.edge = mc_rtos_hal::Edge::Rising;
                irq_cfg.callback = &BarometerTask::data_ready_isr<BufferCapacity>;
                irq_cfg.context = &context;
                pin->attach_interrupt(irq_cfg);
            }
        }

        context.stats.healthy = true;

        uint32_t last_wake_ms = context.hal.time().millis();
        for (;;) {
            if (context.device.has_data_ready_irq()) {
                const uint32_t notified = context.hal.rtos().wait_for_notification(context.config.notification_timeout_ms);
                if (notified == 0U) {
                    ++context.stats.wake_timeouts;
                }
            } else {
                context.hal.time().delay_until_ms(last_wake_ms, context.config.polling_period_ms);
            }

            BarometerSample sample{};
            if (!context.device.read_sample(sample) || !sample.valid) {
                ++context.stats.read_failures;
                context.stats.healthy = false;
                context.hal.time().delay_ms(20);
                continue;
            }

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
