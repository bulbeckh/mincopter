#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

#include <dev/experimental/storage/StorageDevice.h>

namespace mc_experimental {

enum class StorageRequestType : uint8_t {
    Append = 0,
    WriteAt,
    ReadAt,
    Flush,
    EraseBlock,
    EraseAll,
};

struct StorageRequest {
    static constexpr size_t kInlineDataCapacity = 256;

    StorageRequestType type;
    uint32_t address;
    uint32_t length;
    uint32_t block_index;
    uint8_t data[kInlineDataCapacity];
    uint8_t *read_buffer;
};

struct StorageTaskStats {
    uint32_t requests_processed;
    uint32_t requests_dropped;
    uint32_t write_failures;
    uint32_t read_failures;
    uint32_t erase_failures;
    uint32_t sync_failures;
    uint32_t bytes_written;
    uint32_t bytes_read;
    bool healthy;
};

template <size_t Capacity>
class StorageRequestQueue {
public:
    StorageRequestQueue() : write_index_(0), read_index_(0), dropped_(0) {}

    bool push(const StorageRequest &request) {
        const size_t next = (write_index_ + 1U) % Capacity;
        if (next == read_index_) {
            ++dropped_;
            return false;
        }
        queue_[write_index_] = request;
        write_index_ = next;
        return true;
    }

    bool pop(StorageRequest &request) {
        if (read_index_ == write_index_) {
            return false;
        }
        request = queue_[read_index_];
        read_index_ = (read_index_ + 1U) % Capacity;
        return true;
    }

    size_t dropped() const {
        return dropped_;
    }

private:
    StorageRequest queue_[Capacity];
    volatile size_t write_index_;
    volatile size_t read_index_;
    size_t dropped_;
};

struct StorageTaskConfig {
    uint32_t notification_timeout_ms;
    uint32_t flush_period_ms;
};

template <size_t QueueCapacity>
struct StorageTaskContext {
    mc_rtos_hal::Hal &hal;
    StorageDevice &device;
    StorageTaskConfig config;
    StorageRequestQueue<QueueCapacity> *queue;
    void *task_handle;
    StorageTaskStats stats;
    uint32_t append_head;
};

class StorageTask {
public:
    template <size_t QueueCapacity>
    static void task_entry(void *context) {
        run(*static_cast<StorageTaskContext<QueueCapacity> *>(context));
    }

    template <size_t QueueCapacity>
    static bool create(StorageTaskContext<QueueCapacity> &context,
                       const char *name,
                       uint16_t stack_words,
                       mc_rtos_hal::TaskPriority priority) {
        mc_rtos_hal::TaskConfig task_config{};
        task_config.name = name;
        task_config.entry = &StorageTask::task_entry<QueueCapacity>;
        task_config.context = &context;
        task_config.stack_words = stack_words;
        task_config.priority = priority;
        return context.hal.rtos().create_task(task_config, &context.task_handle) == mc_rtos_hal::Status::Ok;
    }

    template <size_t QueueCapacity>
    static bool submit(StorageTaskContext<QueueCapacity> &context, const StorageRequest &request) {
        const bool ok = context.queue->push(request);
        if (!ok) {
            ++context.stats.requests_dropped;
            return false;
        }
        context.hal.rtos().notify_task(context.task_handle);
        return true;
    }

    template <size_t QueueCapacity>
    static bool append(StorageTaskContext<QueueCapacity> &context, const uint8_t *data, size_t len) {
        if (len > StorageRequest::kInlineDataCapacity) {
            return false;
        }
        StorageRequest request{};
        request.type = StorageRequestType::Append;
        request.address = 0;
        request.length = static_cast<uint32_t>(len);
        request.block_index = 0;
        request.read_buffer = nullptr;
        memcpy(request.data, data, len);
        return submit(context, request);
    }

    template <size_t QueueCapacity>
    static bool write_at(StorageTaskContext<QueueCapacity> &context, uint32_t address, const uint8_t *data, size_t len) {
        if (len > StorageRequest::kInlineDataCapacity) {
            return false;
        }
        StorageRequest request{};
        request.type = StorageRequestType::WriteAt;
        request.address = address;
        request.length = static_cast<uint32_t>(len);
        request.block_index = 0;
        request.read_buffer = nullptr;
        memcpy(request.data, data, len);
        return submit(context, request);
    }

    template <size_t QueueCapacity>
    static bool flush(StorageTaskContext<QueueCapacity> &context) {
        StorageRequest request{};
        request.type = StorageRequestType::Flush;
        return submit(context, request);
    }

    template <size_t QueueCapacity>
    static bool erase_block(StorageTaskContext<QueueCapacity> &context, uint32_t block_index) {
        StorageRequest request{};
        request.type = StorageRequestType::EraseBlock;
        request.block_index = block_index;
        return submit(context, request);
    }

private:
    template <size_t QueueCapacity>
    static void run(StorageTaskContext<QueueCapacity> &context) {
        context.stats = {0, 0, 0, 0, 0, 0, 0, 0, false};
        context.append_head = 0;

        if (!context.device.init()) {
            for (;;) {
                context.hal.time().delay_ms(1000);
            }
        }
        context.stats.healthy = true;

        uint32_t last_flush_ms = context.hal.time().millis();

        for (;;) {
            context.hal.rtos().wait_for_notification(context.config.notification_timeout_ms);

            StorageRequest request{};
            while (context.queue->pop(request)) {
                handle_request(context, request);
                ++context.stats.requests_processed;
            }

            const uint32_t now_ms = context.hal.time().millis();
            if ((now_ms - last_flush_ms) >= context.config.flush_period_ms) {
                if (!context.device.sync()) {
                    ++context.stats.sync_failures;
                    context.stats.healthy = false;
                }
                last_flush_ms = now_ms;
            }
        }
    }

    template <size_t QueueCapacity>
    static void handle_request(StorageTaskContext<QueueCapacity> &context, const StorageRequest &request) {
        switch (request.type) {
        case StorageRequestType::Append:
            if (!context.device.write(context.append_head, request.data, request.length)) {
                ++context.stats.write_failures;
                context.stats.healthy = false;
                return;
            }
            context.append_head += request.length;
            context.stats.bytes_written += request.length;
            context.stats.healthy = true;
            break;

        case StorageRequestType::WriteAt:
            if (!context.device.write(request.address, request.data, request.length)) {
                ++context.stats.write_failures;
                context.stats.healthy = false;
                return;
            }
            context.stats.bytes_written += request.length;
            context.stats.healthy = true;
            break;

        case StorageRequestType::ReadAt:
            if (request.read_buffer == nullptr || !context.device.read(request.address, request.read_buffer, request.length)) {
                ++context.stats.read_failures;
                context.stats.healthy = false;
                return;
            }
            context.stats.bytes_read += request.length;
            context.stats.healthy = true;
            break;

        case StorageRequestType::Flush:
            if (!context.device.sync()) {
                ++context.stats.sync_failures;
                context.stats.healthy = false;
                return;
            }
            context.stats.healthy = true;
            break;

        case StorageRequestType::EraseBlock:
            if (!context.device.erase_block(request.block_index)) {
                ++context.stats.erase_failures;
                context.stats.healthy = false;
                return;
            }
            context.stats.healthy = true;
            break;

        case StorageRequestType::EraseAll:
            if (!context.device.erase_all()) {
                ++context.stats.erase_failures;
                context.stats.healthy = false;
                return;
            }
            context.append_head = 0;
            context.stats.healthy = true;
            break;
        }
    }
};

}  // namespace mc_experimental
