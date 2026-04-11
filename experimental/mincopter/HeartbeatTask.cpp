#include <experimental/mincopter/HeartbeatTask.h>

#include <stddef.h>
#include <string.h>

namespace experimental_mincopter {

namespace {

constexpr char kHeartbeatMessage[] = "heartbeat\r\n";
constexpr mc_rtos_hal::Timeout kWriteTimeout{50};

}  // namespace

void HeartbeatTask::task_entry(void *context) {
    run(*static_cast<HeartbeatTaskContext *>(context));
}

bool HeartbeatTask::create(HeartbeatTaskContext &context) {
    mc_rtos_hal::TaskConfig task_config{};
    task_config.name = "heartbeat";
    task_config.entry = &HeartbeatTask::task_entry;
    task_config.context = &context;
    task_config.stack_words = context.config.stack_words;
    task_config.priority = context.config.priority;
    return context.hal.rtos().create_task(task_config, &context.task_handle) == mc_rtos_hal::Status::Ok;
}

void HeartbeatTask::run(HeartbeatTaskContext &context) {
    context.stats = {};

    mc_rtos_hal::UartConfig uart_config{};
    uart_config.baud_rate = context.config.baud_rate;
    uart_config.rx_buffer_size = 64;
    uart_config.tx_buffer_size = 64;
    mc_rtos_hal::UartPort &uart = context.hal.uart(context.config.uart_index);
    if (uart.configure(uart_config) != mc_rtos_hal::Status::Ok) {
        for (;;) {
            context.hal.time().delay_ms(1000);
        }
    }

    uint32_t last_wake_ms = context.hal.time().millis();
    for (;;) {
        size_t written = 0;
        const mc_rtos_hal::Status status =
            uart.write(reinterpret_cast<const uint8_t *>(kHeartbeatMessage),
                       strlen(kHeartbeatMessage),
                       kWriteTimeout,
                       &written);

        if (status == mc_rtos_hal::Status::Ok && written == strlen(kHeartbeatMessage)) {
            ++context.stats.messages_sent;
        } else {
            ++context.stats.write_failures;
        }

        if (context.led != nullptr) {
            context.led->toggle();
        }

        context.hal.time().delay_until_ms(last_wake_ms, context.config.period_ms / 2U);

        if (context.led != nullptr) {
            context.led->toggle();
        }

        context.hal.time().delay_until_ms(last_wake_ms, context.config.period_ms / 2U);
    }
}

}  // namespace experimental_mincopter
