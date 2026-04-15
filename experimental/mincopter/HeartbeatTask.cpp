#include <experimental/mincopter/HeartbeatTask.h>

#include <stdio.h>
#include <stddef.h>
#include <string.h>

namespace experimental_mincopter {

namespace {

constexpr mc_rtos_hal::Timeout kWriteTimeout{50};

size_t format_heartbeat_message(const HeartbeatTaskContext &context, char *buffer, size_t buffer_len) {
    const mc_experimental::ImuTaskStats empty_imu_stats{};
    const mc_experimental::GpsTaskStats empty_gps_stats{};
    const mc_experimental::ImuTaskStats &imu_stats =
        context.imu_stats != nullptr ? *context.imu_stats : empty_imu_stats;
    const mc_experimental::GpsTaskStats &gps_stats =
        context.gps_stats != nullptr ? *context.gps_stats : empty_gps_stats;
    const bool latest_imu_valid = context.channels != nullptr && context.channels->latest_imu.valid;
    const mc_experimental::ImuSample latest_imu =
        latest_imu_valid ? context.channels->latest_imu.sample : mc_experimental::ImuSample{};

    const int len = snprintf(buffer,
                             buffer_len,
                             "heartbeat imu(runs=%lu samples=%lu failures=%lu timeouts=%lu overruns=%lu last_ts_us=%lu recoveries=%lu recovery_failures=%lu healthy=%u) "
                             "accel(valid=%u ts_us=%lu m_s2=%.6f,%.6f,%.6f) "
                             "gps(fixes=%lu service_failures=%lu overruns=%lu polls=%lu last_ts_us=%lu healthy=%u)\r\n",
                             static_cast<unsigned long>(imu_stats.loop_runs),
                             static_cast<unsigned long>(imu_stats.samples_published),
                             static_cast<unsigned long>(imu_stats.read_failures),
                             static_cast<unsigned long>(imu_stats.wake_timeouts),
                             static_cast<unsigned long>(imu_stats.overruns),
                             static_cast<unsigned long>(imu_stats.last_sample_timestamp_us),
                             static_cast<unsigned long>(imu_stats.recovery_attempts),
                             static_cast<unsigned long>(imu_stats.recovery_failures),
                             imu_stats.healthy ? 1U : 0U,
                             latest_imu_valid ? 1U : 0U,
                             static_cast<unsigned long>(latest_imu.timestamp_us),
                             static_cast<double>(latest_imu.accel_m_s2[0]),
                             static_cast<double>(latest_imu.accel_m_s2[1]),
                             static_cast<double>(latest_imu.accel_m_s2[2]),
                             static_cast<unsigned long>(gps_stats.fixes_published),
                             static_cast<unsigned long>(gps_stats.service_failures),
                             static_cast<unsigned long>(gps_stats.overruns),
                             static_cast<unsigned long>(gps_stats.poll_count),
                             static_cast<unsigned long>(gps_stats.last_fix_timestamp_us),
                             gps_stats.healthy ? 1U : 0U);

    if (len < 0) {
        return 0U;
    }
    if (static_cast<size_t>(len) >= buffer_len) {
        return buffer_len > 0U ? buffer_len - 1U : 0U;
    }
    return static_cast<size_t>(len);
}

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
    uart_config.tx_buffer_size = 384;
    mc_rtos_hal::UartPort &uart = context.hal.uart(context.config.uart_index);
    if (uart.configure(uart_config) != mc_rtos_hal::Status::Ok) {
        for (;;) {
            context.hal.time().delay_ms(1000);
        }
    }

    uint32_t last_wake_ms = context.hal.time().millis();
    for (;;) {
        char message[384] = {};
        const size_t message_len = format_heartbeat_message(context, message, sizeof(message));

        size_t written = 0;
        const mc_rtos_hal::Status status =
            uart.write(reinterpret_cast<const uint8_t *>(message),
                       message_len,
                       kWriteTimeout,
                       &written);

        if (status == mc_rtos_hal::Status::Ok && written == message_len) {
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
