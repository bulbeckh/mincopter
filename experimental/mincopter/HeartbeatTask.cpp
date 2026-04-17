#include <experimental/mincopter/HeartbeatTask.h>

#include <stdio.h>
#include <stddef.h>
#include <string.h>

namespace experimental_mincopter {

namespace {

constexpr mc_rtos_hal::Timeout kWriteTimeout{100};

size_t format_heartbeat_message(const HeartbeatTaskContext &context, char *buffer, size_t buffer_len) {
    const mc_experimental::ImuTaskStats empty_imu_stats{};
    const mc_experimental::CompassTaskStats empty_compass_stats{};
    const mc_experimental::BarometerTaskStats empty_barometer_stats{};
    const mc_experimental::GpsTaskStats empty_gps_stats{};
    const mc_experimental::ImuTaskStats &imu_stats =
        context.imu_stats != nullptr ? *context.imu_stats : empty_imu_stats;
    const mc_experimental::CompassTaskStats &compass_stats =
        context.compass_stats != nullptr ? *context.compass_stats : empty_compass_stats;
    const mc_experimental::BarometerTaskStats &barometer_stats =
        context.barometer_stats != nullptr ? *context.barometer_stats : empty_barometer_stats;
    const mc_experimental::GpsTaskStats &gps_stats =
        context.gps_stats != nullptr ? *context.gps_stats : empty_gps_stats;
    const int len = snprintf(buffer,
                             buffer_len,
                             "heartbeat\r\n"
                             "\timu-test stats(runs=%lu samples=%lu failures=%lu timeouts=%lu overruns=%lu last_ts_us=%lu recoveries=%lu recovery_failures=%lu healthy=%u)\r\n"
                             "\tcompass-test stats(samples=%lu failures=%lu timeouts=%lu overruns=%lu last_ts_us=%lu healthy=%u)\r\n"
                             "\tgps-test stats(fixes=%lu service_failures=%lu overruns=%lu polls=%lu last_ts_us=%lu healthy=%u)\r\n"
                             "\tbarometer-test stats(samples=%lu failures=%lu timeouts=%lu overruns=%lu last_ts_us=%lu healthy=%u)\r\n",
                             static_cast<unsigned long>(imu_stats.loop_runs),
                             static_cast<unsigned long>(imu_stats.samples_published),
                             static_cast<unsigned long>(imu_stats.read_failures),
                             static_cast<unsigned long>(imu_stats.wake_timeouts),
                             static_cast<unsigned long>(imu_stats.overruns),
                             static_cast<unsigned long>(imu_stats.last_sample_timestamp_us),
                             static_cast<unsigned long>(imu_stats.recovery_attempts),
                             static_cast<unsigned long>(imu_stats.recovery_failures),
                             imu_stats.healthy ? 1U : 0U,
                             static_cast<unsigned long>(compass_stats.samples_published),
                             static_cast<unsigned long>(compass_stats.read_failures),
                             static_cast<unsigned long>(compass_stats.wake_timeouts),
                             static_cast<unsigned long>(compass_stats.overruns),
                             static_cast<unsigned long>(compass_stats.last_sample_timestamp_us),
                             compass_stats.healthy ? 1U : 0U,
                             static_cast<unsigned long>(gps_stats.fixes_published),
                             static_cast<unsigned long>(gps_stats.service_failures),
                             static_cast<unsigned long>(gps_stats.overruns),
                             static_cast<unsigned long>(gps_stats.poll_count),
                             static_cast<unsigned long>(gps_stats.last_fix_timestamp_us),
                             gps_stats.healthy ? 1U : 0U,
                             static_cast<unsigned long>(barometer_stats.samples_published),
                             static_cast<unsigned long>(barometer_stats.read_failures),
                             static_cast<unsigned long>(barometer_stats.wake_timeouts),
                             static_cast<unsigned long>(barometer_stats.overruns),
                             static_cast<unsigned long>(barometer_stats.last_sample_timestamp_us),
                             barometer_stats.healthy ? 1U : 0U);

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

    uint32_t last_wake_ms = context.hal.time().millis();
    for (;;) {
        char message[1536] = {};
        const size_t message_len = format_heartbeat_message(context, message, sizeof(message));

        size_t written = 0;
        const mc_rtos_hal::Status status =
            context.hal.console().write(reinterpret_cast<const uint8_t *>(message),
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
