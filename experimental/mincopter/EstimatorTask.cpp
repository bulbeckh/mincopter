#include <experimental/mincopter/EstimatorTask.h>

namespace experimental_mincopter {

void EstimatorTask::task_entry(void *context) {
    run(*static_cast<EstimatorTaskContext *>(context));
}

bool EstimatorTask::create(EstimatorTaskContext &context) {
    mc_rtos_hal::TaskConfig task_config{};
    task_config.name = "estimator";
    task_config.entry = &EstimatorTask::task_entry;
    task_config.context = &context;
    task_config.stack_words = context.config.stack_words;
    task_config.priority = context.config.priority;
    return context.hal.rtos().create_task(task_config, &context.task_handle) == mc_rtos_hal::Status::Ok;
}

void EstimatorTask::run(EstimatorTaskContext &context) {
    context.stats = {};
    uint32_t last_wake_ms = context.hal.time().millis();

    for (;;) {
        context.hal.time().delay_until_ms(last_wake_ms, context.config.polling_period_ms);

        mc_experimental::ImuSample imu_sample{};
        while (context.channels.imu_samples.pop(imu_sample)) {
            context.channels.latest_imu.sample = imu_sample;
            context.channels.latest_imu.sequence = imu_sample.sequence;
            context.channels.latest_imu.valid = imu_sample.valid;
            ++context.stats.imu_samples_consumed;
        }

        mc_experimental::CompassSample compass_sample{};
        while (context.channels.compass_samples.pop(compass_sample)) {
            context.channels.latest_compass.sample = compass_sample;
            context.channels.latest_compass.sequence = compass_sample.sequence;
            context.channels.latest_compass.valid = compass_sample.valid;
            ++context.stats.compass_samples_consumed;
        }

        mc_experimental::BarometerSample barometer_sample{};
        while (context.channels.barometer_samples.pop(barometer_sample)) {
            context.channels.latest_barometer.sample = barometer_sample;
            context.channels.latest_barometer.sequence = barometer_sample.sequence;
            context.channels.latest_barometer.valid = barometer_sample.valid;
            ++context.stats.barometer_samples_consumed;
        }

        mc_experimental::GpsFix gps_fix{};
        while (context.channels.gps_fixes.pop(gps_fix)) {
            context.channels.latest_gps.sample = gps_fix;
            context.channels.latest_gps.sequence = gps_fix.sequence;
            context.channels.latest_gps.valid = gps_fix.valid;
            ++context.stats.gps_fixes_consumed;
        }

        ++context.stats.update_count;
    }
}

}  // namespace experimental_mincopter
