#include <experimental/mincopter/ExperimentalRuntime.h>

namespace experimental_mincopter {

ExperimentalRuntime::ExperimentalRuntime(
    mc_rtos_hal::Hal &hal,
    const RuntimeConfig &config,
    std::unique_ptr<mc_experimental::ImuDevice> imu_device,
    std::unique_ptr<mc_experimental::CompassDevice> compass_device,
    std::unique_ptr<mc_experimental::BarometerDevice> barometer_device,
    std::unique_ptr<mc_experimental::GpsDevice> gps_device,
    std::unique_ptr<mc_experimental::StorageDevice> storage_device,
    std::unique_ptr<mc_experimental::LedDevice> status_leds[RuntimeConfig::kStatusLedCount])
    : hal_(hal),
      config_(config),
      channels_(),
      imu_device_(std::move(imu_device)),
      compass_device_(std::move(compass_device)),
      barometer_device_(std::move(barometer_device)),
      gps_device_(std::move(gps_device)),
      storage_device_(std::move(storage_device)),
      status_leds_{},
      imu_task_context_(nullptr),
      compass_task_context_(nullptr),
      barometer_task_context_(nullptr),
      gps_task_context_(nullptr),
      storage_task_context_(nullptr),
      estimator_task_context_{hal_, channels_, config_.estimator, nullptr, {}},
      heartbeat_task_context_{hal_, config_.heartbeat, nullptr, nullptr, {}},
      hal_initialized_(false),
      runtime_assembled_(false),
      tasks_created_(false) {
    for (size_t i = 0; i < RuntimeConfig::kStatusLedCount; ++i) {
        status_leds_[i] = std::move(status_leds[i]);
    }
}

mc_rtos_hal::Hal &ExperimentalRuntime::hal() {
    return hal_;
}

const RuntimeConfig &ExperimentalRuntime::config() const {
    return config_;
}

SensorChannels &ExperimentalRuntime::channels() {
    return channels_;
}

bool ExperimentalRuntime::init_hal() {
    hal_initialized_ = (hal_.init() == mc_rtos_hal::Status::Ok);
    return hal_initialized_;
}

bool ExperimentalRuntime::assemble_runtime() {
    if (!hal_initialized_) {
        return false;
    }

    if (config_.imu.enabled && imu_device_ != nullptr) {
        imu_task_context_ = std::make_unique<mc_experimental::ImuTaskContext<SensorChannels::kImuBufferCapacity>>(
            mc_experimental::ImuTaskContext<SensorChannels::kImuBufferCapacity>{
                hal_,
                *imu_device_,
                {config_.imu.task.notification_timeout_ms, config_.imu.task.polling_period_ms},
                &channels_.imu_samples,
                nullptr,
                {}
            });
    }

    if (config_.compass.enabled && compass_device_ != nullptr) {
        compass_task_context_ = std::make_unique<mc_experimental::CompassTaskContext<SensorChannels::kCompassBufferCapacity>>(
            mc_experimental::CompassTaskContext<SensorChannels::kCompassBufferCapacity>{
                hal_,
                *compass_device_,
                {config_.compass.task.notification_timeout_ms, config_.compass.task.polling_period_ms},
                &channels_.compass_samples,
                nullptr,
                {}
            });
    }

    if (config_.barometer.enabled && barometer_device_ != nullptr) {
        barometer_task_context_ = std::make_unique<mc_experimental::BarometerTaskContext<SensorChannels::kBarometerBufferCapacity>>(
            mc_experimental::BarometerTaskContext<SensorChannels::kBarometerBufferCapacity>{
                hal_,
                *barometer_device_,
                {config_.barometer.task.notification_timeout_ms, config_.barometer.task.polling_period_ms},
                &channels_.barometer_samples,
                nullptr,
                {}
            });
    }

    if (config_.gps.enabled && gps_device_ != nullptr) {
        gps_task_context_ = std::make_unique<mc_experimental::GpsTaskContext<SensorChannels::kGpsBufferCapacity>>(
            mc_experimental::GpsTaskContext<SensorChannels::kGpsBufferCapacity>{
                hal_,
                *gps_device_,
                {config_.gps.expected_fix_rate_hz > 0U ? (1000U / config_.gps.expected_fix_rate_hz) : 200U},
                &channels_.gps_fixes,
                nullptr,
                {}
            });
    }

    if (config_.storage.enabled && storage_device_ != nullptr) {
        storage_task_context_ = std::make_unique<mc_experimental::StorageTaskContext<SensorChannels::kStorageQueueCapacity>>(
            mc_experimental::StorageTaskContext<SensorChannels::kStorageQueueCapacity>{
                hal_,
                *storage_device_,
                {config_.storage.task.notification_timeout_ms, config_.storage.task.flush_period_ms},
                &channels_.storage_requests,
                nullptr,
                {},
                0
            });
    }

    if (config_.leds.enabled) {
        for (size_t i = 0; i < RuntimeConfig::kStatusLedCount; ++i) {
            if (status_leds_[i] != nullptr) {
                status_leds_[i]->init();
            }
        }
    }

    runtime_assembled_ = true;
    return true;
}

bool ExperimentalRuntime::create_tasks() {
    if (!runtime_assembled_) {
        return false;
    }

    bool ok = true;

    if (config_.imu.enabled && imu_task_context_ != nullptr) {
        ok = ok && mc_experimental::ImuTask::create(
            *imu_task_context_,
            "imu",
            config_.imu.task.stack_words,
            config_.imu.task.priority);
    }

    if (config_.compass.enabled && compass_task_context_ != nullptr) {
        ok = ok && mc_experimental::CompassTask::create(
            *compass_task_context_,
            "compass",
            config_.compass.task.stack_words,
            config_.compass.task.priority);
    }

    if (config_.barometer.enabled && barometer_task_context_ != nullptr) {
        ok = ok && mc_experimental::BarometerTask::create(
            *barometer_task_context_,
            "barometer",
            config_.barometer.task.stack_words,
            config_.barometer.task.priority);
    }

    if (config_.gps.enabled && gps_task_context_ != nullptr) {
        ok = ok && mc_experimental::GpsTask::create(
            *gps_task_context_,
            "gps",
            config_.gps.stack_words,
            config_.gps.priority);
    }

    if (config_.storage.enabled && storage_task_context_ != nullptr) {
        ok = ok && mc_experimental::StorageTask::create(
            *storage_task_context_,
            "storage",
            config_.storage.task.stack_words,
            config_.storage.task.priority);
    }

    ok = ok && EstimatorTask::create(estimator_task_context_);

    if (config_.heartbeat.enabled) {
        heartbeat_task_context_.led = status_leds_[0].get();
        ok = ok && HeartbeatTask::create(heartbeat_task_context_);
    }

    tasks_created_ = ok;
    return ok;
}

void ExperimentalRuntime::start_scheduler() {
    if (!tasks_created_) {
        return;
    }
    hal_.rtos().start_scheduler();
}

bool ExperimentalRuntime::start() {
    if (!init_hal()) {
        return false;
    }
    if (!assemble_runtime()) {
        return false;
    }
    if (!create_tasks()) {
        return false;
    }

    start_scheduler();
    return true;
}

}  // namespace experimental_mincopter
