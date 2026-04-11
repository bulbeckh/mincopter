#pragma once

#include <memory>

#include <dev/experimental/barometer/BarometerDevice.h>
#include <dev/experimental/barometer/BarometerTask.h>
#include <dev/experimental/compass/CompassDevice.h>
#include <dev/experimental/compass/CompassTask.h>
#include <dev/experimental/gps/GpsDevice.h>
#include <dev/experimental/gps/GpsTask.h>
#include <dev/experimental/imu/ImuDevice.h>
#include <dev/experimental/imu/ImuTask.h>
#include <dev/experimental/storage/StorageDevice.h>
#include <dev/experimental/storage/StorageTask.h>
#include <experimental/mincopter/EstimatorTask.h>
#include <experimental/mincopter/HeartbeatTask.h>
#include <experimental/mincopter/RuntimeConfig.h>
#include <experimental/mincopter/SensorChannels.h>
#include <rtos_hal/hal.h>

namespace experimental_mincopter {

class ExperimentalRuntime {
public:
    ExperimentalRuntime(
        mc_rtos_hal::Hal &hal,
        const RuntimeConfig &config,
        std::unique_ptr<mc_experimental::ImuDevice> imu_device,
        std::unique_ptr<mc_experimental::CompassDevice> compass_device,
        std::unique_ptr<mc_experimental::BarometerDevice> barometer_device,
        std::unique_ptr<mc_experimental::GpsDevice> gps_device,
        std::unique_ptr<mc_experimental::StorageDevice> storage_device,
        std::unique_ptr<mc_experimental::LedDevice> status_leds[RuntimeConfig::kStatusLedCount]);

    mc_rtos_hal::Hal &hal();
    const RuntimeConfig &config() const;
    SensorChannels &channels();

    bool init_hal();
    bool assemble_runtime();
    bool create_tasks();
    void start_scheduler();

    bool start();

private:
    mc_rtos_hal::Hal &hal_;
    RuntimeConfig config_;
    SensorChannels channels_;

    std::unique_ptr<mc_experimental::ImuDevice> imu_device_;
    std::unique_ptr<mc_experimental::CompassDevice> compass_device_;
    std::unique_ptr<mc_experimental::BarometerDevice> barometer_device_;
    std::unique_ptr<mc_experimental::GpsDevice> gps_device_;
    std::unique_ptr<mc_experimental::StorageDevice> storage_device_;
    std::unique_ptr<mc_experimental::LedDevice> status_leds_[RuntimeConfig::kStatusLedCount];

    std::unique_ptr<mc_experimental::ImuTaskContext<SensorChannels::kImuBufferCapacity>> imu_task_context_;
    std::unique_ptr<mc_experimental::CompassTaskContext<SensorChannels::kCompassBufferCapacity>> compass_task_context_;
    std::unique_ptr<mc_experimental::BarometerTaskContext<SensorChannels::kBarometerBufferCapacity>> barometer_task_context_;
    std::unique_ptr<mc_experimental::GpsTaskContext<SensorChannels::kGpsBufferCapacity>> gps_task_context_;
    std::unique_ptr<mc_experimental::StorageTaskContext<SensorChannels::kStorageQueueCapacity>> storage_task_context_;
    EstimatorTaskContext estimator_task_context_;
    HeartbeatTaskContext heartbeat_task_context_;

    bool hal_initialized_;
    bool runtime_assembled_;
    bool tasks_created_;
};

}  // namespace experimental_mincopter
