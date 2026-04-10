#pragma once

#include <memory>

#include <experimental/mincopter/ExperimentalRuntime.h>
#include <experimental/mincopter/RuntimeConfig.h>
#include <rtos_hal/hal.h>

namespace experimental_mincopter {

struct RuntimeAssembly {
    RuntimeConfig config;
    std::unique_ptr<mc_experimental::ImuDevice> imu_device;
    std::unique_ptr<mc_experimental::CompassDevice> compass_device;
    std::unique_ptr<mc_experimental::BarometerDevice> barometer_device;
    std::unique_ptr<mc_experimental::GpsDevice> gps_device;
    std::unique_ptr<mc_experimental::StorageDevice> storage_device;
};

RuntimeAssembly assemble_runtime_components(mc_rtos_hal::Hal &hal);

}  // namespace experimental_mincopter
