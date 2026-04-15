#pragma once

#include <stddef.h>
#include <stdint.h>

#include <dev/experimental/barometer/BarometerTask.h>
#include <dev/experimental/compass/CompassTask.h>
#include <dev/experimental/gps/GpsTask.h>
#include <dev/experimental/imu/ImuTask.h>
#include <dev/experimental/storage/StorageTask.h>

namespace experimental_mincopter {

template <typename T>
struct LatestSample {
    T sample;
    uint32_t sequence;
    bool valid;
};

struct SensorChannels {
    static constexpr size_t kImuBufferCapacity = 64;
    static constexpr size_t kCompassBufferCapacity = 16;
    static constexpr size_t kBarometerBufferCapacity = 16;
    static constexpr size_t kGpsBufferCapacity = 8;
    static constexpr size_t kStorageQueueCapacity = 16;

    mc_experimental::ImuSampleRingBuffer<kImuBufferCapacity> imu_samples;
    mc_experimental::CompassSampleRingBuffer<kCompassBufferCapacity> compass_samples;
    mc_experimental::BarometerSampleRingBuffer<kBarometerBufferCapacity> barometer_samples;
    mc_experimental::GpsFixRingBuffer<kGpsBufferCapacity> gps_fixes;
    mc_experimental::StorageRequestQueue<kStorageQueueCapacity> storage_requests;

    LatestSample<mc_experimental::ImuSample> latest_imu{};
    LatestSample<mc_experimental::CompassSample> latest_compass{};
    LatestSample<mc_experimental::BarometerSample> latest_barometer{};
    LatestSample<mc_experimental::GpsFix> latest_gps{};
};

}  // namespace experimental_mincopter
