#pragma once

#include <stdint.h>

#include <experimental_build_config.h>

#include <rtos_hal/types.h>

namespace experimental_mincopter {

enum class DeviceTransport : uint8_t {
    None = 0,
    I2c,
    Spi,
    Uart,
};

struct SensorTaskSchedule {
    uint32_t polling_period_ms;
    uint32_t notification_timeout_ms;
    mc_rtos_hal::TaskPriority priority;
    uint16_t stack_words;
};

struct StorageTaskSchedule {
    uint32_t notification_timeout_ms;
    uint32_t flush_period_ms;
    mc_rtos_hal::TaskPriority priority;
    uint16_t stack_words;
};

struct EstimatorTaskSchedule {
    uint32_t polling_period_ms;
    mc_rtos_hal::TaskPriority priority;
    uint16_t stack_words;
};

struct HeartbeatTaskSchedule {
    bool enabled;
    size_t uart_index;
    uint32_t baud_rate;
    uint32_t period_ms;
    mc_rtos_hal::TaskPriority priority;
    uint16_t stack_words;
};

struct RuntimeConfig {
    struct ImuRuntimeConfig {
        bool enabled;
        DeviceTransport transport;
        uint8_t bus_index;
        uint8_t i2c_address;
        uint16_t chip_select_pin;
        uint16_t data_ready_pin;
        bool has_data_ready_irq;
        uint32_t sample_rate_hz;
        SensorTaskSchedule task;
    };

    struct CompassRuntimeConfig {
        bool enabled;
        DeviceTransport transport;
        uint8_t bus_index;
        uint8_t i2c_address;
        uint16_t chip_select_pin;
        uint16_t data_ready_pin;
        bool has_data_ready_irq;
        uint32_t sample_rate_hz;
        SensorTaskSchedule task;
    };

    struct BarometerRuntimeConfig {
        bool enabled;
        DeviceTransport transport;
        uint8_t bus_index;
        uint8_t i2c_address;
        uint16_t chip_select_pin;
        uint16_t data_ready_pin;
        bool has_data_ready_irq;
        uint32_t sample_rate_hz;
        SensorTaskSchedule task;
    };

    struct GpsRuntimeConfig {
        bool enabled;
        DeviceTransport transport;
        size_t uart_index;
        uint32_t baud_rate;
        uint32_t expected_fix_rate_hz;
        uint16_t rx_chunk_size;
        mc_rtos_hal::TaskPriority priority;
        uint16_t stack_words;
    };

    struct StorageRuntimeConfig {
        bool enabled;
        DeviceTransport transport;
        uint8_t bus_index;
        uint16_t chip_select_pin;
        uint16_t reset_pin;
        uint32_t spi_frequency_hz;
        StorageTaskSchedule task;
    };

    ImuRuntimeConfig imu;
    CompassRuntimeConfig compass;
    BarometerRuntimeConfig barometer;
    GpsRuntimeConfig gps;
    StorageRuntimeConfig storage;
    EstimatorTaskSchedule estimator;
    HeartbeatTaskSchedule heartbeat;
};

uint16_t parse_pin_name(const char *pin_name);
RuntimeConfig make_runtime_config();

}  // namespace experimental_mincopter
