#include <experimental/mincopter/RuntimeConfig.h>

namespace experimental_mincopter {

namespace {

uint32_t to_uint32(unsigned value) {
    return static_cast<uint32_t>(value);
}

mc_rtos_hal::TaskPriority default_sensor_priority() {
    return mc_rtos_hal::TaskPriority::High;
}

}  // namespace

uint16_t parse_pin_name(const char *pin_name) {
    if (pin_name == nullptr || pin_name[0] != 'P') {
        return 0;
    }

    const char port_char = pin_name[1];
    if (port_char < 'A' || port_char > 'Z') {
        return 0;
    }

    uint16_t pin_number = 0;
    for (size_t i = 2; pin_name[i] != '\0'; ++i) {
        const char c = pin_name[i];
        if (c < '0' || c > '9') {
            return 0;
        }
        pin_number = static_cast<uint16_t>((pin_number * 10U) + static_cast<uint16_t>(c - '0'));
    }

    const uint16_t port_index = static_cast<uint16_t>(port_char - 'A');
    return static_cast<uint16_t>((port_index * 16U) + pin_number);
}

RuntimeConfig make_runtime_config() {
    RuntimeConfig config{};

    config.imu.enabled = experimental_build_config::imu::enabled;
    config.imu.transport = config.imu.enabled ? DeviceTransport::I2c : DeviceTransport::None;
    config.imu.bus_index = static_cast<uint8_t>(experimental_build_config::imu::bus_index);
    config.imu.i2c_address = static_cast<uint8_t>(experimental_build_config::imu::address);
    config.imu.chip_select_pin = 0;
    config.imu.data_ready_pin = parse_pin_name(experimental_build_config::imu::drdy_pin);
    config.imu.has_data_ready_irq = experimental_build_config::imu::has_data_ready_irq;
    config.imu.sample_rate_hz = to_uint32(experimental_build_config::imu::sample_rate_hz);
    config.imu.task.notification_timeout_ms = 20;
    config.imu.task.polling_period_ms = 1;
    config.imu.task.priority = default_sensor_priority();
    config.imu.task.stack_words = 512;

    config.compass.enabled = experimental_build_config::compass::enabled;
    config.compass.transport = config.compass.enabled ? DeviceTransport::I2c : DeviceTransport::None;
    config.compass.bus_index = static_cast<uint8_t>(experimental_build_config::compass::bus_index);
    config.compass.i2c_address = static_cast<uint8_t>(experimental_build_config::compass::address);
    config.compass.chip_select_pin = 0;
    config.compass.data_ready_pin = parse_pin_name(experimental_build_config::compass::drdy_pin);
    config.compass.has_data_ready_irq = experimental_build_config::compass::has_data_ready_irq;
    config.compass.sample_rate_hz = to_uint32(experimental_build_config::compass::sample_rate_hz);
    config.compass.task.notification_timeout_ms = 50;
    config.compass.task.polling_period_ms = 20;
    config.compass.task.priority = mc_rtos_hal::TaskPriority::Medium;
    config.compass.task.stack_words = 384;

    config.barometer.enabled = experimental_build_config::barometer::enabled;
    config.barometer.transport = config.barometer.enabled ? DeviceTransport::I2c : DeviceTransport::None;
    config.barometer.bus_index = static_cast<uint8_t>(experimental_build_config::barometer::bus_index);
    config.barometer.i2c_address = static_cast<uint8_t>(experimental_build_config::barometer::address);
    config.barometer.chip_select_pin = 0;
    config.barometer.data_ready_pin = parse_pin_name(experimental_build_config::barometer::drdy_pin);
    config.barometer.has_data_ready_irq = experimental_build_config::barometer::has_data_ready_irq;
    config.barometer.sample_rate_hz = to_uint32(experimental_build_config::barometer::sample_rate_hz);
    config.barometer.task.notification_timeout_ms = 100;
    config.barometer.task.polling_period_ms = 20;
    config.barometer.task.priority = mc_rtos_hal::TaskPriority::Medium;
    config.barometer.task.stack_words = 384;

    config.gps.enabled = experimental_build_config::gps::enabled;
    config.gps.transport = config.gps.enabled ? DeviceTransport::Uart : DeviceTransport::None;
    config.gps.uart_index = static_cast<size_t>(experimental_build_config::gps::uart_index);
    config.gps.baud_rate = to_uint32(experimental_build_config::gps::baud);
    config.gps.expected_fix_rate_hz = to_uint32(experimental_build_config::gps::expected_fix_rate_hz);
    config.gps.rx_chunk_size = 64;
    config.gps.priority = mc_rtos_hal::TaskPriority::Low;
    config.gps.stack_words = 512;

    config.storage.enabled = experimental_build_config::storage::enabled;
    config.storage.transport = config.storage.enabled ? DeviceTransport::Spi : DeviceTransport::None;
    config.storage.bus_index = static_cast<uint8_t>(experimental_build_config::storage::spi_index);
    config.storage.reset_pin = parse_pin_name(experimental_build_config::storage::reset_pin);
    config.storage.chip_select_pin = parse_pin_name(experimental_build_config::storage::chip_select_pin);
    config.storage.spi_frequency_hz = to_uint32(experimental_build_config::storage::spi_frequency_hz);
    config.storage.task.notification_timeout_ms = 100;
    config.storage.task.flush_period_ms = 1000;
    config.storage.task.priority = mc_rtos_hal::TaskPriority::Low;
    config.storage.task.stack_words = 512;

    config.estimator.polling_period_ms = 10;
    config.estimator.priority = mc_rtos_hal::TaskPriority::High;
    config.estimator.stack_words = 768;

    config.heartbeat.enabled = experimental_build_config::heartbeat::enabled;
    config.heartbeat.uart_index = static_cast<size_t>(experimental_build_config::heartbeat::uart_index);
    config.heartbeat.baud_rate = to_uint32(experimental_build_config::heartbeat::baud);
    config.heartbeat.period_ms = 1000;
    config.heartbeat.priority = mc_rtos_hal::TaskPriority::Lowest;
    config.heartbeat.stack_words = 256;

    config.leds.enabled = experimental_build_config::leds::enabled;
    config.leds.count = static_cast<uint8_t>(RuntimeConfig::kStatusLedCount);
    config.leds.pins[0] = parse_pin_name(experimental_build_config::leds::status_led0_pin);
    config.leds.pins[1] = parse_pin_name(experimental_build_config::leds::status_led1_pin);
    config.leds.pins[2] = parse_pin_name(experimental_build_config::leds::status_led2_pin);
    config.leds.pins[3] = parse_pin_name(experimental_build_config::leds::status_led3_pin);
    config.leds.active_high[0] = experimental_build_config::leds::active_high;
    config.leds.active_high[1] = experimental_build_config::leds::active_high;
    config.leds.active_high[2] = experimental_build_config::leds::active_high;
    config.leds.active_high[3] = experimental_build_config::leds::active_high;

    return config;
}

}  // namespace experimental_mincopter
