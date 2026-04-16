#include <memory>
#include <stddef.h>
#include <stdint.h>

#include <arm/stm32/hal.h>
#include <dev/experimental/compass/CompassTask.h>
#include <dev/experimental/compass/hmc5843/Hmc5843CompassDevice.h>
#include <dev/experimental/compass/icm20948/Icm20948CompassDevice.h>
#include <dev/experimental/led/LedDevice.h>
#include <experimental/mincopter/RuntimeConfig.h>
#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

namespace {

constexpr size_t kCompassBufferCapacity = 16;
constexpr uint32_t kPrinterPeriodMs = 1000;
constexpr uint16_t kCompassStackWords = 384;
constexpr uint16_t kPrinterStackWords = 768;
constexpr mc_rtos_hal::Timeout kPrintTimeout{50};

struct PrinterTaskContext {
    mc_rtos_hal::Hal *hal;
    mc_experimental::CompassSampleRingBuffer<kCompassBufferCapacity> *ring;
    const mc_experimental::CompassTaskStats *compass_stats;
    mc_experimental::LedDevice *led;
    void *task_handle;
};

stm32::Stm32Hal g_hal;
mc_experimental::CompassSampleRingBuffer<kCompassBufferCapacity> g_compass_ring;
std::unique_ptr<mc_experimental::CompassDevice> g_compass_device;
std::unique_ptr<mc_experimental::LedDevice> g_status_led;
std::unique_ptr<mc_experimental::CompassTaskContext<kCompassBufferCapacity>> g_compass_context;
PrinterTaskContext g_printer_context{};

void panic(const char *message) {
    g_hal.console().printf(kPrintTimeout, "test-compass-task panic: %s\r\n", message);
    for (;;) {
        g_hal.time().delay_ms(1000);
    }
}

void printer_task_entry(void *context) {
    auto &printer = *static_cast<PrinterTaskContext *>(context);
    mc_experimental::CompassSample latest{};
    bool latest_valid = false;
    uint32_t last_wake_ms = printer.hal->time().millis();

    for (;;) {
        mc_experimental::CompassSample sample{};
        while (printer.ring->pop(sample)) {
            latest = sample;
            latest_valid = sample.valid;
        }

        const mc_experimental::CompassTaskStats empty_stats{};
        const mc_experimental::CompassTaskStats &stats =
            printer.compass_stats != nullptr ? *printer.compass_stats : empty_stats;

        printer.hal->console().printf(
            kPrintTimeout,
            "compass-test stats(samples=%lu failures=%lu timeouts=%lu overruns=%lu last_ts_us=%lu healthy=%u) "
            "latest(valid=%u ts_us=%lu field_gauss=%.6f,%.6f,%.6f)\r\n",
            static_cast<unsigned long>(stats.samples_published),
            static_cast<unsigned long>(stats.read_failures),
            static_cast<unsigned long>(stats.wake_timeouts),
            static_cast<unsigned long>(stats.overruns),
            static_cast<unsigned long>(stats.last_sample_timestamp_us),
            stats.healthy ? 1U : 0U,
            latest_valid ? 1U : 0U,
            static_cast<unsigned long>(latest.timestamp_us),
            static_cast<double>(latest.field_gauss[0]),
            static_cast<double>(latest.field_gauss[1]),
            static_cast<double>(latest.field_gauss[2]));

        if (printer.led != nullptr) {
            printer.led->toggle();
        }

        printer.hal->time().delay_until_ms(last_wake_ms, kPrinterPeriodMs / 2U);

        if (printer.led != nullptr) {
            printer.led->toggle();
        }

        printer.hal->time().delay_until_ms(last_wake_ms, kPrinterPeriodMs / 2U);
    }
}

bool create_printer_task(PrinterTaskContext &context) {
    mc_rtos_hal::TaskConfig task_config{};
    task_config.name = "compass-print";
    task_config.entry = &printer_task_entry;
    task_config.context = &context;
    task_config.stack_words = kPrinterStackWords;
    task_config.priority = mc_rtos_hal::TaskPriority::Lowest;
    return context.hal->rtos().create_task(task_config, &context.task_handle) == mc_rtos_hal::Status::Ok;
}

std::unique_ptr<mc_experimental::CompassDevice> make_compass_device(
    const experimental_mincopter::RuntimeConfig &runtime_config) {
#if defined(MC_COMP_HMC5843)
    mc_experimental::Hmc5843Config config{};
    config.i2c_address = runtime_config.compass.i2c_address;
    config.data_ready_pin = runtime_config.compass.data_ready_pin;
    config.has_data_ready_irq = runtime_config.compass.has_data_ready_irq;
    config.sample_rate_hz = runtime_config.compass.sample_rate_hz;
    return std::make_unique<mc_experimental::Hmc5843CompassDevice>(
        g_hal.i2c(runtime_config.compass.bus_index),
        g_hal.time(),
        g_hal.gpio(),
        config);
#elif defined(MC_COMP_ICM20948)
    mc_experimental::Icm20948CompassConfig config{};
    config.i2c_address = runtime_config.compass.i2c_address;
    config.data_ready_pin = runtime_config.compass.data_ready_pin;
    config.has_data_ready_irq = runtime_config.compass.has_data_ready_irq;
    config.sample_rate_hz = runtime_config.compass.sample_rate_hz;
    return std::make_unique<mc_experimental::Icm20948CompassDevice>(
        g_hal.i2c(runtime_config.compass.bus_index),
        g_hal.time(),
        g_hal.gpio(),
        config);
#else
    (void)runtime_config;
    return nullptr;
#endif
}

}  // namespace

int main() {
    if (g_hal.init() != mc_rtos_hal::Status::Ok) {
        for (;;) {
        }
    }

    const experimental_mincopter::RuntimeConfig runtime_config =
        experimental_mincopter::make_runtime_config();

    if (!runtime_config.compass.enabled) {
        panic("compass disabled");
    }

    if (runtime_config.leds.enabled && runtime_config.leds.count > 0U) {
        mc_experimental::LedConfig led_config{};
        led_config.pin = runtime_config.leds.pins[0];
        led_config.active_high = runtime_config.leds.active_high[0];
        led_config.initially_on = false;
        g_status_led = std::make_unique<mc_experimental::LedDevice>(g_hal.gpio(), led_config);
        if (!g_status_led->init()) {
            panic("failed to init status led");
        }
    }

    g_compass_device = make_compass_device(runtime_config);
    if (!g_compass_device) {
        panic("failed to create compass device");
    }

    g_compass_context = std::make_unique<mc_experimental::CompassTaskContext<kCompassBufferCapacity>>(
        mc_experimental::CompassTaskContext<kCompassBufferCapacity>{
            g_hal,
            *g_compass_device,
            {runtime_config.compass.task.notification_timeout_ms,
             runtime_config.compass.task.polling_period_ms},
            &g_compass_ring,
            nullptr,
            {}
        });

    if (!mc_experimental::CompassTask::create(*g_compass_context,
                                              "compass-test",
                                              kCompassStackWords,
                                              runtime_config.compass.task.priority)) {
        panic("failed to create compass task");
    }

    g_printer_context.hal = &g_hal;
    g_printer_context.ring = &g_compass_ring;
    g_printer_context.compass_stats = &g_compass_context->stats;
    g_printer_context.led = g_status_led.get();
    g_printer_context.task_handle = nullptr;

    if (!create_printer_task(g_printer_context)) {
        panic("failed to create printer task");
    }

    g_hal.console().printf(kPrintTimeout, "test-compass-task starting scheduler\r\n");
    g_hal.rtos().start_scheduler();

    panic("scheduler returned");
}
