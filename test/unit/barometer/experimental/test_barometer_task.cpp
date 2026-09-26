#include <memory>
#include <stddef.h>
#include <stdint.h>

#include <arm/stm32/hal.h>
#include <dev/experimental/barometer/BarometerTask.h>
#include <dev/experimental/barometer/bme280/Bme280BarometerDevice.h>
#include <dev/experimental/led/LedDevice.h>
#include <experimental/mincopter/RuntimeConfig.h>
#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

namespace {

constexpr size_t kBarometerBufferCapacity = 16;
constexpr uint32_t kPrinterPeriodMs = 1000;
constexpr uint16_t kBarometerStackWords = 384;
constexpr uint16_t kPrinterStackWords = 768;
constexpr mc_rtos_hal::Timeout kPrintTimeout{50};

struct PrinterTaskContext {
    mc_rtos_hal::Hal *hal;
    mc_experimental::BarometerSampleRingBuffer<kBarometerBufferCapacity> *ring;
    const mc_experimental::BarometerTaskStats *barometer_stats;
    mc_experimental::LedDevice *led;
    void *task_handle;
};

stm32::Stm32Hal g_hal;
mc_experimental::BarometerSampleRingBuffer<kBarometerBufferCapacity> g_barometer_ring;
std::unique_ptr<mc_experimental::Bme280BarometerDevice> g_barometer_device;
std::unique_ptr<mc_experimental::LedDevice> g_status_led;
std::unique_ptr<mc_experimental::BarometerTaskContext<kBarometerBufferCapacity>> g_barometer_context;
PrinterTaskContext g_printer_context{};

void panic(const char *message) {
    g_hal.console().printf(kPrintTimeout, "test-barometer-task panic: %s\r\n", message);
    for (;;) {
        g_hal.time().delay_ms(1000);
    }
}

void printer_task_entry(void *context) {
    auto &printer = *static_cast<PrinterTaskContext *>(context);
    mc_experimental::BarometerSample latest{};
    bool latest_valid = false;
    uint32_t last_wake_ms = printer.hal->time().millis();

    for (;;) {
        mc_experimental::BarometerSample sample{};
        while (printer.ring->pop(sample)) {
            latest = sample;
            latest_valid = sample.valid;
        }

        const mc_experimental::BarometerTaskStats empty_stats{};
        const mc_experimental::BarometerTaskStats &stats =
            printer.barometer_stats != nullptr ? *printer.barometer_stats : empty_stats;

        printer.hal->console().printf(
            kPrintTimeout,
            "barometer-test stats(samples=%lu failures=%lu timeouts=%lu overruns=%lu last_ts_us=%lu healthy=%u) "
            "latest(valid=%u ts_us=%lu seq=%lu pressure_pa=%.3f temp=%.3f)\r\n",
            static_cast<unsigned long>(stats.samples_published),
            static_cast<unsigned long>(stats.read_failures),
            static_cast<unsigned long>(stats.wake_timeouts),
            static_cast<unsigned long>(stats.overruns),
            static_cast<unsigned long>(stats.last_sample_timestamp_us),
            stats.healthy ? 1U : 0U,
            latest_valid ? 1U : 0U,
            static_cast<unsigned long>(latest.timestamp_us),
            static_cast<unsigned long>(latest.sequence),
            static_cast<double>(latest.pressure_pa),
            static_cast<double>(latest.temperature_c));

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
    task_config.name = "baro-print";
    task_config.entry = &printer_task_entry;
    task_config.context = &context;
    task_config.stack_words = kPrinterStackWords;
    task_config.priority = mc_rtos_hal::TaskPriority::Lowest;
    return context.hal->rtos().create_task(task_config, &context.task_handle) == mc_rtos_hal::Status::Ok;
}

}  // namespace

int main() {
    if (g_hal.init() != mc_rtos_hal::Status::Ok) {
        for (;;) {
        }
    }

    const experimental_mincopter::RuntimeConfig runtime_config =
        experimental_mincopter::make_runtime_config();

    if (!runtime_config.barometer.enabled) {
        panic("barometer disabled");
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

    mc_experimental::Bme280Config barometer_device_config{};
    barometer_device_config.i2c_address = runtime_config.barometer.i2c_address;
    barometer_device_config.data_ready_pin = runtime_config.barometer.data_ready_pin;
    barometer_device_config.has_data_ready_irq = runtime_config.barometer.has_data_ready_irq;
    barometer_device_config.sample_rate_hz = runtime_config.barometer.sample_rate_hz;

    g_barometer_device = std::make_unique<mc_experimental::Bme280BarometerDevice>(
        g_hal.i2c(runtime_config.barometer.bus_index),
        g_hal.time(),
        g_hal.gpio(),
        barometer_device_config);

    g_barometer_context = std::make_unique<mc_experimental::BarometerTaskContext<kBarometerBufferCapacity>>(
        mc_experimental::BarometerTaskContext<kBarometerBufferCapacity>{
            g_hal,
            *g_barometer_device,
            {runtime_config.barometer.task.notification_timeout_ms,
             runtime_config.barometer.task.polling_period_ms},
            &g_barometer_ring,
            nullptr,
            {}
        });

    if (!mc_experimental::BarometerTask::create(*g_barometer_context,
                                                "barometer-test",
                                                kBarometerStackWords,
                                                runtime_config.barometer.task.priority)) {
        panic("failed to create barometer task");
    }

    g_printer_context.hal = &g_hal;
    g_printer_context.ring = &g_barometer_ring;
    g_printer_context.barometer_stats = &g_barometer_context->stats;
    g_printer_context.led = g_status_led.get();
    g_printer_context.task_handle = nullptr;

    if (!create_printer_task(g_printer_context)) {
        panic("failed to create printer task");
    }

    g_hal.console().printf(
        kPrintTimeout,
        "test-barometer-task config(bus=%u address=0x%02x poll_ms=%lu drdy=%u) starting scheduler\r\n",
        static_cast<unsigned>(runtime_config.barometer.bus_index),
        static_cast<unsigned>(runtime_config.barometer.i2c_address),
        static_cast<unsigned long>(runtime_config.barometer.task.polling_period_ms),
        runtime_config.barometer.has_data_ready_irq ? 1U : 0U);
    g_hal.rtos().start_scheduler();

    panic("scheduler returned");
}
