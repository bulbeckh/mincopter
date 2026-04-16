#include <memory>
#include <stddef.h>
#include <stdint.h>

#include <arm/stm32/hal.h>
#include <dev/experimental/gps/GpsTask.h>
#include <dev/experimental/gps/ublox_neo/UbxNeoGpsDevice.h>
#include <dev/experimental/led/LedDevice.h>
#include <experimental/mincopter/RuntimeConfig.h>
#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

namespace {

constexpr size_t kGpsBufferCapacity = 8;
constexpr uint32_t kPrinterPeriodMs = 1000;
constexpr uint16_t kGpsStackWords = 512;
constexpr uint16_t kPrinterStackWords = 768;
constexpr mc_rtos_hal::Timeout kPrintTimeout{50};

struct PrinterTaskContext {
    mc_rtos_hal::Hal *hal;
    mc_experimental::GpsFixRingBuffer<kGpsBufferCapacity> *ring;
    const mc_experimental::GpsTaskStats *gps_stats;
    mc_experimental::LedDevice *led;
    void *task_handle;
};

stm32::Stm32Hal g_hal;
mc_experimental::GpsFixRingBuffer<kGpsBufferCapacity> g_gps_ring;
std::unique_ptr<mc_experimental::UbxNeoGpsDevice> g_gps_device;
std::unique_ptr<mc_experimental::LedDevice> g_status_led;
std::unique_ptr<mc_experimental::GpsTaskContext<kGpsBufferCapacity>> g_gps_context;
PrinterTaskContext g_printer_context{};

void panic(const char *message) {
    g_hal.console().printf(kPrintTimeout, "test-gps-task panic: %s\r\n", message);
    for (;;) {
        g_hal.time().delay_ms(1000);
    }
}

unsigned fix_type_value(mc_experimental::GpsFixType fix_type) {
    return static_cast<unsigned>(fix_type);
}

void printer_task_entry(void *context) {
    auto &printer = *static_cast<PrinterTaskContext *>(context);
    mc_experimental::GpsFix latest{};
    bool latest_valid = false;
    uint32_t last_wake_ms = printer.hal->time().millis();

    for (;;) {
        mc_experimental::GpsFix fix{};
        while (printer.ring->pop(fix)) {
            latest = fix;
            latest_valid = fix.valid;
        }

        const mc_experimental::GpsTaskStats empty_stats{};
        const mc_experimental::GpsTaskStats &stats =
            printer.gps_stats != nullptr ? *printer.gps_stats : empty_stats;

        printer.hal->console().printf(
            kPrintTimeout,
            "gps-test stats(fixes=%lu service_failures=%lu overruns=%lu polls=%lu last_ts_us=%lu healthy=%u) "
            "latest(valid=%u ts_us=%lu seq=%lu fix=%u sats=%u lat_e7=%ld lon_e7=%ld alt_cm=%ld "
            "vel_cm_s=%ld,%ld,%ld ground_cm_s=%lu heading_cd=%u)\r\n",
            static_cast<unsigned long>(stats.fixes_published),
            static_cast<unsigned long>(stats.service_failures),
            static_cast<unsigned long>(stats.overruns),
            static_cast<unsigned long>(stats.poll_count),
            static_cast<unsigned long>(stats.last_fix_timestamp_us),
            stats.healthy ? 1U : 0U,
            latest_valid ? 1U : 0U,
            static_cast<unsigned long>(latest.timestamp_us),
            static_cast<unsigned long>(latest.sequence),
            fix_type_value(latest.fix_type),
            static_cast<unsigned>(latest.satellites),
            static_cast<long>(latest.latitude_e7),
            static_cast<long>(latest.longitude_e7),
            static_cast<long>(latest.altitude_cm),
            static_cast<long>(latest.vel_north_cm_s),
            static_cast<long>(latest.vel_east_cm_s),
            static_cast<long>(latest.vel_down_cm_s),
            static_cast<unsigned long>(latest.ground_speed_cm_s),
            static_cast<unsigned>(latest.heading_cd));

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
    task_config.name = "gps-print";
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

    if (!runtime_config.gps.enabled) {
        panic("gps disabled");
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

    mc_experimental::UbxNeoGpsConfig gps_device_config{};
    gps_device_config.uart_index = runtime_config.gps.uart_index;
    gps_device_config.baud_rate = runtime_config.gps.baud_rate;
    gps_device_config.expected_fix_rate_hz = runtime_config.gps.expected_fix_rate_hz;
    gps_device_config.rx_chunk_size = runtime_config.gps.rx_chunk_size;

    g_gps_device = std::make_unique<mc_experimental::UbxNeoGpsDevice>(
        g_hal.uart(runtime_config.gps.uart_index),
        g_hal.time(),
        gps_device_config);

    g_gps_context = std::make_unique<mc_experimental::GpsTaskContext<kGpsBufferCapacity>>(
        mc_experimental::GpsTaskContext<kGpsBufferCapacity>{
            g_hal,
            *g_gps_device,
            {runtime_config.gps.expected_fix_rate_hz > 0U ? (1000U / runtime_config.gps.expected_fix_rate_hz) : 200U},
            &g_gps_ring,
            nullptr,
            {}
        });

    if (!mc_experimental::GpsTask::create(*g_gps_context,
                                          "gps-test",
                                          kGpsStackWords,
                                          runtime_config.gps.priority)) {
        panic("failed to create gps task");
    }

    g_printer_context.hal = &g_hal;
    g_printer_context.ring = &g_gps_ring;
    g_printer_context.gps_stats = &g_gps_context->stats;
    g_printer_context.led = g_status_led.get();
    g_printer_context.task_handle = nullptr;

    if (!create_printer_task(g_printer_context)) {
        panic("failed to create printer task");
    }

    g_hal.console().printf(kPrintTimeout, "test-gps-task starting scheduler\r\n");
    g_hal.rtos().start_scheduler();

    panic("scheduler returned");
}
