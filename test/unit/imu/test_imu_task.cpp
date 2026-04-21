#include <memory>
#include <stddef.h>
#include <stdint.h>

#include <arm/stm32/hal.h>
#include <dev/experimental/imu/ImuTask.h>
#include <dev/experimental/imu/bmi270/Bmi270ImuDevice.h>
#include <dev/experimental/imu/mpu6050/Mpu6050ImuDevice.h>
#include <dev/experimental/led/LedDevice.h>
#include <experimental/mincopter/RuntimeConfig.h>
#include <rtos_hal/hal.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/time.h>

namespace {

constexpr size_t kImuBufferCapacity = 32;
constexpr uint32_t kPrinterPeriodMs = 1000;
constexpr uint16_t kImuStackWords = 1024;
constexpr uint16_t kPrinterStackWords = 768;
constexpr mc_rtos_hal::Timeout kPrintTimeout{50};

struct PrinterTaskContext {
    mc_rtos_hal::Hal *hal;
    mc_experimental::ImuSampleRingBuffer<kImuBufferCapacity> *ring;
    const mc_experimental::ImuTaskStats *imu_stats;
    mc_experimental::LedDevice *led;
    void *task_handle;
};

stm32::Stm32Hal g_hal;
mc_experimental::ImuSampleRingBuffer<kImuBufferCapacity> g_imu_ring;
std::unique_ptr<mc_experimental::ImuDevice> g_imu_device;
std::unique_ptr<mc_experimental::LedDevice> g_status_led;
std::unique_ptr<mc_experimental::ImuTaskContext<kImuBufferCapacity>> g_imu_context;
PrinterTaskContext g_printer_context{};

void panic(const char *message) {
    g_hal.console().printf(kPrintTimeout, "test-imu-task panic: %s\r\n", message);
    for (;;) {
        g_hal.time().delay_ms(1000);
    }
}

void printer_task_entry(void *context) {
    auto &printer = *static_cast<PrinterTaskContext *>(context);
    mc_experimental::ImuSample latest{};
    bool latest_valid = false;
    uint32_t last_wake_ms = printer.hal->time().millis();

    for (;;) {
        mc_experimental::ImuSample sample{};
        while (printer.ring->pop(sample)) {
            latest = sample;
            latest_valid = sample.valid;
        }

        const mc_experimental::ImuTaskStats empty_stats{};
        const mc_experimental::ImuTaskStats &stats =
            printer.imu_stats != nullptr ? *printer.imu_stats : empty_stats;

        printer.hal->console().printf(
            kPrintTimeout,
            "imu-test stats(runs=%lu samples=%lu failures=%lu timeouts=%lu overruns=%lu last_ts_us=%lu recoveries=%lu recovery_failures=%lu healthy=%u) "
            "latest(valid=%u ts_us=%lu accel=%.6f,%.6f,%.6f gyro=%.6f,%.6f,%.6f temp=%.3f)\r\n",
            static_cast<unsigned long>(stats.loop_runs),
            static_cast<unsigned long>(stats.samples_published),
            static_cast<unsigned long>(stats.read_failures),
            static_cast<unsigned long>(stats.wake_timeouts),
            static_cast<unsigned long>(stats.overruns),
            static_cast<unsigned long>(stats.last_sample_timestamp_us),
            static_cast<unsigned long>(stats.recovery_attempts),
            static_cast<unsigned long>(stats.recovery_failures),
            stats.healthy ? 1U : 0U,
            latest_valid ? 1U : 0U,
            static_cast<unsigned long>(latest.timestamp_us),
            static_cast<double>(latest.accel_m_s2[0]),
            static_cast<double>(latest.accel_m_s2[1]),
            static_cast<double>(latest.accel_m_s2[2]),
            static_cast<double>(latest.gyro_rad_s[0]),
            static_cast<double>(latest.gyro_rad_s[1]),
            static_cast<double>(latest.gyro_rad_s[2]),
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
    task_config.name = "imu-print";
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

    if (!runtime_config.imu.enabled) {
        panic("imu disabled");
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

#if defined(MC_IMU_MPU6050)
    mc_experimental::Mpu6050Config imu_device_config{};
    imu_device_config.i2c_address = runtime_config.imu.i2c_address;
    imu_device_config.data_ready_pin = runtime_config.imu.data_ready_pin;
    imu_device_config.has_data_ready_irq = runtime_config.imu.has_data_ready_irq;
    imu_device_config.sample_rate_hz = runtime_config.imu.sample_rate_hz;

    g_imu_device = std::make_unique<mc_experimental::Mpu6050ImuDevice>(
        g_hal.i2c(runtime_config.imu.bus_index),
        g_hal.time(),
        g_hal.gpio(),
        imu_device_config);
#elif defined(MC_IMU_BMI270)
    mc_experimental::Bmi270Config imu_device_config{};
    imu_device_config.i2c_address = runtime_config.imu.i2c_address;
    imu_device_config.data_ready_pin = runtime_config.imu.data_ready_pin;
    imu_device_config.has_data_ready_irq = runtime_config.imu.has_data_ready_irq;
    imu_device_config.sample_rate_hz = runtime_config.imu.sample_rate_hz;

    g_imu_device = std::make_unique<mc_experimental::Bmi270ImuDevice>(
        g_hal.i2c(runtime_config.imu.bus_index),
        g_hal.time(),
        g_hal.gpio(),
        imu_device_config);
#else
    panic("unsupported imu test target");
#endif

    g_imu_context = std::make_unique<mc_experimental::ImuTaskContext<kImuBufferCapacity>>(
        mc_experimental::ImuTaskContext<kImuBufferCapacity>{
            g_hal,
            *g_imu_device,
            {runtime_config.imu.task.notification_timeout_ms,
             runtime_config.imu.task.polling_period_ms,
             runtime_config.imu.task.recovery_failure_threshold,
             runtime_config.imu.task.recovery_wake_timeout_threshold,
             runtime_config.imu.task.recovery_backoff_ms},
            &g_imu_ring,
            nullptr,
            {}
        });

    if (!mc_experimental::ImuTask::create(*g_imu_context,
                                          "imu-test",
                                          kImuStackWords,
                                          runtime_config.imu.task.priority)) {
        panic("failed to create imu task");
    }

    g_printer_context.hal = &g_hal;
    g_printer_context.ring = &g_imu_ring;
    g_printer_context.imu_stats = &g_imu_context->stats;
    g_printer_context.led = g_status_led.get();
    g_printer_context.task_handle = nullptr;

    if (!create_printer_task(g_printer_context)) {
        panic("failed to create printer task");
    }

    g_hal.console().printf(kPrintTimeout, "test-imu-task starting scheduler\r\n");
    g_hal.rtos().start_scheduler();

    panic("scheduler returned");
}
