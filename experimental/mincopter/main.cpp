#include <experimental/mincopter/init.h>
#include <experimental/mincopter/RuntimeAssembly.h>

#include <experimental_platform_hal.h>

namespace {

experimental_platform::Hal hal;
experimental_mincopter::RuntimeAssembly assembly =
    experimental_mincopter::assemble_runtime_components(hal);
experimental_mincopter::ExperimentalRuntime runtime(
    hal,
    assembly.config,
    std::move(assembly.imu_device),
    std::move(assembly.compass_device),
    std::move(assembly.barometer_device),
    std::move(assembly.gps_device),
    std::move(assembly.storage_device));

}  // namespace

int main() {
    if (!experimental_mincopter::initialize_runtime(runtime)) {
        for (;;) {
            runtime.hal().time().delay_ms(1000);
        }
    }

    runtime.start_scheduler();

    for (;;) {
        runtime.hal().time().delay_ms(1000);
    }
}
