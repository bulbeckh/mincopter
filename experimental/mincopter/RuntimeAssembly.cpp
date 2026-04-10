#include <memory>

#include <dev/experimental/barometer/bme280/Bme280BarometerDevice.h>
#include <dev/experimental/compass/hmc5843/Hmc5843CompassDevice.h>
#include <dev/experimental/gps/ublox_neo/UbxNeoGpsDevice.h>
#include <dev/experimental/imu/mpu6050/Mpu6050ImuDevice.h>
#include <dev/experimental/storage/at45db321d/At45Db321dStorageDevice.h>
#include <experimental_build_config.h>
#include <experimental/mincopter/RuntimeAssembly.h>

namespace experimental_mincopter {

RuntimeAssembly assemble_runtime_components(mc_rtos_hal::Hal &hal) {
    RuntimeAssembly assembly{};
    assembly.config = make_runtime_config();

#if defined(MC_IMU_MPU6050)
    if (assembly.config.imu.enabled) {
        mc_experimental::Mpu6050Config device_config{};
        device_config.i2c_address = assembly.config.imu.i2c_address;
        device_config.data_ready_pin = assembly.config.imu.data_ready_pin;
        device_config.has_data_ready_irq = assembly.config.imu.has_data_ready_irq;
        device_config.sample_rate_hz = assembly.config.imu.sample_rate_hz;
        assembly.imu_device = std::make_unique<mc_experimental::Mpu6050ImuDevice>(
            hal.i2c(assembly.config.imu.bus_index),
            hal.time(),
            hal.gpio(),
            device_config);
    }
#endif

#if defined(MC_COMP_HMC5843)
    if (assembly.config.compass.enabled) {
        mc_experimental::Hmc5843Config device_config{};
        device_config.i2c_address = assembly.config.compass.i2c_address;
        device_config.data_ready_pin = assembly.config.compass.data_ready_pin;
        device_config.has_data_ready_irq = assembly.config.compass.has_data_ready_irq;
        device_config.sample_rate_hz = assembly.config.compass.sample_rate_hz;
        assembly.compass_device = std::make_unique<mc_experimental::Hmc5843CompassDevice>(
            hal.i2c(assembly.config.compass.bus_index),
            hal.time(),
            hal.gpio(),
            device_config);
    }
#endif

#if defined(MC_BARO_BME280)
    if (assembly.config.barometer.enabled) {
        mc_experimental::Bme280Config device_config{};
        device_config.i2c_address = assembly.config.barometer.i2c_address;
        device_config.data_ready_pin = assembly.config.barometer.data_ready_pin;
        device_config.has_data_ready_irq = assembly.config.barometer.has_data_ready_irq;
        device_config.sample_rate_hz = assembly.config.barometer.sample_rate_hz;
        assembly.barometer_device = std::make_unique<mc_experimental::Bme280BarometerDevice>(
            hal.i2c(assembly.config.barometer.bus_index),
            hal.time(),
            hal.gpio(),
            device_config);
    }
#endif

#if defined(MC_GPS_UBLOX)
    if (assembly.config.gps.enabled) {
        mc_experimental::UbxNeoGpsConfig device_config{};
        device_config.uart_index = assembly.config.gps.uart_index;
        device_config.baud_rate = assembly.config.gps.baud_rate;
        device_config.expected_fix_rate_hz = assembly.config.gps.expected_fix_rate_hz;
        device_config.rx_chunk_size = assembly.config.gps.rx_chunk_size;
        assembly.gps_device = std::make_unique<mc_experimental::UbxNeoGpsDevice>(
            hal.uart(assembly.config.gps.uart_index),
            hal.time(),
            device_config);
    }
#endif

#if defined(MC_STORAGE_DATAFLASH)
    if (assembly.config.storage.enabled) {
        mc_experimental::At45Db321dConfig device_config{};
        device_config.reset_pin = assembly.config.storage.reset_pin;
        device_config.chip_select_pin = assembly.config.storage.chip_select_pin;
        device_config.spi_frequency_hz = assembly.config.storage.spi_frequency_hz;
        assembly.storage_device = std::make_unique<mc_experimental::At45Db321dStorageDevice>(
            hal.spi(assembly.config.storage.bus_index),
            hal.time(),
            hal.gpio(),
            device_config);
    }
#endif

    return assembly;
}

}  // namespace experimental_mincopter
