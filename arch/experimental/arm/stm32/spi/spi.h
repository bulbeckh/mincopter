#pragma once

#include <memory>

#include <arm/stm32/gpio/gpio.h>
#include <arm/stm32/spi/spi_backend.h>
#include <arm/stm32/sync.h>
#include <rtos_hal/spi.h>

namespace stm32 {

class Stm32SpiBus final : public mc_rtos_hal::SpiBus {
public:
    Stm32SpiBus();

    mc_rtos_hal::Status configure(const Stm32SpiBusConfig &config, mc_rtos_hal::GpioController &gpio);

    mc_rtos_hal::Status lock(mc_rtos_hal::Timeout timeout) override;
    void unlock() override;
    mc_rtos_hal::Status configure_device(const mc_rtos_hal::SpiDeviceConfig &config) override;
    mc_rtos_hal::Status transfer(const uint8_t *tx, uint8_t *rx, size_t len, mc_rtos_hal::Timeout timeout) override;

private:
    Stm32Mutex mutex_;
    std::unique_ptr<Stm32SpiBackend> backend_;
    mc_rtos_hal::GpioController *gpio_;
    mc_rtos_hal::GpioPin *chip_select_;
    mc_rtos_hal::SpiDeviceConfig device_config_;
    bool bus_configured_;
    bool device_configured_;
};

}  // namespace stm32
