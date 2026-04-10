#pragma once

#include <arm/stm32/spi/spi_backend.h>

namespace stm32 {

class Stm32F4xxSpiBackend final : public Stm32SpiBackend {
public:
    Stm32F4xxSpiBackend();

    mc_rtos_hal::Status init(const Stm32SpiBusConfig &config) override;
    mc_rtos_hal::Status configure_device(const mc_rtos_hal::SpiDeviceConfig &config) override;
    mc_rtos_hal::Status transfer(const uint8_t *tx, uint8_t *rx, size_t len, mc_rtos_hal::Timeout timeout) override;

private:
    mc_rtos_hal::Status init_gpio(const Stm32SpiBusConfig &config);
    mc_rtos_hal::Status init_peripheral(const mc_rtos_hal::SpiDeviceConfig &config);
    mc_rtos_hal::Status enable_clock(SPI_TypeDef *instance);
    static uint32_t to_hal_timeout(mc_rtos_hal::Timeout timeout);
    static mc_rtos_hal::Status map_hal_status(HAL_StatusTypeDef hal_status);
    static uint32_t baud_prescaler_for(uint32_t requested_hz);
    static void enable_gpio_clock(GPIO_TypeDef *port);

    SPI_HandleTypeDef *handle_;
    Stm32SpiBusConfig bus_config_;
    bool initialized_;
};

}  // namespace stm32
