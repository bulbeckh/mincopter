#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/spi.h>
#include <stm32f4xx_hal.h>

namespace stm32 {

struct Stm32SpiSignalConfig {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t alternate_function;
};

struct Stm32SpiBusConfig {
    size_t bus_index;
    SPI_TypeDef *instance;
    Stm32SpiSignalConfig sck;
    Stm32SpiSignalConfig miso;
    Stm32SpiSignalConfig mosi;
};

class Stm32SpiBackend {
public:
    virtual ~Stm32SpiBackend() = default;

    virtual mc_rtos_hal::Status init(const Stm32SpiBusConfig &config) = 0;
    virtual mc_rtos_hal::Status configure_device(const mc_rtos_hal::SpiDeviceConfig &config) = 0;
    virtual mc_rtos_hal::Status transfer(const uint8_t *tx, uint8_t *rx, size_t len, mc_rtos_hal::Timeout timeout) = 0;
};

}  // namespace stm32
