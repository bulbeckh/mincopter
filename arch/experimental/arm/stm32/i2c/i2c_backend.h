#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/i2c.h>
#include <stm32f4xx_hal.h>

namespace stm32 {

struct Stm32GpioPinConfig {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t alternate_function;
};

struct Stm32I2cBusConfig {
    size_t bus_index;
    I2C_TypeDef *instance;
    Stm32GpioPinConfig scl;
    Stm32GpioPinConfig sda;
    uint32_t clock_speed_hz;
};

class Stm32I2cBackend {
public:
    virtual ~Stm32I2cBackend() = default;

    virtual mc_rtos_hal::Status init(const Stm32I2cBusConfig &config) = 0;
    virtual mc_rtos_hal::Status write(uint8_t address, const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) = 0;
    virtual mc_rtos_hal::Status read(uint8_t address, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) = 0;
    virtual mc_rtos_hal::Status write_register(uint8_t address, uint8_t reg, uint8_t value, mc_rtos_hal::Timeout timeout) = 0;
    virtual mc_rtos_hal::Status read_registers(uint8_t address, uint8_t reg, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) = 0;
};

}  // namespace stm32
