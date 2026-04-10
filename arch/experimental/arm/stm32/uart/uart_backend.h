#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/uart.h>
#include <stm32f4xx_hal.h>

namespace stm32 {

struct Stm32UartSignalConfig {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t alternate_function;
};

struct Stm32UartPortConfig {
    size_t port_index;
    USART_TypeDef *instance;
    Stm32UartSignalConfig tx;
    Stm32UartSignalConfig rx;
};

class Stm32UartBackend {
public:
    virtual ~Stm32UartBackend() = default;

    virtual mc_rtos_hal::Status init(const Stm32UartPortConfig &config) = 0;
    virtual mc_rtos_hal::Status configure(const mc_rtos_hal::UartConfig &config) = 0;
    virtual mc_rtos_hal::Status write(const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *written) = 0;
    virtual mc_rtos_hal::Status read(uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *read_len) = 0;
};

}  // namespace stm32
