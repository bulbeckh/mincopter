#pragma once

#include <arm/stm32/uart/uart_backend.h>

namespace stm32 {

class Stm32F4xxUartBackend final : public Stm32UartBackend {
public:
    Stm32F4xxUartBackend();

    mc_rtos_hal::Status init(const Stm32UartPortConfig &config) override;
    mc_rtos_hal::Status configure(const mc_rtos_hal::UartConfig &config) override;
    mc_rtos_hal::Status write(const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *written) override;
    mc_rtos_hal::Status read(uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *read_len) override;

private:
    mc_rtos_hal::Status init_gpio(const Stm32UartPortConfig &config);
    mc_rtos_hal::Status enable_clock(USART_TypeDef *instance);
    static uint32_t to_hal_timeout(mc_rtos_hal::Timeout timeout);
    static mc_rtos_hal::Status map_hal_status(HAL_StatusTypeDef hal_status);
    static void enable_gpio_clock(GPIO_TypeDef *port);

    UART_HandleTypeDef *handle_;
    Stm32UartPortConfig port_config_;
    bool initialized_;
};

}  // namespace stm32
